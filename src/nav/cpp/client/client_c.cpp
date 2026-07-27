#include "client_c.h"

#include "client.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace {

struct Handle {
  explicit Handle(int domain_id)
      : client(std::make_unique<lingtu::nav::commands::Client>(domain_id)) {}

  std::unique_ptr<lingtu::nav::commands::Client> client;
  std::mutex path_mutex;
  std::optional<lingtu::nav::commands::PathSnapshot> global_path_staging;
  std::optional<lingtu::nav::commands::PathSnapshot> local_path_staging;
  std::mutex map_scene_mutex;
  std::optional<lingtu::nav::commands::MapSceneSnapshot> map_scene_staging;
  std::uint64_t map_scene_consumer_buffer_retries{0U};
};

// C ABI calls can execute concurrently on one handle. Keep the error paired
// with the calling thread so a successful command cannot erase another
// command's failure before ctypes reads it.
thread_local std::string thread_error;

Handle* asHandle(lingtu_nav_client_handle handle) {
  return static_cast<Handle*>(handle);
}

template <typename Operation>
int invoke(lingtu_nav_client_handle raw_handle, Operation&& operation) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr) {
    thread_error = "navigation client handle is null";
    return -1;
  }
  try {
    operation(*handle->client);
    thread_error.clear();
    return 0;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation command failure";
    return -1;
  }
}

template <std::size_t N>
void copyString(char (&target)[N], const std::string& source) {
  static_assert(N > 0U);
  const std::size_t count = std::min(source.size(), N - 1U);
  std::memcpy(target, source.data(), count);
  target[count] = '\0';
}
bool validateOperatorMotionReceiptBuffer(
    const lingtu_nav_operator_motion_receipt_v1* receipt,
    const char* operation) {
  if (receipt == nullptr) {
    thread_error = std::string(operation) + " receipt is null";
    return false;
  }
  if (receipt->abi_version !=
      LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION) {
    thread_error = std::string(operation) + " receipt ABI version mismatch";
    return false;
  }
  if (receipt->struct_size < sizeof(*receipt)) {
    thread_error = std::string(operation) + " receipt struct is too small";
    return false;
  }
  return true;
}

void copyOperatorMotionReceipt(
    lingtu_nav_operator_motion_receipt_v1* target,
    const lingtu::nav::commands::OperatorMotionCommandReceipt& source) {
  std::memset(target, 0, sizeof(*target));
  target->abi_version = LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION;
  target->struct_size = sizeof(*target);
  target->accepted = source.accepted ? 1 : 0;
  target->action = source.action;
  copyString(target->request_id, source.request_id);
  copyString(target->source_id, source.source_id);
  target->source_epoch = source.source_epoch;
  target->source_sequence = source.source_sequence;
  target->accepted_sequence = source.accepted_sequence;
  target->final_output_sequence = source.final_output_sequence;
  target->endpoint_timestamp_s = source.endpoint_timestamp_s;
  copyString(target->reason, source.reason);
}

void copyGoalStatus(
    lingtu_nav_navigation_goal_status* target,
    const lingtu::nav::commands::NavigationGoalStatusSnapshot& source) {
  std::memset(target, 0, sizeof(*target));
  target->timestamp_s = source.timestamp_s;
  copyString(target->frame_id, source.frame_id);
  copyString(target->boot_id, source.boot_id);
  target->sequence = source.sequence;
  copyString(target->request_id, source.request_id);
  target->state = source.state;
  target->goal_epoch = source.goal_epoch;
  copyString(target->reason, source.reason);
}

enum class PathKind {
  Global,
  Local,
};

int takePath(
    lingtu_nav_client_handle raw_handle,
    lingtu_nav_path_header* header,
    lingtu_nav_path_point* points,
    unsigned long long point_capacity,
    PathKind kind) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || header == nullptr ||
      (point_capacity > 0U && points == nullptr)) {
    thread_error = "navigation path take received an invalid argument";
    return -1;
  }
  try {
    std::lock_guard<std::mutex> lock(handle->path_mutex);
    auto& staging = kind == PathKind::Global
        ? handle->global_path_staging
        : handle->local_path_staging;
    if (!staging.has_value()) {
      lingtu::nav::commands::PathSnapshot snapshot;
      const bool available = kind == PathKind::Global
          ? handle->client->takeGlobalPath(&snapshot)
          : handle->client->takeLocalPath(&snapshot);
      if (!available) {
        std::memset(header, 0, sizeof(*header));
        thread_error.clear();
        return 0;
      }
      staging = std::move(snapshot);
    }

    std::memset(header, 0, sizeof(*header));
    header->timestamp_s = staging->timestamp_s;
    copyString(header->frame_id, staging->frame_id);
    header->receive_sequence = staging->receive_sequence;
    header->point_count =
        static_cast<unsigned long long>(staging->points.size());
    if (header->point_count > point_capacity) {
      thread_error.clear();
      return 2;
    }
    for (std::size_t i = 0U; i < staging->points.size(); ++i) {
      points[i].x = staging->points[i].x;
      points[i].y = staging->points[i].y;
      points[i].z = staging->points[i].z;
    }
    staging.reset();
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation path take failure";
    return -1;
  }
}

void copyMapSceneGridHeader(
    lingtu_nav_map_scene_grid_header_v1* target,
    const lingtu::nav::commands::MapSceneGridSnapshot& source) {
  std::memset(target, 0, sizeof(*target));
  target->width = source.width;
  target->height = source.height;
  target->resolution = source.resolution;
  target->origin_x = source.origin_x;
  target->origin_y = source.origin_y;
  target->origin_z = source.origin_z;
  target->origin_qx = source.origin_qx;
  target->origin_qy = source.origin_qy;
  target->origin_qz = source.origin_qz;
  target->origin_qw = source.origin_qw;
  target->cell_count =
      static_cast<unsigned long long>(source.cells.size());
}

void copyMapSceneHeader(
    lingtu_nav_map_scene_header_v1* target,
    const lingtu::nav::commands::MapSceneSnapshot& source) {
  std::memset(target, 0, sizeof(*target));
  target->abi_version = LINGTU_NAV_MAP_SCENE_ABI_VERSION;
  target->struct_size = sizeof(*target);
  target->timestamp_s = source.timestamp_s;
  copyString(target->frame_id, source.frame_id);
  copyString(target->producer_boot_id, source.producer_boot_id);
  target->receive_sequence = source.receive_sequence;
  target->reset_epoch = source.reset_epoch;
  target->observation_sequence = source.observation_sequence;
  target->generation = source.generation;
  target->live = source.live ? 1 : 0;
  target->sensor_x = source.sensor_x;
  target->sensor_y = source.sensor_y;
  target->sensor_z = source.sensor_z;
  target->sensor_qx = source.sensor_qx;
  target->sensor_qy = source.sensor_qy;
  target->sensor_qz = source.sensor_qz;
  target->sensor_qw = source.sensor_qw;
  target->payload_bytes =
      static_cast<unsigned long long>(source.payload_bytes);
  target->live_point_count =
      static_cast<unsigned long long>(source.live_points.size());
  target->voxel_point_count =
      static_cast<unsigned long long>(source.voxel_points.size());
  target->accumulated_point_count =
      static_cast<unsigned long long>(source.accumulated_points.size());
  copyMapSceneGridHeader(&target->occupancy, source.occupancy);
  copyMapSceneGridHeader(&target->elevation, source.elevation);
  copyMapSceneGridHeader(&target->esdf, source.esdf);
  copyMapSceneGridHeader(&target->traversability, source.traversability);
}

bool mapSceneBuffersTooSmall(
    const lingtu_nav_map_scene_buffers_v1* buffers,
    const lingtu::nav::commands::MapSceneSnapshot& source) {
  if (buffers == nullptr) {
    return !source.live_points.empty() || !source.voxel_points.empty() ||
        !source.accumulated_points.empty() ||
        !source.occupancy.cells.empty() ||
        !source.elevation.cells.empty() || !source.esdf.cells.empty() ||
        !source.traversability.cells.empty();
  }
  return buffers->live_point_capacity < source.live_points.size() ||
      buffers->voxel_point_capacity < source.voxel_points.size() ||
      buffers->accumulated_point_capacity <
          source.accumulated_points.size() ||
      buffers->occupancy_cell_capacity < source.occupancy.cells.size() ||
      buffers->elevation_cell_capacity < source.elevation.cells.size() ||
      buffers->esdf_cell_capacity < source.esdf.cells.size() ||
      buffers->traversability_cell_capacity <
          source.traversability.cells.size();
}

bool mapSceneBufferPointersValid(
    const lingtu_nav_map_scene_buffers_v1& buffers,
    const lingtu::nav::commands::MapSceneSnapshot& source) {
  return (source.live_points.empty() || buffers.live_points != nullptr) &&
      (source.voxel_points.empty() || buffers.voxel_points != nullptr) &&
      (source.accumulated_points.empty() ||
       buffers.accumulated_points != nullptr) &&
      (source.occupancy.cells.empty() ||
       buffers.occupancy_cells != nullptr) &&
      (source.elevation.cells.empty() ||
       buffers.elevation_cells != nullptr) &&
      (source.esdf.cells.empty() || buffers.esdf_cells != nullptr) &&
      (source.traversability.cells.empty() ||
       buffers.traversability_cells != nullptr);
}

void copyMapScenePoints(
    lingtu_nav_map_scene_point_v1* target,
    const std::vector<lingtu::nav::commands::MapScenePoint>& source) {
  for (std::size_t index = 0U; index < source.size(); ++index) {
    target[index].x = source[index].x;
    target[index].y = source[index].y;
    target[index].z = source[index].z;
    target[index].intensity = source[index].intensity;
  }
}

void copyMapSceneCells(float* target, const std::vector<float>& source) {
  if (!source.empty()) {
    std::copy(source.begin(), source.end(), target);
  }
}

}  // namespace

extern "C" {

uint32_t lingtu_nav_client_abi_version(void) {
  return LINGTU_NAV_CLIENT_ABI_VERSION;
}

uint64_t lingtu_nav_client_capabilities(void) {
  return LINGTU_NAV_CLIENT_CAP_NAVIGATION |
      LINGTU_NAV_CLIENT_CAP_INSPECTION |
      LINGTU_NAV_CLIENT_CAP_EXPLORATION |
      LINGTU_NAV_CLIENT_CAP_DIRECTED_EXPLORATION |
      LINGTU_NAV_CLIENT_CAP_OPERATOR_MOTION |
      LINGTU_NAV_CLIENT_CAP_HOST_STATE |
      LINGTU_NAV_CLIENT_CAP_GOAL_STATUS |
      LINGTU_NAV_CLIENT_CAP_PATH_TELEMETRY |
      LINGTU_NAV_CLIENT_CAP_MAP_SCENE |
      LINGTU_NAV_CLIENT_CAP_OPERATOR_MOTION_RECEIPT;
}

lingtu_nav_client_handle lingtu_nav_client_create(int domain_id) {
  try {
    thread_error.clear();
    return new Handle(domain_id);
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return nullptr;
  } catch (...) {
    thread_error = "unknown native navigation client creation failure";
    return nullptr;
  }
}

void lingtu_nav_client_destroy(lingtu_nav_client_handle handle) {
  delete asHandle(handle);
}

int lingtu_nav_client_send_goal(
    lingtu_nav_client_handle handle,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendGoal(x, y, z, yaw, timeout_ms);
  });
}

int lingtu_nav_client_send_goal_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendGoal(
        x, y, z, yaw, timeout_ms, request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_cancel(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().cancel(reason == nullptr ? "cancel" : reason, timeout_ms);
  });
}

int lingtu_nav_client_cancel_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().cancel(
        reason == nullptr ? "cancel" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_send_teleop(
    lingtu_nav_client_handle handle,
    double vx,
    double vy,
    double wz,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendTeleop(vx, vy, wz, timeout_ms);
  });
}

int lingtu_nav_client_send_teleop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double vx,
    double vy,
    double wz,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().sendTeleop(
        vx, vy, wz, timeout_ms, request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_stop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().stop(reason == nullptr ? "stop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_stop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().stop(
        reason == nullptr ? "stop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().estop(reason == nullptr ? "estop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().estop(
        reason == nullptr ? "estop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_clear_estop(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().clearEstop(
        reason == nullptr ? "clear_estop" : reason, timeout_ms);
  });
}

int lingtu_nav_client_clear_estop_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().clearEstop(
        reason == nullptr ? "clear_estop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_autonomy(
    lingtu_nav_client_handle handle,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().resumeAutonomy(
        reason == nullptr ? "resume_autonomy" : reason,
        timeout_ms);
  });
}

int lingtu_nav_client_resume_autonomy_with_id(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.navigation().resumeAutonomy(
        reason == nullptr ? "resume_autonomy" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_start_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* session_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().start(
        session_id == nullptr ? "" : session_id,
        reason == nullptr ? "operator_start" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_pause_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().pause(
        reason == nullptr ? "operator_pause" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().resume(
        reason == nullptr ? "operator_resume" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_stop_exploration(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().stop(
        reason == nullptr ? "operator_stop" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_set_directed_exploration_target(
    lingtu_nav_client_handle handle,
    const char* request_id,
    double x,
    double y,
    double ttl_s,
    const char* session_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().setDirectedTarget(
        x,
        y,
        ttl_s,
        session_id == nullptr ? "" : session_id,
        reason == nullptr ? "operator_directed_explore" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_clear_directed_exploration_target(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* session_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.exploration().clearDirectedTarget(
        session_id == nullptr ? "" : session_id,
        reason == nullptr ? "operator_clear_directed_explore" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_start_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* route_id,
    unsigned long long route_revision,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().start(
        route_id == nullptr ? "" : route_id,
        route_revision,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_pause_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().pause(
        reason == nullptr ? "operator_pause" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_resume_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().resume(
        reason == nullptr ? "operator_resume" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_cancel_inspection(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.inspection().cancel(
        reason == nullptr ? "operator_cancel" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_operator_motion_claim(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    unsigned int lease_ttl_ms,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.operatorMotion().claim(
        source_id == nullptr ? "" : source_id,
        source_epoch,
        sequence,
        lease_ttl_ms,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_operator_motion_sample(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    int deadman,
    double vx,
    double vy,
    double wz,
    unsigned int freshness_budget_ms,
    int timeout_ms) {
  if (deadman != 0 && deadman != 1) {
    thread_error = "operator motion deadman must be 0 or 1";
    return -1;
  }
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.operatorMotion().sample(
        source_id == nullptr ? "" : source_id,
        source_epoch,
        sequence,
        vx,
        vy,
        wz,
        deadman != 0,
        freshness_budget_ms,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_operator_motion_hold(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.operatorMotion().hold(
        source_id == nullptr ? "" : source_id,
        source_epoch,
        sequence,
        reason == nullptr ? "operator_hold" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}

int lingtu_nav_client_operator_motion_release(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms) {
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    client.operatorMotion().release(
        source_id == nullptr ? "" : source_id,
        source_epoch,
        sequence,
        reason == nullptr ? "operator_release" : reason,
        timeout_ms,
        request_id == nullptr ? "" : request_id);
  });
}
int lingtu_nav_client_operator_motion_claim_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    unsigned int lease_ttl_ms,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt) {
  if (!validateOperatorMotionReceiptBuffer(receipt, "operator motion claim")) {
    return -1;
  }
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    copyOperatorMotionReceipt(
        receipt,
        client.operatorMotion().claimWithReceipt(
            source_id == nullptr ? "" : source_id,
            source_epoch,
            sequence,
            lease_ttl_ms,
            timeout_ms,
            request_id == nullptr ? "" : request_id));
  });
}

int lingtu_nav_client_operator_motion_hold_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt) {
  if (!validateOperatorMotionReceiptBuffer(receipt, "operator motion hold")) {
    return -1;
  }
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    copyOperatorMotionReceipt(
        receipt,
        client.operatorMotion().holdWithReceipt(
            source_id == nullptr ? "" : source_id,
            source_epoch,
            sequence,
            reason == nullptr ? "operator_hold" : reason,
            timeout_ms,
            request_id == nullptr ? "" : request_id));
  });
}

int lingtu_nav_client_operator_motion_release_with_receipt_v1(
    lingtu_nav_client_handle handle,
    const char* request_id,
    const char* source_id,
    unsigned long long source_epoch,
    unsigned long long sequence,
    const char* reason,
    int timeout_ms,
    lingtu_nav_operator_motion_receipt_v1* receipt) {
  if (!validateOperatorMotionReceiptBuffer(
          receipt, "operator motion release")) {
    return -1;
  }
  return invoke(handle, [&](lingtu::nav::commands::Client& client) {
    copyOperatorMotionReceipt(
        receipt,
        client.operatorMotion().releaseWithReceipt(
            source_id == nullptr ? "" : source_id,
            source_epoch,
            sequence,
            reason == nullptr ? "operator_release" : reason,
            timeout_ms,
            request_id == nullptr ? "" : request_id));
  });
}

int lingtu_nav_client_read_navigation_state(
    lingtu_nav_client_handle raw_handle,
    lingtu_nav_navigation_state* state) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || state == nullptr) {
    thread_error = "navigation state read received a null argument";
    return -1;
  }
  try {
    const auto snapshot = handle->client->latestNavigationState();
    if (!snapshot.has_value()) {
      thread_error.clear();
      return 0;
    }
    std::memset(state, 0, sizeof(*state));
    state->timestamp_s = snapshot->timestamp_s;
    copyString(state->frame_id, snapshot->frame_id);
    copyString(state->boot_id, snapshot->boot_id);
    state->sequence = snapshot->sequence;
    state->control_mode = snapshot->control_mode;
    state->lifecycle_state = snapshot->lifecycle_state;
    copyString(state->active_request_id, snapshot->active_request_id);
    state->goal_epoch = snapshot->goal_epoch;
    copyString(state->map_id, snapshot->map_id);
    state->map_version = snapshot->map_version;
    copyString(state->map_hash, snapshot->map_hash);
    state->planning_state = snapshot->planning_state;
    state->execution_state = snapshot->execution_state;
    state->recovery_state = snapshot->recovery_state;
    state->progress = snapshot->progress;
    copyString(state->authority, snapshot->authority);
    copyString(state->hold_reason, snapshot->hold_reason);
    copyString(state->failure_code, snapshot->failure_code);
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation state read failure";
    return -1;
  }
}

int lingtu_nav_client_take_navigation_goal_status(
    lingtu_nav_client_handle raw_handle,
    lingtu_nav_navigation_goal_status* status) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || status == nullptr) {
    thread_error = "navigation goal status take received a null argument";
    return -1;
  }
  try {
    lingtu::nav::commands::NavigationGoalStatusSnapshot snapshot;
    if (!handle->client->takeNavigationGoalStatus(&snapshot)) {
      thread_error.clear();
      return 0;
    }
    copyGoalStatus(status, snapshot);
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation goal status take failure";
    return -1;
  }
}

int lingtu_nav_client_get_navigation_goal_status(
    lingtu_nav_client_handle raw_handle,
    const char* request_id,
    lingtu_nav_navigation_goal_status* status) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || request_id == nullptr ||
      *request_id == '\0' || status == nullptr) {
    thread_error = "navigation goal status lookup received an invalid argument";
    return -1;
  }
  try {
    const auto snapshot =
        handle->client->navigationGoalStatus(request_id);
    if (!snapshot.has_value()) {
      thread_error.clear();
      return 0;
    }
    copyGoalStatus(status, *snapshot);
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native navigation goal status lookup failure";
    return -1;
  }
}

int lingtu_nav_client_take_global_path(
    lingtu_nav_client_handle handle,
    lingtu_nav_path_header* header,
    lingtu_nav_path_point* points,
    unsigned long long point_capacity) {
  return takePath(
      handle,
      header,
      points,
      point_capacity,
      PathKind::Global);
}

int lingtu_nav_client_take_local_path(
    lingtu_nav_client_handle handle,
    lingtu_nav_path_header* header,
    lingtu_nav_path_point* points,
    unsigned long long point_capacity) {
  return takePath(
      handle,
      header,
      points,
      point_capacity,
      PathKind::Local);
}

int lingtu_nav_client_take_map_scene_v1(
    lingtu_nav_client_handle raw_handle,
    lingtu_nav_map_scene_header_v1* header,
    const lingtu_nav_map_scene_buffers_v1* buffers) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || header == nullptr ||
      (buffers != nullptr &&
       (buffers->abi_version != LINGTU_NAV_MAP_SCENE_ABI_VERSION ||
        buffers->struct_size < sizeof(*buffers)))) {
    thread_error = "map scene take received an invalid argument";
    return -1;
  }
  try {
    std::lock_guard<std::mutex> lock(handle->map_scene_mutex);
    if (!handle->map_scene_staging.has_value()) {
      lingtu::nav::commands::MapSceneSnapshot snapshot;
      if (!handle->client->takeMapScene(&snapshot)) {
        std::memset(header, 0, sizeof(*header));
        header->abi_version = LINGTU_NAV_MAP_SCENE_ABI_VERSION;
        header->struct_size = sizeof(*header);
        thread_error.clear();
        return 0;
      }
      handle->map_scene_staging = std::move(snapshot);
    }
    const auto& scene = *handle->map_scene_staging;
    copyMapSceneHeader(header, scene);
    if (mapSceneBuffersTooSmall(buffers, scene)) {
      ++handle->map_scene_consumer_buffer_retries;
      thread_error.clear();
      return 2;
    }
    if (buffers == nullptr) {
      handle->map_scene_staging.reset();
      thread_error.clear();
      return 1;
    }
    if (!mapSceneBufferPointersValid(*buffers, scene)) {
      thread_error = "map scene buffers contain a null payload pointer";
      return -1;
    }
    copyMapScenePoints(buffers->live_points, scene.live_points);
    copyMapScenePoints(buffers->voxel_points, scene.voxel_points);
    copyMapScenePoints(
        buffers->accumulated_points, scene.accumulated_points);
    copyMapSceneCells(buffers->occupancy_cells, scene.occupancy.cells);
    copyMapSceneCells(buffers->elevation_cells, scene.elevation.cells);
    copyMapSceneCells(buffers->esdf_cells, scene.esdf.cells);
    copyMapSceneCells(
        buffers->traversability_cells, scene.traversability.cells);
    handle->map_scene_staging.reset();
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native map scene take failure";
    return -1;
  }
}

int lingtu_nav_client_read_map_scene_health_v1(
    lingtu_nav_client_handle raw_handle,
    lingtu_nav_map_scene_health_v1* health) {
  Handle* handle = asHandle(raw_handle);
  if (handle == nullptr || handle->client == nullptr || health == nullptr) {
    thread_error = "map scene health read received a null argument";
    return -1;
  }
  try {
    std::lock_guard<std::mutex> lock(handle->map_scene_mutex);
    const auto source = handle->client->mapSceneHealth();
    std::memset(health, 0, sizeof(*health));
    health->abi_version = LINGTU_NAV_MAP_SCENE_ABI_VERSION;
    health->struct_size = sizeof(*health);
    health->received_samples = source.received_samples;
    health->valid_samples = source.valid_samples;
    health->stale_samples = source.stale_samples;
    health->invalid_samples = source.invalid_samples;
    health->capacity_rejections = source.capacity_rejections;
    health->replaced_samples = source.replaced_samples;
    health->consumer_buffer_retries =
        handle->map_scene_consumer_buffer_retries;
    health->last_receive_sequence = source.last_receive_sequence;
    health->last_generation = source.last_generation;
    health->last_sample_timestamp_s = source.last_sample_timestamp_s;
    health->pending =
        source.pending || handle->map_scene_staging.has_value() ? 1 : 0;
    copyString(health->last_error, source.last_error);
    health->state_received_samples = source.state_received_samples;
    health->state_valid_samples = source.state_valid_samples;
    health->state_stale_samples = source.state_stale_samples;
    health->state_invalid_samples = source.state_invalid_samples;
    health->state_timestamp_s = source.state_timestamp_s;
    copyString(
        health->state_producer_boot_id,
        source.state_producer_boot_id);
    health->state_received = source.state_received ? 1 : 0;
    health->state_running = source.state_running ? 1 : 0;
    health->state_live = source.state_live ? 1 : 0;
    health->state_required_publications_ready =
        source.state_required_publications_ready ? 1 : 0;
    health->state_current_generation_published =
        source.state_current_generation_published ? 1 : 0;
    health->state_capacity_limited =
        source.state_capacity_limited ? 1 : 0;
    health->state_reset_epoch = source.state_reset_epoch;
    health->state_observation_sequence =
        source.state_observation_sequence;
    health->state_generation = source.state_generation;
    health->state_scene_published_generation =
        source.state_scene_published_generation;
    copyString(health->state_error, source.state_error);
    thread_error.clear();
    return 1;
  } catch (const std::exception& exc) {
    thread_error = exc.what();
    return -1;
  } catch (...) {
    thread_error = "unknown native map scene health read failure";
    return -1;
  }
}

const char* lingtu_nav_client_last_error(lingtu_nav_client_handle handle) {
  (void)handle;
  return thread_error.c_str();
}

}  // extern "C"
