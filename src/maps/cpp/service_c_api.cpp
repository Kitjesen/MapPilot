#include "lingtu/maps/c_api/service.h"

#include "lingtu/maps/service.hpp"

#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <utility>

using lingtu::maps::MapsServiceConfig;
using lingtu::maps::MapsServiceCore;
using lingtu::maps::MapSnapshot;
using lingtu::maps::SaveMapRequest;

struct LingtuMapsServiceHandle {
  explicit LingtuMapsServiceHandle(MapsServiceConfig config) : service(std::move(config)) {}
  MapsServiceCore service;
};

namespace {

bool EmptyOrNull(const char* value) {
  return value == nullptr || value[0] == '\0';
}

constexpr uint64_t kCommandJsonCapacityHint = 1024U * 1024U;

template <typename T>
bool ValidSaveStruct(const T* value) {
  return value != nullptr &&
      value->struct_size >= sizeof(T) &&
      value->abi_version == LINGTU_MAPS_SAVE_MAP_ABI_VERSION;
}

SaveMapRequest ToSaveMapRequest(const LingtuMapsSaveMapRequest& input) {
  SaveMapRequest request;
  request.request_id = EmptyOrNull(input.request_id) ? "" : input.request_id;
  request.map_id = EmptyOrNull(input.map_id) ? "" : input.map_id;
  request.require.occupancy = input.require.occupancy != 0U;
  request.require.octomap = input.require.octomap != 0U;
  request.require.esdf = input.require.esdf != 0U;
  request.require.traversability = input.require.traversability != 0U;
  request.require.semantic = input.require.semantic != 0U;
  request.source.voxel_size = input.source.voxel_size;
  request.source.dynamic_filter_enabled = input.source.dynamic_filter_enabled != 0U;
  request.source.dynamic_filter_required = input.source.dynamic_filter_required != 0U;
  request.source.dynamic_filter_command =
      EmptyOrNull(input.source.dynamic_filter_command) ? "" : input.source.dynamic_filter_command;
  request.source.dynamic_filter_timeout_sec = input.source.dynamic_filter_timeout_sec;
  request.source.optimizer_strategy =
      EmptyOrNull(input.source.optimizer_strategy) ? "none" : input.source.optimizer_strategy;
  request.source.optimizer_required = input.source.optimizer_required != 0U;
  request.source.optimizer_command =
      EmptyOrNull(input.source.optimizer_command) ? "" : input.source.optimizer_command;
  request.source.optimizer_timeout_sec = input.source.optimizer_timeout_sec;
  request.octomap.converter_command =
      EmptyOrNull(input.octomap.converter_command) ? "" : input.octomap.converter_command;
  request.octomap.build_mode =
      EmptyOrNull(input.octomap.build_mode) ? "external_pcl_converter" : input.octomap.build_mode;
  request.octomap.resolution = input.octomap.resolution;
  request.octomap.support_dilation_cells = input.octomap.support_dilation_cells;
  request.octomap.free_layers_above = input.octomap.free_layers_above;
  request.octomap.free_dilation_cells = input.octomap.free_dilation_cells;
  request.octomap.frame_id = EmptyOrNull(input.octomap.frame_id) ? "map" : input.octomap.frame_id;
  request.octomap.source_profile =
      EmptyOrNull(input.octomap.source_profile) ? "map_pipeline" : input.octomap.source_profile;
  request.octomap.data_source =
      EmptyOrNull(input.octomap.data_source) ? request.octomap.source_profile : input.octomap.data_source;
  request.octomap.slam_source =
      EmptyOrNull(input.octomap.slam_source) ? "unknown" : input.octomap.slam_source;
  request.octomap.localization_source = EmptyOrNull(input.octomap.localization_source)
      ? request.octomap.slam_source
      : input.octomap.localization_source;
  request.octomap.mapping_source = EmptyOrNull(input.octomap.mapping_source)
      ? "lingtu_maps_pipeline"
      : input.octomap.mapping_source;
  request.octomap.timeout_sec = input.octomap.timeout_sec;
  request.activate_on_success = input.activate_on_success != 0U;
  request.require_slam_healthy = input.require_slam_healthy != 0U;
  request.minimum_point_count = input.minimum_point_count;
  return request;
}

MapSnapshot ToMapSnapshot(const LingtuMapsMapSnapshot& input) {
  MapSnapshot snapshot;
  snapshot.snapshot_id = EmptyOrNull(input.snapshot_id) ? "" : input.snapshot_id;
  snapshot.source_dir = EmptyOrNull(input.source_dir) ? "" : input.source_dir;
  snapshot.frame_id = EmptyOrNull(input.frame_id) ? "map" : input.frame_id;
  snapshot.captured_at_ns = input.captured_at_ns;
  snapshot.first_sequence = input.first_sequence;
  snapshot.last_sequence = input.last_sequence;
  snapshot.source_sha256 = EmptyOrNull(input.source_sha256) ? "" : input.source_sha256;
  snapshot.slam_healthy = input.slam_healthy != 0U;
  snapshot.health_message = EmptyOrNull(input.health_message) ? "" : input.health_message;
  return snapshot;
}

int32_t WriteString(const std::string& value, char* out, uint64_t capacity, uint64_t* out_size) {
  const uint64_t required = static_cast<uint64_t>(value.size() + 1U);
  if (out_size != nullptr) {
    *out_size = required;
  }
  if (out == nullptr || capacity == 0U) {
    return required == 1U ? 0 : 1;
  }
  if (capacity < required) {
    const uint64_t copy_count = capacity - 1U;
    std::memcpy(out, value.data(), static_cast<size_t>(copy_count));
    out[copy_count] = '\0';
    return 1;
  }
  std::memcpy(out, value.c_str(), static_cast<size_t>(required));
  return 0;
}

template <typename Fn>
int32_t WriteProtected(Fn&& fn, char* out, uint64_t capacity, uint64_t* out_size) {
  try {
    return WriteString(fn(), out, capacity, out_size);
  } catch (const std::exception&) {
    return -2;
  } catch (...) {
    return -3;
  }
}

template <typename Fn>
int32_t WriteCommandProtected(Fn&& fn, char* out, uint64_t capacity, uint64_t* out_size) {
  thread_local std::string pending_response;
  if (out == nullptr || capacity == 0U) {
    pending_response.clear();
    if (out_size != nullptr) {
      *out_size = kCommandJsonCapacityHint;
    }
    return 1;
  }
  try {
    if (pending_response.empty()) {
      pending_response = fn();
    }
    const int32_t rc = WriteString(pending_response, out, capacity, out_size);
    if (rc == 0) {
      pending_response.clear();
    }
    return rc;
  } catch (const std::exception&) {
    pending_response.clear();
    return -2;
  } catch (...) {
    pending_response.clear();
    return -3;
  }
}

}  // namespace

extern "C" {

LingtuMapsServiceHandle* lingtu_maps_service_create(
    const char* root_dir,
    const char* active_state_filename) {
  if (EmptyOrNull(root_dir)) {
    return nullptr;
  }
  try {
    MapsServiceConfig config;
    config.store.root_dir = root_dir;
    if (!EmptyOrNull(active_state_filename)) {
      config.store.active_state_filename = active_state_filename;
    }
    return new LingtuMapsServiceHandle(std::move(config));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_maps_service_destroy(LingtuMapsServiceHandle* handle) {
  delete handle;
}

int32_t lingtu_maps_service_list_maps_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected([&]() { return handle->service.ListMapsJson(); }, out, capacity, out_size);
}

int32_t lingtu_maps_service_get_map_types_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected([&]() { return handle->service.GetMapTypesJson(); }, out, capacity, out_size);
}

int32_t lingtu_maps_service_get_record_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetRecordJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_active_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected([&]() { return handle->service.GetActiveMapJson(); }, out, capacity, out_size);
}

int32_t lingtu_maps_service_get_health_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetHealthJson(EmptyOrNull(map_id) ? "" : map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_validate_artifacts_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t require_octomap,
    uint8_t require_occupancy,
    const char* expected_frame_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() {
        return handle->service.ValidateArtifactsJson(
            map_id,
            require_octomap != 0U,
            require_occupancy != 0U,
            EmptyOrNull(expected_frame_id) ? "" : expected_frame_id);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_bundle_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* capability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(capability)) {
    return -1;
  }
  return WriteProtected(
      [&]() {
        return handle->service.GetBundleJson(EmptyOrNull(map_id) ? "" : map_id, capability);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_map_points_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint64_t max_points,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() {
        return handle->service.GetMapPointsJson(
            EmptyOrNull(map_id) ? "" : map_id,
            max_points);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_save_map_begin_json(
    LingtuMapsServiceHandle* handle,
    const LingtuMapsSaveMapRequest* request,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || !ValidSaveStruct(request) ||
      !ValidSaveStruct(&request->require) ||
      !ValidSaveStruct(&request->source) ||
      !ValidSaveStruct(&request->octomap) ||
      EmptyOrNull(request->request_id) || EmptyOrNull(request->map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BeginSaveMapJson(ToSaveMapRequest(*request)); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_save_map_provide_snapshot_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    const LingtuMapsMapSnapshot* snapshot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(job_id) || !ValidSaveStruct(snapshot) ||
      EmptyOrNull(snapshot->source_dir)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.ProvideSaveMapSnapshotJson(job_id, ToMapSnapshot(*snapshot));
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_save_map_status_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(job_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetSaveMapStatusJson(job_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_list_save_map_jobs_json(
    LingtuMapsServiceHandle* handle,
    uint64_t limit,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() {
        return handle->service.ListSaveMapJobsJson(
            static_cast<std::size_t>(limit == 0U ? 100U : limit));
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_save_map_cancel_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(job_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.CancelSaveMapJson(job_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_save_map_retry_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(job_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RetrySaveMapJson(job_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_list_map_versions_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.ListMapVersionsJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_rollback_map_version_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    int64_t version,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || version <= 0) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RollbackMapVersionJson(map_id, version); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_list_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.ListPoiJson(EmptyOrNull(map_id) ? "" : map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_set_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* name,
    double x_m,
    double y_m,
    double z_m,
    double yaw_rad,
    uint8_t has_yaw,
    const char* frame_id,
    const char* tags_json,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(name)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.SetPoiJson(
            EmptyOrNull(map_id) ? "" : map_id,
            name,
            x_m,
            y_m,
            z_m,
            yaw_rad,
            has_yaw != 0U,
            EmptyOrNull(frame_id) ? "map" : frame_id,
            EmptyOrNull(tags_json) ? "{}" : tags_json);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_delete_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* name,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(name)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.DeletePoiJson(EmptyOrNull(map_id) ? "" : map_id, name);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_list_map_graph_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected([&]() { return handle->service.ListMapGraphJson(); }, out, capacity, out_size);
}

int32_t lingtu_maps_service_set_map_edge_json(
    LingtuMapsServiceHandle* handle,
    const char* from_map_id,
    const char* to_map_id,
    const char* edge_type,
    uint8_t bidirectional,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(from_map_id) || EmptyOrNull(to_map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.SetMapEdgeJson(
            from_map_id,
            to_map_id,
            EmptyOrNull(edge_type) ? "link" : edge_type,
            bidirectional != 0U);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_delete_map_edge_json(
    LingtuMapsServiceHandle* handle,
    const char* from_map_id,
    const char* to_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(from_map_id) || EmptyOrNull(to_map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.DeleteMapEdgeJson(from_map_id, to_map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_shortest_route_json(
    LingtuMapsServiceHandle* handle,
    const char* start_map_id,
    const char* goal_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(start_map_id) || EmptyOrNull(goal_map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.ShortestRouteJson(start_map_id, goal_map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_rollback_active_map_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RollbackActiveMapJson(); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_list_active_slots_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.ListActiveSlotsJson(); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(slot)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetActiveSlotJson(slot); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_set_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    const char* map_id,
    uint8_t strict,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(slot) || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.SetActiveSlotJson(slot, map_id, strict != 0U); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_clear_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(slot)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.ClearActiveSlotJson(slot); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_build_queue_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetBuildQueueJson(); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_enqueue_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* artifact_type,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(artifact_type)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.EnqueueBuildJson(map_id, artifact_type); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_pop_build_queue_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.PopBuildQueueJson(); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(request_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.GetArtifactJobJson(request_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_cancel_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(request_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.CancelArtifactJobJson(request_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_retry_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(request_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RetryArtifactJobJson(request_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_audit_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) return -1;
  return WriteCommandProtected(
      [&]() { return handle->service.AuditMapVersionsJson(dry_run != 0U); },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_quarantine_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) return -1;
  return WriteCommandProtected(
      [&]() { return handle->service.QuarantineCorruptVersionsJson(dry_run != 0U); },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_gc_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) return -1;
  return WriteCommandProtected(
      [&]() { return handle->service.GarbageCollectVersionsJson(dry_run != 0U); },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_migrate_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) return -1;
  return WriteCommandProtected(
      [&]() { return handle->service.MigrateMapSchemasJson(dry_run != 0U); },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_export_version_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    int64_t version,
    const char* package_dir,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(package_dir)) return -1;
  return WriteCommandProtected(
      [&]() {
        return handle->service.ExportMapVersionJson(
            map_id, version, package_dir, dry_run != 0U);
      },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_import_package_json(
    LingtuMapsServiceHandle* handle,
    const char* package_dir,
    const char* requested_map_id,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(package_dir)) return -1;
  return WriteCommandProtected(
      [&]() {
        return handle->service.ImportMapPackageJson(
            package_dir,
            EmptyOrNull(requested_map_id) ? "" : requested_map_id,
            dry_run != 0U);
      },
      out, capacity, out_size);
}

int32_t lingtu_maps_service_create_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.CreateMapJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_delete_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.DeleteMapJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_rename_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* new_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(new_map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RenameMapJson(map_id, new_map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_retire_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RetireMapJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_set_active_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t strict,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.SetActiveMapJson(map_id, strict != 0U); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_clear_active_map_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.ClearActiveMapJson(); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_begin_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* artifact_type,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(artifact_type)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BeginBuildJson(map_id, artifact_type); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_finish_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* build_id,
    uint8_t success,
    const char* message,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(build_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.FinishBuildJson(
            map_id,
            build_id,
            success != 0U,
            EmptyOrNull(message) ? "" : message);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_build_status_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetBuildStatusJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_import_pcd_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* source_path,
    double voxel_size,
    uint8_t has_bounds,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(source_path)) {
    return -1;
  }
  lingtu::maps::PcdBounds bounds;
  bounds.enabled = has_bounds != 0U;
  bounds.min_x = min_x;
  bounds.min_y = min_y;
  bounds.min_z = min_z;
  bounds.max_x = max_x;
  bounds.max_y = max_y;
  bounds.max_z = max_z;
  return WriteCommandProtected(
      [&]() {
        return handle->service.ImportPcdJson(map_id, source_path, voxel_size, bounds);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_commit_saved_source_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* source_dir,
    double voxel_size,
    uint8_t dynamic_filter_enabled,
    uint8_t dynamic_filter_required,
    const char* dynamic_filter_command,
    double dynamic_filter_timeout_sec,
    const char* optimizer_strategy,
    uint8_t optimizer_required,
    const char* optimizer_command,
    double optimizer_timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(source_dir)) {
    return -1;
  }
  lingtu::maps::SourceCommitOptions options;
  options.voxel_size = voxel_size;
  options.dynamic_filter_enabled = dynamic_filter_enabled != 0U;
  options.dynamic_filter_required = dynamic_filter_required != 0U;
  options.dynamic_filter_command =
      EmptyOrNull(dynamic_filter_command) ? "" : dynamic_filter_command;
  options.dynamic_filter_timeout_sec =
      dynamic_filter_timeout_sec > 0.0 ? dynamic_filter_timeout_sec : 300.0;
  options.optimizer_strategy = EmptyOrNull(optimizer_strategy) ? "pgo" : optimizer_strategy;
  options.optimizer_required = optimizer_required != 0U;
  options.optimizer_command = EmptyOrNull(optimizer_command) ? "" : optimizer_command;
  options.optimizer_timeout_sec = optimizer_timeout_sec > 0.0 ? optimizer_timeout_sec : 120.0;
  return WriteCommandProtected(
      [&]() {
        return handle->service.CommitSavedSourceJson(map_id, source_dir, options);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_crop_pcd_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t invert,
    double voxel_size,
    uint8_t has_bounds,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  lingtu::maps::PcdBounds bounds;
  bounds.enabled = has_bounds != 0U;
  bounds.min_x = min_x;
  bounds.min_y = min_y;
  bounds.min_z = min_z;
  bounds.max_x = max_x;
  bounds.max_y = max_y;
  bounds.max_z = max_z;
  return WriteCommandProtected(
      [&]() {
        return handle->service.CropPcdJson(map_id, bounds, invert != 0U, voxel_size);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_restore_source_backup_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.RestoreSourceBackupJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_occupancy_snapshot_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BuildOccupancySnapshotJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_octomap_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* converter_command,
    const char* build_mode,
    double resolution,
    int32_t support_dilation_cells,
    int32_t free_layers_above,
    int32_t free_dilation_cells,
    const char* frame_id,
    const char* source_profile,
    const char* data_source,
    const char* slam_source,
    const char* localization_source,
    const char* mapping_source,
    double timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  lingtu::maps::OctomapBuildOptions options;
  options.converter_command = EmptyOrNull(converter_command) ? "" : converter_command;
  options.build_mode = EmptyOrNull(build_mode) ? "external_pcl_converter" : build_mode;
  options.resolution = resolution > 0.0 ? resolution : 0.20;
  options.support_dilation_cells = support_dilation_cells;
  options.free_layers_above = free_layers_above;
  options.free_dilation_cells = free_dilation_cells;
  options.frame_id = EmptyOrNull(frame_id) ? "map" : frame_id;
  options.source_profile = EmptyOrNull(source_profile) ? "maps_pipeline" : source_profile;
  options.data_source = EmptyOrNull(data_source) ? options.source_profile : data_source;
  options.slam_source = EmptyOrNull(slam_source) ? "unknown" : slam_source;
  options.localization_source =
      EmptyOrNull(localization_source) ? options.slam_source : localization_source;
  options.mapping_source = EmptyOrNull(mapping_source) ? "lingtu_maps_pipeline" : mapping_source;
  options.timeout_sec = timeout_sec > 0.0 ? timeout_sec : 60.0;
  return WriteCommandProtected(
      [&]() { return handle->service.BuildOctomapArtifactJson(map_id, options); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_get_voxel_edits_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteProtected(
      [&]() { return handle->service.GetVoxelEditsJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_edit_octomap_voxels_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* editor_command,
    const char* state,
    const char* shape,
    double x_m,
    double y_m,
    double z_m,
    double radius_m,
    double timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(state)) {
    return -1;
  }
  lingtu::maps::OctomapEditOptions options;
  options.editor_command = EmptyOrNull(editor_command) ? "" : editor_command;
  options.state = state;
  options.shape = EmptyOrNull(shape) ? "sphere" : shape;
  options.x_m = x_m;
  options.y_m = y_m;
  options.z_m = z_m;
  options.radius_m = radius_m;
  options.timeout_sec = timeout_sec > 0.0 ? timeout_sec : 15.0;
  return WriteCommandProtected(
      [&]() { return handle->service.EditOctomapVoxelsJson(map_id, options); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_navigation_package_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* converter_command,
    const char* build_mode,
    double resolution,
    int32_t support_dilation_cells,
    int32_t free_layers_above,
    int32_t free_dilation_cells,
    const char* frame_id,
    const char* source_profile,
    const char* data_source,
    const char* slam_source,
    const char* localization_source,
    const char* mapping_source,
    double timeout_sec,
    uint8_t include_esdf,
    uint8_t include_traversability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  lingtu::maps::OctomapBuildOptions options;
  options.converter_command = EmptyOrNull(converter_command) ? "" : converter_command;
  options.build_mode = EmptyOrNull(build_mode) ? "external_pcl_converter" : build_mode;
  options.resolution = resolution > 0.0 ? resolution : 0.20;
  options.support_dilation_cells = support_dilation_cells;
  options.free_layers_above = free_layers_above;
  options.free_dilation_cells = free_dilation_cells;
  options.frame_id = EmptyOrNull(frame_id) ? "map" : frame_id;
  options.source_profile = EmptyOrNull(source_profile) ? "maps_pipeline" : source_profile;
  options.data_source = EmptyOrNull(data_source) ? options.source_profile : data_source;
  options.slam_source = EmptyOrNull(slam_source) ? "unknown" : slam_source;
  options.localization_source =
      EmptyOrNull(localization_source) ? options.slam_source : localization_source;
  options.mapping_source = EmptyOrNull(mapping_source) ? "lingtu_maps_pipeline" : mapping_source;
  options.timeout_sec = timeout_sec > 0.0 ? timeout_sec : 60.0;
  return WriteCommandProtected(
      [&]() {
        return handle->service.BuildNavigationPackageJson(
            map_id,
            options,
            include_esdf != 0U,
            include_traversability != 0U);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_esdf_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BuildEsdfArtifactJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_traversability_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BuildTraversabilityArtifactJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_build_semantic_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() { return handle->service.BuildSemanticArtifactJson(map_id); },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_ingest_localization_health_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    uint8_t localized,
    double position_error_m,
    double covariance_trace,
    double quality,
    const char* source,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.IngestLocalizationHealthJson(
            EmptyOrNull(map_id) ? "" : map_id,
            timestamp_s,
            localized != 0U,
            position_error_m,
            covariance_trace,
            quality,
            EmptyOrNull(source) ? "runtime.localization" : source);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_ingest_planning_outcome_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    uint8_t success,
    const char* planner,
    const char* reason,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.IngestPlanningOutcomeJson(
            EmptyOrNull(map_id) ? "" : map_id,
            timestamp_s,
            success != 0U,
            EmptyOrNull(planner) ? "unknown" : planner,
            EmptyOrNull(reason) ? "" : reason);
      },
      out,
      capacity,
      out_size);
}

int32_t lingtu_maps_service_ingest_collision_event_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    double severity,
    const char* source,
    const char* reason,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return WriteCommandProtected(
      [&]() {
        return handle->service.IngestCollisionEventJson(
            EmptyOrNull(map_id) ? "" : map_id,
            timestamp_s,
            severity,
            EmptyOrNull(source) ? "runtime.safety" : source,
            EmptyOrNull(reason) ? "" : reason);
      },
      out,
      capacity,
      out_size);
}

}  // extern "C"
