#include "lingtu/maps/c_api/rolling_occupancy.h"

#include "lingtu/maps/layers/rolling_occupancy.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>

using lingtu::maps::CloudLayout;
using lingtu::maps::MapCloudFrame;
using lingtu::maps::PointCloudView;
using lingtu::maps::layers::RollingOccupancyCellChunk;
using lingtu::maps::layers::RollingOccupancyConfig;
using lingtu::maps::layers::RollingOccupancyGrid;
using lingtu::maps::layers::RollingOccupancyUpdateStats;

struct LingtuMapsRollingOccupancyHandle {
  explicit LingtuMapsRollingOccupancyHandle(RollingOccupancyConfig config)
      : core(config) {}

  RollingOccupancyGrid core;
};

namespace {

RollingOccupancyConfig ToConfig(const LingtuMapsRollingOccupancyConfig* config) {
  if (config == nullptr) {
    return {};
  }
  RollingOccupancyConfig out;
  out.size_x = config->size_x;
  out.size_y = config->size_y;
  out.size_z = config->size_z;
  out.resolution_m = config->resolution_m;
  out.max_ray_range_m = config->max_ray_range_m;
  out.hit_log_odds = config->hit_log_odds;
  out.miss_log_odds = config->miss_log_odds;
  out.min_log_odds = config->min_log_odds;
  out.max_log_odds = config->max_log_odds;
  out.occupied_probability = config->occupied_probability;
  out.free_probability = config->free_probability;
  out.roll_margin_x = config->roll_margin_x;
  out.roll_margin_y = config->roll_margin_y;
  out.roll_margin_z = config->roll_margin_z;
  out.decay_after_ns = config->decay_after_ns;
  out.decay_factor = config->decay_factor;
  out.auto_roll = config->auto_roll != 0U;
  out.reject_out_of_order = config->reject_out_of_order != 0U;
  return out;
}

template <typename Function>
int32_t Protect(Function&& function) {
  try {
    function();
    return 0;
  } catch (const std::exception&) {
    return -2;
  } catch (...) {
    return -3;
  }
}

std::string FrameId(const char* frame_id) {
  return frame_id == nullptr || frame_id[0] == '\0' ? "map" : frame_id;
}

void CopyStats(
    const RollingOccupancyUpdateStats& source,
    LingtuMapsRollingOccupancyStats* target) {
  if (target == nullptr) {
    return;
  }
  target->input_points = static_cast<std::uint64_t>(source.input_points);
  target->accepted_points = static_cast<std::uint64_t>(source.accepted_points);
  target->rejected_points = static_cast<std::uint64_t>(source.rejected_points);
  target->unique_rays = static_cast<std::uint64_t>(source.unique_rays);
  target->free_updates = static_cast<std::uint64_t>(source.free_updates);
  target->hit_updates = static_cast<std::uint64_t>(source.hit_updates);
  target->rolled_out_cells = static_cast<std::uint64_t>(source.rolled_out_cells);
  target->decayed_cells = static_cast<std::uint64_t>(source.decayed_cells);
  target->generation = source.generation;
  target->rolled = source.rolled ? 1U : 0U;
}

MapCloudFrame SoaFrame(
    const float* x,
    const float* y,
    const float* z,
    std::size_t count,
    const char* frame_id,
    std::int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  PointCloudView cloud;
  cloud.frame_id = FrameId(frame_id);
  cloud.stamp_ns = stamp_ns;
  cloud.layout = CloudLayout::kXyzF32SoA;
  cloud.point_count = count;
  cloud.x = {x, count};
  cloud.y = {y, count};
  cloud.z = {z, count};
  MapCloudFrame frame;
  frame.cloud = cloud;
  frame.sensor_origin_x_m = origin_x_m;
  frame.sensor_origin_y_m = origin_y_m;
  frame.sensor_origin_z_m = origin_z_m;
  return frame;
}

MapCloudFrame InterleavedFrame(
    const float* xyz,
    std::size_t count,
    const char* frame_id,
    std::int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  PointCloudView cloud;
  cloud.frame_id = FrameId(frame_id);
  cloud.stamp_ns = stamp_ns;
  cloud.layout = CloudLayout::kXyzF32Interleaved;
  cloud.point_count = count;
  cloud.interleaved = {xyz, count * 3U};
  MapCloudFrame frame;
  frame.cloud = cloud;
  frame.sensor_origin_x_m = origin_x_m;
  frame.sensor_origin_y_m = origin_y_m;
  frame.sensor_origin_z_m = origin_z_m;
  return frame;
}

int32_t CopyChunk(
    const RollingOccupancyCellChunk& chunk,
    float* out_x_m,
    float* out_y_m,
    float* out_z_m,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_count) {
  if (out_count == nullptr) {
    return -1;
  }
  *out_count = static_cast<std::uint64_t>(chunk.Size());
  if (capacity < chunk.Size()) {
    return -4;
  }
  if (chunk.Empty()) {
    return 0;
  }
  if (out_x_m == nullptr || out_y_m == nullptr || out_z_m == nullptr ||
      out_state == nullptr || out_log_odds_q8 == nullptr) {
    return -1;
  }
  std::copy(chunk.center_x_m.begin(), chunk.center_x_m.end(), out_x_m);
  std::copy(chunk.center_y_m.begin(), chunk.center_y_m.end(), out_y_m);
  std::copy(chunk.center_z_m.begin(), chunk.center_z_m.end(), out_z_m);
  std::copy(chunk.state.begin(), chunk.state.end(), out_state);
  std::copy(chunk.log_odds_q8.begin(), chunk.log_odds_q8.end(), out_log_odds_q8);
  return 0;
}

bool FitsSizeT(std::uint64_t value) {
  return value <= static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max());
}

}  // namespace

extern "C" {

LingtuMapsRollingOccupancyHandle* lingtu_maps_rolling_occupancy_create(
    const LingtuMapsRollingOccupancyConfig* config) {
  try {
    return new LingtuMapsRollingOccupancyHandle(ToConfig(config));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_maps_rolling_occupancy_destroy(
    LingtuMapsRollingOccupancyHandle* handle) {
  delete handle;
}

int32_t lingtu_maps_rolling_occupancy_reset(
    LingtuMapsRollingOccupancyHandle* handle,
    const char* frame_id,
    float center_x_m,
    float center_y_m,
    float center_z_m,
    int64_t stamp_ns) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    handle->core.Reset(
        FrameId(frame_id), center_x_m, center_y_m, center_z_m, stamp_ns);
  });
}

int32_t lingtu_maps_rolling_occupancy_roll_to(
    LingtuMapsRollingOccupancyHandle* handle,
    float center_x_m,
    float center_y_m,
    float center_z_m,
    int64_t stamp_ns,
    uint64_t* out_rolled_cell_count) {
  if (handle == nullptr || out_rolled_cell_count == nullptr) {
    return -1;
  }
  return Protect([&]() {
    const auto chunk =
        handle->core.RollToCenter(center_x_m, center_y_m, center_z_m, stamp_ns);
    *out_rolled_cell_count = static_cast<std::uint64_t>(chunk.Size());
  });
}

int32_t lingtu_maps_rolling_occupancy_update_xyz_soa(
    LingtuMapsRollingOccupancyHandle* handle,
    const float* x,
    const float* y,
    const float* z,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float sensor_origin_x_m,
    float sensor_origin_y_m,
    float sensor_origin_z_m,
    LingtuMapsRollingOccupancyStats* out_stats) {
  if (handle == nullptr || !FitsSizeT(point_count) ||
      (point_count > 0U && (x == nullptr || y == nullptr || z == nullptr))) {
    return -1;
  }
  return Protect([&]() {
    const auto stats = handle->core.Update(SoaFrame(
        x,
        y,
        z,
        static_cast<std::size_t>(point_count),
        frame_id,
        stamp_ns,
        sensor_origin_x_m,
        sensor_origin_y_m,
        sensor_origin_z_m));
    CopyStats(stats, out_stats);
  });
}

int32_t lingtu_maps_rolling_occupancy_update_xyz_interleaved(
    LingtuMapsRollingOccupancyHandle* handle,
    const float* xyz,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float sensor_origin_x_m,
    float sensor_origin_y_m,
    float sensor_origin_z_m,
    LingtuMapsRollingOccupancyStats* out_stats) {
  if (handle == nullptr || !FitsSizeT(point_count) ||
      (point_count > 0U && xyz == nullptr)) {
    return -1;
  }
  return Protect([&]() {
    const auto stats = handle->core.Update(InterleavedFrame(
        xyz,
        static_cast<std::size_t>(point_count),
        frame_id,
        stamp_ns,
        sensor_origin_x_m,
        sensor_origin_y_m,
        sensor_origin_z_m));
    CopyStats(stats, out_stats);
  });
}

int32_t lingtu_maps_rolling_occupancy_decay(
    LingtuMapsRollingOccupancyHandle* handle,
    int64_t now_ns,
    uint64_t* out_decayed_cells) {
  if (handle == nullptr || out_decayed_cells == nullptr) {
    return -1;
  }
  return Protect([&]() {
    *out_decayed_cells = static_cast<std::uint64_t>(handle->core.Decay(now_ns));
  });
}

int32_t lingtu_maps_rolling_occupancy_query(
    const LingtuMapsRollingOccupancyHandle* handle,
    float x_m,
    float y_m,
    float z_m,
    uint8_t* out_state,
    float* out_probability) {
  if (handle == nullptr || out_state == nullptr || out_probability == nullptr) {
    return -1;
  }
  return Protect([&]() {
    *out_state = static_cast<std::uint8_t>(handle->core.StateAt(x_m, y_m, z_m));
    *out_probability = handle->core.OccupancyProbability(x_m, y_m, z_m);
  });
}

int32_t lingtu_maps_rolling_occupancy_snapshot_info(
    const LingtuMapsRollingOccupancyHandle* handle,
    LingtuMapsRollingOccupancySnapshotInfo* out_info) {
  if (handle == nullptr || out_info == nullptr) {
    return -1;
  }
  return Protect([&]() {
    const auto snapshot = handle->core.Snapshot();
    out_info->stamp_ns = snapshot.stamp_ns;
    out_info->generation = snapshot.generation;
    out_info->resolution_m = snapshot.resolution_m;
    out_info->size_x = snapshot.size_x;
    out_info->size_y = snapshot.size_y;
    out_info->size_z = snapshot.size_z;
    out_info->origin_x_m = snapshot.origin_x_m;
    out_info->origin_y_m = snapshot.origin_y_m;
    out_info->origin_z_m = snapshot.origin_z_m;
    out_info->cell_count = static_cast<std::uint64_t>(snapshot.CellCount());
  });
}

int32_t lingtu_maps_rolling_occupancy_snapshot_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_cell_count) {
  if (handle == nullptr || out_cell_count == nullptr) {
    return -1;
  }
  try {
    const auto snapshot = handle->core.Snapshot();
    *out_cell_count = static_cast<std::uint64_t>(snapshot.CellCount());
    if (capacity < snapshot.CellCount()) {
      return -4;
    }
    if (snapshot.CellCount() > 0U &&
        (out_state == nullptr || out_log_odds_q8 == nullptr)) {
      return -1;
    }
    std::copy(snapshot.state.begin(), snapshot.state.end(), out_state);
    std::copy(
        snapshot.log_odds_q8.begin(), snapshot.log_odds_q8.end(), out_log_odds_q8);
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_rolling_occupancy_observed_count(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint64_t* out_count) {
  if (handle == nullptr || out_count == nullptr) {
    return -1;
  }
  return Protect([&]() {
    *out_count = static_cast<std::uint64_t>(handle->core.ObservedCells().Size());
  });
}

int32_t lingtu_maps_rolling_occupancy_observed_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    float* out_x_m,
    float* out_y_m,
    float* out_z_m,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_count) {
  if (handle == nullptr) {
    return -1;
  }
  try {
    return CopyChunk(
        handle->core.ObservedCells(),
        out_x_m,
        out_y_m,
        out_z_m,
        out_state,
        out_log_odds_q8,
        capacity,
        out_count);
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_rolling_occupancy_rolled_out_count(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint64_t* out_count) {
  if (handle == nullptr || out_count == nullptr) {
    return -1;
  }
  return Protect([&]() {
    *out_count = static_cast<std::uint64_t>(handle->core.LastRolledOut().Size());
  });
}

int32_t lingtu_maps_rolling_occupancy_rolled_out_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    float* out_x_m,
    float* out_y_m,
    float* out_z_m,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_count) {
  if (handle == nullptr) {
    return -1;
  }
  try {
    return CopyChunk(
        handle->core.LastRolledOut(),
        out_x_m,
        out_y_m,
        out_z_m,
        out_state,
        out_log_odds_q8,
        capacity,
        out_count);
  } catch (...) {
    return -2;
  }
}

}  // extern "C"
