#include "lingtu/maps/c_api/voxel_layer.h"

#include "lingtu/maps/block_grid.hpp"
#include "lingtu/maps/layers/voxel.hpp"
#include "lingtu/maps/scene.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

using lingtu::maps::CloudLayout;
using lingtu::maps::FloatArrayView;
using lingtu::maps::MapCloudFrame;
using lingtu::maps::PointCloudView;
using lingtu::maps::BlockGridConfig;
using lingtu::maps::PersistentBlockGrid;
using lingtu::maps::SceneHealth;
using lingtu::maps::SceneSnapshotBuilder;
using lingtu::maps::layers::VoxelLayerConfig;
using lingtu::maps::layers::VoxelLayerCore;

struct LingtuMapsVoxelHandle {
  explicit LingtuMapsVoxelHandle(VoxelLayerConfig config)
      : core(config), grid(ToBlockGridConfig(config)), config(config) {}

  static BlockGridConfig ToBlockGridConfig(const VoxelLayerConfig& config) {
    BlockGridConfig out;
    out.cell_size_m = config.voxel_size_m;
    return out;
  }

  VoxelLayerCore core;
  PersistentBlockGrid grid;
  VoxelLayerConfig config;
  mutable std::mutex grid_mutex;
};

namespace {

constexpr uint32_t kAbiVersion = 1U;

VoxelLayerConfig DefaultConfig() {
  return {};
}

VoxelLayerConfig ToConfig(const LingtuMapsVoxelConfig* config) {
  if (config == nullptr) {
    return DefaultConfig();
  }
  VoxelLayerConfig out;
  out.voxel_size_m = config->voxel_size_m;
  out.max_range_m = config->max_range_m;
  out.min_z_m = config->min_z_m;
  out.max_z_m = config->max_z_m;
  out.decay_rate = config->decay_rate;
  out.prune_below_count = config->prune_below_count;
  out.column_carving = config->column_carving != 0U;
  return out;
}

struct ColumnKey {
  std::int32_t x{0};
  std::int32_t y{0};

  bool operator==(const ColumnKey& other) const {
    return x == other.x && y == other.y;
  }
};

struct ColumnKeyHash {
  std::size_t operator()(const ColumnKey& key) const {
    const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.x));
    const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.y));
    return static_cast<std::size_t>(x * 0x9e3779b97f4a7c15ULL ^ (y + 0x85ebca6b));
  }
};

bool IsFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

bool ReadPoint(
    const PointCloudView& cloud,
    std::size_t index,
    float* x_m,
    float* y_m,
    float* z_m) {
  if (x_m == nullptr || y_m == nullptr || z_m == nullptr) {
    return false;
  }
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved: {
      const std::size_t base = index * 3U;
      if (cloud.interleaved.data == nullptr || base + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x_m = cloud.interleaved.data[base];
      *y_m = cloud.interleaved.data[base + 1U];
      *z_m = cloud.interleaved.data[base + 2U];
      return true;
    }
    case CloudLayout::kXyziF32Interleaved: {
      const std::size_t base = index * 4U;
      if (cloud.interleaved.data == nullptr || base + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x_m = cloud.interleaved.data[base];
      *y_m = cloud.interleaved.data[base + 1U];
      *z_m = cloud.interleaved.data[base + 2U];
      return true;
    }
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA:
      if (cloud.x.data == nullptr || cloud.y.data == nullptr || cloud.z.data == nullptr ||
          index >= cloud.x.size || index >= cloud.y.size || index >= cloud.z.size) {
        return false;
      }
      *x_m = cloud.x.data[index];
      *y_m = cloud.y.data[index];
      *z_m = cloud.z.data[index];
      return true;
  }
  return false;
}

void FeedPersistentGrid(LingtuMapsVoxelHandle* handle, const MapCloudFrame& frame) {
  const auto& config = handle->config;
  const PointCloudView& cloud = frame.cloud;
  std::vector<float> hits;
  std::unordered_set<ColumnKey, ColumnKeyHash> columns;
  hits.reserve(cloud.point_count * 3U);
  columns.reserve(cloud.point_count);

  const float max_range_sq = config.max_range_m * config.max_range_m;
  const bool check_range = config.max_range_m > 0.0F;
  for (std::size_t i = 0; i < cloud.point_count; ++i) {
    float x_m = 0.0F;
    float y_m = 0.0F;
    float z_m = 0.0F;
    if (!ReadPoint(cloud, i, &x_m, &y_m, &z_m)) {
      continue;
    }
    if (!IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
      continue;
    }
    if (z_m < config.min_z_m || z_m > config.max_z_m) {
      continue;
    }
    if (check_range) {
      const float dx = x_m - frame.sensor_origin_x_m;
      const float dy = y_m - frame.sensor_origin_y_m;
      const float dz = z_m - frame.sensor_origin_z_m;
      const float dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq > max_range_sq) {
        continue;
      }
    }

    hits.push_back(x_m);
    hits.push_back(y_m);
    hits.push_back(z_m);
    columns.insert({
        static_cast<std::int32_t>(std::floor(x_m / config.voxel_size_m)),
        static_cast<std::int32_t>(std::floor(y_m / config.voxel_size_m)),
    });
  }

  std::lock_guard<std::mutex> lock(handle->grid_mutex);
  handle->grid.SetFrame(frame.cloud.frame_id.empty() ? "map" : frame.cloud.frame_id);
  handle->grid.SetStampNs(frame.cloud.stamp_ns);
  if (config.column_carving) {
    for (const auto& column : columns) {
      handle->grid.ClearColumn(
          (static_cast<float>(column.x) + 0.5F) * config.voxel_size_m,
          (static_cast<float>(column.y) + 0.5F) * config.voxel_size_m);
    }
  }
  if (hits.empty()) {
    return;
  }
  std::vector<float> origins(hits.size());
  for (std::size_t i = 0; i < hits.size() / 3U; ++i) {
    origins[i * 3U] = frame.sensor_origin_x_m;
    origins[i * 3U + 1U] = frame.sensor_origin_y_m;
    origins[i * 3U + 2U] = frame.sensor_origin_z_m;
  }
  static_cast<void>(
      handle->grid.InsertRays(origins.data(), hits.data(), hits.size() / 3U, config.max_range_m));
}

MapCloudFrame BuildFrame(
    PointCloudView cloud,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  cloud.frame_id = (frame_id == nullptr || frame_id[0] == '\0') ? "map" : frame_id;
  cloud.stamp_ns = stamp_ns;
  MapCloudFrame frame;
  frame.cloud = cloud;
  frame.sensor_origin_x_m = origin_x_m;
  frame.sensor_origin_y_m = origin_y_m;
  frame.sensor_origin_z_m = origin_z_m;
  return frame;
}

template <typename Fn>
int32_t Protect(Fn&& fn) {
  try {
    fn();
    return 0;
  } catch (const std::exception&) {
    return -2;
  } catch (...) {
    return -3;
  }
}

}  // namespace

extern "C" {

uint32_t lingtu_maps_abi_version(void) {
  return kAbiVersion;
}

LingtuMapsVoxelHandle* lingtu_maps_voxel_create(
    const LingtuMapsVoxelConfig* config) {
  try {
    return new LingtuMapsVoxelHandle(ToConfig(config));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_maps_voxel_destroy(LingtuMapsVoxelHandle* handle) {
  delete handle;
}

int32_t lingtu_maps_voxel_reset(LingtuMapsVoxelHandle* handle) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([handle]() {
    handle->core.Reset();
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    handle->grid.Reset();
  });
}

int32_t lingtu_maps_voxel_decay(LingtuMapsVoxelHandle* handle) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([handle]() {
    handle->core.Decay();
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    static_cast<void>(handle->grid.Decay(1.0F - handle->config.decay_rate));
  });
}

int32_t lingtu_maps_voxel_update_xyz_interleaved(
    LingtuMapsVoxelHandle* handle,
    const float* xyz,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  if (handle == nullptr || (xyz == nullptr && point_count > 0U)) {
    return -1;
  }
  return Protect([&]() {
    PointCloudView cloud;
    cloud.layout = CloudLayout::kXyzF32Interleaved;
    cloud.point_count = static_cast<std::size_t>(point_count);
    cloud.interleaved = {xyz, static_cast<std::size_t>(point_count * 3U)};
    const auto frame = BuildFrame(cloud, frame_id, stamp_ns, origin_x_m, origin_y_m, origin_z_m);
    handle->core.Update(frame);
    FeedPersistentGrid(handle, frame);
  });
}

int32_t lingtu_maps_voxel_update_xyzi_interleaved(
    LingtuMapsVoxelHandle* handle,
    const float* xyzi,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  if (handle == nullptr || (xyzi == nullptr && point_count > 0U)) {
    return -1;
  }
  return Protect([&]() {
    PointCloudView cloud;
    cloud.layout = CloudLayout::kXyziF32Interleaved;
    cloud.point_count = static_cast<std::size_t>(point_count);
    cloud.interleaved = {xyzi, static_cast<std::size_t>(point_count * 4U)};
    const auto frame = BuildFrame(cloud, frame_id, stamp_ns, origin_x_m, origin_y_m, origin_z_m);
    handle->core.Update(frame);
    FeedPersistentGrid(handle, frame);
  });
}

int32_t lingtu_maps_voxel_update_xyz_soa(
    LingtuMapsVoxelHandle* handle,
    const float* x,
    const float* y,
    const float* z,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m) {
  if (handle == nullptr ||
      ((x == nullptr || y == nullptr || z == nullptr) && point_count > 0U)) {
    return -1;
  }
  return Protect([&]() {
    PointCloudView cloud;
    cloud.layout = CloudLayout::kXyzF32SoA;
    cloud.point_count = static_cast<std::size_t>(point_count);
    cloud.x = {x, static_cast<std::size_t>(point_count)};
    cloud.y = {y, static_cast<std::size_t>(point_count)};
    cloud.z = {z, static_cast<std::size_t>(point_count)};
    const auto frame = BuildFrame(cloud, frame_id, stamp_ns, origin_x_m, origin_y_m, origin_z_m);
    handle->core.Update(frame);
    FeedPersistentGrid(handle, frame);
  });
}

uint64_t lingtu_maps_voxel_count(const LingtuMapsVoxelHandle* handle) {
  if (handle == nullptr) {
    return 0U;
  }
  return static_cast<uint64_t>(handle->core.VoxelCount());
}

int32_t lingtu_maps_voxel_contains(
    const LingtuMapsVoxelHandle* handle,
    float x_m,
    float y_m,
    float z_m) {
  if (handle == nullptr) {
    return -1;
  }
  try {
    return handle->core.Contains(x_m, y_m, z_m) ? 1 : 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_query_count(
    const LingtuMapsVoxelHandle* handle,
    float x_m,
    float y_m,
    float z_m,
    float* out_count) {
  if (handle == nullptr || out_count == nullptr) {
    return -1;
  }
  try {
    *out_count = handle->core.CountAt(x_m, y_m, z_m);
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_stats(
    const LingtuMapsVoxelHandle* handle,
    LingtuMapsVoxelStats* out_stats) {
  if (handle == nullptr || out_stats == nullptr) {
    return -1;
  }
  try {
    const auto stats = handle->core.LastStats();
    out_stats->input_points = static_cast<uint64_t>(stats.input_points);
    out_stats->accepted_points = static_cast<uint64_t>(stats.accepted_points);
    out_stats->input_voxels = static_cast<uint64_t>(stats.input_voxels);
    out_stats->input_columns = static_cast<uint64_t>(stats.input_columns);
    out_stats->carved_columns = static_cast<uint64_t>(stats.carved_columns);
    out_stats->carved_voxels = static_cast<uint64_t>(stats.carved_voxels);
    out_stats->total_voxels = static_cast<uint64_t>(stats.total_voxels);
    {
      std::lock_guard<std::mutex> lock(handle->grid_mutex);
      const auto grid_stats = handle->grid.LastStats();
      const auto snapshot = handle->grid.Snapshot();
      std::uint64_t occupied = 0U;
      for (const auto probability : snapshot.occupancy_probability) {
        if (probability >= 0.60F) {
          ++occupied;
        }
      }
      out_stats->accumulated_cells = static_cast<uint64_t>(snapshot.Size());
      out_stats->accumulated_occupied = occupied;
      out_stats->accumulated_generation = handle->grid.Generation();
      out_stats->ray_updates = static_cast<uint64_t>(grid_stats.rays);
      out_stats->free_updates = static_cast<uint64_t>(grid_stats.free_updates);
      out_stats->hit_updates = static_cast<uint64_t>(grid_stats.hit_updates);
      out_stats->pruned_cells = static_cast<uint64_t>(grid_stats.pruned_cells);
    }
    out_stats->column_carving = stats.column_carving ? 1U : 0U;
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_snapshot_size(
    const LingtuMapsVoxelHandle* handle,
    uint64_t* out_point_count) {
  if (handle == nullptr || out_point_count == nullptr) {
    return -1;
  }
  try {
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    const auto snapshot = handle->grid.Snapshot();
    std::uint64_t occupied = 0U;
    for (const auto probability : snapshot.occupancy_probability) {
      if (probability >= 0.60F) {
        ++occupied;
      }
    }
    *out_point_count = occupied;
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_snapshot_xyz_soa(
    const LingtuMapsVoxelHandle* handle,
    float* out_x,
    float* out_y,
    float* out_z,
    uint64_t capacity,
    uint64_t* out_point_count) {
  if (handle == nullptr || out_point_count == nullptr ||
      ((out_x == nullptr || out_y == nullptr || out_z == nullptr) && capacity > 0U)) {
    return -1;
  }
  try {
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    const auto snapshot = handle->grid.Snapshot();
    std::uint64_t occupied = 0U;
    for (const auto probability : snapshot.occupancy_probability) {
      if (probability >= 0.60F) {
        ++occupied;
      }
    }
    *out_point_count = occupied;
    if (capacity < occupied) {
      return 1;
    }
    std::uint64_t written = 0U;
    for (std::size_t i = 0; i < snapshot.Size(); ++i) {
      if (snapshot.occupancy_probability[i] < 0.60F) {
        continue;
      }
      out_x[written] = snapshot.center_x_m[i];
      out_y[written] = snapshot.center_y_m[i];
      out_z[written] = snapshot.center_z_m[i];
      ++written;
    }
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_snapshot_occupied_xyz_soa(
    const LingtuMapsVoxelHandle* handle,
    float* out_x,
    float* out_y,
    float* out_z,
    uint64_t capacity,
    uint64_t* out_point_count) {
  return lingtu_maps_voxel_snapshot_xyz_soa(
      handle, out_x, out_y, out_z, capacity, out_point_count);
}

int32_t lingtu_maps_voxel_scene_stats(
    const LingtuMapsVoxelHandle* handle,
    LingtuMapsVoxelSceneStats* out_stats) {
  if (handle == nullptr || out_stats == nullptr) {
    return -1;
  }
  try {
    const auto live = handle->core.SnapshotCloud();
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    SceneSnapshotBuilder builder;
    builder.SetFrame(live.frame_id, live.stamp_ns);
    builder.SetHealth(SceneHealth{true, true, true, 0U, ""});
    builder.SetLiveCloud(live.View());
    builder.SetAccumulatedCloud(handle->grid.Snapshot());
    const auto scene = builder.Build();
    std::uint64_t occupied = 0U;
    for (const auto probability : scene.accumulated_cloud.occupancy_probability) {
      if (probability >= 0.60F) {
        ++occupied;
      }
    }
    out_stats->live_voxels = static_cast<uint64_t>(handle->core.VoxelCount());
    out_stats->accumulated_cells = static_cast<uint64_t>(scene.accumulated_cloud.Size());
    out_stats->accumulated_occupied = occupied;
    out_stats->accumulated_generation = scene.accumulated_cloud.generation;
    out_stats->live_points = static_cast<uint64_t>(scene.live_cloud.point_count);
    out_stats->frame_stamp_ns = static_cast<uint64_t>(scene.stamp_ns);
    out_stats->voxel_size_m = handle->config.voxel_size_m;
    out_stats->localization_ok = scene.health.localization_ok ? 1U : 0U;
    out_stats->map_ok = scene.health.map_ok ? 1U : 0U;
    out_stats->planner_ok = scene.health.planner_ok ? 1U : 0U;
    out_stats->status_bits = scene.health.status_bits;
    return 0;
  } catch (...) {
    return -2;
  }
}

int32_t lingtu_maps_voxel_save_binary(
    const LingtuMapsVoxelHandle* handle,
    const char* path) {
  if (handle == nullptr || path == nullptr || path[0] == '\0') {
    return -1;
  }
  return Protect([&]() {
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    handle->grid.SaveBinary(std::filesystem::path(path));
  });
}

int32_t lingtu_maps_voxel_load_binary(
    LingtuMapsVoxelHandle* handle,
    const char* path) {
  if (handle == nullptr || path == nullptr || path[0] == '\0') {
    return -1;
  }
  return Protect([&]() {
    auto loaded = PersistentBlockGrid::LoadBinary(
        std::filesystem::path(path), LingtuMapsVoxelHandle::ToBlockGridConfig(handle->config));
    std::lock_guard<std::mutex> lock(handle->grid_mutex);
    handle->grid = std::move(loaded);
  });
}

int32_t lingtu_maps_voxel_validate_binary(const char* path) {
  if (path == nullptr || path[0] == '\0') {
    return -1;
  }
  try {
    return PersistentBlockGrid::ValidateBinary(std::filesystem::path(path)) ? 0 : 1;
  } catch (...) {
    return -2;
  }
}

}  // extern "C"
