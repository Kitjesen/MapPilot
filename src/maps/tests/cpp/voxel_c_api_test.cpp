#include "lingtu/maps/c_api/voxel_layer.h"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::filesystem::path TempPath() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("lingtu_voxel_c_api_test_" + std::to_string(stamp) + ".ltbg");
}

}  // namespace

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__ << ": "  \
                << #condition << "\n";                                         \
      return 1;                                                                 \
    }                                                                           \
  } while (false)

int main() {
  CHECK(lingtu_maps_abi_version() == 1U);

  LingtuMapsVoxelConfig config{};
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.min_z_m = -10.0F;
  config.max_z_m = 10.0F;
  config.decay_rate = 0.0F;
  config.prune_below_count = 1.0F;
  config.column_carving = 1U;

  LingtuMapsVoxelHandle* handle = lingtu_maps_voxel_create(&config);
  CHECK(handle != nullptr);

  std::vector<float> first{
      0.1F, 0.1F, 0.1F,
      0.1F, 0.1F, 1.1F,
      2.1F, 0.1F, 0.1F,
  };
  CHECK(lingtu_maps_voxel_update_xyz_interleaved(
             handle, first.data(), 3U, "map", 1, 0.0F, 0.0F, 0.0F) == 0);
  CHECK(lingtu_maps_voxel_count(handle) == 3U);

  std::vector<float> second{0.2F, 0.2F, 0.2F};
  CHECK(lingtu_maps_voxel_update_xyz_interleaved(
             handle, second.data(), 1U, "map", 2, 0.0F, 0.0F, 0.0F) == 0);
  CHECK(lingtu_maps_voxel_count(handle) == 2U);
  CHECK(lingtu_maps_voxel_contains(handle, 0.2F, 0.2F, 0.2F) == 1);
  CHECK(lingtu_maps_voxel_contains(handle, 0.1F, 0.1F, 1.1F) == 0);
  CHECK(lingtu_maps_voxel_contains(handle, 2.1F, 0.1F, 0.1F) == 1);
  float count = 0.0F;
  CHECK(lingtu_maps_voxel_query_count(handle, 0.2F, 0.2F, 0.2F, &count) == 0);
  CHECK(count == 1.0F);

  LingtuMapsVoxelStats stats{};
  CHECK(lingtu_maps_voxel_stats(handle, &stats) == 0);
  CHECK(stats.input_points == 1U);
  CHECK(stats.carved_columns == 1U);
  CHECK(stats.carved_voxels == 2U);
  CHECK(stats.total_voxels == 2U);
  CHECK(stats.accumulated_occupied == 2U);
  CHECK(stats.ray_updates == 1U);
  CHECK(stats.hit_updates == 1U);

  uint64_t snapshot_count = 0;
  CHECK(lingtu_maps_voxel_snapshot_size(handle, &snapshot_count) == 0);
  CHECK(snapshot_count == 2U);
  std::vector<float> x(snapshot_count);
  std::vector<float> y(snapshot_count);
  std::vector<float> z(snapshot_count);
  CHECK(lingtu_maps_voxel_snapshot_xyz_soa(
             handle, x.data(), y.data(), z.data(), snapshot_count, &snapshot_count) == 0);
  CHECK(snapshot_count == 2U);

  LingtuMapsVoxelSceneStats scene_stats{};
  CHECK(lingtu_maps_voxel_scene_stats(handle, &scene_stats) == 0);
  CHECK(scene_stats.live_voxels == 2U);
  CHECK(scene_stats.accumulated_occupied == 2U);
  CHECK(scene_stats.map_ok == 1U);

  const auto state_path = TempPath();
  CHECK(lingtu_maps_voxel_save_binary(handle, state_path.string().c_str()) == 0);
  CHECK(lingtu_maps_voxel_validate_binary(state_path.string().c_str()) == 0);
  LingtuMapsVoxelHandle* loaded = lingtu_maps_voxel_create(&config);
  CHECK(loaded != nullptr);
  CHECK(lingtu_maps_voxel_load_binary(loaded, state_path.string().c_str()) == 0);
  CHECK(lingtu_maps_voxel_snapshot_size(loaded, &snapshot_count) == 0);
  CHECK(snapshot_count == 2U);
  lingtu_maps_voxel_destroy(loaded);
  std::filesystem::remove(state_path);

  CHECK(lingtu_maps_voxel_reset(handle) == 0);
  CHECK(lingtu_maps_voxel_count(handle) == 0U);

  lingtu_maps_voxel_destroy(handle);
  return 0;
}

#undef CHECK
