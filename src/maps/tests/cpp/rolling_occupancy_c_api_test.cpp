#include <cassert>
#include <cstdint>
#include <vector>

#include "lingtu/maps/c_api/rolling_occupancy.h"

namespace {

LingtuMapsRollingOccupancyConfig Config() {
  LingtuMapsRollingOccupancyConfig config{};
  config.size_x = 8;
  config.size_y = 8;
  config.size_z = 4;
  config.resolution_m = 1.0F;
  config.max_ray_range_m = 20.0F;
  config.hit_log_odds = 2.0F;
  config.miss_log_odds = 2.0F;
  config.min_log_odds = -4.0F;
  config.max_log_odds = 4.0F;
  config.occupied_probability = 0.65F;
  config.free_probability = 0.35F;
  config.roll_margin_x = 1;
  config.roll_margin_y = 1;
  config.roll_margin_z = 1;
  config.decay_factor = 0.9F;
  config.auto_roll = 1U;
  config.reject_out_of_order = 1U;
  return config;
}

void TestStableCAbi() {
  auto config = Config();
  LingtuMapsRollingOccupancyHandle* handle =
      lingtu_maps_rolling_occupancy_create(&config);
  assert(handle != nullptr);
  assert(lingtu_maps_rolling_occupancy_reset(
             handle, "map", 0.0F, 0.0F, 0.0F, 1) == 0);

  const float x[] = {3.2F};
  const float y[] = {0.1F};
  const float z[] = {0.1F};
  LingtuMapsRollingOccupancyStats stats{};
  assert(lingtu_maps_rolling_occupancy_update_xyz_soa(
             handle, x, y, z, 1U, "map", 10, 0.0F, 0.0F, 0.0F, &stats) == 0);
  assert(stats.hit_updates == 1U);

  std::uint8_t state = 0U;
  float probability = 0.0F;
  assert(lingtu_maps_rolling_occupancy_query(
             handle, 3.2F, 0.1F, 0.1F, &state, &probability) == 0);
  assert(state == LINGTU_MAPS_OCCUPANCY_OCCUPIED);
  assert(probability > 0.8F);

  LingtuMapsRollingOccupancySnapshotInfo info{};
  assert(lingtu_maps_rolling_occupancy_snapshot_info(handle, &info) == 0);
  assert(info.cell_count == 8U * 8U * 4U);
  std::vector<std::uint8_t> states(info.cell_count);
  std::vector<std::int16_t> evidence(info.cell_count);
  std::uint64_t count = 0U;
  assert(lingtu_maps_rolling_occupancy_snapshot_copy(
             handle,
             states.data(),
             evidence.data(),
             info.cell_count,
             &count) == 0);
  assert(count == info.cell_count);

  std::uint64_t observed = 0U;
  assert(lingtu_maps_rolling_occupancy_observed_count(handle, &observed) == 0);
  assert(observed > 0U);
  std::vector<float> ox(observed), oy(observed), oz(observed);
  std::vector<std::uint8_t> observed_state(observed);
  std::vector<std::int16_t> observed_evidence(observed);
  assert(lingtu_maps_rolling_occupancy_observed_copy(
             handle,
             ox.data(),
             oy.data(),
             oz.data(),
             observed_state.data(),
             observed_evidence.data(),
             observed,
             &count) == 0);
  assert(count == observed);

  lingtu_maps_rolling_occupancy_destroy(handle);
}

}  // namespace

int main() {
  TestStableCAbi();
  return 0;
}
