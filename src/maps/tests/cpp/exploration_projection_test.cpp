#include "lingtu/maps/layers/exploration_projection.hpp"

#include <cassert>
#include <iostream>

using lingtu::maps::layers::ExplorationProjectionConfig;
using lingtu::maps::layers::OccupancyState;
using lingtu::maps::layers::ProjectExplorationGrid;
using lingtu::maps::layers::RollingOccupancySnapshot;

namespace {

RollingOccupancySnapshot MakeSnapshot() {
  RollingOccupancySnapshot snapshot;
  snapshot.frame_id = "map";
  snapshot.stamp_ns = 10;
  snapshot.generation = 4U;
  snapshot.resolution_m = 1.0F;
  snapshot.size_x = 3;
  snapshot.size_y = 3;
  snapshot.size_z = 3;
  snapshot.origin_x_m = -1.5F;
  snapshot.origin_y_m = -1.5F;
  snapshot.origin_z_m = -1.5F;
  snapshot.state.assign(
      static_cast<std::size_t>(
          snapshot.size_x * snapshot.size_y * snapshot.size_z),
      static_cast<std::uint8_t>(OccupancyState::kUnknown));
  snapshot.log_odds_q8.assign(snapshot.state.size(), 0);
  for (int y = 0; y < snapshot.size_y; ++y) {
    for (int x = 0; x < snapshot.size_x; ++x) {
      snapshot.state[snapshot.Index(x, y, 1)] =
          static_cast<std::uint8_t>(OccupancyState::kFree);
    }
  }
  snapshot.state[snapshot.Index(1, 1, 1)] =
      static_cast<std::uint8_t>(OccupancyState::kOccupied);
  return snapshot;
}

void TestProjectsTrinaryColumns() {
  ExplorationProjectionConfig config;
  config.robot_z_m = 0.0F;
  config.obstacle_min_z_m = -0.1F;
  config.obstacle_max_z_m = 0.6F;
  config.inflation_radius_m = 0.0F;

  const auto result = ProjectExplorationGrid(
      MakeSnapshot(),
      config);
  assert(result.frame_id == "map");
  assert(result.generation == 4U);
  assert(result.grid.rows == 3);
  assert(result.grid.cols == 3);
  assert(result.occupied_cells == 1U);
  assert(result.free_cells == 8U);
  assert(result.unknown_cells == 0U);
  assert(result.grid.data[4] == 100.0F);
}

void TestInflationIsConservativeAndExplicit() {
  ExplorationProjectionConfig config;
  config.robot_z_m = 0.0F;
  config.obstacle_min_z_m = -0.1F;
  config.obstacle_max_z_m = 0.6F;
  config.inflation_radius_m = 1.01F;

  const auto result = ProjectExplorationGrid(
      MakeSnapshot(),
      config);
  assert(result.occupied_cells == 5U);
  assert(result.free_cells == 4U);
  assert(result.unknown_cells == 0U);
}

void TestUnobservedHeightBandRemainsUnknown() {
  ExplorationProjectionConfig config;
  config.robot_z_m = 10.0F;
  config.obstacle_min_z_m = 0.0F;
  config.obstacle_max_z_m = 1.0F;
  config.inflation_radius_m = 0.0F;

  const auto result = ProjectExplorationGrid(
      MakeSnapshot(),
      config);
  assert(result.occupied_cells == 0U);
  assert(result.free_cells == 0U);
  assert(result.unknown_cells == 9U);
}

}  // namespace

int main() {
  TestProjectsTrinaryColumns();
  TestInflationIsConservativeAndExplicit();
  TestUnobservedHeightBandRemainsUnknown();
  std::cout << "exploration_projection_test passed\n";
  return 0;
}
