#include <cassert>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "lingtu/maps/layers/rolling_occupancy.hpp"

namespace {

using lingtu::maps::CloudLayout;
using lingtu::maps::MapCloudFrame;
using lingtu::maps::PointCloudView;
using lingtu::maps::layers::OccupancyState;
using lingtu::maps::layers::RollingOccupancyConfig;
using lingtu::maps::layers::RollingOccupancyGrid;

RollingOccupancyConfig TestConfig() {
  RollingOccupancyConfig config;
  config.size_x = 8;
  config.size_y = 8;
  config.size_z = 4;
  config.resolution_m = 1.0F;
  config.max_ray_range_m = 20.0F;
  config.hit_log_odds = 4.0F;
  config.miss_log_odds = 2.0F;
  config.inflation_radius_m = 0.01F;
  config.inflation_z_up_m = 0.0F;
  config.inflation_z_down_m = 0.0F;
  config.roll_margin_x = 1;
  config.roll_margin_y = 1;
  config.roll_margin_z = 1;
  config.auto_roll = true;
  return config;
}

MapCloudFrame Frame(
    const std::vector<float>& xyz,
    std::int64_t stamp_ns,
    float origin_x = 0.0F,
    float origin_y = 0.0F,
    float origin_z = 0.0F) {
  PointCloudView cloud;
  cloud.frame_id = "map";
  cloud.stamp_ns = stamp_ns;
  cloud.layout = CloudLayout::kXyzF32Interleaved;
  cloud.point_count = xyz.size() / 3U;
  cloud.interleaved = {xyz.data(), xyz.size()};
  MapCloudFrame frame;
  frame.cloud = cloud;
  frame.sensor_origin_x_m = origin_x;
  frame.sensor_origin_y_m = origin_y;
  frame.sensor_origin_z_m = origin_z;
  return frame;
}

void TestRayProducesFreeAndOccupiedEvidence() {
  RollingOccupancyGrid grid(TestConfig());
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::vector<float> points{3.2F, 0.1F, 0.1F};
  const auto stats = grid.Update(Frame(points, 10));
  assert(stats.input_points == 1U);
  assert(stats.accepted_points == 1U);
  assert(stats.hit_updates == 1U);
  assert(stats.free_updates == 2U);
  assert(grid.StateAt(3.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.StateAt(0.1F, 0.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.OccupancyProbability(3.2F, 0.1F, 0.1F) > 0.8F);
  const auto occupied = grid.InflatedSnapshot();
  assert(occupied.occupied_cells == 1U);
  assert(occupied.Occupied(7, 4, 2));
}

void TestDenseFrameDoesNotOverweightDuplicateVoxels() {
  RollingOccupancyGrid grid(TestConfig());
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  std::vector<float> points;
  for (int index = 0; index < 100; ++index) {
    points.insert(points.end(), {2.2F, 0.1F, 0.1F});
  }
  const auto stats = grid.Update(Frame(points, 10));
  assert(stats.input_points == 100U);
  assert(stats.hit_updates == 1U);
  const auto observed = grid.ObservedCells();
  bool found_hit = false;
  for (std::size_t index = 0U; index < observed.Size(); ++index) {
    if (std::fabs(observed.center_x_m[index] - 2.5F) < 0.01F) {
      assert(observed.hit_count[index] == 1U);
      found_hit = true;
    }
  }
  assert(found_hit);
}

void TestOfficialUnknownPriorRequiresRepeatedHits() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds = std::log(0.85F / 0.15F);
  config.miss_log_odds = -std::log(0.30F / 0.70F);
  config.min_log_odds = std::log(0.12F / 0.88F);
  config.max_log_odds = std::log(0.98F / 0.02F);
  config.occupied_probability = 0.80F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  const std::vector<float> point{2.2F, 0.1F, 0.1F};
  grid.Update(Frame(point, 10));
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) != OccupancyState::kOccupied);
  assert(!grid.Contains(2.2F, 0.1F, 0.1F));
  assert(grid.OccupancyProbability(2.2F, 0.1F, 0.1F) < 0.8);
  assert(grid.InflatedContains(2.2F, 0.1F, 0.1F));
  const auto first_generation = grid.Generation();

  grid.Update(Frame(point, 20));
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.InflatedContains(2.2F, 0.1F, 0.1F));
  assert(grid.Generation() == first_generation);
  grid.Update(Frame({}, 30));
  assert(grid.Contains(2.2F, 0.1F, 0.1F));
  assert(grid.InflatedContains(2.2F, 0.1F, 0.1F));
  assert(grid.Generation() == first_generation);
}

void TestRayFusionMatchesOfficialEndpointAndTraversalVotes() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({
      2.2F, 0.1F, 0.1F,
      3.2F, 0.1F, 0.1F,
      3.2F, 0.1F, 0.1F,
      3.2F, 0.1F, 0.1F,
  }, 10));

  assert(grid.StateAt(1.2F, 0.1F, 0.1F) == OccupancyState::kFree);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kFree);
  assert(grid.StateAt(3.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  // A real endpoint still blocks live collision when traversal votes keep its
  // historical probability free. Missing observations do not clear that hit.
  assert(grid.InflatedContains(2.2F, 0.1F, 0.1F));
  grid.Update(Frame({}, 20));
  assert(grid.InflatedContains(2.2F, 0.1F, 0.1F));
  grid.Update(Frame({3.2F, 0.1F, 0.1F}, 30));
  assert(!grid.InflatedContains(2.2F, 0.1F, 0.1F));
  assert(grid.InflatedContains(3.2F, 0.1F, 0.1F));
}

void TestRayCornerContactLeavesSideCellsUnknown() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({2.2F, 2.2F, 0.1F}, 10, 0.1F, 0.1F, 0.1F));

  assert(grid.StateAt(0.1F, 1.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.StateAt(1.1F, 0.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.StateAt(1.1F, 1.1F, 0.1F) == OccupancyState::kFree);
}

void TestObliqueRayClearsOnlyMeasuredSegment() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  // The new ray crosses (1, 1) and (1, 0), but never (2, 0). Using cell
  // deltas instead of endpoint positions reverses those first/last facts.
  grid.Update(Frame({1.5F, 1.5F, 0.1F, 2.5F, 0.5F, 0.1F}, 10,
                    0.1F, 0.9F, 0.1F));
  assert(grid.InflatedContains(1.5F, 1.5F, 0.1F));
  assert(grid.InflatedContains(2.5F, 0.5F, 0.1F));
  for (int scan = 0; scan < 3; ++scan) {
    grid.Update(Frame({2.9F, 1.1F, 0.1F}, 20 + scan,
                      0.1F, 0.9F, 0.1F));
  }

  assert(grid.StateAt(1.5F, 1.5F, 0.1F) == OccupancyState::kFree);
  assert(!grid.InflatedContains(1.5F, 1.5F, 0.1F));
  assert(grid.StateAt(1.5F, 0.5F, 0.1F) == OccupancyState::kFree);
  assert(grid.StateAt(2.5F, 0.5F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.InflatedContains(2.5F, 0.5F, 0.1F));
}

void TestNewObstacleInKnownFreeSpaceRequiresObservedClearing() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds = std::log(0.85 / 0.15);
  config.miss_log_odds = -std::log(0.30 / 0.70);
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  for (int scan = 0; scan < 3; ++scan) {
    grid.Update(Frame({3.2F, 0.1F, 0.1F}, 10 + scan));
  }
  assert(!grid.InflatedContains(2.2, 0.1, 0.1));

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 20));
  assert(!grid.Contains(2.2, 0.1, 0.1));
  assert(grid.InflatedContains(2.2, 0.1, 0.1));
  for (int scan = 0; scan < 3; ++scan) {
    grid.Update(Frame({0.1F, 2.2F, 0.1F}, 30 + scan));
    assert(grid.InflatedContains(2.2, 0.1, 0.1));
  }
  grid.Update(Frame({}, 40));
  assert(grid.InflatedContains(2.2, 0.1, 0.1));

  // A measured ray through the old endpoint resolves the temporary hit.
  grid.Update(Frame({3.2F, 0.1F, 0.1F}, 50));
  assert(!grid.InflatedContains(2.2, 0.1, 0.1));
}

void TestEndpointMajorityPreservesHistoryUntilObservedFree() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  // Three distinct returns hit one voxel; one farther ray crosses that voxel.
  // Each measured endpoint casts a hit, not an additional miss of its own.
  grid.Update(Frame({2.2F, 0.1F, 0.1F, 2.3F, 0.2F, 0.1F,
                     2.4F, 0.3F, 0.1F, 3.5F, 0.8F, 0.1F}, 10));
  assert(grid.Contains(2.5, 0.5, 0.1));
  assert(grid.InflatedContains(2.5, 0.5, 0.1));

  // An unrelated scan cannot erase the endpoint or its historical occupancy.
  grid.Update(Frame({0.1F, 2.2F, 0.1F}, 20));
  assert(grid.Contains(2.5, 0.5, 0.1));
  assert(grid.InflatedContains(2.5, 0.5, 0.1));

  grid.Update(Frame({3.5F, 0.8F, 0.1F}, 30));
  assert(!grid.Contains(2.5, 0.5, 0.1));
  assert(!grid.InflatedContains(2.5, 0.5, 0.1));
  assert(grid.InflatedContains(3.5, 0.8, 0.1));
}

void TestClippedEndpointContributesOneFreeVote() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.max_ray_range_m = 1.5;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  // The clipped far return and the real hit end in the same voxel. One hit
  // and one miss retain the declared hit-wins-ties probability update.
  grid.Update(Frame({1.2F, 0.1F, 0.1F, 3.2F, 0.4F, 0.1F}, 10));
  assert(grid.Contains(1.5, 0.5, 0.1));
  assert(grid.InflatedContains(1.5, 0.5, 0.1));
  grid.Update(Frame({3.2F, 0.4F, 0.1F}, 20));
  assert(!grid.InflatedContains(1.5, 0.5, 0.1));
}

void TestSharedEndpointRaysRetainDistinctFreeSegments() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid forward(config);
  RollingOccupancyGrid reverse(config);
  const std::vector<float> points{2.9F, 1.1F, 0.1F, 2.1F, 1.9F, 0.1F};
  const std::vector<float> reversed{2.1F, 1.9F, 0.1F, 2.9F, 1.1F, 0.1F};
  const auto stats = forward.Update(Frame(points, 10, 0.1F, 0.1F, 0.1F));
  reverse.Update(Frame(reversed, 10, 0.1F, 0.1F, 0.1F));
  assert(stats.unique_rays == 1U);
  assert(forward.StateAt(1.5, 1.5, 0.1) == OccupancyState::kFree);
  assert(forward.StateAt(2.5, 0.5, 0.1) == OccupancyState::kFree);
  assert(forward.InflatedContains(2.5, 1.5, 0.1));
  assert(forward.Snapshot().state == reverse.Snapshot().state);
  assert(forward.Snapshot().log_odds_q8 == reverse.Snapshot().log_odds_q8);
}

void TestIntersectingRaysClearOnlyObservedHistory() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid forward(config);
  RollingOccupancyGrid reverse(config);
  const std::vector<float> points{3.9F, 1.1F, 0.1F, 2.9F, 1.9F, 0.1F};
  const std::vector<float> reversed{2.9F, 1.9F, 0.1F, 3.9F, 1.1F, 0.1F};
  // The rays share (2, 1) but only the second crosses (1, 1).
  // The control cell (1, -1) is never observed by either clearing segment.
  const std::vector<float> seed{1.5F, 1.5F, 0.1F, 1.5F, -0.5F, 0.1F};
  for (auto* grid : {&forward, &reverse}) {
    for (int scan = 0; scan < 3; ++scan)
      grid->Update(Frame(seed, 10 + scan, 0.1F, 0.9F, 0.1F));
    assert(grid->Contains(1.5, 1.5, 0.1));
    assert(grid->Contains(1.5, -0.5, 0.1));
  }
  for (int scan = 0; scan < 4; ++scan) {
    forward.Update(Frame(points, 20 + scan, 0.1F, 0.9F, 0.1F));
    reverse.Update(Frame(reversed, 20 + scan, 0.1F, 0.9F, 0.1F));
  }
  assert(!forward.InflatedContains(1.5, 1.5, 0.1));
  assert(forward.InflatedContains(1.5, -0.5, 0.1));
  assert(forward.InflatedContains(2.9, 1.9, 0.1));
  assert(forward.InflatedContains(3.9, 1.1, 0.1));
  assert(forward.Snapshot().state == reverse.Snapshot().state);
  assert(forward.Snapshot().log_odds_q8 == reverse.Snapshot().log_odds_q8);
}

void TestSubthresholdObservationIsKnownFreeLikeOfficialGridMap() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds =
      std::log(config.occupied_probability / (1.0F - config.occupied_probability));
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 10));

  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kFree);
  assert(grid.InflatedSnapshot().occupied_cells == 1U);
}

void TestRayStopsAtSensorBoundary() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  grid.Update(Frame({0.5F, 1.5F, 0.1F, 1.5F, 0.5F, 0.1F}, 10,
                    1.0F, 1.0F, 0.1F));
  assert(grid.InflatedContains(0.5F, 1.5F, 0.1F));
  assert(grid.InflatedContains(1.5F, 0.5F, 0.1F));
  for (int scan = 0; scan < 3; ++scan)
    grid.Update(Frame({0.25F, 2.75F, 0.1F}, 20 + scan,
                      1.0F, 1.0F, 0.1F));
  assert(!grid.InflatedContains(0.5F, 1.5F, 0.1F));
  assert(grid.InflatedContains(1.5F, 0.5F, 0.1F));
  assert(grid.StateAt(-0.5F, 0.5F, 0.1F) == OccupancyState::kUnknown);
}

void TestRayCornerContactDoesNotClearNeighbor() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  grid.Update(Frame({1.5F, 0.5F, 0.1F, 0.5F, 1.5F, 0.1F,
                    1.5F, 1.5F, 0.1F}, 10, 2.75F, 2.75F, 0.1F));
  for (int scan = 0; scan < 3; ++scan)
    grid.Update(Frame({0.25F, 0.25F, 0.1F}, 20 + scan,
                      2.75F, 2.75F, 0.1F));
  assert(!grid.InflatedContains(1.5F, 1.5F, 0.1F));
  assert(grid.InflatedContains(1.5F, 0.5F, 0.1F));
  assert(grid.InflatedContains(0.5F, 1.5F, 0.1F));
}

void TestUnresolvedHitsRequireClearingWithoutChangingHistoricalOccupancy() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds = 0.01;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 10));
  const auto first = grid.Generation();
  assert(grid.InflatedContains(2.2, 0.1, 0.1));
  assert(!grid.Contains(2.2, 0.1, 0.1));
  const auto stored = grid.ObservedCells();
  for (auto state : stored.state) {
    assert(state != static_cast<std::uint8_t>(OccupancyState::kOccupied));
  }
  grid.Update(Frame({2.2F, 0.1F, 0.1F, 2.2F, 0.1F, 0.1F}, 20));
  assert(grid.Generation() == first);
  grid.Update(Frame({0.1F, 2.2F, 0.1F}, 30));
  assert(grid.InflatedContains(2.2, 0.1, 0.1));
  assert(grid.InflatedContains(0.1, 2.2, 0.1));
  assert(grid.Generation() == first + 1U);
  grid.Update(Frame({}, 40));
  assert(grid.InflatedSnapshot().occupied_cells == 2U);
  assert(grid.Generation() == first + 1U);
  grid.Update(Frame({3.2F, 0.1F, 0.1F, 0.1F, 3.2F, 0.1F}, 50));
  assert(!grid.InflatedContains(2.2, 0.1, 0.1));
  assert(!grid.InflatedContains(0.1, 2.2, 0.1));
}

void TestCurrentHitRollResetAndSharedInflation() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds = 0.01;
  config.inflation_radius_m = 2.0;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  grid.Update(Frame({-1.2F, 0.1F, 0.1F, -0.2F, 0.1F, 0.1F}, 10));
  assert(grid.InflatedContains(-0.2, 0.1, 0.1));
  const auto rolled = grid.RollToCenter(3.2, 0.0, 0.0, 20);
  assert(!rolled.Empty());
  for (auto state : rolled.state) {
    assert(state != static_cast<std::uint8_t>(OccupancyState::kOccupied));
  }
  assert(grid.InflatedContains(-0.2, 0.1, 0.1));
  assert(!grid.InflatedContains(6.2, 0.1, 0.1));
  grid.Update(Frame({}, 30, 3.2F));
  assert(grid.InflatedContains(-0.2, 0.1, 0.1));
  // A return outside the rolled window clears the remaining old hit by ray.
  grid.Update(Frame({-2.2F, 0.1F, 0.1F}, 35, 1.2F));
  assert(grid.InflatedSnapshot().occupied_cells == 0U);
  grid.Update(Frame({5.2F, 0.1F, 0.1F}, 40, 3.2F));
  assert(grid.InflatedContains(5.2, 0.1, 0.1));
  grid.RollToCenter(20.0, 0.0, 0.0, 50);
  assert(grid.InflatedSnapshot().occupied_cells == 0U);
  grid.Update(Frame({21.2F, 0.1F, 0.1F}, 60, 20.0F));
  assert(grid.InflatedContains(21.2, 0.1, 0.1));
  grid.Reset("map", 0.0, 0.0, 0.0, 70);
  grid.Update(Frame({}, 80));
  assert(grid.InflatedSnapshot().occupied_cells == 0U);
}

void TestRangeClippingDoesNotCreateCurrentCollisionHit() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.max_ray_range_m = 1.5;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);
  const auto stats = grid.Update(Frame({3.2F, 0.1F, 0.1F}, 10));
  assert(stats.accepted_points == 1U);
  assert(stats.hit_updates == 0U);
  assert(grid.InflatedSnapshot().occupied_cells == 0U);
}

void TestInflationExcludesExactRadiusBoundary() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.inflation_radius_m = config.resolution_m;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({0.1F, 0.1F, 0.1F}, 10));

  assert(grid.InflatedContains(0.1F, 0.1F, 0.1F));
  assert(!grid.InflatedContains(1.1F, 0.1F, 0.1F));
}

void TestCollisionGenerationAdvancesOnlyWhenInflatedGeometryChanges() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::uint64_t reset_generation = grid.Generation();

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 10));
  const std::uint64_t occupied_generation = grid.Generation();
  assert(occupied_generation == reset_generation + 1U);
  assert(grid.InflatedSnapshot().generation == occupied_generation);

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 20));
  assert(grid.Generation() == occupied_generation);
  for (std::int64_t stamp = 21; stamp < 30; ++stamp) {
    grid.Update(Frame({2.2F, 0.1F, 0.1F}, stamp));
    assert(grid.Generation() == occupied_generation);
  }

  grid.Update(Frame({3.2F, 0.1F, 0.1F}, 30));
  assert(grid.Generation() == occupied_generation + 1U);
}

void TestWindowRollEmitsOutgoingAndPreservesOverlap() {
  RollingOccupancyGrid grid(TestConfig());
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::vector<float> points{
      -3.2F, 0.1F, 0.1F,
      2.2F, 0.1F, 0.1F,
  };
  grid.Update(Frame(points, 10));
  assert(grid.StateAt(-3.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);

  const auto rolled = grid.RollToCenter(3.2F, 0.0F, 0.0F, 20);
  assert(!rolled.Empty());
  bool outgoing_hit = false;
  for (std::size_t index = 0U; index < rolled.Size(); ++index) {
    if (rolled.state[index] == static_cast<std::uint8_t>(OccupancyState::kOccupied) &&
        rolled.center_x_m[index] < -3.0F) {
      outgoing_hit = true;
    }
  }
  assert(outgoing_hit);
  assert(grid.StateAt(-3.2F, 0.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  const auto occupied = grid.InflatedSnapshot();
  assert(occupied.occupied_cells == 1U);

  const auto snapshot = grid.Snapshot();
  assert(std::fabs(snapshot.origin_x_m - -1.0F) < 0.01F);
  assert(snapshot.CellCount() == 8U * 8U * 4U);
}

void TestSlidingThresholdIsSymmetricLikeUpstream() {
  auto positive_config = TestConfig();
  RollingOccupancyGrid positive(positive_config);
  positive.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const float positive_origin = positive.Snapshot().origin_x_m;
  positive.RollToCenter(3.2F, 0.0F, 0.0F, 2);
  assert(positive.Snapshot().origin_x_m != positive_origin);

  auto negative_config = TestConfig();
  RollingOccupancyGrid negative(negative_config);
  negative.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const float negative_origin = negative.Snapshot().origin_x_m;
  negative.RollToCenter(-2.8F, 0.0F, 0.0F, 2);
  assert(negative.Snapshot().origin_x_m != negative_origin);
}

void TestShortPointOutsideUpstreamLocalRangeIsIgnored() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  const auto stats = grid.Update(
      Frame({2.2F, 0.1F, 0.1F}, 10, -2.8F, 0.1F, 0.1F));

  assert(stats.accepted_points == 0U);
  assert(stats.rejected_points == 1U);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kUnknown);
}

void TestWindowRollClearsInflationFromEveryReusedCell() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.inflation_radius_m = 2.0F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  grid.Update(Frame({-0.8F, 0.1F, 0.1F}, 10));

  grid.RollToCenter(3.2F, 0.0F, 0.0F, 20);

  assert(grid.InflatedContains(-0.8F, 0.1F, 0.1F));
  assert(!grid.InflatedContains(6.2F, 0.1F, 0.1F));
}

void TestAutoRollAndCellMutationCommitOneGeneration() {
  RollingOccupancyGrid grid(TestConfig());
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::uint64_t before = grid.Generation();
  const std::vector<float> points{5.2F, 0.1F, 0.1F};

  const auto stats = grid.Update(Frame(points, 10, 3.2F, 0.1F, 0.1F));

  assert(stats.rolled);
  assert(stats.free_updates > 0U || stats.hit_updates > 0U);
  assert(stats.generation == before + 1U);
  assert(grid.Generation() == before + 1U);
}

void TestDecayAndOutOfOrderGate() {
  auto config = TestConfig();
  config.decay_after_ns = 100;
  config.decay_factor = 0.0F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::vector<float> points{2.2F, 0.1F, 0.1F};
  grid.Update(Frame(points, 10));
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.Decay(200) > 0U);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.InflatedSnapshot().occupied_cells == 0U);

  // Decay uses the runtime clock and must not advance the sensor observation
  // cursor. A newer observation remains valid even when its stamp is below the
  // latest decay tick.
  grid.Update(Frame(points, 100));
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);

  bool rejected = false;
  try {
    grid.Update(Frame(points, 99));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

void TestFrameMismatchFailsClosed() {
  RollingOccupancyGrid grid(TestConfig());
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);
  const std::vector<float> points{1.2F, 0.1F, 0.1F};
  auto frame = Frame(points, 10);
  frame.cloud.frame_id = "odom";
  bool rejected = false;
  try {
    grid.Update(frame);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

void TestIncrementalInflationKeepsSharedCoverageUntilLastSourceClears() {
  auto config = TestConfig();
  config.inflation_radius_m = 2.0F;
  config.decay_after_ns = 50;
  config.decay_factor = 0.0F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 10));
  grid.Update(Frame({0.1F, 2.2F, 0.1F}, 40));
  assert(grid.InflatedContains(1.1F, 1.1F, 0.1F));

  assert(grid.Decay(70) > 0U);
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kUnknown);
  assert(grid.StateAt(0.1F, 2.2F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.InflatedContains(1.1F, 1.1F, 0.1F));

  assert(grid.Decay(100) > 0U);
  assert(!grid.InflatedContains(1.1F, 1.1F, 0.1F));
}

void AssertInflationMatchesFullRebuild(
    const RollingOccupancyGrid& grid, const std::vector<float>& current_hits = {}) {
  const auto config = grid.Config();
  const auto source = grid.Snapshot();
  const auto inflated = grid.InflatedSnapshot();
  std::vector<std::uint8_t> expected(source.CellCount(), 0U);
  std::vector<std::uint8_t> live_hits(source.CellCount(), 0U);
  for (std::size_t i = 0; i < current_hits.size(); i += 3U) {
    const int x = static_cast<int>(std::floor(
        (current_hits[i] - source.origin_x_m) / source.resolution_m));
    const int y = static_cast<int>(std::floor(
        (current_hits[i + 1U] - source.origin_y_m) / source.resolution_m));
    const int z = static_cast<int>(std::floor(
        (current_hits[i + 2U] - source.origin_z_m) / source.resolution_m));
    if (x >= 0 && x < source.size_x && y >= 0 && y < source.size_y &&
        z >= 0 && z < source.size_z) {
      live_hits[source.Index(x, y, z)] = 1U;
    }
  }
  const int xy_cells = static_cast<int>(
      std::ceil(config.inflation_radius_m / config.resolution_m));
  const int down_cells = static_cast<int>(
      std::ceil(config.inflation_z_down_m / config.resolution_m));
  const int up_cells = static_cast<int>(
      std::ceil(config.inflation_z_up_m / config.resolution_m));
  const float radius_squared =
      config.inflation_radius_m * config.inflation_radius_m;
  for (int z = 0; z < source.size_z; ++z) {
    for (int y = 0; y < source.size_y; ++y) {
      for (int x = 0; x < source.size_x; ++x) {
        if (source.state[source.Index(x, y, z)] !=
            static_cast<std::uint8_t>(OccupancyState::kOccupied) &&
            live_hits[source.Index(x, y, z)] == 0U) {
          continue;
        }
        for (int dz = -down_cells; dz <= up_cells; ++dz) {
          const float world_dz = static_cast<float>(dz) * config.resolution_m;
          if ((dz > 0 && world_dz > config.inflation_z_up_m + 1.0e-6F) ||
              (dz < 0 && -world_dz > config.inflation_z_down_m + 1.0e-6F)) {
            continue;
          }
          for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
            for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
              const float world_dx = static_cast<float>(dx) * config.resolution_m;
              const float world_dy = static_cast<float>(dy) * config.resolution_m;
              const int target_x = x + dx;
              const int target_y = y + dy;
              const int target_z = z + dz;
              const bool inside_radius =
                  (dx == 0 && dy == 0) ||
                  world_dx * world_dx + world_dy * world_dy < radius_squared;
              if (!inside_radius ||
                  target_x < 0 || target_x >= source.size_x || target_y < 0 ||
                  target_y >= source.size_y || target_z < 0 ||
                  target_z >= source.size_z) {
                continue;
              }
              expected[source.Index(target_x, target_y, target_z)] = 1U;
            }
          }
        }
      }
    }
  }
  std::size_t count = 0U;
  for (int z = 0; z < source.size_z; ++z) {
    for (int y = 0; y < source.size_y; ++y) {
      for (int x = 0; x < source.size_x; ++x) {
        const bool occupied = expected[source.Index(x, y, z)] != 0U;
        assert(inflated.Occupied(x, y, z) == occupied);
        count += occupied ? 1U : 0U;
      }
    }
  }
  assert(inflated.occupied_cells == count);
}

void TestIncrementalInflationMatchesFullRebuildAcrossUpdates() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.inflation_radius_m = 1.5F;
  config.inflation_z_up_m = 1.0F;
  config.inflation_z_down_m = 1.0F;
  config.decay_after_ns = 25;
  config.decay_factor = 0.0F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  std::uint32_t random = 0x6d2b79f5U;
  for (int step = 1; step <= 30; ++step) {
    const auto snapshot = grid.Snapshot();
    std::vector<float> points;
    for (int point = 0; point < 3; ++point) {
      random = random * 1664525U + 1013904223U;
      const int x = 1 + static_cast<int>(random %
                                         static_cast<std::uint32_t>(snapshot.size_x - 2));
      random = random * 1664525U + 1013904223U;
      const int y = 1 + static_cast<int>(random %
                                         static_cast<std::uint32_t>(snapshot.size_y - 2));
      random = random * 1664525U + 1013904223U;
      const int z = 1 + static_cast<int>(random %
                                         static_cast<std::uint32_t>(snapshot.size_z - 2));
      points.insert(points.end(), {
          static_cast<float>(snapshot.origin_x_m +
                             (static_cast<double>(x) + 0.5) * snapshot.resolution_m),
          static_cast<float>(snapshot.origin_y_m +
                             (static_cast<double>(y) + 0.5) * snapshot.resolution_m),
          static_cast<float>(snapshot.origin_z_m +
                             (static_cast<double>(z) + 0.5) * snapshot.resolution_m),
      });
    }
    const std::int64_t stamp = static_cast<std::int64_t>(step) * 10;
    const float center_x = snapshot.origin_x_m +
                           0.5F * static_cast<float>(snapshot.size_x) * snapshot.resolution_m;
    const float center_y = snapshot.origin_y_m +
                           0.5F * static_cast<float>(snapshot.size_y) * snapshot.resolution_m;
    const float center_z = snapshot.origin_z_m +
                           0.5F * static_cast<float>(snapshot.size_z) * snapshot.resolution_m;
    grid.Update(Frame(points, stamp, center_x, center_y, center_z));
    AssertInflationMatchesFullRebuild(grid, points);
    if (step % 7 == 0) {
      grid.Decay(stamp + 30);
      AssertInflationMatchesFullRebuild(grid);
    }
    if (step == 15) {
      grid.RollToCenter(2.0F, 1.0F, 0.0F, stamp + 1);
      AssertInflationMatchesFullRebuild(grid, points);
    }
  }
}

void TestSubVoxelVerticalInflationUsesOfficialCeil() {
  auto config = TestConfig();
  config.resolution_m = 0.25F;
  config.inflation_z_up_m = 0.10F;
  config.inflation_z_down_m = 0.10F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({0.1F, 0.1F, 0.1F}, 10));
  assert(grid.InflatedContains(0.1F, 0.1F, 0.1F));
  assert(grid.InflatedContains(0.1F, 0.1F, 0.35F));
}

void TestBodyClearancesCoverLowShelfAndOverheadButNotRoad() {
  for (const bool body_envelope : {false, true}) {
    auto config = TestConfig();
    config.size_x = 40;
    config.size_y = 20;
    config.size_z = 40;
    config.resolution_m = 0.05;
    config.auto_roll = false;
    config.inflation_z_up_m = body_envelope ? 0.25 : 0.10;
    config.inflation_z_down_m = body_envelope ? 0.35 : 0.10;
    RollingOccupancyGrid grid(config);
    grid.Reset("map", 0.0, 0.0, 0.5, 1);
    grid.Update(Frame({0.725F, -0.225F, 0.275F,
                       0.725F, 0.225F, 0.775F,
                       -0.725F, 0.0F, 0.025F}, 10, 0.0F, 0.0F, 0.475F));

    assert(grid.InflatedContains(0.725, -0.225, 0.475) == body_envelope);
    assert(grid.InflatedContains(0.725, 0.225, 0.475) == body_envelope);
    assert(!grid.InflatedContains(-0.725, 0.0, 0.475));
  }
}

void TestOddWindowUsesOfficialInitialVoxelOrigin() {
  auto config = TestConfig();
  config.size_x = 5;
  config.size_y = 5;
  config.size_z = 5;
  config.roll_margin_x = 1;
  config.roll_margin_y = 1;
  config.roll_margin_z = 1;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);

  const auto snapshot = grid.Snapshot();
  assert(snapshot.origin_x_m == -3.0);
  assert(snapshot.origin_y_m == -3.0);
  assert(snapshot.origin_z_m == -3.0);
}

void TestZeroLengthReturnIsAnOfficialEndpointHit() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);

  const auto stats = grid.Update(Frame({0.0F, 0.0F, 0.0F}, 10));
  assert(stats.accepted_points == 1U);
  assert(stats.hit_updates == 1U);
  assert(grid.StateAt(0.0, 0.0, 0.0) == OccupancyState::kOccupied);
}

void TestConfiguredLocalUpdateRangeMatchesUpstreamFilter() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.local_update_range_x_m = 0.5;
  config.local_update_range_y_m = 0.5;
  config.local_update_range_z_m = 0.5;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0, 0.0, 0.0, 1);

  const auto stats = grid.Update(Frame({1.2F, 0.0F, 0.0F}, 10));
  assert(stats.accepted_points == 0U);
  assert(stats.rejected_points == 1U);
}

}  // namespace

int main() {
  TestNewObstacleInKnownFreeSpaceRequiresObservedClearing();
  TestEndpointMajorityPreservesHistoryUntilObservedFree();
  TestClippedEndpointContributesOneFreeVote();
  TestSharedEndpointRaysRetainDistinctFreeSegments();
  TestIntersectingRaysClearOnlyObservedHistory();
  TestOfficialUnknownPriorRequiresRepeatedHits();
  TestRayProducesFreeAndOccupiedEvidence();
  TestDenseFrameDoesNotOverweightDuplicateVoxels();
  TestRayFusionMatchesOfficialEndpointAndTraversalVotes();
  TestRayCornerContactLeavesSideCellsUnknown();
  TestObliqueRayClearsOnlyMeasuredSegment();
  TestRayStopsAtSensorBoundary();
  TestRayCornerContactDoesNotClearNeighbor();
  TestSubthresholdObservationIsKnownFreeLikeOfficialGridMap();
  TestUnresolvedHitsRequireClearingWithoutChangingHistoricalOccupancy();
  TestCurrentHitRollResetAndSharedInflation();
  TestRangeClippingDoesNotCreateCurrentCollisionHit();
  TestInflationExcludesExactRadiusBoundary();
  TestCollisionGenerationAdvancesOnlyWhenInflatedGeometryChanges();
  TestWindowRollEmitsOutgoingAndPreservesOverlap();
  TestSlidingThresholdIsSymmetricLikeUpstream();
  TestShortPointOutsideUpstreamLocalRangeIsIgnored();
  TestWindowRollClearsInflationFromEveryReusedCell();
  TestAutoRollAndCellMutationCommitOneGeneration();
  TestDecayAndOutOfOrderGate();
  TestFrameMismatchFailsClosed();
  TestIncrementalInflationKeepsSharedCoverageUntilLastSourceClears();
  TestIncrementalInflationMatchesFullRebuildAcrossUpdates();
  TestSubVoxelVerticalInflationUsesOfficialCeil();
  TestBodyClearancesCoverLowShelfAndOverheadButNotRoad();
  TestOddWindowUsesOfficialInitialVoxelOrigin();
  TestZeroLengthReturnIsAnOfficialEndpointHit();
  TestConfiguredLocalUpdateRangeMatchesUpstreamFilter();
  return 0;
}
