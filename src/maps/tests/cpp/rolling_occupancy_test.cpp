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
  config.hit_log_odds = 2.0F;
  config.miss_log_odds = 2.0F;
  config.inflation_radius_m = 0.0F;
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
  assert(stats.free_updates >= 3U);
  assert(grid.StateAt(3.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.StateAt(0.1F, 0.1F, 0.1F) == OccupancyState::kFree);
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

void TestRayFusionUsesOfficialHitWinsTie() {
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
  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
  assert(grid.StateAt(3.2F, 0.1F, 0.1F) == OccupancyState::kOccupied);
}

void TestRayCornerTieMatchesOfficialSingleAxisStep() {
  auto config = TestConfig();
  config.auto_roll = false;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({2.2F, 2.2F, 0.1F}, 10, 0.1F, 0.1F, 0.1F));

  assert(grid.StateAt(0.1F, 1.1F, 0.1F) == OccupancyState::kFree);
  assert(grid.StateAt(1.1F, 0.1F, 0.1F) == OccupancyState::kUnknown);
}

void TestOccupiedThresholdIsStrictLikeOfficialGridMap() {
  auto config = TestConfig();
  config.auto_roll = false;
  config.hit_log_odds =
      std::log(config.occupied_probability / (1.0F - config.occupied_probability));
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({2.2F, 0.1F, 0.1F}, 10));

  assert(grid.StateAt(2.2F, 0.1F, 0.1F) == OccupancyState::kUnknown);
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

void AssertInflationMatchesFullRebuild(const RollingOccupancyGrid& grid) {
  const auto config = grid.Config();
  const auto source = grid.Snapshot();
  const auto inflated = grid.InflatedSnapshot();
  std::vector<std::uint8_t> expected(source.CellCount(), 0U);
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
            static_cast<std::uint8_t>(OccupancyState::kOccupied)) {
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
          snapshot.origin_x_m + (static_cast<float>(x) + 0.5F) * snapshot.resolution_m,
          snapshot.origin_y_m + (static_cast<float>(y) + 0.5F) * snapshot.resolution_m,
          snapshot.origin_z_m + (static_cast<float>(z) + 0.5F) * snapshot.resolution_m,
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
    AssertInflationMatchesFullRebuild(grid);
    if (step % 7 == 0) {
      grid.Decay(stamp + 30);
      AssertInflationMatchesFullRebuild(grid);
    }
    if (step == 15) {
      grid.RollToCenter(2.0F, 1.0F, 0.0F, stamp + 1);
      AssertInflationMatchesFullRebuild(grid);
    }
  }
}

void TestSubVoxelVerticalInflationDoesNotGrowOneWholeCell() {
  auto config = TestConfig();
  config.resolution_m = 0.25F;
  config.inflation_z_up_m = 0.10F;
  config.inflation_z_down_m = 0.10F;
  RollingOccupancyGrid grid(config);
  grid.Reset("map", 0.0F, 0.0F, 0.0F, 1);

  grid.Update(Frame({0.1F, 0.1F, 0.1F}, 10));
  assert(grid.InflatedContains(0.1F, 0.1F, 0.1F));
  assert(!grid.InflatedContains(0.1F, 0.1F, 0.35F));
}

}  // namespace

int main() {
  TestRayProducesFreeAndOccupiedEvidence();
  TestDenseFrameDoesNotOverweightDuplicateVoxels();
  TestRayFusionUsesOfficialHitWinsTie();
  TestRayCornerTieMatchesOfficialSingleAxisStep();
  TestOccupiedThresholdIsStrictLikeOfficialGridMap();
  TestInflationExcludesExactRadiusBoundary();
  TestCollisionGenerationAdvancesOnlyWhenInflatedGeometryChanges();
  TestWindowRollEmitsOutgoingAndPreservesOverlap();
  TestWindowRollClearsInflationFromEveryReusedCell();
  TestAutoRollAndCellMutationCommitOneGeneration();
  TestDecayAndOutOfOrderGate();
  TestFrameMismatchFailsClosed();
  TestIncrementalInflationKeepsSharedCoverageUntilLastSourceClears();
  TestIncrementalInflationMatchesFullRebuildAcrossUpdates();
  TestSubVoxelVerticalInflationDoesNotGrowOneWholeCell();
  return 0;
}
