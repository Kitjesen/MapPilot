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
  const auto occupied = grid.OccupiedSnapshot(8U);
  assert(occupied.total_cells == 1U);
  assert(occupied.centers_xyz.size() == 3U);
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
  const auto occupied = grid.OccupiedSnapshot(8U);
  assert(occupied.total_cells == 1U);
  assert(occupied.centers_xyz.size() == 3U);

  const auto snapshot = grid.Snapshot();
  assert(std::fabs(snapshot.origin_x_m - -1.0F) < 0.01F);
  assert(snapshot.CellCount() == 8U * 8U * 4U);
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
  assert(grid.OccupiedSnapshot(8U).total_cells == 0U);

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

}  // namespace

int main() {
  TestRayProducesFreeAndOccupiedEvidence();
  TestDenseFrameDoesNotOverweightDuplicateVoxels();
  TestWindowRollEmitsOutgoingAndPreservesOverlap();
  TestAutoRollAndCellMutationCommitOneGeneration();
  TestDecayAndOutOfOrderGate();
  TestFrameMismatchFailsClosed();
  return 0;
}
