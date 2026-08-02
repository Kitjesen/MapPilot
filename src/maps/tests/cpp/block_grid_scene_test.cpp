#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "lingtu/maps/block_grid.hpp"
#include "lingtu/maps/scene.hpp"

namespace {

using lingtu::maps::BlockGridConfig;
using lingtu::maps::BlockGridRoi;
using lingtu::maps::CloudLayout;
using lingtu::maps::OwnedPointCloud;
using lingtu::maps::PersistentBlockGrid;
using lingtu::maps::SceneHealth;
using lingtu::maps::ScenePath;
using lingtu::maps::ScenePose;
using lingtu::maps::SceneSnapshotBuilder;
using lingtu::maps::layers::makeGrid2D;

std::filesystem::path TempDir() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto path = std::filesystem::temp_directory_path() /
              ("lingtu_block_grid_scene_test_" + std::to_string(stamp));
  std::filesystem::create_directories(path);
  return path;
}

BlockGridConfig TestConfig() {
  BlockGridConfig config;
  config.cell_size_m = 1.0F;
  config.block_size = 4;
  config.hit_log_odds = 1.0F;
  config.miss_log_odds = 1.0F;
  config.prune_abs_log_odds_below = 0.01F;
  return config;
}

void TestRayFreeSpaceAndPersistenceRoundTrip() {
  auto root = TempDir();
  const auto artifact = root / "grid.ltbg";

  PersistentBlockGrid grid(TestConfig());
  grid.SetFrame("map");
  grid.SetStampNs(1234);
  grid.InsertRay(0.0F, 0.0F, 0.0F, 3.2F, 0.0F, 0.0F);
  assert(grid.CellCount() >= 4U);
  assert(grid.OccupancyProbability(0.1F, 0.0F, 0.0F) < 0.5F);
  assert(grid.Contains(3.2F, 0.0F, 0.0F));

  grid.SaveBinary(artifact);
  std::string error;
  assert(PersistentBlockGrid::ValidateBinary(artifact, &error));
  assert(error.empty());

  const auto loaded = PersistentBlockGrid::LoadBinary(artifact);
  assert(loaded.CellCount() == grid.CellCount());
  assert(loaded.Contains(3.2F, 0.0F, 0.0F));
  assert(loaded.OccupancyProbability(0.1F, 0.0F, 0.0F) < 0.5F);

  std::filesystem::remove_all(root);
}

void TestColumnClearDecayAndRoiSnapshot() {
  PersistentBlockGrid grid(TestConfig());
  grid.InsertHit(1.2F, 1.2F, 0.2F);
  grid.InsertHit(1.2F, 1.2F, 2.2F);
  grid.InsertHit(5.2F, 1.2F, 0.2F);
  assert(grid.CellCount() == 3U);

  const std::size_t cleared = grid.ClearColumn(1.2F, 1.2F);
  assert(cleared == 2U);
  assert(grid.CellCount() == 1U);
  assert(!grid.Contains(1.2F, 1.2F, 0.2F));
  assert(grid.Contains(5.2F, 1.2F, 0.2F));

  BlockGridRoi roi;
  roi.enabled = true;
  roi.min_x_m = 5.0F;
  roi.max_x_m = 6.0F;
  roi.min_y_m = 1.0F;
  roi.max_y_m = 2.0F;
  roi.min_z_m = 0.0F;
  roi.max_z_m = 1.0F;
  roi.max_cells = 1U;
  const auto snapshot = grid.Snapshot(roi);
  assert(snapshot.Size() == 1U);
  assert(snapshot.center_x_m[0] > 5.0F);

  const auto generation = grid.Generation();
  assert(grid.Decay(0.9F) == 0U);
  assert(grid.Generation() == generation + 1U);
  const std::size_t pruned = grid.Decay(0.0F);
  assert(pruned == 1U);
  assert(grid.CellCount() == 0U);
}

void TestBatchedBandClearAndDeterministicCap() {
  PersistentBlockGrid grid(TestConfig());
  grid.InsertHit(1.2F, 1.2F, 0.2F);
  grid.InsertHit(1.2F, 1.2F, 4.2F);
  grid.InsertHit(2.2F, 1.2F, 0.2F);
  const float columns[] = {1.2F, 1.2F, 2.2F, 1.2F};
  assert(grid.ClearColumns(columns, 2U, -1.0F, 2.0F) == 2U);
  assert(grid.Contains(1.2F, 1.2F, 4.2F));
  assert(grid.CellCount() == 1U);

  PersistentBlockGrid capped(TestConfig());
  capped.InsertHit(9.2F, 0.2F, 0.2F);
  capped.InsertHit(2.2F, 0.2F, 0.2F);
  capped.InsertHit(1.2F, 0.2F, 0.2F);
  BlockGridRoi roi;
  roi.enabled = true;
  roi.min_x_m = -10.0F;
  roi.max_x_m = 10.0F;
  roi.min_y_m = -1.0F;
  roi.max_y_m = 1.0F;
  roi.min_z_m = -1.0F;
  roi.max_z_m = 1.0F;
  roi.max_cells = 2U;
  const auto first = capped.Snapshot(roi);
  const auto second = capped.Snapshot(roi);
  assert(first.ix == second.ix);
  assert(first.Size() == 2U);
  assert(first.ix[0] == 1);
  assert(first.ix[1] == 2);
}

void TestMalformedBinaryRejected() {
  auto root = TempDir();
  const auto artifact = root / "bad.ltbg";
  {
    std::ofstream file(artifact, std::ios::binary | std::ios::trunc);
    file << "not a block grid";
  }
  std::string error;
  assert(!PersistentBlockGrid::ValidateBinary(artifact, &error));
  assert(!error.empty());
  bool threw = false;
  try {
    static_cast<void>(PersistentBlockGrid::LoadBinary(artifact));
  } catch (...) {
    threw = true;
  }
  assert(threw);
  std::filesystem::remove_all(root);
}

void TestRayCapacityRejectionIsAtomic() {
  auto config = TestConfig();
  config.max_runtime_cells = 3U;
  config.max_runtime_blocks = 8U;
  PersistentBlockGrid grid(config);

  const float origins[] = {0.0F, 0.0F, 0.0F};
  const float hits[] = {3.2F, 0.0F, 0.0F};
  const auto stats = grid.InsertRays(origins, hits, 1U);

  assert(stats.rays == 1U);
  assert(stats.capacity_rejections > 0U);
  assert(stats.free_updates == 0U);
  assert(stats.hit_updates == 0U);
  assert(grid.CellCount() == 0U);
  assert(grid.OccupancyProbability(0.1F, 0.0F, 0.0F) == 0.5F);
  assert(grid.OccupancyProbability(3.2F, 0.0F, 0.0F) == 0.5F);
}

void TestSaveRejectsArtifactAbovePersistedLimit() {
  auto root = TempDir();
  const auto artifact = root / "too_large.ltbg";
  auto config = TestConfig();
  config.max_runtime_cells = 8U;
  config.max_persisted_cells = 1U;
  PersistentBlockGrid grid(config);
  grid.InsertHit(0.2F, 0.2F, 0.2F);
  grid.InsertHit(1.2F, 0.2F, 0.2F);
  assert(grid.CellCount() == 2U);

  bool threw = false;
  try {
    grid.SaveBinary(artifact);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  assert(threw);
  assert(!std::filesystem::exists(artifact));
  std::filesystem::remove_all(root);
}

void TestSceneSnapshotContract() {
  OwnedPointCloud live;
  live.frame_id = "camera";
  live.stamp_ns = 99;
  live.layout = CloudLayout::kXyzF32SoA;
  live.point_count = 2;
  live.x = {1.0F, 2.0F};
  live.y = {0.0F, 0.5F};
  live.z = {0.2F, 0.3F};

  PersistentBlockGrid grid(TestConfig());
  grid.SetFrame("map");
  grid.SetStampNs(100);
  grid.InsertHit(2.0F, 0.0F, 0.0F);

  SceneSnapshotBuilder builder;
  builder.SetFrame("map", 100);
  builder.SetHealth(SceneHealth{true, true, false, 0x3U, "planner degraded"});
  builder.SetRobotPose(ScenePose{1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  builder.SetLiveCloud(live.View());
  builder.SetAccumulatedCloud(grid.Snapshot());
  builder.SetFloorHeight(makeGrid2D(2, 2, 0.5, 0.0, 0.0, 0.0F));
  builder.SetTraversabilityCost(makeGrid2D(2, 2, 0.5, 0.0, 0.0, 10.0F));
  builder.SetPath(ScenePath{"map", 100, {1.0F, 2.0F}, {2.0F, 2.5F}, {0.0F, 0.0F}});

  const auto scene = builder.Build();
  assert(scene.frame_id == "map");
  assert(scene.stamp_ns == 100);
  assert(scene.health.localization_ok);
  assert(scene.live_cloud.point_count == 2U);
  assert(scene.accumulated_cloud.Size() == 1U);
  assert(scene.floor_height.data.size() == 4U);
  assert(scene.traversability_cost.data[0] == 10.0F);
  assert(scene.path.x_m.size() == 2U);
}

}  // namespace

int main() {
  TestRayFreeSpaceAndPersistenceRoundTrip();
  TestColumnClearDecayAndRoiSnapshot();
  TestBatchedBandClearAndDeterministicCap();
  TestMalformedBinaryRejected();
  TestRayCapacityRejectionIsAtomic();
  TestSaveRejectsArtifactAbovePersistedLimit();
  TestSceneSnapshotContract();
  return 0;
}
