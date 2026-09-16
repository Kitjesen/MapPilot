#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#include "lingtu/maps/mapd/engine.hpp"

namespace {

using lingtu::maps::CloudLayout;
using lingtu::maps::OwnedPointCloud;
using lingtu::maps::mapd::Config;
using lingtu::maps::mapd::LiveMapEngine;
using lingtu::maps::mapd::Observation;
using lingtu::maps::mapd::SnapshotDetail;
using lingtu::maps::mapd::SubmitCode;

std::int64_t WallTimeNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

Config TestConfig() {
  Config config;
  config.max_points_per_observation = 1000U;
  config.min_range_m = 0.0F;
  config.max_range_m = 50.0F;
  config.min_height_from_sensor_m = -10.0F;
  config.max_height_from_sensor_m = 10.0F;
  config.occupancy_min_height_from_sensor_m = -2.0F;
  config.occupancy_max_height_from_sensor_m = 2.0F;
  config.decay_period = std::chrono::milliseconds(20);
  config.stale_after = std::chrono::seconds(1);
  config.accumulated_decay_factor = 1.0F;
  config.voxel.voxel_size_m = 0.5F;
  config.voxel.decay_rate = 0.0F;
  config.voxel.prune_below_count = 0.5F;
  config.occupancy.size_x = 16;
  config.occupancy.size_y = 16;
  config.occupancy.size_z = 8;
  config.occupancy.resolution_m = 0.5F;
  config.occupancy.hit_log_odds = 4.0F;
  config.occupancy.miss_log_odds = 2.0F;
  config.occupancy.inflation_radius_m = 0.01F;
  config.occupancy.inflation_z_up_m = 0.0F;
  config.occupancy.inflation_z_down_m = 0.0F;
  config.occupancy.roll_margin_x = 2;
  config.occupancy.roll_margin_y = 2;
  config.occupancy.roll_margin_z = 1;
  config.occupancy.max_ray_range_m = 50.0F;
  config.accumulated.cell_size_m = 0.5F;
  config.accumulated.block_size = 8;
  return config;
}

Observation MakeObservation(
    std::uint64_t epoch,
    std::uint64_t sequence,
    double x,
    double y,
    double yaw,
    const std::vector<float>& points) {
  Observation observation;
  observation.reset_epoch = epoch;
  observation.sequence = sequence;
  observation.stamp_ns = WallTimeNs();
  observation.map_frame = "map";
  observation.sensor_frame = "lidar";
  observation.map_sensor.x = x;
  observation.map_sensor.y = y;
  observation.map_sensor.qz = std::sin(yaw * 0.5);
  observation.map_sensor.qw = std::cos(yaw * 0.5);
  observation.sensor_origin_x_m = static_cast<float>(x);
  observation.sensor_origin_y_m = static_cast<float>(y);
  observation.pose_quality = 1.0F;
  observation.pose_state = "TRACKING";
  observation.scan.frame_id = "lidar";
  observation.scan.stamp_ns = observation.stamp_ns;
  observation.scan.layout = CloudLayout::kXyzF32Interleaved;
  observation.scan.point_count = points.size() / 3U;
  observation.scan.interleaved = points;
  return observation;
}

void TestExactPoseTransformAndDerivedLayers() {
  Config config = TestConfig();
  config.max_accumulated_snapshot_cells = 2U;
  LiveMapEngine engine(config);
  engine.Start();
  auto observation = MakeObservation(
      1U,
      1U,
      10.0,
      2.0,
      3.14159265358979323846 / 2.0,
      {1.0F, 0.0F, 0.0F});
  assert(engine.Submit(std::move(observation)).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  const auto snapshot = engine.GetSnapshot();
  assert(snapshot.reset_epoch == 1U);
  assert(snapshot.sequence == 1U);
  assert(snapshot.live_cloud.point_count == 1U);
  assert(std::fabs(snapshot.live_cloud.interleaved[0] - 10.0F) < 1.0e-4F);
  assert(std::fabs(snapshot.live_cloud.interleaved[1] - 3.0F) < 1.0e-4F);
  assert(snapshot.voxel_cloud.point_count == 1U);
  assert(snapshot.accumulated_cloud.Size() > 0U);
  assert(snapshot.accumulated_cloud.Size() <= 2U);
  assert(!snapshot.occupancy.empty());
  assert(!snapshot.esdf.distance.empty());
  const auto state = engine.GetState();
  assert(state.live);
  assert(state.accumulated_cells >= state.accumulated_snapshot_cells);
  engine.Stop();
}

void TestScanOccupancyReceivesPointsBeforeGenericMapFilters() {
  Config config = TestConfig();
  config.min_range_m = 1.0F;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(
      MakeObservation(1U, 1U, 0.0, 0.0, 0.0, {0.5F, 0.0F, 0.0F}))
             .accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  const auto snapshot = engine.GetSnapshot();
  assert(snapshot.live_cloud.point_count == 0U);
  assert(snapshot.voxel_cloud.point_count == 0U);
  assert(snapshot.collision.occupied_cells == 1U);
  engine.Stop();
}

void TestIdentityGateAndEpochReset() {
  LiveMapEngine engine(TestConfig());
  engine.Start();
  assert(engine.Submit(
      MakeObservation(7U, 10U, 0.0, 0.0, 0.0, {2.0F, 0.0F, 0.0F}))
             .accepted());
  assert(engine.WaitUntilProcessed(7U, 10U, std::chrono::seconds(2)));

  const auto duplicate = engine.Submit(
      MakeObservation(7U, 10U, 0.0, 0.0, 0.0, {3.0F, 0.0F, 0.0F}));
  assert(duplicate.code == SubmitCode::kStale);

  assert(engine.Submit(
      MakeObservation(8U, 1U, 20.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F}))
             .accepted());
  assert(engine.WaitUntilProcessed(8U, 1U, std::chrono::seconds(2)));
  const auto snapshot = engine.GetSnapshot();
  assert(snapshot.reset_epoch == 8U);
  assert(snapshot.sequence == 1U);
  assert(snapshot.voxel_cloud.point_count == 1U);
  assert(snapshot.voxel_cloud.x.front() > 19.0F);
  for (const float x : snapshot.accumulated_cloud.center_x_m) {
    assert(x > 19.0F);
  }
  const auto state = engine.GetState();
  assert(state.epoch_resets == 2U);
  assert(state.stale_observations == 1U);
  engine.Stop();
}

void TestIndependentDecayWithoutNewObservations() {
  Config config = TestConfig();
  config.voxel.decay_rate = 0.75F;
  config.voxel.prune_below_count = 0.9F;
  config.occupancy.decay_after_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::milliseconds(30))
          .count();
  config.occupancy.decay_factor = 0.0F;
  config.accumulated_decay_factor = 0.0F;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(
      MakeObservation(1U, 1U, 0.0, 0.0, 0.0, {2.0F, 0.0F, 0.0F}))
             .accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));
  // Processing completion does not pause the independent decay worker. Check
  // one coherent state whether this reader runs before or after the first tick.
  const auto initial = engine.GetState();
  assert(initial.processed_observations == 1U);
  assert((initial.generation == 1U && initial.voxel_cells == 1U &&
          initial.accumulated_cells > 0U) ||
         (initial.generation > 1U && initial.voxel_cells == 0U &&
          initial.accumulated_cells == 0U));

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto state = engine.GetState();
    if (state.voxel_cells == 0U && state.accumulated_cells == 0U) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto state = engine.GetState();
  assert(state.voxel_cells == 0U);
  assert(state.accumulated_cells == 0U);
  assert(state.generation > 1U);
  assert(state.processed_observations == 1U);
  assert(state.last_error.empty());
  engine.Stop();
}

void TestMalformedObservationRejectedBeforeQueue() {
  LiveMapEngine engine(TestConfig());
  engine.Start();
  Observation invalid;
  const auto result = engine.Submit(std::move(invalid));
  assert(result.code == SubmitCode::kInvalid);
  assert(engine.GetState().invalid_observations == 1U);
  engine.Stop();
}

void TestUnsafePoseStateRejected() {
  LiveMapEngine engine(TestConfig());
  engine.Start();
  auto lost =
      MakeObservation(1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F});
  lost.pose_state = "LOST";
  const auto lost_result = engine.Submit(std::move(lost));
  assert(lost_result.code == SubmitCode::kInvalid);

  Config strict = TestConfig();
  strict.accept_degraded_pose = false;
  LiveMapEngine strict_engine(strict);
  strict_engine.Start();
  auto degraded =
      MakeObservation(1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F});
  degraded.pose_state = "DEGRADED";
  assert(strict_engine.Submit(std::move(degraded)).code == SubmitCode::kInvalid);
  strict_engine.Stop();
  engine.Stop();
}

void TestVoxelSnapshotHasHardPointAndRoiBounds() {
  Config config = TestConfig();
  config.max_voxel_snapshot_points = 2U;
  config.voxel_snapshot_radius_m = 3.0F;
  config.voxel_snapshot_min_z_from_sensor_m = -1.0F;
  config.voxel_snapshot_max_z_from_sensor_m = 1.0F;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(MakeObservation(
      1U,
      1U,
      0.0,
      0.0,
      0.0,
      {
          1.0F, 0.0F, 0.0F,
          1.5F, 0.0F, 0.0F,
          2.0F, 0.0F, 0.0F,
          0.5F, 0.0F, 2.0F,
      })).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));
  const auto snapshot = engine.GetSnapshot();
  const auto state = engine.GetState();
  assert(snapshot.voxel_cloud.point_count == 2U);
  assert(state.voxel_points == 2U);
  assert(state.voxel_cells == 3U);
  assert(state.voxel_snapshot_omitted_cells == 1U);
  engine.Stop();
}

void TestSurfaceRetentionAndCapacityRecovery() {
  Config config = TestConfig();
  config.voxel = Config{}.voxel;
  config.voxel.max_voxels = 2U;
  config.voxel_snapshot_radius_m = 3.0F;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(MakeObservation(1U, 1U, 0, 0, 0,
      {1.0F, 0.0F, -0.4F, 1.5F, 0.0F, -0.4F, 2.0F, 0.0F, -0.4F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));
  assert(engine.GetState().capacity_limited);
  // Several maintenance ticks must not erase single-hit observed surfaces.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  assert(engine.GetState().voxel_cells == 2U);
  assert(engine.GetState().capacity_limited);
  // Fully range-filtered observations do not prove geometry recovered.
  assert(engine.Submit(MakeObservation(1U, 2U, 0, 0, 0, {100, 0, 0})).accepted());
  assert(engine.WaitUntilProcessed(1U, 2U, std::chrono::seconds(2)));
  assert(engine.GetState().capacity_limited);
  assert(engine.Submit(MakeObservation(1U, 3U, 10, 0, 0,
      {1.0F, 0.0F, -0.4F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 3U, std::chrono::seconds(2)));
  auto state = engine.GetState();
  assert(!state.capacity_limited);
  assert(state.voxel_capacity_rejections == 1U);
  auto snapshot = engine.GetSnapshot();
  assert(snapshot.voxel_cloud.point_count == 1U);
  assert(snapshot.voxel_cloud.x.front() > 10.0F);
  // An old cell outside the window must not reappear when revisiting it.
  assert(engine.Submit(MakeObservation(1U, 4U, 0, 0, 0,
      {0.5F, 0.0F, -0.4F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 4U, std::chrono::seconds(2)));
  snapshot = engine.GetSnapshot();
  assert(snapshot.voxel_cloud.point_count == 1U);
  assert(snapshot.voxel_cloud.x.front() < 0.6F);
  engine.Stop();
}

void TestColumnCarvingDoesNotClearAdjacentFloorHeight() {
  Config config = TestConfig();
  config.voxel.column_carving = true;
  config.column_carving_min_height_from_sensor_m = -0.7F;
  config.column_carving_max_height_from_sensor_m = 1.8F;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(MakeObservation(
      1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 3.0F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));
  assert(engine.Submit(MakeObservation(
      1U, 2U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 2U, std::chrono::seconds(2)));

  const auto snapshot = engine.GetSnapshot();
  bool saw_current_floor = false;
  bool saw_adjacent_floor = false;
  for (std::size_t index = 0U;
       index < snapshot.voxel_cloud.point_count;
       ++index) {
    saw_current_floor =
        saw_current_floor || snapshot.voxel_cloud.z[index] < 1.0F;
    saw_adjacent_floor =
        saw_adjacent_floor || snapshot.voxel_cloud.z[index] > 2.5F;
  }
  assert(saw_current_floor);
  assert(saw_adjacent_floor);
  assert(engine.GetState().voxel_cells == 2U);
  engine.Stop();
}

void TestSurfaceHitsDoNotClearUnobservedHeights() {
  // A return in the same XY cell does not observe the lower surface. Cover
  // returns both inside and above the height band used by column carving.
  for (const float upper_z : {1.2F, 2.2F}) {
    Config config = TestConfig();
    config.voxel.voxel_size_m = 0.05F;
    config.occupancy.size_z = 16;
    Config carved_config = config;
    carved_config.voxel.column_carving = true;
    LiveMapEngine engine(config);
    LiveMapEngine carved_engine(carved_config);
    engine.Start();
    carved_engine.Start();
    for (const auto& observation : {
             MakeObservation(1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, -0.4F}),
             MakeObservation(1U, 2U, 0.0, 0.0, 0.0, {1.0F, 0.0F, upper_z})}) {
      assert(engine.Submit(observation).accepted());
      assert(carved_engine.Submit(observation).accepted());
      assert(engine.WaitUntilProcessed(
          1U, observation.sequence, std::chrono::seconds(2)));
      assert(carved_engine.WaitUntilProcessed(
          1U, observation.sequence, std::chrono::seconds(2)));
    }

    const auto snapshot = engine.GetSnapshot();
    const auto carved = carved_engine.GetSnapshot();
    assert(snapshot.voxel_cloud.point_count == 2U);
    assert(std::any_of(snapshot.voxel_cloud.z.begin(),
                       snapshot.voxel_cloud.z.end(),
                       [](float z) { return z < 0.0F; }));
    assert(carved.voxel_cloud.point_count == 1U);
    assert(carved.voxel_cloud.z.front() > 1.0F);

    // Surface retention must not change the independent ray occupancy or
    // accumulated map, whose existing column-clearing policy is unchanged.
    assert(snapshot.collision.occupied_cells > 0U);
    assert(snapshot.collision.occupied_bits == carved.collision.occupied_bits);
    assert(snapshot.occupancy.data == carved.occupancy.data);
    assert(snapshot.accumulated_cloud.ix == carved.accumulated_cloud.ix);
    assert(snapshot.accumulated_cloud.iy == carved.accumulated_cloud.iy);
    assert(snapshot.accumulated_cloud.iz == carved.accumulated_cloud.iz);
    assert(snapshot.accumulated_cloud.occupancy_probability ==
           carved.accumulated_cloud.occupancy_probability);
    assert(config.accumulated_column_carving);
    assert(lingtu::maps::layers::VoxelLayerConfig{}.column_carving);
    engine.Stop();
    carved_engine.Stop();
  }
}

void TestRayClearedSurfaceDoesNotLeaveGhosts() {
  Config config = TestConfig();
  LiveMapEngine engine(config);
  engine.Start();
  const auto submit = [&](std::uint64_t seq, const std::vector<float>& points) {
    assert(engine.Submit(MakeObservation(1U, seq, 0, 0, 0, points)).accepted());
    assert(engine.WaitUntilProcessed(1U, seq, std::chrono::seconds(2)));
  };
  submit(1U, {1.25F, .25F, .25F, 1.25F, .25F, -.25F, 1.25F, .25F, 2.25F});
  for (std::uint64_t seq = 2; seq <= 6; ++seq) {
    submit(seq, {2.75F, .25F, .25F});
  }
  const auto cloud = engine.GetSnapshot().voxel_cloud;
  bool ghost = false, floor = false, upper = false, endpoint = false;
  for (std::size_t i = 0; i < cloud.point_count; ++i) {
    ghost |= cloud.x[i] < 1.5F && cloud.z[i] > 0 && cloud.z[i] < .5F;
    floor |= cloud.z[i] < 0;
    upper |= cloud.z[i] > 2;
    endpoint |= cloud.x[i] > 2.5F;
  }
  assert(!ghost);
  assert(floor && upper && endpoint);
  // A new return inside old free space stays until a later ray clears it.
  submit(7U, {1.25F, .25F, .25F});
  submit(8U, {1.25F, .25F, 2.25F});
  const auto reoccupied = engine.GetSnapshot().voxel_cloud;
  bool new_hit = false;
  for (std::size_t i = 0; i < reoccupied.point_count; ++i) {
    new_hit |= reoccupied.x[i] < 1.5F && reoccupied.z[i] > 0 && reoccupied.z[i] < .5F;
  }
  assert(new_hit);
  engine.Stop();
}

void TestRuntimeMapCapacityLimitsFailClosed() {
  Config voxel_config = TestConfig();
  voxel_config.voxel.max_voxels = 2U;
  LiveMapEngine voxel_engine(voxel_config);
  voxel_engine.Start();
  assert(voxel_engine.Submit(MakeObservation(
      1U,
      1U,
      0.0,
      0.0,
      0.0,
      {
          1.0F, 0.0F, 0.0F,
          2.0F, 0.0F, 0.0F,
          3.0F, 0.0F, 0.0F,
      })).accepted());
  assert(voxel_engine.WaitUntilProcessed(
      1U, 1U, std::chrono::seconds(2)));
  auto state = voxel_engine.GetState();
  assert(state.voxel_cells == 2U);
  assert(state.voxel_capacity_rejections == 1U);
  assert(state.capacity_limited);
  voxel_engine.Stop();

  Config accumulated_config = TestConfig();
  accumulated_config.accumulated.max_runtime_cells = 2U;
  accumulated_config.accumulated.max_runtime_blocks = 2U;
  LiveMapEngine accumulated_engine(accumulated_config);
  accumulated_engine.Start();
  assert(accumulated_engine.Submit(MakeObservation(
      1U, 1U, 0.0, 0.0, 0.0, {2.0F, 0.0F, 0.0F})).accepted());
  assert(accumulated_engine.WaitUntilProcessed(
      1U, 1U, std::chrono::seconds(2)));
  state = accumulated_engine.GetState();
  // A ray that cannot fit is rejected atomically. Publishing only its misses
  // would manufacture free space without the corresponding occupied hit.
  assert(state.accumulated_cells == 0U);
  assert(state.accumulated_capacity_rejections > 0U);
  assert(state.capacity_limited);
  accumulated_engine.Stop();
}

void TestCollisionSnapshotCompletenessAndAabb() {
  Config complete_config = TestConfig();
  LiveMapEngine complete_engine(complete_config);
  complete_engine.Start();
  assert(complete_engine.Submit(MakeObservation(
      1U,
      1U,
      0.0,
      0.0,
      0.0,
      {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F})).accepted());
  assert(complete_engine.WaitUntilProcessed(
      1U, 1U, std::chrono::seconds(2)));
  auto snapshot = complete_engine.GetSnapshot();
  const auto first_stamp = snapshot.stamp_ns;
  assert(snapshot.collision.complete);
  assert(snapshot.collision.occupied_cells == 2U);
  const auto collision_generation = snapshot.collision.generation;
  assert(collision_generation > 0U);
  assert(snapshot.collision.size_x == complete_config.occupancy.size_x);
  assert(snapshot.collision.size_y == complete_config.occupancy.size_y);
  assert(snapshot.collision.size_z == complete_config.occupancy.size_z);
  assert(snapshot.collision.occupied_bits.size() ==
         (static_cast<std::size_t>(snapshot.collision.size_x) *
              static_cast<std::size_t>(snapshot.collision.size_y) *
              static_cast<std::size_t>(snapshot.collision.size_z) +
          7U) /
             8U);
  assert(std::fabs(
      snapshot.collision.max_x_m - snapshot.collision.min_x_m -
      complete_config.occupancy.size_x *
          complete_config.occupancy.resolution_m) < 1.0e-5F);
  assert(std::fabs(
      snapshot.collision.max_y_m - snapshot.collision.min_y_m -
      complete_config.occupancy.size_y *
          complete_config.occupancy.resolution_m) < 1.0e-5F);
  assert(std::fabs(
      snapshot.collision.max_z_m - snapshot.collision.min_z_m -
      complete_config.occupancy.size_z *
          complete_config.occupancy.resolution_m) < 1.0e-5F);
  assert(!complete_engine.GetState().capacity_limited);
  assert(complete_engine.GetState().collision_snapshot_builds == 1U);

  assert(complete_engine.Submit(MakeObservation(
      1U,
      2U,
      0.0,
      0.0,
      0.0,
      {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F})).accepted());
  assert(complete_engine.WaitUntilProcessed(
      1U, 2U, std::chrono::seconds(2)));
  snapshot = complete_engine.GetSnapshot();
  assert(snapshot.collision.generation == collision_generation);
  assert(snapshot.sequence == 2U);
  assert(snapshot.stamp_ns >= first_stamp);
  assert(complete_engine.GetState().collision_snapshot_builds == 1U);

  // New occupancy must invalidate the cached bitmap.
  assert(complete_engine.Submit(MakeObservation(
      1U, 3U, 0.0, 0.0, 0.0, {-1.0F, 0.0F, 0.0F})).accepted());
  assert(complete_engine.WaitUntilProcessed(1U, 3U, std::chrono::seconds(2)));
  snapshot = complete_engine.GetSnapshot();
  assert(snapshot.collision.generation > collision_generation);
  assert(snapshot.collision.occupied_cells == 3U);
  assert(complete_engine.GetState().collision_snapshot_builds == 2U);

  // A new epoch cannot reuse the old epoch's bitmap, even if generation repeats.
  assert(complete_engine.Submit(MakeObservation(
      2U, 1U, 0.0, 0.0, 0.0, {0.0F, -1.0F, 0.0F})).accepted());
  assert(complete_engine.WaitUntilProcessed(2U, 1U, std::chrono::seconds(2)));
  snapshot = complete_engine.GetSnapshot();
  assert(snapshot.reset_epoch == 2U);
  assert(snapshot.collision.occupied_cells == 1U);
  assert(complete_engine.GetState().collision_snapshot_builds == 3U);

  const auto previous_min_x = snapshot.collision.min_x_m;
  assert(complete_engine.Submit(MakeObservation(
      2U, 2U, 3.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F})).accepted());
  assert(complete_engine.WaitUntilProcessed(2U, 2U, std::chrono::seconds(2)));
  snapshot = complete_engine.GetSnapshot();
  assert(snapshot.collision.min_x_m > previous_min_x);
  assert(complete_engine.GetState().collision_snapshot_builds == 4U);
  complete_engine.Stop();
}

void TestCollisionSnapshotKeepsNearbyGroundInNearbyCells() {
  Config config;
  config.max_points_per_observation = 1000U;
  config.min_range_m = 0.0F;
  config.max_range_m = 12.0F;
  config.accumulated_decay_factor = 1.0F;
  LiveMapEngine engine(config);
  engine.Start();

  std::vector<float> ground;
  for (int ix = -5; ix <= 20; ++ix) {
    for (int iy = -9; iy <= 9; ++iy) {
      ground.push_back(static_cast<float>(ix) * 0.2F);
      ground.push_back(static_cast<float>(iy) * 0.2F);
      ground.push_back(-0.48F);
    }
  }
  auto observation = MakeObservation(1U, 1U, 0.0, 0.0, 0.0, ground);
  observation.map_sensor.z = 0.48;
  observation.sensor_origin_z_m = 0.48F;
  assert(engine.Submit(std::move(observation)).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));
  observation = MakeObservation(1U, 2U, 0.0, 0.0, 0.0, ground);
  observation.map_sensor.z = 0.48;
  observation.sensor_origin_z_m = 0.48F;
  assert(engine.Submit(std::move(observation)).accepted());
  assert(engine.WaitUntilProcessed(1U, 2U, std::chrono::seconds(2)));

  const auto snapshot = engine.GetSnapshot();
  assert(snapshot.collision.complete);
  assert(snapshot.collision.occupied_cells > 0U);
  assert(snapshot.live_cloud.point_count == ground.size() / 3U);
  for (std::size_t index = 0U; index < snapshot.live_cloud.point_count; ++index) {
    const std::size_t offset = index * 3U;
    assert(snapshot.live_cloud.interleaved[offset] >= -1.01F &&
           snapshot.live_cloud.interleaved[offset] <= 4.01F);
    assert(snapshot.live_cloud.interleaved[offset + 1U] >= -1.81F &&
           snapshot.live_cloud.interleaved[offset + 1U] <= 1.81F);
    assert(std::fabs(snapshot.live_cloud.interleaved[offset + 2U]) < 1.0e-4F);
  }
  const std::size_t plane = static_cast<std::size_t>(snapshot.collision.size_x) *
                            static_cast<std::size_t>(snapshot.collision.size_y);
  for (std::size_t index = 0U;
       index < plane * static_cast<std::size_t>(snapshot.collision.size_z); ++index) {
    if ((snapshot.collision.occupied_bits[index / 8U] &
         static_cast<std::uint8_t>(1U << (index % 8U))) == 0U) {
      continue;
    }
    const std::size_t z_index = index / plane;
    const std::size_t remainder = index % plane;
    const std::size_t y_index = remainder / static_cast<std::size_t>(snapshot.collision.size_x);
    const std::size_t x_index = remainder % static_cast<std::size_t>(snapshot.collision.size_x);
    const float x = snapshot.collision.min_x_m +
                    (static_cast<float>(x_index) + 0.5F) * snapshot.collision.resolution_m;
    const float y = snapshot.collision.min_y_m +
                    (static_cast<float>(y_index) + 0.5F) * snapshot.collision.resolution_m;
    const float z = snapshot.collision.min_z_m +
                    (static_cast<float>(z_index) + 0.5F) * snapshot.collision.resolution_m;
    assert(x >= -1.5F && x <= 4.5F);
    assert(y >= -2.25F && y <= 2.25F);
    assert(z >= -0.25F && z <= 0.5F);
  }
  engine.Stop();
}

void TestRealtimeAndCompleteSnapshotsAreBuiltOnDemand() {
  LiveMapEngine engine(TestConfig());
  engine.Start();
  assert(engine.Submit(MakeObservation(
      1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  auto state = engine.GetState();
  assert(state.realtime_snapshot_builds == 0U);
  assert(state.complete_snapshot_builds == 0U);
  assert(state.realtime_snapshot_generation == 0U);
  assert(state.complete_snapshot_generation == 0U);

  const auto realtime = engine.GetView(SnapshotDetail::kRealtime);
  assert(realtime.snapshot.generation == state.generation);
  assert(realtime.snapshot.voxel_cloud.point_count == 1U);
  assert(realtime.snapshot.collision.occupied_cells == 1U);
  assert(realtime.snapshot.accumulated_cloud.Size() == 0U);
  assert(realtime.snapshot.occupancy.empty());
  assert(realtime.snapshot.elevation.valid.empty());
  assert(realtime.snapshot.esdf.distance.empty());
  assert(realtime.state.realtime_snapshot_builds == 1U);
  assert(realtime.state.complete_snapshot_builds == 0U);
  assert(
      realtime.state.realtime_snapshot_generation ==
      realtime.state.generation);
  assert(realtime.state.complete_snapshot_generation == 0U);

  const auto repeated_realtime = engine.GetView(SnapshotDetail::kRealtime);
  assert(repeated_realtime.state.realtime_snapshot_builds == 1U);
  assert(repeated_realtime.state.complete_snapshot_builds == 0U);

  const auto complete = engine.GetView(SnapshotDetail::kComplete);
  assert(complete.snapshot.generation == realtime.snapshot.generation);
  assert(complete.snapshot.accumulated_cloud.Size() > 0U);
  assert(!complete.snapshot.occupancy.empty());
  assert(!complete.snapshot.esdf.distance.empty());
  assert(complete.state.realtime_snapshot_builds == 1U);
  assert(complete.state.complete_snapshot_builds == 1U);
  assert(
      complete.state.complete_snapshot_generation ==
      complete.state.generation);

  const auto repeated_complete = engine.GetView(SnapshotDetail::kComplete);
  assert(repeated_complete.state.realtime_snapshot_builds == 1U);
  assert(repeated_complete.state.complete_snapshot_builds == 1U);

  assert(engine.Submit(MakeObservation(
      1U, 2U, 0.0, 0.0, 0.0, {2.0F, 0.0F, 0.0F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 2U, std::chrono::seconds(2)));
  state = engine.GetState();
  assert(state.generation > complete.state.generation);
  assert(state.realtime_snapshot_builds == 1U);
  assert(state.complete_snapshot_builds == 1U);
  assert(state.realtime_snapshot_generation < state.generation);
  assert(state.complete_snapshot_generation < state.generation);

  const auto next_realtime = engine.GetView(SnapshotDetail::kRealtime);
  assert(next_realtime.state.realtime_snapshot_builds == 2U);
  assert(next_realtime.state.complete_snapshot_builds == 1U);
  assert(next_realtime.snapshot.occupancy.empty());
  engine.Stop();
}

void TestCollisionOnlyRuntimeSkipsExtendedLayers() {
  Config config = TestConfig();
  config.build_extended_layers = false;
  LiveMapEngine engine(config);
  engine.Start();
  assert(engine.Submit(MakeObservation(
      1U, 1U, 0.0, 0.0, 0.0, {1.0F, 0.0F, 0.0F})).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  const auto realtime = engine.GetView(SnapshotDetail::kRealtime);
  assert(realtime.snapshot.live_cloud.point_count == 1U);
  assert(realtime.snapshot.collision.complete);
  assert(realtime.snapshot.collision.occupied_cells == 1U);
  assert(realtime.snapshot.voxel_cloud.point_count == 0U);

  const auto complete = engine.GetView(SnapshotDetail::kComplete);
  assert(complete.snapshot.accumulated_cloud.Size() == 0U);
  assert(complete.snapshot.occupancy.empty());
  assert(complete.snapshot.elevation.valid.empty());
  assert(complete.snapshot.esdf.distance.empty());
  assert(complete.state.voxel_cells == 0U);
  assert(complete.state.accumulated_cells == 0U);
  engine.Stop();
}

void TestDefaultOccupancyDoesNotDecayOutsideOfficialRayUpdates() {
  Config config;
  assert(config.occupancy.decay_after_ns == 0);
}

void TestOccupancyProjectionUsesPhysicalSensorHeight() {
  Config config = TestConfig();
  config.occupancy_min_height_from_sensor_m = -0.3F;
  config.occupancy_max_height_from_sensor_m = 0.3F;
  LiveMapEngine engine(config);
  engine.Start();

  auto observation = MakeObservation(
      1U, 1U, 0.0, 0.0, 0.0, {0.5F, 0.0F, 1.0F});
  observation.map_sensor.z = 0.0;
  observation.sensor_origin_z_m = 1.0F;
  assert(engine.Submit(std::move(observation)).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  const auto snapshot = engine.GetSnapshot();
  assert(snapshot.sensor_origin_z_m == 1.0F);
  assert(std::find(snapshot.occupancy.data.begin(), snapshot.occupancy.data.end(),
                   100.0F) != snapshot.occupancy.data.end());
  engine.Stop();
}

void TestOccupancyKeepsDoublePrecisionThroughPoseTransform() {
  Config config = TestConfig();
  LiveMapEngine engine(config);
  engine.Start();

  auto observation = MakeObservation(
      1U, 1U, 0.09999998, 0.0, 0.0, {0.4F, 0.0F, 0.0F});
  observation.sensor_origin_x_m = 0.09999998F;
  assert(engine.Submit(std::move(observation)).accepted());
  assert(engine.WaitUntilProcessed(1U, 1U, std::chrono::seconds(2)));

  const auto collision = engine.GetView(SnapshotDetail::kRealtime).snapshot.collision;
  const auto occupied = [&collision](int x, int y, int z) {
    const std::size_t linear =
        (static_cast<std::size_t>(z) * static_cast<std::size_t>(collision.size_y) +
         static_cast<std::size_t>(y)) *
            static_cast<std::size_t>(collision.size_x) +
        static_cast<std::size_t>(x);
    return (collision.occupied_bits[linear / 8U] &
            static_cast<std::uint8_t>(1U << (linear % 8U))) != 0U;
  };
  assert(occupied(8, 8, 4));
  assert(!occupied(9, 8, 4));
  engine.Stop();
}

void TestGroundModelConsumesSoaSnapshotAndResetsWithEpoch() {
  Config config = TestConfig();
  config.voxel.voxel_size_m = 0.05F;
  LiveMapEngine engine(config);
  engine.Start();
  std::vector<float> floor;
  for (int r = 0; r < 16; ++r) for (int c = 0; c < 16; ++c)
    floor.insert(floor.end(), {0.525F+c*0.05F, -0.375F+r*0.05F, -0.325F});
  assert(engine.Submit(MakeObservation(1, 1, 0, 0, 0, floor)).accepted());
  assert(engine.WaitUntilProcessed(1, 1, std::chrono::seconds(2)));
  const auto first = engine.GetSnapshot();
  assert(first.voxel_cloud.layout == CloudLayout::kXyzF32SoA);
  assert(std::count_if(first.ground_surface.height.data.begin(), first.ground_surface.height.data.end(),
      [](float z) { return std::isfinite(z) && std::abs(z+0.325F) < 0.03F; }) > 4);
  assert(std::count(first.surface_projection.data.begin(), first.surface_projection.data.end(), 0.0F) > 4);
  assert(engine.Submit(MakeObservation(2, 1, 0, 0, 0, {0.525F, 0.025F, 0.5F})).accepted());
  assert(engine.WaitUntilProcessed(2, 1, std::chrono::seconds(2)));
  const auto reset = engine.GetSnapshot();
  assert(std::none_of(reset.ground_surface.height.data.begin(), reset.ground_surface.height.data.end(),
      [](float z) { return std::isfinite(z); }));
  engine.Stop();
}

void TestGroundModelIsIndependentOfDisplayPointCap() {
  Config config = TestConfig();
  config.voxel.voxel_size_m = 0.05F;
  config.max_voxel_snapshot_points = 64U;
  LiveMapEngine capped(config);
  config.max_voxel_snapshot_points = 1000U;
  LiveMapEngine complete(config);
  capped.Start();
  complete.Start();

  std::vector<float> floor;
  for (int r = 0; r < 8; ++r) for (int c = 0; c < 8; ++c)
    floor.insert(floor.end(), {1.025F+c*0.05F, -0.375F+r*0.05F, -0.325F});
  std::vector<float> wall;
  for (int r = 0; r < 8; ++r) for (int z = 0; z < 17; ++z)
    wall.insert(wall.end(), {0.325F, 0.025F+r*0.05F, 0.125F+z*0.05F});

  for (auto* engine : {&capped, &complete}) {
    assert(engine->Submit(MakeObservation(1, 1, 0, 0, 0, floor)).accepted());
    assert(engine->WaitUntilProcessed(1, 1, std::chrono::seconds(2)));
    const auto initial = engine->GetSnapshot();
    assert(std::count(initial.surface_projection.data.begin(),
                      initial.surface_projection.data.end(), 0.0F) >= 3);
    // Later, nearer wall returns exceed the display budget without moving the
    // rolling window or ray-clearing the earlier floor in the opposite sector.
    assert(engine->Submit(MakeObservation(1, 2, 0, 0, 0, wall)).accepted());
    assert(engine->WaitUntilProcessed(1, 2, std::chrono::seconds(2)));
  }
  const auto small = capped.GetView(SnapshotDetail::kComplete);
  const auto large = complete.GetView(SnapshotDetail::kComplete);
  assert(small.state.voxel_cells == large.state.voxel_cells);
  assert(small.state.voxel_cells == (floor.size()+wall.size())/3U);
  assert(small.snapshot.voxel_cloud.point_count == 64U);
  assert(large.snapshot.voxel_cloud.point_count == large.state.voxel_cells);
  assert(small.state.voxel_snapshot_omitted_cells == small.state.voxel_cells-64U);
  assert(std::all_of(small.snapshot.voxel_cloud.x.begin(),
                    small.snapshot.voxel_cloud.x.end(), [](float x) { return x < 0.4F; }));

  const auto& a = small.snapshot.ground_surface.height;
  const auto& b = large.snapshot.ground_surface.height;
  assert(lingtu::maps::layers::sameGeometry(a, b));
  for (std::size_t i = 0; i < a.data.size(); ++i)
    assert((std::isnan(a.data[i]) && std::isnan(b.data[i])) ||
           std::abs(a.data[i]-b.data[i]) < 1e-6F);
  assert(small.snapshot.surface_projection.data == large.snapshot.surface_projection.data);
  assert(std::count(small.snapshot.surface_projection.data.begin(),
                    small.snapshot.surface_projection.data.end(), 0.0F) >= 3);
  capped.Stop();
  complete.Stop();
}

}  // namespace

int main() {
  TestGroundModelIsIndependentOfDisplayPointCap();
  TestGroundModelConsumesSoaSnapshotAndResetsWithEpoch();
  TestExactPoseTransformAndDerivedLayers();
  TestScanOccupancyReceivesPointsBeforeGenericMapFilters();
  TestIdentityGateAndEpochReset();
  TestIndependentDecayWithoutNewObservations();
  TestMalformedObservationRejectedBeforeQueue();
  TestUnsafePoseStateRejected();
  TestVoxelSnapshotHasHardPointAndRoiBounds();
  TestColumnCarvingDoesNotClearAdjacentFloorHeight();
  TestSurfaceRetentionAndCapacityRecovery();
  TestSurfaceHitsDoNotClearUnobservedHeights();
  TestRuntimeMapCapacityLimitsFailClosed();
  TestRayClearedSurfaceDoesNotLeaveGhosts();
  TestCollisionSnapshotCompletenessAndAabb();
  TestCollisionSnapshotKeepsNearbyGroundInNearbyCells();
  TestRealtimeAndCompleteSnapshotsAreBuiltOnDemand();
  TestCollisionOnlyRuntimeSkipsExtendedLayers();
  TestDefaultOccupancyDoesNotDecayOutsideOfficialRayUpdates();
  TestOccupancyProjectionUsesPhysicalSensorHeight();
  TestOccupancyKeepsDoublePrecisionThroughPoseTransform();
  return 0;
}
