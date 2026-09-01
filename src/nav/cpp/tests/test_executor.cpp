#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "navigation/executor.hpp"

namespace {

lingtu::nav::navigation::Executor makeConfiguredExecutor(
    lingtu::nav::navigation::ExecutorConfig config,
    nav_kernel::LocalPlannerParams planner_params, const char *path_library) {
  nav_kernel::local::Planner planner(planner_params);
  EXPECT_TRUE(planner.configure(path_library));
  return lingtu::nav::navigation::Executor(std::move(config), std::move(planner));
}

lingtu::nav::navigation::Executor makeLoop(bool use_traversability = false,
                                            double corridor_lookahead_m = 2.0,
                                            double teleop_max_deviation_deg = 55.0,
                                            int debug_candidate_limit = 0) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  config.teleop_intent_max_deviation_deg = teleop_max_deviation_deg;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = use_traversability;
  planner.traversabilityNearFieldStop = use_traversability;
  planner.debugCandidateLimit = debug_candidate_limit;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  return makeConfiguredExecutor(std::move(config), planner, LINGTU_TEST_PATH_LIBRARY);
}

lingtu::nav::navigation::Executor makeCmuTeleopAvoidLoop(
    bool use_traversability = false, double teleop_horizon_m = 2.0) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.teleop_intent_horizon_m = teleop_horizon_m;
  config.teleop_intent_max_deviation_deg = 55.0;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = use_traversability;
  planner.traversabilityNearFieldStop = use_traversability;
  planner.vehicleLength = 1.0;
  planner.vehicleWidth = 0.6;
  planner.footprintPadding = 0.15;
  planner.nearFieldStopDis = 0.5;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  return makeConfiguredExecutor(std::move(config), planner, LINGTU_TEST_PATH_LIBRARY);
}

lingtu::nav::navigation::Executor makeScanExecutor(double corridor_lookahead_m = 3.0,
                                                   double vehicle_length_m = 0.6) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  planner.backend = nav_kernel::LocalPlannerBackend::Scan;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = false;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  planner.vehicleLength = vehicle_length_m;
  planner.scan.voxelResolution = 0.10;
  planner.scan.horizontalRange = 4.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 2.0;
  config.follower.nominalDt = 0.05;
  return makeConfiguredExecutor(std::move(config), planner, "");
}

lingtu::nav::navigation::Executor makeRecoveryDisabledLoop() {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  planner.checkObstacle = false;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = true;
  planner.traversabilityNearFieldStop = false;
  planner.traversabilityHardCost = 90.0;
  config.recovery.blocked_interval_s = 0.0;
  config.recovery.max_attempts = 0;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  return makeConfiguredExecutor(std::move(config), planner, LINGTU_TEST_PATH_LIBRARY);
}
lingtu::nav::navigation::Executor makeRecoverySafetyLoop(double no_progress_timeout_s = 2.0,
                                                          int max_attempts = 3,
                                                          double near_field_stop_dis = 0.5) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = true;
  planner.traversabilityNearFieldStop = false;
  planner.traversabilityHardCost = 90.0;
  planner.dirThre = 20.0;
  config.recovery.blocked_interval_s = 0.0;
  config.recovery.rotation_timeout_s = 0.1;
  config.recovery.translation_timeout_s = no_progress_timeout_s;
  config.recovery.max_attempts = max_attempts;
  planner.nearFieldStopDis = near_field_stop_dis;
  planner.vehicleLength = 0.80;
  planner.vehicleWidth = 0.60;
  planner.footprintPadding = 0.27;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  return makeConfiguredExecutor(std::move(config), planner, LINGTU_TEST_PATH_LIBRARY);
}

std::vector<float> makeRotationOnlyObservedGrid(int rows, int cols, double resolution,
                                                double origin) {
  std::vector<float> grid(rows * cols, 95.0f);
  for (int row = 0; row < rows; ++row) {
    const double y = origin + (static_cast<double>(row) + 0.5) * resolution;
    for (int col = 0; col < cols; ++col) {
      const double x = origin + (static_cast<double>(col) + 0.5) * resolution;
      if (std::abs(x) <= 0.84 && std::abs(y) <= 0.84) {
        grid[row * cols + col] = 0.0f;
      }
    }
  }
  return grid;
}

std::vector<float> makeRotationOnlyObstacleRing() {
  std::vector<float> obstacle_xyzh;
  for (int degrees = -180; degrees < 180; degrees += 5) {
    const double angle = degrees * M_PI / 180.0;
    obstacle_xyzh.push_back(static_cast<float>(1.10 * std::cos(angle)));
    obstacle_xyzh.push_back(static_cast<float>(1.10 * std::sin(angle)));
    obstacle_xyzh.push_back(0.0f);
    obstacle_xyzh.push_back(1.0f);
  }
  return obstacle_xyzh;
}

int obstacleCount(const std::vector<float> &obstacle_xyzh) {
  return static_cast<int>(obstacle_xyzh.size() / 4);
}

lingtu::nav::navigation::ExecutionObservation observation(std::uint64_t frame_epoch,
                                                          std::uint64_t cloud_generation,
                                                          std::uint64_t traversability_generation,
                                                          double odom_stamp_s, double cloud_stamp_s,
                                                          double traversability_stamp_s) {
  lingtu::nav::navigation::ExecutionObservation view;
  view.frame_epoch = frame_epoch;
  view.cloud_generation = cloud_generation;
  view.traversability_generation = traversability_generation;
  view.odom_stamp_s = odom_stamp_s;
  view.cloud_stamp_s = cloud_stamp_s;
  view.traversability_stamp_s = traversability_stamp_s;
  return view;
}

lingtu::nav::navigation::ExecutionObservation emptyScanObservation(double stamp_s,
                                                                   std::uint64_t generation = 1) {
  auto view = observation(1, generation, 0, stamp_s, stamp_s, 0.0);
  static thread_local lingtu::nav::tests::CollisionBitmap bitmap(
      {-10.0, -10.0, -2.0}, {10.0, 10.0, 2.0}, 0.10);
  bitmap.clear();
  view.collision = bitmap.view(stamp_s, generation);
  return view;
}

void setScanCollision(lingtu::nav::navigation::ExecutionObservation &observation,
                      const std::vector<float> &xyz) {
  // ponytail: executor copies the view on submission, so serial gtest needs one owner.
  static thread_local lingtu::nav::tests::CollisionBitmap bitmap;
  const auto previous = observation.collision;
  bitmap = lingtu::nav::tests::CollisionBitmap(
      previous.aabbMin, previous.aabbMax, previous.resolution);
  bitmap.occupyInflated(xyz, 0.40, 0.10, 0.10);
  observation.collision = bitmap.view(previous.stampS, previous.generation);
  observation.collision.resetEpoch = previous.resetEpoch;
  observation.collision.observationSequence = previous.observationSequence;
  observation.collision.receiveStampS = previous.receiveStampS;
  observation.collision.complete = previous.complete;
  observation.collision.live = previous.live;
}

void expectObservationWaitStopped(const lingtu::nav::navigation::ExecutionOutput &out) {
  EXPECT_EQ(out.reason, "recovery_observation_wait");
  EXPECT_EQ(out.recovery_reason, "recovery_observation_wait");
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}
nav_kernel::Pose pose(double x, double y, double z, double yaw) {
  nav_kernel::Pose p;
  p.position = {x, y, z};
  p.yaw = yaw;
  return p;
}

lingtu::nav::navigation::Route route(
    std::vector<nav_kernel::Vec3> points, std::optional<double> final_yaw = std::nullopt,
    std::optional<double> goal_tolerance_m = std::nullopt,
    std::optional<double> yaw_tolerance_rad = std::nullopt) {
  return {points, final_yaw, goal_tolerance_m, yaw_tolerance_rad};
}

lingtu::nav::navigation::ExecutionInput routeInput(
    const nav_kernel::Pose &body, const float *obstacle_xyzh, int obstacle_count,
    double timestamp_s,
    lingtu::nav::navigation::TraversabilityGridView traversability = {},
    lingtu::nav::navigation::ExecutionObservation observation = {}) {
  lingtu::nav::navigation::ExecutionInput input;
  input.mapBody = body;
  input.odomBody = body;
  input.obstacleXyzhMap = obstacle_xyzh;
  input.obstacleCount = obstacle_count;
  input.timestampS = timestamp_s;
  input.traversability = traversability;
  input.observation = observation;
  return input;
}

lingtu::nav::navigation::ExecutionInput intentInput(
    const nav_kernel::Pose &body, const nav_kernel::Twist &intent,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
    lingtu::nav::navigation::TraversabilityGridView traversability = {},
    lingtu::nav::navigation::ExecutionObservation observation = {}) {
  auto input = routeInput(body, obstacle_xyzh, obstacle_count, timestamp_s, traversability,
                          observation);
  input.mode = lingtu::nav::navigation::ExecutionMode::MotionIntent;
  input.motionIntent = intent;
  return input;
}

lingtu::nav::navigation::ExecutionInput odomInput(
    const nav_kernel::Pose &map_body, const nav_kernel::Pose &odom_body,
    const lingtu::nav::navigation::MapFromOdomTransform &map_from_odom,
    const float *obstacle_xyzh_map, int obstacle_count, double timestamp_s,
    lingtu::nav::navigation::TraversabilityGridView traversability = {},
    lingtu::nav::navigation::ExecutionObservation observation = {}) {
  auto input = routeInput(map_body, obstacle_xyzh_map, obstacle_count, timestamp_s,
                          traversability, observation);
  input.odomBody = odom_body;
  input.mapFromOdom = map_from_odom;
  return input;
}

template <typename Tick>
lingtu::nav::navigation::ExecutionOutput awaitScanOutput(Tick tick) {
  auto output = tick();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         (output.reason == "local_plan_pending" || output.reason == "local_plan_stale" ||
          output.reason == "local_intent_pending" || output.reason == "local_intent_stale" ||
          output.reason == "scan_trajectory_expired" ||
          output.reason == "teleop_assist_pending")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output = tick();
  }
  return output;
}

}  // namespace

TEST(Executor, PlansLocalPathAndCmdVelFromGlobalPath) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  const auto next = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.02));

  EXPECT_TRUE(out.active);
  EXPECT_FALSE(out.goal_reached);
  EXPECT_TRUE(out.path_found);
  EXPECT_FALSE(out.near_field_stop);
  EXPECT_GE(out.local_path_body.size(), 2u);
  EXPECT_EQ(out.local_path_body.size(), out.local_path_map.size());
  EXPECT_GT(next.cmd_vel.vx, 0.0);
}

TEST(Executor, SuspendAutonomyRetainsGoalAndResetsStallTiming) {
  auto loop = makeLoop();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {3.0, 0.0, 0.0},
      },
      0.75));

  const auto moving = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  ASSERT_TRUE(moving.active);
  ASSERT_TRUE(moving.path_found);

  loop.suspendAutonomy();

  EXPECT_TRUE(loop.hasRoute());
  const auto resumed = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 100.0));
  EXPECT_TRUE(resumed.active);
  EXPECT_TRUE(resumed.path_found);
  EXPECT_EQ(resumed.recovery_state, 0)
      << "a pause must not carry stale stall/recovery timing into resume";
  EXPECT_FALSE(resumed.recovery_exhausted);
}

TEST(Executor, StopsWhenGoalReached) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.0},
  }));

  const auto out = loop.tick(routeInput(pose(0.2, 0.0, 0.0, 0.0), nullptr, 0, 1.0));

  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_EQ(out.target_index, 1u);
  EXPECT_DOUBLE_EQ(out.target.x, 0.2);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanUsesSeparatePlanarAndHeightGoalTolerances) {
  auto loop = makeScanExecutor();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.20},
      },
      std::nullopt, 0.10));

  loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  const auto reached = loop.tick(routeInput(pose(0.95, 0.0, 0.0, 0.0), nullptr, 0, 1.1));

  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
  EXPECT_EQ(reached.reason, "goal_reached");
}

TEST(Executor, ScanAnchorsRouteHeight) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({
      {0.0, 0.0, 0.30},
      {1.0, 1.0, 0.50},
      {4.0, 1.0, 0.50},
  }));
  (void)executor.tick(routeInput(pose(0.0, 0.0, 0.30, 0.0), nullptr, 0, 1.0, {},
                       emptyScanObservation(1.0)));
  const auto observation = emptyScanObservation(2.0, 2);

  const auto output = awaitScanOutput([&]() {
    return executor.tick(routeInput(pose(1.05, 1.0, 0.30, 0.0), nullptr, 0, 2.0, {}, observation));
  });

  EXPECT_TRUE(output.path_found) << output.reason;
  EXPECT_NE(output.reason, "boundary_hypothesis_failed");
}

TEST(Executor, UsesPerLegInspectionArrivalTolerance) {
  auto loop = makeLoop();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0},
      },
      std::nullopt, 0.1));

  const auto outside = loop.tick(routeInput(pose(0.7, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  EXPECT_TRUE(outside.active);
  EXPECT_FALSE(outside.goal_reached);

  const auto inside = loop.tick(routeInput(pose(0.92, 0.0, 0.0, 0.0), nullptr, 0, 1.1));
  EXPECT_FALSE(inside.active);
  EXPECT_TRUE(inside.goal_reached);
}

TEST(Executor, UsesPerLegInspectionYawTolerance) {
  auto loop = makeLoop();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      1.0, 0.35, 0.5));

  const auto reached = loop.tick(routeInput(pose(0.2, 0.0, 0.0, 0.6), nullptr, 0, 1.0));
  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
}

TEST(Executor, TeleopIntentPlansWithoutGlobalPathAndHonorsRequestedSpeed) {
  auto loop = makeLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.25;

  lingtu::nav::navigation::ExecutionOutput out;
  for (int i = 0; i < 4; ++i) {
    out = loop.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0 + 0.05 * i));
  }

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found);
  EXPECT_GE(out.local_path_body.size(), 2u);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(Executor, ScanTeleopIntentPublishesTelemetryAndTracksSpline) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  const auto observation = emptyScanObservation(1.0);

  const auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found) << out.reason;
  EXPECT_GE(out.local_path_body.size(), 2U);
  EXPECT_GE(out.local_planner_debug.trajectoryPointCount, 2);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.reason, "teleop_assist_spline_ready");
}

TEST(Executor, ScanLateralIntentUsesOfficialHeadingAlignment) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vy = 0.25;
  const auto observation = emptyScanObservation(1.0);

  const auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     1.0, {}, observation));
  });

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.hold_body_heading);
  EXPECT_TRUE(out.trajectory_frozen);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-6);
  EXPECT_NEAR(out.cmd_vel.vy, 0.0, 1e-6);
  EXPECT_GT(out.cmd_vel.wz, 0.0);
  EXPECT_LE(out.target_distance_m, 0.6 + 1e-6);
}

TEST(Executor, ScanDiagonalIntentUsesOfficialWorldVelocityControl) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  intent.vy = 0.25;
  const auto observation = emptyScanObservation(1.0);

  const auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     1.0, {}, observation));
  });

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.hold_body_heading);
  EXPECT_FALSE(out.trajectory_frozen);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_GT(out.cmd_vel.vy, 0.0);
  EXPECT_GT(out.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanReverseUsesOfficialHeadingAlignment) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = -0.25;
  const auto observation = emptyScanObservation(1.0);

  const auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     1.0, {}, observation));
  });

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.hold_body_heading);
  EXPECT_TRUE(out.trajectory_frozen);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-6);
  EXPECT_NEAR(out.cmd_vel.vy, 0.0, 1e-6);
  EXPECT_GT(std::abs(out.cmd_vel.wz), 0.0);
}

TEST(Executor, ScanDirectionChangeNeverExecutesThePreviousIntentSpline) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist forward;
  forward.vx = 0.25;
  auto observation = emptyScanObservation(1.0);
  const auto moving = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), forward, nullptr, 0,
                                     1.0, {}, observation));
  });
  ASSERT_TRUE(moving.path_found) << moving.reason;
  ASSERT_GT(moving.cmd_vel.vx, 0.0);

  nav_kernel::Twist lateral;
  lateral.vy = 0.25;
  const auto transition = executor.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.0), lateral, nullptr, 0, 1.05, {}, observation));

  EXPECT_LE(std::abs(transition.cmd_vel.vx), 1e-9)
      << "a new intent must not execute the previous forward spline";

  observation = emptyScanObservation(1.05, 2);
  const auto switched = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), lateral, nullptr, 0,
                                     1.05, {}, observation));
  });
  ASSERT_TRUE(switched.path_found) << switched.reason;
  EXPECT_FALSE(switched.hold_body_heading);
  EXPECT_TRUE(switched.trajectory_frozen);
  EXPECT_NEAR(switched.cmd_vel.vx, 0.0, 1e-6);
  EXPECT_NEAR(switched.cmd_vel.vy, 0.0, 1e-6);
  EXPECT_GT(switched.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanOfficialControllerFreezesTrajectoryClockWhileAligningHeading) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 2.0, 0.0},
      {0.0, 3.0, 0.0},
  }));
  const auto observation = emptyScanObservation(1.0);

  double now = 1.0;
  const auto first = awaitScanOutput(
      [&]() { return executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now, {}, observation)); });
  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_GE(first.local_planner_debug.trajectoryPointCount, 2);
  ASSERT_TRUE(first.trajectory_frozen);
  EXPECT_DOUBLE_EQ(first.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(first.cmd_vel.vy, 0.0);
  EXPECT_GT(first.cmd_vel.wz, 0.0);

  now += 0.05;
  auto aligned_observation = emptyScanObservation(now);
  auto aligned = executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.5 * M_PI), nullptr, 0, now, {},
                                aligned_observation));
  for (int tick = 0; tick < 20 && aligned.cmd_vel.vx <= 0.0; ++tick) {
    EXPECT_GE(aligned.cmd_vel.vx, 0.0) << "tick=" << tick;
    now += 0.05;
    aligned_observation = emptyScanObservation(now);
    aligned = executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.5 * M_PI), nullptr, 0, now, {},
                             aligned_observation));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(aligned.path_found) << aligned.reason;
  EXPECT_FALSE(aligned.trajectory_frozen);
  EXPECT_GT(aligned.cmd_vel.vx, 0.0);
}

TEST(Executor, ScanTeleopIntentReceivesCollisionSnapshot) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  const std::vector<float> occupied_xyz{1.0F, 0.0F, 0.0F};
  lingtu::nav::navigation::ExecutionObservation observation;
  observation.frame_epoch = 1;
  observation.cloud_generation = 1;
  lingtu::nav::tests::CollisionBitmap collision(
      {-4.0, -4.0, -1.0}, {4.0, 4.0, 1.0}, 0.1);
  collision.occupyPoints(occupied_xyz);
  observation.collision = collision.view();
  observation.collision.complete = false;

  const auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });

  EXPECT_FALSE(out.path_found);
  EXPECT_EQ(out.reason, "collision_map_incomplete");
}

TEST(Executor, ScanKeepsSafeIntentOnMapChange) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  observation.collision.observationSequence = 2;
  observation.collision.generation = 2;
  observation.collision.stampS = 1.05;
  observation.collision.receiveStampS = 1.05;
  const auto during_replan =
      executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(during_replan.path_found) << during_replan.reason;
  EXPECT_FALSE(during_replan.near_field_stop) << during_replan.reason;
  EXPECT_NE(during_replan.reason, "local_intent_pending");
  EXPECT_GT(during_replan.cmd_vel.vx, 0.0);
}

TEST(Executor, ScanKeepsSafeIntentAcrossBodyHeightOscillation) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(
        intentInput(pose(0.0, 0.0, 0.34, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  observation = emptyScanObservation(1.05, 2);
  const auto during_replan = executor.tick(
      intentInput(pose(0.0, 0.0, 0.39, 0.0), intent, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(during_replan.path_found) << during_replan.reason;
  EXPECT_NE(during_replan.reason, "local_intent_pending");
}

TEST(Executor, ScanKeepsSafeIntentAcrossMillimeterPoseNoise) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(
        intentInput(pose(0.0, 0.0, 0.34, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  const std::vector<float> occupied_xyz{0.0F, 0.65F, 0.34F};
  observation = emptyScanObservation(1.05, 2);
  setScanCollision(observation, occupied_xyz);
  const auto during_replan = executor.tick(
      intentInput(pose(0.0, -0.006, 0.34, 0.0), intent, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(during_replan.path_found) << during_replan.reason;
  EXPECT_NE(during_replan.reason, "local_intent_pending");
}

TEST(Executor, ScanKeepsSafeIntentWhileBodyTurnsAlongDetour) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(
        intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  const auto turning = executor.tick(
      intentInput(pose(0.0, 0.0, 0.0, -0.20), intent, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(turning.path_found) << turning.reason;
  EXPECT_FALSE(turning.near_field_stop) << turning.reason;
  EXPECT_NE(turning.reason, "local_intent_pending");
}

TEST(Executor, ScanKeepsSafeIntentWhileReturningToTeleopCorridor) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  const auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(
        intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  const auto detouring = executor.tick(
      intentInput(pose(0.2, 0.8, 0.0, -0.20), intent, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(detouring.path_found) << detouring.reason;
  EXPECT_FALSE(detouring.near_field_stop) << detouring.reason;
  EXPECT_NE(detouring.reason, "local_intent_pending");
}

TEST(Executor, ScanReplacesBlockedSplineAfterInflatedMapChanges) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto ready = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  std::vector<float> occupied_xyz;
  for (int x = 4; x <= 8; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -3; z <= 3; ++z) {
        occupied_xyz.push_back(0.10F * static_cast<float>(x));
        occupied_xyz.push_back(0.10F * static_cast<float>(y));
        occupied_xyz.push_back(0.10F * static_cast<float>(z));
      }
    }
  }
  observation.cloud_generation = 2;
  observation.cloud_stamp_s = 1.05;
  setScanCollision(observation, occupied_xyz);
  observation.collision.observationSequence = 2;
  observation.collision.generation = 2;
  observation.collision.stampS = 1.05;
  observation.collision.receiveStampS = 1.05;

  auto updated = executor.tick(
      intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.05, {}, observation));
  const auto path_is_safe = [&](const auto &output) {
    for (std::size_t index = 0; index < output.local_path_body.size(); ++index) {
      const auto &point = output.local_path_body[index];
      const auto &next = output.local_path_body[
          std::min(index + 1U, output.local_path_body.size() - 1U)];
      const double yaw = std::atan2(next.y - point.y, next.x - point.x);
      for (const double sign : {-1.0, 1.0}) {
        const nav_kernel::Vec3 cylinder{
            point.x + sign * 0.25 * std::cos(yaw),
            point.y + sign * 0.25 * std::sin(yaw), point.z};
        if (observation.collision.occupied(cylinder))
          return false;
      }
    }
    return true;
  };
  for (int tick = 1; tick <= 500 && updated.path_found &&
                     !path_is_safe(updated); ++tick) {
    const double now = 1.05 + 0.01 * static_cast<double>(tick);
    updated = executor.tick(
        intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, now, {}, observation));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_TRUE(!updated.path_found || path_is_safe(updated)) << updated.reason;
}

TEST(Executor, CmuRotatesWhenNoPath) {
  auto loop = makeCmuTeleopAvoidLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.3;
  std::vector<float> obstacle_xyzh;
  for (int i = -2; i <= 2; ++i) {
    obstacle_xyzh.push_back(0.90f);
    obstacle_xyzh.push_back(0.08f * static_cast<float>(i));
    obstacle_xyzh.push_back(0.0f);
    obstacle_xyzh.push_back(1.0f);
  }

  const auto out = loop.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0));

  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_GT(std::abs(out.cmd_vel.wz), 0.01);
  EXPECT_FALSE(out.hold_body_heading);
  EXPECT_EQ(out.reason, out.recovery_reason);
  ASSERT_TRUE(out.recovery_verified) << out.recovery_reason;
  ASSERT_EQ(out.recovery_action,
            static_cast<int>(nav_kernel::RecoveryAction::Rotate));
  ASSERT_GT(std::abs(out.recovery_rotation_target_rad), 0.10);

  const double selected_yaw = out.recovery_rotation_target_rad;
  const auto rotating = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.5 * selected_yaw), intent,
      obstacle_xyzh.data(), static_cast<int>(obstacle_xyzh.size() / 4), 1.05));
  EXPECT_DOUBLE_EQ(rotating.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(rotating.cmd_vel.vy, 0.0);
  EXPECT_EQ(std::signbit(rotating.cmd_vel.wz), std::signbit(out.cmd_vel.wz));
  EXPECT_NEAR(rotating.recovery_rotation_target_rad, selected_yaw, 1e-9)
      << "the selected yaw must stay latched while rotation is in progress";

  const auto completed = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, selected_yaw), intent, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.1));
  EXPECT_TRUE(completed.recovery_observation_refresh_required)
      << completed.recovery_reason;
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.wz, 0.0);

  const auto replanned = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, selected_yaw), intent, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.2));
  EXPECT_TRUE(replanned.path_found) << replanned.reason;
  EXPECT_FALSE(replanned.near_field_stop) << replanned.reason;
  EXPECT_EQ(replanned.reason, "teleop_assist_control_ready");

  const auto resumed = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, selected_yaw), intent, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.21));
  EXPECT_TRUE(resumed.path_found) << resumed.reason;
  EXPECT_GT(resumed.cmd_vel.vx, 0.01) << resumed.reason;
}

TEST(Executor, ReverseOverridesBlockedState) {
  auto loop = makeCmuTeleopAvoidLoop();
  std::vector<float> obstacle_xyzh;
  for (int i = -3; i <= 3; ++i) {
    obstacle_xyzh.insert(obstacle_xyzh.end(),
                         {0.60f, 0.06f * static_cast<float>(i), 0.0f, 1.0f});
  }

  nav_kernel::Twist forward;
  forward.vx = 0.30;
  const auto blocked = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.0), forward, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.0));
  EXPECT_FALSE(blocked.path_found);
  EXPECT_DOUBLE_EQ(blocked.cmd_vel.vx, 0.0);

  nav_kernel::Twist reverse;
  reverse.vx = -0.30;
  const auto selected = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.0), reverse, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.05));
  const auto retreat = loop.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.0), reverse, obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4), 1.06));

  EXPECT_TRUE(selected.path_found) << selected.reason;
  EXPECT_TRUE(retreat.path_found) << retreat.reason;
  EXPECT_FALSE(retreat.near_field_stop) << retreat.reason;
  EXPECT_EQ(retreat.recovery_state, 0) << retreat.reason;
  EXPECT_LT(retreat.cmd_vel.vx, 0.0) << retreat.reason;
}

TEST(Executor, TeleopIntentHardLimitsSelectedPathEndDirection) {
  auto loop = makeLoop(false, 2.0, 5.0);
  nav_kernel::Twist intent;
  intent.vx = 0.3;
  std::vector<float> obstacle_xyzh;
  for (int i = -2; i <= 2; ++i) {
    obstacle_xyzh.push_back(0.9f);
    obstacle_xyzh.push_back(0.08f * static_cast<float>(i));
    obstacle_xyzh.push_back(0.0f);
    obstacle_xyzh.push_back(1.0f);
  }

  const auto out = loop.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0));

  if (out.path_found) {
    ASSERT_GE(out.local_path_body.size(), 2U);
    const auto &before_end = out.local_path_body[out.local_path_body.size() - 2];
    const auto &end = out.local_path_body.back();
    const double end_direction_deg =
        std::atan2(end.y - before_end.y, end.x - before_end.x) * 180.0 / M_PI;
    EXPECT_LE(std::abs(end_direction_deg), 5.0 + 1e-6);
  } else {
    EXPECT_EQ(out.cmd_vel.vx, 0.0);
    EXPECT_EQ(out.cmd_vel.vy, 0.0);
    EXPECT_GT(std::abs(out.cmd_vel.wz), 0.01);
    EXPECT_TRUE(out.recovery_verified);
    EXPECT_EQ(out.recovery_action,
              static_cast<int>(nav_kernel::RecoveryAction::Rotate));
  }
}

TEST(Executor, TeleopIntentRotatesBeforeRetryingWhenTranslationIsSurrounded) {
  auto loop = makeLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.3;
  std::vector<float> obstacle_xyzh;
  for (int degrees = -180; degrees < 180; degrees += 5) {
    const double angle = degrees * M_PI / 180.0;
    obstacle_xyzh.push_back(static_cast<float>(0.8 * std::cos(angle)));
    obstacle_xyzh.push_back(static_cast<float>(0.8 * std::sin(angle)));
    obstacle_xyzh.push_back(0.0f);
    obstacle_xyzh.push_back(1.0f);
  }

  const auto out = loop.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0));

  EXPECT_FALSE(out.path_found);
  EXPECT_EQ(out.reason, "recovery_rotation_active");
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_GT(std::abs(out.cmd_vel.wz), 0.01);
  EXPECT_EQ(out.recovery_state, 1);
  EXPECT_TRUE(out.recovery_verified);
}

TEST(Executor, DisabledRecoveryKeepsGoalActiveForTheNextLocalReplan) {
  auto loop = makeRecoveryDisabledLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));
  std::vector<float> risk_grid(9 * 9, 95.0f);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 9, 9, 0.5, -2.0, -2.0, 1,
  };

  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability));
  const auto retry = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.1, traversability));

  EXPECT_TRUE(out.active);
  EXPECT_FALSE(out.path_found);
  EXPECT_FALSE(out.recovery_exhausted);
  EXPECT_EQ(out.recovery_state, 0);
  EXPECT_NE(out.reason, "local_recovery_exhausted");
  EXPECT_TRUE(out.local_path_body.empty());
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
  EXPECT_TRUE(retry.active);
  EXPECT_FALSE(retry.recovery_exhausted);
  EXPECT_NE(retry.reason, "local_recovery_exhausted");
}
TEST(Executor, RecoveryDoesNotBackUpThroughBlockedRearFootprint) {
  auto loop = makeRecoverySafetyLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  constexpr int kRows = 41;
  constexpr int kCols = 41;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.05;
  std::vector<float> grid(kRows * kCols, 95.0f);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 1,
  };

  const std::vector<float> obstacle_xyzh = {
      -0.90f, 0.48f, 0.0f, 1.0f, -0.90f, -0.48f, 0.0f, 1.0f,
  };
  (void)loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                  static_cast<int>(obstacle_xyzh.size() / 4), 1.0, traversability));
  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                             static_cast<int>(obstacle_xyzh.size() / 4), 1.2, traversability));

  for (const auto &point : out.local_path_body) {
    EXPECT_GE(point.x, -1e-6);
  }
  EXPECT_GE(out.cmd_vel.vx, -1e-6);
}
TEST(Executor, RecoverySelectsOnlyReachableLateralExit) {
  auto loop = makeRecoverySafetyLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -3.05;
  std::vector<float> grid(kRows * kCols, 95.0f);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (x >= -0.80 && x <= 0.80 && y >= -0.65 && y <= 1.80) {
        grid[row * kCols + col] = 0.0f;
      }
    }
  }
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 2,
  };

  (void)loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability));
  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability));

  ASSERT_GE(out.local_path_body.size(), 2u);
  EXPECT_EQ(out.recovery_state, 2);
  EXPECT_GT(out.local_path_body.back().y, 0.6);
  EXPECT_LT(std::abs(out.local_path_body.back().x), 0.25);
  EXPECT_GT(out.cmd_vel.vy, 0.0) << "reason=" << out.reason << " verified=" << out.recovery_verified
                                 << " progress=" << out.recovery_progress
                                 << " path_size=" << out.local_path_body.size() << " first_next=("
                                 << out.local_path_body[1].x << "," << out.local_path_body[1].y
                                 << ")";
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 0.05);
}

TEST(Executor, RecoveryProgressRequiresOdometryMovement) {
  auto loop = makeRecoverySafetyLoop(0.1, 1);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -3.05;
  std::vector<float> grid(kRows * kCols, 95.0f);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (x >= -0.80 && x <= 0.80 && y >= -0.65 && y <= 1.80) {
        grid[row * kCols + col] = 0.0f;
      }
    }
  }
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 3,
  };

  (void)loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability));
  const auto started = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability));
  ASSERT_EQ(started.recovery_state, 2);
  EXPECT_TRUE(started.recovery_verified);
  EXPECT_DOUBLE_EQ(started.recovery_progress, 0.0);
  EXPECT_GT(started.cmd_vel.vy, 0.0);

  const auto stalled = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.2, traversability));
  EXPECT_TRUE(stalled.recovery_exhausted);
  EXPECT_EQ(stalled.reason, "local_recovery_exhausted");
  EXPECT_DOUBLE_EQ(stalled.recovery_progress, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.wz, 0.0);
}

TEST(Executor, RecoveryWaitsForFreshCloudAndTraversabilityAfterRotationCompletes) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid = makeRotationOnlyObservedGrid(kRows, kCols, kResolution, kOrigin);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 10,
  };
  const std::vector<float> obstacle_xyzh = makeRotationOnlyObstacleRing();

  const auto base = observation(1, 10, 20, 1.00, 0.95, 0.96);
  const auto rotating = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.00, traversability, base));

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason << " recovery_reason=" << rotating.recovery_reason
      << " candidates=" << rotating.recovery_candidate_count;
  ASSERT_TRUE(rotating.recovery_verified);
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);
  EXPECT_FALSE(rotating.recovery_observation_refresh_required);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.20, traversability,
                                   observation(1, 10, 20, 1.20, 0.95, 0.96)));

  EXPECT_TRUE(completed.recovery_observation_refresh_required);
  EXPECT_EQ(completed.reason, "recovery_rotation_complete");
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.wz, 0.0);

  const auto unchanged = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.25, traversability,
                                   observation(1, 10, 20, 1.25, 0.95, 0.96)));
  expectObservationWaitStopped(unchanged);

  const auto cloud_only = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                    obstacleCount(obstacle_xyzh), 1.30, traversability,
                                    observation(1, 11, 20, 1.30, 1.30, 0.96)));
  expectObservationWaitStopped(cloud_only);

  const auto advanced_generation_stale_stamp = loop.tick(routeInput(
      pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.35,
      traversability, observation(1, 12, 21, 1.35, 1.20, 1.20)));
  expectObservationWaitStopped(advanced_generation_stale_stamp);

  const auto released = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.40, traversability,
                                  observation(1, 12, 21, 1.40, 1.31, 1.32)));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}

TEST(Executor, RecoveryObservationWaitRebasesOnFrameEpochChange) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid = makeRotationOnlyObservedGrid(kRows, kCols, kResolution, kOrigin);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 11,
  };
  const std::vector<float> obstacle_xyzh = makeRotationOnlyObstacleRing();

  const auto rotating =
      loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.00,
                traversability, observation(1, 10, 20, 1.00, 0.95, 0.96)));

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason << " recovery_reason=" << rotating.recovery_reason;
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.20, traversability,
                                   observation(1, 10, 20, 1.20, 0.95, 0.96)));
  ASSERT_TRUE(completed.recovery_observation_refresh_required);

  const auto rebased = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                 obstacleCount(obstacle_xyzh), 1.25, traversability,
                                 observation(2, 1, 1, 1.25, 1.25, 1.25)));
  expectObservationWaitStopped(rebased);

  const auto same_epoch_same_generation = loop.tick(routeInput(
      pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.30,
      traversability, observation(2, 1, 1, 1.30, 1.30, 1.30)));
  expectObservationWaitStopped(same_epoch_same_generation);

  const auto released = loop.tick(routeInput(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.35, traversability,
                                  observation(2, 2, 2, 1.35, 1.31, 1.32)));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}
TEST(Executor, AlignsRequestedYawBeforeReportingGoalReached) {
  auto loop = makeLoop();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      1.0));

  const auto aligning = loop.tick(routeInput(pose(0.2, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  EXPECT_TRUE(aligning.active);
  EXPECT_FALSE(aligning.goal_reached);
  EXPECT_EQ(aligning.reason, "aligning_goal_yaw");
  EXPECT_DOUBLE_EQ(aligning.cmd_vel.vx, 0.0);
  EXPECT_GT(aligning.cmd_vel.wz, 0.0);

  const auto reached = loop.tick(routeInput(pose(0.2, 0.0, 0.0, 0.98), nullptr, 0, 1.1));
  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
  EXPECT_EQ(reached.reason, "goal_reached");
  EXPECT_DOUBLE_EQ(reached.cmd_vel.wz, 0.0);
}

TEST(Executor, GoalYawAlignmentUsesShortestWrappedError) {
  auto loop = makeLoop();
  loop.setRoute(route(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      -3.05));

  const auto out = loop.tick(routeInput(pose(0.2, 0.0, 0.0, 3.10), nullptr, 0, 1.0));
  EXPECT_TRUE(out.active);
  EXPECT_EQ(out.reason, "aligning_goal_yaw");
  EXPECT_GT(out.cmd_vel.wz, 0.0);
  EXPECT_LT(std::abs(out.cmd_vel.wz), 0.25);
}

TEST(Executor, RelocalizationJumpSelectsPathAheadOfRobot) {
  auto loop = makeLoop();
  std::vector<nav_kernel::Vec3> path;
  for (int i = 0; i <= 100; ++i) {
    path.push_back({0.1 * static_cast<double>(i), 0.0, 0.0});
  }
  loop.setRoute(route(path));

  const auto out = loop.tick(routeInput(pose(8.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));

  EXPECT_GE(out.target_index, 80u);
  EXPECT_GE(out.target.x, 8.0);
}

TEST(Executor, ShortCorridorLookaheadDoesNotCutAcrossUpcomingCorner) {
  auto loop = makeLoop(false, 0.6);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.0},
      {0.4, 0.0, 0.0},
      {0.6, 0.0, 0.0},
      {0.8, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {1.0, 0.2, 0.0},
      {1.0, 0.4, 0.0},
  }));

  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));

  EXPECT_LE(out.target.x, 0.8);
  EXPECT_DOUBLE_EQ(out.target.y, 0.0);
}

TEST(Executor, RollingLocalPathDoesNotTriggerFinalGoalSlowdown) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {5.0, 0.0, 0.0},
  }));

  lingtu::nav::navigation::ExecutionOutput out;
  for (int i = 0; i < 6; ++i) {
    out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0 + 0.05 * i));
  }

  EXPECT_NEAR(out.cmd_vel.vx, 0.5, 1e-6);
}

TEST(Executor, NearFieldObstacleBlocksCmdVel) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));
  const std::vector<float> obstacle_xyzh = {
      0.45f,
      0.0f,
      0.0f,
      1.0f,
  };

  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                             static_cast<int>(obstacle_xyzh.size() / 4), 1.0));

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Executor, SafeDetourOverridesStraightStop) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));
  const std::vector<float> obstacle_xyzh = {
      0.85f,
      0.0f,
      0.0f,
      1.0f,
  };

  const auto out =
      loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.0));

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.near_field_stop)
      << "a collision-checked local detour must supersede the blocked straight intent";
  EXPECT_EQ(out.reason, "control_ready");
  EXPECT_FALSE(out.trajectory_frozen);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_GT(std::abs(out.cmd_vel.wz), 1e-6);
}

TEST(Executor, KeepsDetourSideAcrossSmallObstacleJitter) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));
  const auto obstacle = [](float lateral_offset) {
    std::vector<float> points;
    for (int index = -3; index <= 3; ++index) {
      points.insert(points.end(), {
                                      0.90F,
                                      lateral_offset + 0.08F * static_cast<float>(index),
                                      0.0F,
                                      1.0F,
                                  });
    }
    return points;
  };
  const auto left_bias = obstacle(0.015F);
  const auto right_bias = obstacle(-0.015F);

  const auto first =
      loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), left_bias.data(), obstacleCount(left_bias), 1.0));
  const std::vector<float> transient_close_obstacle{
      0.45F,
      0.0F,
      0.0F,
      1.0F,
  };
  const auto stopped = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), transient_close_obstacle.data(),
                                 obstacleCount(transient_close_obstacle), 1.025));
  const auto second =
      loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), right_bias.data(), obstacleCount(right_bias), 1.05));

  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_TRUE(stopped.near_field_stop);
  ASSERT_TRUE(second.path_found) << second.reason;
  ASSERT_FALSE(first.local_path_body.empty());
  ASSERT_FALSE(second.local_path_body.empty());
  const double first_lateral = first.local_path_body.back().y;
  const double second_lateral = second.local_path_body.back().y;
  ASSERT_GT(std::abs(first_lateral), 0.1);
  ASSERT_GT(std::abs(second_lateral), 0.1);
  EXPECT_GT(first_lateral * second_lateral, 0.0)
      << "a safe committed detour must not switch sides on sensor jitter";
}

TEST(Executor, CommittedGuideDoesNotReplaceGlobalRouteBends) {
  auto loop = makeLoop(false, 3.0, 55.0, 1);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.3, -0.1, 0.0},
      {0.5, -0.3, 0.0},
      {0.7, -0.7, 0.0},
      {1.1, -0.9, 0.0},
      {1.5, -0.7, 0.0},
      {2.1, -0.3, 0.0},
      {3.0, 0.0, 0.0},
  }));

  const auto first = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  const auto second = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.05));

  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_TRUE(second.path_found) << second.reason;
  EXPECT_GT(second.local_planner_debug.relativeGoalDistanceM, 1.0)
      << "the committed local guide must augment, not truncate, the global route";
}

TEST(Executor, PassedLocalGuideNeverCommandsReverse) {
  auto loop = makeLoop(false, 3.0);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));

  const auto first = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0));
  ASSERT_TRUE(first.path_found) << first.reason;

  const auto advanced = loop.tick(routeInput(pose(2.5, 0.0, 0.0, 0.0), nullptr, 0, 1.05));
  ASSERT_TRUE(advanced.path_found) << advanced.reason;
  EXPECT_GE(advanced.cmd_vel.vx, 0.0)
      << "a consumed local guide must not pull the robot back toward old path points";
  ASSERT_FALSE(advanced.local_path_map.empty());
  EXPECT_GE(advanced.local_path_map.back().x, 2.5 - 1e-6);
}

TEST(Executor, AutonomousCmuTurnsInsteadOfDrivingBackward) {
  auto loop = makeLoop(false, 3.0);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {-1.0, 0.0, 0.0},
      {-3.0, 0.0, 0.0},
  }));

  loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.1));
  const auto output = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.15));
  EXPECT_GE(output.cmd_vel.vx, 0.0)
      << "autonomous CMU must use the Go2 forward-drive contract";
}

TEST(Executor, TraversabilityGridBlocksCmdVel) {
  auto loop = makeLoop(true);
  loop.setRoute(route({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  }));
  std::vector<float> risk_grid(5 * 5, 0.0f);
  risk_grid[2 * 5 + 1] = 95.0f;
  risk_grid[2 * 5 + 2] = 95.0f;
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 5, 5, 0.25, 0.0, -0.5,
  };

  const auto out = loop.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability));

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanRecoveryDoesNotRequireCmuPathLibrary) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));

  nav_kernel::Pose body;
  auto observation = emptyScanObservation(1.0);
  observation.traversability_generation = 1;
  observation.odom_stamp_s = 1.0;
  observation.cloud_stamp_s = 1.0;
  observation.traversability_stamp_s = 1.0;

  const auto moving =
      awaitScanOutput([&]() { return executor.tick(routeInput(body, nullptr, 0, 1.0, {}, observation)); });
  ASSERT_TRUE(moving.path_found) << moving.reason;
  EXPECT_EQ(moving.local_planner_debug.backend, nav_kernel::LocalPlannerBackend::Scan);

  observation.cloud_generation = 2;
  observation.odom_stamp_s = 3.1;
  observation.cloud_stamp_s = 3.1;
  observation.collision.observationSequence = 2;
  observation.collision.generation = 2;
  observation.collision.stampS = 3.1;
  observation.collision.receiveStampS = 3.1;
  const auto recovering =
      awaitScanOutput([&]() { return executor.tick(routeInput(body, nullptr, 0, 3.1, {}, observation)); });

  EXPECT_TRUE(recovering.recovery_verified) << recovering.reason;
  EXPECT_EQ(recovering.recovery_state, 2);
  EXPECT_EQ(recovering.recovery_reason, "recovery_translation_active");
  EXPECT_GE(recovering.local_path_body.size(), 2U);
  EXPECT_EQ(recovering.local_planner_debug.backend, nav_kernel::LocalPlannerBackend::Scan);
}

TEST(Executor, ScanKeepsSafeTrajectoryWhileNewCollisionGenerationPlans) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));

  nav_kernel::Pose body;
  auto observation = emptyScanObservation(1.0);
  observation.odom_stamp_s = 1.0;
  observation.cloud_stamp_s = 1.0;

  const auto initial =
      awaitScanOutput([&]() { return executor.tick(routeInput(body, nullptr, 0, 1.0, {}, observation)); });
  ASSERT_TRUE(initial.path_found) << initial.reason;

  observation.cloud_generation = 2;
  observation.odom_stamp_s = 1.05;
  observation.cloud_stamp_s = 1.05;
  observation.collision.observationSequence = 2;
  observation.collision.generation = 2;
  observation.collision.stampS = 1.05;
  observation.collision.receiveStampS = 1.05;

  const auto during_replan = executor.tick(routeInput(body, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(during_replan.path_found) << during_replan.reason;
  EXPECT_FALSE(during_replan.near_field_stop) << during_replan.reason;
  EXPECT_NE(during_replan.reason, "local_plan_pending");
}

TEST(Executor, ScanContinuesSafePrefixDuringReplan) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));

  nav_kernel::Pose body;
  auto observation = emptyScanObservation(1.0, 1U);
  const auto initial =
      awaitScanOutput([&]() { return executor.tick(routeInput(body, nullptr, 0, 1.0, {}, observation)); });
  ASSERT_TRUE(initial.path_found) << initial.reason;
  ASSERT_GT(initial.cmd_vel.vx, 0.0);

  std::vector<float> occupied_xyz{1.40F, 0.0F, 0.0F};
  observation = emptyScanObservation(1.05, 2U);
  setScanCollision(observation, occupied_xyz);

  const auto handoff = executor.tick(routeInput(body, nullptr, 0, 1.05, {}, observation));

  EXPECT_TRUE(handoff.path_found) << handoff.reason;
  EXPECT_FALSE(handoff.near_field_stop) << handoff.reason;
  EXPECT_NE(handoff.reason, "local_plan_pending");
  EXPECT_GT(handoff.cmd_vel.vx, 0.0);
}

TEST(Executor, ScanAcceptsSafeCompletionWhileCollisionMapKeepsAdvancing) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.8, 0.5, 0.0},
      {1.6, 0.5, 0.0},
      {2.4, 0.0, 0.0},
  }));

  nav_kernel::Pose body;
  auto current = emptyScanObservation(1.0, 1U);
  auto output = executor.tick(routeInput(body, nullptr, 0, 1.0, {}, current));
  for (std::uint64_t generation = 2U;
       generation < 500U && !output.path_found; ++generation) {
    const double timestamp = 1.0 + 0.002 * static_cast<double>(generation);
    current = emptyScanObservation(timestamp, generation);
    output = executor.tick(routeInput(body, nullptr, 0, timestamp, {}, current));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_TRUE(output.path_found) << output.reason;
  EXPECT_FALSE(output.near_field_stop);
}

TEST(Executor, ScanReferencePathPreservesEndpointAndElevation) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({
      {0.0, 0.0, 0.0},
      {0.8, 0.6, 0.15},
      {1.6, 0.6, 0.35},
      {2.4, 0.0, 0.55},
  }));
  auto observation = emptyScanObservation(1.0);
  observation.body_velocity_valid = true;

  const auto output = awaitScanOutput(
      [&]() { return executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, {}, observation)); });

  ASSERT_TRUE(output.path_found) << output.reason;
  ASSERT_GE(output.local_path_map.size(), 3U);
  ASSERT_GE(output.local_planner_debug.trajectoryPointCount, 3);
  const auto elevated = std::max_element(output.local_path_map.begin(), output.local_path_map.end(),
                                         [](const auto &a, const auto &b) { return a.z < b.z; });
  EXPECT_GT(elevated->z, 0.40);
  EXPECT_NEAR(output.target.x, 2.4, 1e-6);
  EXPECT_NEAR(output.target.y, 0.0, 1e-6);
  EXPECT_NEAR(output.target.z, 0.55, 1e-6);
  EXPECT_EQ(output.reason, "spline_control_ready");
}

TEST(Executor, ScanAnchorsGroundRouteHeightToRobotBody) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({
      {0.1, 0.1, 0.30},
      {1.0, 0.1, 0.30},
      {2.0, 0.1, 0.30},
      {3.0, 0.1, 0.30},
  }));
  const auto observation = emptyScanObservation(1.0);

  const auto output = awaitScanOutput([&]() {
    return executor.tick(routeInput(pose(0.0, 0.0, 0.48, 0.0), nullptr, 0, 1.0, {}, observation));
  });

  ASSERT_TRUE(output.path_found) << output.reason;
  EXPECT_EQ(output.reason, "spline_control_ready");
  ASSERT_GE(output.local_path_map.size(), 2U);
  for (const auto &point : output.local_path_map) {
    EXPECT_NEAR(point.z, 0.48, 1e-6);
  }
}

TEST(Executor, RouteHeightCalibrationIsStableAcrossProgress) {
  auto executor = makeLoop(false, 1.0);
  executor.setRoute(route({
      {0.0, 0.0, 0.30},
      {1.0, 0.0, 0.30},
      {2.0, 0.0, 0.60},
      {3.0, 0.0, 0.90},
      {4.0, 0.0, 0.90},
  }));

  const auto first =
      executor.tick(routeInput(pose(0.0, 0.0, 0.48, 0.0), nullptr, 0, 1.0));
  EXPECT_NEAR(first.target.z, 0.48, 1e-6);

  const auto later =
      executor.tick(routeInput(pose(2.0, 0.0, 0.48, 0.0), nullptr, 0, 1.1));
  EXPECT_GT(later.target.z, 1.0)
      << "route elevation must not be shifted back down when the robot has not climbed";
}

TEST(Executor, ScanProjectsOntoSparseRouteInsteadOfDrivingBackToWaypoint) {
  auto executor = makeScanExecutor(3.0);
  executor.setRoute(route({
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {10.0, 10.0, 0.0},
  }));
  const auto observation = emptyScanObservation(1.0);

  const auto output = awaitScanOutput([&]() {
    return executor.tick(routeInput(pose(5.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, {}, observation));
  });

  ASSERT_TRUE(output.path_found) << output.reason;
  EXPECT_NEAR(output.target.x, 8.0, 1e-6);
  EXPECT_NEAR(output.target.y, 0.0, 1e-6);
  ASSERT_GE(output.local_path_body.size(), 2U);
  for (const auto &point : output.local_path_body) {
    EXPECT_GE(point.x, -1e-6) << "the local route must not fold back to the sparse segment start";
  }
}

TEST(Executor, OdomLocalFrameUsesOdomRiskAndPublishesMapPaths) {
  auto loop = makeLoop(true, 2.0, 55.0, 1);
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute(route({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
      {10.0, -1.0, 0.0},
      {10.0, 0.0, 0.0},
  }));

  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;

  std::vector<float> risk_grid(9 * 9, 0.0f);
  risk_grid[4 * 9 + 6] = 95.0f;
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 9, 9, 0.25, -1.0, -1.0, 7,
  };

  const auto out = loop.tick(odomInput(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0),
                                 map_from_odom, nullptr, 0, 1.0, traversability));

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found);
  EXPECT_TRUE(out.near_field_stop)
      << "the risk cell is in odom, not at the map-frame vehicle position";
  EXPECT_EQ(out.target_index, 2u);
  EXPECT_NEAR(out.target.x, 10.0, 1e-9);
  EXPECT_NEAR(out.target.y, -1.0, 1e-9);
  ASSERT_EQ(out.local_path_body.size(), out.local_path_map.size());
  ASSERT_FALSE(out.local_path_body.empty());
  for (std::size_t index = 0; index < out.local_path_body.size(); ++index) {
    const auto &body = out.local_path_body[index];
    EXPECT_NEAR(out.local_path_map[index].x, 10.0 - body.y, 1e-9);
    EXPECT_NEAR(out.local_path_map[index].y, -3.0 + body.x, 1e-9);
    EXPECT_NEAR(out.local_path_map[index].z, body.z, 1e-9);
  }
  ASSERT_FALSE(out.local_planner_debug.candidates.empty());
  const auto &candidate = out.local_planner_debug.candidates.front();
  ASSERT_FALSE(candidate.path.empty());
  EXPECT_GT(candidate.path.front().x, 6.0)
      << "debug candidate path must be published in map, not odom";
}

TEST(Executor, OdomLocalFrameTransformsMapObstaclesIntoPlannerFrame) {
  auto loop = makeLoop();
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute(route({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
      {10.0, -1.0, 0.0},
  }));

  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;
  const std::vector<float> obstacle_xyzh_map = {
      10.0f,
      -2.55f,
      0.0f,
      1.0f,
  };

  const auto out =
      loop.tick(odomInput(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0), map_from_odom,
                    obstacle_xyzh_map.data(), obstacleCount(obstacle_xyzh_map), 1.0));

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop) << "the map-frame obstacle is 0.45 m ahead in odom";
  EXPECT_EQ(out.reason, "near_field_stop");
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Executor, OdomLocalFrameTransformsCompleteCollisionLayerAndAabb) {
  auto loop = makeScanExecutor();
  loop.setRoute(route({
      {10.0, -3.0, 0.0},
      {10.0, -2.2, 0.0},
      {10.0, -1.4, 0.0},
      {10.0, -0.6, 0.0},
  }));
  const double right_angle = 3.14159265358979323846 / 2.0;
  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;

  std::vector<float> occupied_map;
  for (int forward = 8; forward <= 16; ++forward) {
    for (int lateral = -11; lateral <= 11; ++lateral) {
      for (int vertical = -4; vertical <= 4; ++vertical) {
        const double x_odom = static_cast<double>(forward) * 0.10;
        const double y_odom = static_cast<double>(lateral) * 0.10;
        occupied_map.push_back(static_cast<float>(10.0 - y_odom));
        occupied_map.push_back(static_cast<float>(-3.0 + x_odom));
        occupied_map.push_back(static_cast<float>(vertical) * 0.10F);
      }
    }
  }
  auto obs = observation(1U, 1U, 1U, 1.0, 1.0, 1.0);
  lingtu::nav::tests::CollisionBitmap collision(
      {5.0, -8.0, -2.0}, {15.0, 2.0, 2.0}, 0.10);
  collision.occupyPoints(occupied_map);
  obs.collision = collision.view(1.0, 12U);
  obs.collision.resetEpoch = 3U;
  obs.collision.observationSequence = 9U;
  obs.collision.gridFromPlanningTranslation = map_from_odom.translation;
  obs.collision.gridFromPlanningYaw = map_from_odom.yaw;
  ASSERT_TRUE(obs.collision.occupied({1.2, 0.0, 0.0}));

  const auto out = awaitScanOutput([&]() {
    return loop.tick(odomInput(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0),
                         map_from_odom, nullptr, 0, 1.0, {}, obs));
  });

  ASSERT_TRUE(out.path_found) << out.reason;
  for (std::size_t index = 0; index < out.local_path_body.size(); ++index) {
    const auto &point = out.local_path_body[index];
    const auto &next = out.local_path_body[
        std::min(index + 1U, out.local_path_body.size() - 1U)];
    const double yaw = std::atan2(next.y - point.y, next.x - point.x);
    for (const double sign : {-1.0, 1.0}) {
      EXPECT_FALSE(obs.collision.occupied(
          {point.x + sign * 0.25 * std::cos(yaw),
           point.y + sign * 0.25 * std::sin(yaw), point.z}));
    }
  }
}

TEST(Executor, OdomLocalFrameKeepsTerminalGoalInMap) {
  auto loop = makeLoop();
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute(route({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
  }));

  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;
  const auto out = loop.tick(odomInput(pose(10.0, -2.0, 0.0, right_angle), pose(1.0, 0.0, 0.0, 0.0),
                                 map_from_odom, nullptr, 0, 1.0));

  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_EQ(out.reason, "goal_reached");
  EXPECT_NEAR(out.target.x, 10.0, 1e-9);
  EXPECT_NEAR(out.target.y, -2.0, 1e-9);
}

TEST(Executor, OdomLocalFrameFailsSafeForInvalidTransform) {
  auto loop = makeLoop();
  loop.setRoute(route({
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
  }));

  lingtu::nav::navigation::MapFromOdomTransform invalid_transform;
  invalid_transform.yaw = std::numeric_limits<double>::infinity();
  const auto out = loop.tick(odomInput(pose(1.0, 0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0, 0.0),
                                 invalid_transform, nullptr, 0, 1.0));

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.reason, "invalid_map_from_odom");
  EXPECT_TRUE(loop.hasRoute());
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}
