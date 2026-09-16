#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "navigation/executor.hpp"
#include "trajectory/spline.hpp"

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
                                                   double vehicle_length_m = 0.6,
                                                   int recovery_max_attempts = 3,
                                                   double recovery_blocked_interval_s = 2.0) {
  lingtu::nav::navigation::ExecutorConfig config;
  config.planning_frame = lingtu::nav::navigation::PlanningFrame::Map;
  nav_kernel::LocalPlannerParams planner;
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  config.recovery.max_attempts = recovery_max_attempts;
  config.recovery.blocked_interval_s = recovery_blocked_interval_s;
  planner.backend = nav_kernel::LocalPlannerBackend::Scan;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = false;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  planner.vehicleLength = vehicle_length_m;
  planner.scan.voxelResolution = 0.10;
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
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline &&
         (output.reason == "local_plan_pending" || output.reason == "local_plan_stale" ||
          output.reason == "local_intent_pending" || output.reason == "local_intent_stale" ||
          output.reason == "scan_trajectory_expired" ||
          output.reason == "scan_init" || output.reason == "scan_wait_target" ||
          output.reason == "scan_generate_trajectory" || output.reason == "scan_replan_trajectory" ||
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

  const auto stopped = executor.tick(
      routeInput(pose(1.05, 1.0, 0.30, 0.0), nullptr, 0, 2.0, {}, observation));
  EXPECT_EQ(stopped.reason, "scan_execution_clock_discontinuity");
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);

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
  double now = 1.0;
  const auto tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     now, {}, emptyScanObservation(now)));
  };
  auto out = awaitScanOutput(tick);
  ASSERT_TRUE(out.tracking.active) << out.reason;
  EXPECT_DOUBLE_EQ(out.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-9);
  now += 0.05;
  out = tick();

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found) << out.reason;
  EXPECT_GE(out.local_path_body.size(), 2U);
  EXPECT_GE(out.local_planner_debug.trajectoryPointCount, 2);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.reason, "teleop_assist_spline_ready");
}

void checkScanHeldLowSpeedIntent(double requested_speed) {
  // Product teleop_avoid limits, with ideal command integration instead of a robot.
  // Require 90% of the requested speed after one second, without exceeding it.
  // The planner acceleration limit constrains spline derivatives, not the
  // follower's position-feedback command; record that command slope separately.
  lingtu::nav::navigation::ExecutorConfig config;
  config.planning_frame = lingtu::nav::navigation::PlanningFrame::Map;
  config.max_speed = 0.75;
  config.teleop_intent_horizon_m = 3.5;
  config.follower.maxAccel = 0.5;
  config.follower.nominalDt = 0.01;
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.scan.maxVelocity = 0.75;
  params.scan.maxAcceleration = 0.5;
  params.scan.planningHorizon = 3.5;
  params.scan.voxelResolution = 0.10;
  auto executor = makeConfiguredExecutor(config, params, "");
  auto body = pose(0.0, 0.0, 0.4, 0.0);
  nav_kernel::Twist measured;
  double now = 100.0;
  const auto tick = [&]() {
    auto observation = emptyScanObservation(now);
    observation.clock_mode = nav_kernel::PlanClockMode::External;
    observation.body_velocity_valid = true;
    observation.body_linear_velocity = {measured.vx, measured.vy, 0.0};
    observation.body_yaw_rate = measured.wz;
    return executor.tick(intentInput(body, {requested_speed, 0.0, 0.0}, nullptr, 0, now, {}, observation));
  };
  const auto initializing = std::chrono::steady_clock::now();
  auto output = awaitScanOutput(tick);
  const double initialization_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - initializing).count();
  ASSERT_TRUE(output.tracking.active) << output.reason;
  std::printf("SCAN_LOW_SPEED requested_mps=%.6f initialization_ms=%.6f initial_duration_s=%.6f controls=%d\n",
              requested_speed, initialization_ms, output.tracking.durationS,
              output.local_planner_debug.trajectoryPointCount);
  testing::Test::RecordProperty("requested_speed_mps", std::to_string(requested_speed));
  testing::Test::RecordProperty("initialization_ms", std::to_string(initialization_ms));
  testing::Test::RecordProperty("initial_duration_s", std::to_string(output.tracking.durationS));
  double preview_speed = 0.0;
  double preview_acceleration = 0.0;
  const auto &preview = output.local_path_map;
  ASSERT_GE(preview.size(), 3U);
  const double preview_dt = output.tracking.durationS / (preview.size() - 1U);
  nav_kernel::Vec3 previous_preview_velocity;
  for (std::size_t index = 1; index < preview.size(); ++index) {
    const nav_kernel::Vec3 velocity{
        (preview[index].x - preview[index - 1U].x) / preview_dt,
        (preview[index].y - preview[index - 1U].y) / preview_dt,
        (preview[index].z - preview[index - 1U].z) / preview_dt};
    preview_speed = std::max(preview_speed,
        std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
    if (index > 1U) {
      const double ax = (velocity.x - previous_preview_velocity.x) / preview_dt;
      const double ay = (velocity.y - previous_preview_velocity.y) / preview_dt;
      const double az = (velocity.z - previous_preview_velocity.z) / preview_dt;
      preview_acceleration = std::max(preview_acceleration, std::sqrt(ax * ax + ay * ay + az * az));
    }
    previous_preview_velocity = velocity;
  }
  // Preview differences describe this sampling resolution, not an analytic bound.
  std::printf("SCAN_LOW_SPEED preview_dt_s=%.6f preview_max_speed=%.9f "
              "preview_max_acceleration=%.9f\n", preview_dt, preview_speed, preview_acceleration);
  testing::Test::RecordProperty("preview_sampled_max_speed_mps", std::to_string(preview_speed));
  testing::Test::RecordProperty("preview_sampled_max_acceleration_mps2", std::to_string(preview_acceleration));

  const auto first_id = output.tracking.trajectoryId;
  const auto started = std::chrono::steady_clock::now();
  double max_speed = 0.0;
  double max_acceleration = 0.0;
  int max_acceleration_step = 0;
  nav_kernel::Twist max_acceleration_previous_cmd, max_acceleration_cmd;
  nav_kernel::Pose max_acceleration_pose;
  nav_kernel::FollowerTracking max_acceleration_tracking;
  double speed_at_one_second = 0.0;
  for (int step = 1; step <= 400; ++step) {
    constexpr double dt = 0.01;
    measured = output.cmd_vel;
    const double c = std::cos(body.yaw);
    const double s = std::sin(body.yaw);
    body.position.x += (c * measured.vx - s * measured.vy) * dt;
    body.position.y += (s * measured.vx + c * measured.vy) * dt;
    body.yaw += measured.wz * dt;
    now = 100.0 + step * dt;
    std::this_thread::sleep_until(started + std::chrono::milliseconds(step * 10));
    output = tick();
    ASSERT_TRUE(output.tracking.active) << "step=" << step << " " << output.reason;
    const double speed = std::hypot(output.cmd_vel.vx, output.cmd_vel.vy);
    max_speed = std::max(max_speed, speed);
    const double acceleration = std::hypot(
        output.cmd_vel.vx - measured.vx, output.cmd_vel.vy - measured.vy) / dt;
    if (acceleration > max_acceleration) {
      max_acceleration = acceleration;
      max_acceleration_step = step;
      max_acceleration_previous_cmd = measured;
      max_acceleration_cmd = output.cmd_vel;
      max_acceleration_pose = body;
      max_acceleration_tracking = output.tracking;
    }
    if (step == 100) speed_at_one_second = speed;
    if (step == 50 || step == 100 || step == 200 || step == 400) {
      std::printf("SCAN_LOW_SPEED elapsed_s=%.2f speed_mps=%.9f x_m=%.9f "
                  "trajectory_id=%lld execution_s=%.6f duration_s=%.6f frozen=%d\n",
                  step * dt, speed, body.position.x,
                  static_cast<long long>(output.tracking.trajectoryId),
                  output.tracking.executionTimeS, output.tracking.durationS,
                  output.tracking.executionFrozen ? 1 : 0);
      testing::Test::RecordProperty("speed_at_" + std::to_string(step * 10) + "ms", std::to_string(speed));
    }
  }
  testing::Test::RecordProperty("maximum_speed_mps", std::to_string(max_speed));
  testing::Test::RecordProperty("maximum_command_acceleration_mps2", std::to_string(max_acceleration));
  testing::Test::RecordProperty("maximum_command_acceleration_time_s", std::to_string(max_acceleration_step * 0.01));
  std::printf("SCAN_LOW_SPEED max_command_acceleration=%.9f at_elapsed_s=%.2f\n",
              max_acceleration, max_acceleration_step * 0.01);
  std::printf("SCAN_LOW_SPEED peak_acceleration previous_cmd=(%.9f,%.9f,%.9f) "
              "command=(%.9f,%.9f,%.9f) pose=(%.9f,%.9f,%.9f,%.9f) "
              "trajectory_id=%lld execution_s=%.9f position_error_m=%.9f speed_limit_mps=%.9f\n",
              max_acceleration_previous_cmd.vx, max_acceleration_previous_cmd.vy,
              max_acceleration_previous_cmd.wz, max_acceleration_cmd.vx,
              max_acceleration_cmd.vy, max_acceleration_cmd.wz,
              max_acceleration_pose.position.x, max_acceleration_pose.position.y,
              max_acceleration_pose.position.z, max_acceleration_pose.yaw,
              static_cast<long long>(max_acceleration_tracking.trajectoryId),
              max_acceleration_tracking.executionTimeS,
              max_acceleration_tracking.positionErrorM, max_acceleration_tracking.speedLimitMps);
  EXPECT_LE(max_speed, requested_speed + 1e-6);
  EXPECT_GE(speed_at_one_second, requested_speed * 0.9);
  EXPECT_EQ(output.tracking.trajectoryId, first_id)
      << "Less than one metre of travel must not restart the stable reference";
  EXPECT_NEAR(output.tracking.executionTimeS, 4.0, 1e-8);
  measured = output.cmd_vel;
  now += 0.01;
  auto stopped = executor.tick(intentInput(body, {}, nullptr, 0, now, {}, emptyScanObservation(now)));
  EXPECT_FALSE(stopped.tracking.active);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vy, 0.0);
}

TEST(Executor, ScanHeldLowSpeedIntentAcceleratesWithoutRepeatedRestart) {
  checkScanHeldLowSpeedIntent(0.2);
}

TEST(Executor, ScanHeldVeryLowSpeedIntentReportsPlanningCostAndResponse) {
  checkScanHeldLowSpeedIntent(0.05);
}

TEST(Executor, ScanMotionIntentPublishedSplineHonorsExactDerivativeLimits) {
  struct Scenario {
    double speed;
    nav_kernel::Vec3 initial_velocity;
    double direction_rad{0.0};
  };
  for (const auto &scenario : std::vector<Scenario>{
           {0.2, {}}, {0.05, {}}, {0.2, {0.1, 0.0, 0.0}},
           {0.2, {-0.1, 0.0, 0.0}}, {0.2, {0.0, 0.1, 0.0}},
           {0.2, {}, M_PI * 0.25}}) {
    SCOPED_TRACE("speed=" + std::to_string(scenario.speed) +
                 " initial_vx=" + std::to_string(scenario.initial_velocity.x) +
                 " initial_vy=" + std::to_string(scenario.initial_velocity.y));
    nav_kernel::LocalPlannerParams params;
    params.backend = nav_kernel::LocalPlannerBackend::Scan;
    params.scan.maxVelocity = 0.75;
    params.scan.maxAcceleration = 0.5;
    params.scan.planningHorizon = 3.5;
    params.scan.voxelResolution = 0.10;
    nav_kernel::local::Planner planner(params);
    ASSERT_TRUE(planner.configure(""));
    const std::vector<nav_kernel::Vec3> route{
        {0.0, 0.0, 0.4},
        {3.5 * std::cos(scenario.direction_rad), 3.5 * std::sin(scenario.direction_rad), 0.4}};
    nav_kernel::LocalPlanRequest request;
    request.robot.pose = {route.front(), 0.0};
    request.robot.kinematics.valid = true;
    request.robot.kinematics.linearVelocity = scenario.initial_velocity;
    request.objective = nav_kernel::MotionIntentTarget{
        {scenario.direction_rad * 180.0 / M_PI, scenario.speed / 0.75, 3.5, 90.0},
        {route.data(), static_cast<int>(route.size()), 1U, false}};
    request.maxLinearSpeedMps = scenario.speed;
    request.identity = {1U, 1U, 0U};
    request.clock = {100.0, false, nav_kernel::PlanClockMode::External};
    request.environment.collision = emptyScanObservation(100.0).collision;
    auto plan = planner.plan(request);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!plan.ready() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      plan = planner.plan(request);
    }
    EXPECT_TRUE(plan.ready()) << planner.debugSnapshot().searchReason;
    if (!plan.ready()) continue;
    const auto *target = std::get_if<nav_kernel::SplineTarget>(&plan.target());
    ASSERT_NE(target, nullptr);
    ASSERT_GT(target->knots.size(), 1U);
    Eigen::MatrixXd points(3, static_cast<Eigen::Index>(target->controls.size()));
    for (std::size_t index = 0; index < target->controls.size(); ++index) {
      const auto &point = target->controls[index];
      points.col(static_cast<Eigen::Index>(index)) << point.x, point.y, point.z;
    }
    Eigen::VectorXd knots(static_cast<Eigen::Index>(target->knots.size()));
    for (std::size_t index = 0; index < target->knots.size(); ++index)
      knots[static_cast<Eigen::Index>(index)] = target->knots[index];
    nav_kernel::local::scan::upstream::UniformBspline position(
        points, target->order, target->knots[1] - target->knots[0]);
    position.setKnot(knots);
    const auto velocity = position.getDerivative();
    const auto acceleration = velocity.getDerivative();
    const double duration = position.getTimeSum();
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.005)));
    double maximum_speed = 0.0;
    double maximum_acceleration = 0.0;
    double maximum_speed_time = 0.0;
    double maximum_acceleration_time = 0.0;
    for (int index = 0; index <= samples; ++index) {
      const double time = duration * static_cast<double>(index) / samples;
      const double speed = velocity.evaluateDeBoorT(time).norm();
      const double accel = acceleration.evaluateDeBoorT(time).norm();
      if (speed > maximum_speed) {
        maximum_speed = speed;
        maximum_speed_time = time;
      }
      if (accel > maximum_acceleration) {
        maximum_acceleration = accel;
        maximum_acceleration_time = time;
      }
    }
    const Eigen::Vector3d initial_position = position.evaluateDeBoorT(0.0);
    const Eigen::Vector3d initial_velocity = velocity.evaluateDeBoorT(0.0);
    const Eigen::Vector3d initial_acceleration = acceleration.evaluateDeBoorT(0.0);
    const Eigen::Vector3d source_position(route.front().x, route.front().y, route.front().z);
    const Eigen::Vector3d source_velocity(scenario.initial_velocity.x,
                                         scenario.initial_velocity.y,
                                         scenario.initial_velocity.z);
    const double initial_position_error = (initial_position - source_position).norm();
    const double initial_velocity_error = (initial_velocity - source_velocity).norm();
    std::printf("SCAN_EXACT_DERIVATIVES request=%.6f direction_rad=%.6f source_v=(%.6f,%.6f,%.6f) "
                "duration=%.9f controls=%zu max_v=%.9f max_a=%.9f max_v_t=%.9f max_a_t=%.9f "
                "v0=(%.9f,%.9f,%.9f) a0=(%.9f,%.9f,%.9f)\n",
                scenario.speed, scenario.direction_rad, scenario.initial_velocity.x, scenario.initial_velocity.y,
                scenario.initial_velocity.z, duration, target->controls.size(),
                maximum_speed, maximum_acceleration, maximum_speed_time,
                maximum_acceleration_time, initial_velocity.x(),
                initial_velocity.y(), initial_velocity.z(), initial_acceleration.x(),
                initial_acceleration.y(), initial_acceleration.z());
    std::printf("SCAN_EXACT_BOUNDARY initial_position_error_m=%.12f "
                "initial_velocity_error_mps=%.12f initial_acceleration_norm=%.12f\n",
                initial_position_error, initial_velocity_error, initial_acceleration.norm());
    // Evaluate the actual derivative splines, not differences of the preview.
    EXPECT_LE(maximum_speed, scenario.speed + 1e-6);
    EXPECT_LE(maximum_acceleration, 0.5 + 1e-5);
    // MotionIntent restores its measured initial P/V/A analytically after time
    // scaling; this tolerance covers floating-point arithmetic, not retiming loss.
    EXPECT_LE(initial_position_error, 1e-8);
    EXPECT_LE(initial_velocity_error, 1e-8);
    EXPECT_LE(initial_acceleration.norm(), 1e-8);
  }
}

TEST(Executor, ScanExternalClockAndExplicitPausesKeepPlannerAndFollowerTogether) {
  auto executor = makeScanExecutor();
  double now = 1.0;
  auto tick = [&]() {
    auto observation = emptyScanObservation(now);
    observation.clock_mode = nav_kernel::PlanClockMode::External;
    return executor.tick(intentInput(pose(0, 0, 0, 0), {0.25, 0, 0}, nullptr, 0,
                                     now, {}, observation));
  };
  auto output = awaitScanOutput(tick);
  ASSERT_TRUE(output.tracking.active);
  now += 0.05;
  output = tick();
  const auto trajectory_id = output.tracking.trajectoryId;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  auto held = tick();
  EXPECT_DOUBLE_EQ(held.local_planner_debug.timestampS, now);
  EXPECT_DOUBLE_EQ(held.tracking.executionTimeS, output.tracking.executionTimeS);
  for (const double gap : {0.1, 0.3, 0.5}) {
    executor.pauseLinearMotion();
    now += gap;
    const auto resumed = tick();
    EXPECT_EQ(resumed.tracking.trajectoryId, trajectory_id);
    EXPECT_NEAR(resumed.tracking.executionTimeS, output.tracking.executionTimeS, 1e-12);
    now += 0.01;
    output = tick();
    EXPECT_GT(output.tracking.executionTimeS, resumed.tracking.executionTimeS);
  }
}

TEST(Executor, ScanUnannouncedClockGapInvalidatesOldSplineBeforeResuming) {
  for (const double gap : {0.3, 0.5, -0.1}) {
    SCOPED_TRACE(gap);
    auto executor = makeScanExecutor();
    double now = 1.0;
    auto body = pose(0, 0, 0, 0);
    auto tick = [&]() {
      auto observation = emptyScanObservation(now);
      observation.clock_mode = nav_kernel::PlanClockMode::External;
      return executor.tick(intentInput(body, {0.25, 0, 0}, nullptr, 0,
                                       now, {}, observation));
    };
    ASSERT_TRUE(awaitScanOutput(tick).tracking.active);
    now += gap;
    body.position.x = 0.15;
    const auto stopped = tick();
    EXPECT_FALSE(stopped.tracking.active);
    EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);
    EXPECT_EQ(stopped.reason, "scan_execution_clock_discontinuity");
    const auto fresh = awaitScanOutput(tick);
    ASSERT_TRUE(fresh.tracking.active) << fresh.reason;
    EXPECT_DOUBLE_EQ(fresh.tracking.executionTimeS, 0.0);
    EXPECT_LT(fresh.tracking.positionErrorM, 0.01);
  }
}

TEST(Executor, ScanTeleopReplanDiscardsOldSplineBeforeFollowerRestart) {
  auto executor = makeScanExecutor();
  double now = 1.0;
  auto body = pose(0.0, 0.0, 0.0, 0.0);
  auto tick = [&]() {
    return executor.tick(intentInput(body, {0.25, 0, 0}, nullptr, 0, now, {},
                                     emptyScanObservation(now)));
  };
  const auto initial = awaitScanOutput(tick);
  ASSERT_TRUE(initial.tracking.active) << initial.reason;
  now += 0.1;
  body.position.x = 0.2;
  (void)tick();
  executor.replanTeleop();
  const auto waiting = tick();
  EXPECT_FALSE(waiting.tracking.active);
  EXPECT_DOUBLE_EQ(waiting.cmd_vel.vx, 0.0);
  const auto fresh = awaitScanOutput(tick);
  EXPECT_TRUE(fresh.tracking.active) << fresh.reason;
  EXPECT_DOUBLE_EQ(fresh.tracking.executionTimeS, 0.0);
  executor.stopLinearMotion();
  const auto stopped = tick();
  EXPECT_FALSE(stopped.tracking.active);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);
}

TEST(Executor, ScanTeleopSpeedDecreaseWaitsForSlowerPlanAndResumes) {
  auto executor = makeScanExecutor();
  double speed = 0.5;
  double now = 1.0;
  auto tick = [&]() {
    return executor.tick(intentInput(pose(0, 0, 0, 0), {speed, 0, 0}, nullptr, 0, now, {},
                                     emptyScanObservation(now)));
  };
  const auto initial = awaitScanOutput(tick);
  ASSERT_TRUE(initial.tracking.active) << initial.reason;
  speed = 0.1;
  now += 0.05;
  auto output = tick();
  ASSERT_EQ(output.reason, "teleop_assist_speed_replan");
  EXPECT_TRUE(output.tracking.executionFrozen);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline &&
         (!output.tracking.active || output.tracking.executionFrozen)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output = tick();
  }
  ASSERT_TRUE(output.tracking.active) << output.reason;
  EXPECT_FALSE(output.tracking.executionFrozen) << output.reason;
  EXPECT_DOUBLE_EQ(output.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(output.cmd_vel.vx, 0.0, 1e-9);
  EXPECT_LE(std::hypot(output.cmd_vel.vx, output.cmd_vel.vy), speed + 1e-9);
  // The replacement starts from the measured stationary P/V/A. Check resumed
  // motion after execution time advances, not at the exact zero-velocity start.
  now += 0.05;
  output = tick();
  ASSERT_TRUE(output.tracking.active) << output.reason;
  EXPECT_FALSE(output.tracking.executionFrozen) << output.reason;
  EXPECT_NEAR(output.tracking.executionTimeS, 0.05, 1e-9);
  EXPECT_GT(output.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(output.cmd_vel.vx, output.cmd_vel.vy), speed + 1e-9);
  EXPECT_NEAR(output.tracking.speedLimitMps, speed, 1e-9);
}

TEST(Executor, ScanLateralIntentPreservesBodyHeading) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vy = 0.25;
  double now = 1.0;
  const auto tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     now, {}, emptyScanObservation(now)));
  };
  auto out = awaitScanOutput(tick);
  ASSERT_TRUE(out.tracking.active) << out.reason;
  EXPECT_DOUBLE_EQ(out.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(out.cmd_vel.vy, 0.0, 1e-9);
  now += 0.05;
  out = tick();

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.trajectory_frozen);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-6);
  EXPECT_GT(out.cmd_vel.vy, 0.0);
  EXPECT_NEAR(out.cmd_vel.wz, 0.0, 1e-6);
  EXPECT_NEAR(out.target_distance_m, 3.5, 1e-6);
}

TEST(Executor, ScanDiagonalIntentPreservesBodyHeading) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  intent.vy = 0.25;
  const auto observation = emptyScanObservation(1.0);

  auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     1.0, {}, observation));
  });

  for (int step = 1; step <= 10; ++step) {
    const double now = 1.0 + step * 0.05;
    out = executor.tick(intentInput(pose(0, 0, 0, 0), intent, nullptr, 0, now, {},
                                    emptyScanObservation(now)));
  }

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.trajectory_frozen);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_GT(out.cmd_vel.vy, 0.0);
  EXPECT_NEAR(out.cmd_vel.wz, 0.0, 1e-6);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), std::hypot(intent.vx, intent.vy) + 1e-9);
}

TEST(Executor, ScanReverseIntentDoesNotTurnAround) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = -0.25;
  double now = 1.0;
  const auto tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0,
                                     now, {}, emptyScanObservation(now)));
  };
  auto out = awaitScanOutput(tick);
  ASSERT_TRUE(out.tracking.active) << out.reason;
  EXPECT_DOUBLE_EQ(out.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-9);
  now += 0.05;
  out = tick();

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.trajectory_frozen);
  EXPECT_LT(out.cmd_vel.vx, 0.0);
  EXPECT_NEAR(out.cmd_vel.vy, 0.0, 1e-6);
  EXPECT_NEAR(out.cmd_vel.wz, 0.0, 1e-6);
}

TEST(Executor, ScanDirectionChangeNeverExecutesThePreviousIntentSpline) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist forward;
  forward.vx = 0.25;
  double now = 1.0;
  const auto forward_tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), forward, nullptr, 0,
                                     now, {}, emptyScanObservation(now)));
  };
  auto moving = awaitScanOutput(forward_tick);
  ASSERT_TRUE(moving.tracking.active) << moving.reason;
  EXPECT_DOUBLE_EQ(moving.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(moving.cmd_vel.vx, 0.0, 1e-9);
  now += 0.05;
  moving = forward_tick();
  ASSERT_TRUE(moving.path_found) << moving.reason;
  ASSERT_GT(moving.cmd_vel.vx, 0.0);

  nav_kernel::Twist lateral;
  lateral.vy = 0.25;
  now += 0.05;
  const auto transition = executor.tick(intentInput(
      pose(0.0, 0.0, 0.0, 0.0), lateral, nullptr, 0, now, {}, emptyScanObservation(now)));

  EXPECT_LE(std::abs(transition.cmd_vel.vx), 1e-9)
      << "a new intent must not execute the previous forward spline";

  const auto lateral_tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), lateral, nullptr, 0,
                                     now, {}, emptyScanObservation(now, 2)));
  };
  auto switched = awaitScanOutput(lateral_tick);
  ASSERT_TRUE(switched.tracking.active) << switched.reason;
  EXPECT_DOUBLE_EQ(switched.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(switched.cmd_vel.vx, 0.0, 1e-9);
  EXPECT_NEAR(switched.cmd_vel.vy, 0.0, 1e-9);
  now += 0.05;
  switched = lateral_tick();
  ASSERT_TRUE(switched.path_found) << switched.reason;
  EXPECT_FALSE(switched.trajectory_frozen);
  EXPECT_NEAR(switched.cmd_vel.vx, 0.0, 1e-6);
  EXPECT_GT(switched.cmd_vel.vy, 0.0);
  EXPECT_NEAR(switched.cmd_vel.wz, 0.0, 1e-6);
}

TEST(Executor, ScanLeftAndRightRemainBodyRelativeAtDifferentHeadings) {
  for (const double yaw : {0.0, 1.2, -2.4}) {
    for (const double lateral : {-0.25, 0.25}) {
      SCOPED_TRACE("yaw=" + std::to_string(yaw) + " lateral=" + std::to_string(lateral));
      auto executor = makeScanExecutor();
      const auto body = pose(0.0, 0.0, 0.0, yaw);
      double now = 1.0;
      const auto tick = [&]() {
        return executor.tick(intentInput(body, {0.0, lateral, 0.0}, nullptr, 0,
                                         now, {}, emptyScanObservation(now)));
      };
      auto out = awaitScanOutput(tick);
      ASSERT_TRUE(out.tracking.active) << out.reason;
      EXPECT_DOUBLE_EQ(out.tracking.executionTimeS, 0.0);
      EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-9);
      EXPECT_NEAR(out.cmd_vel.vy, 0.0, 1e-9);
      // Exact stationary boundary conditions imply zero command at first ready.
      now += 0.05;
      out = tick();
      ASSERT_TRUE(out.path_found) << out.reason;
      EXPECT_FALSE(out.trajectory_frozen);
      EXPECT_NEAR(out.tracking.executionTimeS, 0.05, 1e-9);
      const double direction_error = std::atan2(std::abs(out.cmd_vel.vx), std::abs(out.cmd_vel.vy));
      std::printf("SCAN_BODY_DIRECTION yaw=%.6f request_vy=%.6f elapsed_s=%.6f "
                  "vx=%.12f vy=%.12f wz=%.12f direction_error_deg=%.9f\n",
                  yaw, lateral, out.tracking.executionTimeS, out.cmd_vel.vx,
                  out.cmd_vel.vy, out.cmd_vel.wz, direction_error * 180.0 / M_PI);
      // Verify the body-frame mapping within one degree; an optimized spline
      // need not be an exact straight line at every sample after the boundary.
      EXPECT_LE(direction_error, M_PI / 180.0);
      EXPECT_GT(out.cmd_vel.vy * lateral, 0.0);
      EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), std::abs(lateral) + 1e-9);
      EXPECT_NEAR(out.cmd_vel.wz, 0.0, 1e-6);
    }
  }
}

TEST(Executor, ScanTeleopHonorsExplicitYawWhileTranslating) {
  auto executor = makeScanExecutor();
  double now = 1.0;
  const auto tick = [&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), {0.25, 0.0, -0.3},
                                     nullptr, 0, now, {}, emptyScanObservation(now)));
  };
  auto out = awaitScanOutput(tick);
  ASSERT_TRUE(out.tracking.active) << out.reason;
  EXPECT_DOUBLE_EQ(out.tracking.executionTimeS, 0.0);
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 1e-9);
  EXPECT_NEAR(out.cmd_vel.wz, -0.3, 1e-6);
  now += 0.05;
  out = tick();
  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_NEAR(out.cmd_vel.wz, -0.3, 1e-6);
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

TEST(Executor, ScanOccupiedStartDepartsOnlyInTheFreshRequestedDirection) {
  auto executor = makeScanExecutor(3.0, 0.6, 0);
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -1}, {4, 4, 1}, 0.1);
  // The rear cylinder starts in the first cell. Forward leaves it into free
  // cells, while a left request enters a separate occupied cell.
  collision.occupy({-0.18, 0.0, 0.0});
  collision.occupy({-0.18, 0.15, 0.0});
  auto observation = emptyScanObservation(1.0);
  observation.collision = collision.view(1.0, 1);
  double now = 1.0;
  const auto tick = [&](const nav_kernel::Twist &intent) {
    return executor.tick(intentInput(pose(0, 0, 0, 0), intent, nullptr, 0,
                                     now, {}, observation));
  };

  auto forward = awaitScanOutput([&] { return tick({0.25, 0.0, 0.0}); });
  ASSERT_TRUE(forward.path_found) << forward.reason;
  ASSERT_EQ(forward.local_path_body.size(), 2U);
  EXPECT_EQ(forward.reason, "scan_boundary_departure");
  EXPECT_TRUE(forward.recovery_verified);
  EXPECT_EQ(forward.recovery_action,
            static_cast<int>(nav_kernel::RecoveryAction::Translate));
  EXPECT_NEAR(forward.local_path_body.back().x, 0.35, 1e-9);
  EXPECT_NEAR(forward.local_path_body.back().y, 0.0, 1e-9);
  EXPECT_GT(forward.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(forward.cmd_vel.vx, forward.cmd_vel.vy), 0.15 + 1e-9);
  EXPECT_DOUBLE_EQ(forward.cmd_vel.wz, 0.0);

  now += 0.05;
  const auto released = tick({});
  EXPECT_FALSE(released.path_found);
  EXPECT_EQ(released.reason, "teleop_intent_idle");
  EXPECT_DOUBLE_EQ(std::hypot(released.cmd_vel.vx, released.cmd_vel.vy), 0.0);

  now += 0.05;
  forward = awaitScanOutput([&] { return tick({0.25, 0.0, 0.0}); });
  ASSERT_TRUE(forward.recovery_verified) << forward.reason;
  now += 0.05;
  const auto changed = tick({0.0, 0.25, 0.0});
  EXPECT_EQ(changed.reason, "teleop_intent_direction_changed");
  EXPECT_FALSE(changed.path_found);
  EXPECT_DOUBLE_EQ(std::hypot(changed.cmd_vel.vx, changed.cmd_vel.vy), 0.0);

  now += 0.05;
  const auto left = awaitScanOutput([&] { return tick({0.0, 0.25, 0.0}); });
  EXPECT_FALSE(left.recovery_verified) << left.reason;
  EXPECT_FALSE(left.path_found) << left.reason;
  EXPECT_DOUBLE_EQ(std::hypot(left.cmd_vel.vx, left.cmd_vel.vy), 0.0);
  EXPECT_DOUBLE_EQ(left.cmd_vel.wz, 0.0);

  auto yaw_executor = makeScanExecutor(3.0, 0.6, 0);
  now += 0.05;
  const auto with_yaw = awaitScanOutput([&] {
    return yaw_executor.tick(intentInput(pose(0, 0, 0, 0), {0.25, 0.0, 0.1},
                                         nullptr, 0, now, {}, observation));
  });
  EXPECT_FALSE(with_yaw.recovery_verified) << with_yaw.reason;
  EXPECT_FALSE(with_yaw.path_found) << with_yaw.reason;
  EXPECT_DOUBLE_EQ(std::hypot(with_yaw.cmd_vel.vx, with_yaw.cmd_vel.vy), 0.0);
  EXPECT_DOUBLE_EQ(with_yaw.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanBoundaryDepartureRespectsLowerRequestedSpeed) {
  auto executor = makeScanExecutor(3.0, 0.6, 0);
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -1}, {4, 4, 1}, 0.1);
  collision.occupy({-0.18, 0.0, 0.0});
  auto observation = emptyScanObservation(1.0);
  observation.collision = collision.view(1.0, 1);
  // Wait for the asynchronous planner result before measuring the speed ramp.
  const auto initialized = awaitScanOutput([&] {
    return executor.tick(intentInput(pose(0, 0, 0, 0), {0.05, 0.0, 0.0},
                                     nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(initialized.recovery_verified) << initialized.reason;
  double peak = 0.0;
  for (int tick = 0; tick < 40; ++tick) {
    const double now = 1.0 + (tick + 1) * 0.02;
    observation.collision = collision.view(now, 1);
    const auto output = executor.tick(intentInput(
        pose(0, 0, 0, 0), {0.05, 0.0, 0.0}, nullptr, 0, now, {}, observation));
    const double speed = std::hypot(output.cmd_vel.vx, output.cmd_vel.vy);
    EXPECT_LE(speed, 0.05 + 1e-9);
    peak = std::max(peak, speed);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_GT(peak, 0.0);
}

TEST(Executor, ScanFreeStartPlannerFailureDoesNotBecomeDepartureRecovery) {
  auto executor = makeScanExecutor(3.0, 0.6, 0);
  lingtu::nav::tests::CollisionBitmap collision({-1, -1, -1}, {1, 1, 1}, 0.1);
  for (int x = -10; x < 10; ++x) {
    for (int y = -10; y < 10; ++y) {
      // Leave only the current fixed-heading front and rear cells free.
      if (y == 0 && (x == -2 || x == 1)) continue;
      collision.occupy({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.05});
    }
  }
  auto observation = emptyScanObservation(1.0);
  observation.collision = collision.view(1.0, 1);
  const auto output = awaitScanOutput([&] {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0),
                                     {0.25, 0.0, 0.0}, nullptr, 0,
                                     1.0, {}, observation));
  });
  EXPECT_EQ(output.reason, "scan_local_target_blocked");
  EXPECT_FALSE(output.recovery_verified);
  EXPECT_EQ(output.recovery_action,
            static_cast<int>(nav_kernel::RecoveryAction::None));
  EXPECT_FALSE(output.path_found);
  EXPECT_DOUBLE_EQ(std::hypot(output.cmd_vel.vx, output.cmd_vel.vy), 0.0);
}

TEST(Executor, ScanEnclosedLateralIntentNeverStartsUnrequestedRecoveryTurn) {
  auto executor = makeScanExecutor();
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -1}, {4, 4, 1}, 0.1);
  for (int x = -40; x < 40; ++x)
    for (int y = -40; y < 40; ++y)
      if (std::abs(x) > 12 || std::abs(y) > 12)
        for (int z = -10; z < 10; ++z)
          collision.occupy({(x + .5) * .1, (y + .5) * .1, (z + .5) * .1});
  auto observation = emptyScanObservation(1.0);
  observation.collision = collision.view(1.0, 1);
  auto out = awaitScanOutput([&] {
    return executor.tick(intentInput(pose(0, 0, 0, 0), {0, .25, 0}, nullptr, 0,
                                     1.0, {}, observation));
  });
  // A safe prefix or braking spline may still be published inside the room.
  EXPECT_FALSE(out.recovery_verified);
  EXPECT_EQ(out.recovery_state, 0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanGoalJustBeyondCorridorDoesNotCreateReverseExcursion) {
  auto executor = makeScanExecutor(3.5);
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.7, 0.0, 0.0}}));
  const auto out = awaitScanOutput([&]() {
    return executor.tick(routeInput(pose(0, 0, 0.5, M_PI), nullptr, 0, 1.0, {},
                                    emptyScanObservation(1.0)));
  });
  ASSERT_TRUE(out.path_found) << out.reason;
  ASSERT_GE(out.local_path_map.size(), 2);
  double minimum_x = 0.0;
  for (const auto &point : out.local_path_map)
    minimum_x = std::min(minimum_x, point.x);
  EXPECT_GE(minimum_x, -0.02)
      << "A corridor cut is not a new waypoint in the full reference route";
  EXPECT_TRUE(out.trajectory_frozen);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
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

TEST(Executor, TeleopGuideFollowsExplicitSteeringAndKeepsReleasedHeading) {
  for (const bool scan : {false, true}) {
    SCOPED_TRACE(scan ? "SCAN" : "CMU");
    auto executor = scan ? makeScanExecutor() : makeCmuTeleopAvoidLoop();
    const auto initial = executor.tick(intentInput(
        pose(0.0, 0.0, 0.0, 0.0), {0.25, 0.0, 0.0}, nullptr, 0, 1.0, {},
        emptyScanObservation(1.0)));
    const double horizon = initial.target.x;
    ASSERT_GT(horizon, 0.0);

    // Steering must move the guide even before travelling the advance distance.
    const auto turning = executor.tick(intentInput(
        pose(0.0, 0.0, 0.0, 0.20), {0.25, 0.0, 0.25}, nullptr, 0, 1.1, {},
        emptyScanObservation(1.1)));
    EXPECT_NEAR(turning.target.x, horizon * std::cos(0.20), 1e-9);
    EXPECT_NEAR(turning.target.y, horizon * std::sin(0.20), 1e-9);

    // Release before another angular update: retain the final measured heading.
    const auto released = executor.tick(intentInput(
        pose(0.1, 0.0, 0.0, 0.26), {0.25, 0.0, 0.0}, nullptr, 0, 1.2, {},
        emptyScanObservation(1.2)));
    EXPECT_NEAR(released.target.x, 0.1 + horizon * std::cos(0.26), 1e-9);
    EXPECT_NEAR(released.target.y, horizon * std::sin(0.26), 1e-9);

    // An avoidance-induced turn must not keep rotating the operator's guide.
    const auto detouring = executor.tick(intentInput(
        pose(0.1, 0.0, 0.0, 0.50), {0.25, 0.0, 0.0}, nullptr, 0, 1.3, {},
        emptyScanObservation(1.3)));
    EXPECT_DOUBLE_EQ(detouring.target.x, released.target.x);
    EXPECT_DOUBLE_EQ(detouring.target.y, released.target.y);
  }
}

TEST(Executor, ScanReleasedSteeringDoesNotPullBackToOldHeading) {
  auto executor = makeScanExecutor();
  auto out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.0), {0.25, 0.0, 0.25},
                                     nullptr, 0, 1.0, {}, emptyScanObservation(1.0)));
  });
  ASSERT_TRUE(out.path_found) << out.reason;
  out = awaitScanOutput([&]() {
    return executor.tick(intentInput(pose(0.0, 0.0, 0.0, 0.12), {0.25, 0.0, 0.0},
                                     nullptr, 0, 1.1, {}, emptyScanObservation(1.1)));
  });
  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_NEAR(out.cmd_vel.wz, 0.0, 1e-6);
}

TEST(Executor, ScanHeldIntentPublishesStableReferenceSegments) {
  auto executor = makeScanExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  auto observation = emptyScanObservation(1.0);

  const auto initial = awaitScanOutput([&]() {
    return executor.tick(
        intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(initial.path_found) << initial.reason;

  observation = emptyScanObservation(1.05, 2);
  intent.vx = 0.45;
  const auto within_segment = executor.tick(
      intentInput(pose(0.2, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.05, {}, observation));
  EXPECT_DOUBLE_EQ(within_segment.target.x, initial.target.x);
  EXPECT_DOUBLE_EQ(within_segment.target.y, initial.target.y);

  observation = emptyScanObservation(1.10, 3);
  const auto advanced = executor.tick(
      intentInput(pose(1.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.10, {}, observation));
  EXPECT_GT(advanced.target.x, initial.target.x + 0.5);
  EXPECT_DOUBLE_EQ(advanced.target.y, initial.target.y);
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

TEST(Executor, ScanCollisionStopsOrReplans) {
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
  const auto stopped = [](const auto &output) {
    return std::abs(output.cmd_vel.vx) < 1e-6 &&
           std::abs(output.cmd_vel.vy) < 1e-6 &&
           std::abs(output.cmd_vel.wz) < 1e-6;
  };
  for (int tick = 1; tick <= 300 && updated.path_found &&
                     !path_is_safe(updated) && !stopped(updated); ++tick) {
    const double now = 1.05 + 0.01 * static_cast<double>(tick);
    updated = executor.tick(
        intentInput(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, now, {}, observation));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(!updated.path_found || path_is_safe(updated) || stopped(updated))
      << updated.reason;
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

TEST(Executor, FinalBrakingRejectionSurvivesSuccessfulReplanAndTriggersRecovery) {
  auto executor = makeScanExecutor(3.0, 0.6, 3, 0.1);
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));
  const auto tick = [&](double now) {
    return executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now, {},
                                    emptyScanObservation(now)));
  };
  auto first = awaitScanOutput([&] { return tick(1.0); });
  ASSERT_TRUE(first.path_found) << first.reason;
  executor.reportFinalMotionBlocked(true, 1.0);
  executor.stopLinearMotion();
  const auto replanned = awaitScanOutput([&] { return tick(1.05); });
  ASSERT_TRUE(replanned.path_found) << replanned.reason;
  EXPECT_EQ(replanned.recovery_trigger, "inactive");
  executor.reportFinalMotionBlocked(true, 1.05);
  executor.stopLinearMotion();
  const auto recovery = tick(1.11);
  EXPECT_EQ(recovery.recovery_trigger, "blocked") << recovery.reason;
  EXPECT_TRUE(recovery.recovery_verified) << recovery.recovery_reason;
  EXPECT_GT(recovery.recovery_state, 0);
}

TEST(Executor, AcceptedMotionAndNewRouteClearFinalBrakingRejection) {
  auto executor = makeScanExecutor(3.0, 0.6, 3, 0.1);
  const auto path = route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
  executor.setRoute(path);
  executor.reportFinalMotionBlocked(true, 1.0);
  executor.reportFinalMotionBlocked(false, 1.05);
  const auto tick = [&](double now) {
    return executor.tick(routeInput(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now, {},
                                    emptyScanObservation(now)));
  };
  const auto accepted = awaitScanOutput([&] { return tick(1.11); });
  EXPECT_EQ(accepted.recovery_trigger, "inactive") << accepted.reason;
  executor.reportFinalMotionBlocked(true, 1.12);
  executor.setRoute(path);
  const auto replacement = awaitScanOutput([&] { return tick(1.25); });
  EXPECT_EQ(replacement.recovery_trigger, "inactive") << replacement.reason;
}

TEST(Executor, NoSafeRecoveryCandidateConsumesOnlyFreshObservationAttempts) {
  auto loop = makeRecoverySafetyLoop();
  loop.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));
  std::vector<float> blocked(41 * 41, 95.0f);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      blocked.data(), 41, 41, 0.1, -2.05, -2.05, 1};
  const auto tick = [&](double now, std::uint64_t cloud, std::uint64_t terrain) {
    return loop.tick(routeInput(
        pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now, traversability,
        observation(1, cloud, terrain, now, now, now)));
  };

  const auto first = tick(1.0, 10, 20);
  ASSERT_EQ(first.recovery_attempt, 1) << first.recovery_reason;
  ASSERT_TRUE(first.recovery_observation_refresh_required);
  EXPECT_FALSE(first.recovery_exhausted);
  EXPECT_FALSE(first.recovery_verified);
  for (int index = 1; index <= 9; ++index) {
    const auto unchanged = tick(1.0 + 0.01 * index, 10, 20);
    expectObservationWaitStopped(unchanged);
    EXPECT_FALSE(unchanged.recovery_exhausted);
  }
  expectObservationWaitStopped(tick(1.10, 11, 20));
  expectObservationWaitStopped(tick(1.11, 10, 21));

  const auto second = tick(1.12, 11, 21);
  ASSERT_EQ(second.recovery_attempt, 2) << second.recovery_reason;
  ASSERT_TRUE(second.recovery_observation_refresh_required);
  EXPECT_FALSE(second.recovery_exhausted);
  for (int index = 1; index <= 9; ++index) {
    expectObservationWaitStopped(tick(1.12 + 0.01 * index, 11, 21));
  }

  const auto third = tick(1.22, 12, 22);
  EXPECT_EQ(third.recovery_attempt, 3);
  EXPECT_TRUE(third.recovery_exhausted);
  EXPECT_FALSE(third.recovery_observation_refresh_required);
  EXPECT_EQ(third.reason, "local_recovery_exhausted");
  EXPECT_DOUBLE_EQ(third.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(third.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(third.cmd_vel.wz, 0.0);
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
  EXPECT_TRUE(out.trajectory_frozen);
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

TEST(Executor, ScanBlockedBoundaryWaitsForFreshObservationThenUsesOnlyRearExit) {
  auto executor = makeScanExecutor(3.0, 0.6, 3, 0.1);
  const auto body = pose(0.0, 0.05, 0.435, 0.0);
  executor.setRoute(route({body.position, {3.0, 0.05, 0.435}}));
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -1}, {4, 4, 2}, 0.1);
  for (int x = -10; x < 10; ++x) {
    for (int y = -5; y < 5; ++y) {
      collision.occupy({(x + 0.5) * 0.1, (y + 0.5) * 0.1, body.position.z});
    }
  }
  auto observed = emptyScanObservation(1.0);
  observed.collision = collision.view(1.0, 1);
  const auto tick = [&](double now) {
    observed.odom_stamp_s = now;
    return executor.tick(routeInput(body, nullptr, 0, now, {}, observed));
  };

  const auto blocked = awaitScanOutput([&] { return tick(1.0); });
  ASSERT_FALSE(blocked.path_found) << blocked.reason;
  ASSERT_FALSE(blocked.recovery_verified);
  EXPECT_EQ(blocked.recovery_attempt, 0);
  const auto before_interval = tick(1.05);
  EXPECT_EQ(before_interval.recovery_trigger, "inactive");
  EXPECT_EQ(before_interval.recovery_attempt, 0);

  const auto first_attempt = awaitScanOutput([&] { return tick(1.11); });
  ASSERT_EQ(first_attempt.recovery_trigger, "blocked");
  ASSERT_EQ(first_attempt.recovery_attempt, 1) << first_attempt.recovery_reason;
  ASSERT_TRUE(first_attempt.recovery_observation_refresh_required);
  EXPECT_FALSE(first_attempt.recovery_verified);
  EXPECT_FALSE(first_attempt.recovery_exhausted);
  EXPECT_DOUBLE_EQ(first_attempt.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(first_attempt.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(first_attempt.cmd_vel.wz, 0.0);
  for (int index = 1; index <= 9; ++index) {
    expectObservationWaitStopped(tick(1.11 + 0.01 * index));
  }

  collision.clear();
  for (int x = -10; x < 10; ++x) {
    for (int y = -5; y < 5; ++y) {
      // Keep the front cylinder in an inflated boundary cell. A single-row
      // rear corridor excludes the other fifteen departure directions.
      if (y == 0 && x < 1) continue;
      collision.occupy({(x + 0.5) * 0.1, (y + 0.5) * 0.1, body.position.z});
    }
  }
  observed = emptyScanObservation(1.21, 2);
  observed.collision = collision.view(1.21, 2);
  ASSERT_TRUE(observed.collision.occupied({0.18, 0.05, body.position.z}));
  ASSERT_FALSE(observed.collision.occupied({-0.18, 0.05, body.position.z}));

  const auto reversing = awaitScanOutput([&] { return tick(1.21); });
  ASSERT_TRUE(reversing.recovery_verified) << reversing.recovery_reason;
  EXPECT_EQ(reversing.recovery_state, 2);
  EXPECT_EQ(reversing.recovery_attempt, 2);
  EXPECT_EQ(reversing.recovery_candidate_count, 1);
  EXPECT_EQ(reversing.recovery_action,
            static_cast<int>(nav_kernel::RecoveryAction::Translate));
  ASSERT_EQ(reversing.local_path_body.size(), 2U);
  EXPECT_NEAR(reversing.local_path_body.back().x, -0.35, 1e-9);
  EXPECT_NEAR(reversing.local_path_body.back().y, 0.0, 1e-9);
  EXPECT_LT(reversing.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(reversing.cmd_vel.vx, reversing.cmd_vel.vy), 0.15 + 1e-9);
  EXPECT_NEAR(reversing.cmd_vel.vy, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(reversing.cmd_vel.wz, 0.0);
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

  // Lack of physical progress is observed over normal control ticks, not a
  // single missing interval that now invalidates the old spline.
  auto recovering = moving;
  for (int tick = 1; tick <= 60 && !recovering.recovery_verified; ++tick) {
    const double now = 1.0 + 0.1 * tick;
    observation = emptyScanObservation(now, tick + 1);
    observation.traversability_generation = tick + 1;
    observation.traversability_stamp_s = now;
    recovering = awaitScanOutput([&]() {
      return executor.tick(routeInput(body, nullptr, 0, now, {}, observation));
    });
  }

  EXPECT_TRUE(recovering.recovery_verified) << recovering.reason;
  EXPECT_EQ(recovering.recovery_state, 2);
  EXPECT_EQ(recovering.recovery_reason, "recovery_translation_active");
  EXPECT_GE(recovering.local_path_body.size(), 2U);
  EXPECT_EQ(recovering.local_planner_debug.backend, nav_kernel::LocalPlannerBackend::Scan);

  ASSERT_FALSE(recovering.local_path_map.empty());
  const auto recovery_end = recovering.local_path_map.back();
  const double remaining = std::hypot(recovery_end.x - body.position.x,
                                       recovery_end.y - body.position.y);
  body.position.x = recovery_end.x - .14 * (recovery_end.x - body.position.x) / remaining;
  body.position.y = recovery_end.y - .14 * (recovery_end.y - body.position.y) / remaining;
  const double approaching_time = observation.odom_stamp_s + .1;
  observation = emptyScanObservation(approaching_time, 99);
  const auto approaching = executor.tick(routeInput(body, nullptr, 0, approaching_time, {}, observation));
  EXPECT_TRUE(approaching.recovery_verified) << approaching.reason;
  EXPECT_GT(std::hypot(approaching.cmd_vel.vx, approaching.cmd_vel.vy), 0.0)
      << "recovery must keep approaching inside the normal goal stopping radius";

  const double jump_time = observation.odom_stamp_s + 0.5;
  observation = emptyScanObservation(jump_time, 100);
  const auto stopped = executor.tick(routeInput(body, nullptr, 0, jump_time, {}, observation));
  EXPECT_EQ(stopped.reason, "scan_execution_clock_discontinuity");
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);
  const auto restarted = awaitScanOutput([&]() {
    return executor.tick(routeInput(body, nullptr, 0, jump_time, {}, observation));
  });
  EXPECT_TRUE(restarted.tracking.active) << restarted.reason;
  EXPECT_EQ(restarted.recovery_state, 0);
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

TEST(Executor, ScanMapReferenceSurvivesTfCorrections) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{1.0, -1.0, 0.0}, {1.0, 2.0, 0.0}}));
  const auto body = pose(1.0, -1.0, 0.5, M_PI / 2.0);
  auto observation = emptyScanObservation(1.0);
  observation.body_velocity_valid = true;
  lingtu::nav::navigation::MapFromOdomTransform tf{{2.0, -2.0, 0.2}, 0.4};
  auto input = [&]() {
    return odomInput(body, pose(tf.odomPointFromMap(body.position).x,
                               tf.odomPointFromMap(body.position).y,
                               tf.odomPointFromMap(body.position).z,
                               body.yaw - tf.yaw), tf, nullptr, 0, 1.0, {}, observation);
  };
  const auto initial = awaitScanOutput([&]() { return executor.tick(input()); });
  ASSERT_TRUE(initial.path_found) << initial.reason;
  ASSERT_GE(initial.local_path_map.size(), 2U);
  EXPECT_GT(initial.cmd_vel.vx, 0.0);
  EXPECT_NEAR(initial.cmd_vel.vy, 0.0, 0.02);
  EXPECT_NEAR(initial.local_path_map.back().x, 1.0, 0.02);
  for (int index = 0; index < 5; ++index) {
    tf.translation.x += 0.01;
    tf.yaw += 0.005;
    const auto next = executor.tick(input());
    ASSERT_TRUE(next.path_found) << next.reason;
    ASSERT_EQ(next.local_path_map.size(), initial.local_path_map.size());
    for (std::size_t point = 0; point < initial.local_path_map.size(); ++point) {
      ASSERT_NEAR(next.local_path_map[point].x, initial.local_path_map[point].x, 1e-6);
      ASSERT_NEAR(next.local_path_map[point].y, initial.local_path_map[point].y, 1e-6);
      ASSERT_NEAR(next.local_path_map[point].z, initial.local_path_map[point].z, 1e-6);
    }
  }
}

TEST(Executor, MapPlanningDoesNotConsumeOdomTransform) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{1.0, -1.0, 0.0}, {1.0, 2.0, 0.0}}));
  const auto body = pose(1.0, -1.0, 0.5, M_PI / 2.0);
  auto observation = emptyScanObservation(1.0);
  lingtu::nav::navigation::MapFromOdomTransform tf{};
  tf.yaw = std::numeric_limits<double>::quiet_NaN();
  const auto output = awaitScanOutput([&]() {
    return executor.tick(odomInput(body, {}, tf, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(output.path_found) << output.reason;
  EXPECT_GT(output.cmd_vel.vx, 0.0);
}

TEST(Executor, ScanEpochDiscardsOldReferenceAndSpline) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}));
  auto body = pose(0.0, 0.0, 0.5, 0.0);
  auto observation = emptyScanObservation(1.0);
  const auto initial = awaitScanOutput([&]() {
    return executor.tick(routeInput(body, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(initial.path_found) << initial.reason;
  ++observation.frame_epoch;
  body.position.x = 0.5;
  const auto reset = executor.tick(routeInput(body, nullptr, 0, 1.1, {}, observation));
  EXPECT_FALSE(reset.path_found);
  EXPECT_DOUBLE_EQ(reset.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(reset.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(reset.cmd_vel.wz, 0.0);
  const auto resumed = awaitScanOutput([&]() {
    return executor.tick(routeInput(body, nullptr, 0, 1.1, {}, observation));
  });
  ASSERT_TRUE(resumed.path_found) << resumed.reason;
  ASSERT_FALSE(resumed.local_path_map.empty());
  EXPECT_NEAR(resumed.local_path_map.front().x, body.position.x, 0.01);
  executor.clear();
  const auto cancelled = executor.tick(routeInput(body, nullptr, 0, 1.2, {}, observation));
  EXPECT_FALSE(cancelled.path_found);
  EXPECT_DOUBLE_EQ(cancelled.cmd_vel.vx, 0.0);
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

TEST(Executor, ScanInitializationFailureReportsCauseAndStops) {
  auto executor = makeScanExecutor();
  executor.setRoute(route({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}}));
  auto observation = emptyScanObservation(1.0);
  setScanCollision(observation, {0.05f, 0.05f, 0.55f});
  const auto output = awaitScanOutput([&]() {
    return executor.tick(routeInput(pose(0.05, 0.05, 0.55, 0.0), nullptr, 0,
                                    1.0, {}, observation));
  });
  EXPECT_EQ(output.reason, "scan_initialization_failed");
  EXPECT_FALSE(output.path_found);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.wz, 0.0);
}

TEST(Executor, ScanStallStopsWhenRecoveryDisabledUntilRouteReplacement) {
  lingtu::nav::navigation::ExecutorConfig config;
  config.planning_frame = lingtu::nav::navigation::PlanningFrame::Map;
  config.recovery.max_attempts = 0;
  config.recovery.blocked_interval_s = 0.5;
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.scan.voxelResolution = 0.10;
  auto executor = makeConfiguredExecutor(config, params, "");
  const auto guide = route({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  executor.setRoute(guide);
  auto run = [&](double time) {
    return executor.tick(routeInput(pose(0, 0, .5, 0), nullptr, 0, time, {},
                                    emptyScanObservation(time)));
  };
  auto output = awaitScanOutput([&]() { return run(1.0); });
  ASSERT_TRUE(output.path_found) << output.reason;
  bool moving = false;
  for (int tick = 1; tick <= 40; ++tick) {
    output = run(1.0 + tick * .05);
    moving = moving || output.cmd_vel.vx > 0.0;
    if (output.reason == "autonomy_motion_stalled") break;
  }
  ASSERT_TRUE(moving);
  ASSERT_EQ(output.reason, "autonomy_motion_stalled");
  for (int tick = 1; tick <= 10; ++tick) {
    output = run(3.0 + tick * .05);
    EXPECT_EQ(output.reason, "autonomy_motion_stalled");
    EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd_vel.vy, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd_vel.wz, 0.0);
  }
  executor.setRoute(guide);
  output = awaitScanOutput([&]() { return run(4.0); });
  EXPECT_TRUE(output.path_found) << output.reason;
  EXPECT_NE(output.reason, "autonomy_motion_stalled");
}

namespace {
lingtu::nav::navigation::Executor makeProgressLoop(bool recovery_enabled = false,
                                                   double speed = 0.15) {
  lingtu::nav::navigation::ExecutorConfig config;
  config.max_speed = speed;
  config.recovery.max_attempts = recovery_enabled ? 3 : 0;
  config.recovery.blocked_interval_s = 0.5;
  config.follower.maxAccel = 0.5;
  config.follower.linearStopThreshold = 0.0;
  config.follower.slowDwnDisThre = 0.0;
  nav_kernel::LocalPlannerParams planner;
  planner.checkObstacle = false;
  planner.useTerrainAnalysis = false;
  planner.useTraversabilityCost = false;
  return makeConfiguredExecutor(config, planner, LINGTU_TEST_PATH_LIBRARY);
}
}  // namespace

TEST(Executor, StallDoesNotCountWrongDirectionDriftHeaveOrYawAsTranslation) {
  for (const int scenario : {0, 1, 2, 3}) {
    SCOPED_TRACE(scenario);
    auto executor = makeProgressLoop();
    executor.setRoute(route({{0, 0, 0}, {10, 0, 0}}));
    lingtu::nav::navigation::ExecutionOutput output;
    // A corrective heading-alignment phase gets its own observation interval.
    for (int tick = 0; tick <= 40; ++tick) {
      const double t = tick * 0.05;
      auto body = pose(0, 0, 0, 0);
      lingtu::nav::navigation::ExecutionObservation obs;
      obs.body_velocity_valid = true;
      if (scenario == 0) { body.position.y = .15 * t; obs.body_linear_velocity.y = .15; }
      if (scenario == 1) { body.position.x = -.15 * t; obs.body_linear_velocity.x = -.15; }
      if (scenario == 2) { body.position.z = .08 * std::sin(8 * t); }
      if (scenario == 3) { body.yaw = -.15 * t; obs.body_yaw_rate = -.15; }
      output = executor.tick(routeInput(body, nullptr, 0, 1.0 + t, {}, obs));
      if (output.reason == "autonomy_motion_stalled") break;
    }
    EXPECT_EQ(output.reason, "autonomy_motion_stalled");
    EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd_vel.vy, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd_vel.wz, 0.0);
  }
}

TEST(Executor, StallUsesSidewaysCommandInPlanningFrame) {
  auto executor = makeProgressLoop();
  executor.setRoute(route({{0, 0, 0}, {.38, 0, 0}}, std::nullopt, .05));
  lingtu::nav::navigation::ExecutionOutput output;
  bool sideways_commanded = false;
  constexpr double yaw = M_PI / 3;
  for (int tick = 0; tick <= 40; ++tick) {
    const double t = tick * 0.05;
    lingtu::nav::navigation::ExecutionObservation obs;
    obs.body_velocity_valid = true;
    obs.body_linear_velocity.x = std::sin(yaw) * .03;
    obs.body_linear_velocity.y = std::cos(yaw) * .03;
    output = executor.tick(routeInput(pose(0, .03 * t, 0, yaw), nullptr, 0,
                                      1.0 + t, {}, obs));
    sideways_commanded = sideways_commanded || output.cmd_vel.vy < -.01;
    if (output.reason == "autonomy_motion_stalled") break;
  }
  EXPECT_TRUE(sideways_commanded);
  EXPECT_EQ(output.reason, "autonomy_motion_stalled");
}

TEST(Executor, StallStartsExistingRecoveryWhenDriftingAwayFromPath) {
  auto executor = makeProgressLoop(true);
  executor.setRoute(route({{0, 0, 0}, {10, 0, 0}}));
  lingtu::nav::navigation::ExecutionOutput output;
  for (int tick = 0; tick <= 16; ++tick) {
    const double t = tick * 0.05;
    output = executor.tick(routeInput(pose(-.15 * t, 0, 0, 0), nullptr, 0, 1.0 + t));
    if (output.recovery_trigger == "stalled") break;
  }
  EXPECT_EQ(output.recovery_trigger, "stalled");
  EXPECT_TRUE(output.recovery_verified) << output.recovery_reason;
  EXPECT_GT(output.recovery_state, 0);
}

TEST(Executor, StallAllowsSlowProgressAtDifferentBodyHeadings) {
  for (const double yaw : {0.0, M_PI / 2}) {
    SCOPED_TRACE(yaw);
    auto executor = makeProgressLoop(false, .03);
    executor.setRoute(route({{0, 0, 0}, {10 * std::cos(yaw), 10 * std::sin(yaw), 0}}));
    bool translation_commanded = false;
    for (int tick = 0; tick <= 80; ++tick) {
      const double t = tick * .05;
      const auto output = executor.tick(routeInput(pose(.03 * t * std::cos(yaw),
                                                        .03 * t * std::sin(yaw), 0, yaw),
                                                   nullptr, 0, 1.0 + t));
      translation_commanded = translation_commanded || output.cmd_vel.vx > .02;
      ASSERT_TRUE(output.path_found) << output.reason;
      ASSERT_NE(output.reason, "autonomy_motion_stalled");
      ASSERT_EQ(output.recovery_trigger, "inactive");
    }
    EXPECT_TRUE(translation_commanded);
  }
}

TEST(Executor, StallDuringHeadingAlignmentRequiresRotationInRequestedDirection) {
  for (const bool correct_direction : {false, true}) {
    SCOPED_TRACE(correct_direction);
    auto executor = makeProgressLoop();
    executor.setRoute(route({{0, 0, 0}, {0, 10, 0}}));
    lingtu::nav::navigation::ExecutionOutput output;
    auto body = pose(0, 0, 0, 0);
    bool requested_rotation = false;
    for (int tick = 0; tick <= 24; ++tick) {
      const double t = tick * .05;
      if (tick > 0) {
        if (correct_direction) {
          body.position.x += .05 * (std::cos(body.yaw) * output.cmd_vel.vx -
                                    std::sin(body.yaw) * output.cmd_vel.vy);
          body.position.y += .05 * (std::sin(body.yaw) * output.cmd_vel.vx +
                                    std::cos(body.yaw) * output.cmd_vel.vy);
          body.yaw += .05 * output.cmd_vel.wz;
        } else {
          body.yaw -= .05 * .15;
        }
      }
      output = executor.tick(routeInput(body, nullptr, 0, 1.0 + t));
      requested_rotation = requested_rotation || output.cmd_vel.wz > 0.0;
      if (output.reason == "autonomy_motion_stalled") break;
    }
    EXPECT_TRUE(requested_rotation);
    if (correct_direction) {
      EXPECT_NE(output.reason, "autonomy_motion_stalled");
    } else {
      EXPECT_EQ(output.reason, "autonomy_motion_stalled");
      EXPECT_DOUBLE_EQ(output.cmd_vel.wz, 0.0);
    }
  }
}

TEST(Executor, StallDoesNotCountOscillationOrInstantaneousSpeedAsProgress) {
  auto executor = makeProgressLoop();
  executor.setRoute(route({{0, 0, 0}, {10, 0, 0}}));
  lingtu::nav::navigation::ExecutionOutput output;
  for (int tick = 0; tick <= 60; ++tick) {
    const double t = tick * .05;
    lingtu::nav::navigation::ExecutionObservation obs;
    obs.body_velocity_valid = true;
    obs.body_linear_velocity.x = .4 * std::cos(40 * t);
    output = executor.tick(routeInput(pose(.01 * std::sin(40 * t), 0, 0, 0),
                                      nullptr, 0, 1.0 + t, {}, obs));
    if (output.reason == "autonomy_motion_stalled") break;
  }
  EXPECT_EQ(output.reason, "autonomy_motion_stalled");
}

TEST(Executor, StallAllowsFiftyHertzPoseUpdatesAtHundredHertzControl) {
  auto executor = makeProgressLoop(false, .03);
  executor.setRoute(route({{0, 0, 0}, {10, 0, 0}}));
  for (int tick = 0; tick <= 150; ++tick) {
    const double t = tick * .01;
    const double pose_time = (tick / 2) * .02;
    const auto output = executor.tick(routeInput(pose(.03 * pose_time, 0, 0, 0),
                                                 nullptr, 0, 1.0 + t));
    ASSERT_NE(output.reason, "autonomy_motion_stalled");
    ASSERT_EQ(output.recovery_trigger, "inactive");
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
  lingtu::nav::navigation::ExecutorConfig config;
  config.planning_frame = lingtu::nav::navigation::PlanningFrame::Odom;
  config.corridor_lookahead_m = 3.0;
  config.max_speed = 0.5;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 2.0;
  config.follower.nominalDt = 0.05;
  nav_kernel::LocalPlannerParams planner;
  planner.backend = nav_kernel::LocalPlannerBackend::Scan;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = false;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  planner.scan.voxelResolution = 0.10;
  auto loop = makeConfiguredExecutor(std::move(config), planner, "");
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

  const auto tick = [&]() {
    return loop.tick(odomInput(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0),
                         map_from_odom, nullptr, 0, 1.0, {}, obs));
  };
  auto out = awaitScanOutput(tick);
  // The first polynomial seed may be blocked; this test verifies the eventual
  // detour and frame transform, including SCAN's bounded initialization retries.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (out.reason == "scan_initialization_failed" &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    out = awaitScanOutput(tick);
  }

  ASSERT_TRUE(out.path_found) << out.reason;
  for (std::size_t index = 0; index < out.local_path_body.size(); ++index) {
    const auto &point = out.local_path_body[index];
    const auto &next = out.local_path_body[
        std::min(index + 1U, out.local_path_body.size() - 1U)];
    const double yaw = std::atan2(next.y - point.y, next.x - point.x);
    for (const double sign : {-1.0, 1.0}) {
      EXPECT_FALSE(obs.collision.occupied(
          {point.x + sign * planner.scan.cylinderOffset * std::cos(yaw),
           point.y + sign * planner.scan.cylinderOffset * std::sin(yaw), point.z}));
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


TEST(Executor, TaskSpeedLimitCapsOutputAndResetsWithNextRoute) {
  auto loop = makeLoop();
  auto limited = route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
  limited.maxSpeedMps = 0.12;
  loop.setRoute(limited);
  bool moved = false;
  for (int i = 0; i < 20; ++i) {
    auto out = loop.tick(routeInput(pose(0, 0, 0, 0), nullptr, 0, 1.0 + i * 0.05));
    const double speed = std::hypot(out.cmd_vel.vx, out.cmd_vel.vy);
    EXPECT_LE(speed, 0.120001);
    moved = moved || speed > 0.01;
  }
  EXPECT_TRUE(moved);
  EXPECT_DOUBLE_EQ(loop.activeMaxSpeedMps(), 0.12);
  loop.setRoute(route({{0, 0, 0}, {3, 0, 0}}));
  EXPECT_DOUBLE_EQ(loop.activeMaxSpeedMps(), 0.5);
  limited.maxSpeedMps = 1.0;
  loop.setRoute(limited);
  EXPECT_DOUBLE_EQ(loop.activeMaxSpeedMps(), 0.5);
}

TEST(Executor, ScanTaskSpeedLimitAllowsBoundedMotion) {
  auto executor = makeScanExecutor();
  auto limited = route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
  limited.maxSpeedMps = 0.12;
  executor.setRoute(limited);
  auto body = pose(0.0, 0.0, 0.5, 0.0);
  auto observation = emptyScanObservation(1.0);
  observation.clock_mode = nav_kernel::PlanClockMode::External;
  auto out = awaitScanOutput([&]() {
    return executor.tick(routeInput(body, nullptr, 0, 1.0, {}, observation));
  });
  ASSERT_TRUE(out.path_found) << out.reason;
  bool moved = false;
  for (int i = 1; i <= 10; ++i) {
    observation = emptyScanObservation(1.0 + i * 0.05);
    observation.clock_mode = nav_kernel::PlanClockMode::External;
    out = executor.tick(routeInput(body, nullptr, 0, 1.0 + i * 0.05, {}, observation));
    const double speed = std::hypot(out.cmd_vel.vx, out.cmd_vel.vy);
    EXPECT_LE(speed, 0.120001);
    moved = moved || speed > 1e-4;
  }
  EXPECT_TRUE(moved) << out.reason << " vx=" << out.cmd_vel.vx
                    << " time=" << out.tracking.executionTimeS
                    << " duration=" << out.tracking.durationS
                    << " error=" << out.tracking.positionErrorM;
}
