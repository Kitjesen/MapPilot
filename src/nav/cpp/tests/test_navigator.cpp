#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <vector>

#include "navigation/navigator.hpp"

namespace {

lingtu::nav::navigation::Navigator makeLoop(bool use_traversability = false,
                                            double corridor_lookahead_m = 2.0,
                                            double teleop_max_deviation_deg = 55.0,
                                            int debug_candidate_limit = 0) {
  lingtu::nav::navigation::NavigatorConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  config.teleop_intent_max_deviation_deg = teleop_max_deviation_deg;
  config.planner.checkObstacle = true;
  config.planner.useTerrainAnalysis = true;
  config.planner.useTraversabilityCost = use_traversability;
  config.planner.traversabilityNearFieldStop = use_traversability;
  config.planner.debugCandidateLimit = debug_candidate_limit;
  config.planner.autonomySpeed = 0.5;
  config.planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  lingtu::nav::navigation::Navigator loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}

lingtu::nav::navigation::Navigator makeScanNavigator(double corridor_lookahead_m = 3.0) {
  lingtu::nav::navigation::NavigatorConfig config;
  config.path_library_dir.clear();
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  config.planner.backend = nav_kernel::LocalPlannerBackend::Scan;
  config.planner.checkObstacle = true;
  config.planner.useTerrainAnalysis = true;
  config.planner.useTraversabilityCost = false;
  config.planner.autonomySpeed = 0.5;
  config.planner.maxSpeed = 1.0;
  config.planner.scan.voxelResolution = 0.10;
  config.planner.scan.horizontalRange = 4.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 2.0;
  config.follower.nominalDt = 0.05;
  lingtu::nav::navigation::Navigator navigator(config);
  EXPECT_TRUE(navigator.configure());
  return navigator;
}

lingtu::nav::navigation::Navigator makeRecoveryExhaustionLoop() {
  lingtu::nav::navigation::NavigatorConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.planner.checkObstacle = false;
  config.planner.useTerrainAnalysis = true;
  config.planner.useTraversabilityCost = true;
  config.planner.traversabilityNearFieldStop = false;
  config.planner.traversabilityHardCost = 90.0;
  config.recovery.blocked_interval_s = 0.0;
  config.recovery.max_attempts = 0;
  config.planner.autonomySpeed = 0.5;
  config.planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  lingtu::nav::navigation::Navigator loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}
lingtu::nav::navigation::Navigator makeRecoverySafetyLoop(double no_progress_timeout_s = 2.0,
                                                          int max_attempts = 3,
                                                          double near_field_stop_dis = 0.5) {
  lingtu::nav::navigation::NavigatorConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.planner.checkObstacle = true;
  config.planner.useTerrainAnalysis = true;
  config.planner.useTraversabilityCost = true;
  config.planner.traversabilityNearFieldStop = false;
  config.planner.traversabilityHardCost = 90.0;
  config.planner.dirThre = 20.0;
  config.recovery.blocked_interval_s = 0.0;
  config.recovery.rotation_timeout_s = 0.1;
  config.recovery.translation_timeout_s = no_progress_timeout_s;
  config.recovery.max_attempts = max_attempts;
  config.planner.nearFieldStopDis = near_field_stop_dis;
  config.planner.vehicleLength = 0.80;
  config.planner.vehicleWidth = 0.60;
  config.planner.footprintPadding = 0.27;
  config.planner.autonomySpeed = 0.5;
  config.planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  config.follower.baseLookAheadDis = 0.25;
  config.follower.lookAheadRatio = 0.2;
  lingtu::nav::navigation::Navigator loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
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

lingtu::nav::navigation::NavigatorObservation observation(std::uint64_t frame_epoch,
                                                          std::uint64_t cloud_generation,
                                                          std::uint64_t traversability_generation,
                                                          double odom_stamp_s, double cloud_stamp_s,
                                                          double traversability_stamp_s) {
  lingtu::nav::navigation::NavigatorObservation view;
  view.frame_epoch = frame_epoch;
  view.cloud_generation = cloud_generation;
  view.traversability_generation = traversability_generation;
  view.odom_stamp_s = odom_stamp_s;
  view.cloud_stamp_s = cloud_stamp_s;
  view.traversability_stamp_s = traversability_stamp_s;
  return view;
}

void expectObservationWaitStopped(const lingtu::nav::navigation::NavigatorOutput &out) {
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

template <typename Tick>
lingtu::nav::navigation::NavigatorOutput awaitScanOutput(Tick tick) {
  auto output = tick();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         (output.reason == "local_plan_pending" || output.reason == "local_plan_stale" ||
          output.reason == "local_intent_pending" || output.reason == "local_intent_stale" ||
          output.reason == "scan_trajectory_expired")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output = tick();
  }
  return output;
}

}  // namespace

TEST(Navigator, PlansLocalPathAndCmdVelFromGlobalPath) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  const auto next = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.02);

  EXPECT_TRUE(out.active);
  EXPECT_FALSE(out.goal_reached);
  EXPECT_TRUE(out.path_found);
  EXPECT_FALSE(out.near_field_stop);
  EXPECT_GE(out.local_path_body.size(), 2u);
  EXPECT_EQ(out.local_path_body.size(), out.local_path_map.size());
  EXPECT_GT(next.cmd_vel.vx, 0.0);
}

TEST(Navigator, SuspendAutonomyRetainsGoalAndResetsStallTiming) {
  auto loop = makeLoop();
  loop.setRoute(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {3.0, 0.0, 0.0},
      },
      0.75);

  const auto moving = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  ASSERT_TRUE(moving.active);
  ASSERT_TRUE(moving.path_found);

  loop.suspendAutonomy();

  EXPECT_TRUE(loop.hasRoute());
  const auto resumed = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 100.0);
  EXPECT_TRUE(resumed.active);
  EXPECT_TRUE(resumed.path_found);
  EXPECT_EQ(resumed.recovery_state, 0)
      << "a pause must not carry stale stall/recovery timing into resume";
  EXPECT_FALSE(resumed.recovery_exhausted);
}

TEST(Navigator, StopsWhenGoalReached) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.0},
  });

  const auto out = loop.tick(pose(0.2, 0.0, 0.0, 0.0), nullptr, 0, 1.0);

  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_EQ(out.target_index, 1u);
  EXPECT_DOUBLE_EQ(out.target.x, 0.2);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Navigator, UsesPerLegInspectionArrivalTolerance) {
  auto loop = makeLoop();
  loop.setRoute(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0},
      },
      std::nullopt, 0.1);

  const auto outside = loop.tick(pose(0.7, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  EXPECT_TRUE(outside.active);
  EXPECT_FALSE(outside.goal_reached);

  const auto inside = loop.tick(pose(0.92, 0.0, 0.0, 0.0), nullptr, 0, 1.1);
  EXPECT_FALSE(inside.active);
  EXPECT_TRUE(inside.goal_reached);
}

TEST(Navigator, UsesPerLegInspectionYawTolerance) {
  auto loop = makeLoop();
  loop.setRoute(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      1.0, 0.35, 0.5);

  const auto reached = loop.tick(pose(0.2, 0.0, 0.0, 0.6), nullptr, 0, 1.0);
  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
}

TEST(Navigator, TeleopIntentPlansWithoutGlobalPathAndHonorsRequestedSpeed) {
  auto loop = makeLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.25;

  lingtu::nav::navigation::NavigatorOutput out;
  for (int i = 0; i < 4; ++i) {
    out = loop.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0 + 0.05 * i);
  }

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found);
  EXPECT_GE(out.local_path_body.size(), 2u);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(Navigator, ScanTeleopIntentUsesTimedTrajectory) {
  auto navigator = makeScanNavigator();
  nav_kernel::Twist intent;
  intent.vx = 0.25;

  const auto out = awaitScanOutput(
      [&]() { return navigator.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0); });

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found) << out.reason;
  EXPECT_GE(out.local_path_body.size(), 2U);
  EXPECT_GE(out.local_trajectory_body.size(), 2U);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.reason, "teleop_assist_trajectory_ready");
}

TEST(Navigator, ScanFreezesTrajectoryClockWhileAligningHeading) {
  auto navigator = makeScanNavigator();
  navigator.setRoute({
      {0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 2.0, 0.0},
      {0.0, 3.0, 0.0},
  });

  double now = 1.0;
  const auto first =
      awaitScanOutput([&]() { return navigator.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now); });
  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_GE(first.local_trajectory_body.size(), 2U);
  EXPECT_TRUE(first.trajectory_frozen);
  EXPECT_DOUBLE_EQ(first.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(first.cmd_vel.vy, 0.0);
  EXPECT_GT(first.cmd_vel.wz, 0.0);
  const double initial_end_time = first.local_trajectory_body.back().timeFromStartS;

  now += 0.35;
  const auto frozen = navigator.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, now);
  ASSERT_TRUE(frozen.path_found) << frozen.reason;
  ASSERT_GE(frozen.local_trajectory_body.size(), 2U);
  EXPECT_TRUE(frozen.trajectory_frozen);
  EXPECT_NEAR(frozen.local_trajectory_body.back().timeFromStartS, initial_end_time, 1e-6);

  now += 0.05;
  const auto aligned = navigator.tick(pose(0.0, 0.0, 0.0, 0.5 * M_PI), nullptr, 0, now);
  ASSERT_TRUE(aligned.path_found) << aligned.reason;
  EXPECT_FALSE(aligned.trajectory_frozen);
  EXPECT_GT(aligned.cmd_vel.vx, 0.0);
}

TEST(Navigator, ScanTeleopIntentReceivesCollisionSnapshot) {
  auto navigator = makeScanNavigator();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  const std::vector<float> occupied_xyz{1.0F, 0.0F, 0.0F};
  lingtu::nav::navigation::NavigatorObservation observation;
  observation.frame_epoch = 1;
  observation.cloud_generation = 1;
  observation.collision = {
      occupied_xyz.data(),
      1,
      0.1,
      {-4.0, -4.0, -1.0},
      {4.0, 4.0, 1.0},
      1,
      1,
      1,
      1.0,
      1.0,
      false,
      true,
  };

  const auto out = awaitScanOutput([&]() {
    return navigator.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation);
  });

  EXPECT_FALSE(out.path_found);
  EXPECT_EQ(out.reason, "collision_map_incomplete");
}

TEST(Navigator, ScanStopsOldTrajectoryWhenCollisionGenerationChanges) {
  auto navigator = makeScanNavigator();
  nav_kernel::Twist intent;
  intent.vx = 0.25;
  lingtu::nav::navigation::NavigatorObservation observation;
  observation.frame_epoch = 1;
  observation.cloud_generation = 1;
  observation.collision = {
      nullptr, 0, 0.1, {-4.0, -4.0, -1.0}, {4.0, 4.0, 1.0}, 1, 1, 1, 1.0, 1.0, true, true,
  };

  const auto ready = awaitScanOutput([&]() {
    return navigator.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.0, {}, observation);
  });
  ASSERT_TRUE(ready.path_found) << ready.reason;

  observation.collision.observationSequence = 2;
  observation.collision.generation = 2;
  observation.collision.stampS = 1.05;
  observation.collision.receiveStampS = 1.05;
  const auto pending =
      navigator.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, nullptr, 0, 1.05, {}, observation);

  EXPECT_FALSE(pending.path_found);
  EXPECT_EQ(pending.reason, "local_intent_pending");
  EXPECT_DOUBLE_EQ(pending.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(pending.cmd_vel.vy, 0.0);
}

TEST(Navigator, TeleopIntentSelectsDetourInsteadOfRawForwardStop) {
  auto loop = makeLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.3;
  std::vector<float> obstacle_xyzh;
  for (int i = -2; i <= 2; ++i) {
    obstacle_xyzh.push_back(0.9f);
    obstacle_xyzh.push_back(0.08f * static_cast<float>(i));
    obstacle_xyzh.push_back(0.0f);
    obstacle_xyzh.push_back(1.0f);
  }

  const auto out = loop.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0);

  EXPECT_TRUE(out.path_found);
  EXPECT_GE(out.local_path_body.size(), 2u);
  double max_lateral = 0.0;
  for (const auto &point : out.local_path_body) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.1);
  EXPECT_NE(out.reason, "teleop_assist_no_path");
}

TEST(Navigator, TeleopIntentHardLimitsSelectedPathEndDirection) {
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

  const auto out = loop.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0);

  if (out.path_found) {
    ASSERT_GE(out.local_path_body.size(), 2U);
    const auto &before_end = out.local_path_body[out.local_path_body.size() - 2];
    const auto &end = out.local_path_body.back();
    const double end_direction_deg =
        std::atan2(end.y - before_end.y, end.x - before_end.x) * 180.0 / M_PI;
    EXPECT_LE(std::abs(end_direction_deg), 5.0 + 1e-6);
  } else {
    EXPECT_EQ(out.cmd_vel.vx, 0.0);
    EXPECT_EQ(out.cmd_vel.wz, 0.0);
  }
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(Navigator, TeleopIntentStopsWithoutAutonomousRecoveryWhenSurrounded) {
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

  const auto out = loop.tickIntent(pose(0.0, 0.0, 0.0, 0.0), intent, obstacle_xyzh.data(),
                                   static_cast<int>(obstacle_xyzh.size() / 4), 1.0);

  EXPECT_FALSE(out.path_found);
  EXPECT_EQ(out.reason, "near_field_stop");
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(Navigator, ReportsLocalRecoveryExhaustedAsTerminalStop) {
  auto loop = makeRecoveryExhaustionLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  std::vector<float> risk_grid(9 * 9, 95.0f);
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 9, 9, 0.5, -2.0, -2.0, 1,
  };

  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);

  EXPECT_TRUE(out.active);
  EXPECT_FALSE(out.path_found);
  EXPECT_TRUE(out.recovery_exhausted);
  EXPECT_EQ(out.recovery_state, 0);
  EXPECT_EQ(out.reason, "local_recovery_exhausted");
  EXPECT_TRUE(out.local_path_body.empty());
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}
TEST(Navigator, RecoveryDoesNotBackUpThroughBlockedRearFootprint) {
  auto loop = makeRecoverySafetyLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

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
  (void)loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                  static_cast<int>(obstacle_xyzh.size() / 4), 1.0, traversability);
  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                             static_cast<int>(obstacle_xyzh.size() / 4), 1.2, traversability);

  for (const auto &point : out.local_path_body) {
    EXPECT_GE(point.x, -1e-6);
  }
  EXPECT_GE(out.cmd_vel.vx, -1e-6);
}
TEST(Navigator, RecoverySelectsOnlyReachableLateralExit) {
  auto loop = makeRecoverySafetyLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

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

  (void)loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability);
  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);

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

TEST(Navigator, RecoveryProgressRequiresOdometryMovement) {
  auto loop = makeRecoverySafetyLoop(0.1, 1);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

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

  (void)loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability);
  const auto started = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);
  ASSERT_EQ(started.recovery_state, 2);
  EXPECT_TRUE(started.recovery_verified);
  EXPECT_DOUBLE_EQ(started.recovery_progress, 0.0);
  EXPECT_GT(started.cmd_vel.vy, 0.0);

  const auto stalled = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.2, traversability);
  EXPECT_TRUE(stalled.recovery_exhausted);
  EXPECT_EQ(stalled.reason, "local_recovery_exhausted");
  EXPECT_DOUBLE_EQ(stalled.recovery_progress, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.wz, 0.0);
}

TEST(Navigator, RecoveryWaitsForFreshCloudAndTraversabilityAfterRotationCompletes) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

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
  const auto rotating = loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.00, traversability, base);

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason << " recovery_reason=" << rotating.recovery_reason
      << " candidates=" << rotating.recovery_candidate_count;
  ASSERT_TRUE(rotating.recovery_verified);
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);
  EXPECT_FALSE(rotating.recovery_observation_refresh_required);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.20, traversability,
                                   observation(1, 10, 20, 1.20, 0.95, 0.96));

  EXPECT_TRUE(completed.recovery_observation_refresh_required);
  EXPECT_EQ(completed.reason, "recovery_rotation_complete");
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.wz, 0.0);

  const auto unchanged = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.25, traversability,
                                   observation(1, 10, 20, 1.25, 0.95, 0.96));
  expectObservationWaitStopped(unchanged);

  const auto cloud_only = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                    obstacleCount(obstacle_xyzh), 1.30, traversability,
                                    observation(1, 11, 20, 1.30, 1.30, 0.96));
  expectObservationWaitStopped(cloud_only);

  const auto advanced_generation_stale_stamp = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.35,
      traversability, observation(1, 12, 21, 1.35, 1.20, 1.20));
  expectObservationWaitStopped(advanced_generation_stale_stamp);

  const auto released = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.40, traversability,
                                  observation(1, 12, 21, 1.40, 1.31, 1.32));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}

TEST(Navigator, RecoveryObservationWaitRebasesOnFrameEpochChange) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

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
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.00,
                traversability, observation(1, 10, 20, 1.00, 0.95, 0.96));

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason << " recovery_reason=" << rotating.recovery_reason;
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                   obstacleCount(obstacle_xyzh), 1.20, traversability,
                                   observation(1, 10, 20, 1.20, 0.95, 0.96));
  ASSERT_TRUE(completed.recovery_observation_refresh_required);

  const auto rebased = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                 obstacleCount(obstacle_xyzh), 1.25, traversability,
                                 observation(2, 1, 1, 1.25, 1.25, 1.25));
  expectObservationWaitStopped(rebased);

  const auto same_epoch_same_generation = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.30,
      traversability, observation(2, 1, 1, 1.30, 1.30, 1.30));
  expectObservationWaitStopped(same_epoch_same_generation);

  const auto released = loop.tick(pose(0.0, 0.0, 0.0, completed_yaw), obstacle_xyzh.data(),
                                  obstacleCount(obstacle_xyzh), 1.35, traversability,
                                  observation(2, 2, 2, 1.35, 1.31, 1.32));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}
TEST(Navigator, AlignsRequestedYawBeforeReportingGoalReached) {
  auto loop = makeLoop();
  loop.setRoute(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      1.0);

  const auto aligning = loop.tick(pose(0.2, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  EXPECT_TRUE(aligning.active);
  EXPECT_FALSE(aligning.goal_reached);
  EXPECT_EQ(aligning.reason, "aligning_goal_yaw");
  EXPECT_DOUBLE_EQ(aligning.cmd_vel.vx, 0.0);
  EXPECT_GT(aligning.cmd_vel.wz, 0.0);

  const auto reached = loop.tick(pose(0.2, 0.0, 0.0, 0.98), nullptr, 0, 1.1);
  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
  EXPECT_EQ(reached.reason, "goal_reached");
  EXPECT_DOUBLE_EQ(reached.cmd_vel.wz, 0.0);
}

TEST(Navigator, GoalYawAlignmentUsesShortestWrappedError) {
  auto loop = makeLoop();
  loop.setRoute(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      -3.05);

  const auto out = loop.tick(pose(0.2, 0.0, 0.0, 3.10), nullptr, 0, 1.0);
  EXPECT_TRUE(out.active);
  EXPECT_EQ(out.reason, "aligning_goal_yaw");
  EXPECT_GT(out.cmd_vel.wz, 0.0);
  EXPECT_LT(std::abs(out.cmd_vel.wz), 0.25);
}

TEST(Navigator, RelocalizationJumpSelectsPathAheadOfRobot) {
  auto loop = makeLoop();
  std::vector<nav_kernel::Vec3> path;
  for (int i = 0; i <= 100; ++i) {
    path.push_back({0.1 * static_cast<double>(i), 0.0, 0.0});
  }
  loop.setRoute(path);

  const auto out = loop.tick(pose(8.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);

  EXPECT_GE(out.target_index, 80u);
  EXPECT_GE(out.target.x, 8.0);
}

TEST(Navigator, ShortCorridorLookaheadDoesNotCutAcrossUpcomingCorner) {
  auto loop = makeLoop(false, 0.6);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.0},
      {0.4, 0.0, 0.0},
      {0.6, 0.0, 0.0},
      {0.8, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {1.0, 0.2, 0.0},
      {1.0, 0.4, 0.0},
  });

  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);

  EXPECT_LE(out.target.x, 0.8);
  EXPECT_DOUBLE_EQ(out.target.y, 0.0);
}

TEST(Navigator, RollingLocalPathDoesNotTriggerFinalGoalSlowdown) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {5.0, 0.0, 0.0},
  });

  lingtu::nav::navigation::NavigatorOutput out;
  for (int i = 0; i < 6; ++i) {
    out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0 + 0.05 * i);
  }

  EXPECT_NEAR(out.cmd_vel.vx, 0.5, 1e-6);
}

TEST(Navigator, NearFieldObstacleBlocksCmdVel) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  const std::vector<float> obstacle_xyzh = {
      0.45f,
      0.0f,
      0.0f,
      1.0f,
  };

  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(),
                             static_cast<int>(obstacle_xyzh.size() / 4), 1.0);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Navigator, SafeLocalDetourOverridesStraightIntentNearFieldStop) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  const std::vector<float> obstacle_xyzh = {
      0.85f,
      0.0f,
      0.0f,
      1.0f,
  };

  const auto out =
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), obstacle_xyzh.data(), obstacleCount(obstacle_xyzh), 1.0);

  ASSERT_TRUE(out.path_found) << out.reason;
  EXPECT_FALSE(out.near_field_stop)
      << "a collision-checked local detour must supersede the blocked straight intent";
  EXPECT_EQ(out.reason, "control_ready");
  EXPECT_GT(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 1e-6);
}

TEST(Navigator, KeepsDetourSideAcrossSmallObstacleJitter) {
  auto loop = makeLoop();
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
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
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), left_bias.data(), obstacleCount(left_bias), 1.0);
  const std::vector<float> transient_close_obstacle{
      0.45F,
      0.0F,
      0.0F,
      1.0F,
  };
  const auto stopped = loop.tick(pose(0.0, 0.0, 0.0, 0.0), transient_close_obstacle.data(),
                                 obstacleCount(transient_close_obstacle), 1.025);
  const auto second =
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), right_bias.data(), obstacleCount(right_bias), 1.05);

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

TEST(Navigator, CommittedGuideDoesNotReplaceGlobalRouteBends) {
  auto loop = makeLoop(false, 3.0, 55.0, 1);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {0.3, -0.1, 0.0},
      {0.5, -0.3, 0.0},
      {0.7, -0.7, 0.0},
      {1.1, -0.9, 0.0},
      {1.5, -0.7, 0.0},
      {2.1, -0.3, 0.0},
      {3.0, 0.0, 0.0},
  });

  const auto first = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  const auto second = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.05);

  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_TRUE(second.path_found) << second.reason;
  EXPECT_GT(second.local_planner_debug.relativeGoalDistanceM, 1.0)
      << "the committed local guide must augment, not truncate, the global route";
}

TEST(Navigator, TraversabilityGridBlocksCmdVel) {
  auto loop = makeLoop(true);
  loop.setRoute({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  std::vector<float> risk_grid(5 * 5, 0.0f);
  risk_grid[2 * 5 + 1] = 95.0f;
  risk_grid[2 * 5 + 2] = 95.0f;
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 5, 5, 0.25, 0.0, -0.5,
  };

  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Navigator, ScanRecoveryDoesNotRequireCmuPathLibrary) {
  auto navigator = makeScanNavigator();
  navigator.setRoute({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});

  nav_kernel::Pose body;
  lingtu::nav::navigation::NavigatorObservation observation;
  observation.frame_epoch = 1;
  observation.cloud_generation = 1;
  observation.traversability_generation = 1;
  observation.odom_stamp_s = 1.0;
  observation.cloud_stamp_s = 1.0;
  observation.traversability_stamp_s = 1.0;
  observation.collision.resolution = 0.10;
  observation.collision.aabbMin = {-10.0, -10.0, -2.0};
  observation.collision.aabbMax = {10.0, 10.0, 2.0};
  observation.collision.resetEpoch = 1;
  observation.collision.observationSequence = 1;
  observation.collision.generation = 1;
  observation.collision.stampS = 1.0;
  observation.collision.receiveStampS = 1.0;
  observation.collision.complete = true;
  observation.collision.live = true;

  const auto moving =
      awaitScanOutput([&]() { return navigator.tick(body, nullptr, 0, 1.0, {}, observation); });
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
      awaitScanOutput([&]() { return navigator.tick(body, nullptr, 0, 3.1, {}, observation); });

  EXPECT_TRUE(recovering.recovery_verified) << recovering.reason;
  EXPECT_EQ(recovering.recovery_state, 2);
  EXPECT_EQ(recovering.recovery_reason, "recovery_translation_active");
  EXPECT_GE(recovering.local_path_body.size(), 2U);
  EXPECT_EQ(recovering.local_planner_debug.backend, nav_kernel::LocalPlannerBackend::Scan);
}

TEST(Navigator, ScanPreservesRouteBendsAndElevation) {
  auto navigator = makeScanNavigator();
  navigator.setRoute({
      {0.0, 0.0, 0.0},
      {0.8, 0.6, 0.15},
      {1.6, 0.6, 0.35},
      {2.4, 0.0, 0.55},
  });
  lingtu::nav::navigation::NavigatorObservation observation;
  observation.frame_epoch = 1;
  observation.body_velocity_valid = true;

  const auto output = awaitScanOutput(
      [&]() { return navigator.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, {}, observation); });

  ASSERT_TRUE(output.path_found) << output.reason;
  ASSERT_GE(output.local_path_map.size(), 3U);
  ASSERT_GE(output.local_trajectory_body.size(), 3U);
  const auto lateral = std::max_element(output.local_path_map.begin(), output.local_path_map.end(),
                                        [](const auto &a, const auto &b) { return a.y < b.y; });
  const auto elevated = std::max_element(output.local_path_map.begin(), output.local_path_map.end(),
                                         [](const auto &a, const auto &b) { return a.z < b.z; });
  EXPECT_GT(lateral->y, 0.35);
  EXPECT_GT(elevated->z, 0.40);
  EXPECT_EQ(output.reason, "trajectory_control_ready");
}

TEST(Navigator, ScanProjectsOntoSparseRouteInsteadOfDrivingBackToWaypoint) {
  auto navigator = makeScanNavigator(3.0);
  navigator.setRoute({
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {10.0, 10.0, 0.0},
  });

  const auto output =
      awaitScanOutput([&]() { return navigator.tick(pose(5.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0); });

  ASSERT_TRUE(output.path_found) << output.reason;
  EXPECT_NEAR(output.target.x, 8.0, 1e-6);
  EXPECT_NEAR(output.target.y, 0.0, 1e-6);
  ASSERT_GE(output.local_path_body.size(), 2U);
  for (const auto &point : output.local_path_body) {
    EXPECT_GE(point.x, -1e-6) << "the local route must not fold back to the sparse segment start";
  }
}

TEST(Navigator, OdomLocalFrameUsesOdomRiskAndPublishesMapPaths) {
  auto loop = makeLoop(true, 2.0, 55.0, 1);
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
      {10.0, -1.0, 0.0},
      {10.0, 0.0, 0.0},
  });

  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;

  std::vector<float> risk_grid(9 * 9, 0.0f);
  risk_grid[4 * 9 + 6] = 95.0f;
  const lingtu::nav::navigation::TraversabilityGridView traversability{
      risk_grid.data(), 9, 9, 0.25, -1.0, -1.0, 7,
  };

  const auto out = loop.tickOdom(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0),
                                 map_from_odom, nullptr, 0, 1.0, traversability);

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

TEST(Navigator, OdomLocalFrameTransformsMapObstaclesIntoPlannerFrame) {
  auto loop = makeLoop();
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
      {10.0, -1.0, 0.0},
  });

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
      loop.tickOdom(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0), map_from_odom,
                    obstacle_xyzh_map.data(), obstacleCount(obstacle_xyzh_map), 1.0);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop) << "the map-frame obstacle is 0.45 m ahead in odom";
  EXPECT_EQ(out.reason, "near_field_stop");
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}

TEST(Navigator, OdomLocalFrameTransformsCompleteCollisionLayerAndAabb) {
  auto loop = makeScanNavigator();
  loop.setRoute({
      {10.0, -3.0, 0.0},
      {10.0, -2.2, 0.0},
      {10.0, -1.4, 0.0},
      {10.0, -0.6, 0.0},
  });
  const double right_angle = 3.14159265358979323846 / 2.0;
  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;

  std::vector<float> occupied_map;
  for (int lateral = -7; lateral <= 7; ++lateral) {
    for (int vertical = -3; vertical <= 3; ++vertical) {
      const double y_odom = static_cast<double>(lateral) * 0.10;
      occupied_map.push_back(static_cast<float>(10.0 - y_odom));
      occupied_map.push_back(-1.8F);
      occupied_map.push_back(static_cast<float>(vertical) * 0.10F);
    }
  }
  auto obs = observation(1U, 1U, 1U, 1.0, 1.0, 1.0);
  obs.collision = {
      occupied_map.data(),
      static_cast<int>(occupied_map.size() / 3U),
      0.10,
      {5.0, -8.0, -2.0},
      {15.0, 2.0, 2.0},
      3U,
      9U,
      12U,
      1.0,
      1.0,
      true,
      true,
  };

  const auto out = awaitScanOutput([&]() {
    return loop.tickOdom(pose(10.0, -3.0, 0.0, right_angle), pose(0.0, 0.0, 0.0, 0.0),
                         map_from_odom, nullptr, 0, 1.0, {}, obs);
  });

  ASSERT_TRUE(out.path_found) << out.reason;
  const auto lateral =
      std::max_element(out.local_path_body.begin(), out.local_path_body.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, out.local_path_body.end());
  EXPECT_GT(std::abs(lateral->y), 0.75);
}

TEST(Navigator, OdomLocalFrameKeepsTerminalGoalInMap) {
  auto loop = makeLoop();
  const double right_angle = std::acos(-1.0) * 0.5;
  loop.setRoute({
      {10.0, -3.0, 0.0},
      {10.0, -2.0, 0.0},
  });

  lingtu::nav::navigation::MapFromOdomTransform map_from_odom;
  map_from_odom.translation = {10.0, -3.0, 0.0};
  map_from_odom.yaw = right_angle;
  const auto out = loop.tickOdom(pose(10.0, -2.0, 0.0, right_angle), pose(1.0, 0.0, 0.0, 0.0),
                                 map_from_odom, nullptr, 0, 1.0);

  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_EQ(out.reason, "goal_reached");
  EXPECT_NEAR(out.target.x, 10.0, 1e-9);
  EXPECT_NEAR(out.target.y, -2.0, 1e-9);
}

TEST(Navigator, OdomLocalFrameFailsSafeForInvalidTransform) {
  auto loop = makeLoop();
  loop.setRoute({
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
  });

  lingtu::nav::navigation::MapFromOdomTransform invalid_transform;
  invalid_transform.yaw = std::numeric_limits<double>::infinity();
  const auto out = loop.tickOdom(pose(1.0, 0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0, 0.0),
                                 invalid_transform, nullptr, 0, 1.0);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.reason, "invalid_map_from_odom");
  EXPECT_TRUE(loop.hasRoute());
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
}
