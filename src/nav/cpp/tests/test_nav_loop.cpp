#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

#include "nav_loop.hpp"

namespace {

lingtu::nav::plan::NavLoop makeLoop(
    bool use_traversability = false,
    double corridor_lookahead_m = 2.0,
    double teleop_max_deviation_deg = 55.0) {
  lingtu::nav::plan::NavLoopConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = corridor_lookahead_m;
  config.max_speed = 0.5;
  config.teleop_intent_max_deviation_deg = teleop_max_deviation_deg;
  config.local_planner.checkObstacle = true;
  config.local_planner.useTerrainAnalysis = true;
  config.local_planner.useTraversabilityCost = use_traversability;
  config.local_planner.traversabilityNearFieldStop = use_traversability;
  config.local_planner.autonomySpeed = 0.5;
  config.local_planner.maxSpeed = 1.0;
  config.path_follower.maxSpeed = 0.5;
  config.path_follower.maxAccel = 10.0;
  config.path_follower.nominalDt = 0.05;
  config.path_follower.baseLookAheadDis = 0.25;
  config.path_follower.lookAheadRatio = 0.2;
  lingtu::nav::plan::NavLoop loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}

lingtu::nav::plan::NavLoop makeRecoveryExhaustionLoop() {
  lingtu::nav::plan::NavLoopConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.local_planner.checkObstacle = false;
  config.local_planner.useTerrainAnalysis = true;
  config.local_planner.useTraversabilityCost = true;
  config.local_planner.traversabilityNearFieldStop = false;
  config.local_planner.traversabilityHardCost = 90.0;
  config.local_planner.recoveryBlockedThre = 0.0;
  config.local_planner.recoveryMaxCycles = 0;
  config.local_planner.autonomySpeed = 0.5;
  config.local_planner.maxSpeed = 1.0;
  config.path_follower.maxSpeed = 0.5;
  config.path_follower.maxAccel = 10.0;
  config.path_follower.nominalDt = 0.05;
  config.path_follower.baseLookAheadDis = 0.25;
  config.path_follower.lookAheadRatio = 0.2;
  lingtu::nav::plan::NavLoop loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}
lingtu::nav::plan::NavLoop makeRecoverySafetyLoop(
    double no_progress_timeout_s = 2.0,
    int max_attempts = 3,
    double near_field_stop_dis = 0.5) {
  lingtu::nav::plan::NavLoopConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.local_planner.checkObstacle = true;
  config.local_planner.useTerrainAnalysis = true;
  config.local_planner.useTraversabilityCost = true;
  config.local_planner.traversabilityNearFieldStop = false;
  config.local_planner.traversabilityHardCost = 90.0;
  config.local_planner.dirThre = 20.0;
  config.local_planner.recoveryBlockedThre = 0.0;
  config.local_planner.recoveryRotateTime = 0.1;
  config.local_planner.recoveryBackupTime = no_progress_timeout_s;
  config.local_planner.recoveryMaxCycles = max_attempts;
  config.local_planner.nearFieldStopDis = near_field_stop_dis;
  config.local_planner.vehicleLength = 0.80;
  config.local_planner.vehicleWidth = 0.60;
  config.local_planner.footprintPadding = 0.27;
  config.local_planner.autonomySpeed = 0.5;
  config.local_planner.maxSpeed = 1.0;
  config.path_follower.maxSpeed = 0.5;
  config.path_follower.maxAccel = 10.0;
  config.path_follower.nominalDt = 0.05;
  config.path_follower.baseLookAheadDis = 0.25;
  config.path_follower.lookAheadRatio = 0.2;
  lingtu::nav::plan::NavLoop loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}

std::vector<float> makeRotationOnlyObservedGrid(
    int rows,
    int cols,
    double resolution,
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

int obstacleCount(const std::vector<float>& obstacle_xyzh) {
  return static_cast<int>(obstacle_xyzh.size() / 4);
}

lingtu::nav::plan::PlannerObservationView observation(
    std::uint64_t frame_epoch,
    std::uint64_t cloud_generation,
    std::uint64_t traversability_generation,
    double odom_stamp_s,
    double cloud_stamp_s,
    double traversability_stamp_s) {
  lingtu::nav::plan::PlannerObservationView view;
  view.frame_epoch = frame_epoch;
  view.cloud_generation = cloud_generation;
  view.traversability_generation = traversability_generation;
  view.odom_stamp_s = odom_stamp_s;
  view.cloud_stamp_s = cloud_stamp_s;
  view.traversability_stamp_s = traversability_stamp_s;
  return view;
}

void expectObservationWaitStopped(
    const lingtu::nav::plan::NavLoopOutput& out) {
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

}  // namespace

TEST(NavLoop, PlansLocalPathAndCmdVelFromGlobalPath) {
  auto loop = makeLoop();
  loop.setGlobalPath({
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

TEST(NavLoop, StopsWhenGoalReached) {
  auto loop = makeLoop();
  loop.setGlobalPath({
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

TEST(NavLoop, UsesPerLegInspectionArrivalTolerance) {
  auto loop = makeLoop();
  loop.setGlobalPath(
      {
          {0.0, 0.0, 0.0},
          {1.0, 0.0, 0.0},
      },
      std::nullopt,
      0.1);

  const auto outside = loop.tick(pose(0.7, 0.0, 0.0, 0.0), nullptr, 0, 1.0);
  EXPECT_TRUE(outside.active);
  EXPECT_FALSE(outside.goal_reached);

  const auto inside = loop.tick(pose(0.92, 0.0, 0.0, 0.0), nullptr, 0, 1.1);
  EXPECT_FALSE(inside.active);
  EXPECT_TRUE(inside.goal_reached);
}

TEST(NavLoop, UsesPerLegInspectionYawTolerance) {
  auto loop = makeLoop();
  loop.setGlobalPath(
      {
          {0.0, 0.0, 0.0},
          {0.2, 0.0, 0.0},
      },
      1.0,
      0.35,
      0.5);

  const auto reached = loop.tick(pose(0.2, 0.0, 0.0, 0.6), nullptr, 0, 1.0);
  EXPECT_FALSE(reached.active);
  EXPECT_TRUE(reached.goal_reached);
}

TEST(NavLoop, TeleopIntentPlansWithoutGlobalPathAndHonorsRequestedSpeed) {
  auto loop = makeLoop();
  nav_kernel::Twist intent;
  intent.vx = 0.25;

  lingtu::nav::plan::NavLoopOutput out;
  for (int i = 0; i < 4; ++i) {
    out = loop.tickTeleopIntent(
        pose(0.0, 0.0, 0.0, 0.0),
        intent,
        nullptr,
        0,
        1.0 + 0.05 * i);
  }

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.path_found);
  EXPECT_GE(out.local_path_body.size(), 2u);
  EXPECT_GT(out.cmd_vel.vx, 0.0);
  EXPECT_LE(std::hypot(out.cmd_vel.vx, out.cmd_vel.vy), 0.25 + 1e-6);
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(NavLoop, TeleopIntentSelectsDetourInsteadOfRawForwardStop) {
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

  const auto out = loop.tickTeleopIntent(
      pose(0.0, 0.0, 0.0, 0.0),
      intent,
      obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4),
      1.0);

  EXPECT_TRUE(out.path_found);
  EXPECT_GE(out.local_path_body.size(), 2u);
  double max_lateral = 0.0;
  for (const auto& point : out.local_path_body) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.1);
  EXPECT_NE(out.reason, "teleop_assist_no_path");
}

TEST(NavLoop, TeleopIntentHardLimitsSelectedPathEndDirection) {
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

  const auto out = loop.tickTeleopIntent(
      pose(0.0, 0.0, 0.0, 0.0),
      intent,
      obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4),
      1.0);

  if (out.path_found) {
    ASSERT_GE(out.local_path_body.size(), 2U);
    const auto& before_end = out.local_path_body[out.local_path_body.size() - 2];
    const auto& end = out.local_path_body.back();
    const double end_direction_deg =
        std::atan2(end.y - before_end.y, end.x - before_end.x) * 180.0 / M_PI;
    EXPECT_LE(std::abs(end_direction_deg), 5.0 + 1e-6);
  } else {
    EXPECT_EQ(out.cmd_vel.vx, 0.0);
    EXPECT_EQ(out.cmd_vel.wz, 0.0);
  }
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(NavLoop, TeleopIntentStopsWithoutAutonomousRecoveryWhenSurrounded) {
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

  const auto out = loop.tickTeleopIntent(
      pose(0.0, 0.0, 0.0, 0.0),
      intent,
      obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4),
      1.0);

  EXPECT_FALSE(out.path_found);
  EXPECT_EQ(out.reason, "teleop_assist_no_path");
  EXPECT_DOUBLE_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_vel.wz, 0.0);
  EXPECT_EQ(out.recovery_state, 0);
}

TEST(NavLoop, ReportsLocalRecoveryExhaustedAsTerminalStop) {
  auto loop = makeRecoveryExhaustionLoop();
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  std::vector<float> risk_grid(9 * 9, 95.0f);
  const lingtu::nav::plan::TraversabilityGridView traversability{
      risk_grid.data(),
      9,
      9,
      0.5,
      -2.0,
      -2.0,
      1,
  };

  const auto out = loop.tick(
      pose(0.0, 0.0, 0.0, 0.0),
      nullptr,
      0,
      1.0,
      traversability);

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
TEST(NavLoop, RecoveryDoesNotBackUpThroughBlockedRearFootprint) {
  auto loop = makeRecoverySafetyLoop();
  loop.setGlobalPath({
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
  const lingtu::nav::plan::TraversabilityGridView traversability{
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
TEST(NavLoop, RecoverySelectsOnlyReachableLateralExit) {
  auto loop = makeRecoverySafetyLoop();
  loop.setGlobalPath({
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
  const lingtu::nav::plan::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 2,
  };

  (void)loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability);
  const auto out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);

  ASSERT_GE(out.local_path_body.size(), 2u);
  EXPECT_EQ(out.recovery_state, 2);
  EXPECT_GT(out.local_path_body.back().y, 0.6);
  EXPECT_LT(std::abs(out.local_path_body.back().x), 0.25);
  EXPECT_GT(out.cmd_vel.vy, 0.0)
      << "reason=" << out.reason << " verified=" << out.recovery_verified
      << " progress=" << out.recovery_progress << " path_size=" << out.local_path_body.size()
      << " first_next=(" << out.local_path_body[1].x << "," << out.local_path_body[1].y << ")";
  EXPECT_NEAR(out.cmd_vel.vx, 0.0, 0.05);
}

TEST(NavLoop, RecoveryProgressRequiresOdometryMovement) {
  auto loop = makeRecoverySafetyLoop(0.1, 1);
  loop.setGlobalPath({
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
  const lingtu::nav::plan::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 3,
  };

  (void)loop.tick(
      pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 0.9, traversability);
  const auto started =
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0, traversability);
  ASSERT_EQ(started.recovery_state, 2);
  EXPECT_TRUE(started.recovery_verified);
  EXPECT_DOUBLE_EQ(started.recovery_progress, 0.0);
  EXPECT_GT(started.cmd_vel.vy, 0.0);

  const auto stalled =
      loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.2, traversability);
  EXPECT_TRUE(stalled.recovery_exhausted);
  EXPECT_EQ(stalled.reason, "local_recovery_exhausted");
  EXPECT_DOUBLE_EQ(stalled.recovery_progress, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(stalled.cmd_vel.wz, 0.0);
}

TEST(NavLoop, RecoveryWaitsForFreshCloudAndTraversabilityAfterRotationCompletes) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid =
      makeRotationOnlyObservedGrid(kRows, kCols, kResolution, kOrigin);
  const lingtu::nav::plan::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 10,
  };
  const std::vector<float> obstacle_xyzh = makeRotationOnlyObstacleRing();

  const auto base = observation(1, 10, 20, 1.00, 0.95, 0.96);
  const auto rotating = loop.tick(
      pose(0.0, 0.0, 0.0, 0.0),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.00,
      traversability,
      base);

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason
      << " recovery_reason=" << rotating.recovery_reason
      << " candidates=" << rotating.recovery_candidate_count;
  ASSERT_TRUE(rotating.recovery_verified);
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);
  EXPECT_FALSE(rotating.recovery_observation_refresh_required);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.20,
      traversability,
      observation(1, 10, 20, 1.20, 0.95, 0.96));

  EXPECT_TRUE(completed.recovery_observation_refresh_required);
  EXPECT_EQ(completed.reason, "recovery_rotation_complete");
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(completed.cmd_vel.wz, 0.0);

  const auto unchanged = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.25,
      traversability,
      observation(1, 10, 20, 1.25, 0.95, 0.96));
  expectObservationWaitStopped(unchanged);

  const auto cloud_only = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.30,
      traversability,
      observation(1, 11, 20, 1.30, 1.30, 0.96));
  expectObservationWaitStopped(cloud_only);

  const auto advanced_generation_stale_stamp = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.35,
      traversability,
      observation(1, 12, 21, 1.35, 1.20, 1.20));
  expectObservationWaitStopped(advanced_generation_stale_stamp);

  const auto released = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.40,
      traversability,
      observation(1, 12, 21, 1.40, 1.31, 1.32));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason
      << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}

TEST(NavLoop, RecoveryObservationWaitRebasesOnFrameEpochChange) {
  auto loop = makeRecoverySafetyLoop(2.0, 3, 0.2);
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid =
      makeRotationOnlyObservedGrid(kRows, kCols, kResolution, kOrigin);
  const lingtu::nav::plan::TraversabilityGridView traversability{
      grid.data(), kRows, kCols, kResolution, kOrigin, kOrigin, 11,
  };
  const std::vector<float> obstacle_xyzh = makeRotationOnlyObstacleRing();

  const auto rotating = loop.tick(
      pose(0.0, 0.0, 0.0, 0.0),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.00,
      traversability,
      observation(1, 10, 20, 1.00, 0.95, 0.96));

  ASSERT_EQ(rotating.recovery_state, 1)
      << "reason=" << rotating.reason
      << " recovery_reason=" << rotating.recovery_reason;
  ASSERT_NE(rotating.cmd_vel.wz, 0.0);

  const double completed_yaw = rotating.cmd_vel.wz > 0.0 ? 0.35 : -0.35;
  const auto completed = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.20,
      traversability,
      observation(1, 10, 20, 1.20, 0.95, 0.96));
  ASSERT_TRUE(completed.recovery_observation_refresh_required);

  const auto rebased = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.25,
      traversability,
      observation(2, 1, 1, 1.25, 1.25, 1.25));
  expectObservationWaitStopped(rebased);

  const auto same_epoch_same_generation = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.30,
      traversability,
      observation(2, 1, 1, 1.30, 1.30, 1.30));
  expectObservationWaitStopped(same_epoch_same_generation);

  const auto released = loop.tick(
      pose(0.0, 0.0, 0.0, completed_yaw),
      obstacle_xyzh.data(),
      obstacleCount(obstacle_xyzh),
      1.35,
      traversability,
      observation(2, 2, 2, 1.35, 1.31, 1.32));

  EXPECT_NE(released.reason, "recovery_observation_wait");
  EXPECT_FALSE(released.near_field_stop);
  EXPECT_FALSE(released.recovery_observation_refresh_required);
  EXPECT_EQ(released.recovery_state, 1)
      << "reason=" << released.reason
      << " recovery_reason=" << released.recovery_reason;
  EXPECT_NE(released.cmd_vel.wz, 0.0);
}
TEST(NavLoop, AlignsRequestedYawBeforeReportingGoalReached) {
  auto loop = makeLoop();
  loop.setGlobalPath(
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

TEST(NavLoop, GoalYawAlignmentUsesShortestWrappedError) {
  auto loop = makeLoop();
  loop.setGlobalPath(
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

TEST(NavLoop, RelocalizationJumpSelectsPathAheadOfRobot) {
  auto loop = makeLoop();
  std::vector<nav_kernel::Vec3> path;
  for (int i = 0; i <= 100; ++i) {
    path.push_back({0.1 * static_cast<double>(i), 0.0, 0.0});
  }
  loop.setGlobalPath(path);

  const auto out = loop.tick(pose(8.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0);

  EXPECT_GE(out.target_index, 80u);
  EXPECT_GE(out.target.x, 8.0);
}

TEST(NavLoop, ShortCorridorLookaheadDoesNotCutAcrossUpcomingCorner) {
  auto loop = makeLoop(false, 0.6);
  loop.setGlobalPath({
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

TEST(NavLoop, RollingLocalPathDoesNotTriggerFinalGoalSlowdown) {
  auto loop = makeLoop();
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {5.0, 0.0, 0.0},
  });

  lingtu::nav::plan::NavLoopOutput out;
  for (int i = 0; i < 6; ++i) {
    out = loop.tick(pose(0.0, 0.0, 0.0, 0.0), nullptr, 0, 1.0 + 0.05 * i);
  }

  EXPECT_NEAR(out.cmd_vel.vx, 0.5, 1e-6);
}

TEST(NavLoop, NearFieldObstacleBlocksCmdVel) {
  auto loop = makeLoop();
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  const std::vector<float> obstacle_xyzh = {
      0.45f, 0.0f, 0.0f, 1.0f,
  };

  const auto out = loop.tick(
      pose(0.0, 0.0, 0.0, 0.0),
      obstacle_xyzh.data(),
      static_cast<int>(obstacle_xyzh.size() / 4),
      1.0);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}

TEST(NavLoop, TraversabilityGridBlocksCmdVel) {
  auto loop = makeLoop(true);
  loop.setGlobalPath({
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  });
  std::vector<float> risk_grid(5 * 5, 0.0f);
  risk_grid[2 * 5 + 1] = 95.0f;
  risk_grid[2 * 5 + 2] = 95.0f;
  const lingtu::nav::plan::TraversabilityGridView traversability{
      risk_grid.data(),
      5,
      5,
      0.25,
      0.0,
      -0.5,
  };

  const auto out = loop.tick(
      pose(0.0, 0.0, 0.0, 0.0),
      nullptr,
      0,
      1.0,
      traversability);

  EXPECT_TRUE(out.active);
  EXPECT_TRUE(out.near_field_stop);
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
}
