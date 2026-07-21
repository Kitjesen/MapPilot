#include "nav_loop.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

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
