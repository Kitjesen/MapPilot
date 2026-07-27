#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "motion/control_authority.hpp"
#include "motion/teleop_safety.hpp"
#include "nav_loop.hpp"

#ifndef LINGTU_TEST_PATH_LIBRARY
#error "LINGTU_TEST_PATH_LIBRARY must name the LocalPlanner path library"
#endif

namespace {

nav_kernel::Pose originPose() {
  nav_kernel::Pose pose;
  pose.yaw = 0.0;
  return pose;
}

lingtu::nav::plan::NavLoop makeAssistedLoop() {
  lingtu::nav::plan::NavLoopConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.max_speed = 0.4;
  config.teleop_intent_horizon_m = 2.0;
  config.teleop_intent_max_deviation_deg = 55.0;
  config.path_follower.maxAccel = 1.0;
  config.path_follower.nominalDt = 0.05;
  lingtu::nav::plan::NavLoop loop(config);
  EXPECT_TRUE(loop.configure());
  return loop;
}

std::vector<float> forwardObstacleCluster() {
  std::vector<float> points;
  for (int i = -2; i <= 2; ++i) {
    points.push_back(0.9F);
    points.push_back(0.08F * static_cast<float>(i));
    points.push_back(0.0F);
    points.push_back(1.0F);
  }
  return points;
}

TEST(AssistedTeleop, TakeoverDetoursThenFinalSafetyCanVetoAndResumeStaysFresh) {
  lingtu::nav::endpoint::ControlAuthority authority;
  ASSERT_TRUE(authority.activatePath());

  nav_kernel::Twist intent;
  intent.vx = 0.3;
  ASSERT_TRUE(authority.beginOperatorTakeover(intent, 1.0));
  EXPECT_TRUE(authority.operatorTakeoverLatched());
  EXPECT_FALSE(authority.pathActive());
  EXPECT_FALSE(authority.activatePath());

  auto loop = makeAssistedLoop();
  const auto obstacles = forwardObstacleCluster();
  auto assisted = loop.tickTeleopIntent(originPose(), *authority.teleopRequest(), obstacles.data(),
                                        static_cast<int>(obstacles.size() / 4), 1.0);

  ASSERT_TRUE(assisted.path_found);
  ASSERT_GE(assisted.local_path_map.size(), 2U);
  EXPECT_EQ(assisted.recovery_state, 0);
  double max_lateral = 0.0;
  for (const auto &point : assisted.local_path_map) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.1);

  lingtu::nav::endpoint::CommandSafetyConfig safety;
  safety.check_obstacle = true;
  safety.use_traversability_cost = false;
  const auto accepted = lingtu::nav::endpoint::evaluateAutonomyPathSafety(
      safety, assisted.cmd_vel, originPose(), assisted.local_path_map, obstacles, {}, false);
  EXPECT_FALSE(accepted.stopped);

  const auto &blocked_point = assisted.local_path_map[assisted.local_path_map.size() / 2];
  const std::vector<float> obstacle_on_detour{
      static_cast<float>(blocked_point.x),
      static_cast<float>(blocked_point.y),
      0.3F,
      1.0F,
  };
  const auto vetoed = lingtu::nav::endpoint::evaluateAutonomyPathSafety(
      safety, assisted.cmd_vel, originPose(), assisted.local_path_map, obstacle_on_detour, {},
      false);
  EXPECT_TRUE(vetoed.stopped);
  EXPECT_DOUBLE_EQ(lingtu::nav::endpoint::linearSpeed(vetoed.cmd), 0.0);

  authority.holdOperatorTakeover();
  EXPECT_TRUE(authority.operatorTakeoverLatched());
  ASSERT_TRUE(authority.resumeAutonomy());
  EXPECT_FALSE(authority.pathActive());
  EXPECT_FALSE(authority.teleopRequest().has_value());
  EXPECT_TRUE(authority.activatePath());
}

}  // namespace
