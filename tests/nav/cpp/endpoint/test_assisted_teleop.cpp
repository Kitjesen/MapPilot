#include <algorithm>
#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>
#include <vector>

#include "control/authority.hpp"
#include "navigation/executor.hpp"

#ifndef LINGTU_TEST_PATH_LIBRARY
#error "LINGTU_TEST_PATH_LIBRARY must name the LocalPlanner path library"
#endif

namespace {

lingtu::nav::navigation::Executor makeConfiguredExecutor(
    lingtu::nav::navigation::ExecutorConfig config,
    nav_kernel::LocalPlannerParams planner_params,
    const std::string &path_library) {
  nav_kernel::local::Planner planner(planner_params);
  EXPECT_TRUE(planner.configure(path_library));
  return lingtu::nav::navigation::Executor(std::move(config), std::move(planner));
}

nav_kernel::Pose originPose() {
  nav_kernel::Pose pose;
  pose.yaw = 0.0;
  return pose;
}

lingtu::nav::navigation::ExecutionInput intentInput(
    const nav_kernel::Pose &body, const nav_kernel::Twist &intent,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
    lingtu::nav::navigation::TraversabilityGridView traversability = {}) {
  lingtu::nav::navigation::ExecutionInput input;
  input.mode = lingtu::nav::navigation::ExecutionMode::MotionIntent;
  input.mapBody = body;
  input.odomBody = body;
  input.obstacleXyzhMap = obstacle_xyzh;
  input.obstacleCount = obstacle_count;
  input.timestampS = timestamp_s;
  input.traversability = traversability;
  input.motionIntent = intent;
  return input;
}

lingtu::nav::navigation::Executor makeAssistedLoop() {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.max_speed = 0.4;
  config.teleop_intent_horizon_m = 2.0;
  config.teleop_intent_max_deviation_deg = 55.0;
  config.follower.maxAccel = 1.0;
  config.follower.nominalDt = 0.05;
  return makeConfiguredExecutor(std::move(config), planner,
                                LINGTU_TEST_PATH_LIBRARY);
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

std::string thunderPathLibrary() {
  return (std::filesystem::path(LINGTU_TEST_PATH_LIBRARY).parent_path() / "thunder").string();
}

TEST(AssistedTeleop, TakeoverDetoursAndResumeStaysFresh) {
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
  auto assisted = loop.tick(intentInput(originPose(), *authority.teleopRequest(), obstacles.data(),
                                        static_cast<int>(obstacles.size() / 4), 1.0));

  ASSERT_TRUE(assisted.path_found);
  ASSERT_GE(assisted.local_path_map.size(), 2U);
  EXPECT_EQ(assisted.recovery_state, 0);
  double max_lateral = 0.0;
  for (const auto &point : assisted.local_path_map) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.1);

  authority.holdOperatorTakeover();
  EXPECT_TRUE(authority.operatorTakeoverLatched());
  ASSERT_TRUE(authority.resumeMotion());
  EXPECT_FALSE(authority.pathActive());
  EXPECT_FALSE(authority.teleopRequest().has_value());
  EXPECT_TRUE(authority.activatePath());
}

TEST(AssistedTeleop, Go2ObstacleUsesCmuCurvedPath) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.max_speed = 0.4;
  config.teleop_intent_horizon_m = 2.0;
  config.teleop_intent_max_deviation_deg = 55.0;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = false;
  planner.vehicleLength = 0.76;
  planner.vehicleWidth = 0.31;
  planner.footprintPadding = 0.10;
  planner.nearFieldStopDis = 0.5;
  config.follower.maxSpeed = 0.4;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  auto loop = makeConfiguredExecutor(std::move(config), planner,
                                     LINGTU_TEST_PATH_LIBRARY);

  nav_kernel::Twist intent;
  intent.vx = 0.3;
  std::vector<float> obstacles;
  for (int i = -2; i <= 2; ++i) {
    obstacles.insert(obstacles.end(),
                     {0.90f, 0.08f * static_cast<float>(i), 0.0f, 1.0f});
  }
  const auto assisted = loop.tick(intentInput(originPose(), intent, obstacles.data(),
                                        static_cast<int>(obstacles.size() / 4), 1.0));
  ASSERT_TRUE(assisted.path_found) << assisted.reason;
  ASSERT_GE(assisted.local_path_map.size(), 3U);
  double max_lateral = 0.0;
  for (const auto &point : assisted.local_path_map) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.1);
  EXPECT_NEAR(assisted.cmd_vel.vy, 0.0, 1e-12);
  EXPECT_GT(std::abs(assisted.cmd_vel.wz), 0.01);
}

TEST(AssistedTeleop, ThunderWideObstacleStartsTraversabilitySafeLateralDetour) {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.max_speed = 0.5;
  config.teleop_intent_horizon_m = 3.5;
  config.teleop_intent_max_deviation_deg = 55.0;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = true;
  planner.vehicleLength = 1.0;
  planner.vehicleWidth = 0.6;
  planner.footprintPadding = 0.15;
  planner.nearFieldStopDis = 0.55;
  planner.traversabilityHardCost = 80.0;
  planner.traversabilitySoftCost = 40.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  auto loop = makeConfiguredExecutor(std::move(config), planner,
                                     thunderPathLibrary());

  std::vector<float> obstacles;
  for (int index = -6; index <= 6; ++index) {
    obstacles.insert(obstacles.end(), {1.55F, 0.10F * static_cast<float>(index), 0.30F, 0.80F});
  }

  constexpr int rows = 50;
  constexpr int cols = 50;
  constexpr double resolution = 0.20;
  constexpr double origin_x = -5.0;
  constexpr double origin_y = -5.0;
  std::vector<float> terrain(static_cast<std::size_t>(rows * cols), 0.0F);
  for (int row = 0; row < rows; ++row) {
    const double y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
    for (int col = 0; col < cols; ++col) {
      const double x = origin_x + (static_cast<double>(col) + 0.5) * resolution;
      if (x >= 1.5 && x <= 2.5 && std::abs(y) <= 0.6) {
        terrain[static_cast<std::size_t>(row * cols + col)] = 100.0F;
      }
    }
  }
  const lingtu::nav::navigation::TraversabilityGridView terrain_view{
      terrain.data(), rows, cols, resolution, origin_x, origin_y, 1U,
  };

  const nav_kernel::Twist intent{0.5, 0.0, 0.0};
  const auto assisted = loop.tick(intentInput(originPose(), intent, obstacles.data(),
                                        static_cast<int>(obstacles.size() / 4), 1.0,
                                        terrain_view));

  ASSERT_TRUE(assisted.path_found) << assisted.reason;
  ASSERT_GE(assisted.local_path_map.size(), 3U);
  double max_lateral = 0.0;
  for (const auto &point : assisted.local_path_map) {
    max_lateral = std::max(max_lateral, std::abs(point.y));
  }
  EXPECT_GT(max_lateral, 0.5);
}

}  // namespace
