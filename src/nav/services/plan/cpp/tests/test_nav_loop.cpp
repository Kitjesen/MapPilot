#include "nav_loop.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

lingtu::nav::plan::NavLoop makeLoop(bool use_traversability = false) {
  lingtu::nav::plan::NavLoopConfig config;
  config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
  config.corridor_lookahead_m = 2.0;
  config.max_speed = 0.5;
  config.local_planner.checkObstacle = true;
  config.local_planner.useTerrainAnalysis = true;
  config.local_planner.useTraversabilityCost = use_traversability;
  config.local_planner.autonomySpeed = 0.5;
  config.local_planner.maxSpeed = 1.0;
  config.path_follower.maxSpeed = 0.5;
  config.path_follower.maxAccel = 10.0;
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
  EXPECT_EQ(out.cmd_vel.vx, 0.0);
  EXPECT_EQ(out.cmd_vel.wz, 0.0);
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
      0.25f, 0.0f, 0.0f, 1.0f,
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
  risk_grid[2 * 5 + 0] = 95.0f;
  risk_grid[2 * 5 + 1] = 95.0f;
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
