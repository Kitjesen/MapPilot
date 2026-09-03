#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "navigation/executor.hpp"

namespace {

lingtu::nav::navigation::Executor makeConfiguredExecutor(
    lingtu::nav::navigation::ExecutorConfig config,
    nav_kernel::LocalPlannerParams planner_params, const char *path_library) {
  nav_kernel::local::Planner planner(planner_params);
  EXPECT_TRUE(planner.configure(path_library));
  return lingtu::nav::navigation::Executor(std::move(config), std::move(planner));
}

nav_kernel::Pose pose(double yaw) {
  nav_kernel::Pose value;
  value.yaw = yaw;
  return value;
}

lingtu::nav::navigation::ExecutionInput intentInput(
    const nav_kernel::Pose &body, const nav_kernel::Twist &intent,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s) {
  lingtu::nav::navigation::ExecutionInput input;
  input.mode = lingtu::nav::navigation::ExecutionMode::MotionIntent;
  input.mapBody = body;
  input.odomBody = body;
  input.obstacleXyzhMap = obstacle_xyzh;
  input.obstacleCount = obstacle_count;
  input.timestampS = timestamp_s;
  input.motionIntent = intent;
  return input;
}

lingtu::nav::navigation::Executor makeExecutor() {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.max_speed = 0.5;
  config.teleop_intent_max_deviation_deg = 90.0;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.twoWayDrive = false;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 1.0;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 10.0;
  config.follower.nominalDt = 0.05;
  return makeConfiguredExecutor(std::move(config), planner, LINGTU_TEST_GO2_PATH_LIBRARY);
}

std::vector<float> forwardObstacle() {
  std::vector<float> points;
  for (int index = -2; index <= 2; ++index) {
    points.push_back(0.9F);
    points.push_back(0.08F * static_cast<float>(index));
    points.push_back(0.0F);
    points.push_back(1.0F);
  }
  return points;
}

lingtu::nav::navigation::Executor makeAcceptanceExecutor() {
  lingtu::nav::navigation::ExecutorConfig config;
  nav_kernel::LocalPlannerParams planner;
  config.max_speed = 0.5;
  config.teleop_intent_horizon_m = 3.5;
  config.teleop_intent_max_deviation_deg = 90.0;
  planner.checkObstacle = true;
  planner.useTerrainAnalysis = true;
  planner.useTraversabilityCost = false;
  planner.twoWayDrive = false;
  planner.autonomySpeed = 0.5;
  planner.maxSpeed = 0.5;
  planner.adjacentRange = 3.0;
  planner.vehicleLength = 1.0;
  planner.vehicleWidth = 0.6;
  planner.footprintPadding = 0.15;
  config.follower.maxSpeed = 0.5;
  config.follower.maxAccel = 2.0;
  config.follower.nominalDt = 0.05;
  return makeConfiguredExecutor(std::move(config), planner,
                                LINGTU_TEST_THUNDER_PATH_LIBRARY);
}

std::vector<float> acceptanceObstacle() {
  std::vector<float> points;
  for (int index = -11; index <= 11; ++index) {
    points.push_back(1.55F);
    points.push_back(0.05F * static_cast<float>(index));
    points.push_back(0.65F);
    points.push_back(0.65F);
  }
  return points;
}

std::vector<float> boxObstacle() {
  constexpr float kMinX = 1.55F;
  constexpr float kMaxX = 2.45F;
  constexpr float kMinY = -0.55F;
  constexpr float kMaxY = 0.55F;
  constexpr float kStep = 0.05F;
  std::vector<float> points;
  for (float x = kMinX; x <= kMaxX + 1e-5F; x += kStep) {
    points.insert(points.end(), {x, kMinY, 0.65F, 0.65F});
    points.insert(points.end(), {x, kMaxY, 0.65F, 0.65F});
  }
  for (float y = kMinY + kStep; y < kMaxY - 1e-5F; y += kStep) {
    points.insert(points.end(), {kMinX, y, 0.65F, 0.65F});
    points.insert(points.end(), {kMaxX, y, 0.65F, 0.65F});
  }
  return points;
}

bool overlapsInflatedObstacle(const nav_kernel::Pose &body) {
  constexpr double kBodyHalfLength = 0.50;
  constexpr double kBodyHalfWidth = 0.30;
  constexpr double kObstacleCenterX = 2.00;
  constexpr double kObstacleCenterY = 0.00;
  constexpr double kObstacleHalfLength = 0.45 + 0.08;
  constexpr double kObstacleHalfWidth = 0.55 + 0.08;

  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  const double dx = body.position.x - kObstacleCenterX;
  const double dy = body.position.y - kObstacleCenterY;
  const double axes[4][2] = {{1.0, 0.0}, {0.0, 1.0}, {c, s}, {-s, c}};
  for (const auto &axis : axes) {
    const double center_distance = std::abs(dx * axis[0] + dy * axis[1]);
    const double body_radius =
        kBodyHalfLength * std::abs(c * axis[0] + s * axis[1]) +
        kBodyHalfWidth * std::abs(-s * axis[0] + c * axis[1]);
    const double obstacle_radius =
        kObstacleHalfLength * std::abs(axis[0]) +
        kObstacleHalfWidth * std::abs(axis[1]);
    if (center_distance > body_radius + obstacle_radius) {
      return false;
    }
  }
  return true;
}

TEST(CmuTeleop, UsesCmuFollower) {
  auto executor = makeExecutor();
  const auto obstacles = forwardObstacle();
  nav_kernel::Twist intent;
  intent.vx = 0.3;

  const auto selected = executor.tick(intentInput(
      pose(0.0), intent, obstacles.data(), static_cast<int>(obstacles.size() / 4), 1.0));

  ASSERT_TRUE(selected.path_found) << selected.reason;
  ASSERT_GE(selected.local_path_body.size(), 2U);
  const auto lookahead = std::find_if(
      selected.local_path_body.begin(), selected.local_path_body.end(),
      [](const nav_kernel::Vec3 &point) { return std::hypot(point.x, point.y) >= 0.5; });
  ASSERT_NE(lookahead, selected.local_path_body.end());
  const double path_heading = std::atan2(lookahead->y, lookahead->x);
  ASSERT_GT(std::abs(path_heading), 0.40);
  ASSERT_LT(std::abs(path_heading), 1.5);

  EXPECT_DOUBLE_EQ(selected.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(selected.cmd_vel.vy, 0.0);
  EXPECT_GT(std::abs(selected.cmd_vel.wz), 0.01);
  EXPECT_TRUE(selected.trajectory_frozen);
  EXPECT_EQ(selected.reason, "teleop_assist_heading_alignment");
  EXPECT_EQ(selected.recovery_action,
            static_cast<int>(nav_kernel::RecoveryAction::None));

  std::vector<float> newly_blocked;
  for (int degrees = -180; degrees < 180; degrees += 5) {
    const double angle = static_cast<double>(degrees) * M_PI / 180.0;
    newly_blocked.push_back(static_cast<float>(0.8 * std::cos(angle)));
    newly_blocked.push_back(static_cast<float>(0.8 * std::sin(angle)));
    newly_blocked.push_back(0.0F);
    newly_blocked.push_back(1.0F);
  }
  const auto replanned = executor.tick(intentInput(
      pose(0.0), intent, newly_blocked.data(),
      static_cast<int>(newly_blocked.size() / 4), 1.05));
  EXPECT_FALSE(replanned.path_found)
      << "a new near-field obstacle must be replanned instead of following a cached path";
  EXPECT_NE(replanned.reason, "teleop_assist_control_ready");
}

TEST(CmuTeleop, DetoursWithThunderFootprint) {
  auto executor = makeAcceptanceExecutor();
  const auto obstacles = acceptanceObstacle();
  nav_kernel::Twist intent;
  intent.vx = 0.5;

  const auto selected = executor.tick(intentInput(
      pose(0.0), intent, obstacles.data(), static_cast<int>(obstacles.size() / 4), 1.0));

  ASSERT_TRUE(selected.path_found) << selected.reason << ": "
                                   << selected.local_planner_debug.searchReason;
  ASSERT_GE(selected.local_path_body.size(), 2U);
  const auto lateral = std::max_element(
      selected.local_path_body.begin(), selected.local_path_body.end(),
      [](const nav_kernel::Vec3 &lhs, const nav_kernel::Vec3 &rhs) {
        return std::abs(lhs.y) < std::abs(rhs.y);
      });
  ASSERT_NE(lateral, selected.local_path_body.end());
  EXPECT_GT(std::abs(lateral->y), 0.75)
      << "the Thunder correspondence library must reject the old Go2-sized detour";
}

TEST(CmuTeleop, RollsPastBoxWithPhysicalClearance) {
  constexpr double kObstacleMaxX = 2.45;
  auto executor = makeAcceptanceExecutor();
  const auto obstacles = boxObstacle();
  nav_kernel::Twist intent;
  intent.vx = 0.5;
  nav_kernel::Pose body = pose(0.0);
  constexpr double kDt = 0.05;

  bool collided = false;
  int path_ticks = 0;
  double max_lateral = 0.0;
  for (int tick = 0; tick < 900 && body.position.x < 14.0; ++tick) {
    const auto output = executor.tick(intentInput(
        body, intent, obstacles.data(), static_cast<int>(obstacles.size() / 4),
        1.0 + kDt * static_cast<double>(tick)));
    if (output.path_found) {
      ++path_ticks;
    }
    const double c = std::cos(body.yaw);
    const double s = std::sin(body.yaw);
    body.position.x += kDt * (c * output.cmd_vel.vx - s * output.cmd_vel.vy);
    body.position.y += kDt * (s * output.cmd_vel.vx + c * output.cmd_vel.vy);
    body.yaw = std::remainder(body.yaw + kDt * output.cmd_vel.wz, 2.0 * M_PI);
    max_lateral = std::max(max_lateral, std::abs(body.position.y));
    collided = collided || overlapsInflatedObstacle(body);
  }

  EXPECT_FALSE(collided)
      << "CMU planning and following must keep the physical Thunder body plus 8 cm clear";
  EXPECT_GT(path_ticks, 20);
  EXPECT_GT(max_lateral, 0.75);
  EXPECT_GT(body.position.x, 13.5)
      << "the rolling CMU plan must continue beyond the obstacle";
  EXPECT_LE(std::abs(body.position.y), 0.25)
      << "after the detour, assisted teleop must rejoin the admitted corridor";
  EXPECT_LE(std::abs(std::remainder(body.yaw, 2.0 * M_PI)), 0.35)
      << "a forward detour must not flip the follower into reverse travel";
}

TEST(CmuTeleop, PreservesAdmittedDirection) {
  auto executor = makeAcceptanceExecutor();
  nav_kernel::Twist intent;
  intent.vx = 0.5;

  const auto initial = executor.tick(intentInput(pose(0.0), intent, nullptr, 0, 1.0));
  ASSERT_TRUE(initial.path_found) << initial.reason;
  EXPECT_NEAR(initial.target.y, 0.0, 1e-9);

  auto detoured = pose(-0.5);
  detoured.position = {1.0, -1.0, 0.0};
  const auto replanned = executor.tick(intentInput(detoured, intent, nullptr, 0, 1.05));

  ASSERT_TRUE(replanned.path_found) << replanned.reason;
  EXPECT_NEAR(replanned.target.x, 4.5, 1e-9);
  EXPECT_NEAR(replanned.target.y, 0.0, 1e-9)
      << "planner-induced yaw must not rotate the admitted operator direction";
}

TEST(CmuTeleop, LateralIntentTurnsTowardSelectedPath) {
  auto executor = makeAcceptanceExecutor();
  nav_kernel::Twist intent;
  intent.vy = 0.5;

  const auto first = executor.tick(intentInput(pose(0.0), intent, nullptr, 0, 1.00));
  const auto second = executor.tick(intentInput(pose(0.0), intent, nullptr, 0, 1.05));

  ASSERT_TRUE(first.path_found) << first.reason;
  ASSERT_TRUE(second.path_found) << second.reason;
  EXPECT_NEAR(second.cmd_vel.vx, 0.0, 1e-9);
  EXPECT_NEAR(second.cmd_vel.vy, 0.0, 1e-9);
  EXPECT_GT(second.cmd_vel.wz, 0.0);
  EXPECT_TRUE(second.trajectory_frozen);
}

TEST(CmuTeleop, DirectionChangeBrakesBeforeFollowingNewPath) {
  auto executor = makeAcceptanceExecutor();
  nav_kernel::Twist forward;
  forward.vx = 0.5;
  nav_kernel::Twist lateral;
  lateral.vy = 0.5;

  auto previous = executor.tick(intentInput(pose(0.0), forward, nullptr, 0, 1.00));
  for (int step = 1; step <= 6; ++step) {
    previous = executor.tick(intentInput(pose(0.0), forward, nullptr, 0,
                                    1.00 + 0.05 * static_cast<double>(step)));
  }
  ASSERT_GT(previous.cmd_vel.vx, 0.40);

  const auto changed = executor.tick(intentInput(pose(0.0), lateral, nullptr, 0, 1.35));
  const double delta = std::hypot(changed.cmd_vel.vx - previous.cmd_vel.vx,
                                  changed.cmd_vel.vy - previous.cmd_vel.vy);

  ASSERT_TRUE(changed.path_found) << changed.reason;
  EXPECT_EQ(changed.reason, "teleop_assist_direction_transition");
  EXPECT_GT(changed.cmd_vel.vx * previous.cmd_vel.vx +
                changed.cmd_vel.vy * previous.cmd_vel.vy,
            0.0);
  EXPECT_LT(std::hypot(changed.cmd_vel.vx, changed.cmd_vel.vy),
            std::hypot(previous.cmd_vel.vx, previous.cmd_vel.vy));
  EXPECT_LE(delta, 2.0 * 0.05 + 1e-9);

  auto settled = changed;
  for (int step = 1; step <= 10; ++step) {
    settled = executor.tick(intentInput(pose(0.0), lateral, nullptr, 0,
                                   1.35 + 0.05 * static_cast<double>(step)));
  }
  EXPECT_NEAR(settled.cmd_vel.vx, 0.0, 1e-9);
  EXPECT_NEAR(settled.cmd_vel.vy, 0.0, 1e-9);
  EXPECT_GT(settled.cmd_vel.wz, 0.0);
  EXPECT_TRUE(settled.trajectory_frozen);
  EXPECT_NE(settled.reason, "teleop_assist_direction_transition");
}

}  // namespace
