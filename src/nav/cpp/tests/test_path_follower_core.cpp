#include <gtest/gtest.h>
#include "nav_kernel/path_follower_core.hpp"
#include <algorithm>
#include <cmath>

using namespace nav_kernel;

// ── adaptiveLookAhead ──

TEST(AdaptiveLookAhead, ZeroSpeed) {
  PathFollowerParams p;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  EXPECT_DOUBLE_EQ(adaptiveLookAhead(0.0, p), 0.3);
}

TEST(AdaptiveLookAhead, HighSpeed) {
  PathFollowerParams p;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  p.maxLookAheadDis = 2.0;
  // speed=5 → 0.3 + 0.5*5 = 2.8, clamped to 2.0
  EXPECT_DOUBLE_EQ(adaptiveLookAhead(5.0, p), 2.0);
}

TEST(AdaptiveLookAhead, NormalSpeed) {
  PathFollowerParams p;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  // speed=0.6 → 0.3 + 0.5*0.6 = 0.6
  EXPECT_DOUBLE_EQ(adaptiveLookAhead(0.6, p), 0.6);
}

TEST(AdaptiveLookAhead, MinClamp) {
  PathFollowerParams p;
  p.baseLookAheadDis = 0.05;
  p.lookAheadRatio = 0.1;
  p.minLookAheadDis = 0.2;
  // speed=0.1 → 0.05 + 0.1*0.1 = 0.06, clamped to 0.2
  EXPECT_DOUBLE_EQ(adaptiveLookAhead(0.1, p), 0.2);
}

// ── computeControl — straight path ──

class PathFollowerStraight : public ::testing::Test {
protected:
  void SetUp() override {
    p.maxSpeed = 1.0;
    p.maxAccel = 1.0;
    p.dirDiffThre = 0.1;
    p.stopDisThre = 0.2;
    p.omniDirGoalThre = 1.0;
    p.omniDirDiffThre = 1.5;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio = 0.5;

    // 直线路径 0→3m
    for (int i = 0; i <= 30; i++) {
      path.push_back({0.1 * i, 0.0, 0.0});
    }
  }

  PathFollowerParams p;
  PathFollowerState  state;
  std::vector<Vec3>  path;
};

TEST_F(PathFollowerStraight, CanAccelForward) {
  Vec3 robot{0.0, 0.0, 0.0};
  // Use two updates so the command passes the minimum non-zero speed threshold.
  computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);
  auto out = computeControl(robot, 0.0, path, 1.0, 0.05, 1.0, 0, p, state);
  EXPECT_TRUE(out.canAccel);
  EXPECT_GT(out.cmd.vx, 0.0);
  EXPECT_NEAR(out.dirDiff, 0.0, 0.01);
}

TEST_F(PathFollowerStraight, AccelerationUsesElapsedControlTime) {
  Vec3 robot{0.0, 0.0, 0.0};
  p.nominalDt = 0.05;  // 20 Hz fallback for the first control sample.
  const auto first = computeControl(robot, 0.0, path, 1.0, 10.0, 1.0, 0, p, state);
  const auto second = computeControl(robot, 0.0, path, 1.0, 10.05, 1.0, 0, p, state);

  EXPECT_NEAR(state.vehicleSpeed, 0.10, 1e-6);
  EXPECT_NEAR(first.cmd.vx, 0.05, 1e-6);
  EXPECT_NEAR(second.cmd.vx, 0.10, 1e-6);
}

TEST_F(PathFollowerStraight, AccelerationClampsLongControlGap) {
  Vec3 robot{0.0, 0.0, 0.0};
  p.nominalDt = 0.05;
  p.maxDt = 0.10;
  computeControl(robot, 0.0, path, 1.0, 10.0, 1.0, 0, p, state);
  const auto after_stall =
      computeControl(robot, 0.0, path, 1.0, 10.5, 1.0, 0, p, state);

  EXPECT_NEAR(state.vehicleSpeed, 0.15, 1e-6);
  EXPECT_NEAR(after_stall.cmd.vx, 0.15, 1e-6);
}

TEST_F(PathFollowerStraight, PersistsTargetIndexAndTracksPathSize) {
  Vec3 robot{0.0, 0.0, 0.0};
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_TRUE(out.canAccel);
  EXPECT_EQ(state.lastPathSize, static_cast<int>(path.size()));
  EXPECT_EQ(state.lastPathPointID, state.pathPointID);
  EXPECT_GT(state.pathPointID, 0);

  std::vector<Vec3> shorter_path = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  computeControl(robot, 0.0, shorter_path, 1.0, 0.1, 1.0, 0, p, state);

  EXPECT_EQ(state.lastPathSize, static_cast<int>(shorter_path.size()));
  EXPECT_EQ(state.lastPathPointID, state.pathPointID);
  EXPECT_LT(state.pathPointID, static_cast<int>(shorter_path.size()));
}

TEST_F(PathFollowerStraight, StopsAtGoal) {
  Vec3 robot{2.95, 0.0, 0.0};  // 接近终点
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);
  // endDis ≈ 0.05 < stopDisThre, canAccel=false
  EXPECT_FALSE(out.canAccel);
}

TEST_F(PathFollowerStraight, SafetyStopLevel1) {
  Vec3 robot{0.0, 0.0, 0.0};
  auto out = computeControl(robot, 0.4, path, 1.0, 0.0, 1.0, 1, p, state);
  // safetyStop=1: linear speed = 0, yaw rate preserved
  EXPECT_DOUBLE_EQ(state.vehicleSpeed, 0.0);
  EXPECT_NE(out.cmd.wz, 0.0);
}

TEST_F(PathFollowerStraight, SafetyStopLevel2) {
  Vec3 robot{0.0, 0.0, 0.0};
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 2, p, state);
  EXPECT_DOUBLE_EQ(state.vehicleSpeed, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0);
}

// ── computeControl — direction threshold ──

TEST(PathFollowerDirThre, LargeAngleStopsLinear) {
  PathFollowerParams p;
  p.dirDiffThre = 0.1;  // ~5.7°
  p.omniDirGoalThre = 1.0;
  p.omniDirDiffThre = 1.5;
  p.stopDisThre = 0.2;
  PathFollowerState state;

  // Path going forward (+x), but robot facing 90° left
  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};
  double yawDiff = M_PI / 2;  // 90° error

  auto out = computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);
  // dirDiff ≈ 90° > dirDiffThre(5.7°), dis > omniDirGoalThre? No, dis≈3 > 1
  // canAccel needs: |dirDiff| < 0.1 OR (dis<1.0 && |dirDiff|<1.5)
  // |dirDiff|≈1.57 > 0.1 and dis≈3 > 1.0, so canAccel=false
  EXPECT_FALSE(out.canAccel);
  // But yaw rate should be nonzero (turning to correct heading)
  EXPECT_NE(out.cmd.wz, 0.0);
}

TEST(PathFollowerDirThre, OmniExceptionNearGoal) {
  PathFollowerParams p;
  p.dirDiffThre = 0.1;
  p.omniDirGoalThre = 1.0;
  p.omniDirDiffThre = 1.5;  // ~86°
  p.stopDisThre = 0.2;
  p.baseLookAheadDis = 0.1;
  p.lookAheadRatio = 0.1;
  p.minLookAheadDis = 0.1;
  PathFollowerState state;

  // 短路径 — 目标 0.6m 前方, robot facing 45° off
  std::vector<Vec3> path = {{0.3, 0.0, 0.0}, {0.6, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};
  double yawDiff = M_PI / 4;  // 45°

  auto out = computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);
  // dis ≈ 0.6 < omniDirGoalThre(1.0) && |dirDiff|≈0.785 < omniDirDiffThre(1.5)
  // && dis > stopDisThre(0.2) → canAccel=true (omni exception!)
  EXPECT_TRUE(out.canAccel);
}

// ── computeControl — two-way drive ──

TEST(PathFollowerTwoWay, ReverseWhenBehind) {
  PathFollowerParams p;
  p.twoWayDrive = true;
  p.switchTimeThre = 0.0;  // 无延迟
  PathFollowerState state;

  // 路径在机器人正后方
  std::vector<Vec3> path = {{-1.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, 0.0, path, 1.0, 1.0, 1.0, 0, p, state);
  EXPECT_FALSE(state.navFwd);
}

// ── computeControl — omni velocity decomposition ──

TEST(PathFollowerOmni, VelocityDecomposition) {
  PathFollowerParams p;
  p.omniDirGoalThre = 1.0;
  p.dirDiffThre = 0.1;
  p.stopDisThre = 0.2;
  p.maxAccel = 100.0;  // 让 stepToward 一步到位

  PathFollowerState state;
  state.vehicleSpeed = 0.5;  // 已有速度

  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};
  double yawDiff = 0.3;  // ~17° heading error

  auto out = computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);
  // omni: vx = cos(dirDiff)*speed, vy = -sin(dirDiff)*speed
  if (std::fabs(state.vehicleSpeed) > 1e-4) {
    EXPECT_NEAR(out.cmd.vx, std::cos(out.dirDiff) * state.vehicleSpeed, 0.01);
    EXPECT_NEAR(out.cmd.vy, -std::sin(out.dirDiff) * state.vehicleSpeed, 0.01);
  }
}

// ── computeControl — slow factor ──

TEST(PathFollowerSlow, SlowFactorReducesSpeed) {
  PathFollowerParams p;
  p.maxAccel = 100.0;
  p.dirDiffThre = 0.5;
  PathFollowerState state;

  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out1 = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);
  double speed1 = state.vehicleSpeed;

  state = {};
  auto out2 = computeControl(robot, 0.0, path, 1.0, 0.0, 0.25, 0, p, state);
  double speed2 = state.vehicleSpeed;

  EXPECT_LT(speed2, speed1);
}

TEST(PathFollowerSlow, MinimumSpeedPreventsFarPathStall) {
  PathFollowerParams p;
  p.maxSpeed = 0.2;
  p.minSpeed = 0.08;
  p.maxAccel = 100.0;
  p.dirDiffThre = 0.5;
  PathFollowerState state;
  const std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};

  const auto out = computeControl(
      {0.0, 0.0, 0.0}, 0.0, path, 0.1, 0.0, 1.0, 0, p, state);

  EXPECT_TRUE(out.canAccel);
  EXPECT_NEAR(out.cmd.vx, 0.08, 1e-6);
}

TEST(PathFollowerSlow, MinimumSpeedDoesNotDefeatGoalSlowdown) {
  PathFollowerParams p;
  p.maxSpeed = 0.2;
  p.minSpeed = 0.08;
  p.maxAccel = 100.0;
  p.dirDiffThre = 0.5;
  p.stopDisThre = 0.01;
  p.slowDwnDisThre = 1.0;
  PathFollowerState state;
  const std::vector<Vec3> path = {{0.025, 0.0, 0.0}, {0.05, 0.0, 0.0}};

  const auto out = computeControl(
      {0.0, 0.0, 0.0}, 0.0, path, 0.1, 0.0, 1.0, 0, p, state);

  EXPECT_TRUE(out.canAccel);
  EXPECT_NEAR(out.cmd.vx, 0.004, 1e-6);
}

TEST(PathFollowerSlow, TurnSpeedCouplingReducesHighYawLinearSpeed) {
  PathFollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 100.0;
  p.maxYawRate = 45.0;
  p.dirDiffThre = 1.0;
  p.turnSpeedYawRateStart = 0.10;
  p.turnSpeedMinScale = 0.40;
  PathFollowerState state;

  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, -0.20, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_NEAR(out.turnSpeedScale, 0.40, 1e-6);
  EXPECT_NEAR(state.vehicleSpeed, 0.40, 1e-6);
  EXPECT_GT(std::fabs(out.cmd.wz), p.turnSpeedYawRateStart);
}

TEST(PathFollowerSlow, TurnSpeedCouplingDefaultsToNoSpeedChange) {
  PathFollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 100.0;
  p.maxYawRate = 45.0;
  p.dirDiffThre = 1.0;
  PathFollowerState state;

  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, -0.20, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_NEAR(out.turnSpeedScale, 1.0, 1e-6);
  EXPECT_NEAR(state.vehicleSpeed, 1.0, 1e-6);
}

// ── computeControl — empty path ──

TEST(PathFollowerEdge, EmptyPath) {
  PathFollowerParams p;
  PathFollowerState state;
  std::vector<Vec3> path;
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0);
}

TEST(PathFollowerEdge, SinglePointPath) {
  PathFollowerParams p;
  PathFollowerState state;
  std::vector<Vec3> path = {{1.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);
  // pathSize <= 1 → joySpeed2 = 0, yawRate = 0
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0);
}

TEST(PathFollowerGroundTruth, FixedOffsetCommandMatchesGolden) {
  PathFollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 50.0;
  p.maxYawRate = 90.0;
  p.dirDiffThre = 1.5;
  p.twoWayDrive = false;
  PathFollowerState state;
  state.vehicleSpeed = 1.0;

  const std::vector<Vec3> path = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const Vec3 robot{0.0, 0.2, 0.0};

  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_TRUE(out.canAccel);
  EXPECT_NEAR(out.dirDiff, 0.19739556, 1e-6);
  EXPECT_NEAR(out.cmd.vx, 0.98058068, 1e-6);
  EXPECT_NEAR(out.cmd.vy, -0.19611614, 1e-6);
  EXPECT_NEAR(out.cmd.wz, -1.48046673, 1e-6);
  EXPECT_NEAR(state.vehicleSpeed, 1.0, 1e-6);
}

TEST(PathFollowerGroundTruth, ClosedLoopSimulationConverges) {
  PathFollowerParams p;
  p.maxSpeed = 0.8;
  p.maxAccel = 4.0;
  p.maxYawRate = 90.0;
  p.dirDiffThre = 0.6;
  p.baseLookAheadDis = 0.35;
  p.minLookAheadDis = 0.25;
  p.maxLookAheadDis = 1.0;
  p.twoWayDrive = false;
  PathFollowerState state;

  std::vector<Vec3> path;
  for (int i = 0; i <= 50; ++i) {
    path.push_back({0.1 * i, 0.0, 0.0});
  }

  Vec3 robot{0.0, 0.35, 0.0};
  double yaw = 0.0;
  double min_goal_distance = std::hypot(path.back().x - robot.x, path.back().y - robot.y);
  double max_speed_seen = 0.0;
  double max_yaw_seen = 0.0;
  constexpr double dt = 0.01;

  for (int tick = 0; tick < 1000; ++tick) {
    auto out = computeControl(robot, yaw, path, 1.0, tick * dt, 1.0, 0, p, state);
    max_speed_seen = std::max(max_speed_seen, std::hypot(out.cmd.vx, out.cmd.vy));
    max_yaw_seen = std::max(max_yaw_seen, std::fabs(out.cmd.wz));

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    robot.x += (cos_yaw * out.cmd.vx - sin_yaw * out.cmd.vy) * dt;
    robot.y += (sin_yaw * out.cmd.vx + cos_yaw * out.cmd.vy) * dt;
    yaw += out.cmd.wz * dt;

    const double goal_distance = std::hypot(path.back().x - robot.x, path.back().y - robot.y);
    min_goal_distance = std::min(min_goal_distance, goal_distance);
  }

  EXPECT_LT(min_goal_distance, 0.35);
  EXPECT_LT(std::fabs(robot.y), 0.12);
  EXPECT_LE(max_speed_seen, p.maxSpeed + 1e-6);
  EXPECT_LE(max_yaw_seen, p.maxYawRate * M_PI / 180.0 + 1e-6);
}
