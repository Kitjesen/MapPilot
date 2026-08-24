#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "tracking/follower.hpp"

using namespace nav_kernel;

namespace {

FollowerOutput followPath(Follower &follower, const std::vector<Vec3> &path,
                          const FollowerParams &params, double time, Vec3 vehicle = {},
                          double yaw = 0.0, double requested_speed = 1.0, double slow_factor = 1.0,
                          int safety_stop = 0, double goal_distance = -1.0) {
  FollowerInput input(path);
  input.vehicleRelative = vehicle;
  input.vehicleYawRelative = yaw;
  input.requestedSpeed = requested_speed;
  input.currentTime = time;
  input.slowFactor = slow_factor;
  input.safetyStop = safety_stop;
  input.params = params;
  input.goalDistance = goal_distance;
  return follower.follow(input);
}

FollowerOutput followTrajectory(Follower &follower, const std::vector<TrajectoryPoint> &trajectory,
                                const FollowerParams &params, double time, Twist measured = {},
                                double slow_factor = 1.0, int safety_stop = 0) {
  FollowerInput input(trajectory);
  input.measuredBodyTwist = measured;
  input.currentTime = time;
  input.slowFactor = slow_factor;
  input.safetyStop = safety_stop;
  input.params = params;
  return follower.follow(input);
}

std::vector<Vec3> straightPath(double length = 3.0) {
  std::vector<Vec3> path;
  for (int i = 0; i <= static_cast<int>(length * 10.0); ++i) {
    path.push_back({0.1 * i, 0.0, 0.0});
  }
  return path;
}

}  // namespace

TEST(FollowerPath, AcceleratesUsingElapsedControlTime) {
  FollowerParams params;
  params.maxSpeed = 1.0;
  params.maxAccel = 1.0;
  params.nominalDt = 0.05;
  Follower follower;
  const auto path = straightPath();

  const auto first = followPath(follower, path, params, 10.0);
  const auto second = followPath(follower, path, params, 10.05);

  EXPECT_NEAR(first.cmd.vx, 0.05, 1e-6);
  EXPECT_NEAR(second.cmd.vx, 0.10, 1e-6);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Path);
  EXPECT_NEAR(follower.diagnostics().linearSpeed, 0.10, 1e-6);
}

TEST(FollowerPath, ClampsLongControlGap) {
  FollowerParams params;
  params.maxAccel = 1.0;
  params.nominalDt = 0.05;
  params.maxDt = 0.10;
  Follower follower;
  const auto path = straightPath();

  followPath(follower, path, params, 10.0);
  const auto output = followPath(follower, path, params, 10.5);

  EXPECT_NEAR(output.cmd.vx, 0.15, 1e-6);
}

TEST(FollowerPath, StopsAtGoalAndHonorsSafetyLevels) {
  FollowerParams params;
  params.maxAccel = 100.0;
  const auto path = straightPath();

  Follower at_goal;
  const auto goal = followPath(at_goal, path, params, 0.0, {2.95, 0.0, 0.0});
  EXPECT_FALSE(goal.canAccelerate);

  Follower linear_stop;
  const auto level_one = followPath(linear_stop, path, params, 0.0, {}, 0.4, 1.0, 1.0, 1);
  EXPECT_DOUBLE_EQ(level_one.cmd.vx, 0.0);
  EXPECT_NE(level_one.cmd.wz, 0.0);

  Follower full_stop;
  const auto level_two = followPath(full_stop, path, params, 0.0, {}, 0.4, 1.0, 1.0, 2);
  EXPECT_DOUBLE_EQ(level_two.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(level_two.cmd.wz, 0.0);
}

TEST(FollowerPath, DirectionThresholdControlsLinearAcceleration) {
  FollowerParams params;
  params.dirDiffThre = 0.1;
  params.omniDirGoalThre = 1.0;
  params.omniDirDiffThre = 1.5;
  params.twoWayDrive = false;
  Follower follower;
  const std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};

  const auto output = followPath(follower, path, params, 0.0, {}, M_PI / 2.0);

  EXPECT_FALSE(output.canAccelerate);
  EXPECT_DOUBLE_EQ(output.cmd.vx, 0.0);
  EXPECT_NE(output.cmd.wz, 0.0);
}

TEST(FollowerPath, OmniMotionCanTranslateWithModerateHeadingError) {
  FollowerParams params;
  params.dirDiffThre = 0.1;
  params.omniDirGoalThre = 1.0;
  params.omniDirDiffThre = 1.5;
  params.stopDisThre = 0.2;
  params.maxAccel = 100.0;
  Follower follower;
  const std::vector<Vec3> path = {{0.3, 0.0, 0.0}, {0.6, 0.0, 0.0}};

  const auto output = followPath(follower, path, params, 0.0, {}, M_PI / 4.0);

  EXPECT_TRUE(output.canAccelerate);
  EXPECT_GT(std::hypot(output.cmd.vx, output.cmd.vy), 0.0);
}

TEST(FollowerPath, ReversesWhenTargetIsBehind) {
  FollowerParams params;
  params.twoWayDrive = true;
  params.switchTimeThre = 0.0;
  params.maxAccel = 100.0;
  Follower follower;
  const std::vector<Vec3> path = {{-1.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}};

  const auto output = followPath(follower, path, params, 1.0);

  EXPECT_LT(output.cmd.vx, 0.0);
  EXPECT_FALSE(follower.diagnostics().forward);
}

TEST(FollowerPath, SlowFactorMinimumSpeedAndTurnCouplingCompose) {
  FollowerParams params;
  params.maxSpeed = 1.0;
  params.minSpeed = 0.08;
  params.maxAccel = 100.0;
  params.maxYawRate = 45.0;
  params.dirDiffThre = 1.0;
  params.turnSpeedYawRateStart = 0.10;
  params.turnSpeedMinScale = 0.40;
  Follower follower;
  const auto path = straightPath(5.0);

  const auto output = followPath(follower, path, params, 0.0, {}, -0.20, 0.1, 0.5);

  EXPECT_NEAR(output.turnSpeedScale, 0.40, 1e-6);
  EXPECT_NEAR(std::hypot(output.cmd.vx, output.cmd.vy), 0.02, 1e-6);
}

TEST(FollowerPath, EmptyAndSinglePointTargetsStayStopped) {
  FollowerParams params;
  Follower empty_follower;
  const std::vector<Vec3> empty;
  const auto empty_output = followPath(empty_follower, empty, params, 0.0);
  EXPECT_DOUBLE_EQ(empty_output.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(empty_output.cmd.wz, 0.0);

  Follower single_follower;
  const std::vector<Vec3> single{{1.0, 0.0, 0.0}};
  const auto single_output = followPath(single_follower, single, params, 0.0);
  EXPECT_DOUBLE_EQ(single_output.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(single_output.cmd.wz, 0.0);
  EXPECT_NEAR(single_output.endDistance, 1.0, 1e-9);
}

TEST(FollowerPath, FixedOffsetCommandMatchesGolden) {
  FollowerParams params;
  params.maxSpeed = 1.0;
  params.maxAccel = 1000.0;
  params.maxYawRate = 90.0;
  params.dirDiffThre = 1.5;
  params.twoWayDrive = false;
  Follower follower;
  const std::vector<Vec3> path = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};

  const auto output = followPath(follower, path, params, 0.0, {0.0, 0.2, 0.0});

  EXPECT_TRUE(output.canAccelerate);
  EXPECT_NEAR(output.directionError, 0.19739556, 1e-6);
  EXPECT_NEAR(output.cmd.vx, 0.98058068, 1e-6);
  EXPECT_NEAR(output.cmd.vy, -0.19611614, 1e-6);
  EXPECT_NEAR(output.cmd.wz, -1.48046673, 1e-6);
}

TEST(FollowerTrajectory, RegisteredAlgorithmTracksTimedTrajectory) {
  FollowerParams params;
  params.maxSpeed = 0.8;
  params.maxAccel = 1.0;
  params.nominalDt = 0.05;
  params.trajectoryLookAheadS = 0.20;
  params.trajectoryPositionGain = 1.0;
  std::vector<TrajectoryPoint> trajectory(3);
  trajectory[0].position = {0.05, 0.01, 0.0};
  trajectory[0].velocity = {0.10, 0.0, 0.0};
  trajectory[0].timeFromStartS = 0.0;
  trajectory[1].position = {0.10, 0.05, 0.02};
  trajectory[1].velocity = {0.30, 0.10, 0.05};
  trajectory[1].yaw = 0.10;
  trajectory[1].timeFromStartS = 0.20;
  trajectory[2] = trajectory[1];
  trajectory[2].position = {0.30, 0.12, 0.08};
  trajectory[2].timeFromStartS = 0.60;
  Follower follower;

  const auto output = followTrajectory(follower, trajectory, params, 1.0);

  EXPECT_GT(output.cmd.vx, 0.0);
  EXPECT_GT(output.cmd.vy, 0.0);
  EXPECT_GT(output.cmd.wz, 0.0);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Trajectory);
}

TEST(FollowerTrajectory, UsesCurrentSampleForTranslationAndLookaheadForHeading) {
  FollowerParams params;
  params.maxSpeed = 1.0;
  params.maxAccel = 100.0;
  params.trajectoryLookAheadS = 0.20;
  params.trajectoryPositionGain = 1.0;
  std::vector<TrajectoryPoint> trajectory(2);
  trajectory[0].position = {0.05, 0.0, 0.0};
  trajectory[0].velocity = {0.10, 0.0, 0.0};
  trajectory[1].position = {0.80, 0.0, 0.0};
  trajectory[1].velocity = {0.30, 0.0, 0.0};
  trajectory[1].timeFromStartS = 0.20;
  Follower follower;

  const auto output = followTrajectory(follower, trajectory, params, 1.0);

  EXPECT_FALSE(output.executionFrozen);
  EXPECT_NEAR(output.cmd.vx, 0.15, 1e-6);
  EXPECT_NEAR(output.cmd.vy, 0.0, 1e-6);
  EXPECT_NEAR(output.directionError, 0.0, 1e-6);
}

TEST(FollowerTrajectory, FreezesTranslationUntilPlannedHeadingIsAligned) {
  FollowerParams params;
  params.maxSpeed = 0.8;
  params.maxAccel = 1.0;
  params.nominalDt = 0.05;
  params.maxYawRate = 60.0;
  params.trajectoryLookAheadS = 0.20;
  params.trajectoryHeadingErrorThreshold = 0.40;
  std::vector<TrajectoryPoint> trajectory(3);
  trajectory[1].position = {0.0, 0.4, 0.0};
  trajectory[1].velocity = {0.0, 0.3, 0.0};
  trajectory[1].yaw = 0.5 * M_PI;
  trajectory[1].timeFromStartS = 0.20;
  trajectory[2] = trajectory[1];
  trajectory[2].position = {0.0, 0.8, 0.0};
  trajectory[2].timeFromStartS = 0.60;
  Follower follower;

  const auto output = followTrajectory(follower, trajectory, params, 1.0);

  EXPECT_TRUE(output.executionFrozen);
  EXPECT_DOUBLE_EQ(output.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.vy, 0.0);
  EXPECT_GT(output.cmd.wz, 0.0);
  EXPECT_FALSE(output.canAccelerate);
}

TEST(FollowerRegistry, SwitchesAlgorithmsBehindOneFollowInterface) {
  FollowerParams params;
  params.maxAccel = 10.0;
  params.nominalDt = 0.05;
  Follower follower;
  const auto path = straightPath();
  std::vector<TrajectoryPoint> trajectory(2);
  trajectory[1].position = {0.5, 0.0, 0.0};
  trajectory[1].velocity = {0.2, 0.0, 0.0};
  trajectory[1].timeFromStartS = 0.5;

  followPath(follower, path, params, 0.0);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Path);

  followTrajectory(follower, trajectory, params, 0.1);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Trajectory);

  const auto returned = followPath(follower, path, params, 0.2);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Path);
  EXPECT_NEAR(returned.cmd.vx, params.maxAccel * params.nominalDt, 1e-6);
}

TEST(FollowerLifecycle, StopAndResetAreAlgorithmIndependent) {
  FollowerParams params;
  params.maxAccel = 100.0;
  Follower follower;
  const auto path = straightPath();

  followPath(follower, path, params, 0.0);
  ASSERT_GT(follower.diagnostics().linearSpeed, 0.0);
  follower.stopLinear();
  EXPECT_DOUBLE_EQ(follower.diagnostics().linearSpeed, 0.0);

  follower.reset();
  EXPECT_FALSE(follower.diagnostics().active);
}

TEST(FollowerPath, ClosedLoopSimulationConverges) {
  FollowerParams params;
  params.maxSpeed = 0.8;
  params.maxAccel = 4.0;
  params.maxYawRate = 90.0;
  params.dirDiffThre = 0.6;
  params.baseLookAheadDis = 0.35;
  params.minLookAheadDis = 0.25;
  params.maxLookAheadDis = 1.0;
  params.twoWayDrive = false;
  Follower follower;
  const auto path = straightPath(5.0);

  Vec3 robot{0.0, 0.35, 0.0};
  double yaw = 0.0;
  double min_goal_distance = std::hypot(path.back().x - robot.x, path.back().y - robot.y);
  constexpr double dt = 0.01;

  for (int tick = 0; tick < 1000; ++tick) {
    const auto output = followPath(follower, path, params, tick * dt, robot, yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    robot.x += (cos_yaw * output.cmd.vx - sin_yaw * output.cmd.vy) * dt;
    robot.y += (sin_yaw * output.cmd.vx + cos_yaw * output.cmd.vy) * dt;
    yaw += output.cmd.wz * dt;
    min_goal_distance =
        std::min(min_goal_distance, std::hypot(path.back().x - robot.x, path.back().y - robot.y));
  }

  EXPECT_LT(min_goal_distance, 0.35);
  EXPECT_LT(std::fabs(robot.y), 0.12);
}
