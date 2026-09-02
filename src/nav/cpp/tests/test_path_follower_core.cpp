#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "planning/local/planner.hpp"
#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"
#include "tracking/follower.hpp"

using namespace nav_kernel;

namespace {

FollowerOutput followPath(Follower &follower, const std::vector<Vec3> &path,
                          const FollowerParams &params, double time, Vec3 vehicle = {},
                          double yaw = 0.0, double requested_speed = 1.0, double slow_factor = 1.0,
                          int safety_stop = 0, double goal_distance = -1.0) {
  FollowerState state;
  state.vehicleRelative = vehicle;
  state.vehicleYawRelative = yaw;
  state.requestedSpeed = requested_speed;
  state.currentTime = time;
  state.slowFactor = slow_factor;
  state.safetyStop = safety_stop;
  state.params = params;
  state.goalDistance = goal_distance;
  state.standardPathProfile = false;
  return follower.follow(LocalPlan::path(path), state);
}

FollowerOutput followSpline(Follower &follower, const SplineTarget &spline,
                            const FollowerParams &params, double time,
                            Vec3 vehicle = {}, double yaw = 0.0,
                            double requested_speed = 1.0, double slow_factor = 1.0) {
  FollowerState state;
  state.vehicleRelative = vehicle;
  state.vehicleYawRelative = yaw;
  state.requestedSpeed = requested_speed;
  state.currentTime = time;
  state.slowFactor = slow_factor;
  state.params = params;
  return follower.follow(LocalPlan::spline(spline), state);
}

SplineTarget scanSpline(std::vector<Vec3> controls, int degree, double interval,
                        std::int64_t trajectory_id = 1) {
  Eigen::MatrixXd matrix(3, static_cast<Eigen::Index>(controls.size()));
  for (std::size_t index = 0; index < controls.size(); ++index) {
    matrix(0, static_cast<Eigen::Index>(index)) = controls[index].x;
    matrix(1, static_cast<Eigen::Index>(index)) = controls[index].y;
    matrix(2, static_cast<Eigen::Index>(index)) = controls[index].z;
  }
  const local::scan::upstream::UniformBspline spline(matrix, degree, interval);
  const Eigen::VectorXd knots = spline.getKnot();
  SplineTarget target;
  target.controls = std::move(controls);
  target.order = degree;
  target.startTimeS = 0.0;
  target.knots.assign(knots.data(), knots.data() + knots.size());
  target.trajectoryId = trajectory_id;
  return target;
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
  params.headingAlignEnterRad = 0.1;
  params.headingAlignExitRad = 0.05;
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

TEST(FollowerPath, CmuProfileMatchesUpstreamAndKeepsDriveMode) {
  FollowerParams base;
  base.maxSpeed = 0.5;
  base.maxYawRateRadS = 0.55;
  base.headingAlignEnterRad = 0.8;
  base.headingAlignExitRad = 0.3;
  base.twoWayDrive = true;
  const FollowerParams params = cmuFollowerParams(base);

  EXPECT_DOUBLE_EQ(params.maxSpeed, 0.5);
  EXPECT_DOUBLE_EQ(params.baseLookAheadDis, 0.5);
  EXPECT_DOUBLE_EQ(params.minLookAheadDis, 0.5);
  EXPECT_DOUBLE_EQ(params.maxLookAheadDis, 0.5);
  EXPECT_DOUBLE_EQ(params.yawRateGain, 1.5);
  EXPECT_DOUBLE_EQ(params.maxYawRateRadS, 0.55);
  EXPECT_DOUBLE_EQ(params.maxAccel, 2.0);
  EXPECT_DOUBLE_EQ(params.linearStopThreshold, 0.02);
  EXPECT_DOUBLE_EQ(params.headingAlignEnterRad, 0.8);
  EXPECT_DOUBLE_EQ(params.headingAlignExitRad, 0.3);
  EXPECT_DOUBLE_EQ(params.omniDirGoalThre, 0.4);
  EXPECT_DOUBLE_EQ(params.omniDirDiffThre, 1.5);
  EXPECT_DOUBLE_EQ(params.stopDisThre, 0.3);
  EXPECT_DOUBLE_EQ(params.slowDwnDisThre, 0.75);
  EXPECT_TRUE(params.twoWayDrive);
  EXPECT_TRUE(params.noRotAtGoal);

  base.twoWayDrive = false;
  EXPECT_FALSE(cmuFollowerParams(base).twoWayDrive);
}

TEST(FollowerPath, CmuSuppressesTheFirstAccelerationQuantum) {
  FollowerParams params;
  params.baseLookAheadDis = 0.5;
  params.lookAheadRatio = 0.0;
  params.minLookAheadDis = 0.5;
  params.maxLookAheadDis = 0.5;
  params.maxSpeed = 1.0;
  params.maxAccel = 2.0;
  params.linearStopThreshold = 0.02;
  params.nominalDt = 0.01;
  params.headingAlignEnterRad = 0.4;
  params.headingAlignExitRad = 0.4;
  params.omniDirGoalThre = 0.4;
  params.omniDirDiffThre = 1.5;
  params.twoWayDrive = false;
  Follower follower;
  const auto path = straightPath();

  const auto first = followPath(follower, path, params, 1.00);
  const auto second = followPath(follower, path, params, 1.01);

  EXPECT_DOUBLE_EQ(first.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(first.cmd.vy, 0.0);
  EXPECT_NEAR(second.cmd.vx, 0.04, 1e-9);
}

TEST(FollowerPath, CmuDeceleratesInsteadOfHardStoppingForHeadingError) {
  FollowerParams params;
  params.baseLookAheadDis = 0.5;
  params.lookAheadRatio = 0.0;
  params.minLookAheadDis = 0.5;
  params.maxLookAheadDis = 0.5;
  params.maxSpeed = 1.0;
  params.maxAccel = 2.0;
  params.linearStopThreshold = 0.02;
  params.nominalDt = 0.01;
  params.headingAlignEnterRad = 0.4;
  params.headingAlignExitRad = 0.4;
  params.omniDirGoalThre = 0.4;
  params.omniDirDiffThre = 1.5;
  params.twoWayDrive = false;
  Follower follower;
  const auto path = straightPath();

  for (int tick = 0; tick < 20; ++tick) {
    followPath(follower, path, params, 1.00 + 0.01 * tick);
  }
  ASSERT_NEAR(follower.diagnostics().linearSpeed, 0.40, 1e-9);

  const auto turning = followPath(follower, path, params, 1.20, {}, 0.60);

  EXPECT_FALSE(turning.canAccelerate);
  EXPECT_TRUE(turning.executionFrozen);
  EXPECT_NEAR(std::hypot(turning.cmd.vx, turning.cmd.vy), 0.38, 1e-9);
  EXPECT_NE(turning.cmd.wz, 0.0);
}

TEST(FollowerPath, YawRateLimitUsesRadiansPerSecond) {
  FollowerParams params;
  params.maxYawRateRadS = 0.4;
  params.yawRateGain = 10.0;
  params.stopYawRateGain = 10.0;
  params.maxAccel = 100.0;
  params.twoWayDrive = false;
  params.omniDirGoalThre = 0.0;
  Follower follower;
  const std::vector<Vec3> path = {{0.0, 1.0, 0.0}, {0.0, 2.0, 0.0}};

  const auto output = followPath(follower, path, params, 0.0);

  EXPECT_NEAR(std::abs(output.cmd.wz), 0.4, 1e-9);
}

TEST(FollowerPath, LimitsYawAccelerationAcrossReplannedDirections) {
  FollowerParams params;
  params.maxYawRateRadS = 0.8;
  params.yawRateGain = 10.0;
  params.stopYawRateGain = 10.0;
  params.maxAccel = 100.0;
  params.maxYawAccelRadS2 = 2.0;
  params.nominalDt = 0.05;
  params.maxDt = 0.05;
  params.twoWayDrive = false;
  params.omniDirGoalThre = 0.0;
  Follower follower;
  const std::vector<Vec3> left = {{0.0, 1.0, 0.0}, {0.0, 2.0, 0.0}};
  const std::vector<Vec3> right = {{0.0, -1.0, 0.0}, {0.0, -2.0, 0.0}};

  const auto first = followPath(follower, left, params, 1.00);
  const auto reversed = followPath(follower, right, params, 1.05);

  EXPECT_NEAR(first.cmd.wz, 0.10, 1e-9);
  EXPECT_NEAR(reversed.cmd.wz, 0.0, 1e-9);
}

TEST(FollowerPath, HeadingAlignmentUsesEnterExitHysteresis) {
  FollowerParams params;
  params.headingAlignEnterRad = 0.60;
  params.headingAlignExitRad = 0.20;
  params.maxYawRateRadS = 1.0;
  params.maxAccel = 100.0;
  params.twoWayDrive = false;
  params.omniDirGoalThre = 0.0;
  Follower follower;
  const std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};

  const auto entered = followPath(follower, path, params, 0.00, {}, 0.65);
  EXPECT_TRUE(entered.executionFrozen);
  EXPECT_DOUBLE_EQ(entered.cmd.vx, 0.0);

  const auto held = followPath(follower, path, params, 0.05, {}, 0.30);
  EXPECT_TRUE(held.executionFrozen);
  EXPECT_DOUBLE_EQ(held.cmd.vx, 0.0);

  const auto released = followPath(follower, path, params, 0.10, {}, 0.15);
  EXPECT_FALSE(released.executionFrozen);
  EXPECT_GT(released.cmd.vx, 0.0);

  const auto inside_band = followPath(follower, path, params, 0.15, {}, 0.30);
  EXPECT_FALSE(inside_band.executionFrozen);
  EXPECT_GT(inside_band.cmd.vx, 0.0);
}

TEST(FollowerPath, OmniMotionCanTranslateWithModerateHeadingError) {
  FollowerParams params;
  params.headingAlignEnterRad = 0.1;
  params.headingAlignExitRad = 0.05;
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

TEST(FollowerPath, CornerGateReachesTurnBeforeLookingPastIt) {
  FollowerParams params;
  params.baseLookAheadDis = 0.60;
  params.lookAheadRatio = 0.0;
  params.minLookAheadDis = 0.60;
  params.maxLookAheadDis = 0.60;
  params.maxSpeed = 0.5;
  params.maxAccel = 100.0;
  params.maxYawRateRadS = 0.0;
  params.headingAlignEnterRad = M_PI + 0.1;
  params.headingAlignExitRad = M_PI;
  params.omniDirGoalThre = 5.0;
  params.omniDirDiffThre = M_PI + 0.1;
  params.cornerGate = true;

  std::vector<Vec3> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back({0.0, 0.1 * i, 0.0});
  }
  for (int i = 1; i <= 20; ++i) {
    path.push_back({0.1 * i, 1.0, 0.0});
  }

  Follower follower;
  const auto before_corner = followPath(follower, path, params, 1.0, {0.0, 0.55, 0.0});
  EXPECT_NEAR(before_corner.cmd.vx, 0.0, 1e-9);
  EXPECT_GT(before_corner.cmd.vy, 0.0);

  const auto at_corner = followPath(follower, path, params, 1.05, {0.0, 0.95, 0.0});
  EXPECT_GT(at_corner.cmd.vx, 0.0);
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
  params.maxYawRateRadS = M_PI / 4.0;
  params.headingAlignEnterRad = 1.0;
  params.headingAlignExitRad = 0.5;
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
  params.maxYawRateRadS = M_PI / 2.0;
  params.headingAlignEnterRad = 1.5;
  params.headingAlignExitRad = 0.75;
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

TEST(FollowerSpline, DefaultsMatchOfficialScanController) {
  const SplineFollowerParams params;

  EXPECT_DOUBLE_EQ(params.timeForward, 0.8);
  EXPECT_DOUBLE_EQ(params.headingErrorThreshold, 0.8);
  EXPECT_DOUBLE_EQ(params.positionGain, 0.8);
  EXPECT_DOUBLE_EQ(params.yawGain, 1.5);
  EXPECT_DOUBLE_EQ(params.maxVx, 0.75);
  EXPECT_DOUBLE_EQ(params.maxVy, 0.35);
  EXPECT_DOUBLE_EQ(params.maxYawRateRadS, 1.0);
  EXPECT_DOUBLE_EQ(params.finishDistance, 0.15);
}

TEST(FollowerSpline, UsesOfficialFeedForwardAndLiveWorldPoseError) {
  FollowerParams params;
  params.spline.positionGain = 0.8;
  params.spline.headingErrorThreshold = 2.0;
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 1, 1.0);
  Follower follower;

  (void)followSpline(follower, trajectory, params, 10.0,
                     {-0.25, 0.0, 0.0});
  const auto output =
      followSpline(follower, trajectory, params, 10.1, {-0.25, 0.0, 0.0});

  EXPECT_NEAR(output.cmd.vx, 0.75, 1e-9);
  EXPECT_NEAR(output.cmd.vy, 0.0, 1e-9);
  EXPECT_NEAR(output.directionError, 0.0, 1e-9);
}

TEST(FollowerSpline, AppliesRequestedSpeedAndSlowFactor) {
  FollowerParams params;
  params.spline.positionGain = 0.0;
  params.spline.headingErrorThreshold = 2.0;
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 1, 1.0);
  Follower follower;

  const auto output = followSpline(follower, trajectory, params, 0.1, {}, 0.0, 0.5, 0.5);

  EXPECT_NEAR(output.cmd.vx, params.spline.maxVx * 0.25, 1e-9);
  EXPECT_NEAR(output.cmd.vy, 0.0, 1e-9);
}

TEST(FollowerSpline, ConvertsWorldVelocityIntoCurrentBodyFrame) {
  FollowerParams params;
  params.spline.positionGain = 0.0;
  params.spline.headingErrorThreshold = 2.0;
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 1, 1.0);
  Follower follower;

  (void)followSpline(follower, trajectory, params, 2.0, {}, M_PI / 2.0);
  const auto output =
      followSpline(follower, trajectory, params, 2.1, {}, M_PI / 2.0);

  EXPECT_NEAR(output.cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(output.cmd.vy, -0.35, 1e-9);
}

TEST(FollowerSpline, FreezesOfficialExecutionClockWhileHeadingIsMisaligned) {
  FollowerParams params;
  params.spline.headingErrorThreshold = 0.8;
  params.spline.positionGain = 0.0;
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}, 1, 1.0);
  Follower follower;

  const auto frozen = followSpline(follower, trajectory, params, 1.0);
  const auto still_frozen = followSpline(follower, trajectory, params, 1.1);
  const auto aligned =
      followSpline(follower, trajectory, params, 1.2, {}, M_PI / 2.0);

  EXPECT_TRUE(frozen.executionFrozen);
  EXPECT_TRUE(still_frozen.executionFrozen);
  EXPECT_DOUBLE_EQ(frozen.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(frozen.cmd.vy, 0.0);
  EXPECT_GT(frozen.cmd.wz, 0.0);
  EXPECT_FALSE(aligned.executionFrozen);
  EXPECT_GT(aligned.cmd.vx, 0.0);
}

TEST(FollowerSpline, ResetsExecutionOnlyForANewOfficialTrajectoryId) {
  FollowerParams params;
  params.spline.positionGain = 0.0;
  params.spline.headingErrorThreshold = 2.0;
  const SplineTarget first =
      scanSpline({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 1, 1.0, 7);
  const SplineTarget replacement =
      scanSpline({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}, 1, 1.0, 8);
  Follower follower;

  (void)followSpline(follower, first, params, 4.0);
  const auto advanced = followSpline(follower, first, params, 4.2);
  const auto reset = followSpline(follower, replacement, params, 4.2);

  EXPECT_GT(advanced.cmd.vx, 0.0);
  EXPECT_NEAR(reset.cmd.vx, 0.0, 1e-9);
  EXPECT_GT(reset.cmd.vy, 0.0);
}

TEST(FollowerSpline, StopsAtOfficialFinishDistance) {
  FollowerParams params;
  params.spline.finishDistance = 0.15;
  params.spline.headingErrorThreshold = 2.0;
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}}, 1, 0.1);
  Follower follower;

  (void)followSpline(follower, trajectory, params, 1.0);
  const auto output =
      followSpline(follower, trajectory, params, 1.2, {0.1, 0.0, 0.0});

  EXPECT_TRUE(output.finished);
  EXPECT_DOUBLE_EQ(output.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.wz, 0.0);
}

TEST(FollowerRegistry, SwitchesAlgorithmsBehindOneFollowInterface) {
  FollowerParams params;
  params.maxAccel = 10.0;
  params.nominalDt = 0.05;
  Follower follower;
  const auto path = straightPath();
  const SplineTarget trajectory =
      scanSpline({{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, 1, 0.5);

  followPath(follower, path, params, 0.0);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Path);

  followSpline(follower, trajectory, params, 0.1);
  EXPECT_EQ(follower.diagnostics().algorithm, FollowerAlgorithm::Spline);

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
  params.maxYawRateRadS = M_PI / 2.0;
  params.headingAlignEnterRad = 0.6;
  params.headingAlignExitRad = 0.3;
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
