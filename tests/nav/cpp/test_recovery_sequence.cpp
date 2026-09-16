#include <array>

#include <gtest/gtest.h>

#include "navigation/recovery.hpp"
#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"

namespace {

nav_kernel::LocalPlanRequest freeSpaceInput(double timestamp_s = 1.0) {
  static const std::array<nav_kernel::Vec3, 2> route{{
      {0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
  }};
  nav_kernel::LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  input.objective = nav_kernel::RouteTarget{
      {route.data(), static_cast<int>(route.size()), 1, false}};
  input.clock.timestampS = timestamp_s;
  return input;
}

nav_kernel::LocalPlannerParams freeSpaceParams() {
  nav_kernel::LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  return params;
}

TEST(RecoverySequence, DefaultOrderKeepsOmnidirectionalTranslationFirst) {
  lingtu::nav::navigation::RecoveryConfig config;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  const auto output = recovery.step(freeSpaceInput());

  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.action, nav_kernel::RecoveryAction::Translate);
  EXPECT_EQ(output.state, 2);
  EXPECT_GE(output.path_body.size(), 2U);
}

TEST(RecoverySequence, NoSafeCandidateRequestsObservationUnlessAttemptsAreExhausted) {
  auto params = freeSpaceParams();
  params.useTraversabilityCost = true;
  std::array<float, 81> blocked;
  blocked.fill(95.0f);
  auto input = freeSpaceInput();
  input.environment.traversability = {
      blocked.data(), 9, 9, 0.5, -2.0, -2.0};
  lingtu::nav::navigation::RecoveryConfig config;
  config.max_attempts = 3;
  lingtu::nav::navigation::Recovery recovery(params, config);

  for (int attempt = 1; attempt <= config.max_attempts; ++attempt) {
    // Executor admits each retry only after the requested observation refresh.
    input.clock.timestampS = 1.0 + 0.1 * attempt;
    const auto output = recovery.step(input);
    EXPECT_EQ(output.attempt, attempt);
    EXPECT_FALSE(output.verified);
    EXPECT_EQ(output.action, nav_kernel::RecoveryAction::None);
    EXPECT_EQ(output.observation_refresh_required, attempt < config.max_attempts);
    EXPECT_EQ(output.exhausted, attempt == config.max_attempts);
  }
}

TEST(RecoverySequence, TranslationBesideEndpointMustStillReachExit) {
  lingtu::nav::navigation::RecoveryConfig config;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);
  auto input = freeSpaceInput();
  const auto started = recovery.step(input);
  ASSERT_EQ(started.action, nav_kernel::RecoveryAction::Translate);
  const auto end = started.path_body.back();
  const double length = std::hypot(end.x, end.y);
  ASSERT_GT(length, .1);
  input.robot.pose.position = {end.x - .18 * end.y / length,
                               end.y + .18 * end.x / length, end.z};
  input.clock.timestampS += .1;
  const auto beside = recovery.step(input);
  EXPECT_TRUE(beside.active);
  EXPECT_FALSE(beside.observation_refresh_required);
  EXPECT_EQ(beside.reason, "recovery_translation_active");
  input.robot.pose.position = end;
  input.clock.timestampS += .1;
  const auto arrived = recovery.step(input);
  EXPECT_TRUE(arrived.observation_refresh_required);
  EXPECT_EQ(arrived.reason, "recovery_translation_complete");
}

TEST(RecoverySequence, TranslationOffPathSelectsNextBehaviorBeforeFalseCompletion) {
  lingtu::nav::navigation::RecoveryConfig config;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);
  auto input = freeSpaceInput();
  const auto started = recovery.step(input);
  ASSERT_EQ(started.action, nav_kernel::RecoveryAction::Translate);
  const auto end = started.path_body.back();
  const double length = std::hypot(end.x, end.y);
  ASSERT_GT(length, .1);
  input.robot.pose.position = {end.x - .8 * end.y / length,
                               end.y + .8 * end.x / length, end.z};
  input.clock.timestampS += .1;
  const auto rejected = recovery.step(input);
  EXPECT_FALSE(rejected.observation_refresh_required);
  EXPECT_EQ(rejected.action, nav_kernel::RecoveryAction::Rotate);
  EXPECT_EQ(rejected.attempt, 2);
}

TEST(RecoverySequence, ConfiguredRotationFirstSelectsRotation) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.behavior_order = {
      nav_kernel::RecoveryAction::Rotate,
      nav_kernel::RecoveryAction::Translate,
  };
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  const auto output = recovery.step(freeSpaceInput());

  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.action, nav_kernel::RecoveryAction::Rotate);
  EXPECT_EQ(output.state, 1);
  EXPECT_TRUE(output.direct_command);
  EXPECT_NE(output.rotation_direction, 0);
  EXPECT_GE(std::abs(output.rotation_target_rad), config.min_rotation_rad);
  EXPECT_LE(std::abs(output.rotation_target_rad), config.max_rotation_rad);
}

TEST(RecoverySequence, InvalidEntriesAreIgnoredWithoutDisablingRecovery) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.behavior_order = {
      nav_kernel::RecoveryAction::None,
      nav_kernel::RecoveryAction::Rotate,
      nav_kernel::RecoveryAction::Rotate,
  };
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  const auto output = recovery.step(freeSpaceInput());

  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.action, nav_kernel::RecoveryAction::Rotate);
  EXPECT_EQ(output.state, 1);
}

TEST(RecoverySequence, FailedActionAdvancesToNextConfiguredBehavior) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.translation_timeout_s = 0.10;
  config.max_attempts = 3;
  config.behavior_order = {
      nav_kernel::RecoveryAction::Translate,
      nav_kernel::RecoveryAction::Rotate,
  };
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  const auto translating = recovery.step(freeSpaceInput(1.0));
  ASSERT_EQ(translating.action, nav_kernel::RecoveryAction::Translate);

  const auto rotating = recovery.step(freeSpaceInput(1.2));

  EXPECT_TRUE(rotating.active);
  EXPECT_FALSE(rotating.exhausted);
  EXPECT_EQ(rotating.attempt, 2);
  EXPECT_EQ(rotating.action, nav_kernel::RecoveryAction::Rotate);
  EXPECT_EQ(rotating.state, 1);
}

TEST(RecoverySequence, RotationCompletesFromFiftyHertzOdometryProgress) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.behavior_order = {nav_kernel::RecoveryAction::Rotate};
  config.rotation_timeout_s = 0.15;
  config.rotation_rate_rad_s = 0.25;
  config.min_rotation_rad = 0.60;
  config.max_rotation_rad = 0.60;
  config.rotation_candidate_step_rad = 0.10;
  config.rotation_sample_step_rad = 0.05;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  double timestamp_s = 1.0;
  double yaw_rad = 0.0;
  auto output = recovery.step(freeSpaceInput(timestamp_s));
  ASSERT_EQ(output.action, nav_kernel::RecoveryAction::Rotate);

  for (int tick = 0; tick < 200 && output.active; ++tick) {
    timestamp_s += 0.02;
    yaw_rad += static_cast<double>(output.rotation_direction) *
               config.rotation_rate_rad_s * 0.02;
    auto input = freeSpaceInput(timestamp_s);
    input.robot.pose.yaw = yaw_rad;
    output = recovery.step(input);
  }

  EXPECT_FALSE(output.active);
  EXPECT_FALSE(output.exhausted);
  EXPECT_TRUE(output.observation_refresh_required);
  EXPECT_EQ(output.reason, "recovery_rotation_complete");
  EXPECT_GE(std::abs(yaw_rad), 0.50);
  EXPECT_LE(std::abs(yaw_rad), 0.65);
}

TEST(RecoverySequence, SuccessfulActionAdvancesBeforeTheNextReplanAttempt) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.behavior_order = {
      nav_kernel::RecoveryAction::Rotate,
      nav_kernel::RecoveryAction::Translate,
  };
  config.max_attempts = 3;
  config.rotation_timeout_s = 0.15;
  config.rotation_rate_rad_s = 0.5;
  config.min_rotation_rad = 0.20;
  config.max_rotation_rad = 0.20;
  config.rotation_candidate_step_rad = 0.10;
  config.rotation_sample_step_rad = 0.05;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  double timestamp_s = 1.0;
  double yaw_rad = 0.0;
  auto output = recovery.step(freeSpaceInput(timestamp_s));
  ASSERT_EQ(output.action, nav_kernel::RecoveryAction::Rotate);

  for (int tick = 0; tick < 100 && output.active; ++tick) {
    timestamp_s += 0.02;
    yaw_rad += static_cast<double>(output.rotation_direction) *
               config.rotation_rate_rad_s * 0.02;
    auto input = freeSpaceInput(timestamp_s);
    input.robot.pose.yaw = yaw_rad;
    output = recovery.step(input);
  }
  ASSERT_FALSE(output.active);
  ASSERT_TRUE(output.observation_refresh_required);

  timestamp_s += 0.02;
  auto retry_input = freeSpaceInput(timestamp_s);
  retry_input.robot.pose.yaw = yaw_rad;
  const auto retry = recovery.step(retry_input);

  EXPECT_TRUE(retry.active);
  EXPECT_FALSE(retry.exhausted);
  EXPECT_EQ(retry.attempt, 2);
  EXPECT_EQ(retry.action, nav_kernel::RecoveryAction::Translate);
}

TEST(RecoverySequence, SuccessfulActionsStillConsumeTheAttemptBudget) {
  lingtu::nav::navigation::RecoveryConfig config;
  config.behavior_order = {nav_kernel::RecoveryAction::Rotate};
  config.max_attempts = 1;
  config.rotation_timeout_s = 0.15;
  config.rotation_rate_rad_s = 0.5;
  config.min_rotation_rad = 0.20;
  config.max_rotation_rad = 0.20;
  config.rotation_candidate_step_rad = 0.10;
  config.rotation_sample_step_rad = 0.05;
  lingtu::nav::navigation::Recovery recovery(freeSpaceParams(), config);

  double timestamp_s = 1.0;
  double yaw_rad = 0.0;
  auto output = recovery.step(freeSpaceInput(timestamp_s));
  ASSERT_EQ(output.action, nav_kernel::RecoveryAction::Rotate);

  for (int tick = 0; tick < 100 && output.active; ++tick) {
    timestamp_s += 0.02;
    yaw_rad += static_cast<double>(output.rotation_direction) *
               config.rotation_rate_rad_s * 0.02;
    auto input = freeSpaceInput(timestamp_s);
    input.robot.pose.yaw = yaw_rad;
    output = recovery.step(input);
  }
  ASSERT_FALSE(output.active);

  timestamp_s += 0.02;
  auto retry_input = freeSpaceInput(timestamp_s);
  retry_input.robot.pose.yaw = yaw_rad;
  const auto retry = recovery.step(retry_input);

  EXPECT_FALSE(retry.active);
  EXPECT_TRUE(retry.exhausted);
  EXPECT_EQ(retry.attempt, 1);
}

TEST(RecoverySequence, ScanEscapesBesideInflatedWallWithoutInflatingItAgain) {
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -2}, {4, 4, 3}, .05);
  // Inflation reaches beside the body, but neither cylinder centre is occupied.
  for (double x = -2; x < 2; x += .025)
    collision.occupy({x, .175, .435});
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.scan.cylinderOffset = .25;
  params.scan.bodyClearanceBelow = .05;
  params.scan.bodyClearanceAbove = .35;
  params.useTraversabilityCost = true;
  auto input = freeSpaceInput();
  input.robot.pose.position.z = .435;
  input.environment.collision = collision.view();
  lingtu::nav::navigation::Recovery recovery(params, {});

  const auto output = recovery.step(input);
  ASSERT_TRUE(output.active) << output.reason;
  ASSERT_TRUE(output.verified);
  ASSERT_EQ(output.action, nav_kernel::RecoveryAction::Translate);
  nav_kernel::local::scan::Grid grid(params, input);
  for (const auto& point : output.path_body) {
    EXPECT_TRUE(grid.obstacleFree({point.x, point.y, .435}, 0.0));
    EXPECT_LT(point.y, .15);
  }

  // A newly observed obstacle at the actual body height must stop recovery.
  collision.occupyInflated({.25f, 0, .435f}, .20, .1, .1);
  input.clock.timestampS = 1.02;
  input.environment.collision = collision.view(1.02, 2);
  const auto blocked = recovery.step(input);
  EXPECT_EQ(blocked.state, 0);
  EXPECT_EQ(blocked.action, nav_kernel::RecoveryAction::None);
  EXPECT_FALSE(blocked.verified);
}

TEST(RecoverySequence, ScanKeepsBodyHeightAndRequiresFreshCompleteCollisionMap) {
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -2}, {4, 4, 3}, .05);
  // An inflated voxel above the body origin is not an obstacle at body height.
  collision.occupy({.25, 0, .675});
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.scan.cylinderOffset = .25;
  params.scan.bodyClearanceBelow = .05;
  params.scan.bodyClearanceAbove = .35;
  params.useTraversabilityCost = true;
  auto input = freeSpaceInput();
  input.robot.pose.position.z = .435;
  input.environment.collision = collision.view();
  lingtu::nav::navigation::Recovery recovery(params, {});
  EXPECT_TRUE(recovery.step(input).active);
  input.clock.timestampS = 2.0;
  EXPECT_EQ(recovery.step(input).reason, "recovery_collision_map_stale");
  input.environment.collision = collision.view(2.0, 2);
  input.environment.collision.complete = false;
  EXPECT_EQ(recovery.step(input).reason, "recovery_collision_map_incomplete");
  input.environment.collision = {};
  EXPECT_EQ(recovery.step(input).reason, "recovery_collision_map_invalid");
}

TEST(RecoverySequence, ScanExternalClockUsesRebasedReceiveTimeInsteadOfWallSourceTime) {
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -2}, {4, 4, 3}, .05);
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  auto input = freeSpaceInput(1700000000.1);
  input.clock.mode = nav_kernel::PlanClockMode::External;
  input.environment.collision = collision.view(1800000000.0);
  input.environment.collision.receiveStampS = 1700000000.0;
  lingtu::nav::navigation::Recovery recovery(params, {});
  EXPECT_TRUE(recovery.step(input).verified);
  input.clock.timestampS += 1.0;
  EXPECT_EQ(recovery.step(input).reason, "recovery_collision_map_stale");
}

TEST(RecoverySequence, ScanCanLeaveAnInflatedBoundaryCellWithoutTurning) {
  lingtu::nav::tests::CollisionBitmap collision({-4, -4, -2}, {4, 4, 3}, .05);
  collision.occupy({.275, .025, .435});
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.scan.cylinderOffset = .25;
  auto input = freeSpaceInput();
  input.robot.pose.position = {.025, .025, .435};
  input.environment.collision = collision.view();
  lingtu::nav::navigation::Recovery recovery(params, {});
  const auto output = recovery.step(input);
  EXPECT_TRUE(output.verified) << output.reason;
  EXPECT_EQ(output.action, nav_kernel::RecoveryAction::Translate);
  EXPECT_FALSE(output.direct_command);
  ASSERT_EQ(output.path_body.size(), 2U);
  nav_kernel::local::scan::Grid grid(params, input);
  const auto world = nav_kernel::RecoveryPlanner::bodyPathToWorld(output.path_body, input.robot.pose);
  EXPECT_TRUE(grid.boundaryDepartureFree(input.robot.pose, world.back()));
  input.robot.pose.position.z += .003;
  input.clock.timestampS = 1.02;
  input.environment.collision = collision.view(1.02, 2);
  EXPECT_TRUE(recovery.step(input).verified)
      << "small body bobbing must keep the planar exit valid at the actual height";
}

}  // namespace
