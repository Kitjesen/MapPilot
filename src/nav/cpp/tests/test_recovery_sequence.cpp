#include <array>

#include <gtest/gtest.h>

#include "navigation/recovery.hpp"

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

}  // namespace
