#include <array>

#include <gtest/gtest.h>

#include "navigation/recovery.hpp"
#include "navigation/recovery_search.hpp"
#include "collision/body.hpp"
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
      {route.data(), static_cast<int>(route.size()), 1}};
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

TEST(RecoverySequence, NoSafeCandidateRetriesCurrentGeometryUntilAttemptsAreExhausted) {
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
    input.clock.timestampS = 1.0 + 0.1 * attempt;
    const auto output = recovery.step(input);
    EXPECT_EQ(output.attempt, attempt);
    EXPECT_FALSE(output.verified);
    EXPECT_EQ(output.action, nav_kernel::RecoveryAction::None);
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
  EXPECT_EQ(beside.reason, "recovery_translation_active");
  input.robot.pose.position = end;
  input.clock.timestampS += .1;
  const auto arrived = recovery.step(input);
  EXPECT_FALSE(arrived.active);
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

TEST(RecoverySequence, ScanTranslationIsAnUnverifiedGoalUntilPlannerAcceptsIt) {
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  auto input = freeSpaceInput();
  lingtu::nav::navigation::Recovery recovery(params, {});
  const auto target = recovery.step(input);
  EXPECT_TRUE(target.active);
  EXPECT_FALSE(target.verified);
  EXPECT_FALSE(target.direct_command);
  EXPECT_EQ(target.action, nav_kernel::RecoveryAction::Translate);
  ASSERT_EQ(target.path_world.size(), 2U);
  input.clock.timestampS += .1;
  const auto pending = recovery.step(input);
  EXPECT_EQ(pending.attempt, target.attempt);
  ASSERT_EQ(pending.path_world.size(), 2U);
  EXPECT_DOUBLE_EQ(pending.path_world.back().x, target.path_world.back().x);
  EXPECT_DOUBLE_EQ(pending.path_world.back().y, target.path_world.back().y);
  input.robot.pose.position = target.path_world.back();
  input.clock.timestampS += .1;
  const auto arrived = recovery.step(input);
  EXPECT_FALSE(arrived.active);
  EXPECT_EQ(arrived.reason, "recovery_translation_complete");
}

TEST(RecoverySequence, ExplicitBodyGridChecksSweptMotionWhenPointSamplingIsDisabled) {
  lingtu::nav::tests::CollisionBitmap raw({-2,-2,-1},{2,2,2},.1);
  lingtu::nav::tests::CollisionBitmap inflated({-2,-2,-1},{2,2,2},.1);
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  lingtu::nav::collision::Part part;
  part.half_size = {.1, .03, .04};
  body->parts.push_back(part);
  auto params = freeSpaceParams();
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.scan.voxelResolution = .1;
  auto input = freeSpaceInput();
  input.body = body;
  input.robot.pose.position.z = .5;
  const auto snapshot = [&] {
    input.environment.collision = inflated.view();
    const auto measured = raw.view();
    input.environment.collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(
            measured.inflatedBits, measured.inflatedBits + measured.inflatedBytes);
    input.environment.collision.knownFreeStorage =
        std::make_shared<const std::vector<std::uint8_t>>(measured.inflatedBytes, 255U);
  };
  lingtu::nav::navigation::detail::RecoverySearchParams search_params;
  search_params.checkObstacles = false;
  search_params.requireTraversability = false;
  lingtu::nav::navigation::detail::RecoverySearch search(search_params);
  lingtu::nav::navigation::detail::RecoverySearchInput search_input;
  search_input.vehiclePose = input.robot.pose;
  raw.occupy({.55, .05, .55});
  snapshot();
  nav_kernel::local::scan::Grid translation(params, input);
  search_input.collisionGrid = &translation;
  ASSERT_EQ(translation.inflatedOccupancy({0, 0, .5}, 0), 0);
  ASSERT_EQ(translation.inflatedOccupancy({1, 0, .5}, 0), 0);
  EXPECT_FALSE(search.validateBodyPath(search_input, {{0, 0, 0}, {1, 0, 0}}));

  raw.clear();
  raw.occupy({.35, .35, .55});
  body->parts.front().half_size.x = .45;
  snapshot();
  nav_kernel::local::scan::Grid rotation(params, input);
  search_input.collisionGrid = &rotation;
  ASSERT_EQ(rotation.inflatedOccupancy({0, 0, .5}, 0), 0);
  ASSERT_EQ(rotation.inflatedOccupancy({0, 0, .5}, M_PI / 2), 0);
  EXPECT_FALSE(search.validateRotation(search_input, M_PI / 2));
}


}  // namespace
