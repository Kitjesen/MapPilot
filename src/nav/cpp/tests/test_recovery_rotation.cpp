#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "planning/local/recovery.hpp"

namespace {

using nav_kernel::PlanStatus;
using nav_kernel::RecoveryAction;
using nav_kernel::RecoveryPlanner;
using nav_kernel::RecoveryPlannerInput;
using nav_kernel::RecoveryPlannerParams;

RecoveryPlannerParams rotationParams() {
  RecoveryPlannerParams params;
  params.vehicleLength = 0.80;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.10;
  params.checkObstacles = true;
  params.requireTraversability = false;
  params.searchRadius = 1.20;
  params.minRotationRad = 0.20;
  params.maxRotationRad = 1.20;
  params.rotationCandidateStepRad = 0.20;
  params.rotationSampleStepRad = 0.05;
  params.rotationRate = 0.25;
  return params;
}

TEST(RecoveryRotation, SelectsAngleNearestGoalWhenCorridorReachIsEqual) {
  auto params = rotationParams();
  params.checkObstacles = false;
  RecoveryPlannerInput input;
  input.goalDirectionBodyRad = 1.0;
  input.rejectedTranslationDirectionMask = 0xFFFFU;

  const auto result = RecoveryPlanner(params).plan(input);

  ASSERT_EQ(result.status, PlanStatus::RotationReady);
  ASSERT_EQ(result.action, RecoveryAction::Rotate);
  EXPECT_NEAR(result.rotationDeltaRad, 1.0, 1e-9);
  EXPECT_NEAR(result.diagnostics.selectedRotationRad, 1.0, 1e-9);
  EXPECT_NEAR(result.diagnostics.selectedForwardReachM, params.searchRadius, 1e-9);
}

TEST(RecoveryRotation, IncludesMaximumAngleWhenStepDoesNotLandOnIt) {
  auto params = rotationParams();
  params.checkObstacles = false;
  params.maxRotationRad = 1.10;
  params.rotationCandidateStepRad = 0.20;
  RecoveryPlannerInput input;
  input.goalDirectionBodyRad = 1.10;
  input.rejectedTranslationDirectionMask = 0xFFFFU;

  const auto result = RecoveryPlanner(params).plan(input);

  ASSERT_EQ(result.status, PlanStatus::RotationReady);
  EXPECT_NEAR(result.rotationDeltaRad, 1.10, 1e-9);
  EXPECT_NEAR(result.diagnostics.selectedRotationRad, 1.10, 1e-9);
}

TEST(RecoveryRotation, ChoosesLargerAngleThatOpensForwardCorridor) {
  const auto params = rotationParams();
  std::vector<float> obstacle_x;
  std::vector<float> obstacle_y;
  std::vector<float> obstacle_height;
  for (int index = -3; index <= 3; ++index) {
    obstacle_x.push_back(0.75F);
    obstacle_y.push_back(0.08F * static_cast<float>(index));
    obstacle_height.push_back(1.0F);
  }

  RecoveryPlannerInput input;
  input.obstacleX = obstacle_x.data();
  input.obstacleY = obstacle_y.data();
  input.obstacleHeight = obstacle_height.data();
  input.obstacleCount = static_cast<int>(obstacle_x.size());
  input.goalDirectionBodyRad = 0.0;
  input.rejectedTranslationDirectionMask = 0xFFFFU;

  const auto result = RecoveryPlanner(params).plan(input);

  ASSERT_EQ(result.status, PlanStatus::RotationReady);
  EXPECT_GT(std::abs(result.rotationDeltaRad), 0.35);
  EXPECT_GT(result.diagnostics.selectedForwardReachM, 0.80);
  EXPECT_GT(result.diagnostics.rotationCandidateCount, 2);
}

}  // namespace
