#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/bspline_opt/bspline_optimizer.h"
#include "planning/local/scan/upstream/bspline_opt/lbfgs.hpp"

namespace {

using namespace nav_kernel::local::scan::upstream;

TEST(ScanRefineSolver, CubicFallbackKeepsTrialInsideItsBracket) {
  // Captured from the constrained garage refine: the cubic has no minimum.
  double x = 0, fx = 0.024790567639842942, dx = -0.020584614952887565;
  double y = 1, fy = .1, dy = .1;
  double trial = 0.032369093694582435;
  double value = 0.024134902879559637, derivative = -0.01992933171757719;
  int bracketed = 1;
  ASSERT_EQ(lbfgs::update_trial_interval(&x, &fx, &dx, &y, &fy, &dy,
      &trial, &value, &derivative, 0, 1, &bracketed), 0);
  EXPECT_GT(trial, std::min(x, y));
  EXPECT_LT(trial, std::max(x, y));
}

BsplineOptimizerParams parameters() {
  BsplineOptimizerParams params;
  params.lambda_smooth = 1.0;
  params.lambda_collision = 0.5;
  params.lambda_feasibility = 0.1;
  params.lambda_fitness = 1.0;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  return params;
}

void referenceFrom(BsplineOptimizer &optimizer, const Eigen::MatrixXd &controls,
                   double interval) {
  const UniformBspline reference(controls, 3, interval);
  optimizer.ref_pts_.clear();
  for (int sample = 0; sample <= controls.cols() - 3; ++sample)
    optimizer.ref_pts_.push_back(reference.evaluateDeBoorT(interval * sample));
}

Eigen::MatrixXd roundingFailureControls() {
  Eigen::MatrixXd controls(3, 9);
  for (int index = 0; index < controls.cols(); ++index) {
    const double wave = std::sin(index);
    controls.col(index) = Eigen::Vector3d{
        0.1 * (index - 1) + 1e-12 * wave,
        1e-12 * wave,
        0.5 + 0.01 * wave};
  }
  return controls;
}

TEST(ScanRefineSolver, LineSearchFailureIsRejectedWithoutChangingFixedBoundaries) {
  nav_kernel::LocalPlannerParams local;
  local.checkObstacle = false;
  local.scan.voxelResolution = 0.1;
  nav_kernel::LocalPlanRequest request;
  const nav_kernel::local::scan::Grid grid(local, request);
  ASSERT_TRUE(grid.valid());
  BsplineOptimizer optimizer;
  optimizer.setParam(parameters());
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));

  // Finite, positive inputs with a short interval cause the fixed-Z cost to
  // dominate the tiny XY change. The real line search fails and rolls back.
  const Eigen::MatrixXd controls = roundingFailureControls();
  constexpr double interval = 5e-5;
  referenceFrom(optimizer, controls, interval);
  Eigen::MatrixXd output;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls, interval, output));
  const auto &debug = optimizer.optimizationDebug();
  ASSERT_TRUE(debug.optimizerReturnCodeValid);
  EXPECT_LT(debug.optimizerReturnCode, 0);
  EXPECT_NE(debug.optimizerReturnCode, lbfgs::LBFGSERR_MAXIMUMITERATION);
  EXPECT_EQ(debug.reason, "optimizer_error");
  EXPECT_FALSE(debug.collisionValid);
  ASSERT_EQ(output.rows(), controls.rows());
  ASSERT_EQ(output.cols(), controls.cols());
  EXPECT_TRUE(output.allFinite());
  // The solver may accept iterations before the failing line search. Its last
  // accepted interior is not necessarily the initial input. Only the fixed
  // boundary controls must remain identical through the public refine API.
  EXPECT_EQ((output.leftCols(3) - controls.leftCols(3)).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ((output.rightCols(3) - controls.rightCols(3)).cwiseAbs().maxCoeff(), 0.0);
}

TEST(ScanRefineSolver, SuccessfulRefineStillRejectsAnOccupiedTrajectory) {
  lingtu::nav::tests::CollisionBitmap bitmap({-2,-2,-1}, {2,2,2}, 0.1);
  nav_kernel::LocalPlannerParams local;
  local.checkObstacle = true;
  local.scan.voxelResolution = 0.1;
  local.scan.cylinderOffset = 0.0;
  nav_kernel::LocalPlanRequest request;
  request.environment.collision = bitmap.view();
  const nav_kernel::local::scan::Grid grid(local, request);
  ASSERT_TRUE(grid.valid());
  BsplineOptimizer optimizer;
  optimizer.setParam(parameters());
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3, 9);
  for (int index = 0; index < controls.cols(); ++index)
    controls.col(index) = Eigen::Vector3d{0.1 * (index - 1), 0.0, 0.5};
  referenceFrom(optimizer, controls, 0.5);

  Eigen::MatrixXd output;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  EXPECT_EQ(optimizer.optimizationDebug().reason, "accepted");
  bitmap.occupy({0.05, 0.05, 0.55});
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  EXPECT_EQ(optimizer.optimizationDebug().reason, "collision_after_refine");
  EXPECT_TRUE(optimizer.optimizationDebug().collisionValid);
  bitmap.clear();
  EXPECT_TRUE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  EXPECT_EQ(optimizer.optimizationDebug().reason, "accepted");
  EXPECT_FALSE(optimizer.optimizationDebug().collisionValid);
}

}  // namespace
