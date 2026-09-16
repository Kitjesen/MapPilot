#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/plan_env/grid_map.h"
#include "planning/local/scan/upstream/plan_manage/planner_manager.h"

namespace {

using namespace nav_kernel::local::scan::upstream;

void expectMovingBoundaryPlan(const Eigen::Vector3d &start,
                              const Eigen::Vector3d &velocity,
                              const Eigen::Vector3d &acceleration,
                              const Eigen::Vector3d &target) {
  lingtu::nav::tests::CollisionBitmap bitmap;
  const std::vector<nav_kernel::Vec3> route{
      {start.x(), start.y(), start.z()},
      {target.x(), target.y(), target.z()}};
  nav_kernel::LocalPlanRequest request;
  request.robot.pose = {route.front(), 0.0};
  request.objective = nav_kernel::RouteTarget{
      {route.data(), static_cast<int>(route.size()), 1, false}};
  request.identity = {1, 1, 0};
  request.clock.timestampS = 1.0;
  request.environment.collision = bitmap.view();
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.scan.voxelResolution = 0.1;
  params.scan.cylinderOffset = 0.0;
  nav_kernel::local::scan::Grid grid(params, request);

  PlanParameters plan;
  plan.max_vel_ = 0.5;
  plan.max_acc_ = 0.5;
  plan.max_jerk_ = 4.0;
  plan.ctrl_pt_dist = 0.2;
  plan.motion_intent_ = true;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1.0;
  optimizer.lambda_collision = 1.0;
  optimizer.lambda_feasibility = 0.1;
  optimizer.lambda_fitness = 1.0;
  optimizer.dist0 = 0.2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  SCANPlannerManager manager;
  manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));

  ASSERT_TRUE(manager.reboundReplan(start, velocity, acceleration, target,
                                  Eigen::Vector3d::Zero(), true, false, 1.0))
      << manager.reboundDebug().stage << ":" << manager.reboundDebug().reason;
  const auto &position = manager.local_data_.position_traj_;
  const auto speed = position.getDerivative();
  const auto acc = speed.getDerivative();
  const double duration = position.getTimeSum();
  ASSERT_TRUE(std::isfinite(duration));
  ASSERT_GT(duration, 0.0);
  EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
  EXPECT_LE((speed.evaluateDeBoorT(0.0) - velocity).norm(), 1e-9);
  EXPECT_LE((acc.evaluateDeBoorT(0.0) - acceleration).norm(), 1e-9);
  EXPECT_LE((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
  EXPECT_LE(speed.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(acc.evaluateDeBoorT(duration).norm(), 1e-9);

  // Certify every quadratic velocity span by its three Bezier controls.
  const Eigen::MatrixXd controls = speed.getControlPoint();
  for (Eigen::Index index = 0; index + 2 < controls.cols(); ++index) {
    SCOPED_TRACE(index);
    EXPECT_LE(((controls.col(index) + controls.col(index + 1)) / 2.0).norm(),
              plan.max_vel_ + 1e-9);
    EXPECT_LE(controls.col(index + 1).norm(), plan.max_vel_ + 1e-9);
    EXPECT_LE(((controls.col(index + 1) + controls.col(index + 2)) / 2.0).norm(),
              plan.max_vel_ + 1e-9);
  }
  EXPECT_LE(acc.getControlPoint().colwise().norm().maxCoeff(),
            plan.max_acc_ + 1e-9);
  if (acceleration.squaredNorm() == 0.0) {
    EXPECT_LE(controls.colwise().norm().maxCoeff(), plan.max_vel_ + 1e-9);
  } else {
    // The old ratio rejected this extrapolated control despite the certificate.
    EXPECT_GT(controls.col(0).norm(), plan.max_vel_);
  }
}

}  // namespace

TEST(ScanSplineVelocityBounds, NearSpeedCapBrakingStartConverges) {
  // Physical boundary from Go2 failure 065; use the deterministic public seed.
  expectMovingBoundaryPlan(
      {0.8828517837168606, 0.0012427338868311276, -0.0038471295677491415},
      {0.4998194419679588, 0.002355654363685327, -2.073090335543537e-05},
      {-0.002164648251727582, 7.515736899741209e-06, 0.0001586942315220117},
      {1.7495133210260394, 0.0038531350481869966, -0.0038455564774449863});
}

TEST(ScanSplineVelocityBounds, BrakingStartRetimesBackwardTargetWithoutOverflow) {
  // Physical boundary from Go2 failure 071, isolated from target selection.
  expectMovingBoundaryPlan(
      {1.478020382134976, 0.003472465915068783, -0.0038479666213952174},
      {0.4714734304819697, 0.0010641731673858852, 3.0343679116609405e-06},
      {-0.06498463704119181, -0.0013243995008085188, -1.4876682586278112e-05},
      {-0.2719746094673958, -0.0007143427281083086, -0.0038479666213952174});
}

TEST(ScanSplineVelocityBounds, ZeroAccelerationRetainsBoundaryAtSpeedCap) {
  expectMovingBoundaryPlan({0.0, 0.0, 0.5}, {0.5, 0.0, 0.0},
                           Eigen::Vector3d::Zero(), {2.0, 0.3, 0.5});
}
