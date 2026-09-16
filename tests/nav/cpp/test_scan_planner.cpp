#include <gtest/gtest.h>

#include <memory>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/path_searching/dyn_a_star.h"
#include "planning/local/scan/upstream/plan_env/grid_map.h"
#include "planning/local/scan/upstream/plan_manage/scan_replan_fsm.h"
#include "trajectory/spline.hpp"

namespace {

using lingtu::nav::tests::CollisionBitmap;
using nav_kernel::LocalPlan;
using nav_kernel::LocalPlannerBackend;
using nav_kernel::LocalPlannerParams;
using nav_kernel::LocalPlanRequest;
using nav_kernel::RouteTarget;
using nav_kernel::SplineTarget;
using nav_kernel::Vec3;

LocalPlannerParams scanParams() {
  LocalPlannerParams params;
  params.backend = LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.autonomySpeed = 0.5;
  params.maxSpeed = 0.5;
  params.adjacentRange = 2.5;
  params.scan.voxelResolution = 0.1;
  params.scan.controlPointSpacing = 0.2;
  params.scan.cylinderOffset = 0.0;
  params.scan.maxAcceleration = 0.5;
  return params;
}

struct RequestFixture {
  explicit RequestFixture(std::vector<Vec3> value, double resolution = 0.1)
      : route(std::move(value)), bitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, resolution) {
    request.robot.pose = {route.front(), 0.0};
    request.objective = RouteTarget{{route.data(), static_cast<int>(route.size()), 1, false}};
    request.identity = {1, 1, 0};
    request.clock.timestampS = 1.0;
    request.environment.collision = bitmap.view(1.0, 1);
  }

  void refreshCollision(std::uint64_t generation) {
    request.identity.obstacleGeneration = generation;
    request.environment.collision = bitmap.view(request.clock.timestampS, generation);
  }

  std::vector<Vec3> route;
  CollisionBitmap bitmap;
  LocalPlanRequest request;
};

}  // namespace

namespace {

struct ScanAttemptFixture {
  RequestFixture request{{{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}}};
  nav_kernel::local::scan::Grid grid{scanParams(), request.request};
  nav_kernel::local::scan::upstream::SCANPlannerManager manager;

  explicit ScanAttemptFixture(bool motion_intent = false) {
    using namespace nav_kernel::local::scan::upstream;
    PlanParameters plan;
    plan.max_vel_ = motion_intent ? 0.2 : 0.5;
    plan.max_acc_ = 0.5;
    plan.max_jerk_ = 4.0;
    plan.ctrl_pt_dist = 0.2;
    plan.motion_intent_ = motion_intent;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth = 1.0;
    optimizer.lambda_collision = 0.5;
    optimizer.lambda_feasibility = 0.1;
    optimizer.lambda_fitness = 1.0;
    optimizer.dist0 = 0.2;
    optimizer.max_vel = plan.max_vel_;
    optimizer.max_acc = plan.max_acc_;
    manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));
  }

  bool replan(const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d &acceleration = Eigen::Vector3d::Zero()) {
    return manager.reboundReplan({0.0, 0.0, 0.5}, velocity, acceleration,
                                 {2.0, 0.0, 0.5}, Eigen::Vector3d::Zero(),
                                 true, false, 1.0);
  }
};

}  // namespace

TEST(ScanMotionIntentBoundary, KeepsNonzeroInitialVelocityAndAccelerationWithinFinalBounds) {
  ScanAttemptFixture fixture(true);
  const Eigen::Vector3d start(0.0, 0.0, 0.5);
  const Eigen::Vector3d initial_velocity(0.1, 0.0, 0.0);
  const Eigen::Vector3d initial_acceleration(0.1, 0.0, 0.0);
  ASSERT_TRUE(fixture.replan(initial_velocity, initial_acceleration))
      << fixture.manager.reboundDebug().stage << ":" << fixture.manager.reboundDebug().reason;
  const auto &position = fixture.manager.local_data_.position_traj_;
  const auto velocity = position.getDerivative();
  const auto acceleration = velocity.getDerivative();
  const double duration = position.getTimeSum();
  EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
  EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
  EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(), 1e-9);
  EXPECT_LE((position.evaluateDeBoorT(duration) - Eigen::Vector3d(2.0, 0.0, 0.5)).norm(), 1e-9);
  EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(), 0.2 + 1e-9);
  EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
}

TEST(ScanMotionIntentRetiming,
     PreservesMovingStartForShortLateralTargetWithinLimits) {
  using namespace nav_kernel::local::scan::upstream;
  const Eigen::Vector3d start(0.6034440286763203, -0.1933650899937829,
                              -0.0011775607026522361);
  const Eigen::Vector3d initial_acceleration = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(0.82179419696400013, 0.25643842712090337,
                               -0.0011775607026522361);
  const std::vector<Eigen::Vector3d> initial_velocities{
      {0.32530685229850964, -0.0578586930675915, -0.026038490671518016},
      {0.5, 0.0, 0.0}};
  for (const Eigen::Vector3d &initial_velocity : initial_velocities) {
    SCOPED_TRACE(::testing::Message() << "initial_velocity="
                                      << initial_velocity.transpose());
    RequestFixture request({{start.x(), start.y(), start.z()},
                            {target.x(), target.y(), target.z()}},
                           0.05);
    auto params = scanParams();
    params.scan.voxelResolution = 0.05;
    params.scan.maxVelocity = 0.5;
    params.scan.maxAcceleration = 0.5;
    nav_kernel::local::scan::Grid grid(params, request.request);

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

    ASSERT_TRUE(manager.reboundReplan(
        start, initial_velocity, initial_acceleration, target,
        Eigen::Vector3d::Zero(), true, false, 1.0))
        << manager.reboundDebug().stage << ":" << manager.reboundDebug().reason;
    const auto &position = manager.local_data_.position_traj_;
    const auto velocity = position.getDerivative();
    const auto acceleration = velocity.getDerivative();
    const double duration = position.getTimeSum();
    EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
    EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
    EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(),
              1e-9);
    EXPECT_LE((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
    EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
    EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
    EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(),
              0.5 + 1e-9);
    EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(),
              0.5 + 1e-9);
  }
}

TEST(ScanMotionIntentRetiming, ConvergesWithNonzeroThreeAxisStartAcceleration) {
  using namespace nav_kernel::local::scan::upstream;
  // Physical boundary from a Go2 replan that exhausted three dilation passes.
  const Eigen::Vector3d start(1.2191940642, -0.2191074321, 0.0653090089);
  const Eigen::Vector3d initial_velocity(0.3141680282, -0.0369077086, 0.1247349412);
  const Eigen::Vector3d initial_acceleration(-0.1668969950, 0.0574541903, -0.1276989702);
  const Eigen::Vector3d target(4.0680212247, 0.1674350790, -0.0102147490);
  RequestFixture request({{start.x(), start.y(), start.z()},
                          {target.x(), target.y(), target.z()}}, 0.05);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Grid grid(params, request.request);
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

  ASSERT_TRUE(manager.reboundReplan(start, initial_velocity, initial_acceleration,
                                  target, Eigen::Vector3d::Zero(), true, false, 1.0))
      << manager.reboundDebug().stage << ":" << manager.reboundDebug().reason;
  const auto &position = manager.local_data_.position_traj_;
  const auto velocity = position.getDerivative();
  const auto acceleration = velocity.getDerivative();
  const double duration = position.getTimeSum();
  EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
  EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
  EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(), 1e-9);
  EXPECT_LE((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
  EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
  EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
}

TEST(ScanBsplineFeasibility, SpeedOnlyViolationUsesActualSpeedRatio) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  constexpr double interval = 0.2;
  Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(3, 10);
  for (int index = 0; index < controls.cols(); ++index)
    controls(0, index) = 0.051 * index * interval;
  UniformBspline spline(controls, 3, interval);
  spline.setPhysicalLimits(0.05, 0.5, 0.0);
  double ratio = 0.0;
  EXPECT_FALSE(spline.checkFeasibility(ratio, false));
  EXPECT_NEAR(ratio, 1.02, 1e-12);
}

TEST(ScanBsplineParameterization, MatchesDenseLeastSquaresWithThreeAxisBoundaryData) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  // Keep the original dense mathematical problem as a reference for the sparse
  // solver, including inconsistent observations that require least squares.
  for (const int count : {7, 93, 772}) {
    SCOPED_TRACE(count);
    const double interval = count == 772 ? 0.09128709291752769 : 0.2;
    Eigen::MatrixXd collocation = Eigen::MatrixXd::Zero(count + 4, count + 2);
    Eigen::MatrixXd observations(count + 4, 3);
    std::vector<Eigen::Vector3d> points;
    for (int index = 0; index < count; ++index) {
      const double phase = static_cast<double>(index) / (count - 1);
      points.emplace_back(3.5 * phase, 0.3 * std::sin(4.0 * phase),
                          0.5 - 0.1 * phase + 0.02 * std::sin(7.0 * phase));
      observations.row(index) = points.back().transpose();
      collocation.block<1, 3>(index, index) << 1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0;
    }
    const std::vector<Eigen::Vector3d> derivatives{
        {-0.05, 0.10, -0.01}, {0.03, -0.04, 0.02},
        {0.02, -0.03, 0.01}, {-0.01, 0.02, -0.03}};
    for (int end = 0; end < 2; ++end) {
      const int column = end * (count - 1);
      collocation.block<1, 3>(count + end, column) <<
          -0.5 / interval, 0.0, 0.5 / interval;
      collocation.block<1, 3>(count + 2 + end, column) <<
          1.0 / (interval * interval), -2.0 / (interval * interval),
          1.0 / (interval * interval);
    }
    for (int index = 0; index < 4; ++index)
      observations.row(count + index) = derivatives[index].transpose();

    const Eigen::MatrixXd dense =
        collocation.colPivHouseholderQr().solve(observations);
    Eigen::MatrixXd sparse;
    UniformBspline::parameterizeToBspline(interval, points, derivatives, sparse);
    ASSERT_EQ(sparse.rows(), 3);
    ASSERT_EQ(sparse.cols(), count + 2);
    ASSERT_TRUE(sparse.allFinite());
    const double control_error = (sparse.transpose() - dense).cwiseAbs().maxCoeff();
    const double residual_error =
        (collocation * sparse.transpose() - observations).squaredNorm() -
        (collocation * dense - observations).squaredNorm();
    EXPECT_LE(control_error, 1e-9);
    EXPECT_NEAR(residual_error, 0.0, 1e-10);
    RecordProperty("control_error_" + std::to_string(count), std::to_string(control_error));
    const UniformBspline dense_spline(dense.transpose(), 3, interval);
    const UniformBspline sparse_spline(sparse, 3, interval);
    for (const double time : {0.0, sparse_spline.getTimeSum()}) {
      EXPECT_LE((sparse_spline.evaluateDeBoorT(time) -
                 dense_spline.evaluateDeBoorT(time)).norm(), 1e-9);
      EXPECT_LE((sparse_spline.getDerivative().evaluateDeBoorT(time) -
                 dense_spline.getDerivative().evaluateDeBoorT(time)).norm(), 1e-8);
      EXPECT_LE((sparse_spline.getDerivative().getDerivative().evaluateDeBoorT(time) -
                 dense_spline.getDerivative().getDerivative().evaluateDeBoorT(time)).norm(), 1e-7);
    }
  }
}

TEST(ScanBsplineParameterization, ReproducesConsistentNonzeroPositionVelocityAcceleration) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  const Eigen::Vector3d position(2.0, -1.0, 0.4);
  const Eigen::Vector3d velocity(-0.12, 0.08, -0.03);
  const Eigen::Vector3d acceleration(0.01, 0.02, -0.003);
  const Eigen::Vector3d jerk(0.001, -0.002, 0.0004);
  const auto point = [&](double time) {
    return Eigen::Vector3d(position + time * velocity +
        0.5 * time * time * acceleration + time * time * time * jerk / 6.0);
  };
  const auto speed = [&](double time) {
    return Eigen::Vector3d(velocity + time * acceleration + 0.5 * time * time * jerk);
  };
  constexpr double interval = 0.2;
  constexpr int count = 25;
  constexpr double duration = (count - 1) * interval;
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index < count; ++index) points.push_back(point(index * interval));
  const std::vector<Eigen::Vector3d> derivatives{
      velocity, speed(duration), acceleration, acceleration + duration * jerk};
  Eigen::MatrixXd controls;
  UniformBspline::parameterizeToBspline(interval, points, derivatives, controls);
  const UniformBspline spline(controls, 3, interval);
  EXPECT_NEAR(spline.getTimeSum(), duration, 1e-12);
  for (const double time : {0.0, duration / 2.0, duration}) {
    EXPECT_LE((spline.evaluateDeBoorT(time) - point(time)).norm(), 1e-10);
    EXPECT_LE((spline.getDerivative().evaluateDeBoorT(time) - speed(time)).norm(), 1e-10);
    EXPECT_LE((spline.getDerivative().getDerivative().evaluateDeBoorT(time) -
               (acceleration + time * jerk)).norm(), 1e-9);
  }
}

TEST(ScanAttemptDiagnostics, DistinguishesCollisionFromDynamicFailureAndClearsOnSuccess) {
  ScanAttemptFixture fixture;
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.request.bitmap.occupy({x, y, z});

  ASSERT_FALSE(fixture.replan());
  const auto failed = fixture.manager.reboundDebug();
  EXPECT_TRUE(failed.attempted);
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(failed.attemptId, 1U);
  EXPECT_EQ(failed.stage, "rebound_optimization");
  EXPECT_EQ(failed.reason, "collision_at_trajectory_start");
  EXPECT_TRUE(failed.optimizerReturnCodeValid);
  EXPECT_TRUE(failed.collisionValid);
  EXPECT_EQ(failed.collisionState, 1);
  EXPECT_GE(failed.collisionTimeS, 0.0);
  EXPECT_EQ(fixture.grid.inflatedOccupancy(
                {failed.collisionPosition.x(), failed.collisionPosition.y(),
                 failed.collisionPosition.z()}, 0.0), 1);
  EXPECT_FALSE(failed.dynamicViolationValid);

  ASSERT_GT(failed.candidateControlPoints.cols(), 3);
  ASSERT_GT(failed.candidateIntervalS, 0.0);
  const nav_kernel::local::scan::upstream::UniformBspline rejected(
      failed.candidateControlPoints, 3, failed.candidateIntervalS);
  EXPECT_LT((rejected.evaluateDeBoorT(failed.collisionTimeS) -
             failed.collisionPosition).norm(), 1e-10);

  fixture.request.bitmap.clear();
  fixture.request.refreshCollision(2);
  fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
  ASSERT_TRUE(fixture.replan()) << fixture.manager.reboundDebug().reason;
  const auto &ready = fixture.manager.reboundDebug();
  EXPECT_EQ(ready.attemptId, 2U);
  EXPECT_TRUE(ready.success);
  EXPECT_EQ(ready.stage, "complete");
  EXPECT_EQ(ready.reason, "accepted");
  EXPECT_FALSE(ready.collisionValid);
  EXPECT_FALSE(ready.dynamicViolationValid);
  EXPECT_EQ(ready.candidateControlPoints.size(), 0);

  EXPECT_FALSE(fixture.manager.reboundReplan(
      {0.0, 0.0, 0.5}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      {0.0, 0.0, 0.5}, Eigen::Vector3d::Zero(), true, false, 2.0));
  const auto &empty = fixture.manager.reboundDebug();
  EXPECT_EQ(empty.attemptId, 3U);
  EXPECT_EQ(empty.stage, "initialization");
  EXPECT_EQ(empty.reason, "target_too_close");
  EXPECT_FALSE(empty.optimizerReturnCodeValid);
  EXPECT_FALSE(empty.collisionValid);
  EXPECT_EQ(empty.candidateControlPoints.size(), 0);
}

TEST(ScanAttemptDiagnostics, RecordsMeasuredBoundaryVelocityAndAccelerationViolations) {
  for (const bool speed : {true, false}) {
    SCOPED_TRACE(speed ? "speed" : "acceleration");
    ScanAttemptFixture fixture;
    fixture.manager.pp_.vel_tolerance_ = 0.0;
    fixture.manager.pp_.acc_tolerance_ = 0.0;
    const Eigen::Vector3d velocity = speed ? Eigen::Vector3d{1.0, 0.0, 0.0}
                                          : Eigen::Vector3d::Zero();
    const Eigen::Vector3d acceleration = speed ? Eigen::Vector3d::Zero()
                                              : Eigen::Vector3d{2.0, 0.0, 0.0};
    ASSERT_FALSE(fixture.replan(velocity, acceleration));
    const auto &failed = fixture.manager.reboundDebug();
    EXPECT_EQ(failed.stage, "dynamic_feasibility");
    EXPECT_EQ(failed.reason, speed ? "speed_limit_exceeded" : "acceleration_limit_exceeded");
    EXPECT_EQ(failed.dynamicQuantity, speed ? "speed" : "acceleration");
    EXPECT_TRUE(failed.dynamicViolationValid);
    EXPECT_GT(failed.dynamicValue, failed.dynamicLimit);
    EXPECT_DOUBLE_EQ(failed.dynamicLimit, 0.5);
    EXPECT_GE(failed.dynamicTimeS, 0.0);
    EXPECT_TRUE(failed.optimizerReturnCodeValid);
    EXPECT_FALSE(failed.collisionValid);
    EXPECT_TRUE(failed.polyInit);
    EXPECT_FALSE(failed.randomPolyInit);
    EXPECT_EQ(failed.startVelocity, velocity);
    EXPECT_EQ(failed.startAcceleration, acceleration);
    ASSERT_GT(failed.candidateControlPoints.cols(), 3);
    const nav_kernel::local::scan::upstream::UniformBspline rejected(
        failed.candidateControlPoints, 3, failed.candidateIntervalS);
    const auto derivative = speed ? rejected.getDerivative()
                                  : rejected.getDerivative().getDerivative();
    EXPECT_NEAR(derivative.evaluateDeBoorT(failed.dynamicTimeS).norm(),
                failed.dynamicValue, 1e-10);
  }
}

TEST(ScanReboundInitialization, FinishesCollisionSegmentBeyondCheckedPrefix) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.5}, {4.0, 0.0, 0.5}});
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 0.4, 4.0, 90.0}, {fixture.route.data(), 2, 1, false}};
  for (int x = 12; x <= 33; ++x)
    for (int y = -3; y <= 2; ++y)
      fixture.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, 0.55});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  const auto map = std::make_shared<GridMap>(grid);
  BsplineOptimizer optimizer;
  BsplineOptimizerParams params;
  params.lambda_smooth = params.lambda_collision = params.lambda_fitness = 1.0;
  params.lambda_feasibility = 0.1;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  optimizer.setParam(params);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, {100, 100, 10});
  Eigen::MatrixXd controls(3, 43);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.1 * (i - 1), 0.0, 0.5};
  ASSERT_EQ(grid.inflatedOccupancy({2.7, 0.0, 0.5}, 0.0), 1);
  const auto guides = optimizer.initControlPoints(controls);
  ASSERT_EQ(guides.size(), 1U);
  ASSERT_GT(guides.front().size(), 2U);
  EXPECT_GT(guides.front().back().x(), 3.3);
  // A* nodes guide the optimizer; the published spline is checked separately.
  for (const auto &point : guides.front())
    EXPECT_EQ(map->getInflateOccupancy(point, 0.0), 0);
}

TEST(ScanMotionIntentBoundary, DetoursAroundObstacleExtendingBeyondCheckedPrefix) {
  ScanAttemptFixture fixture(true);
  for (int x = 6; x <= 16; ++x)
    for (int y = -3; y <= 2; ++y)
      fixture.request.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, 0.55});
  ASSERT_TRUE(fixture.replan()) << fixture.manager.reboundDebug().stage << ":"
                              << fixture.manager.reboundDebug().reason;
  const auto &position = fixture.manager.local_data_.position_traj_;
  const auto velocity = position.getDerivative();
  const auto acceleration = velocity.getDerivative();
  EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(), 0.2 + 1e-9);
  EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
  const int samples = static_cast<int>(std::ceil(position.getTimeSum() / 0.002));
  Eigen::Vector3d previous = position.evaluateDeBoorT(0.0);
  double lateral = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const auto p = position.evaluateDeBoorT(position.getTimeSum() * i / samples);
    ASSERT_EQ(fixture.grid.segmentInflatedOccupancy(
                  {previous.x(), previous.y(), previous.z()}, 0.0,
                  {p.x(), p.y(), p.z()}, 0.0), 0) << "sample " << i;
    lateral = std::max(lateral, std::abs(p.y()));
    previous = p;
  }
  EXPECT_GT(lateral, 0.3);
  EXPECT_LE((previous - Eigen::Vector3d(2.0, 0.0, 0.5)).norm(), 1e-9);
}

TEST(ScanAttemptDiagnostics, RefineReportsItsCollisionAndClearsItOnNextSuccess) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  BsplineOptimizer optimizer;
  BsplineOptimizerParams params;
  params.lambda_smooth = 1.0;
  params.lambda_collision = 0.5;
  params.lambda_feasibility = 0.1;
  params.lambda_fitness = 1.0;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  optimizer.setParam(params);
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3, 9);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.25 * (i - 1), 0.0, 0.5};
  const UniformBspline candidate(controls, 3, 0.5);
  const double sampleInterval = candidate.getTimeSum() / (controls.cols() - 3);
  for (double t = 0.0; t < candidate.getTimeSum() + 1e-4; t += sampleInterval)
    optimizer.ref_pts_.push_back(candidate.evaluateDeBoorT(t));
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.bitmap.occupy({x, y, z});
  Eigen::MatrixXd output;
  ASSERT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  const auto failed = optimizer.optimizationDebug();
  EXPECT_EQ(failed.reason, "collision_after_refine");
  EXPECT_TRUE(failed.optimizerReturnCodeValid);
  EXPECT_TRUE(failed.collisionValid);
  EXPECT_EQ(failed.collisionState, 1);
  EXPECT_GE(failed.collisionTimeS, 0.0);
  const UniformBspline rejected(output, 3, 0.5);
  EXPECT_LT((rejected.evaluateDeBoorT(failed.collisionTimeS) -
             failed.collisionPosition).norm(), 1e-10);
  fixture.bitmap.clear();
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  const auto &ready = optimizer.optimizationDebug();
  EXPECT_EQ(ready.reason, "accepted");
  EXPECT_TRUE(ready.optimizerReturnCodeValid);
  EXPECT_FALSE(ready.collisionValid);
}

TEST(ScanTrajectoryCollision, RejectsObstacleInFinalThirdAndAcceptsClearedTrajectory) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.525}, {4.0, 0.0, 0.525}});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3, 43);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.1 * (i - 1), 0.0, 0.525};
  const UniformBspline candidate(controls, 3, 0.2);
  fixture.bitmap.occupy({3.55, 0.025, 0.525});
  ASSERT_EQ(grid.inflatedOccupancy({3.55, 0.0, 0.525}, 0.0), 1);
  EXPECT_FALSE(optimizer.checkTrajectoryCollisionFree(candidate));
  EXPECT_TRUE(optimizer.optimizationDebug().collisionValid);
  EXPECT_GT(optimizer.optimizationDebug().collisionTimeS, candidate.getTimeSum() * 2.0 / 3.0);
  fixture.bitmap.clear();
  EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(candidate));
  EXPECT_FALSE(optimizer.optimizationDebug().collisionValid);
}

TEST(ScanMotionIntentBoundary, CannotPublishTrajectoryWithBlockedFinalApproach) {
  ScanAttemptFixture fixture(true);
  for (int x = 19; x <= 23; ++x)
    for (int y = -3; y <= 2; ++y)
      for (int z = 2; z <= 8; ++z)
        fixture.request.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, z * 0.1 + 0.05});
  ASSERT_EQ(fixture.grid.inflatedOccupancy({2.0, 0.0, 0.5}, 0.0), 1);
  EXPECT_FALSE(fixture.replan());
  EXPECT_FALSE(fixture.manager.reboundDebug().success);
  EXPECT_TRUE(fixture.manager.reboundDebug().collisionValid);
}

TEST(ScanAttemptDiagnostics, BackendRetainsFirstFailedTickAndOwnsItsCompleteMapAcrossReset) {
  for (const bool shared : {true, false}) {
    SCOPED_TRACE(shared ? "shared" : "borrowed");
    RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
    const auto occupyStart = [&] {
      for (double x = -0.25; x < 0.3; x += 0.1)
        for (double y = -0.25; y < 0.3; y += 0.1)
          for (double z = 0.25; z < 0.8; z += 0.1)
            fixture.bitmap.occupy({x, y, z});
    };
    occupyStart();
    fixture.refreshCollision(2);
    auto &collision = fixture.request.environment.collision;
    std::shared_ptr<const std::vector<std::uint8_t>> originalStorage;
    if (shared) {
      originalStorage = std::make_shared<const std::vector<std::uint8_t>>(
          collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
      collision.inflatedStorage = originalStorage;
      collision.inflatedBits = originalStorage->data();
    }
    nav_kernel::local::scan::Backend backend(scanParams());
    const auto tick = [&] {
      fixture.request.clock.timestampS += 0.01;
      return backend.tick(fixture.request);
    };
    for (int i = 0; i < 20 && !backend.debugSnapshot().lastScanFailure; ++i) tick();
    const auto first = backend.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(first);
    EXPECT_EQ(first->sequence, 1U);
    EXPECT_FALSE(first->attempt.success);
    EXPECT_EQ(first->attempt.stage, "rebound_optimization");
    EXPECT_EQ(first->attempt.reason, "collision_at_trajectory_start");
    EXPECT_TRUE(first->checkObstacle);
    EXPECT_EQ(first->referenceGeneration, 1U);
    EXPECT_FALSE(first->referenceReachesGoal);
    EXPECT_EQ(first->reference.size(), 2U);
    EXPECT_GT(first->candidateControlPoints.size(), 3U);
    ASSERT_TRUE(first->collision.inflatedStorage);
    EXPECT_EQ(first->collision.inflatedBytes, collision.inflatedBytes);
    EXPECT_EQ(first->collision.inflatedBits, first->collision.inflatedStorage->data());
    if (shared) {
      EXPECT_EQ(first->collision.inflatedStorage, originalStorage);
    } else {
      EXPECT_NE(first->collision.inflatedBits, collision.inflatedBits);
      fixture.bitmap.clear();
      nav_kernel::LocalPlanRequest capturedRequest;
      capturedRequest.environment.collision = first->collision;
      nav_kernel::local::scan::Grid capturedGrid(scanParams(), capturedRequest);
      EXPECT_EQ(capturedGrid.inflatedOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
      occupyStart();
    }
    for (int i = 0; i < 4; ++i) tick();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    const auto lastAttempt = backend.debugSnapshot().scanAttempt.attemptId;
    EXPECT_GE(lastAttempt, first->attempt.attemptId);
    (void)backend.tick(fixture.request, [] { return true; });
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    EXPECT_EQ(backend.debugSnapshot().scanAttempt.attemptId, lastAttempt);
    backend.reset();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    tick();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    for (int i = 0; i < 20 && backend.debugSnapshot().lastScanFailure == first; ++i) tick();
    const auto second = backend.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(second);
    EXPECT_NE(second, first);
    EXPECT_EQ(second->sequence, 2U);
  }
}

TEST(ScanAttemptDiagnostics, BackendSuccessfulAttemptRearmsFailureCapture) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  auto params = scanParams();
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {2.0, 0.0, 0.0};
  nav_kernel::local::scan::Backend backend(params);
  const auto tick = [&] {
    fixture.request.clock.timestampS += 0.01;
    return backend.tick(fixture.request);
  };
  for (int i = 0; i < 20 && !backend.debugSnapshot().lastScanFailure; ++i) tick();
  const auto first = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(first);
  EXPECT_EQ(first->attempt.stage, "dynamic_feasibility");
  fixture.request.robot.kinematics.linearVelocity = {};
  LocalPlan ready;
  for (int i = 0; i < 30 && !ready.ready(); ++i) ready = tick();
  ASSERT_TRUE(ready.ready()) << backend.debugSnapshot().searchReason;
  EXPECT_TRUE(backend.debugSnapshot().scanAttempt.success);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.collisionValid);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.dynamicViolationValid);
  EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);

  // A supported speed reduction restarts from measured motion in the same manager.
  fixture.request.maxLinearSpeedMps = 0.3;
  fixture.request.robot.kinematics.linearVelocity = {2.0, 0.0, 0.0};
  for (int i = 0; i < 30 && backend.debugSnapshot().lastScanFailure == first; ++i) tick();
  const auto second = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(second);
  EXPECT_NE(second, first);
  EXPECT_EQ(second->sequence, first->sequence + 1U);
  EXPECT_EQ(second->attempt.stage, "dynamic_feasibility");
}

TEST(ScanAttemptDiagnostics, FacadeRetainsCompletedFailureAcrossResetAndPending) {
  for (const bool consumeCompletion : {false, true}) {
    SCOPED_TRACE(consumeCompletion ? "completion polled" : "completion not polled");
    RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
    for (double x = -0.25; x < 0.3; x += 0.1)
      for (double y = -0.25; y < 0.3; y += 0.1)
        for (double z = 0.25; z < 0.8; z += 0.1)
          fixture.bitmap.occupy({x, y, z});
    fixture.refreshCollision(2);
    nav_kernel::local::Planner planner(scanParams());
    ASSERT_TRUE(planner.configure(""));
    ASSERT_EQ(planner.plan(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!planner.debugSnapshot().lastScanFailure &&
           std::chrono::steady_clock::now() < deadline) {
      if (consumeCompletion)
        (void)planner.plan(fixture.request);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto failed = planner.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(failed);
    if (consumeCompletion) {
      (void)planner.plan(fixture.request);
      EXPECT_TRUE(planner.debugSnapshot().scanAttempt.attempted);
    } else {
      // The completed worker attempt is visible before the normal plan poll.
      EXPECT_FALSE(planner.debugSnapshot().scanAttempt.attempted);
    }
    planner.reset();
    EXPECT_EQ(planner.debugSnapshot().lastScanFailure, failed);
    fixture.bitmap.clear();
    fixture.refreshCollision(3);
    const LocalPlan pending = planner.plan(fixture.request);
    EXPECT_EQ(pending.status(), nav_kernel::LocalPlanStatus::Pending);
    EXPECT_FALSE(pending.ready());
    EXPECT_EQ(planner.debugSnapshot().lastScanFailure, failed);
  }
}

TEST(ScanAttemptDiagnostics, CancelledAttemptIsNotCapturedByNextCollisionTick) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.bitmap.occupy({x, y, z});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Backend backend(scanParams());
  for (int i = 0; i < 10 &&
       backend.debugSnapshot().searchReason != "scan_generate_trajectory"; ++i) {
    fixture.request.clock.timestampS += 0.01;
    (void)backend.tick(fixture.request);
  }
  ASSERT_EQ(backend.debugSnapshot().searchReason, "scan_generate_trajectory");
  ASSERT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  int cancelChecks = 0;
  const LocalPlan cancelled = backend.tick(fixture.request, [&] {
    return ++cancelChecks == 2;
  });
  ASSERT_EQ(cancelChecks, 2);
  EXPECT_EQ(cancelled.status(), nav_kernel::LocalPlanStatus::Cancelled);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  EXPECT_FALSE(backend.debugSnapshot().lastScanFailure);
  fixture.request.clock.timestampS += 0.01;
  (void)backend.checkCollision(fixture.request);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  EXPECT_FALSE(backend.debugSnapshot().lastScanFailure);
  fixture.request.clock.timestampS += 0.01;
  (void)backend.tick(fixture.request);
  const auto accepted = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(accepted);
  EXPECT_EQ(accepted->sequence, 1U);
  EXPECT_GE(accepted->attempt.attemptId, 2U);
  EXPECT_EQ(accepted->attempt.reason, "collision_at_trajectory_start");
}

TEST(ScanTrajectoryValidation, SamplesFullTrajectoriesAcrossRoutesAndObstacles) {
  const std::vector<std::vector<Vec3>> routes{
      {{0, 0, 0.5}, {2, 0, 0.5}},
      {{0, 0, 0.5}, {0, 2, 0.5}},
      {{0, 0, 0.5}, {-2, -1, 0.5}},
      {{0, 0, 0.5}, {1, 0.75, 0.5}, {2, 0, 0.5}},
      {{0, 0, 0.5}, {1, 0, 0.5}, {2.5, 0, 0.5}},
  };
  for (std::size_t scene = 0; scene < routes.size(); ++scene) {
    SCOPED_TRACE(scene);
    RequestFixture fixture(routes[scene], 0.05);
    LocalPlannerParams params;
    params.backend = LocalPlannerBackend::Scan;
    params.scan.maxVelocity = 0.75;
    params.scan.maxAcceleration = 0.50;
    if (scene == 4) {
      // Already inflated Mapd cells: the core must detour, not inflate again.
      for (int x = 16; x <= 24; ++x)
        for (int y = -8; y <= 8; ++y)
          for (int z = -20; z < 40; ++z)
            fixture.bitmap.occupy({0.05 * x + 0.025, 0.05 * y + 0.025, 0.05 * z + 0.025});
    }
    fixture.refreshCollision(2);
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int tick = 0; tick < 50 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS = 1.0 + tick * 0.01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
    ASSERT_TRUE(spline.valid());
    ASSERT_GT(spline.duration(), 0.0);
    nav_kernel::local::scan::Grid grid(params, fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    const auto start = spline.position(0.0);
    EXPECT_NEAR(start.x, fixture.route.front().x, 0.02);
    EXPECT_NEAR(start.y, fixture.route.front().y, 0.02);
    const auto end = spline.position(spline.duration());
    EXPECT_NEAR(end.x, fixture.route.back().x, 0.15);
    EXPECT_NEAR(end.y, fixture.route.back().y, 0.15);
    const int samples = static_cast<int>(std::ceil(spline.duration() / 0.002));
    int occupiedSamples = 0;
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;
    for (int i = 0; i <= samples; ++i) {
      const double t = spline.duration() * i / samples;
      const auto position = spline.position(t);
      const auto velocity = spline.velocity(t);
      ASSERT_TRUE(std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z));
      ASSERT_TRUE(std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z));
      const double yaw = std::atan2(velocity.y, velocity.x);
      if (!grid.obstacleFree(position, yaw)) ++occupiedSamples;
      maxVelocity = std::max(maxVelocity, std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
      // Finite differences audit the emitted trajectory independently of the optimizer's feasibility check.
      constexpr double delta = 0.0001;
      if (t >= delta && t <= spline.duration() - delta) {
        const auto before = spline.velocity(t - delta);
        const auto after = spline.velocity(t + delta);
        const double ax = (after.x - before.x) / (2 * delta);
        const double ay = (after.y - before.y) / (2 * delta);
        const double az = (after.z - before.z) / (2 * delta);
        maxAcceleration = std::max(maxAcceleration, std::sqrt(ax * ax + ay * ay + az * az));
      }
    }
    EXPECT_EQ(occupiedSamples, 0);
    EXPECT_LE(maxVelocity, params.scan.maxVelocity + params.scan.velocityTolerance + 0.001);
    EXPECT_LE(maxAcceleration, params.scan.maxAcceleration + params.scan.accelerationTolerance + 0.001);
    std::cout << "SCAN scene=" << scene << " samples=" << samples + 1
              << " max_velocity=" << maxVelocity << " max_acceleration=" << maxAcceleration
              << " occupied_samples=" << occupiedSamples << '\n';
  }
}

TEST(ScanDefaults, UsesOfficialGridAndSearchScale) {
  const nav_kernel::ScanPlannerParams params;
  EXPECT_DOUBLE_EQ(params.voxelResolution, 0.05);
  EXPECT_DOUBLE_EQ(params.cylinderOffset, 0.18);
  EXPECT_DOUBLE_EQ(nav_kernel::local::scan::Backend::fsmPeriodS(), 0.01);
  EXPECT_DOUBLE_EQ(nav_kernel::local::scan::Backend::collisionPeriodS(), 0.05);
}

TEST(ScanBackend, FailedInitializationReportsBlockedAndRecoversAfterMapChange) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {2.025, 0.025, 0.525}}, 0.05);
  for (int x = -3; x <= 3; ++x)
    for (int y = -3; y <= 3; ++y)
      fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 8; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.status() == nav_kernel::LocalPlanStatus::Blocked) break;
  }
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_initialization_failed");
  EXPECT_EQ(backend.checkCollision(fixture.request).status(), nav_kernel::LocalPlanStatus::Blocked);
  fixture.bitmap = CollisionBitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, 0.05);
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
}

TEST(ScanBackend, BlockedForwardTargetsDoNotRemainPendingAndRecoverWhenClear) {
  // The distant goal is outside the rolling map. Only a few centimetres of
  // the reference remain free, as at the captured stair entry.
  RequestFixture fixture({{0, 0, 0.5}, {6, 0, 0.5}}, 0.05);
  for (int x = 1; x < 100; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
  EXPECT_FALSE(plan.ready());
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
}

TEST(ScanBackend, MotionIntentFindsEitherOpenSideWhenReferenceTargetOrConnectionIsBlocked) {
  for (const int wallEnd : {18, 45}) {
    SCOPED_TRACE(wallEnd);
    for (const double openSide : {-1.0, 1.0}) {
      SCOPED_TRACE(openSide);
      RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
      for (int x = 1; x < wallEnd; ++x)
        for (int y = -2; y <= 2; ++y)
          for (int z = 1; z <= 9; ++z)
            fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
      for (int x = -45; x < 45; ++x)
        for (int y = 1; y < 45; ++y)
          for (int z = 1; z <= 9; ++z)
            fixture.bitmap.occupy({x * 0.1 + 0.025, -openSide * y * 0.1 + 0.025,
                                  z * 0.1 + 0.025});
      fixture.refreshCollision(2);
      fixture.request.objective = nav_kernel::MotionIntentTarget{
          {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
      fixture.request.maxLinearSpeedMps = 0.5;
      const auto params = scanParams();
      const nav_kernel::local::scan::Grid grid(params, fixture.request);
      ASSERT_EQ(grid.inflatedOccupancy(fixture.route.back(), 0.0), wallEnd == 45 ? 1 : 0);
      ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                            fixture.route.back(), 0.0), 1);
      nav_kernel::local::scan::Backend backend(params);
      LocalPlan plan;
      for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
        fixture.request.clock.timestampS += 0.01;
        plan = backend.tick(fixture.request);
      }
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
      const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
      const auto endpoint = spline.position(spline.duration());
      ASSERT_GT(openSide * (endpoint.y - 0.025), 0.5);
      EXPECT_GE(endpoint.x, 0.025 - 1e-8);
      for (double time = 0.0; time < spline.duration(); time += 0.01) {
        const auto position = spline.position(time);
        const auto next = spline.position(std::min(time + 0.01, spline.duration()));
        ASSERT_EQ(grid.segmentInflatedOccupancy(position, 0.0, next, 0.0), 0);
        const auto velocity = spline.velocity(time);
        const double nextTime = std::min(time + 0.01, spline.duration());
        const auto nextVelocity = spline.velocity(nextTime);
        EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
        EXPECT_LE(std::hypot(nextVelocity.x - velocity.x, nextVelocity.y - velocity.y) /
                      (nextTime - time), 0.5 + 1e-6);
      }
    }
  }
}

TEST(ScanBackend, MotionIntentShortensBlockedConnectionWhenSideTargetsAreUnavailable) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = -49; x < 49; ++x)
    for (int z = -9; z < 19; ++z) {
      for (const double y : {-0.175, 0.225})
        fixture.bitmap.occupy({x * 0.1 + 0.025, y, z * 0.1 + 0.025});
    }
  for (int y = -2; y <= 2; ++y)
    for (int z = -9; z < 19; ++z)
      fixture.bitmap.occupy({0.825, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
  fixture.request.maxLinearSpeedMps = 0.5;
  const auto params = scanParams();
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.inflatedOccupancy(fixture.route.back(), 0.0), 0);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        fixture.route.back(), 0.0), 1);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const auto endpoint = spline.position(spline.duration());
  EXPECT_GT(endpoint.x, 0.4);
  EXPECT_LT(endpoint.x, 0.8);
  EXPECT_NEAR(endpoint.y, 0.025, 1e-6);
  for (double t = 0.0; t < spline.duration(); t += 0.01) {
    ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
        spline.position(std::min(t + 0.01, spline.duration())), 0.0), 0);
    const auto velocity = spline.velocity(t);
    EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
  }
}

TEST(ScanBackend, MotionIntentPrefersAvailableRequestedDirectionOverLongSideDetour) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {0.025, 3.525, 0.525}}, 0.05);
  // Left has room for a shorter advance before the wall. Forward/backward
  // detours are longer, but should not override an immediately usable A input.
  for (int x = -99; x < 99; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {90.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        fixture.route.back(), 0.0), 1);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        {0.025, 0.900, 0.525}, 0.0), 0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const auto endpoint = spline.position(spline.duration());
  EXPECT_NEAR(endpoint.x, 0.025, 1e-6);
  EXPECT_GT(endpoint.y, 0.5);
  EXPECT_LT(endpoint.y, 1.025);
  for (double t = 0.0; t < spline.duration(); t += 0.01)
    ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
        spline.position(std::min(t + 0.01, spline.duration())), 0.0), 0);
}

TEST(ScanBackend, MotionIntentUsesShortSideTargetsWhenLongCandidatesAreBlocked) {
  for (const double distance : {0.4, 0.3, 0.2}) {
    SCOPED_TRACE(distance);
    for (const double openSide : {-1.0, 1.0}) {
      SCOPED_TRACE(openSide);
      for (const bool referenceBlocked : {false, true}) {
        SCOPED_TRACE(referenceBlocked);
        RequestFixture fixture({{0.025, 1.025, 0.525}, {3.525, 1.025, 0.525}}, 0.05);
        // A narrow lateral pocket has room for this short target, while the
        // forward connection and all longer side targets cross a wall.
        for (int x = -99; x < 99; ++x) {
          fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025 - openSide * 0.05, 0.525});
          fixture.bitmap.occupy({x * 0.05 + 0.025,
                                1.025 + openSide * (distance + 0.05), 0.525});
        }
        for (int y = -99; y < 99; ++y)
          fixture.bitmap.occupy({0.075, y * 0.05 + 0.025, 0.525});
        if (referenceBlocked) {
          for (int x = 2; x < 75; ++x)
            fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
        }
        fixture.refreshCollision(2);
        fixture.request.objective = nav_kernel::MotionIntentTarget{
            {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
        fixture.request.maxLinearSpeedMps = 0.5;
        auto params = scanParams();
        params.scan.voxelResolution = 0.05;
        const nav_kernel::local::scan::Grid grid(params, fixture.request);
        ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                              fixture.route.back(), 0.0), 1);
        ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
            {0.025, 1.025 + openSide * 0.5, 0.525}, 0.0), 1);
        nav_kernel::local::scan::Backend backend(params);
        LocalPlan plan;
        bool referenceAttempted = false;
        for (int tick = 0; tick < 100 && !plan.ready(); ++tick) {
          fixture.request.clock.timestampS += 0.01;
          plan = backend.tick(fixture.request);
          const auto failure = backend.debugSnapshot().lastScanFailure;
          if (!referenceBlocked && !referenceAttempted && failure) {
            // Preserve the existing reference optimizer before adding short
            // targets; some blocked straight guides still yield a safe curve.
            EXPECT_GT(failure->targetPosition.x, 1.0);
            EXPECT_NEAR(failure->targetPosition.y, 1.025, 1e-6);
            referenceAttempted = true;
          }
        }
        EXPECT_EQ(referenceAttempted, !referenceBlocked);
        ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
        const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
        const auto endpoint = spline.position(spline.duration());
        EXPECT_NEAR(endpoint.x, 0.025, 1e-6);
        EXPECT_NEAR(openSide * (endpoint.y - 1.025), distance, 1e-6);
        for (double t = 0.0; t < spline.duration(); t += 0.01) {
          const double nextTime = std::min(t + 0.01, spline.duration());
          ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
              spline.position(nextTime), 0.0), 0);
          const auto velocity = spline.velocity(t);
          const auto nextVelocity = spline.velocity(nextTime);
          EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
          EXPECT_LE(std::hypot(nextVelocity.x - velocity.x, nextVelocity.y - velocity.y) /
                        (nextTime - t), 0.5 + 1e-6);
        }
      }
    }
  }
}

TEST(ScanBackend, MotionIntentDoesNotDetourOutsideAllowedDirection) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 0.0}, {fixture.route.data(), 2, 1, false}};
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
}

TEST(ScanReplanFsm, FailedMotionIntentDetourAdvancesOnNextTimerCallback) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy(
            {x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.maxVelocity = 0.05;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  PlanParameters plan;
  plan.max_vel_ = 0.05;
  plan.max_acc_ = 0.5;
  plan.max_jerk_ = 4.0;
  plan.ctrl_pt_dist = 0.2;
  plan.planning_horizon_ = 3.5;
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
  ScanReplanParams fsmParams;
  fsmParams.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  fsmParams.planningHorizon = 3.5;
  fsmParams.noReplanThreshold = 0.1;
  fsmParams.replanThreshold = 1.0;
  SCANReplanFSM fsm(manager, fsmParams);
  FsmInput input;
  input.nowS = 1.0;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0.025, 0.025, 0.525};
  input.odometry->velocity = {0.1, 0.0, 0.0};
  input.motionIntentMaxDeviationRad = M_PI / 2.0;
  input.referencePath = std::vector<Eigen::Vector3d>{
      {0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  input.nowS += 0.01;
  (void)fsm.tick(input);
  input.nowS += 0.01;
  (void)fsm.tick(input);

  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d firstTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_FALSE(manager.reboundDebug().randomPolyInit);
  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d retriedTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_TRUE(manager.reboundDebug().randomPolyInit);
  EXPECT_LE((retriedTarget - firstTarget).norm(), 1e-9);
  fixture.bitmap.occupyInflated(
      {static_cast<float>(firstTarget.x()), static_cast<float>(firstTarget.y()),
       static_cast<float>(firstTarget.z())},
      0.15, 0.3, 0.3);
  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d secondTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_FALSE(manager.reboundDebug().randomPolyInit);
  EXPECT_GT((secondTarget - firstTarget).head<2>().norm(), 0.2);
  EXPECT_NEAR((secondTarget - input.odometry->position).head<2>().norm(),
              (firstTarget - input.odometry->position).head<2>().norm(), 1e-6);
}

TEST(ScanBackend, MotionIntentKeepsSideTargetThenRejoinsClearedReference) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
  fixture.request.maxLinearSpeedMps = 0.5;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto firstTarget = firstView.position(firstView.duration());
  fixture.request.robot.pose.position = firstView.position(firstView.duration() * 0.5);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = firstView.velocity(firstView.duration() * 0.5);
  fixture.request.clock.timestampS = first.startTimeS + firstView.duration() * 0.5;
  for (int tick = 0; tick < 30; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId) break;
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto continued = std::get<SplineTarget>(plan.target());
  ASSERT_GT(continued.trajectoryId, first.trajectoryId);
  const nav_kernel::SplineView continuedView(continued);
  const auto continuedTarget = continuedView.position(continuedView.duration());
  EXPECT_NEAR(continuedTarget.x, firstTarget.x, 1e-6);
  EXPECT_NEAR(continuedTarget.y, firstTarget.y, 1e-6);

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.robot.kinematics.linearVelocity = {};
  fixture.request.clock.timestampS = continued.startTimeS + continuedView.duration() + 0.2;
  for (int tick = 0; tick < 30; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > continued.trajectoryId) break;
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto rejoined = std::get<SplineTarget>(plan.target());
  ASSERT_GT(rejoined.trajectoryId, continued.trajectoryId);
  const nav_kernel::SplineView rejoinedView(rejoined);
  const auto rejoinedTarget = rejoinedView.position(rejoinedView.duration());
  EXPECT_NEAR(rejoinedTarget.x, fixture.route.back().x, 0.02);
  EXPECT_NEAR(rejoinedTarget.y, fixture.route.back().y, 0.02);
}

TEST(ScanBackend, MotionIntentRejectsFreeSideEndpointsBehindAnEnclosure) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  for (int i = -2; i <= 2; ++i)
    for (int z = 1; z <= 9; ++z)
      for (const double side : {-1.0, 1.0}) {
        fixture.bitmap.occupy({i * 0.1 + 0.025, side * 0.2 + 0.025, z * 0.1 + 0.025});
        fixture.bitmap.occupy({side * 0.2 + 0.025, i * 0.1 + 0.025, z * 0.1 + 0.025});
      }
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
  const auto params = scanParams();
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.inflatedOccupancy({0.025, 2.025, 0.525}, 0.0), 0);
  ASSERT_EQ(grid.inflatedOccupancy({0.025, -1.975, 0.525}, 0.0), 0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
}

TEST(ScanBackend, FinalApproachBelowTwentyCentimetresProducesMotionTrajectory) {
  RequestFixture fixture({{0, 0, 0.5}, {0.18, 0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_NEAR(spline.position(spline.duration()).x, 0.18, 0.02);
}

TEST(ScanBackend, OccupiedEndpointFallbackRetainsGoalForMapRecovery) {
  RequestFixture fixture({{0, 0, 0.5}, {2, 0, 0.5}}, 0.05);
  for (int x = 1; x < 50; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_NEAR(spline.position(spline.duration()).x, 2.0, 0.02);
}

TEST(ScanTrajectoryValidation, TranslatedWallHasSafePrefixAndReplansBeforeUnsafeTail) {
  RequestFixture fixture({{10.0, -3.0, 0.0}, {10.0, -2.2, 0.0},
                          {10.0, -1.4, 0.0}, {10.0, -0.6, 0.0}});
  fixture.bitmap = CollisionBitmap({5.0, -8.0, -2.0}, {15.0, 2.0, 2.0}, 0.1);
  for (int forward = 8; forward <= 16; ++forward)
    for (int lateral = -11; lateral <= 11; ++lateral)
      for (int vertical = -4; vertical <= 4; ++vertical)
        fixture.bitmap.occupy({static_cast<float>(10.0 - lateral * 0.1),
                               static_cast<float>(-3.0 + forward * 0.1),
                               static_cast<float>(vertical) * 0.1F});
  fixture.refreshCollision(2);
  fixture.request.robot.pose.yaw = std::acos(-1.0) * 0.5;
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.cylinderOffset = 0.18;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 50 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView spline(original);
  ASSERT_TRUE(spline.valid());
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const int samples = static_cast<int>(std::ceil(spline.duration() / 0.002));
  const double checkedPrefix = spline.duration() * 2.0 / 3.0;
  double firstUnsafeTail = spline.duration();
  for (int index = 0; index <= samples; ++index) {
    const double time = spline.duration() * index / samples;
    const Vec3 position = spline.position(time);
    const Vec3 velocity = spline.velocity(time);
    ASSERT_TRUE(std::isfinite(position.x) && std::isfinite(position.y) &&
                std::isfinite(position.z));
    ASSERT_TRUE(std::isfinite(velocity.x) && std::isfinite(velocity.y) &&
                std::isfinite(velocity.z));
    const bool free = grid.obstacleFree(position, std::atan2(velocity.y, velocity.x));
    if (time <= checkedPrefix) {
      EXPECT_TRUE(free) << "time=" << time << " fraction=" << time / spline.duration();
    } else if (!free) {
      firstUnsafeTail = std::min(firstUnsafeTail, time);
    }
  }
  if (firstUnsafeTail < spline.duration()) {
    // The upstream rolling-horizon contract checks the tail as execution enters it.
    double time = checkedPrefix + 0.01;
    bool replaced = false;
    for (int tick = 0; tick < 20 && time < firstUnsafeTail && !replaced; ++tick, time += 0.01) {
      fixture.request.clock.timestampS = original.startTimeS + time;
      fixture.request.robot.pose.position = spline.position(time);
      fixture.request.robot.kinematics.linearVelocity = spline.velocity(time);
      fixture.request.robot.kinematics.valid = true;
      const Vec3 velocity = fixture.request.robot.kinematics.linearVelocity;
      fixture.request.robot.pose.yaw = std::atan2(velocity.y, velocity.x);
      plan = tick == 0 ? backend.checkCollision(fixture.request) : backend.tick(fixture.request);
      replaced = plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop ||
                 (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId != original.trajectoryId);
    }
    ASSERT_TRUE(replaced) << "the unsafe tail must be replaced or stopped before its first collision";
    if (plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop) {
      EXPECT_FALSE(plan.ready());
      return;
    }
    const nav_kernel::SplineView replacement(std::get<SplineTarget>(plan.target()));
    ASSERT_TRUE(replacement.valid());
    for (double time = 0.0; time < replacement.duration() * 2.0 / 3.0; time += 0.002) {
      const Vec3 position = replacement.position(time);
      const Vec3 velocity = replacement.velocity(time);
      EXPECT_TRUE(grid.obstacleFree(position, std::atan2(velocity.y, velocity.x)));
    }
  }
}

TEST(ScanTrajectoryValidation, EmergencyStopIsNotReportedAsReadyAndRecoversWhenClear) {
  RequestFixture fixture({{0, 0, 0.5}, {2, 0, 0.5}}, 0.05);
  LocalPlannerParams params;
  params.backend = LocalPlannerBackend::Scan;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + tick * 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto firstId = std::get<SplineTarget>(plan.target()).trajectoryId;
  // A wall fills the complete local volume ahead; there is no detour.
  for (int x = 7; x <= 17; ++x)
    for (int y = -100; y < 100; ++y)
      for (int z = -20; z < 40; ++z)
        fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, z * 0.05 + 0.025});
  fixture.request.clock.timestampS += 0.10;
  fixture.refreshCollision(2);
  plan = backend.checkCollision(fixture.request);
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::NearFieldStop);
  ASSERT_FALSE(plan.ready());
  EXPECT_TRUE(plan.previewPath().empty());
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_emergency_stop");
  for (int tick = 0; tick < 3; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    EXPECT_FALSE(plan.ready());
  }

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto &target = std::get<SplineTarget>(plan.target());
  EXPECT_GT(target.trajectoryId, firstId);
  const nav_kernel::SplineView spline(target);
  ASSERT_TRUE(spline.valid());
  EXPECT_GT(spline.position(spline.duration()).x, 1.0);
}

TEST(ScanGridAdapter, DisabledCollisionCheckingPreservesSamplingResolution) {
  auto params = scanParams();
  params.checkObstacle = false;
  LocalPlanRequest request;
  nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid());
  EXPECT_DOUBLE_EQ(grid.resolution(), params.scan.voxelResolution);
  EXPECT_GT(grid.resolution(), 0.0);
  const nav_kernel::local::scan::upstream::GridMap map(grid);
  EXPECT_DOUBLE_EQ(map.getResolution(), params.scan.voxelResolution);
  EXPECT_EQ(map.getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 0);
  EXPECT_EQ(map.getInflateOccupancySegment({-100.0, 0.0, 0.5}, 0.0,
                                          {100.0, 0.0, 0.5}, 0.0), 0);

  params.scan.voxelResolution = 0.0;
  nav_kernel::local::scan::Grid invalid(params, request);
  EXPECT_FALSE(invalid.valid());
  EXPECT_EQ(invalid.reason(), "grid_resolution_invalid");
}

TEST(ScanGridAdapter, SegmentDetectsThinCellsBetweenBothCylinderCenters) {
  const Vec3 start{-0.45, 0.05, 0.55};
  const Vec3 end{0.65, 0.05, 0.55};
  const double yaw = std::acos(-1.0) * 0.5;
  auto params = scanParams();
  params.scan.cylinderOffset = 0.2;
  for (const double sign : {-1.0, 1.0}) {
    RequestFixture fixture({start, end});
    fixture.bitmap.occupy({0.05, 0.05 + sign * params.scan.cylinderOffset, 0.55});
    fixture.refreshCollision(2);
    nav_kernel::local::scan::Grid grid(params, fixture.request);
    ASSERT_EQ(grid.inflatedOccupancy(start, yaw), 0);
    ASSERT_EQ(grid.inflatedOccupancy(end, yaw), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy(start, yaw, end, yaw), 1);
    EXPECT_EQ(grid.segmentInflatedOccupancy(end, yaw, start, yaw), 1);
    EXPECT_EQ(grid.segmentInflatedOccupancy(start, yaw, start, yaw), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy({-5.1, 0.05, 0.55}, yaw, start, yaw), -1);
  }
}

TEST(ScanGridAdapter, SegmentTransformsPlanningCoordinatesOnce) {
  const Vec3 start{-0.45, 0.05, 0.55};
  const Vec3 end{0.65, 0.05, 0.55};
  RequestFixture fixture({start, end});
  fixture.bitmap.occupy({1.25, -0.55, 0.75});
  fixture.refreshCollision(2);
  fixture.request.environment.collision.gridFromPlanningTranslation = {1.3, -0.6, 0.2};
  fixture.request.environment.collision.gridFromPlanningYaw = std::acos(-1.0) * 0.5;
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  const nav_kernel::local::scan::upstream::GridMap map(grid);
  ASSERT_EQ(map.getInflateOccupancy({start.x, start.y, start.z}, 0.0), 0);
  ASSERT_EQ(map.getInflateOccupancy({end.x, end.y, end.z}, 0.0), 0);
  EXPECT_EQ(map.getInflateOccupancySegment({start.x, start.y, start.z}, 0.0,
                                          {end.x, end.y, end.z}, 0.0), 1);
}

TEST(ScanGridAdapter, SegmentChecksVerticalCellsAndCornerContact) {
  RequestFixture fixture({{0.05, 0.05, 0.05}, {0.05, 0.05, 0.95}});
  fixture.bitmap.occupy({0.05, 0.05, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid vertical(scanParams(), fixture.request);
  EXPECT_EQ(vertical.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                              fixture.route.back(), 0.0), 1);

  fixture.bitmap.clear();
  fixture.bitmap.occupy({0.15, 0.15, 0.55});
  fixture.refreshCollision(3);
  nav_kernel::local::scan::Grid corner(scanParams(), fixture.request);
  ASSERT_EQ(corner.inflatedOccupancy({0.15, 0.05, 0.55}, 0.0), 0);
  ASSERT_EQ(corner.inflatedOccupancy({0.05, 0.15, 0.55}, 0.0), 0);
  EXPECT_EQ(corner.segmentInflatedOccupancy({0.15, 0.05, 0.55}, 0.0,
                                           {0.05, 0.15, 0.55}, 0.0), 1);
}

TEST(ScanGridAdapter, LateralIntentUsesBodyFootprintInsteadOfPathTangent) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {0.05, 2.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.30, 0.75, 0.55});
  fixture.refreshCollision(2);
  const Vec3 end{0.05, 1.05, 0.55};
  nav_kernel::local::scan::Grid autonomous(params, fixture.request);
  EXPECT_EQ(autonomous.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 0);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {90.0, 0.5, 2.0, 90.0}, {fixture.route.data(), 2, 1, false}};
  nav_kernel::local::scan::Grid lateral(params, fixture.request);
  EXPECT_EQ(lateral.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 1);
  EXPECT_EQ(lateral.inflatedOccupancy({0.05, 0.75, 0.55}, M_PI / 2), 1);
}

TEST(ScanGridAdapter, BrakingSweepUsesActualPoseAndLatencyBeforeStopping) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.70, 0.45, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const nav_kernel::Pose actual{{0.05, 0.45, 0.55}, 0.0};
  // The planned centreline is clear; the physical robot has deviated sideways.
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0.5, 0, 0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0.5, 0, 0}, 0.35, 0.5, 1.0), 1);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0.5, 0, 0}, 0.0, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {-0.5, 0, 0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0, -0.5, 0}, 0.35, 0.5, 1.0), 0);
}

TEST(ScanGridAdapter, BrakingSweepChecksRotationAndMapBoundary) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.20, 0.20, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0, 0, 1.0}, 0.35, 0.5, 1.0), 1);
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0, 0, -1.0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy({{4.55, 0.05, 0.55}, 0}, {0.5, 0, 0}, 0.35, 0.5, 1.0), -1);
}

TEST(ScanGridAdapter, BrakingLimitApproachesStairsWithoutInventingBodyHeight) {
  RequestFixture fixture({{0.05, 0.05, 0.47}, {2.05, 0.05, 1.47}}, 0.05);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  params.scan.cylinderOffset = 0.25;
  // Inflated upper treads intersect a fast, level braking sweep even though
  // the robot can still approach the first step at a lower speed.
  for (int x = 18; x <= 40; ++x)
    for (int y = -10; y <= 10; ++y)
      fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, 0.475});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const auto body = fixture.request.robot.pose;
  ASSERT_EQ(grid.brakingOccupancy(body, {0.75, 0, 0}, 0.35, 0.5, 1.0), 1);
  const double scale = grid.brakingScale(body, {0.75, 0, 0}, 0.35, 0.5, 1.0);
  EXPECT_GT(scale, 0.3);
  EXPECT_LT(scale, 1.0);
  EXPECT_EQ(grid.brakingOccupancy(body, {0.75 * scale, 0, 0}, 0.35, 0.5, 1.0), 0);
  // Moving the real body into that tread must still stop. The ascending
  // reference endpoint cannot authorize lifting the safety footprint.
  EXPECT_DOUBLE_EQ(grid.brakingScale({{0.70, 0.05, 0.47}, 0},
                                      {0.75, 0, 0}, 0.35, 0.5, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(grid.brakingScale(body, {-0.5, 0, 0}, 0.35, 0.5, 1.0), 1.0);
}

TEST(ScanGridAdapter, BrakingLimitRetainsCollisionCheckedTurningArc) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.50, 0.25, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const auto body = fixture.request.robot.pose;
  const nav_kernel::Twist command{0.5, 0, 1.0};
  ASSERT_NE(grid.brakingOccupancy(body, command, 0.35, 0.5, 1.0), 0);
  const double scale = grid.brakingScale(body, command, 0.35, 0.5, 1.0);
  EXPECT_GT(scale, 0.0);
  EXPECT_LT(scale, 1.0);
  EXPECT_EQ(grid.brakingOccupancy(body, {0.5 * scale, 0, scale}, 0.35, 0.5, 1.0), 0);
}

TEST(ScanGridAdapter, ReadsMapdInflatedBitsWithoutReinflating) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.bitmap.occupy({0.0, 0.0, 0.5});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.occupiedCellCount(), 1);
  EXPECT_EQ(grid.collisionPointCount(), 1);

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  EXPECT_EQ(map->getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
  EXPECT_EQ(map->getInflateOccupancy({0.5, 0.0, 0.5}, 0.0), 0);
}

TEST(ScanGridAdapter, PreservesOfficialBoundaryAndDoubleCylinderSemantics) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_EQ(grid.inflatedOccupancy({-4.7501, 0.0, 0.5}, 3.14159265358979323846), -1);
  EXPECT_EQ(grid.inflatedOccupancy({-4.7498, 0.0, 0.5}, 3.14159265358979323846), 0);

  fixture.bitmap.occupy({0.25, 0.0, 0.5});
  fixture.refreshCollision(3);
  nav_kernel::local::scan::Grid occupiedGrid(params, fixture.request);
  EXPECT_EQ(occupiedGrid.inflatedOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
}

TEST(ScanDynAStar, MovesOccupiedStartBackwardLikeUpstream) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {1.5, 0.0, 0.5}});
  fixture.bitmap.occupy({0.0, 0.0, 0.5});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  nav_kernel::local::scan::upstream::AStar search;
  search.initGridMap(map, {40, 40, 20});
  ASSERT_EQ(search.search(0.1, {0.0, 0.0, 0.5}, {1.5, 0.0, 0.5}),
            nav_kernel::local::scan::upstream::AStarResult::Success);
  const auto path = search.path();
  ASSERT_GE(path.size(), 2U);
  EXPECT_LT(path.front().x(), -0.05);
  EXPECT_NEAR(path.back().x(), 1.5, 0.11);
}

TEST(ScanDynAStar, DoesNotPublishPartialProgressPath) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  for (int x = -5; x <= 5; ++x) {
    for (int y = -50; y <= 50; ++y)
      fixture.bitmap.occupy(
          {0.1 * static_cast<double>(x), 0.1 * static_cast<double>(y), 0.5});
  }
  fixture.refreshCollision(2);
  ASSERT_TRUE(fixture.request.environment.collision.occupied({0.0, 0.1, 0.5}));
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 0.1, 0.5}, 0.0), 1);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 1.9, 0.5}, 0.0), 1);
  nav_kernel::local::scan::upstream::AStar search;
  search.initGridMap(map, {40, 40, 20});

  const auto result = search.search(0.1, {-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5});
  const auto blockedPath = search.path();
  EXPECT_EQ(result, nav_kernel::local::scan::upstream::AStarResult::SearchError);
  EXPECT_TRUE(blockedPath.empty());
}

TEST(ScanBackend, EmitsOfficialBsplineAfterFsmTransitions) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());

  LocalPlan plan;
  for (int tick = 0; tick < 8 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    fixture.refreshCollision(1);
    plan = backend.tick(fixture.request);
  }

  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto *spline = std::get_if<SplineTarget>(&plan.target());
  ASSERT_NE(spline, nullptr);
  EXPECT_EQ(spline->order, 3);
  EXPECT_GT(spline->trajectoryId, 0);
  EXPECT_GT(spline->startTimeS, fixture.request.clock.timestampS);
  EXPECT_GE(spline->controls.size(), 4U);
  EXPECT_EQ(spline->knots.size(),
            spline->controls.size() + static_cast<std::size_t>(spline->order) + 1U);
  const auto &preview = plan.previewPath();
  ASSERT_FALSE(preview.empty());
  EXPECT_EQ(preview.data(), plan.previewPath().data());
  EXPECT_NEAR(preview.back().z, 0.5, 1e-6);
}

TEST(ScanBackend, KeepsCommittedTrajectoryWhileOfficialFsmReplans) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());

  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto firstId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.request.clock.timestampS += 0.01;
  fixture.refreshCollision(2);
  const LocalPlan retained = backend.tick(fixture.request);

  ASSERT_TRUE(retained.ready());
  EXPECT_GE(std::get<SplineTarget>(retained.target()).trajectoryId, firstId);
}

TEST(ScanBackend, CollisionTickReplacesOrStopsThePublishedSpline) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());

  fixture.bitmap.occupyInflated({0.5F, 0.0F, 0.5F}, 0.4, 0.2, 0.2);
  fixture.request.clock.timestampS += 0.05;
  fixture.refreshCollision(2);
  const LocalPlan retained = backend.tick(fixture.request);
  ASSERT_TRUE(retained.ready());
  const auto retainedId =
      std::get<SplineTarget>(retained.target()).trajectoryId;
  const LocalPlan updated = backend.checkCollision(fixture.request);

  if (!updated.ready()) {
    EXPECT_TRUE(updated.status() == nav_kernel::LocalPlanStatus::NearFieldStop ||
                updated.status() == nav_kernel::LocalPlanStatus::Blocked);
    return;
  }
  ASSERT_TRUE(updated.ready());
  const auto updatedId =
      std::get<SplineTarget>(updated.target()).trajectoryId;
  EXPECT_GE(updatedId, retainedId);
  if (updatedId > retainedId) {
    nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    for (const Vec3 &point : updated.previewPath())
      EXPECT_TRUE(grid.obstacleFree(point, 0.0));
  }
}

TEST(ScanBackend, KeepsPublishedSplineUntilNewReferenceIsPlanned) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto previousId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.route = {{0.0, 0.0, 0.5}, {-2.0, 0.0, 0.5}};
  fixture.request.objective = RouteTarget{{
      fixture.route.data(), static_cast<int>(fixture.route.size()), 2, false}};
  fixture.request.clock.timestampS += 0.01;
  const LocalPlan changed = backend.tick(fixture.request);

  ASSERT_TRUE(changed.ready());
  EXPECT_EQ(std::get<SplineTarget>(changed.target()).trajectoryId, previousId);

  LocalPlan replacement = changed;
  for (int tick = 0; tick < 8 &&
                     std::get<SplineTarget>(replacement.target()).trajectoryId == previousId;
       ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
  }
  ASSERT_TRUE(replacement.ready());
  EXPECT_GT(std::get<SplineTarget>(replacement.target()).trajectoryId, previousId);
}

TEST(ScanBackend, AcceptsChangedReferenceShapeWithANewGeneration) {
  RequestFixture fixture(
      {{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto previousId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.route[1] = {1.0, 0.8, 0.5};
  fixture.request.objective = RouteTarget{{
      fixture.route.data(), static_cast<int>(fixture.route.size()), 2, false}};
  fixture.request.clock.timestampS += 0.01;
  LocalPlan replacement = backend.tick(fixture.request);
  for (int tick = 0; tick < 8 &&
                     std::get<SplineTarget>(replacement.target()).trajectoryId == previousId;
       ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
  }

  ASSERT_TRUE(replacement.ready());
  EXPECT_GT(std::get<SplineTarget>(replacement.target()).trajectoryId, previousId);
}

TEST(ScanReplanFsm, HeldMotionIntentRecoversAfterFailureLimitWithoutAnotherReference) {
  using namespace nav_kernel::local::scan::upstream;
  for (const bool motion_intent : {false, true}) {
    SCOPED_TRACE(motion_intent ? "held motion intent" : "route");
    ScanAttemptFixture fixture(motion_intent);
    for (double x = -0.25; x < 0.3; x += 0.1)
      for (double y = -0.25; y < 0.3; y += 0.1)
        for (double z = 0.25; z < 0.8; z += 0.1)
          fixture.request.bitmap.occupy({x, y, z});

    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = 0.1;
    params.replanThreshold = 1.0;
    params.maxReplanFailCount = 1;
    SCANReplanFSM fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1.0;
    input.executionFrozen = true;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0.0, 0.0, 0.5};
    input.referencePath = std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    const auto tick = [&] {
      input.nowS += SCANReplanFSM::kTickPeriodS;
      return fsm.tick(input);
    };
    FsmOutput output;
    for (int i = 0; i < 5 && fsm.state() != ScanReplanState::EMERGENCY_STOP; ++i)
      output = tick();
    ASSERT_EQ(fsm.state(), ScanReplanState::EMERGENCY_STOP);
    ASSERT_EQ(fixture.manager.reboundDebug().reason, "collision_at_trajectory_start");
    EXPECT_FALSE(output.trajectory.has_value());
    ASSERT_TRUE(tick().emergencyStopIssued);

    fixture.request.bitmap.clear();
    fixture.request.refreshCollision(2);
    fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
    // A cleared map cannot authorize replanning until observed motion stops.
    input.odometry->velocity = {0.2, 0.0, 0.0};
    EXPECT_EQ(tick().state, ScanReplanState::EMERGENCY_STOP);
    EXPECT_TRUE(fsm.hasTarget());
    input.odometry->velocity.setZero();
    output = tick();
    if (!motion_intent) {
      EXPECT_EQ(output.state, ScanReplanState::WAIT_TARGET);
      EXPECT_FALSE(fsm.hasTarget());
      continue;
    }

    ASSERT_EQ(output.state, ScanReplanState::GEN_NEW_TRAJ);
    ASSERT_TRUE(fsm.hasTarget());
    output = tick();
    ASSERT_EQ(output.state, ScanReplanState::EXEC_TRAJ)
        << fixture.manager.reboundDebug().reason;
    ASSERT_TRUE(output.trajectory.has_value());
    EXPECT_TRUE(fixture.manager.reboundDebug().success);
    EXPECT_LE((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(0.0) -
               input.odometry->position).norm(), 1e-9);
  }
}

TEST(ScanReplanFsm, ReferenceCallbackDoesNotAdvanceFrozenTrajectoryTime) {
  using nav_kernel::local::scan::upstream::FsmInput;
  using nav_kernel::local::scan::upstream::FsmOdometry;
  using nav_kernel::local::scan::upstream::SCANPlannerManager;
  using nav_kernel::local::scan::upstream::SCANReplanFSM;
  using nav_kernel::local::scan::upstream::ScanNavigationMode;
  using nav_kernel::local::scan::upstream::ScanReplanParams;

  SCANPlannerManager manager;
  manager.pp_.max_vel_ = 0.75;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.bodyHeight = 0.4;
  SCANReplanFSM fsm(manager, params);

  FsmInput timer;
  timer.nowS = 9.95;
  timer.executionFrozen = true;
  timer.odometry = FsmOdometry{};
  (void)fsm.tick(timer);

  manager.local_data_.start_time_ = 5.0;
  FsmInput pathCallback;
  pathCallback.nowS = 10.0;
  pathCallback.executionFrozen = true;
  pathCallback.odometry = FsmOdometry{};
  pathCallback.referencePath = std::vector<Eigen::Vector3d>{
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto output = fsm.tick(pathCallback);

  ASSERT_TRUE(output.targetAccepted);
  EXPECT_DOUBLE_EQ(manager.local_data_.start_time_, 5.0);
}

TEST(ScanReplanFsm, ExplicitFreezePreservesProgressAcrossLongTimerGaps) {
  using namespace nav_kernel::local::scan::upstream;
  for (const double gap : {0.1, 0.3, 0.5}) {
    SCOPED_TRACE(gap);
    SCANPlannerManager manager;
    SCANReplanFSM fsm(manager, ScanReplanParams{});
    FsmInput input;
    input.nowS = 10.0;
    input.odometry = FsmOdometry{};
    input.executionFrozen = true;
    (void)fsm.tick(input);
    manager.local_data_.start_time_ = 5.0;
    manager.local_data_.traj_id_ = 1;
    input.nowS += gap;
    (void)fsm.tick(input);
    EXPECT_NEAR(input.nowS - manager.local_data_.start_time_, 5.0, 1e-12);
    manager.local_data_.start_time_ = 0.0;
    input.nowS += gap;
    (void)fsm.tick(input);
    EXPECT_NEAR(manager.local_data_.start_time_, gap, 1e-12);
  }
}

TEST(ScanBackend, SpeedReductionInvalidatesOldTrajectoryWithoutRestartingIds) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  EXPECT_DOUBLE_EQ(original.maxLinearSpeedMps, scanParams().scan.maxVelocity);

  fixture.request.maxLinearSpeedMps = 0.2;
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);

  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget slower = std::get<SplineTarget>(plan.target());
  EXPECT_GT(slower.trajectoryId, original.trajectoryId);
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, 0.2);
  EXPECT_GT(nav_kernel::SplineView(slower).duration(),
            nav_kernel::SplineView(original).duration());
}

TEST(ScanBackend, ContinuousSpeedUpdatesDoNotStarveFsmTimers) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  std::int64_t lastId = 0;
  int publications = 0;
  for (int tick = 0; tick < 16; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.request.maxLinearSpeedMps = 0.2 + tick * 0.01;
    const LocalPlan plan = backend.tick(fixture.request);
    if (!plan.ready()) continue;
    const auto &spline = std::get<SplineTarget>(plan.target());
    if (spline.trajectoryId == lastId) continue;
    EXPECT_GT(spline.trajectoryId, lastId);
    EXPECT_DOUBLE_EQ(spline.maxLinearSpeedMps, fixture.request.maxLinearSpeedMps);
    lastId = spline.trajectoryId;
    ++publications;
  }
  EXPECT_GE(publications, 3);
}

TEST(ScanBackend, SpeedIncreaseRetainsCurrentTrajectoryUntilReplacement) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.2;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto oldId = std::get<SplineTarget>(plan.target()).trajectoryId;
  fixture.request.maxLinearSpeedMps = 0.5;
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.2);
  for (int tick = 0; tick < 20 &&
                     std::get<SplineTarget>(plan.target()).trajectoryId == oldId; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    ASSERT_TRUE(plan.ready());
  }
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.5);
}

TEST(ScanBackend, SpeedReductionStartsFromStoppedOdometryInsteadOfOldSplineVelocity) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView originalView(original);
  const double movingTime = 0.4 * originalView.duration();
  const Vec3 oldVelocity = originalView.velocity(movingTime);
  ASSERT_GT(std::hypot(oldVelocity.x, oldVelocity.y), 0.2);
  fixture.request.robot.pose.position = originalView.position(movingTime);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {};
  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.executionFrozen = true;
  fixture.request.clock.timestampS = original.startTimeS + movingTime;

  // The collision timer may receive the lower limit before the FSM timer.
  plan = backend.checkCollision(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto &slower = std::get<SplineTarget>(plan.target());
  ASSERT_GT(slower.trajectoryId, original.trajectoryId);
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, 0.1);
  const nav_kernel::SplineView slowerView(slower);
  const Vec3 start = slowerView.position(0.0);
  const Vec3 velocity = slowerView.velocity(0.0);
  EXPECT_NEAR(start.x, fixture.request.robot.pose.position.x, 0.02);
  EXPECT_NEAR(start.y, fixture.request.robot.pose.position.y, 0.02);
  EXPECT_NEAR(velocity.x, 0.0, 0.01);
  EXPECT_NEAR(velocity.y, 0.0, 0.01);
}

TEST(ScanBackend, ContinuousSpeedReductionsDoNotRestartPendingGeneration) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  std::int64_t lastId = std::get<SplineTarget>(plan.target()).trajectoryId;
  int publications = 0;
  for (int tick = 0; tick < 16; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.request.maxLinearSpeedMps = 0.6 - tick * 0.01;
    plan = backend.tick(fixture.request);
    if (!plan.ready()) continue;
    const auto &spline = std::get<SplineTarget>(plan.target());
    if (spline.trajectoryId == lastId) continue;
    EXPECT_GT(spline.trajectoryId, lastId);
    EXPECT_DOUBLE_EQ(spline.maxLinearSpeedMps, fixture.request.maxLinearSpeedMps);
    lastId = spline.trajectoryId;
    ++publications;
  }
  EXPECT_GE(publications, 3);
}

TEST(ScanBackend, RestoringSpeedCannotReviveInvalidatedTrajectory) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.5;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto oldId = std::get<SplineTarget>(plan.target()).trajectoryId;

  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.tick(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  fixture.request.maxLinearSpeedMps = 0.5;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.checkCollision(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready()) break;
    EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.5);
}

TEST(ScanBackend, BlockedReferenceDuringSpeedReductionCannotReviveOldSpline) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.tick(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  fixture.request.maxLinearSpeedMps = 0.5;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    EXPECT_FALSE(plan.ready());
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
}

TEST(ScanBackend, PreservesRequestedSpeedBelowConfigurationFloor) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.03;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.03);
}

TEST(ScanBackend, FinishedShortHorizonContinuesWithoutAnotherGoal) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  auto params = scanParams();
  params.scan.planningHorizon = 0.5;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto endpoint = firstView.position(firstView.duration());
  ASSERT_LT(endpoint.x, 1.0);
  fixture.request.robot.pose.position = endpoint;
  fixture.request.clock.timestampS = first.startTimeS + firstView.duration() + 0.03;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(plan.ready());
  const auto next = std::get<SplineTarget>(plan.target());
  EXPECT_GT(next.trajectoryId, first.trajectoryId);
  const nav_kernel::SplineView nextView(next);
  EXPECT_GT(nextView.position(nextView.duration()).x, endpoint.x + 0.1);
}

TEST(ScanBackend, HeldMotionIntentReplansBeforeShortSegmentStops) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  // Keep planning completion at the input time to compare exact boundaries.
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 0.5, 3.0, 90.0}, {fixture.route.data(), 2, 1, false}};
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.planningHorizon = 0.5;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto start = firstView.position(0.0);
  const auto endpoint = firstView.position(firstView.duration());
  const double segmentLength = nav_kernel::distance3D(start, endpoint);
  ASSERT_LT(segmentLength, params.scan.replanDistance);

  double continuationTime = 0.0;
  const double continuationDistance =
      std::min(params.scan.replanDistance,
               std::max(params.scan.noReplanDistance, 0.5 * segmentLength));
  for (int sample = 1; sample < 1000; ++sample) {
    const double time = firstView.duration() * sample / 1000.0;
    if (nav_kernel::distance3D(start, firstView.position(time)) >
        continuationDistance + 1e-3) {
      continuationTime = time;
      break;
    }
  }
  ASSERT_GT(continuationTime, 0.0);
  ASSERT_LT(continuationTime, firstView.duration() - 0.02);
  fixture.request.robot.pose.position = firstView.position(continuationTime);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = firstView.velocity(continuationTime);
  fixture.request.clock.timestampS = first.startTimeS + continuationTime;

  LocalPlan replacement;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
    if (replacement.ready() &&
        std::get<SplineTarget>(replacement.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(replacement.ready()) << backend.debugSnapshot().searchReason;
  const auto continued = std::get<SplineTarget>(replacement.target());
  ASSERT_GT(continued.trajectoryId, first.trajectoryId);
  ASSERT_LT(continued.startTimeS, first.startTimeS + firstView.duration());

  const double oldTime = continued.startTimeS - first.startTimeS;
  const nav_kernel::SplineView continuedView(continued);
  EXPECT_LT(nav_kernel::distance3D(continuedView.position(0.0),
                                   firstView.position(oldTime)), 1e-8);
  EXPECT_LT(nav_kernel::distance3D(continuedView.velocity(0.0),
                                   firstView.velocity(oldTime)), 1e-8);
  const auto initialVelocity = continuedView.velocity(0.0);
  EXPECT_GT(std::hypot(initialVelocity.x, initialVelocity.y), 0.05);
}

TEST(ScanBackend, HeldMotionIntentRetainsShortSideTargetUntilMeasuredArrival) {
  for (const bool obstructTail : {false, true}) {
    SCOPED_TRACE(obstructTail);
    RequestFixture fixture({{0.025, 1.025, 0.525}, {3.525, 1.025, 0.525}}, 0.05);
    const auto setPocket = [&](bool oppositeOpen) {
      fixture.bitmap.clear();
      for (int x = -99; x < 99; ++x) {
        fixture.bitmap.occupy({x * 0.05 + 0.025, 0.675, 0.525});
        if (!oppositeOpen)
          fixture.bitmap.occupy({x * 0.05 + 0.025, 1.075, 0.525});
      }
      for (int y = -99; y < 99; ++y)
        fixture.bitmap.occupy({0.075, y * 0.05 + 0.025, 0.525});
      for (int x = 2; x < 75; ++x)
        fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
    };
    setPocket(false);
    fixture.refreshCollision(2);
    fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
    fixture.request.objective = nav_kernel::MotionIntentTarget{
        {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1, false}};
    fixture.request.maxLinearSpeedMps = 0.5;
    auto params = scanParams();
    params.scan.voxelResolution = 0.05;
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int tick = 0; tick < 100 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    const auto first = std::get<SplineTarget>(plan.target());
    const nav_kernel::SplineView firstView(first);
    const auto start = firstView.position(0.0);
    const auto endpoint = firstView.position(firstView.duration());
    ASSERT_NEAR(endpoint.x, start.x, 1e-6);
    ASSERT_NEAR(start.y - endpoint.y, 0.3, 1e-6);

    // The opposite side becomes free while W and its reference stay unchanged.
    setPocket(true);
    fixture.refreshCollision(3);
    double pastHalfTime = 0.0;
    for (int sample = 1; sample < 1000; ++sample) {
      const double time = firstView.duration() * sample / 1000.0;
      if (start.y - firstView.position(time).y > 0.16) {
        pastHalfTime = time;
        break;
      }
    }
    ASSERT_GT(pastHalfTime, 0.0);
    const auto plannedPosition = firstView.position(pastHalfTime);
    fixture.request.robot.pose.position = plannedPosition;
    fixture.request.robot.pose.position.y += 0.13;
    fixture.request.robot.kinematics.valid = true;
    fixture.request.robot.kinematics.linearVelocity = {0.0, -0.1, 0.0};
    ASSERT_LT(nav_kernel::distance3D(plannedPosition, endpoint), 0.2);
    ASSERT_GT(nav_kernel::distance3D(fixture.request.robot.pose.position, endpoint), 0.2);
    fixture.request.clock.timestampS = first.startTimeS + pastHalfTime;
    for (int tick = 0; tick < 3; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
      EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_execute_trajectory");
      EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
      const nav_kernel::SplineView retained(std::get<SplineTarget>(plan.target()));
      EXPECT_LT(nav_kernel::distance3D(retained.position(retained.duration()), endpoint), 1e-6);
    }

    if (obstructTail) {
      // Deferring the progress timer must not suppress collision-timer replans.
      for (int x = -99; x < 99; ++x)
        fixture.bitmap.occupy({x * 0.05 + 0.025, 0.825, 0.525});
      fixture.refreshCollision(4);
      fixture.request.clock.timestampS += 0.01;
      const nav_kernel::local::scan::Grid blockedGrid(params, fixture.request);
      ASSERT_EQ(blockedGrid.segmentInflatedOccupancy(
                    firstView.position(fixture.request.clock.timestampS - first.startTimeS),
                    0.0, endpoint, 0.0), 1);
      const auto priorAttemptId = backend.debugSnapshot().scanAttempt.attemptId;
      plan = backend.checkCollision(fixture.request);
      const auto collisionDebug = backend.debugSnapshot();
      EXPECT_TRUE(collisionDebug.scanAttempt.attemptId > priorAttemptId ||
                  (plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop &&
                   collisionDebug.searchReason == "scan_emergency_stop") ||
                  (plan.status() == nav_kernel::LocalPlanStatus::Blocked &&
                   collisionDebug.searchReason == "scan_local_target_blocked"))
          << collisionDebug.searchReason;
      if (plan.ready()) {
        ASSERT_NE(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
        const nav_kernel::SplineView replacement(std::get<SplineTarget>(plan.target()));
        for (double t = 0.0; t < replacement.duration(); t += 0.01) {
          ASSERT_EQ(blockedGrid.segmentInflatedOccupancy(replacement.position(t), 0.0,
              replacement.position(std::min(t + 0.01, replacement.duration())), 0.0), 0);
        }
      }
      continue;
    }

    // Nominal completion is still 15 cm short in measured motion. Continue the
    // retained endpoint rather than treating the new-candidate 20 cm rule as arrival.
    fixture.request.robot.pose.position = endpoint;
    fixture.request.robot.pose.position.y += 0.15;
    fixture.request.clock.timestampS = first.startTimeS + firstView.duration() + 0.03;
    for (int tick = 0; tick < 30; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
        break;
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    const auto continuation = std::get<SplineTarget>(plan.target());
    ASSERT_GT(continuation.trajectoryId, first.trajectoryId);
    const nav_kernel::SplineView continuedView(continuation);
    EXPECT_LT(nav_kernel::distance3D(continuedView.position(0.0),
                                    fixture.request.robot.pose.position), 1e-6);
    EXPECT_LT(nav_kernel::distance3D(continuedView.velocity(0.0),
                                    fixture.request.robot.kinematics.linearVelocity), 1e-6);
    EXPECT_LT(nav_kernel::distance3D(continuedView.position(continuedView.duration()), endpoint), 1e-6);

    fixture.request.robot.pose.position = endpoint;
    fixture.request.robot.kinematics.linearVelocity = {};
    fixture.request.clock.timestampS = continuation.startTimeS + continuedView.duration() + 0.03;
    for (int tick = 0; tick < 30; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > continuation.trajectoryId)
        break;
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    ASSERT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, continuation.trajectoryId);
    const nav_kernel::SplineView afterArrival(std::get<SplineTarget>(plan.target()));
    EXPECT_GT(afterArrival.position(afterArrival.duration()).y, endpoint.y + 0.2);
  }
}

TEST(ScanBackend, EndTimeAloneDoesNotFinishAnUnreachedReference) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  fixture.request.robot.pose.position.x = 0.2;
  fixture.request.clock.timestampS = first.startTimeS + nav_kernel::SplineView(first).duration() + 0.3;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
}

TEST(ScanBackend, ReachedReferenceWaitsWithoutStartingAnotherTrajectory) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  fixture.request.robot.pose.position = fixture.route.back();
  fixture.request.clock.timestampS = first.startTimeS + nav_kernel::SplineView(first).duration() + 0.3;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_wait_target");
}

TEST(ScanBackend, BlockedReferenceDoesNotReplaceFsmTimerCallbacks) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget initial = std::get<SplineTarget>(plan.target());
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  fixture.request.objective = RouteTarget{{fixture.route.data(), 2, 2, false}};
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, initial.trajectoryId);

  // The subscriber event must not suppress the next timer callback. That
  // callback invalidates the old spline when no usable target remains.
  fixture.request.clock.timestampS =
      initial.startTimeS + nav_kernel::SplineView(initial).duration() + 0.3;
  plan = backend.tick(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.objective = RouteTarget{{fixture.route.data(), 2, 3, false}};
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, initial.trajectoryId);
}

TEST(ScanBackend, BlockedReferenceRecoversOnMapUpdateWithoutAnotherClick) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready());
}

TEST(ScanGrid, BoundaryDepartureLeavesOnlyInitialCellAndChecksBothCylinders) {
  RequestFixture f({{.025, .025, .525}, {.375, .025, .525}}, .05);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  f.bitmap.occupy({.275, .025, .525});
  f.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, f.request);
  const auto pose = f.request.robot.pose;
  EXPECT_FALSE(grid.obstacleFree(pose.position, pose.yaw));
  EXPECT_TRUE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, .026, .525}));
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .725}));

  // A second occupied cell along the exit is forbidden, even if the endpoint is free.
  f.bitmap.occupy({.275, -.025, .525});
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
  f.bitmap.clear();
  f.bitmap.occupy({.275, .025, .525});
  f.bitmap.occupy({-.225, -.125, .525});
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
}

TEST(ScanGrid, BoundaryDepartureRejectsActualMotionIntoAnObstacle) {
  RequestFixture f({{.025, .025, .525}, {.025, -.325, .525}}, .05);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  f.bitmap.occupy({.275, .025, .525});
  f.bitmap.occupy({.325, .025, .525});
  f.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, f.request);
  auto pose = f.request.robot.pose;
  pose.yaw = 0;
  const nav_kernel::Twist command{0, -.15, 0};
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, command, {}, .15, .35, .5));
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, command, {0, -.1, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {.1, 0, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {0, -.2, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.15, .1}, {}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {0, 0, .1}, .15, .35, .5));
  // A larger configured command still needs a complete exit and braking room.
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {0, -.25, 0}, .3, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, .35, .1));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, 1.0, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {.1, 0, 0}, .3, .35, .5));
  f.bitmap.occupy({.275, -.025, .525});
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, .35, .5));
}
