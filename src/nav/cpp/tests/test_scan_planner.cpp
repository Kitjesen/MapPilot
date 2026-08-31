#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "planning/local/planner.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/optimizer.hpp"
#include "planning/local/scan/search.hpp"
#include "planning/local/scan/seed.hpp"
#include "planning/local/scan/spline.hpp"
#include "trajectory/spline.hpp"

namespace {

nav_kernel::LocalPlannerParams scanParams() {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.vehicleLength = 0.60;
  params.vehicleWidth = 0.45;
  params.footprintPadding = 0.05;
  params.scan.cylinderOffset = 0.25 * params.vehicleLength;
  params.scan.cylinderRadius =
      std::hypot(0.25 * params.vehicleLength, 0.5 * params.vehicleWidth);
  params.autonomySpeed = 0.40;
  params.maxSpeed = 1.0;
  params.scan.voxelResolution = 0.10;
  params.scan.horizontalRange = 4.0;
  params.scan.routeZTolerance = 0.30;
  params.scan.maxSearchNodes = 20000;
  return params;
}

nav_kernel::LocalPlanRequest inputFor(const std::vector<nav_kernel::Vec3> &route, double timestamp,
                                    std::uint64_t generation = 1) {
  nav_kernel::LocalPlanRequest input;
  input.robot.pose = {route.front(), 0.0};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), generation, false}};
  input.clock.timestampS = timestamp;
  input.identity = {1, generation, 0};
  input.environment.collision.resolution = 0.10;
  input.environment.collision.aabbMin = {-5.0, -5.0, -5.0};
  input.environment.collision.aabbMax = {5.0, 5.0, 5.0};
  input.environment.collision.resetEpoch = 1;
  input.environment.collision.observationSequence = generation;
  input.environment.collision.generation = generation;
  input.environment.collision.stampS = timestamp;
  input.environment.collision.receiveStampS = timestamp;
  input.environment.collision.complete = true;
  input.environment.collision.live = true;
  return input;
}

std::vector<float> collisionCenters(const std::vector<float> &xyzh) {
  std::vector<float> xyz;
  xyz.reserve((xyzh.size() / 4U) * 3U);
  for (std::size_t offset = 0; offset + 3U < xyzh.size(); offset += 4U) {
    xyz.insert(xyz.end(), {xyzh[offset], xyzh[offset + 1U], xyzh[offset + 2U]});
  }
  return xyz;
}

const nav_kernel::SplineTarget &splineTarget(const nav_kernel::LocalPlan &plan) {
  return std::get<nav_kernel::SplineTarget>(plan.target());
}

std::vector<nav_kernel::TrajectoryPoint> trajectory(const nav_kernel::LocalPlan &plan) {
  const auto &target = splineTarget(plan);
  const nav_kernel::SplineView spline(target);
  if (!spline.valid()) return {};

  const double step = std::max(0.01, std::min(0.05, target.intervalS * 0.25));
  const int segments = std::max(1, static_cast<int>(std::ceil(spline.duration() / step)));
  std::vector<nav_kernel::TrajectoryPoint> samples;
  samples.reserve(static_cast<std::size_t>(segments) + 1U);
  for (int index = 0; index <= segments; ++index) {
    const double time = spline.duration() * static_cast<double>(index) /
                        static_cast<double>(segments);
    samples.push_back({spline.position(time), spline.velocity(time), {}, 0.0, 0.0, time});
  }
  return samples;
}

void setCollision(nav_kernel::LocalPlanRequest &input, const std::vector<float> &xyz) {
  input.environment.collision.occupiedXyz = xyz.data();
  input.environment.collision.occupiedCount = static_cast<int>(xyz.size() / 3U);
}

}  // namespace

TEST(ScanDefaults, UsesOfficialFiveCentimeterResolutionAndFiveMeterSearchPool) {
  const nav_kernel::ScanPlannerParams params;

  EXPECT_DOUBLE_EQ(params.voxelResolution, 0.05);
  EXPECT_DOUBLE_EQ(params.horizontalRange, 2.5);
  EXPECT_DOUBLE_EQ(params.cylinderRadius, 0.40);
  EXPECT_DOUBLE_EQ(params.cylinderOffset, 0.25);
}

TEST(ScanGrid, CentersFiveMeterSearchPoolBetweenRobotAndLocalTarget) {
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  params.scan.horizontalRange = 2.5;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {1.75, 0.0, 0.0},
      {3.5, 0.0, 0.0},
  };

  const nav_kernel::local::scan::Grid grid(params, inputFor(route, 1.0));

  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_TRUE(grid.contains(grid.index(route.front())));
  EXPECT_TRUE(grid.contains(grid.index(route.back())));
  EXPECT_TRUE(grid.obstacleFree(route.front(), 0.0));
  EXPECT_TRUE(grid.obstacleFree(route.back(), 0.0));
}

TEST(ScanCollisionCoverage, RotatedBoxRejectsFootprintOverhang) {
  nav_kernel::LocalCollisionMapView collision;
  collision.aabbMin = {-3.0, -3.0, -1.0};
  collision.aabbMax = {3.0, 3.0, 1.0};
  collision.boxCenter = {0.0, 0.0, 0.0};
  collision.boxHalf = {2.0, 1.0, 1.0};
  collision.boxYaw = 0.25 * M_PI;
  collision.hasBox = true;

  const nav_kernel::Vec3 inside_axis{
      std::cos(collision.boxYaw) * 1.5,
      std::sin(collision.boxYaw) * 1.5,
      0.0,
  };
  EXPECT_TRUE(collision.coversCylinder(inside_axis, 0.20, 0.25, 0.35));
  EXPECT_FALSE(collision.coversCylinder({1.8, 1.8, 0.0}, 0.20, 0.25, 0.35));

  const nav_kernel::Vec3 near_side{
      -std::sin(collision.boxYaw) * 0.85,
      std::cos(collision.boxYaw) * 0.85,
      0.0,
  };
  EXPECT_TRUE(collision.covers(near_side));
  EXPECT_FALSE(collision.coversCylinder(near_side, 0.20, 0.25, 0.35));
}

TEST(ScanCollisionCoverage, ArbitraryYawVoxelKeepsItsRotatedExtentOccupied) {
  auto params = scanParams();
  params.useTraversabilityCost = false;
  params.footprintPadding = 0.0;
  params.scan.cylinderOffset = 0.0;
  params.scan.cylinderRadius = 0.05;
  const std::vector<nav_kernel::Vec3> route{
      {-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}};
  std::vector<float> occupied{0.0F, 0.0F, 0.5F};
  auto input = inputFor(route, 1.0);
  input.environment.collision.occupiedXyz = occupied.data();
  input.environment.collision.occupiedCount = 1;
  input.environment.collision.resolution = 0.2;
  input.environment.collision.boxCenter = {0.0, 0.0, 0.5};
  input.environment.collision.boxHalf = {4.0, 4.0, 1.5};
  input.environment.collision.boxYaw = 0.25 * M_PI;
  input.environment.collision.hasBox = true;
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_FALSE(grid.obstacleFree(nav_kernel::Vec3{0.13, 0.0, 0.5}, 0.0))
      << "a 45-degree source voxel reaches beyond its unrotated half extent";
  EXPECT_TRUE(grid.obstacleFree(nav_kernel::Vec3{0.35, 0.0, 0.5}, 0.0));
}

TEST(ScanGrid, SeparatesObstacleInflationFromBodyCoverageHeight) {
  auto params = scanParams();
  params.footprintPadding = 0.0;
  params.scan.cylinderOffset = 0.0;
  params.scan.cylinderRadius = 0.05;
  params.scan.inflationZUp = 0.10;
  params.scan.inflationZDown = 0.10;
  params.scan.bodyClearanceBelow = 0.25;
  params.scan.bodyClearanceAbove = 0.35;
  const std::vector<nav_kernel::Vec3> route{
      {-1.0, 0.0, 0.50}, {1.0, 0.0, 0.50}};
  std::vector<float> occupied{0.0F, 0.0F, 0.80F};
  auto input = inputFor(route, 1.0);
  setCollision(input, occupied);

  const nav_kernel::local::scan::Grid grid(params, input);

  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_TRUE(grid.obstacleFree(nav_kernel::Vec3{0.0, 0.0, 0.50}, 0.0))
      << "body coverage height proves the snapshot ROI, but must not inflate every obstacle";
  EXPECT_FALSE(grid.obstacleFree(nav_kernel::Vec3{0.0, 0.0, 0.70}, 0.0))
      << "the configured vertical obstacle inflation still applies";
}

TEST(ScanCollisionCoverage, ObservedRouteDoesNotRequireTheWholeSearchWindow) {
  auto params = scanParams();
  params.useTraversabilityCost = false;
  params.scan.horizontalRange = 4.0;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.environment.collision.aabbMin = {-1.0, -1.0, -1.0};
  input.environment.collision.aabbMax = {2.0, 1.0, 1.0};
  nav_kernel::local::scan::Backend planner(params);

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_TRUE(result.ready());
}

TEST(ScanLocalPlanner, IgnoresVerticalBodyHeaveForGroundFollowingSeed) {
  auto params = scanParams();
  params.vehicleLength = 1.0;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.0;
  params.autonomySpeed = 0.50;
  params.maxSpeed = 0.50;
  params.scan.horizontalRange = 3.50;
  params.scan.cylinderRadius = 0.40;
  params.scan.cylinderOffset = 0.25;
  params.scan.bodyClearanceBelow = 0.25;
  params.scan.bodyClearanceAbove = 0.35;

  const std::vector<nav_kernel::Vec3> route{
      {6.9493794976, -1.1759416534, 0.3489005532},
      {6.9493794976, -1.2150000000, 0.3489005532},
      {5.8050000000, -1.2150000000, 0.3489005532},
      {5.5350000000, -1.3050000000, 0.4389005532},
      {3.9778757287, -1.3050000000, 0.4389005532},
  };
  auto input = inputFor(route, 1788018443.0);
  input.robot.pose = {route.front(), 0.0027956124};
  std::get<nav_kernel::RouteTarget>(input.objective).route.reachesGoal = false;
  input.robot.kinematics.valid = true;
  input.robot.kinematics.linearVelocity = {-1.3e-5, -1.3e-5, -1.0e-3};
  input.environment.collision.resolution = 0.25;
  input.environment.collision.aabbMin = {-13.0, -21.25, -5.75};
  input.environment.collision.aabbMax = {27.0, 18.75, 6.25};

  nav_kernel::local::scan::Backend planner(params);
  const auto result = planner.plan(input);

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_TRUE(result.ready());
  EXPECT_GE(trajectory(result).size(), 2U);
}

TEST(ScanSeed, PolynomialInitializationPreservesBoundaryStateAndLinearHeight) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> guide{{0.0, 0.0, 0.10}, {2.0, 0.0, 0.50}};
  auto input = inputFor(guide, 1.0);
  std::get<nav_kernel::RouteTarget>(input.objective).route.reachesGoal = true;
  input.robot.kinematics.valid = true;
  input.robot.kinematics.linearVelocity = {0.15, 0.0, 0.02};
  input.robot.kinematics.linearAcceleration = {0.05, 0.0, 0.01};

  const auto seed = nav_kernel::local::scan::buildSeed(
      guide, input, params, nav_kernel::local::scan::SeedMode::Polynomial);

  ASSERT_TRUE(seed.valid());
  EXPECT_EQ(seed.mode, nav_kernel::local::scan::SeedMode::Polynomial);
  EXPECT_NEAR(seed.samples.front().x, guide.front().x, 1e-9);
  EXPECT_NEAR(seed.samples.back().x, guide.back().x, 1e-9);
  EXPECT_NEAR(seed.samples.front().z, guide.front().z, 1e-9);
  EXPECT_NEAR(seed.samples.back().z, guide.back().z, 1e-9);
  EXPECT_NEAR(seed.boundary[0].x, input.robot.kinematics.linearVelocity.x, 1e-9);
  EXPECT_NEAR(seed.boundary[2].x, 0.0, 1e-9);
  for (std::size_t index = 1U; index < seed.samples.size(); ++index) {
    EXPECT_GE(seed.samples[index].z + 1e-9, seed.samples[index - 1U].z);
  }
}

TEST(ScanSeed, InitialBoundaryRejectsVelocityAwayFromTheLocalTarget) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> guide{
      {0.0, 0.0, 0.0}, {0.25, 0.02, 0.0}, {0.25, 1.2, 0.0}};
  auto input = inputFor(guide, 1.0);
  input.robot.kinematics.valid = true;
  input.robot.kinematics.linearVelocity = {-0.10, 0.0, 0.0};
  input.robot.kinematics.linearAcceleration = {-1.0, 0.5, 0.0};

  const auto seed = nav_kernel::local::scan::buildSeed(
      guide, input, params, nav_kernel::local::scan::SeedMode::Guide);

  ASSERT_TRUE(seed.valid());
  EXPECT_NEAR(seed.boundary[0].x, 0.0, 1e-12);
  EXPECT_NEAR(seed.boundary[0].y, 0.0, 1e-12);
  EXPECT_NEAR(seed.boundary[2].x, 0.0, 1e-12);
  EXPECT_NEAR(seed.boundary[2].y, 0.0, 1e-12);
}

TEST(ScanSeed, PreviousInitializationSamplesTheRemainingSplineBeforeJoiningTarget) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> old_guide{{0.0, 0.0, 0.0}, {2.0, 0.3, 0.0}};
  auto old_input = inputFor(old_guide, 1.0);
  old_input.robot.kinematics.valid = true;
  old_input.robot.kinematics.linearVelocity = {0.2, 0.0, 0.0};
  const auto old_seed = nav_kernel::local::scan::buildSeed(
      old_guide, old_input, params, nav_kernel::local::scan::SeedMode::Polynomial);
  ASSERT_TRUE(old_seed.valid());
  const auto old_controls = nav_kernel::UniformSpline::parameterize(
      old_seed.interval, old_seed.samples, old_seed.boundary);
  const nav_kernel::UniformSpline previous(old_controls, 3, old_seed.interval);
  ASSERT_TRUE(previous.valid());

  const std::vector<nav_kernel::Vec3> new_guide{{0.03, 0.0, 0.0}, {2.5, 0.6, 0.0}};
  auto new_input = inputFor(new_guide, 1.10);
  new_input.robot.kinematics = old_input.robot.kinematics;
  const auto seed = nav_kernel::local::scan::buildSeed(
      new_guide, new_input, params, nav_kernel::local::scan::SeedMode::Previous,
      {&previous, 1.0});

  ASSERT_TRUE(seed.valid());
  EXPECT_EQ(seed.mode, nav_kernel::local::scan::SeedMode::Previous);
  EXPECT_NEAR(seed.samples.front().x, previous.evaluate(0.10).x, 0.08);
  EXPECT_NEAR(seed.samples.back().x, new_guide.back().x, 1e-9);
  EXPECT_NEAR(seed.boundary[0].x, previous.derivative().evaluate(0.10).x, 1e-6);
}

TEST(ScanSeed, RandomRetryUsesMinimumSnapAndStillSuppressesVerticalDeviation) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> guide{{0.0, 0.0, 0.10}, {2.5, 0.0, 0.60}};
  auto input = inputFor(guide, 1.0);

  const auto seed = nav_kernel::local::scan::buildSeed(
      guide, input, params, nav_kernel::local::scan::SeedMode::RandomPolynomial, {}, 3);

  ASSERT_TRUE(seed.valid());
  EXPECT_EQ(seed.mode, nav_kernel::local::scan::SeedMode::RandomPolynomial);
  double lateral = 0.0;
  for (const auto &sample : seed.samples) {
    lateral = std::max(lateral, std::abs(sample.y));
  }
  EXPECT_GT(lateral, 1e-3);
  EXPECT_NEAR(seed.samples.front().z, guide.front().z, 1e-9);
  EXPECT_NEAR(seed.samples.back().z, guide.back().z, 1e-9);
}

TEST(ScanUniformSpline, PreservesOfficialBoundaryDerivativeParameterization) {
  constexpr double interval = 0.20;
  std::vector<nav_kernel::Vec3> samples;
  for (int index = 0; index < 7; ++index) {
    samples.push_back({interval * static_cast<double>(index), 0.0, 0.0});
  }
  const std::array<nav_kernel::Vec3, 4> derivatives{
      nav_kernel::Vec3{1.0, 0.0, 0.0},
      nav_kernel::Vec3{1.0, 0.0, 0.0},
      nav_kernel::Vec3{},
      nav_kernel::Vec3{},
  };

  const auto controls =
      nav_kernel::UniformSpline::parameterize(interval, samples, derivatives);
  const nav_kernel::UniformSpline spline(controls, 3, interval);

  ASSERT_TRUE(spline.valid());
  EXPECT_EQ(controls.size(), samples.size() + 2U);
  EXPECT_NEAR(spline.duration(), interval * (samples.size() - 1U), 1e-10);
  EXPECT_NEAR(spline.evaluate(0.0).x, samples.front().x, 1e-8);
  EXPECT_NEAR(spline.evaluate(spline.duration()).x, samples.back().x, 1e-8);
  const auto velocity = spline.derivative();
  EXPECT_NEAR(velocity.evaluate(0.0).x, 1.0, 1e-8);
  EXPECT_NEAR(velocity.evaluate(spline.duration()).x, 1.0, 1e-8);
}

TEST(ScanUniformSpline, ComputesOfficialComponentWiseFeasibilityScale) {
  const nav_kernel::UniformSpline spline({{0.0, 0.0, 0.0},
                                                       {1.0, 0.0, 0.0},
                                                       {2.0, 0.0, 0.0},
                                                       {3.0, 0.0, 0.0},
                                                       {4.0, 0.0, 0.0},
                                                       {5.0, 0.0, 0.0}},
                                                      3, 0.10);

  ASSERT_TRUE(spline.valid());
  EXPECT_GT(spline.feasibilityRatio(1.0, 1.0), 9.0);
}

TEST(ScanUniformSpline, TrackingViewMatchesOwningSplineAndDerivative) {
  const std::vector<nav_kernel::Vec3> controls{
      {0.0, 0.0, 0.0},
      {0.2, 0.1, 0.0},
      {0.6, 0.3, 0.1},
      {1.1, 0.4, 0.2},
      {1.5, 0.2, 0.2},
      {1.8, 0.0, 0.1},
  };
  const nav_kernel::UniformSpline spline(controls, 3, 0.2);
  ASSERT_TRUE(spline.valid());
  const nav_kernel::UniformSpline velocity = spline.derivative();
  ASSERT_TRUE(velocity.valid());
  const nav_kernel::SplineTarget target{controls, 3, 0.2, 0.0};
  const nav_kernel::SplineView view(target);
  ASSERT_TRUE(view.valid());
  EXPECT_DOUBLE_EQ(view.duration(), spline.duration());

  for (double time : {0.0, 0.05, 0.20, 0.45, spline.duration()}) {
    const auto expected_position = spline.evaluate(time);
    const auto expected_velocity = velocity.evaluate(time);
    const auto actual_position = view.position(time);
    const auto actual_velocity = view.velocity(time);
    EXPECT_NEAR(actual_position.x, expected_position.x, 1e-12);
    EXPECT_NEAR(actual_position.y, expected_position.y, 1e-12);
    EXPECT_NEAR(actual_position.z, expected_position.z, 1e-12);
    EXPECT_NEAR(actual_velocity.x, expected_velocity.x, 1e-12);
    EXPECT_NEAR(actual_velocity.y, expected_velocity.y, 1e-12);
    EXPECT_NEAR(actual_velocity.z, expected_velocity.z, 1e-12);
  }
}

TEST(ScanSpline, NearStraightGuideUsesOfficialPolynomialInsteadOfLoopingWaypointSeed) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> guide{
      {0.0, 0.0, 0.0}, {0.08, 0.18, 0.0}, {0.08, 0.60, 0.0}, {0.08, 1.20, 0.0}};
  auto input = inputFor(guide, 1.0);
  input.robot.kinematics.valid = true;
  input.robot.kinematics.linearVelocity = {0.02, 0.04, 0.0};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  const auto spline =
      nav_kernel::local::scan::buildSpline(grid, guide, input, params);

  ASSERT_TRUE(spline.valid()) << spline.reason;
  EXPECT_EQ(spline.seedMode, nav_kernel::local::scan::SeedMode::Polynomial);
  ASSERT_GE(spline.trajectory.size(), 2U);
  const auto lookahead = std::find_if(
      spline.trajectory.begin(), spline.trajectory.end(),
      [](const auto &sample) { return sample.timeFromStartS >= 0.35; });
  ASSERT_NE(lookahead, spline.trajectory.end());
  EXPECT_GT(lookahead->position.y - spline.trajectory.front().position.y, 0.0);
}

TEST(ScanSpline, MaterialRouteShapeStillUsesGuideSeed) {
  auto params = scanParams();
  const std::vector<nav_kernel::Vec3> guide{
      {0.0, 0.0, 0.0}, {0.8, 0.7, 0.0}, {1.6, 0.7, 0.0}, {2.4, 0.0, 0.0}};
  auto input = inputFor(guide, 1.0);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  const auto spline =
      nav_kernel::local::scan::buildSpline(grid, guide, input, params);

  ASSERT_TRUE(spline.valid()) << spline.reason;
  EXPECT_EQ(spline.seedMode, nav_kernel::local::scan::SeedMode::Guide);
}

TEST(ScanLocalPlanner, HonorsCancellationBeforePlanning) {
  auto params = scanParams();
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto input = inputFor(route, 1.0);

  const auto result = planner.plan(input, [] { return true; });

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::Cancelled);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "planning_cancelled");
}

TEST(ScanLocalPlanner, PreservesCancellationReasonDuringSplineConstruction) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto input = inputFor(route, 1.0);
  int checks = 0;

  const auto result = planner.plan(input, [&checks] { return ++checks >= 5; });

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::Cancelled);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "planning_cancelled");
}

TEST(ScanLocalPlanner, RejectsMissingRouteExplicitly) {
  nav_kernel::local::scan::Backend planner(scanParams());

  const auto result = planner.plan({});

  EXPECT_EQ(planner.debugSnapshot().backend, nav_kernel::LocalPlannerBackend::Scan);
  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_FALSE(result.ready());
  EXPECT_EQ(planner.debugSnapshot().searchReason, "route_invalid");
}

TEST(ScanLocalPlanner, PlansAssistedIntentWithoutCmuFallback) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  const auto guide = std::get<nav_kernel::RouteTarget>(input.objective).route;
  input.objective = nav_kernel::MotionIntentTarget{{0.0, 0.5, 2.0, 45.0}, guide};

  const auto result = planner.plan(input);

  EXPECT_EQ(planner.debugSnapshot().backend, nav_kernel::LocalPlannerBackend::Scan);
  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_intent_ready");
  EXPECT_TRUE(result.ready());
  EXPECT_GE(result.previewPath().size(), 2U);
  EXPECT_GE(trajectory(result).size(), 2U);
  EXPECT_GE(splineTarget(result).controls.size(), 4U);
  EXPECT_EQ(splineTarget(result).degree, 3);
  EXPECT_GT(splineTarget(result).intervalS, 0.0);
  EXPECT_DOUBLE_EQ(splineTarget(result).timeS, 0.0);
  double max_speed = 0.0;
  for (const auto &point : trajectory(result)) {
    max_speed = std::max(max_speed, std::hypot(point.velocity.x, point.velocity.y));
  }
  EXPECT_LE(max_speed, 0.21);
}

TEST(ScanLocalPlanner, RejectsMissingAuthoritativeCollisionMap) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.environment.collision = {};

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_FALSE(result.ready());
  EXPECT_EQ(planner.debugSnapshot().searchReason, "collision_map_missing");
}

TEST(ScanLocalPlanner, PreservesGlobalRouteElevation) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.8, 0.0, 0.18},
      {1.6, 0.0, 0.42},
      {2.4, 0.0, 0.62},
  };

  const auto result = planner.plan(inputFor(route, 1.0));

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready)
      << planner.debugSnapshot().searchReason << " restarts=" << planner.debugSnapshot().reboundRestarts
      << " evaluations=" << planner.debugSnapshot().optimizerEvaluations
      << " segments=" << planner.debugSnapshot().collisionSegments;
  ASSERT_GE(result.previewPath().size(), 3U);
  ASSERT_GE(trajectory(result).size(), 3U);
  const auto result_path = result.previewPath();
  EXPECT_GT(std::max_element(result_path.begin(), result_path.end(),
                             [](const auto &a, const auto &b) { return a.z < b.z; })
                ->z,
            0.45);
  EXPECT_GT(trajectory(result).back().position.z, 0.45);
}

TEST(ScanLocalPlanner, RejectsRouteThatRequiresAnImpossibleSlope) {
  auto params = scanParams();
  params.scan.maxSlope = 0.40;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.4, 0.0, 0.8},
      {0.8, 0.0, 1.6},
  };

  const auto result = planner.plan(inputFor(route, 1.0));

  EXPECT_NE(result.status(), nav_kernel::LocalPlanStatus::Ready);
  EXPECT_FALSE(result.ready());
}

TEST(ScanLocalPlanner, IgnoresMeasuredBodyHeaveOnRouteEntryEdge) {
  auto params = scanParams();
  params.scan.maxSlope = 0.40;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.05, 0.0, 0.10}, {1.0, 0.0, 0.40}};
  auto input = inputFor(route, 1.0);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  EXPECT_NE(searched.reason, "route_slope_exceeded");
}

TEST(ScanLocalPlanner, KeepsRouteShapeWhenTargetsMatch) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> left{
      {0.0, 0.0, 0.0}, {0.8, 0.7, 0.0}, {1.6, 0.7, 0.0}, {2.4, 0.0, 0.0}};
  const std::vector<nav_kernel::Vec3> right{
      {0.0, 0.0, 0.0}, {0.8, -0.7, 0.0}, {1.6, -0.7, 0.0}, {2.4, 0.0, 0.0}};

  const auto left_result = planner.plan(inputFor(left, 1.0, 1));
  const auto right_result = planner.plan(inputFor(right, 2.0, 2));

  ASSERT_EQ(left_result.status(), nav_kernel::LocalPlanStatus::Ready);
  ASSERT_EQ(right_result.status(), nav_kernel::LocalPlanStatus::Ready);
  const auto left_path = left_result.previewPath();
  const auto right_path = right_result.previewPath();
  const auto max_left = std::max_element(left_path.begin(), left_path.end(),
                                         [](const auto &a, const auto &b) { return a.y < b.y; });
  const auto min_right = std::min_element(right_path.begin(), right_path.end(),
                                          [](const auto &a, const auto &b) { return a.y < b.y; });
  ASSERT_NE(max_left, left_path.end());
  ASSERT_NE(min_right, right_path.end());
  EXPECT_GT(max_left->y, 0.35);
  EXPECT_LT(min_right->y, -0.35);
}

TEST(ScanLocalPlanner, ReplansFromPreviousTrajectoryAcrossUpdates) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 7);
  first_input.robot.kinematics.valid = true;
  first_input.robot.kinematics.linearVelocity = {0.2, 0.0, 0.0};
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready);
  EXPECT_DOUBLE_EQ(splineTarget(first).startTimeS, first_input.clock.timestampS);

  auto second_input = inputFor(route, 1.10, 7);
  second_input.clock.executionTimeS = 1.04;
  second_input.robot.pose.position = {0.02, 0.0, 0.0};
  second_input.robot.kinematics = first_input.robot.kinematics;
  const auto second = planner.plan(second_input);

  ASSERT_EQ(second.status(), nav_kernel::LocalPlanStatus::Ready);
  EXPECT_NE(planner.debugSnapshot().searchReason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_DOUBLE_EQ(splineTarget(second).startTimeS, second_input.clock.executionTimeS);
}

TEST(ScanLocalPlanner, ContinuousReplanStartsAtMeasuredVehiclePose) {
  const auto params = scanParams();
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 8);
  first_input.clock.executionTimeS = 1.0;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  const nav_kernel::UniformSpline previous(
      splineTarget(first).controls, splineTarget(first).degree, splineTarget(first).intervalS);
  ASSERT_TRUE(previous.valid());

  auto delayed_input = inputFor(route, 2.0, 8);
  delayed_input.clock.executionTimeS = 2.0;
  delayed_input.robot.pose.position = {0.0, 0.0, 0.0};
  const nav_kernel::local::scan::SeedHistory history{
      &previous, splineTarget(first).startTimeS};
  const auto seed = nav_kernel::local::scan::buildSeed(
      route, delayed_input, params,
      nav_kernel::local::scan::SeedMode::Previous, history, 0);
  ASSERT_TRUE(seed.valid());
  EXPECT_NEAR(seed.samples.front().x, delayed_input.robot.pose.position.x, 1e-9);
  EXPECT_NEAR(seed.samples.front().y, delayed_input.robot.pose.position.y, 1e-9);

  const auto replanned = planner.plan(delayed_input);

  ASSERT_EQ(replanned.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_TRUE(planner.debugSnapshot().continuityReused);
  ASSERT_FALSE(replanned.previewPath().empty());
  EXPECT_LT(std::hypot(replanned.previewPath().front().x, replanned.previewPath().front().y), 0.05)
      << "the ROS-free previous-trajectory seed must start at measured pose, not the old spline's "
         "time-predicted pose";
}

TEST(ScanLocalPlanner, SameMapStillRunsContinuousReplan) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 9);
  first_input.identity.obstacleGeneration = 11;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready);

  auto second_input = inputFor(route, 1.05, 9);
  second_input.identity.obstacleGeneration = 11;
  second_input.robot.pose.position = {0.01, 0.0, 0.0};
  const auto second = planner.plan(second_input);

  ASSERT_EQ(second.status(), nav_kernel::LocalPlanStatus::Ready);
  EXPECT_NE(planner.debugSnapshot().searchReason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_GE(trajectory(second).size(), 2U);
}

TEST(ScanLocalPlanner, FrozenExecutionClockDoesNotAdvanceReusableTrajectory) {
  const auto params = scanParams();
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 19);
  first_input.clock.executionTimeS = 1.0;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_GE(trajectory(first).size(), 2U);
  ASSERT_GE(splineTarget(first).controls.size(), 4U);

  auto frozen_input = inputFor(route, 1.40, 19);
  frozen_input.clock.executionTimeS = 1.0;
  const auto frozen = planner.plan(frozen_input);

  ASSERT_EQ(frozen.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_frozen_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_DOUBLE_EQ(splineTarget(frozen).startTimeS, frozen_input.clock.executionTimeS);
  EXPECT_DOUBLE_EQ(splineTarget(frozen).timeS, splineTarget(first).timeS);
  ASSERT_GE(trajectory(frozen).size(), 2U);
  EXPECT_NEAR(trajectory(frozen)[1].position.x,
              trajectory(first)[1].position.x, 1e-6);
  EXPECT_NEAR(trajectory(frozen)[1].position.y,
              trajectory(first)[1].position.y, 1e-6);
}

TEST(ScanLocalPlanner, FrozenReuseKeepsDesiredTrajectoryWhenVehicleDrifts) {
  const auto params = scanParams();
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 20);
  first_input.clock.executionTimeS = 1.0;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_GE(trajectory(first).size(), 2U);

  auto frozen_input = inputFor(route, 1.40, 20);
  frozen_input.clock.executionTimeS = 1.0;
  frozen_input.robot.pose.position = {0.0, 0.20, 0.0};
  frozen_input.robot.pose.yaw = 1.0;
  const auto frozen = planner.plan(frozen_input);

  ASSERT_EQ(frozen.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_EQ(planner.debugSnapshot().searchReason, "scan_frozen_trajectory_reused");
  ASSERT_GE(trajectory(frozen).size(), 2U);
  EXPECT_GT(std::hypot(trajectory(frozen).front().position.x,
                       trajectory(frozen).front().position.y),
            0.15);
  const double tangent_x = trajectory(frozen)[1].position.x -
                           trajectory(frozen).front().position.x;
  const double tangent_y = trajectory(frozen)[1].position.y -
                           trajectory(frozen).front().position.y;
  EXPECT_NEAR(std::atan2(tangent_y, tangent_x), -frozen_input.robot.pose.yaw, 1e-3);
}

TEST(ScanLocalPlanner, ReplansPreviousTrajectoryAgainstChangedMap) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 10);
  first_input.identity.obstacleGeneration = 11;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready);

  auto changed_input = inputFor(route, 1.10, 10);
  changed_input.identity.obstacleGeneration = 12;
  changed_input.robot.pose.position = {0.02, 0.0, 0.0};
  const auto replanned = planner.plan(changed_input);

  ASSERT_EQ(replanned.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_NE(planner.debugSnapshot().searchReason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_GE(trajectory(replanned).size(), 2U);
}

TEST(ScanLocalPlanner, ReusesSafePrefixOnlyWhenFreshReplanFails) {
  auto params = scanParams();
  params.scan.continuityHorizon = 0.50;
  params.scan.maxSearchNodes = 100;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 13);
  first_input.identity.obstacleGeneration = 21;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready);

  std::vector<float> occupied;
  for (int yi = -40; yi <= 40; ++yi) {
    occupied.push_back(1.2F);
    occupied.push_back(static_cast<float>(yi) * 0.10F);
    occupied.push_back(0.0F);
  }
  auto changed_input = inputFor(route, 1.10, 13);
  changed_input.identity.obstacleGeneration = 22;
  setCollision(changed_input, occupied);

  const auto reused = planner.plan(changed_input);

  ASSERT_EQ(reused.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(reused.ready());
  EXPECT_GE(trajectory(reused).size(), 2U);

  changed_input.clock.timestampS = 20.0;
  changed_input.environment.collision.stampS = changed_input.clock.timestampS;
  changed_input.environment.collision.receiveStampS = changed_input.clock.timestampS;
  const auto expired = planner.plan(changed_input);
  EXPECT_NE(expired.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_NE(planner.debugSnapshot().searchReason, "scan_safe_trajectory_reused");
  EXPECT_FALSE(planner.debugSnapshot().continuityReused);
}

TEST(ScanLocalPlanner, ContinuousReplansReachConfiguredCruiseSpeed) {
  auto params = scanParams();
  params.autonomySpeed = 0.60;
  params.maxSpeed = 0.60;
  params.scan.maxAcceleration = 1.0;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {1.2, 0.0, 0.0}, {2.4, 0.0, 0.0}, {3.6, 0.0, 0.0}};

  double timestamp = 1.0;
  double position = 0.0;
  double speed = 0.0;
  double peak_speed = 0.0;
  for (int replan = 0; replan < 12; ++replan) {
    auto input = inputFor(route, timestamp, 41);
    input.robot.pose.position = {position, 0.0, 0.0};
    input.robot.kinematics.valid = true;
    input.robot.kinematics.linearVelocity = {speed, 0.0, 0.0};
    const auto result = planner.plan(input);
    ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
    const auto result_trajectory = trajectory(result);
    ASSERT_GE(result_trajectory.size(), 2U);
    if (replan > 0)
      EXPECT_TRUE(planner.debugSnapshot().continuityReused);

    const auto sample = std::min_element(
        result_trajectory.begin(), result_trajectory.end(), [](const auto &left, const auto &right) {
          return std::abs(left.timeFromStartS - 0.20) <
                 std::abs(right.timeFromStartS - 0.20);
        });
    ASSERT_NE(sample, result_trajectory.end());
    position += sample->position.x;
    speed = sample->velocity.x;
    peak_speed = std::max(peak_speed, std::hypot(sample->velocity.x, sample->velocity.y));
    timestamp += 0.20;
  }

  EXPECT_GE(peak_speed, 0.50);
}

TEST(ScanLocalPlanner, ResetsToOdometryWhenVehicleLeavesActiveTrajectory) {
  auto params = scanParams();
  params.autonomySpeed = 0.60;
  params.maxSpeed = 0.60;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {1.2, 0.0, 0.0}, {2.4, 0.0, 0.0}, {3.6, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 42);
  first_input.robot.kinematics.valid = true;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;

  auto overshot_input = inputFor(route, 1.20, 42);
  overshot_input.robot.pose.position = {0.80, 0.0, 0.0};
  overshot_input.robot.kinematics.valid = true;
  overshot_input.robot.kinematics.linearVelocity = {0.40, 0.0, 0.0};
  const auto reset = planner.plan(overshot_input);

  ASSERT_EQ(reset.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_GE(trajectory(reset).size(), 2U);
  EXPECT_FALSE(planner.debugSnapshot().continuityReused);
  EXPECT_DOUBLE_EQ(splineTarget(reset).startTimeS, overshot_input.clock.timestampS);
  EXPECT_LT(std::hypot(trajectory(reset).front().position.x,
                       trajectory(reset).front().position.y),
            0.15);
  EXPECT_GT(trajectory(reset).front().velocity.x, 0.0);
}

TEST(ScanLocalPlanner, SearchesAroundThreeDimensionalObstacleBand) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};
  std::vector<float> obstacles;
  for (int yi = -7; yi <= 7; ++yi) {
    for (int zi = -1; zi <= 3; ++zi) {
      obstacles.push_back(1.2F);
      obstacles.push_back(static_cast<float>(yi) * 0.10F);
      obstacles.push_back(static_cast<float>(zi) * 0.10F);
      obstacles.push_back(0.8F);
    }
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready)
      << planner.debugSnapshot().searchReason << " restarts=" << planner.debugSnapshot().reboundRestarts
      << " evaluations=" << planner.debugSnapshot().optimizerEvaluations
      << " segments=" << planner.debugSnapshot().collisionSegments;
  const auto result_path = result.previewPath();
  const auto lateral =
      std::max_element(result_path.begin(), result_path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result_path.end());
  EXPECT_GT(std::abs(lateral->y), 0.75);
}

TEST(ScanLocalPlanner, UsesCompleteMapdCollisionLayerWithoutLegacyCloud) {
  auto params = scanParams();
  params.scan.maxSearchNodes = 100000;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};
  std::vector<float> occupied;
  for (int yi = -7; yi <= 7; ++yi) {
    for (int zi = -3; zi <= 3; ++zi) {
      occupied.push_back(1.2F);
      occupied.push_back(static_cast<float>(yi) * 0.10F);
      occupied.push_back(static_cast<float>(zi) * 0.10F);
    }
  }
  auto input = inputFor(route, 1.0);
  input.environment.collision = {
      occupied.data(),
      static_cast<int>(occupied.size() / 3U),
      0.10,
      {-5.0, -5.0, -2.0},
      {5.0, 5.0, 2.0},
      3U,
      9U,
      12U,
      1'700'000'000.0,
      1.0,
      true,
      true,
  };

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  const auto result_path = result.previewPath();
  const auto lateral =
      std::max_element(result_path.begin(), result_path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result_path.end());
  EXPECT_GT(std::abs(lateral->y), 0.75);
}

TEST(ScanLocalPlanner, IgnoresMapdGroundBelowBodyCollisionEnvelope) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.48},
      {0.8, 0.0, 0.48},
      {1.6, 0.0, 0.48},
      {2.4, 0.0, 0.48},
  };
  std::vector<float> occupied;
  std::vector<float> legacy_obstacles;
  for (int xi = -20; xi <= 20; ++xi) {
    for (int yi = -20; yi <= 20; ++yi) {
      const float x = static_cast<float>(xi) * 0.10F;
      const float y = static_cast<float>(yi) * 0.10F;
      occupied.push_back(x);
      occupied.push_back(y);
      occupied.push_back(0.0F);
      legacy_obstacles.push_back(x);
      legacy_obstacles.push_back(y);
      legacy_obstacles.push_back(0.0F);
      legacy_obstacles.push_back(0.8F);
    }
  }
  auto input = inputFor(route, 1.0);
  input.environment.obstacles = {
      legacy_obstacles.data(),
      static_cast<int>(legacy_obstacles.size() / 4U),
  };
  input.environment.collision = {
      occupied.data(),
      static_cast<int>(occupied.size() / 3U),
      0.10,
      {-5.0, -5.0, -2.0},
      {5.0, 5.0, 2.0},
      3U,
      9U,
      12U,
      1'700'000'000.0,
      1.0,
      true,
      true,
  };

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  ASSERT_TRUE(result.ready());
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_route_ready");
  const auto result_path = result.previewPath();
  const auto lateral =
      std::max_element(result_path.begin(), result_path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result_path.end());
  EXPECT_LT(std::abs(lateral->y), 0.20)
      << "ground below the body collision envelope must not force a detour";
}

TEST(ScanLocalPlanner, RejectsIncompleteMapdCollisionLayer) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.environment.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 1.0, false, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "collision_map_incomplete");
  EXPECT_FALSE(result.ready());
}

TEST(ScanLocalPlanner, RejectsMapdCollisionOlderThanDefaultControlWindow) {
  const auto params = scanParams();
  EXPECT_DOUBLE_EQ(params.scan.collisionMaxAge, 0.50);
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.51);
  input.environment.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 1.0, true, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "collision_map_stale");
  EXPECT_FALSE(result.ready());
}

TEST(ScanLocalPlanner, RejectsCollisionWithoutReceiverClockTimestamp) {
  nav_kernel::local::scan::Backend planner(scanParams());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.environment.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 0.0, true, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "collision_map_invalid");
  EXPECT_FALSE(result.ready());
}

TEST(ScanLocalPlanner, PreservesThreeDimensionalRouteBends) {
  const auto params = scanParams();
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.4, 0.0, 0.8},
      {1.4, 0.0, -0.2},
      {2.4, 0.0, 0.6},
  };
  const auto input = inputFor(route, 1.0);

  const nav_kernel::local::scan::Grid grid(params, input);

  ASSERT_TRUE(grid.valid());
  ASSERT_EQ(grid.route().size(), route.size());
  for (std::size_t i = 0; i < grid.route().size(); ++i) {
    EXPECT_NEAR(grid.route()[i].x, route[i].x, 1e-9);
    EXPECT_NEAR(grid.route()[i].y, route[i].y, 1e-9);
    EXPECT_NEAR(grid.route()[i].z, route[i].z, 1e-9);
  }
}

TEST(ScanLocalPlanner, ProjectedSearchUsesFullThreeDimensionalOccupancy) {
  auto params = scanParams();
  params.scan.verticalMargin = 1.2;
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.3}, {1.6, 0.0, 0.6}};

  const nav_kernel::local::scan::Grid grid(params, inputFor(route, 1.0));

  ASSERT_TRUE(grid.valid()) << grid.reason();
  ASSERT_GT(grid.sizeZ(), 1);
  EXPECT_EQ(grid.cellCount(), grid.sizeX() * grid.sizeY() * grid.sizeZ());
  EXPECT_NE(grid.linear({3, 4, 0}), grid.linear({3, 4, grid.sizeZ() - 1}));
}

TEST(ScanLocalPlanner, ProjectedSearchDoesNotEscapeByClimbingOverWall) {
  auto params = scanParams();
  params.scan.routeZTolerance = 1.0;
  params.scan.verticalMargin = 1.2;
  params.scan.maxSearchNodes = 100000;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};
  std::vector<float> obstacles;
  for (int yi = -40; yi <= 40; ++yi) {
    obstacles.push_back(1.2F);
    obstacles.push_back(static_cast<float>(yi) * 0.10F);
    obstacles.push_back(0.0F);
    obstacles.push_back(0.8F);
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_boundary_fallback_ready");
  ASSERT_TRUE(result.ready());
  const auto result_path = result.previewPath();
  const auto highest = std::max_element(result_path.begin(), result_path.end(),
                                        [](const auto &a, const auto &b) { return a.z < b.z; });
  ASSERT_NE(highest, result_path.end());
  EXPECT_LT(std::abs(highest->z), 0.15) << "projected A* must not escape the wall through Z";
  EXPECT_GT(nav_kernel::distance3D(result.previewPath().back(), route.back()), 0.5);
}

TEST(ScanLocalPlanner, ProjectedSearchInterpolatesZFromSegmentEndpoints) {
  auto params = scanParams();
  params.scan.routeZTolerance = 0.8;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.2}, {1.6, 0.0, 0.4}, {2.4, 0.0, 0.6}};
  std::vector<float> obstacles;
  for (int yi = -7; yi <= 7; ++yi) {
    for (int zi = -10; zi <= 10; ++zi) {
      obstacles.push_back(1.2F);
      obstacles.push_back(static_cast<float>(yi) * 0.10F);
      obstacles.push_back(static_cast<float>(zi) * 0.10F);
      obstacles.push_back(0.8F);
    }
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  ASSERT_TRUE(searched.found()) << searched.reason;
  for (const auto &point : searched.path) {
    EXPECT_NEAR(point.z, 0.25 * point.x, grid.resolution() * 0.51);
  }
}

TEST(ScanLocalPlanner, ProjectedSearchOnlyReplacesCollidingRouteSegment) {
  auto params = scanParams();
  params.scan.maxSearchNodes = 100000;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.5, 0.8, 0.0}, {1.0, 0.8, 0.0}, {1.5, 0.8, 0.0}, {2.0, 0.0, 0.0},
  };
  std::vector<float> obstacles;
  for (int yi = 5; yi <= 11; ++yi) {
    for (int zi = -5; zi <= 5; ++zi) {
      obstacles.push_back(1.25F);
      obstacles.push_back(static_cast<float>(yi) * 0.10F);
      obstacles.push_back(static_cast<float>(zi) * 0.10F);
      obstacles.push_back(0.8F);
    }
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  ASSERT_TRUE(searched.found()) << searched.reason;
  EXPECT_EQ(searched.reason, "segment_search_ready");
  const auto preserved =
      std::find_if(searched.path.begin(), searched.path.end(), [&](const auto &point) {
        return nav_kernel::distance3D(point, route[1]) < 1e-5;
      });
  EXPECT_NE(preserved, searched.path.end())
      << "projected A* must splice only the colliding edge group";
}

TEST(ScanLocalPlanner, TwinCylinderCollisionDependsOnYaw) {
  auto params = scanParams();
  params.vehicleLength = 1.20;
  params.vehicleWidth = 0.30;
  params.footprintPadding = 0.0;
  params.scan.endpointSearchMargin = 0.0;
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  std::vector<float> obstacles{0.45F, 0.0F, 0.0F, 0.8F};
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());
  const auto center = grid.index(input.robot.pose.position);

  EXPECT_FALSE(grid.free(center, 0.0));
  EXPECT_TRUE(grid.free(center, 0.5 * M_PI));
}

TEST(ScanLocalPlanner, TraversabilityChecksTheWholeRobotFootprint) {
  auto params = scanParams();
  params.useTraversabilityCost = true;
  params.traversabilityHardCost = 90.0;
  params.vehicleLength = 1.0;
  params.vehicleWidth = 0.6;
  params.footprintPadding = 0.1;
  params.scan.endpointSearchMargin = 0.0;
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  constexpr int rows = 80;
  constexpr int cols = 80;
  constexpr double resolution = 0.1;
  constexpr double origin = -4.0;
  std::vector<float> traversability(rows * cols, 0.0F);
  const auto cell = [&](double coordinate) {
    return static_cast<int>(std::floor((coordinate - origin) / resolution));
  };
  traversability[cell(0.35) * cols + cell(1.05)] = 95.0F;
  auto input = inputFor(route, 1.0);
  input.environment.traversability = {traversability.data(), rows, cols, resolution, origin, origin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto path_center = grid.index({1.0, 0.0, 0.0});
  ASSERT_FALSE(grid.free(path_center, 0.0))
      << "SCAN must reject a centerline whose rectangular body footprint reaches hard terrain";
}

TEST(ScanLocalPlanner, SearchesThroughObservedCorridorAroundMapdObstacle) {
  auto params = scanParams();
  params.useTraversabilityCost = false;
  params.traversabilityHardCost = 90.0;
  params.vehicleLength = 1.00;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.15;
  params.scan.voxelResolution = 0.15;
  params.scan.endpointSearchMargin = 0.12;
  params.scan.maxSearchNodes = 12000;
  nav_kernel::local::scan::Backend planner(params);

  std::vector<nav_kernel::Vec3> route;
  route.push_back({0.0, 0.0, 0.48});
  for (int index = 0; index < 16; ++index) {
    const double route_z = std::max(0.30, 0.48 - 0.04 * static_cast<double>(index + 1));
    route.push_back({0.1 + 0.2 * static_cast<double>(index), 0.1, route_z});
  }

  std::vector<float> occupied;
  for (int xi = -4; xi <= 16; ++xi) {
    for (int yi = -7; yi <= 7; ++yi) {
      occupied.push_back(static_cast<float>(xi) * 0.25F + 0.125F);
      occupied.push_back(static_cast<float>(yi) * 0.25F - 0.125F);
      occupied.push_back(0.125F);
    }
  }
  std::vector<float> legacy_obstacles;
  for (int xi = -10; xi <= 40; ++xi) {
    for (int side = -1; side <= 1; side += 2) {
      for (int band = 0; band < 2; ++band) {
        legacy_obstacles.push_back(static_cast<float>(xi) * 0.10F);
        legacy_obstacles.push_back(static_cast<float>(side) *
                                   (1.84F + static_cast<float>(band) * 0.10F));
        legacy_obstacles.push_back(0.72F);
        legacy_obstacles.push_back(0.24F);
      }
    }
  }
  for (int yi = -1; yi <= 1; yi += 2) {
    const float y = static_cast<float>(yi) * 0.125F;
    for (float x = 1.125F; x <= 1.375F; x += 0.25F) {
      for (int zi = 0; zi < 2; ++zi) {
        const float z = 0.375F + static_cast<float>(zi) * 0.25F;
        occupied.push_back(x);
        occupied.push_back(y);
        occupied.push_back(z);
      }
    }
  }
  for (int xi = -4; xi <= 16; ++xi) {
    for (int zi = 0; zi < 3; ++zi) {
      const float z = 0.125F + static_cast<float>(zi) * 0.25F;
      for (int side = -1; side <= 1; side += 2) {
        for (float rail_y = 1.875F; rail_y <= 2.125F; rail_y += 0.25F) {
          occupied.push_back(static_cast<float>(xi) * 0.25F + 0.125F);
          occupied.push_back(static_cast<float>(side) * rail_y);
          occupied.push_back(z);
        }
      }
    }
  }

  constexpr int rows = 60;
  constexpr int cols = 60;
  constexpr double resolution = 0.20;
  constexpr double origin = -6.0;
  std::vector<float> traversability(rows * cols, 95.0F);
  const auto cell = [&](double coordinate) {
    return static_cast<int>(std::floor((coordinate - origin) / resolution));
  };
  for (int row = cell(-1.6); row <= cell(1.4); ++row) {
    for (int col = cell(-1.0); col <= cell(3.8); ++col) {
      traversability[row * cols + col] = 0.0F;
    }
  }

  auto input = inputFor(route, 1.0);
  input.robot.pose.position = {0.0, 0.0, 0.48};
  input.environment.traversability = {traversability.data(), rows, cols, resolution, origin, origin};
  input.environment.obstacles = {legacy_obstacles.data(), static_cast<int>(legacy_obstacles.size() / 4U)};
  input.environment.collision = {
      occupied.data(),
      static_cast<int>(occupied.size() / 3U),
      0.25,
      {-20.0, -20.0, -5.75},
      {20.0, 20.0, 6.25},
      3U,
      9U,
      12U,
      1.0,
      1.0,
      true,
      true,
  };

  const auto result = planner.plan(input);
  const auto debug = planner.debugSnapshot();

  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready)
      << planner.debugSnapshot().searchReason << " expanded=" << debug.expandedNodes;
  ASSERT_TRUE(result.ready());
  const auto result_path = result.previewPath();
  const auto lateral =
      std::max_element(result_path.begin(), result_path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result_path.end());
  EXPECT_GT(std::abs(lateral->y), 0.70);
  EXPECT_LT(std::abs(lateral->y), 1.50);
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_search_ready");
  // Full 3D occupancy retains every inflated voxel layer.
  EXPECT_GE(debug.occupiedCellCount, 600);
}

TEST(ScanLocalPlanner, NearBodyObstacleIsNotDiscardedByPlannerGrid) {
  auto params = scanParams();
  params.useTraversabilityCost = false;
  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  std::vector<float> obstacles{0.25F, 0.0F, 0.0F, 0.8F};
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status(), nav_kernel::LocalPlanStatus::NearFieldStop);
  EXPECT_TRUE(result.status() == nav_kernel::LocalPlanStatus::NearFieldStop);
}

TEST(ScanLocalPlanner, ReboundLbfgsKeepsProjectedHeightFixed) {
  auto params = scanParams();
  params.scan.routeZTolerance = 0.8;
  params.scan.smoothingIterations = 16;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.2}, {1.6, 0.0, 0.4}, {2.4, 0.0, 0.6}};
  std::vector<float> obstacles;
  for (int yi = -7; yi <= 7; ++yi) {
    for (int zi = -10; zi <= 10; ++zi) {
      obstacles.push_back(1.2F);
      obstacles.push_back(static_cast<float>(yi) * 0.10F);
      obstacles.push_back(static_cast<float>(zi) * 0.10F);
      obstacles.push_back(0.8F);
    }
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());
  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);
  ASSERT_TRUE(searched.found()) << searched.reason;

  const auto seed = nav_kernel::local::scan::buildSeed(
      searched.path, input, params,
      nav_kernel::local::scan::SeedMode::Polynomial);
  ASSERT_TRUE(seed.valid());
  const auto seed_controls =
      nav_kernel::UniformSpline::parameterize(
          seed.interval, seed.samples, seed.boundary);
  ASSERT_GE(seed_controls.size(), 6U);
  const auto rebound = nav_kernel::local::scan::optimizeRebound(
      grid, seed_controls, seed.interval, params, nullptr);
  ASSERT_TRUE(rebound.anchorsComplete);
  EXPECT_TRUE(rebound.attempted);
  ASSERT_EQ(rebound.controls.size(), seed_controls.size());
  for (std::size_t index = 0U; index < seed_controls.size(); ++index) {
    EXPECT_DOUBLE_EQ(rebound.controls[index].z, seed_controls[index].z)
        << "official SCAN suppresses the complete Z-gradient at control "
        << index;
  }

  const auto spline = nav_kernel::local::scan::buildSpline(grid, searched.path, input, params);

  ASSERT_TRUE(spline.valid())
      << spline.reason << " segments=" << spline.collisionSegments
      << " anchor_searches=" << spline.anchorSearches
      << " restarts=" << spline.reboundRestarts;
  EXPECT_TRUE(spline.optimizerUsed);
  EXPECT_TRUE(spline.zGradientSuppressed);
  EXPECT_GT(spline.optimizerEvaluations, 1)
      << "status=" << spline.optimizerStatus << " initial=" << spline.optimizerInitialCost
      << " final=" << spline.optimizerFinalCost;
  EXPECT_FALSE(spline.fallback)
      << "the projected A* rebound guide should resolve this collision band"
      << " status=" << spline.optimizerStatus << " evaluations=" << spline.optimizerEvaluations
      << " initial=" << spline.optimizerInitialCost << " final=" << spline.optimizerFinalCost;
}

TEST(ScanLocalPlanner, BoundaryFallbackNeverExecutesVirtualLayer) {
  auto params = scanParams();
  params.scan.horizontalRange = 2.0;
  params.scan.maxSearchNodes = 100000;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.6, 0.0, 0.0}, {1.2, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  std::vector<float> obstacles;
  for (int yi = -22; yi <= 22; ++yi) {
    obstacles.push_back(0.8F);
    obstacles.push_back(static_cast<float>(yi) * 0.10F);
    obstacles.push_back(0.0F);
    obstacles.push_back(0.8F);
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  ASSERT_TRUE(searched.found()) << searched.reason;
  EXPECT_TRUE(searched.boundaryFallback);
  EXPECT_EQ(searched.reason, "boundary_fallback");
  ASSERT_GE(searched.path.size(), 2U);
  for (std::size_t i = 0; i < searched.path.size(); ++i) {
    EXPECT_TRUE(grid.contains(grid.index(searched.path[i])));
    if (i > 0) {
      EXPECT_TRUE(grid.segmentFree(searched.path[i - 1], searched.path[i]));
    }
    EXPECT_GE(searched.path[i].x, -0.05)
        << "fallback must not send the robot away from a forward goal";
  }
  EXPECT_GT(searched.path.back().x, 0.05);
  EXPECT_NEAR(nav_kernel::distance3D(searched.path.back(), searched.fallbackTarget), 0.0, 1e-8);
  EXPECT_GT(nav_kernel::distance3D(searched.path.back(), route.back()), grid.resolution())
      << "the hypothesis target itself must not become executable output";

  nav_kernel::local::scan::Backend planner(params);
  const auto result = planner.plan(input);
  ASSERT_EQ(result.status(), nav_kernel::LocalPlanStatus::Ready) << planner.debugSnapshot().searchReason;
  EXPECT_EQ(planner.debugSnapshot().searchReason, "scan_boundary_fallback_ready");
  ASSERT_TRUE(result.ready());
  for (const auto &point : result.previewPath()) {
    EXPECT_TRUE(grid.contains(grid.index(point)));
  }
}

TEST(ScanLocalPlanner, BoundaryFallbackEscapesWhenInsideGoalIsUnreachable) {
  auto params = scanParams();
  params.scan.horizontalRange = 2.0;
  params.scan.maxSearchNodes = 100000;
  params.vehicleLength = 0.30;
  params.vehicleWidth = 0.20;
  params.footprintPadding = 0.0;
  params.scan.cylinderOffset = 0.25 * params.vehicleLength;
  params.scan.cylinderRadius =
      std::hypot(0.25 * params.vehicleLength, 0.5 * params.vehicleWidth);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.4, 0.0, 0.0}};
  std::vector<float> obstacles;
  constexpr int kRingPoints = 96;
  constexpr double kRingRadius = 0.80;
  for (int index = 0; index < kRingPoints; ++index) {
    const double angle = 2.0 * M_PI * static_cast<double>(index) /
                         static_cast<double>(kRingPoints);
    obstacles.push_back(static_cast<float>(1.4 + kRingRadius * std::cos(angle)));
    obstacles.push_back(static_cast<float>(kRingRadius * std::sin(angle)));
    obstacles.push_back(0.0F);
    obstacles.push_back(0.8F);
  }
  auto input = inputFor(route, 1.0);
  const auto occupied = collisionCenters(obstacles);
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  ASSERT_TRUE(grid.contains(grid.index(route.back())));
  ASSERT_TRUE(grid.free(route.back(), 0.0));

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  ASSERT_TRUE(searched.found()) << searched.reason;
  EXPECT_TRUE(searched.boundaryFallback);
  EXPECT_EQ(searched.reason, "boundary_fallback");
  ASSERT_GE(searched.path.size(), 2U);
  for (const auto &point : searched.path)
    EXPECT_TRUE(grid.contains(grid.index(point)));
}

TEST(ScanLocalPlanner, BoundaryFallbackStopsAtForwardObservedFrontier) {
  auto params = scanParams();
  params.useTraversabilityCost = true;
  params.traversabilityHardCost = 80.0;
  params.scan.horizontalRange = 2.0;
  params.scan.maxSearchNodes = 100000;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.6, 0.0, 0.0}, {1.2, 0.0, 0.0}, {1.8, 0.0, 0.0}};
  constexpr int kRows = 60;
  constexpr int kCols = 60;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -3.0;
  std::vector<float> risk(static_cast<std::size_t>(kRows * kCols), 100.0F);
  for (int row = 20; row <= 39; ++row) {
    for (int col = 24; col <= 41; ++col) {
      risk[static_cast<std::size_t>(row * kCols + col)] = 0.0F;
    }
  }
  auto input = inputFor(route, 1.0);
  input.environment.traversability = {risk.data(), kRows, kCols, kResolution, kOrigin, kOrigin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  ASSERT_TRUE(searched.found()) << searched.reason;
  EXPECT_TRUE(searched.boundaryFallback);
  ASSERT_GE(searched.path.size(), 2U);
  EXPECT_GT(searched.path.back().x, 0.4);
  EXPECT_LT(searched.path.back().x, 1.2);
  EXPECT_NEAR(searched.path.back().y, 0.0, 0.2);
  EXPECT_LT(nav_kernel::distance3D(searched.path.back(), route.back()),
            nav_kernel::distance3D(route.front(), route.back()));
}

TEST(ScanLocalPlanner, UsesExactVehiclePoseForStartTraversability) {
  auto params = scanParams();
  params.useTraversabilityCost = true;
  params.traversabilityHardCost = 80.0;
  params.scan.horizontalRange = 2.0;
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  constexpr int kRows = 40;
  constexpr int kCols = 40;
  constexpr double kResolution = 0.05;
  constexpr double kOrigin = -1.0;
  std::vector<float> risk(static_cast<std::size_t>(kRows * kCols), 100.0F);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (std::abs(x) < 0.375 && std::abs(y) < 0.300) {
        risk[static_cast<std::size_t>(row * kCols + col)] = 0.0F;
      }
    }
  }
  auto input = inputFor(route, 1.0);
  input.environment.traversability = {risk.data(), kRows, kCols, kResolution, kOrigin, kOrigin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_TRUE(grid.free(input.robot.pose.position, input.robot.pose.yaw));
  EXPECT_FALSE(grid.free(grid.index(input.robot.pose.position), input.robot.pose.yaw))
      << "this fixture must expose the voxel-centre offset regression";
  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);
  EXPECT_NE(searched.reason, "start_traversability_blocked")
      << "the occupied current pose is the search seed; incremental poses remain checked";
}

TEST(ScanLocalPlanner, ChecksOccupiedStartInPlannedPathDirection) {
  auto params = scanParams();
  params.scan.horizontalRange = 2.0;
  params.scan.maxSearchNodes = 100000;
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.robot.pose.yaw = 0.5 * M_PI;
  const std::vector<float> occupied{
      0.0F, 0.50F, 0.0F,
      1.0F, 0.00F, 0.0F,
  };
  setCollision(input, occupied);
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_FALSE(grid.obstacleFree(input.robot.pose.position, input.robot.pose.yaw));
  EXPECT_TRUE(grid.obstacleFree(input.robot.pose.position, 0.0));

  const auto searched =
      nav_kernel::local::scan::search(grid, input.robot.pose.position, input.robot.pose.yaw, params);

  EXPECT_TRUE(searched.found()) << searched.reason;
  EXPECT_NE(searched.reason, "start_obstacle_blocked");
}
