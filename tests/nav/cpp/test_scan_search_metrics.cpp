#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"
#include "planning/local/scan/upstream/path_searching/dyn_a_star.h"
#include "planning/local/scan/upstream/traj_utils/polynomial_traj.h"

namespace {

using namespace nav_kernel::local::scan::upstream;

PolynomialTraj polynomial(const std::vector<double> &x, double duration) {
  PolynomialTraj result;
  const std::vector<double> zero(x.size(), 0.0);
  result.addSegment(x, zero, zero, duration);
  result.init();
  return result;
}

UniformBspline straightSpline(double duration) {
  Eigen::MatrixXd controls(3, 2);
  controls << 0.0, 3.0, 0.0, 4.0, 0.0, 0.0;
  return UniformBspline(controls, 1, duration);
}

struct SearchMap {
  lingtu::nav::tests::CollisionBitmap bitmap{{0, 0, -1}, {15, 15, 2}, 0.5};
  nav_kernel::LocalPlanRequest request{};
  nav_kernel::LocalPlannerParams params{};

  SearchMap() {
    // These obstacles allow a six-diagonal, four-axial route, but require
    // decreasing scores for nodes that were already discovered.
    for (const auto &cell : std::array<std::array<int, 2>, 5>{
             {{4, 4}, {8, 5}, {8, 7}, {9, 6}, {9, 7}}}) {
      bitmap.occupy({static_cast<double>(cell[0]),
                     static_cast<double>(cell[1]), 0.0});
    }
    params.checkObstacle = true;
    params.scan.voxelResolution = 0.5;
    params.scan.cylinderOffset = 0.0;
    request.environment.collision = bitmap.view();
  }
};

TEST(ScanAStarCoordinates, PreservesPhysicalEndpointsInBothDirectionsAndHeightSigns) {
  nav_kernel::LocalPlannerParams params;
  params.checkObstacle = false;
  params.scan.voxelResolution = .1;
  nav_kernel::LocalPlanRequest request;
  nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  AStar search;
  search.initGridMap(std::make_shared<GridMap>(grid), {30, 30, 30});
  search.setTimeSource([] { return 0.0; });
  for (double x : {-1.0, 1.0}) for (double y : {-1.0, 1.0})
    for (double z : {-1.0, 1.0}) {
      // Endpoints are exact 10 cm lattice positions about this shifted centre.
      // Their physical positions must not depend on the signs of local XYZ.
      const Eigen::Vector3d centre{1.0, .5, .5};
      const Eigen::Vector3d delta{.4*x, .2*y, .2*z};
      const Eigen::Vector3d start=centre-delta, end=centre+delta;
      ASSERT_EQ(search.search(.1, start, end), AStarResult::Success);
      const auto route=search.path();
      ASSERT_GE(route.size(), 2U);
      EXPECT_LT((route.front()-start).norm(), 1e-12);
      EXPECT_LT((route.back()-end).norm(), 1e-12);
      const Eigen::Vector2d direction=(end-start).head<2>();
      for (const auto& point : route) {
        const double progress=(point-start).head<2>().dot(direction)/direction.squaredNorm();
        const double height=start.z()+std::clamp(progress, 0.0, 1.0)*(end.z()-start.z());
        // Actual returned/query points stay within half a vertical search cell
        // of the requested search plane, including its negative-height half.
        EXPECT_LE(std::abs(point.z()-height), .05+1e-12);
      }
    }
}

TEST(ScanAStarQueue, ReturnsTheShorterRouteAfterAnOpenNodeScoreDecreases) {
  SearchMap fixture;
  nav_kernel::local::scan::Grid grid(fixture.params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  AStar search;
  search.initGridMap(std::make_shared<GridMap>(grid), {30, 30, 6});
  search.setTimeSource([] { return 0.0; });
  ASSERT_EQ(search.search(1.0, {2, 2, 0}, {12, 8, 0}), AStarResult::Success);
  const auto route = search.path();
  ASSERT_GE(route.size(), 2U);
  ASSERT_NEAR((route.front() - Eigen::Vector3d(2, 2, 0)).norm(), 0.0, 1e-12);
  double length = 0.0;
  for (std::size_t index = 1; index < route.size(); ++index)
    length += (route[index] - route[index - 1]).norm();
  // Independent lower bound: dx=10, dy=6 needs >=6 diagonals + 4 axial moves.
  EXPECT_NEAR(length, 4.0 + 6.0 * std::sqrt(2.0), 1e-12);
}

TEST(ScanAStarQueue, ExpandsEachReachableNodeAtMostOnceAfterScoreDecreases) {
  SearchMap fixture;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx != 0 || dy != 0) fixture.bitmap.occupy({12.0 + dx, 8.0 + dy, 0});
    }
  }
  fixture.request.environment.collision = fixture.bitmap.view();
  nav_kernel::local::scan::Grid grid(fixture.params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  AStar search;
  search.initGridMap(std::make_shared<GridMap>(grid), {30, 30, 6});
  search.setTimeSource([] { return 0.0; });
  ASSERT_EQ(search.search(1.0, {2, 2, 0}, {12, 8, 0}), AStarResult::SearchError);
  // Only integer XY cells 1..14 are inside the map. The goal is isolated;
  // 13 occupied cells and that goal cannot be expanded from the start.
  EXPECT_LE(search.expandedNodes(), 14 * 14 - 13 - 1);
}

TEST(ScanAStarQueue, ClearsThePreviousSuccessfulPathWhenTheNextSearchFails) {
  SearchMap fixture;
  nav_kernel::local::scan::Grid grid(fixture.params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  AStar search;
  search.initGridMap(std::make_shared<GridMap>(grid), {30, 30, 6});
  search.setTimeSource([] { return 0.0; });
  ASSERT_EQ(search.search(1.0, {2, 2, 0}, {12, 8, 0}), AStarResult::Success);
  ASSERT_FALSE(search.path().empty());
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx != 0 || dy != 0) fixture.bitmap.occupy({12.0 + dx, 8.0 + dy, 0});
    }
  }
  ASSERT_EQ(search.search(1.0, {2, 2, 0}, {12, 8, 0}), AStarResult::SearchError);
  EXPECT_TRUE(search.path().empty());
}

TEST(ScanPolynomialMetrics, ReturnsCachedLengthDividedByDurationAsMeanVelocity) {
  auto trajectory = polynomial({0, 0, 2, 0}, 0.035);
  trajectory.getTraj();
  const double sampledLength = trajectory.getLength();
  ASSERT_NEAR(sampledLength, 0.06, 1e-12);
  EXPECT_NEAR(trajectory.getMeanVel(), sampledLength / 0.035, 1e-12);
}

TEST(ScanPolynomialMetrics, AveragesVelocityAtEachSampleTime) {
  auto trajectory = polynomial({1, 0, 0}, 0.035);
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxVel(mean, maximum);
  // x=t^2: samples at 0,.01,.02,.03 have speeds 0,.02,.04,.06.
  EXPECT_NEAR(mean, 0.03, 1e-12);
}

TEST(ScanPolynomialMetrics, FindsMaximumVelocityAmongTheSampleTimes) {
  auto trajectory = polynomial({1, 0, 0}, 0.035);
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxVel(mean, maximum);
  EXPECT_NEAR(maximum, 0.06, 1e-12);
}

TEST(ScanPolynomialMetrics, RestartsVelocitySamplingAtEachSegment) {
  auto trajectory = polynomial({1, 0, 0}, 0.025);
  trajectory.addSegment({3, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0.015);
  trajectory.init();
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxVel(mean, maximum);
  EXPECT_NEAR(mean, 0.024, 1e-12);
}

TEST(ScanPolynomialMetrics, AveragesAccelerationAtEachSampleTime) {
  auto trajectory = polynomial({1, 0, 0, 0}, 0.035);
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxAcc(mean, maximum);
  // x=t^3: a=6t, sampled at 0,.01,.02,.03.
  EXPECT_NEAR(mean, 0.09, 1e-12);
}

TEST(ScanPolynomialMetrics, FindsMaximumAccelerationAmongTheSampleTimes) {
  auto trajectory = polynomial({1, 0, 0, 0}, 0.035);
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxAcc(mean, maximum);
  EXPECT_NEAR(maximum, 0.18, 1e-12);
}

TEST(ScanPolynomialMetrics, RestartsAccelerationSamplingAtEachSegment) {
  auto trajectory = polynomial({1, 0, 0, 0}, 0.025);
  trajectory.addSegment({2, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 0.015);
  trajectory.init();
  double mean = 0.0, maximum = 0.0;
  trajectory.getMeanAndMaxAcc(mean, maximum);
  EXPECT_NEAR(mean, 0.06, 1e-12);
}

TEST(ScanPolynomialMetrics, IntegratesTimeVaryingAccelerationSquared) {
  auto trajectory = polynomial({1, 0, 0, 0}, 2.0);
  // Integral of (6t)^2 from 0 to 2 is 96.
  EXPECT_NEAR(trajectory.getAccCost(), 96.0, 1e-12);
}

TEST(ScanPolynomialMetrics, IntegratesQuinticAccelerationSquared) {
  auto trajectory = polynomial({1, 0, 0, 0, 0, 0}, 1.0);
  // x=t^5 gives a=20t^3, whose squared integral over [0,1] is 400/7.
  EXPECT_NEAR(trajectory.getAccCost(), 400.0 / 7.0, 1e-12);
}

TEST(ScanPolynomialMetrics, IncludesCrossTermsInTheAccelerationIntegral) {
  auto trajectory = polynomial({1, 1, 0, 0}, 2.0);
  // Integral of (6t+2)^2 from 0 to 2 is 96+48+8.
  EXPECT_NEAR(trajectory.getAccCost(), 152.0, 1e-12);
}

TEST(ScanPolynomialMetrics, AddsAccelerationCostAcrossAllThreeAxes) {
  PolynomialTraj trajectory;
  trajectory.addSegment({1, 0, 0, 0}, {0, 2, 0, 0}, {0, 0, 0, 0}, 2.0);
  trajectory.init();
  EXPECT_NEAR(trajectory.getAccCost(), 128.0, 1e-12);
}

TEST(ScanPolynomialMetrics, IntegratesEachSegmentUsingItsOwnPolynomialOrder) {
  auto trajectory = polynomial({1, 0, 0}, 2.0);
  trajectory.addSegment({1, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 1.0);
  trajectory.init();
  EXPECT_NEAR(trajectory.getAccCost(), 20.0, 1e-12);
}

TEST(ScanSplineMetrics, IncludesTheFinalPartialSamplingIntervalInLength) {
  EXPECT_NEAR(straightSpline(0.035).getLength(0.01), 5.0, 1e-12);
}

TEST(ScanSplineMetrics, MeasuresLengthWhenDurationIsShorterThanSamplingInterval) {
  EXPECT_NEAR(straightSpline(0.005).getLength(0.01), 5.0, 1e-12);
}

TEST(ScanSplineMetrics, DoesNotCountTheEndpointTwiceForAnExactSamplingInterval) {
  EXPECT_NEAR(straightSpline(0.04).getLength(0.01), 5.0, 1e-12);
}

}  // namespace
