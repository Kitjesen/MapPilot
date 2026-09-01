#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/path_searching/dyn_a_star.h"
#include "planning/local/scan/upstream/plan_env/grid_map.h"

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
  params.scan.cylinderRadius = 0.1;
  params.scan.maxAcceleration = 0.5;
  return params;
}

struct RequestFixture {
  explicit RequestFixture(std::vector<Vec3> value)
      : route(std::move(value)), bitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, 0.1) {
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

TEST(ScanDefaults, UsesOfficialGridAndSearchScale) {
  const nav_kernel::ScanPlannerParams params;
  EXPECT_DOUBLE_EQ(params.voxelResolution, 0.05);
  EXPECT_DOUBLE_EQ(params.cylinderRadius, 0.40);
  EXPECT_DOUBLE_EQ(params.cylinderOffset, 0.25);
}

TEST(ScanGridAdapter, ReadsMapdInflatedBitsWithoutReinflating) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.bitmap.occupy({0.0, 0.0, 0.5});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  EXPECT_EQ(map->getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
  EXPECT_EQ(map->getInflateOccupancy({0.5, 0.0, 0.5}, 0.0), 0);
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
  for (int x = -2; x <= 2; ++x) {
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
    plan = backend.plan(fixture.request);
  }

  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto *spline = std::get_if<SplineTarget>(&plan.target());
  ASSERT_NE(spline, nullptr);
  EXPECT_EQ(spline->order, 3);
  EXPECT_GT(spline->trajectoryId, 0);
  EXPECT_GE(spline->controls.size(), 4U);
  EXPECT_EQ(spline->knots.size(),
            spline->controls.size() + static_cast<std::size_t>(spline->order) + 1U);
}

TEST(ScanBackend, KeepsCommittedTrajectoryWhileOfficialFsmReplans) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());

  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.plan(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto firstId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.request.clock.timestampS += 0.01;
  fixture.refreshCollision(2);
  const LocalPlan retained = backend.plan(fixture.request);

  ASSERT_TRUE(retained.ready());
  EXPECT_GE(std::get<SplineTarget>(retained.target()).trajectoryId, firstId);
}

TEST(ScanBackend, DoesNotReturnCommittedSplineAfterCollisionWithoutReplacement) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.plan(fixture.request);
  }
  ASSERT_TRUE(ready.ready());

  fixture.bitmap.occupyInflated({0.5F, 0.0F, 0.5F}, 0.4, 0.2, 0.2);
  fixture.request.clock.timestampS += 0.05;
  fixture.refreshCollision(2);
  const LocalPlan updated = backend.plan(fixture.request);

  if (updated.ready()) {
    nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    for (const Vec3 &point : updated.previewPath())
      EXPECT_TRUE(grid.obstacleFree(point, 0.0));
  } else {
    EXPECT_EQ(updated.status(), nav_kernel::LocalPlanStatus::Blocked);
  }
}

TEST(ScanBackend, DoesNotPublishOldSplineForNewRouteIdentity) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.plan(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto previousId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.route = {{0.0, 0.0, 0.5}, {-2.0, 0.0, 0.5}};
  fixture.request.objective = RouteTarget{{
      fixture.route.data(), static_cast<int>(fixture.route.size()), 2, false}};
  fixture.request.clock.timestampS += 0.01;
  const LocalPlan changed = backend.plan(fixture.request);

  if (changed.ready()) {
    EXPECT_GT(std::get<SplineTarget>(changed.target()).trajectoryId, previousId);
  } else {
    EXPECT_EQ(changed.status(), nav_kernel::LocalPlanStatus::Pending);
  }
}
