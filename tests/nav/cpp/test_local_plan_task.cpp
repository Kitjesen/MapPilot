#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/task.hpp"

namespace {

nav_kernel::LocalPlannerParams scanParams() {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
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
  RequestFixture()
      : route{{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}},
        bitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, 0.1) {
    request.robot.pose = {route.front(), 0.0};
    request.objective = nav_kernel::RouteTarget{
        {route.data(), static_cast<int>(route.size()), 1, false}};
    request.identity = {1, 1, 0};
    request.clock.timestampS = 1.0;
    request.environment.collision = bitmap.view(1.0, 1);
  }

  void setGeneration(std::uint64_t generation) {
    request.identity.obstacleGeneration = generation;
    request.environment.collision =
        bitmap.view(request.clock.timestampS, generation);
  }

  void setIntent(double directionBodyDeg, std::uint64_t generation = 1) {
    request.objective = nav_kernel::MotionIntentTarget{
        {directionBodyDeg, 1.0, 2.0, 90.0},
        {route.data(), static_cast<int>(route.size()), generation, false}};
  }

  std::vector<nav_kernel::Vec3> route;
  lingtu::nav::tests::CollisionBitmap bitmap;
  nav_kernel::LocalPlanRequest request;
};

nav_kernel::LocalPlan waitForPlan(nav_kernel::local::LocalPlanTask &task,
                                  RequestFixture &fixture) {
  nav_kernel::LocalPlan plan;
  for (int tick = 0; tick < 400 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    fixture.setGeneration(1);
    plan = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return plan;
}

}  // namespace

TEST(LocalPlanTask, RejectsInvalidRouteWithoutStartingWork) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  nav_kernel::LocalPlanRequest request;
  EXPECT_EQ(task.update(request).plan.status(),
            nav_kernel::LocalPlanStatus::InvalidInput);
}

TEST(LocalPlanTask, RunsFsmOnOwnedTimer) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;

  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_TRUE(task.update(fixture.request).plan.ready());
}

TEST(LocalPlanTask, KeepsOfficialWorldFrameSplineDuringMapRefresh) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto &first = std::get<nav_kernel::SplineTarget>(ready.target());
  ASSERT_FALSE(first.controls.empty());

  fixture.request.robot.pose.position.x = 0.2;
  fixture.request.clock.timestampS += 0.01;
  fixture.setGeneration(2);
  const nav_kernel::LocalPlan retained = task.update(fixture.request).plan;

  ASSERT_TRUE(retained.ready());
  const auto &second = std::get<nav_kernel::SplineTarget>(retained.target());
  EXPECT_EQ(second.trajectoryId, first.trajectoryId);
  EXPECT_DOUBLE_EQ(second.controls.front().x, first.controls.front().x);
  EXPECT_DOUBLE_EQ(second.controls.front().y, first.controls.front().y);
}

TEST(LocalPlanTask, StampsSplineWhenAsyncPlanningCompletes) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto &spline = std::get<nav_kernel::SplineTarget>(ready.target());

  EXPECT_NEAR(spline.startTimeS, fixture.request.clock.timestampS, 0.05);
}

TEST(LocalPlanTask, NewIntentGenerationDoesNotReuseOldSpline) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.setIntent(0.0);
  ASSERT_TRUE(waitForPlan(task, fixture).ready());

  fixture.setIntent(90.0, 2);
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
}

TEST(LocalPlanTask, ProcessesResetEpochOnCollisionTimer) {
  nav_kernel::local::LocalPlanTask task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto firstId =
      std::get<nav_kernel::SplineTarget>(ready.target()).trajectoryId;

  fixture.bitmap.occupyInflated({0.5F, 0.0F, 0.5F}, 0.4, 0.2, 0.2);
  nav_kernel::LocalPlan updated;
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.setGeneration(1);
    fixture.request.environment.collision.resetEpoch = 2;
    updated = task.update(fixture.request).plan;
    if (updated.ready() &&
        std::get<nav_kernel::SplineTarget>(updated.target()).trajectoryId >
            firstId) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(updated.ready());
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(updated.target()).trajectoryId,
            firstId);
}
