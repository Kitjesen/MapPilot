#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "planning/local/scan/task.hpp"

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

nav_kernel::LocalPlan waitForPlan(nav_kernel::local::scan::Task &task,
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
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  nav_kernel::LocalPlanRequest request;
  EXPECT_EQ(task.update(request).plan.status(),
            nav_kernel::LocalPlanStatus::InvalidInput);
}

TEST(LocalPlanTask, RejectsNonFiniteReferenceSnapshot) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.route.back().x = std::numeric_limits<double>::quiet_NaN();
  auto result = task.update(fixture.request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result.plan.status() == nav_kernel::LocalPlanStatus::Pending &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    result = task.update(fixture.request);
  }
  EXPECT_EQ(result.plan.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_FALSE(result.plan.ready());
}

TEST(LocalPlanTask, RunsFsmOnOwnedTimer) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;

  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_TRUE(task.update(fixture.request).plan.ready());
}

TEST(LocalPlanTask, PausesOwnedClockForTimerGeneratedReferences) {
  nav_kernel::LocalPlannerParams runningParams = scanParams();
  runningParams.scan.noReplanDistance = 0.02;
  runningParams.scan.replanDistance = 0.02;

  nav_kernel::local::scan::Task pausedTask(runningParams);
  nav_kernel::local::scan::Task runningTask(runningParams);
  ASSERT_TRUE(pausedTask.configure());
  ASSERT_TRUE(runningTask.configure());
  RequestFixture pausedFixture;
  RequestFixture runningFixture;

  const nav_kernel::LocalPlan pausedInitial =
      waitForPlan(pausedTask, pausedFixture);
  pausedTask.pause();
  const nav_kernel::LocalPlan runningInitial =
      waitForPlan(runningTask, runningFixture);
  ASSERT_TRUE(pausedInitial.ready());
  ASSERT_TRUE(runningInitial.ready());
  const auto pausedInitialId =
      std::get<nav_kernel::SplineTarget>(pausedInitial.target()).trajectoryId;
  const auto runningInitialId =
      std::get<nav_kernel::SplineTarget>(runningInitial.target()).trajectoryId;

  // Neither task receives a source update while timer callbacks advance.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  const nav_kernel::LocalPlan pausedAfter =
      pausedTask.update(pausedFixture.request).plan;
  const nav_kernel::LocalPlan runningAfter =
      runningTask.update(runningFixture.request).plan;
  ASSERT_TRUE(pausedAfter.ready());
  ASSERT_TRUE(runningAfter.ready());
  EXPECT_EQ(std::get<nav_kernel::SplineTarget>(pausedAfter.target()).trajectoryId,
            pausedInitialId);
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(runningAfter.target()).trajectoryId,
             runningInitialId);
}

TEST(LocalPlanTask, KeepsOfficialWorldFrameSplineDuringMapRefresh) {
  nav_kernel::local::scan::Task task(scanParams());
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
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto &spline = std::get<nav_kernel::SplineTarget>(ready.target());

  EXPECT_NEAR(spline.startTimeS, fixture.request.clock.timestampS, 0.05);
}

TEST(LocalPlanTask, KeepsPublishedSplineUntilNewIntentIsPlanned) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.setIntent(0.0);
  ASSERT_TRUE(waitForPlan(task, fixture).ready());

  fixture.setIntent(90.0, 2);
  fixture.request.clock.timestampS += 0.01;
  EXPECT_TRUE(task.update(fixture.request).plan.ready());
}

TEST(LocalPlanTask, RouteGenerationOwnsReferenceReplacement) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan initial = waitForPlan(task, fixture);
  ASSERT_TRUE(initial.ready());
  ASSERT_FALSE(initial.previewPath().empty());
  EXPECT_GT(initial.previewPath().back().x,
            std::abs(initial.previewPath().back().y));

  fixture.route[1] = {0.0, 2.0, 0.5};
  nav_kernel::LocalPlan retained = initial;
  for (int tick = 0; tick < 60; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    retained = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(retained.ready());
  ASSERT_FALSE(retained.previewPath().empty());
  EXPECT_GT(retained.previewPath().back().x,
            std::abs(retained.previewPath().back().y));

  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 2, false}};
  nav_kernel::LocalPlan replaced = retained;
  for (int tick = 0; tick < 200; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replaced = task.update(fixture.request).plan;
    if (replaced.ready() && !replaced.previewPath().empty() &&
        replaced.previewPath().back().y >
            std::abs(replaced.previewPath().back().x)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(replaced.ready());
  ASSERT_FALSE(replaced.previewPath().empty());
  EXPECT_GT(replaced.previewPath().back().y,
            std::abs(replaced.previewPath().back().x));
}

TEST(LocalPlanTask, OwnsReferenceAcrossAsyncTicks) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);

  // Destroy the caller's buffer while the worker still owns this generation.
  std::vector<nav_kernel::Vec3>{{0.0, 0.0, 0.5}, {0.0, 2.0, 0.5}}
      .swap(fixture.route);
  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 1, false}};

  const nav_kernel::LocalPlan plan = waitForPlan(task, fixture);
  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().x,
            std::abs(plan.previewPath().back().y));
}

TEST(LocalPlanTask, UsesCompleteReferenceInsteadOfShortGuide) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const std::vector<nav_kernel::Vec3> reference{
      {0.0, 0.0, 0.5}, {0.0, 1.0, 0.5}, {0.0, 2.0, 0.5}};
  fixture.request.reference = {
      reference.data(), static_cast<int>(reference.size()), 1, true};

  const nav_kernel::LocalPlan plan = waitForPlan(task, fixture);

  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().y,
            std::abs(plan.previewPath().back().x));
}

TEST(LocalPlanTask, DropsCompletionForSupersededReference) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fixture.route[1] = {0.0, 2.0, 0.5};
  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 2, false}};

  nav_kernel::LocalPlan plan;
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS = 1.1 + 0.01 * tick;
    fixture.setGeneration(1);
    plan = task.update(fixture.request).plan;
    if (plan.ready()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().y,
            std::abs(plan.previewPath().back().x));
}

TEST(LocalPlanTask, ProcessesResetEpochOnCollisionTimer) {
  nav_kernel::local::scan::Task task(scanParams());
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
