#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <vector>

#include "planning/local/task.hpp"

TEST(LocalPlanTask, OwnsSubmittedInput) {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  params.scan.voxelResolution = 0.10;
  params.scan.horizontalRange = 4.0;

  nav_kernel::local::LocalPlanTask task(params);
  ASSERT_TRUE(task.configure());

  std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {1.0, 0.2, 0.0},
      {2.0, 0.2, 0.0},
  };
  nav_kernel::LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), 7U, false}};
  input.identity.frameEpoch = 3U;
  input.identity.obstacleGeneration = 11U;
  input.clock.timestampS = 1.0;
  input.clock.executionTimeS = 0.75;

  auto update = task.update(input);
  EXPECT_EQ(update.plan.status(), nav_kernel::LocalPlanStatus::Pending);
  route[1].x = std::numeric_limits<double>::quiet_NaN();

  const std::vector<nav_kernel::Vec3> current_route{
      {0.0, 0.0, 0.0},
      {1.0, 0.2, 0.0},
      {2.0, 0.2, 0.0},
  };
  nav_kernel::LocalPlanRequest current = input;
  current.objective = nav_kernel::RouteTarget{{
      current_route.data(), static_cast<int>(current_route.size()), 7U, false}};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline && !update.plan.ready()) {
    update = task.update(current);
    if (!update.plan.ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  EXPECT_EQ(update.plan.status(), nav_kernel::LocalPlanStatus::Ready)
      << update.debug.searchReason;
  EXPECT_TRUE(update.plan.ready());
  EXPECT_GE(update.plan.previewPath().size(), 2U);
  EXPECT_TRUE(std::holds_alternative<nav_kernel::SplineTarget>(update.plan.target()));
  EXPECT_FALSE(std::get<nav_kernel::SplineTarget>(update.plan.target()).controls.empty());
}

TEST(LocalPlanTask, PublishesDeadlineFailureInsteadOfLatePlan) {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.scan.voxelResolution = 0.10;
  params.scan.horizontalRange = 4.0;
  params.scan.planningDeadlineS = 0.01;

  nav_kernel::local::LocalPlanTask task(params);
  ASSERT_TRUE(task.configure());

  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  std::vector<float> occupied_xyz(200000U * 3U);
  for (std::size_t index = 0U; index < occupied_xyz.size(); index += 3U) {
    occupied_xyz[index] = 3.0F;
    occupied_xyz[index + 1U] = 3.0F;
    occupied_xyz[index + 2U] = 0.5F;
  }
  nav_kernel::LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), 1U, false}};
  input.identity = {1U, 1U, 0U};
  input.environment.collision = {
      occupied_xyz.data(),
      static_cast<int>(occupied_xyz.size() / 3U),
      0.10,
      {-5.0, -5.0, -2.0},
      {5.0, 5.0, 2.0},
      1U,
      1U,
      1U,
      1.0,
      1.0,
      true,
      true,
  };
  input.clock.timestampS = 1.0;

  auto update = task.update(input);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         update.plan.status() == nav_kernel::LocalPlanStatus::Pending) {
    update = task.update(input);
    if (update.plan.status() == nav_kernel::LocalPlanStatus::Pending)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_EQ(update.plan.status(), nav_kernel::LocalPlanStatus::Expired);
  EXPECT_EQ(update.debug.searchReason, "planning_deadline_exceeded");
  EXPECT_FALSE(update.plan.ready());
  EXPECT_TRUE(std::get<nav_kernel::PathTarget>(update.plan.target()).points.empty());
  EXPECT_EQ(update.plan.hints().slowdownLevel, 0);
}
