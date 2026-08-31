#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

#include "planner_fixture.hpp"
#include "planning/local/planner.hpp"
#include "planning/local/recovery.hpp"

using namespace nav_kernel;

static int correspondenceVoxel(double x, double y, double searchRadius) {
  constexpr double voxelSize = 0.02;
  constexpr double offsetX = 3.2;
  constexpr double offsetY = 4.5;
  constexpr int voxelCountY = 451;
  const double scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX;
  const int indexX = static_cast<int>((offsetX + voxelSize * 0.5 - x) / voxelSize);
  const int indexY = static_cast<int>((offsetY + voxelSize * 0.5 - y / scaleY) / voxelSize);
  return voxelCountY * indexX + indexY;
}

static std::filesystem::path writeMinimalPlannerPaths(const std::string &name,
                                                      double searchRadius = 0.45,
                                                      int blockedVoxel = -1) {
  auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  {
    std::ofstream f(dir / "startPaths.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kGroupNum << "\nend_header\n";
    for (int g = 0; g < kGroupNum; g++) {
      f << "1 0 0 " << g << "\n";
    }
  }

  {
    std::ofstream f(dir / "pathList.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kPathNum << "\nend_header\n";
    for (int i = 0; i < kPathNum; i++) {
      f << "1 0 0 " << i << " 3\n";
    }
  }

  {
    std::ofstream f(dir / "correspondences.txt");
    for (int i = 0; i < 161 * 451; i++) {
      f << i << ' ';
      if (i == blockedVoxel) {
        for (int path = 0; path < kPathNum; ++path) {
          f << path << ' ';
        }
      }
      f << "-1\n";
    }
  }

  {
    std::ofstream f(dir / "search_radius.txt");
    f << searchRadius << '\n';
  }

  return dir;
}

static std::filesystem::path writePlannerPathsWithStartPoints(const std::string &name,
                                                              const std::vector<double> &start_xs) {
  auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  {
    std::ofstream f(dir / "startPaths.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kGroupNum * start_xs.size()
      << "\nend_header\n";
    for (int g = 0; g < kGroupNum; g++) {
      for (double start_x : start_xs) {
        f << start_x << " 0 0 " << g << "\n";
      }
    }
  }

  {
    std::ofstream f(dir / "pathList.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kPathNum << "\nend_header\n";
    for (int i = 0; i < kPathNum; i++) {
      f << "1 0 0 " << i << " 3\n";
    }
  }

  {
    std::ofstream f(dir / "correspondences.txt");
    for (int i = 0; i < 161 * 451; i++) {
      f << i << " -1\n";
    }
  }

  {
    std::ofstream f(dir / "search_radius.txt");
    f << "0.45\n";
  }

  return dir;
}

static std::filesystem::path writePlannerPathsWithStartPoint(const std::string &name,
                                                             double start_x) {
  return writePlannerPathsWithStartPoints(name, {start_x});
}

TEST(LocalPlanner, RejectsTruncatedStartPathLibrary) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_truncated_start_paths_fixture");
  {
    std::ofstream f(pathsDir / "startPaths.ply", std::ios::trunc);
    f << "ply\nformat ascii 1.0\nelement vertex 1\nend_header\n";
  }

  PlannerFixture planner;
  EXPECT_FALSE(planner.configure(pathsDir.string()));
  EXPECT_FALSE(planner.configured());
}

TEST(LocalPlanner, RejectsMissingSearchRadiusMetadata) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_missing_search_radius_fixture");
  std::filesystem::remove(pathsDir / "search_radius.txt");

  PlannerFixture planner;
  EXPECT_FALSE(planner.configure(pathsDir.string()));
  EXPECT_FALSE(planner.configured());
}

TEST(LocalPlanner, UsesPathLibrarySearchRadiusForVoxelLookup) {
  constexpr double searchRadius = 0.60;
  constexpr double obstacleX = 1.0;
  constexpr double obstacleY = 0.5;
  const int libraryVoxel = correspondenceVoxel(obstacleX, obstacleY, searchRadius);
  ASSERT_NE(libraryVoxel, correspondenceVoxel(obstacleX, obstacleY, 0.45));
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_search_radius_voxel_fixture", searchRadius, libraryVoxel);

  LocalPlannerParams params;
  params.checkObstacle = true;
  params.checkRotObstacle = false;
  params.useTraversabilityCost = false;
  params.pathScale = 1.0;
  params.minPathScale = 1.0;
  params.pathScaleBySpeed = false;
  params.adjacentRange = 2.0;
  params.minPathRange = 2.0;
  params.pathRangeBySpeed = false;
  params.dirThre = 0.1;
  params.pointPerPathThre = 1;
  params.nearFieldStopDis = 0.0;

  PlannerFixture planner(params);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0.0, 0.0, 0.0, 0.0);
  planner.setGoal(3.0, 0.0);
  const std::vector<float> obstacle{
      static_cast<float>(obstacleX), static_cast<float>(obstacleY), 1.0F, 1.0F};

  const auto result = planner.plan(obstacle.data(), 1, 1.0);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, FailedReloadKeepsPreviousPathLibraryAtomic) {
  auto goodPaths = writePlannerPathsWithStartPoint("nav_kernel_atomic_path_library_good", 0.4);
  auto badPaths = writePlannerPathsWithStartPoint("nav_kernel_atomic_path_library_bad", 4.0);
  std::filesystem::remove(badPaths / "pathList.ply");

  LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;
  PlannerFixture planner(params);
  ASSERT_TRUE(planner.configure(goodPaths.string()));
  planner.setVehicle(0.0, 0.0, 0.0, 0.0);
  planner.setGoal(5.0, 0.0);
  const auto before = planner.plan(nullptr, 0, 1.0);
  ASSERT_FALSE(before.previewPath().empty());
  ASSERT_FALSE(before.previewPath().empty());

  EXPECT_FALSE(planner.configure(badPaths.string()));
  EXPECT_TRUE(planner.configured());
  const auto after = planner.plan(nullptr, 0, 2.0);
  ASSERT_FALSE(after.previewPath().empty());
  ASSERT_FALSE(after.previewPath().empty());
  EXPECT_NEAR(after.previewPath().front().x, before.previewPath().front().x, 1e-6);
  EXPECT_NEAR(after.previewPath().front().y, before.previewPath().front().y, 1e-6);
}

TEST(LocalPlanner, CmuUsesSalientRouteBendWhenTerminalTargetsMatch) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_cmu_route_bend_guide", 0.4);
  LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;
  local::Planner planner(params);
  ASSERT_TRUE(planner.configure(pathsDir.string()));

  const auto plan_route = [&planner](const std::array<Vec3, 3> &route, double timestamp) {
    LocalPlanRequest input;
    input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
    input.objective = nav_kernel::RouteTarget{{
        route.data(), static_cast<int>(route.size()), 1, false}};
    input.clock.timestampS = timestamp;
    return planner.plan(input);
  };
  const auto left = plan_route({{{0.0, 0.0, 0.0}, {0.5, 1.0, 0.0}, {2.0, 0.0, 0.0}}}, 1.0);
  const auto right = plan_route({{{0.0, 0.0, 0.0}, {0.5, -1.0, 0.0}, {2.0, 0.0, 0.0}}}, 2.0);

  ASSERT_FALSE(left.previewPath().empty());
  ASSERT_FALSE(right.previewPath().empty());
  ASSERT_FALSE(left.previewPath().empty());
  ASSERT_FALSE(right.previewPath().empty());
  EXPECT_GT(std::abs(left.previewPath().back().y - right.previewPath().back().y), 0.1);
}

TEST(LocalPlanner, CmuKeepsForwardHorizonWhenSalientBendIsNear) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_cmu_near_bend_forward_horizon", 0.4);
  LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;
  params.debugCandidateLimit = 1;
  local::Planner planner(params);
  ASSERT_TRUE(planner.configure(pathsDir.string()));

  const std::array<Vec3, 3> route{{
      {0.0, 0.0, 0.0},
      {0.10, 0.25, 0.0},
      {2.0, 0.0, 0.0},
  }};
  LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), 1, false}};
  input.clock.timestampS = 1.0;

  const auto result = planner.plan(input);

  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_GT(result.previewPath().back().y, 0.05)
      << "the near bend must still determine the local path direction";
  EXPECT_GT(planner.debugSnapshot().relativeGoalDistanceM, 1.5)
      << "a near route bend must not crop the local path to the follower stop distance";
}

TEST(LocalPlanner, CmuUsesMostSalientBendInDenseGridRoute) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_cmu_dense_route_bend_guide", 0.4);
  LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = false;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;
  params.debugCandidateLimit = 1;
  local::Planner planner(params);
  ASSERT_TRUE(planner.configure(pathsDir.string()));

  const std::array<Vec3, 8> route{{
      {0.0, 0.0, 0.0},
      {0.2, 0.0, 0.0},
      {0.4, 0.2, 0.0},
      {0.6, 0.4, 0.0},
      {0.8, 0.7, 0.0},
      {1.2, 0.9, 0.0},
      {1.8, 0.7, 0.0},
      {2.4, 0.2, 0.0},
  }};
  LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), 1, false}};
  input.clock.timestampS = 1.0;

  const auto result = planner.plan(input);

  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_GT(planner.debugSnapshot().relativeGoalDistanceM, 1.0);
  EXPECT_GT(result.previewPath().back().y, 0.1);
}

TEST(LocalPlanRequest, MissingGridDoesNotReusePreviousFrame) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_atomic_plan_input_grid", 0.4);

  LocalPlannerParams params;
  params.checkObstacle = false;
  params.useTraversabilityCost = true;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;

  local::Planner planner(params);
  ASSERT_TRUE(planner.configure(pathsDir.string()));

  std::vector<float> blockedGrid(5 * 5, std::numeric_limits<float>::quiet_NaN());
  LocalPlanRequest input;
  input.robot.pose = {{0.0, 0.0, 0.0}, 0.0};
  const std::array<Vec3, 2> route{{{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}};
  input.objective = nav_kernel::RouteTarget{{
      route.data(), static_cast<int>(route.size()), 1, true}};
  input.environment.traversability = {blockedGrid.data(), 5, 5, 1.0, -2.5, -2.5};
  input.clock.timestampS = 1.0;

  const auto blocked = planner.plan(input);
  EXPECT_FALSE(blocked.ready());

  input.environment.traversability = {};
  input.clock.timestampS = 2.0;
  const auto clear = planner.plan(input);
  EXPECT_FALSE(clear.previewPath().empty());
}

// Local planner

TEST(LocalPlanner, CheckRotObstacleFiltersPlanSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_check_rot_obstacle_fixture");

  LocalPlannerParams p;
  EXPECT_FALSE(p.useCost);
  p.checkObstacle = false;
  p.checkRotObstacle = true;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  const float angle = -10.0f * static_cast<float>(M_PI) / 180.0f;
  const float radius = 0.45f;
  std::vector<float> cloud = {radius * std::cos(angle), radius * std::sin(angle), 0.0f, 1.0f};

  auto result = planner.plan(cloud.data(), 1, 0.0);

  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_GT(result.previewPath().front().y, 0.1);
  EXPECT_GT(result.previewPath().front().x, 0.9);
  const auto snapshot = planner.debugSnapshot();
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::any_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                          [](const LocalPlanCandidate &candidate) {
                            return candidate.state == LocalCandidateState::RotationBlocked;
                          }));
  EXPECT_TRUE(std::none_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                           [](const LocalPlanCandidate &candidate) {
                             return candidate.selected &&
                                    candidate.state == LocalCandidateState::RotationBlocked;
                           }));
}

TEST(LocalPlanner, TwoWayDriveKeepsCloseBehindGoalTrackable) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_two_way_close_behind_fixture", 0.4);

  LocalPlannerParams p;
  p.twoWayDrive = true;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.goalBehindRange = 0.8;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-0.6, 0);

  auto result = planner.plan(nullptr, 0, 1.0);

  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_LT(result.previewPath().front().x, -0.1);
}

TEST(LocalPlanner, SingleDirectionDriveReleasesCloseBehindGoalAfterFreeze) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_single_direction_close_behind_fixture", 0.4);

  LocalPlannerParams p;
  p.twoWayDrive = false;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.goalBehindRange = 0.8;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-0.6, 0);

  auto result = planner.plan(nullptr, 0, 1.0);

  EXPECT_FALSE(result.ready());

  result = planner.plan(nullptr, 0, 3.1);
  EXPECT_GT(planner.debugSnapshot().relativeGoalDistanceM, 0.5)
      << "single-direction drive must release the goal after the configured heading freeze";
}

// RotLUT precomputed weights

TEST(LocalPlanner, TraversabilityGridTriggersNearFieldStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_stop_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(5 * 5, 0.0f);
  riskGrid[2 * 5 + 2] = 95.0f;
  riskGrid[2 * 5 + 3] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), 5, 5, 0.25, 0.0, -0.5);

  auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, FrontTraversabilityDoesNotStopReverseIntent) {
  auto pathsDir =
      writeMinimalPlannerPaths("nav_kernel_reverse_ignores_front_traversability_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  constexpr int kSize = 25;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -1.25;
  std::vector<float> riskGrid(kSize * kSize, 0.0f);
  const int row = static_cast<int>((0.0 - kOrigin) / kResolution);
  const int frontCol = static_cast<int>((0.6 - kOrigin) / kResolution);
  riskGrid[row * kSize + frontCol] = 100.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kSize, kSize, kResolution, kOrigin, kOrigin);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, RearTraversabilityStopsReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_reverse_checks_rear_traversability_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  constexpr int kSize = 25;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -1.25;
  std::vector<float> riskGrid(kSize * kSize, 0.0f);
  const int row = static_cast<int>((0.0 - kOrigin) / kResolution);
  const int rearCol = static_cast<int>((-0.6 - kOrigin) / kResolution);
  riskGrid[row * kSize + rearCol] = 100.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kSize, kSize, kResolution, kOrigin, kOrigin);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, FootprintPointsDoNotTriggerNearFieldStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_footprint_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.45f, -0.20f, 0.45f, 0.45f, 0.48f, -0.10f, 0.50f, 0.50f};

  auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  EXPECT_FALSE(result.previewPath().empty());
}

TEST(LocalPlanner, NearBodySelfReturnsDoNotTrapThePaddedFootprint) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_near_body_self_return_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.15;
  p.selfFilterPadding = 0.03;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // Returns from the articulated body can land just outside the nominal
  // rectangle. They are robot self points, not four independent obstacles.
  std::vector<float> cloud = {
      0.52f, 0.0f, 0.40f, 0.40f, -0.52f, 0.0f, 0.40f, 0.40f,
      0.0f, 0.32f, 0.40f, 0.40f, 0.0f, -0.32f, 0.40f, 0.40f,
  };
  const auto result =
      planner.planObjective(cloud.data(), static_cast<int>(cloud.size() / 4), 1.0,
                         0.0, 1.0, 3.5, 55.0);

  EXPECT_FALSE(result.previewPath().empty());
  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, TeleopSoftTerrainSlowdownIsNotCommitted) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_teleop_soft_terrain_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.15;
  p.checkObstacle = true;
  p.useTerrainAnalysis = true;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.nearFieldStopDis = 0.55;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kSize = 81;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kSize * kSize, 0.0f);
  for (int row = 0; row < kSize; ++row) {
    for (int col = 0; col < kSize; ++col) {
      const double worldX = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (worldX >= 2.8) {
        riskGrid[row * kSize + col] = 50.0f;
      }
    }
  }
  planner.setTraversabilityGrid(riskGrid.data(), kSize, kSize, kResolution, kOrigin, kOrigin);

  // A distant soft-cost band may affect path preference, but it must not
  // become a speed limit committed for the entire selected path. The final
  // safety controller applies the local slowdown when the robot reaches it.
  std::vector<float> cloud = {1.40f, 0.0f, 0.40f, 0.40f};
  const auto result =
      planner.planObjective(cloud.data(), static_cast<int>(cloud.size() / 4), 1.0,
                         0.0, 1.0, 3.5, 55.0);

  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_EQ(result.hints().slowdownLevel, 0);
}

TEST(LocalPlanner, ObstacleAtConfiguredMinimumHeightIsStillAvoided) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_minimum_obstacle_height_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.useTerrainAnalysis = true;
  p.obstacleHeightThre = 0.10;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 0.10f, 0.10f};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.ready());
  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, ObstacleWallWithoutLateralCorridorStops) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_blocked_lateral_corridor_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 0.76;
  p.vehicleWidth = 0.31;
  p.footprintPadding = 0.10;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.50;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0.0, 0.0, 0.0, 0.0);
  planner.setGoal(2.0, 0.0);

  std::vector<float> cloud;
  for (double y = -1.50; y <= 1.50 + 1e-9; y += 0.05) {
    cloud.insert(cloud.end(), {0.60f, static_cast<float>(y), 0.40f, 0.40f});
  }
  const auto result = planner.planObjective(cloud.data(), static_cast<int>(cloud.size() / 4), 1.0,
                                         0.0, 0.75, 2.0, 55.0);

  EXPECT_FALSE(result.ready());
  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, DoesNotEmitGroupWhoseFootprintSweepHitsObstacle) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_output_group_footprint_collision_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.15;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.5;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // This point is outside the immediate near-field stop sweep, but it lies
  // inside the rectangular footprint swept by the emitted 1 m group path.
  // The legacy correspondence table is intentionally empty, reproducing the
  // case where correspondence scoring says "free" but the exact group sweep rejects it.
  std::vector<float> cloud = {1.4f, 0.4f, 0.4f, 0.4f};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_FALSE(result.previewPath().empty());
  const double halfLength = p.vehicleLength * 0.5 + p.footprintPadding;
  const double halfWidth = p.vehicleWidth * 0.5 + p.footprintPadding;
  const double obstacleX = cloud[0];
  const double obstacleY = cloud[1];
  bool outputSweepBlocked = false;
  double previousX = 0.0;
  double previousY = 0.0;
  for (const auto &point : result.previewPath()) {
    const double dx = point.x - previousX;
    const double dy = point.y - previousY;
    const double length = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.02)));
    for (int step = 1; step <= steps; ++step) {
      const double alpha = static_cast<double>(step) / steps;
      const double centerX = previousX + dx * alpha;
      const double centerY = previousY + dy * alpha;
      const double obstacleDx = obstacleX - centerX;
      const double obstacleDy = obstacleY - centerY;
      const double localX = obstacleDx * c + obstacleDy * s;
      const double localY = -obstacleDx * s + obstacleDy * c;
      if (std::fabs(localX) <= halfLength && std::fabs(localY) <= halfWidth) {
        outputSweepBlocked = true;
      }
    }
    previousX = point.x;
    previousY = point.y;
  }
  EXPECT_FALSE(outputSweepBlocked);
  const auto snapshot = planner.debugSnapshot();
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::any_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                          [](const LocalPlanCandidate &candidate) {
                            return candidate.state == LocalCandidateState::CollisionBlocked;
                          }));
}

TEST(LocalPlanner, NearFieldCandidateUsesCurrentBodyYaw) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_near_field_current_body_yaw_fixture");
  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.12;
  p.checkObstacle = true;
  p.useTerrainAnalysis = true;
  p.useTraversabilityCost = false;
  p.obstacleHeightThre = 0.10;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0.0, 0.0, 0.0, 0.0);
  planner.setGoal(1.1, 1.0);

  // A diagonal path can look clear if the footprint is rotated instantly to
  // the path tangent. The real body still has yaw=0 in this near-field sweep.
  std::vector<float> cloud = {1.05F, -0.10F, 0.40F, 0.40F};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.ready() && result.status() == LocalPlanStatus::NearFieldStop)
      << "a selected candidate must satisfy the same near-field body-yaw sweep";
}

TEST(LocalPlanner, TerminalFootprintChecksObstacleBeyondCenterHorizon) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_terminal_footprint_obstacle_fixture", 1.0);

  LocalPlannerParams p;
  p.vehicleLength = 0.8;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.1;
  p.adjacentRange = 1.0;
  p.minPathRange = 1.0;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // The path center stops at the 1 m planning horizon, while the padded
  // footprint reaches 0.5 m farther. This obstacle must not disappear merely
  // because its center is just outside adjacentRange.
  std::vector<float> cloud = {1.25f, 0.0f, 0.4f, 0.4f};
  const auto result =
      planner.planObjective(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0, 0.0, 1.0, 1.0, 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, SideRearObstacleDoesNotStopDiagonalForwardIntent) {
  auto pathsDir =
      writeMinimalPlannerPaths("nav_kernel_diagonal_forward_ignores_side_rear_obstacle_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.27;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5.0, -1.071);  // atan2(-1.071, 5.0) ~= -12.1 deg

  std::vector<float> cloud = {-0.29f, -0.614f, 0.344f, 0.344f};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  EXPECT_FALSE(result.previewPath().empty());
}

TEST(LocalPlanner, FrontObstacleDoesNotStopReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_reverse_ignores_front_obstacle_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 0.45f, 0.45f};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, PaddingObstacleAllowsReverse) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_padding_obstacle_reverse_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.15;
  p.selfFilterPadding = 0.03;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  // The return is outside the physical/self-filter footprint but inside the
  // padded footprint. Reverse motion increases its clearance at every sample.
  std::vector<float> cloud = {0.60f, 0.0f, 0.45f, 0.45f};
  const auto result = planner.planObjective(
      cloud.data(), 1, 0.0, 180.0, 1.0, 2.0, 5.0);

  EXPECT_FALSE(result.previewPath().empty());
  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_LT(result.previewPath().back().x, 0.0);
}

TEST(LocalPlanner, PaddingObstacleBlocksForward) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_padding_obstacle_forward_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.footprintPadding = 0.15;
  p.selfFilterPadding = 0.03;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.60f, 0.0f, 0.45f, 0.45f};
  const auto result = planner.planObjective(
      cloud.data(), 1, 0.0, 0.0, 1.0, 2.0, 5.0);

  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop || !result.ready());
}

TEST(LocalPlanner, OverheadPointAboveDefaultBodyEnvelopeDoesNotStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_overhead_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTerrainAnalysis = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 2.5f, 2.5f};

  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.status() == LocalPlanStatus::NearFieldStop);
  EXPECT_FALSE(result.previewPath().empty());
}

TEST(LocalPlanner, OverheadHeightLimitIsConfigurable) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_configurable_overhead_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTerrainAnalysis = false;
  p.obstacleHeightMax = 3.0;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 2.5f, 2.5f};
  const auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_TRUE(result.status() == LocalPlanStatus::NearFieldStop);
}

TEST(LocalPlanner, TraversabilityHardCostBlocksPathSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_block_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.ready());
}
TEST(LocalPlanner, TraversabilityOutsideGridFailsClosed) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_out_of_grid_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(3 * 3, 0.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 3, 3, 0.2, -0.3, -0.3);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.ready());
}
TEST(LocalPlanner, TraversabilitySlightlyBelowOriginFailsClosed) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_traversability_floor_fixture", 0.31);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.0;
  p.vehicleWidth = 0.0;
  p.footprintPadding = 0.0;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  std::vector<float> riskGrid(3 * 3, 95.0f);
  riskGrid[1 * 3 + 0] = 0.0f;
  planner.setTraversabilityGrid(riskGrid.data(), 3, 3, 0.2, -0.3, -0.3);

  const auto result = planner.planObjective(nullptr, 0, 0.0, -180.0, 1.0, 1.0, 0.0);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, TraversabilityRiskUsesRobotFootprint) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_footprint_fixture", 1.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int sideRow = static_cast<int>(std::floor((0.35 - kOrigin) / kResolution));
  const int pathCol = static_cast<int>(std::floor((1.0 - kOrigin) / kResolution));
  ASSERT_GE(sideRow, 0);
  ASSERT_LT(sideRow, kRows);
  ASSERT_GE(pathCol, 0);
  ASSERT_LT(pathCol, kCols);
  riskGrid[sideRow * kCols + pathCol] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.5, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, CurrentFootprintBlindCellsDoNotBlockAFreeExit) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_current_footprint_fixture", 1.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.0;
  constexpr double kHalfLength = 0.50;
  constexpr double kHalfWidth = 0.40;
  const double cellHalf = kResolution * 0.5;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (std::abs(x) + cellHalf <= kHalfLength + 1e-9 &&
          std::abs(y) + cellHalf <= kHalfWidth + 1e-9) {
        riskGrid[row * kCols + col] = 100.0f;
      }
    }
  }
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.5, 0.1);

  EXPECT_FALSE(result.previewPath().empty());
}

TEST(LocalPlanner, PaddingOnlyRearHardCellDoesNotBlockForwardEscape) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_padding_escape_fixture", 0.75);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.nearFieldStopDis = 1.0;
  p.vehicleLength = 0.76;
  p.vehicleWidth = 0.31;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 31;
  constexpr int kCols = 31;
  constexpr double kResolution = 0.2;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int rearCol = static_cast<int>(std::floor((-0.5 - kOrigin) / kResolution));
  for (double y : {-0.1, 0.1}) {
    const int row = static_cast<int>(std::floor((y - kOrigin) / kResolution));
    riskGrid[row * kCols + rearCol] = 100.0f;
  }
  // This wall starts beyond the 0.75 m library path's swept footprint.
  const int wallCol = static_cast<int>(std::floor((1.5 - kOrigin) / kResolution));
  for (double y = -1.5; y <= 1.5 + 1e-9; y += kResolution) {
    const int row = static_cast<int>(std::floor((y - kOrigin) / kResolution));
    riskGrid[row * kCols + wallCol] = 100.0f;
  }
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 2.0, 0.1);

  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_FALSE(result.previewPath().empty());
  EXPECT_LE(std::hypot(result.previewPath().back().x, result.previewPath().back().y), 1.0 + 1e-9);
}

TEST(LocalPlanner, PaddingOnlyFrontHardCellStillBlocksStraightEntry) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_padding_entry_fixture", 0.75);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.nearFieldStopDis = 1.0;
  p.vehicleLength = 0.76;
  p.vehicleWidth = 0.31;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 31;
  constexpr int kCols = 31;
  constexpr double kResolution = 0.2;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int frontCol = static_cast<int>(std::floor((0.5 - kOrigin) / kResolution));
  const int rightRow = static_cast<int>(std::floor((-0.3 - kOrigin) / kResolution));
  riskGrid[rightRow * kCols + frontCol] = 100.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.125, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, CurrentFootprintOutsideTraversabilityCoverageFailsClosed) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_initial_coverage_fixture", 1.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOriginX = -0.49;
  constexpr double kOriginY = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOriginX, kOriginY);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.5, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, TraversabilityRiskSweepsBetweenSparsePathPoints) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_sparse_segment_fixture", 2.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 81;
  constexpr int kCols = 81;
  constexpr double kResolution = 0.05;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int gapRow = static_cast<int>(std::floor((0.35 - kOrigin) / kResolution));
  const int gapCol = static_cast<int>(std::floor((1.0 - kOrigin) / kResolution));
  ASSERT_GE(gapRow, 0);
  ASSERT_LT(gapRow, kRows);
  ASSERT_GE(gapCol, 0);
  ASSERT_LT(gapCol, kCols);
  riskGrid[gapRow * kCols + gapCol] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 2.5, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, TraversabilityRiskChecksSparseSegmentPrefixInsideHorizon) {
  auto pathsDir = writePlannerPathsWithStartPoints(
      "nav_kernel_traversability_sparse_horizon_fixture", {0.2, 2.0});

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 81;
  constexpr int kCols = 81;
  constexpr double kResolution = 0.05;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int gapRow = static_cast<int>(std::floor((0.35 - kOrigin) / kResolution));
  const int gapCol = static_cast<int>(std::floor((0.8 - kOrigin) / kResolution));
  ASSERT_GE(gapRow, 0);
  ASSERT_LT(gapRow, kRows);
  ASSERT_GE(gapCol, 0);
  ASSERT_LT(gapCol, kCols);
  riskGrid[gapRow * kCols + gapCol] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.0, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, TraversabilityOutsideRectangleDoesNotUseCircumscribedRadius) {
  auto pathsDir =
      writePlannerPathsWithStartPoint("nav_kernel_traversability_not_circle_fixture", 1.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  constexpr int kRows = 61;
  constexpr int kCols = 61;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int outsideRectRow = static_cast<int>(std::floor((0.55 - kOrigin) / kResolution));
  const int pathCol = static_cast<int>(std::floor((1.0 - kOrigin) / kResolution));
  riskGrid[outsideRectRow * kCols + pathCol] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 0.0, 1.0, 1.5, 0.1);

  EXPECT_FALSE(result.previewPath().empty());
}

TEST(LocalPlanner, TraversabilityFootprintUsesPathTangentYaw) {
  auto pathsDir = writePlannerPathsWithStartPoint("nav_kernel_traversability_yaw_fixture", 1.0);

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.minPathScale = 1.0;
  p.vehicleLength = 0.80;
  p.vehicleWidth = 0.60;
  p.footprintPadding = 0.10;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 4.2);

  constexpr int kRows = 81;
  constexpr int kCols = 81;
  constexpr double kResolution = 0.05;
  constexpr double kOrigin = -2.0;
  std::vector<float> riskGrid(kRows * kCols, 0.0f);
  const int yawSensitiveRow = static_cast<int>(std::floor((1.23 - kOrigin) / kResolution));
  const int yawSensitiveCol = static_cast<int>(std::floor((0.92 - kOrigin) / kResolution));
  riskGrid[yawSensitiveRow * kCols + yawSensitiveCol] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), kRows, kCols, kResolution, kOrigin, kOrigin);

  const auto result = planner.planObjective(nullptr, 0, 0.0, 40.0, 1.0, 1.5, 0.1);

  EXPECT_FALSE(result.ready());
}

TEST(LocalPlanner, UnknownTraversabilityCellFailsClosed) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_unknown_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(5 * 5, std::numeric_limits<float>::quiet_NaN());
  planner.setTraversabilityGrid(riskGrid.data(), 5, 5, 1.0, -2.5, -2.5);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.ready());
}

TEST(RecoveryPlanner, FallsBackToVerifiedRotationWhenTranslationIsBlocked) {
  RecoveryPlannerParams params;
  params.vehicleLength = 0.80;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.27;
  params.checkObstacles = false;
  params.requireTraversability = true;
  params.traversabilityHardCost = 90.0;
  params.searchRadius = 1.2;
  params.latticeResolution = 0.1;
  params.minTranslationDistance = 0.35;
  params.minRotationRad = 0.35;
  params.maxRotationRad = 0.35;
  params.rotationCandidateStepRad = 0.35;
  params.rotationSampleStepRad = 0.05;

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid(kRows * kCols, 95.0f);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (std::abs(x) <= 0.84 && std::abs(y) <= 0.84) {
        grid[row * kCols + col] = 0.0f;
      }
    }
  }

  RecoveryPlannerInput input;
  input.traversabilityGrid = grid.data();
  input.traversabilityRows = kRows;
  input.traversabilityCols = kCols;
  input.traversabilityResolution = kResolution;
  input.traversabilityOriginX = kOrigin;
  input.traversabilityOriginY = kOrigin;

  const RecoveryPlanResult result = RecoveryPlanner(params).plan(input);

  EXPECT_TRUE(result.verified);
  EXPECT_EQ(result.status, PlanStatus::RotationReady);
  EXPECT_EQ(result.action, RecoveryAction::Rotate);
  EXPECT_TRUE(result.pathBody.empty());
  EXPECT_NEAR(std::abs(result.rotationDeltaRad), params.minRotationRad, 1e-9);
  EXPECT_GT(std::abs(result.directCommand.wz), 0.0);
}

TEST(RecoveryPlanner, CurrentFootprintBlindCellsDoNotBlockVerifiedMotion) {
  RecoveryPlannerParams params;
  params.vehicleLength = 0.80;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.10;
  params.checkObstacles = false;
  params.requireTraversability = true;
  params.traversabilityHardCost = 90.0;
  params.edgeSampleResolution = 0.05;
  params.minRotationRad = 0.35;
  params.maxRotationRad = 0.35;
  params.rotationCandidateStepRad = 0.35;
  params.rotationSampleStepRad = 0.05;

  constexpr int kRows = 41;
  constexpr int kCols = 41;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -2.05;
  constexpr double kHalfLength = 0.50;
  constexpr double kHalfWidth = 0.40;
  const double cellHalf = kResolution * 0.5;
  std::vector<float> grid(kRows * kCols, 0.0f);
  for (int row = 0; row < kRows; ++row) {
    const double y = kOrigin + (static_cast<double>(row) + 0.5) * kResolution;
    for (int col = 0; col < kCols; ++col) {
      const double x = kOrigin + (static_cast<double>(col) + 0.5) * kResolution;
      if (std::abs(x) + cellHalf <= kHalfLength + 1e-9 &&
          std::abs(y) + cellHalf <= kHalfWidth + 1e-9) {
        grid[row * kCols + col] = 100.0f;
      }
    }
  }

  RecoveryPlannerInput input;
  input.traversabilityGrid = grid.data();
  input.traversabilityRows = kRows;
  input.traversabilityCols = kCols;
  input.traversabilityResolution = kResolution;
  input.traversabilityOriginX = kOrigin;
  input.traversabilityOriginY = kOrigin;

  const RecoveryPlanner planner(params);
  const std::vector<Vec3> forwardPath{{0.0, 0.0, 0.0}, {0.60, 0.0, 0.0}};
  EXPECT_TRUE(planner.validateBodyPath(input, forwardPath));
  EXPECT_TRUE(planner.validateRotation(input, params.minRotationRad));

  const int outsideCol = static_cast<int>(std::floor((0.70 - kOrigin) / kResolution));
  const int outsideRow = static_cast<int>(std::floor((0.0 - kOrigin) / kResolution));
  ASSERT_GE(outsideCol, 0);
  ASSERT_LT(outsideCol, kCols);
  ASSERT_GE(outsideRow, 0);
  ASSERT_LT(outsideRow, kRows);
  grid[outsideRow * kCols + outsideCol] = 100.0f;
  EXPECT_FALSE(planner.validateBodyPath(input, forwardPath));
}

TEST(RecoveryPlanner, LimitedForwardObservationRejectsRearAndSideTranslations) {
  RecoveryPlannerParams params;
  params.vehicleLength = 0.80;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.27;
  params.checkObstacles = false;
  params.requireTraversability = true;
  params.traversabilityHardCost = 90.0;
  params.searchRadius = 1.2;
  params.latticeResolution = 0.1;
  params.minTranslationDistance = 0.35;
  params.minRotationRad = 0.35;
  params.maxRotationRad = 0.35;
  params.rotationCandidateStepRad = 0.35;
  params.rotationSampleStepRad = 0.05;

  constexpr int kRows = 201;
  constexpr int kCols = 201;
  constexpr double kResolution = 0.02;
  constexpr double kOrigin = -2.01;
  std::vector<float> grid(kRows * kCols, 100.0f);

  const auto setFree = [&](double x, double y) {
    const int col = static_cast<int>(std::floor((x - kOrigin) / kResolution));
    const int row = static_cast<int>(std::floor((y - kOrigin) / kResolution));
    if (row >= 0 && row < kRows && col >= 0 && col < kCols) {
      grid[row * kCols + col] = 0.0f;
    }
  };

  for (double x = -0.68; x <= 0.68; x += kResolution) {
    for (double y = -0.58; y <= 0.58; y += kResolution) {
      setFree(x, y);
    }
  }
  for (double x = 0.40; x <= 1.20; x += kResolution) {
    setFree(x, 0.0);
    setFree(x, 0.10);
    setFree(x, -0.10);
  }

  RecoveryPlannerInput input;
  input.traversabilityGrid = grid.data();
  input.traversabilityRows = kRows;
  input.traversabilityCols = kCols;
  input.traversabilityResolution = kResolution;
  input.traversabilityOriginX = kOrigin;
  input.traversabilityOriginY = kOrigin;
  input.goalDirectionBodyRad = M_PI;

  const RecoveryPlanResult result = RecoveryPlanner(params).plan(input);

  EXPECT_NE(result.status, PlanStatus::TranslationReady);
  EXPECT_NE(result.action, RecoveryAction::Translate);
  EXPECT_TRUE(result.pathBody.empty());
  EXPECT_TRUE(result.status == PlanStatus::RotationReady ||
              result.status == PlanStatus::NoSafeCandidate);
}
TEST(LocalPlanner, DebugSnapshotShowsRepresentativeCandidatesAndSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_candidates_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_TRUE(snapshot.valid);
  ASSERT_FALSE(snapshot.candidates.empty());
  EXPECT_EQ(snapshot.validRotationCount, static_cast<int>(snapshot.candidates.size()));
  EXPECT_DOUBLE_EQ(snapshot.traversabilitySoftCost, p.traversabilitySoftCost);
  EXPECT_DOUBLE_EQ(snapshot.traversabilityHardCost, p.traversabilityHardCost);
  EXPECT_LE(snapshot.candidates.size(), static_cast<std::size_t>(kRotDirs));
  const auto selected_count =
      std::count_if(snapshot.candidates.begin(), snapshot.candidates.end(),
                    [](const LocalPlanCandidate &candidate) { return candidate.selected; });
  ASSERT_EQ(selected_count, 1);
  const auto selected =
      std::find_if(snapshot.candidates.begin(), snapshot.candidates.end(),
                   [](const LocalPlanCandidate &candidate) { return candidate.selected; });
  ASSERT_NE(selected, snapshot.candidates.end());
  EXPECT_EQ(selected->state, LocalCandidateState::Feasible);
  ASSERT_FALSE(selected->path.empty());
  EXPECT_NEAR(selected->path.front().x, result.previewPath().front().x, 1e-9);
  EXPECT_NEAR(selected->path.front().y, result.previewPath().front().y, 1e-9);
}

TEST(LocalPlanner, DebugSnapshotMarksSoftTerrainCostWithoutHardBlocking) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_soft_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 0.02;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_FALSE(result.previewPath().empty());
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::any_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                          [](const LocalPlanCandidate &candidate) {
                            return candidate.state == LocalCandidateState::TerrainCost &&
                                   candidate.terrainRisk >= 40.0 && candidate.terrainRisk < 90.0;
                          }));
}

TEST(LocalPlanner, DebugSnapshotDoesNotClaimTerrainCostWhenWeightIsZero) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_zero_terrain_weight_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 0.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_FALSE(result.previewPath().empty());
  const auto selected =
      std::find_if(snapshot.candidates.begin(), snapshot.candidates.end(),
                   [](const LocalPlanCandidate &candidate) { return candidate.selected; });
  ASSERT_NE(selected, snapshot.candidates.end());
  EXPECT_EQ(selected->state, LocalCandidateState::Feasible);
  EXPECT_EQ(selected->terrainSoftPenalizedPathCount, 0);
}

TEST(LocalPlanner, DebugSnapshotClassifiesSoftTerrainZeroingAsTerrainBlocked) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_soft_zero_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 1.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  EXPECT_FALSE(result.ready());
  EXPECT_TRUE(std::any_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                          [](const LocalPlanCandidate &candidate) {
                            return candidate.state == LocalCandidateState::TerrainBlocked &&
                                   candidate.terrainAllowedPathCount > 0 &&
                                   candidate.heightCostAllowedPathCount > 0 &&
                                   candidate.contributingPathCount == 0;
                          }));
}

TEST(LocalPlanner, DebugSnapshotCollisionGatePrecedesHardTerrainGate) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_collision_before_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.pointPerPathThre = 0;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);
  (void)planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();
  const auto forward = std::find_if(
      snapshot.candidates.begin(), snapshot.candidates.end(),
      [](const LocalPlanCandidate &candidate) { return candidate.rotationIndex == 18; });

  ASSERT_NE(forward, snapshot.candidates.end());
  EXPECT_EQ(forward->state, LocalCandidateState::CollisionBlocked);
  EXPECT_GT(forward->totalPathCount, 0);
  EXPECT_EQ(forward->collisionFreePathCount, 0);
  EXPECT_LT(forward->terrainRisk, 0.0);
}

TEST(LocalPlanner, DebugSnapshotDistinguishesTerrainBlockedCandidates) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  PlannerFixture planner(p);
  ASSERT_TRUE(planner.configure(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  EXPECT_FALSE(result.ready());
  ASSERT_TRUE(snapshot.valid);
  ASSERT_FALSE(snapshot.candidates.empty());
  EXPECT_TRUE(std::any_of(snapshot.candidates.begin(), snapshot.candidates.end(),
                          [](const LocalPlanCandidate &candidate) {
                            return candidate.state == LocalCandidateState::TerrainBlocked &&
                                   candidate.terrainRisk >= 80.0;
                          }));
}
