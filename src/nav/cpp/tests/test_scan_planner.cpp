#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "planning/local/planner.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/search.hpp"
#include "planning/local/scan/spline.hpp"
#include "planning/local/scan/uniform.hpp"

namespace {

nav_kernel::LocalPlannerParams scanParams() {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.vehicleLength = 0.60;
  params.vehicleWidth = 0.45;
  params.footprintPadding = 0.05;
  params.autonomySpeed = 0.40;
  params.maxSpeed = 1.0;
  params.scan.voxelResolution = 0.10;
  params.scan.horizontalRange = 4.0;
  params.scan.routeZTolerance = 0.30;
  params.scan.maxSearchNodes = 20000;
  return params;
}

nav_kernel::LocalPlanInput inputFor(const std::vector<nav_kernel::Vec3> &route, double timestamp,
                                    std::uint64_t generation = 1) {
  nav_kernel::LocalPlanInput input;
  input.vehicle = {route.front(), 0.0};
  input.route = {route.data(), static_cast<int>(route.size()), generation, false};
  input.timestampS = timestamp;
  input.identity.frameEpoch = 1;
  return input;
}

}  // namespace

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
      nav_kernel::local::scan::UniformSpline::parameterize(interval, samples, derivatives);
  const nav_kernel::local::scan::UniformSpline spline(controls, 3, interval);

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
  const nav_kernel::local::scan::UniformSpline spline({{0.0, 0.0, 0.0},
                                                       {1.0, 0.0, 0.0},
                                                       {2.0, 0.0, 0.0},
                                                       {3.0, 0.0, 0.0},
                                                       {4.0, 0.0, 0.0},
                                                       {5.0, 0.0, 0.0}},
                                                      3, 0.10);

  ASSERT_TRUE(spline.valid());
  EXPECT_GT(spline.feasibilityRatio(1.0, 1.0), 9.0);
}

TEST(ScanLocalPlanner, RejectsMissingRouteExplicitly) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());

  const auto result = planner.plan({});

  EXPECT_EQ(result.backend, nav_kernel::LocalPlannerBackend::Scan);
  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_FALSE(result.pathFound);
  EXPECT_EQ(result.reason, "route_invalid");
}

TEST(ScanLocalPlanner, PlansAssistedIntentWithoutCmuFallback) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure("missing-cmu-path-library"));
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto input = inputFor(route, 1.0);

  const auto result = planner.planIntent(input, {0.0, 0.5, 2.0, 45.0});

  EXPECT_EQ(result.backend, nav_kernel::LocalPlannerBackend::Scan);
  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  EXPECT_EQ(result.reason, "scan_intent_ready");
  EXPECT_TRUE(result.pathFound);
  EXPECT_GE(result.path.size(), 2U);
  EXPECT_GE(result.trajectory.size(), 2U);
  double max_speed = 0.0;
  for (const auto &point : result.trajectory) {
    max_speed = std::max(max_speed, std::hypot(point.velocity.x, point.velocity.y));
  }
  EXPECT_LE(max_speed, 0.21);
}

TEST(ScanLocalPlanner, PreservesGlobalRouteElevation) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.8, 0.0, 0.18},
      {1.6, 0.0, 0.42},
      {2.4, 0.0, 0.62},
  };

  const auto result = planner.plan(inputFor(route, 1.0));

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  ASSERT_GE(result.path.size(), 3U);
  ASSERT_GE(result.trajectory.size(), 3U);
  EXPECT_GT(std::max_element(result.path.begin(), result.path.end(),
                             [](const auto &a, const auto &b) { return a.z < b.z; })
                ->z,
            0.45);
  EXPECT_GT(result.trajectory.back().position.z, 0.45);
}

TEST(ScanLocalPlanner, RejectsRouteThatRequiresAnImpossibleSlope) {
  auto params = scanParams();
  params.scan.maxSlope = 0.40;
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.4, 0.0, 0.8},
      {0.8, 0.0, 1.6},
  };

  const auto result = planner.plan(inputFor(route, 1.0));

  EXPECT_NE(result.status, nav_kernel::LocalPlanStatus::Ready);
  EXPECT_FALSE(result.pathFound);
}

TEST(ScanLocalPlanner, KeepsRouteShapeWhenTargetsMatch) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> left{
      {0.0, 0.0, 0.0}, {0.8, 0.7, 0.0}, {1.6, 0.7, 0.0}, {2.4, 0.0, 0.0}};
  const std::vector<nav_kernel::Vec3> right{
      {0.0, 0.0, 0.0}, {0.8, -0.7, 0.0}, {1.6, -0.7, 0.0}, {2.4, 0.0, 0.0}};

  const auto left_result = planner.plan(inputFor(left, 1.0, 1));
  const auto right_result = planner.plan(inputFor(right, 2.0, 2));

  ASSERT_EQ(left_result.status, nav_kernel::LocalPlanStatus::Ready);
  ASSERT_EQ(right_result.status, nav_kernel::LocalPlanStatus::Ready);
  const auto max_left = std::max_element(left_result.path.begin(), left_result.path.end(),
                                         [](const auto &a, const auto &b) { return a.y < b.y; });
  const auto min_right = std::min_element(right_result.path.begin(), right_result.path.end(),
                                          [](const auto &a, const auto &b) { return a.y < b.y; });
  ASSERT_NE(max_left, left_result.path.end());
  ASSERT_NE(min_right, right_result.path.end());
  EXPECT_GT(max_left->y, 0.35);
  EXPECT_LT(min_right->y, -0.35);
}

TEST(ScanLocalPlanner, ReusesSafePrefixAcrossReplans) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 7);
  first_input.kinematics.valid = true;
  first_input.kinematics.linearVelocity = {0.2, 0.0, 0.0};
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status, nav_kernel::LocalPlanStatus::Ready);

  auto second_input = inputFor(route, 1.10, 7);
  second_input.vehicle.position = {0.02, 0.0, 0.0};
  second_input.kinematics = first_input.kinematics;
  const auto second = planner.plan(second_input);

  ASSERT_EQ(second.status, nav_kernel::LocalPlanStatus::Ready);
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
}

TEST(ScanLocalPlanner, RevalidatesTrajectoryWhileMapIdentityIsUnchanged) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 9);
  first_input.identity.obstacleGeneration = 11;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status, nav_kernel::LocalPlanStatus::Ready);

  auto second_input = inputFor(route, 1.05, 9);
  second_input.identity.obstacleGeneration = 11;
  second_input.vehicle.position = {0.01, 0.0, 0.0};
  const auto second = planner.plan(second_input);

  ASSERT_EQ(second.status, nav_kernel::LocalPlanStatus::Ready);
  EXPECT_EQ(second.reason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_EQ(planner.debugSnapshot().expandedNodes, 0);
  EXPECT_GE(second.trajectory.size(), 2U);
}

TEST(ScanLocalPlanner, RevalidatesSafeTrajectoryBeforeSearchingChangedMap) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.3, 0.0}, {1.6, 0.3, 0.0}, {2.4, 0.0, 0.0}};

  auto first_input = inputFor(route, 1.0, 10);
  first_input.identity.obstacleGeneration = 11;
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status, nav_kernel::LocalPlanStatus::Ready);

  auto changed_input = inputFor(route, 1.10, 10);
  changed_input.identity.obstacleGeneration = 12;
  changed_input.vehicle.position = {0.02, 0.0, 0.0};
  const auto reused = planner.plan(changed_input);

  ASSERT_EQ(reused.status, nav_kernel::LocalPlanStatus::Ready) << reused.reason;
  EXPECT_EQ(reused.reason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(planner.debugSnapshot().continuityReused);
  EXPECT_EQ(planner.debugSnapshot().expandedNodes, 0);
  EXPECT_DOUBLE_EQ(planner.debugSnapshot().searchTimeMs, 0.0);
  EXPECT_GE(reused.trajectory.size(), 2U);
}

TEST(ScanLocalPlanner, ReusesSafePrefixThenSearchesFreshAfterExpiry) {
  auto params = scanParams();
  params.useTraversabilityCost = true;
  params.traversabilityHardCost = 90.0;
  params.scan.continuityHorizon = 0.50;
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}, {2.4, 0.0, 0.0}};

  constexpr int rows = 80;
  constexpr int cols = 80;
  constexpr double resolution = 0.10;
  constexpr double origin = -4.0;
  std::vector<float> open(rows * cols, 0.0F);
  auto first_input = inputFor(route, 1.0, 13);
  first_input.identity.obstacleGeneration = 21;
  first_input.identity.traversabilityGeneration = 31;
  first_input.traversability = {open.data(), rows, cols, resolution, origin, origin};
  const auto first = planner.plan(first_input);
  ASSERT_EQ(first.status, nav_kernel::LocalPlanStatus::Ready);

  std::vector<float> closed(rows * cols, 100.0F);
  const auto cell = [&](double coordinate) {
    return static_cast<int>(std::floor((coordinate - origin) / resolution));
  };
  for (int row = cell(-1.0); row <= cell(1.0); ++row) {
    for (int col = cell(-1.0); col <= cell(0.60); ++col) {
      closed[row * cols + col] = 0.0F;
    }
  }
  auto changed_input = inputFor(route, 1.10, 13);
  changed_input.identity.obstacleGeneration = 22;
  changed_input.identity.traversabilityGeneration = 32;
  changed_input.traversability = {closed.data(), rows, cols, resolution, origin, origin};

  const auto reused = planner.plan(changed_input);

  ASSERT_EQ(reused.status, nav_kernel::LocalPlanStatus::Ready) << reused.reason;
  EXPECT_EQ(reused.reason, "scan_safe_trajectory_reused");
  EXPECT_TRUE(reused.pathFound);
  EXPECT_GE(reused.trajectory.size(), 2U);

  changed_input.timestampS = 1.60;
  const auto expired = planner.plan(changed_input);
  EXPECT_EQ(expired.status, nav_kernel::LocalPlanStatus::Ready) << expired.reason;
  EXPECT_EQ(expired.reason, "scan_boundary_fallback_ready");
  EXPECT_FALSE(planner.debugSnapshot().continuityReused);
}

TEST(ScanLocalPlanner, SearchesAroundThreeDimensionalObstacleBand) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  const auto lateral =
      std::max_element(result.path.begin(), result.path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result.path.end());
  EXPECT_GT(std::abs(lateral->y), 0.75);
}

TEST(ScanLocalPlanner, UsesCompleteMapdCollisionLayerWithoutLegacyCloud) {
  auto params = scanParams();
  params.scan.maxSearchNodes = 100000;
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
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
  input.collision = {
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

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  const auto lateral =
      std::max_element(result.path.begin(), result.path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result.path.end());
  EXPECT_GT(std::abs(lateral->y), 0.75);
}

TEST(ScanLocalPlanner, IgnoresMapdGroundBelowBodyCollisionEnvelope) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
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
  input.obstacles = {
      legacy_obstacles.data(),
      static_cast<int>(legacy_obstacles.size() / 4U),
  };
  input.collision = {
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

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  ASSERT_TRUE(result.pathFound);
  EXPECT_EQ(result.reason, "scan_route_ready");
  const auto lateral =
      std::max_element(result.path.begin(), result.path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result.path.end());
  EXPECT_LT(std::abs(lateral->y), 0.20)
      << "ground below the body collision envelope must not force a detour";
}

TEST(ScanLocalPlanner, RejectsIncompleteMapdCollisionLayer) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 1.0, false, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(result.reason, "collision_map_incomplete");
  EXPECT_FALSE(result.pathFound);
}

TEST(ScanLocalPlanner, RejectsMapdCollisionOlderThanDefaultControlWindow) {
  const auto params = scanParams();
  EXPECT_DOUBLE_EQ(params.scan.collisionMaxAge, 0.50);
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.51);
  input.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 1.0, true, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(result.reason, "collision_map_stale");
  EXPECT_FALSE(result.pathFound);
}

TEST(ScanLocalPlanner, RejectsCollisionWithoutReceiverClockTimestamp) {
  nav_kernel::local::Planner planner(scanParams());
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  auto input = inputFor(route, 1.0);
  input.collision = {
      nullptr, 0, 0.10, {-5.0, -5.0, -2.0}, {5.0, 5.0, 2.0}, 3U, 9U, 12U, 1.0, 0.0, true, true,
  };

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_EQ(result.reason, "collision_map_invalid");
  EXPECT_FALSE(result.pathFound);
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
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};

  const auto result = planner.plan(input);

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  EXPECT_EQ(result.reason, "scan_boundary_fallback_ready");
  ASSERT_TRUE(result.pathFound);
  const auto highest = std::max_element(result.path.begin(), result.path.end(),
                                        [](const auto &a, const auto &b) { return a.z < b.z; });
  ASSERT_NE(highest, result.path.end());
  EXPECT_LT(std::abs(highest->z), 0.15) << "projected A* must not escape the wall through Z";
  EXPECT_GT(nav_kernel::distance3D(result.path.back(), route.back()), 0.5);
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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);

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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);

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
  params.scan.obstacleClearance = 0.0;
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  std::vector<float> obstacles{0.45F, 0.0F, 0.0F, 0.8F};
  auto input = inputFor(route, 1.0);
  input.obstacles = {obstacles.data(), 1};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());
  const auto center = grid.index(input.vehicle.position);

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
  params.scan.obstacleClearance = 0.0;
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
  input.traversability = {traversability.data(), rows, cols, resolution, origin, origin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto path_center = grid.index({1.0, 0.0, 0.0});
  ASSERT_FALSE(grid.free(path_center, 0.0))
      << "SCAN must reject a centerline whose rectangular body footprint reaches hard terrain";
}

TEST(ScanLocalPlanner, SearchesThroughObservedCorridorAroundMapdObstacle) {
  auto params = scanParams();
  params.useTraversabilityCost = true;
  params.traversabilityHardCost = 90.0;
  params.vehicleLength = 1.00;
  params.vehicleWidth = 0.60;
  params.footprintPadding = 0.15;
  params.scan.voxelResolution = 0.15;
  params.scan.obstacleClearance = 0.12;
  params.scan.maxSearchNodes = 12000;
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());

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
  input.vehicle.position = {0.0, 0.0, 0.48};
  input.traversability = {traversability.data(), rows, cols, resolution, origin, origin};
  input.obstacles = {legacy_obstacles.data(), static_cast<int>(legacy_obstacles.size() / 4U)};
  input.collision = {
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

  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready)
      << result.reason << " expanded=" << debug.expandedNodes;
  ASSERT_TRUE(result.pathFound);
  const auto lateral =
      std::max_element(result.path.begin(), result.path.end(),
                       [](const auto &a, const auto &b) { return std::abs(a.y) < std::abs(b.y); });
  ASSERT_NE(lateral, result.path.end());
  EXPECT_GT(std::abs(lateral->y), 0.70);
  EXPECT_LT(std::abs(lateral->y), 1.50);
  EXPECT_EQ(result.reason, "scan_search_ready");
  // Full 3D occupancy retains every inflated voxel layer.
  EXPECT_GE(debug.occupiedCellCount, 600);
  EXPECT_EQ(debug.legacyObstaclePointCount, 0)
      << "complete Mapd collision data must replace, not duplicate, the raw-cloud fallback";
}

TEST(ScanLocalPlanner, NearBodyObstacleIsNotDiscardedByPlannerGrid) {
  auto params = scanParams();
  params.useTraversabilityCost = false;
  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
  const std::vector<nav_kernel::Vec3> route{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}, {1.6, 0.0, 0.0}};
  std::vector<float> obstacles{0.25F, 0.0F, 0.0F, 0.8F};
  auto input = inputFor(route, 1.0);
  input.obstacles = {obstacles.data(), 1};

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_TRUE(result.nearFieldStop);
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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());
  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);
  ASSERT_TRUE(searched.found()) << searched.reason;

  const auto spline = nav_kernel::local::scan::buildSpline(grid, searched.path, input, params);

  ASSERT_TRUE(spline.valid());
  EXPECT_TRUE(spline.optimizerUsed);
  EXPECT_TRUE(spline.zGradientSuppressed);
  EXPECT_GT(spline.optimizerEvaluations, 1)
      << "status=" << spline.optimizerStatus << " initial=" << spline.optimizerInitialCost
      << " final=" << spline.optimizerFinalCost;
  EXPECT_FALSE(spline.fallback)
      << "the projected A* rebound guide should resolve this collision band"
      << " status=" << spline.optimizerStatus << " evaluations=" << spline.optimizerEvaluations
      << " initial=" << spline.optimizerInitialCost << " final=" << spline.optimizerFinalCost;
  for (const auto &point : spline.path) {
    EXPECT_NEAR(point.z, 0.25 * point.x, 0.08);
  }
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
  input.obstacles = {obstacles.data(), static_cast<int>(obstacles.size() / 4)};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid());

  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);

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

  nav_kernel::local::Planner planner(params);
  ASSERT_TRUE(planner.configure());
  const auto result = planner.plan(input);
  ASSERT_EQ(result.status, nav_kernel::LocalPlanStatus::Ready) << result.reason;
  EXPECT_EQ(result.reason, "scan_boundary_fallback_ready");
  ASSERT_TRUE(result.pathFound);
  for (const auto &point : result.path) {
    EXPECT_TRUE(grid.contains(grid.index(point)));
  }
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
  input.traversability = {risk.data(), kRows, kCols, kResolution, kOrigin, kOrigin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);

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
  input.traversability = {risk.data(), kRows, kCols, kResolution, kOrigin, kOrigin};
  const nav_kernel::local::scan::Grid grid(params, input);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_TRUE(grid.free(input.vehicle.position, input.vehicle.yaw));
  EXPECT_FALSE(grid.free(grid.index(input.vehicle.position), input.vehicle.yaw))
      << "this fixture must expose the voxel-centre offset regression";
  const auto searched =
      nav_kernel::local::scan::search(grid, input.vehicle.position, input.vehicle.yaw, params);
  EXPECT_NE(searched.reason, "start_traversability_blocked")
      << "the occupied current pose is the search seed; incremental poses remain checked";
}
