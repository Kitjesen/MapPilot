#include "far/far_planner.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace {

using lingtu::nav::plan::GlobalPlanRequest;
using lingtu::nav::plan::far::FarGridMap;
using lingtu::nav::plan::far::FarPlanner;
using lingtu::nav::plan::far::FarPlannerConfig;
using lingtu::nav::plan::far::kFreeCell;
using lingtu::nav::plan::far::kOccupiedCell;
using lingtu::nav::plan::far::kUnknownCell;

FarGridMap OpenMap(std::uint64_t generation = 1U) {
  FarGridMap map;
  map.width = 30;
  map.height = 20;
  map.resolution_m = 0.5;
  map.origin_x_m = 0.0;
  map.origin_y_m = 0.0;
  map.frame_id = "map";
  map.generation = generation;
  map.cells.assign(map.CellCount(), kFreeCell);
  return map;
}

FarPlannerConfig TestConfig(bool unknown_fallback = false) {
  FarPlannerConfig config;
  config.robot_radius_m = 0.0;
  config.obstacle_clearance_m = 0.0;
  config.max_visibility_distance_m = 100.0;
  config.allow_unknown_fallback = unknown_fallback;
  config.snap_search_radius_cells = 4;
  config.max_graph_nodes = 512U;
  config.max_visibility_pairs = 10000U;
  return config;
}

GlobalPlanRequest Request() {
  GlobalPlanRequest request;
  request.start = {1.25, 5.25, 0.0};
  request.goal = {13.75, 5.25, 0.0};
  request.options.max_iterations = 10000;
  return request;
}

void AddVerticalWall(FarGridMap* map, int x, int first_y, int last_y) {
  for (int y = first_y; y <= last_y; ++y) {
    map->cells[map->Index(x, y)] = kOccupiedCell;
  }
}

}  // namespace

TEST(FarPlanner, PlansDirectlyAcrossKnownFreeSpace) {
  FarPlanner planner(TestConfig());
  planner.UpdateMap(OpenMap());

  auto request = Request();
  request.map_generation = 1U;
  const auto result = planner.Plan(request);

  ASSERT_TRUE(result.ok) << result.failure_reason;
  EXPECT_TRUE(result.reached_goal);
  ASSERT_EQ(result.path.size(), 2U);
  EXPECT_EQ(result.map_generation, 1U);
  EXPECT_EQ(planner.LastDiagnostics().planning_phase, "known_free");
  EXPECT_FALSE(planner.LastDiagnostics().used_unknown_space);
}

TEST(FarPlanner, RejectsTemporaryOverlayUntilBackendCanHonorIt) {
  FarPlanner planner(TestConfig());
  planner.UpdateMap(OpenMap());

  auto request = Request();
  request.temporary_overlay.revision = 3U;
  request.temporary_overlay.frame_epoch = 5U;
  request.temporary_overlay.obstacle_generation = 7U;
  request.temporary_overlay.blocked_regions.push_back(
      {{7.5, 5.25, 0.0}, 1.0, -0.5, 0.5});

  const auto result = planner.Plan(request);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure_reason, "far_temporary_overlay_unsupported");
  EXPECT_EQ(result.overlay_revision, request.temporary_overlay.revision);
  EXPECT_EQ(result.overlay_frame_epoch, request.temporary_overlay.frame_epoch);
  EXPECT_EQ(
      result.overlay_obstacle_generation,
      request.temporary_overlay.obstacle_generation);
  EXPECT_EQ(
      result.overlay_traversability_generation,
      request.temporary_overlay.traversability_generation);
}

TEST(FarPlanner, RoutesAroundOccupiedWallWithVisibilityCorners) {
  auto map = OpenMap();
  AddVerticalWall(&map, 15, 3, 16);
  FarPlanner planner(TestConfig());
  planner.UpdateMap(std::move(map));

  const auto result = planner.Plan(Request());

  ASSERT_TRUE(result.ok) << result.failure_reason;
  EXPECT_TRUE(result.reached_goal);
  EXPECT_GE(result.path.size(), 3U);
  EXPECT_GT(planner.LastDiagnostics().graph_nodes, 0U);
  EXPECT_GT(planner.LastDiagnostics().visibility_pairs, 0U);
}

TEST(FarPlanner, UnknownSpaceIsOnlyUsedByExplicitFallback) {
  auto map = OpenMap();
  for (int y = 0; y < map.height; ++y) {
    map.cells[map.Index(15, y)] = kUnknownCell;
  }
  FarPlanner strict(TestConfig(false));
  strict.UpdateMap(map);
  const auto strict_result = strict.Plan(Request());
  EXPECT_FALSE(strict_result.ok);
  EXPECT_EQ(strict_result.failure_reason, "far_no_path");
  EXPECT_EQ(strict.LastDiagnostics().planning_phase, "known_free_failed");

  FarPlanner fallback(TestConfig(true));
  fallback.UpdateMap(std::move(map));
  const auto fallback_result = fallback.Plan(Request());
  ASSERT_TRUE(fallback_result.ok) << fallback_result.failure_reason;
  EXPECT_TRUE(fallback_result.reached_goal);
  EXPECT_EQ(fallback.LastDiagnostics().planning_phase, "unknown_fallback");
  EXPECT_TRUE(fallback.LastDiagnostics().used_unknown_space);
  EXPECT_GT(fallback.LastDiagnostics().unknown_cells_traversed, 0U);
}

TEST(FarPlanner, FullyOccupiedBarrierFailsClosed) {
  auto map = OpenMap();
  AddVerticalWall(&map, 15, 0, map.height - 1);
  FarPlanner planner(TestConfig(true));
  planner.UpdateMap(std::move(map));

  const auto result = planner.Plan(Request());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure_reason, "far_no_path");
  EXPECT_FALSE(result.reached_goal);
}

TEST(FarPlanner, RejectsStaleAndSameGenerationDrift) {
  FarPlanner planner(TestConfig());
  planner.UpdateMap(OpenMap(5U));

  EXPECT_THROW(planner.UpdateMap(OpenMap(4U)), std::invalid_argument);
  auto drift = OpenMap(5U);
  drift.cells[drift.Index(4, 4)] = kUnknownCell;
  EXPECT_THROW(planner.UpdateMap(std::move(drift)), std::invalid_argument);

  auto request = Request();
  request.map_generation = 4U;
  const auto result = planner.Plan(request);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure_reason, "far_stale_map_generation");
}

TEST(FarPlanner, ReusesUnaffectedVisibilityEdgesOnIncrementalUpdate) {
  auto initial = OpenMap(10U);
  AddVerticalWall(&initial, 15, 3, 16);
  FarPlanner planner(TestConfig());
  planner.UpdateMap(initial);
  ASSERT_GT(planner.LastDiagnostics().visibility_pairs, 0U);

  auto updated = initial;
  updated.generation = 11U;
  updated.cells[updated.Index(1, 1)] = kUnknownCell;
  planner.UpdateMap(std::move(updated));

  EXPECT_EQ(planner.LastDiagnostics().map_update_mode, "incremental");
  EXPECT_EQ(planner.LastDiagnostics().changed_cells, 1U);
  EXPECT_GT(planner.LastDiagnostics().reusable_edges, 0U);
  EXPECT_LT(
      planner.LastDiagnostics().recomputed_edges,
      planner.LastDiagnostics().visibility_pairs);
}

TEST(FarPlanner, CancellationStopsSearchWithoutReturningAPath) {
  auto map = OpenMap();
  AddVerticalWall(&map, 15, 3, 16);
  FarPlanner planner(TestConfig());
  planner.UpdateMap(std::move(map));

  const auto result = planner.Plan(Request(), []() { return true; });

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.cancelled);
  EXPECT_EQ(result.failure_reason, "far_cancelled");
  EXPECT_TRUE(planner.LastDiagnostics().cancelled);
}

TEST(FarPlanner, ValidatesTrinaryMapContract) {
  auto invalid = OpenMap();
  invalid.cells[0] = 42;
  FarPlanner planner(TestConfig());
  EXPECT_THROW(planner.UpdateMap(std::move(invalid)), std::invalid_argument);
  EXPECT_FALSE(planner.HasMap());
}
