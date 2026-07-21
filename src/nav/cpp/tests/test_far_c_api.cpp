#include "far/far_c_api.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace {

LingtuNavFarConfig Config(bool unknown_fallback = false) {
  LingtuNavFarConfig config{};
  config.struct_size = sizeof(config);
  config.abi_version = LINGTU_NAV_FAR_ABI_VERSION;
  config.robot_radius_m = 0.0;
  config.obstacle_clearance_m = 0.0;
  config.max_visibility_distance_m = 100.0;
  config.unknown_cost_multiplier = 6.0;
  config.corner_separation_cells = 1;
  config.snap_search_radius_cells = 4;
  config.max_graph_nodes = 512U;
  config.max_visibility_pairs = 10000U;
  config.max_search_expansions = 10000U;
  config.allow_unknown_fallback = unknown_fallback ? 1U : 0U;
  config.simplify_path = 1U;
  return config;
}

LingtuNavFarMap Map(
    const std::vector<std::int8_t>& cells,
    std::uint64_t generation) {
  LingtuNavFarMap map{};
  map.struct_size = sizeof(map);
  map.abi_version = LINGTU_NAV_FAR_ABI_VERSION;
  map.width = 20;
  map.height = 12;
  map.resolution_m = 0.5;
  map.frame_id = "map";
  map.generation = generation;
  map.cells = cells.data();
  map.cell_count = cells.size();
  return map;
}

LingtuNavFarPlanRequest Request() {
  LingtuNavFarPlanRequest request{};
  request.struct_size = sizeof(request);
  request.abi_version = LINGTU_NAV_FAR_ABI_VERSION;
  request.start_x = 1.25;
  request.start_y = 3.25;
  request.goal_x = 8.75;
  request.goal_y = 3.25;
  request.expected_map_generation = 1U;
  request.max_iterations = 10000;
  request.terminal_goal_tolerance_m = 0.5;
  request.terminal_goal_xy_tolerance_m = 0.6;
  request.terminal_goal_z_tolerance_m = 0.75;
  return request;
}

std::string LastError(LingtuNavFarHandle* handle) {
  std::uint64_t needed = 0U;
  EXPECT_EQ(lingtu_nav_far_last_error(handle, nullptr, 0U, &needed), 1);
  std::vector<char> value(static_cast<std::size_t>(needed), '\0');
  EXPECT_EQ(
      lingtu_nav_far_last_error(handle, value.data(), value.size(), &needed),
      0);
  return value.data();
}

}  // namespace

TEST(FarCApi, ProbeAndReadReturnTheSameCachedPlan) {
  auto config = Config();
  LingtuNavFarHandle* handle = lingtu_nav_far_create(&config);
  ASSERT_NE(handle, nullptr);
  std::vector<std::int8_t> cells(20U * 12U, 0);
  auto map = Map(cells, 1U);
  ASSERT_EQ(lingtu_nav_far_update_map(handle, &map), 0) << LastError(handle);

  auto request = Request();
  LingtuNavFarPlanResult result{};
  result.struct_size = sizeof(result);
  result.abi_version = LINGTU_NAV_FAR_ABI_VERSION;
  std::uint64_t path_points = 0U;
  std::uint64_t reason_size = 0U;
  ASSERT_EQ(
      lingtu_nav_far_plan(
          handle,
          &request,
          nullptr,
          nullptr,
          nullptr,
          0U,
          &path_points,
          &result,
          nullptr,
          0U,
          &reason_size),
      1);
  ASSERT_EQ(path_points, 2U);
  ASSERT_EQ(result.ok, 1U);
  const double first_elapsed = result.elapsed_ms;

  std::vector<double> path(static_cast<std::size_t>(path_points) * 3U, 0.0);
  std::vector<char> reason(static_cast<std::size_t>(reason_size), '\0');
  ASSERT_EQ(
      lingtu_nav_far_plan(
          handle,
          &request,
          nullptr,
          nullptr,
          path.data(),
          path_points,
          &path_points,
          &result,
          reason.data(),
          reason.size(),
          &reason_size),
      0);
  EXPECT_EQ(result.elapsed_ms, first_elapsed);
  EXPECT_EQ(result.planning_phase, LINGTU_NAV_FAR_PHASE_KNOWN_FREE);
  EXPECT_EQ(result.map_update_mode, LINGTU_NAV_FAR_UPDATE_FULL);
  EXPECT_STREQ(reason.data(), "");
  EXPECT_DOUBLE_EQ(path[0], request.start_x);
  EXPECT_DOUBLE_EQ(path[path.size() - 2U], request.goal_y);

  lingtu_nav_far_destroy(handle);
}

TEST(FarCApi, StaleMapFailureIsReportedAndDoesNotReplaceCurrentGeneration) {
  auto config = Config();
  LingtuNavFarHandle* handle = lingtu_nav_far_create(&config);
  ASSERT_NE(handle, nullptr);
  std::vector<std::int8_t> cells(20U * 12U, 0);
  auto current = Map(cells, 2U);
  ASSERT_EQ(lingtu_nav_far_update_map(handle, &current), 0);
  auto stale = Map(cells, 1U);

  EXPECT_EQ(lingtu_nav_far_update_map(handle, &stale), -1);
  EXPECT_NE(LastError(handle).find("stale map generation"), std::string::npos);

  lingtu_nav_far_destroy(handle);
}

TEST(FarCApi, RejectsPartialMapIdentity) {
  auto config = Config();
  LingtuNavFarHandle* handle = lingtu_nav_far_create(&config);
  ASSERT_NE(handle, nullptr);
  std::vector<std::int8_t> cells(20U * 12U, 0);
  auto map = Map(cells, 1U);
  map.map_id = "warehouse";

  EXPECT_EQ(lingtu_nav_far_update_map(handle, &map), -1);
  EXPECT_NE(LastError(handle).find("requires map_id"), std::string::npos);

  lingtu_nav_far_destroy(handle);
}

TEST(FarCApi, RejectsUnsupportedRequestAbi) {
  auto config = Config();
  LingtuNavFarHandle* handle = lingtu_nav_far_create(&config);
  ASSERT_NE(handle, nullptr);
  auto request = Request();
  request.abi_version += 1U;
  std::uint64_t path_points = 0U;
  std::uint64_t reason_size = 0U;

  EXPECT_EQ(
      lingtu_nav_far_plan(
          handle,
          &request,
          nullptr,
          nullptr,
          nullptr,
          0U,
          &path_points,
          nullptr,
          nullptr,
          0U,
          &reason_size),
      -1);

  lingtu_nav_far_destroy(handle);
}
