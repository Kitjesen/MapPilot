#include "planning/global/far/api.h"

#include "planning/global/far/planner.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using lingtu::nav::plan::GlobalPlanRequest;
using lingtu::nav::plan::GlobalPlanResult;
using lingtu::nav::plan::far_planner::FarDiagnostics;
using lingtu::nav::plan::far_planner::FarGridMap;
using lingtu::nav::plan::far_planner::FarPlanner;
using lingtu::nav::plan::far_planner::FarPlannerConfig;

namespace {

bool EmptyOrNull(const char* value) {
  return value == nullptr || *value == '\0';
}

FarPlannerConfig ConvertConfig(const LingtuNavFarConfig& value) {
  FarPlannerConfig config;
  config.robot_radius_m = value.robot_radius_m;
  config.obstacle_clearance_m = value.obstacle_clearance_m;
  config.max_visibility_distance_m = value.max_visibility_distance_m;
  config.unknown_cost_multiplier = value.unknown_cost_multiplier;
  config.corner_separation_cells = value.corner_separation_cells;
  config.snap_search_radius_cells = value.snap_search_radius_cells;
  if (value.max_graph_nodes > std::numeric_limits<std::size_t>::max() ||
      value.max_visibility_pairs > std::numeric_limits<std::size_t>::max() ||
      value.max_search_expansions > std::numeric_limits<std::size_t>::max()) {
    throw std::length_error("FAR C ABI size setting exceeds this process address space");
  }
  config.max_graph_nodes = static_cast<std::size_t>(value.max_graph_nodes);
  config.max_visibility_pairs = static_cast<std::size_t>(value.max_visibility_pairs);
  config.max_search_expansions = static_cast<std::size_t>(value.max_search_expansions);
  config.allow_unknown_fallback = value.allow_unknown_fallback != 0U;
  config.simplify_path = value.simplify_path != 0U;
  return config;
}

struct RequestKey {
  double values[6]{};
  std::uint64_t generation{0U};
  std::int32_t max_iterations{0};
  double tolerances[3]{};

  bool operator==(const RequestKey& other) const {
    for (std::size_t index = 0U; index < 6U; ++index) {
      if (values[index] != other.values[index]) return false;
    }
    for (std::size_t index = 0U; index < 3U; ++index) {
      if (tolerances[index] != other.tolerances[index]) return false;
    }
    return generation == other.generation && max_iterations == other.max_iterations;
  }
};

RequestKey MakeKey(const LingtuNavFarPlanRequest& request) {
  RequestKey key;
  key.values[0] = request.start_x;
  key.values[1] = request.start_y;
  key.values[2] = request.start_z;
  key.values[3] = request.goal_x;
  key.values[4] = request.goal_y;
  key.values[5] = request.goal_z;
  key.generation = request.expected_map_generation;
  key.max_iterations = request.max_iterations;
  key.tolerances[0] = request.terminal_goal_tolerance_m;
  key.tolerances[1] = request.terminal_goal_xy_tolerance_m;
  key.tolerances[2] = request.terminal_goal_z_tolerance_m;
  return key;
}

GlobalPlanRequest ConvertRequest(const LingtuNavFarPlanRequest& value) {
  GlobalPlanRequest request;
  request.start = {value.start_x, value.start_y, value.start_z};
  request.goal = {value.goal_x, value.goal_y, value.goal_z};
  request.map_generation = value.expected_map_generation;
  request.options.max_iterations = value.max_iterations;
  request.options.terminal_goal_tolerance_m = value.terminal_goal_tolerance_m;
  request.options.terminal_goal_xy_tolerance_m = value.terminal_goal_xy_tolerance_m;
  request.options.terminal_goal_z_tolerance_m = value.terminal_goal_z_tolerance_m;
  return request;
}

int PlanningPhase(const FarDiagnostics& diagnostics) {
  if (diagnostics.planning_phase == "known_free") {
    return LINGTU_NAV_FAR_PHASE_KNOWN_FREE;
  }
  if (diagnostics.planning_phase == "unknown_fallback") {
    return LINGTU_NAV_FAR_PHASE_UNKNOWN_FALLBACK;
  }
  if (!diagnostics.failure_reason.empty()) {
    return LINGTU_NAV_FAR_PHASE_FAILED;
  }
  return LINGTU_NAV_FAR_PHASE_NONE;
}

int UpdateMode(const FarDiagnostics& diagnostics) {
  if (diagnostics.map_update_mode == "full") return LINGTU_NAV_FAR_UPDATE_FULL;
  if (diagnostics.map_update_mode == "incremental") return LINGTU_NAV_FAR_UPDATE_INCREMENTAL;
  if (diagnostics.map_update_mode == "noop_same_generation") return LINGTU_NAV_FAR_UPDATE_NOOP;
  return LINGTU_NAV_FAR_UPDATE_NONE;
}

struct CachedPlan {
  RequestKey request;
  std::uint64_t map_generation{0U};
  GlobalPlanResult result;
  FarDiagnostics diagnostics;
};

}  // namespace

struct LingtuNavFarHandle {
  explicit LingtuNavFarHandle(FarPlannerConfig config)
      : planner(std::move(config)) {}

  FarPlanner planner;
  std::mutex mutex;
  std::string last_error;
  std::optional<CachedPlan> cached_plan;
};

extern "C" {

LingtuNavFarHandle* lingtu_nav_far_create(const LingtuNavFarConfig* config) {
  if (config == nullptr || config->struct_size < sizeof(LingtuNavFarConfig) ||
      config->abi_version != LINGTU_NAV_FAR_ABI_VERSION) {
    return nullptr;
  }
  try {
    return new LingtuNavFarHandle(ConvertConfig(*config));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_nav_far_destroy(LingtuNavFarHandle* handle) {
  delete handle;
}

int32_t lingtu_nav_far_update_map(
    LingtuNavFarHandle* handle,
    const LingtuNavFarMap* map) {
  if (handle == nullptr || map == nullptr ||
      map->struct_size < sizeof(LingtuNavFarMap) ||
      map->abi_version != LINGTU_NAV_FAR_ABI_VERSION ||
      EmptyOrNull(map->frame_id) || map->cells == nullptr) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(handle->mutex);
  try {
    FarGridMap value;
    value.width = map->width;
    value.height = map->height;
    value.resolution_m = map->resolution_m;
    value.origin_x_m = map->origin_x_m;
    value.origin_y_m = map->origin_y_m;
    value.frame_id = map->frame_id;
    value.generation = map->generation;
    if (map->cell_count > std::numeric_limits<std::size_t>::max()) {
      throw std::length_error("FAR map cell count exceeds this process address space");
    }
    value.cells.assign(map->cells, map->cells + static_cast<std::size_t>(map->cell_count));
    const bool has_map_id = !EmptyOrNull(map->map_id);
    const bool has_map_content_epoch = map->map_content_epoch > 0;
    const bool has_any_identity = has_map_id || map->map_content_epoch != 0;
    if (has_any_identity && !(has_map_id && has_map_content_epoch)) {
      throw std::invalid_argument("FAR map identity requires map_id and positive map_content_epoch together");
    }
    if (has_any_identity) {
      value.identity.map_id = map->map_id;
      value.identity.content_epoch = map->map_content_epoch;
      value.identity.frame_id = value.frame_id;
    }
    handle->planner.UpdateMap(std::move(value));
    handle->cached_plan.reset();
    handle->last_error.clear();
    return 0;
  } catch (const std::exception& exc) {
    handle->last_error = exc.what();
    return -1;
  }
}

int32_t lingtu_nav_far_plan(
    LingtuNavFarHandle* handle,
    const LingtuNavFarPlanRequest* request,
    LingtuNavFarCancelCheck cancel_check,
    void* cancel_user_data,
    double* path_xyz,
    uint64_t path_capacity_points,
    uint64_t* out_path_points,
    LingtuNavFarPlanResult* result,
    char* failure_reason,
    uint64_t failure_reason_capacity,
    uint64_t* out_failure_reason_size) {
  if (handle == nullptr || request == nullptr || out_path_points == nullptr ||
      out_failure_reason_size == nullptr ||
      request->struct_size < sizeof(LingtuNavFarPlanRequest) ||
      request->abi_version != LINGTU_NAV_FAR_ABI_VERSION ||
      (result != nullptr &&
       (result->struct_size < sizeof(LingtuNavFarPlanResult) ||
        result->abi_version != LINGTU_NAV_FAR_ABI_VERSION))) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(handle->mutex);
  try {
    const auto key = MakeKey(*request);
    if (!handle->cached_plan || !(handle->cached_plan->request == key) ||
        handle->cached_plan->map_generation != handle->planner.Map().generation) {
      const auto cancel = cancel_check == nullptr
          ? lingtu::nav::plan::GlobalPlanCancelCheck{}
          : lingtu::nav::plan::GlobalPlanCancelCheck(
                [cancel_check, cancel_user_data]() {
                  return cancel_check(cancel_user_data) != 0U;
                });
      CachedPlan cached;
      cached.request = key;
      cached.map_generation = handle->planner.Map().generation;
      cached.result = handle->planner.Plan(ConvertRequest(*request), cancel);
      cached.diagnostics = handle->planner.LastDiagnostics();
      handle->cached_plan = std::move(cached);
    }
    const auto& cached = *handle->cached_plan;
    const auto point_count = static_cast<std::uint64_t>(cached.result.path.size());
    const auto reason_size = static_cast<std::uint64_t>(
        cached.result.failure_reason.size() + 1U);
    *out_path_points = point_count;
    *out_failure_reason_size = reason_size;

    if (result != nullptr) {
      result->ok = cached.result.ok ? 1U : 0U;
      result->reached_goal = cached.result.reached_goal ? 1U : 0U;
      result->cancelled = cached.result.cancelled ? 1U : 0U;
      result->used_unknown_space = cached.diagnostics.used_unknown_space ? 1U : 0U;
      result->start_snapped = cached.diagnostics.start_snapped ? 1U : 0U;
      result->goal_snapped = cached.diagnostics.goal_snapped ? 1U : 0U;
      result->planning_phase = PlanningPhase(cached.diagnostics);
      result->map_update_mode = UpdateMode(cached.diagnostics);
      result->map_generation = cached.result.map_generation;
      result->changed_cells = cached.diagnostics.changed_cells;
      result->graph_nodes = cached.diagnostics.graph_nodes;
      result->visibility_pairs = cached.diagnostics.visibility_pairs;
      result->reusable_edges = cached.diagnostics.reusable_edges;
      result->recomputed_edges = cached.diagnostics.recomputed_edges;
      result->search_expansions = cached.diagnostics.search_expansions;
      result->unknown_cells_traversed = cached.diagnostics.unknown_cells_traversed;
      result->goal_error_m = cached.result.goal_error_m;
      result->goal_xy_error_m = cached.result.goal_xy_error_m;
      result->goal_z_error_m = cached.result.goal_z_error_m;
      result->elapsed_ms = cached.result.elapsed_ms;
    }

    const bool path_fits = point_count == 0U ||
        (path_xyz != nullptr && path_capacity_points >= point_count);
    const bool reason_fits = failure_reason != nullptr &&
        failure_reason_capacity >= reason_size;
    if (!path_fits || !reason_fits) {
      return 1;
    }
    for (std::size_t index = 0U; index < cached.result.path.size(); ++index) {
      path_xyz[index * 3U] = cached.result.path[index].x;
      path_xyz[index * 3U + 1U] = cached.result.path[index].y;
      path_xyz[index * 3U + 2U] = cached.result.path[index].z;
    }
    std::memcpy(
        failure_reason,
        cached.result.failure_reason.c_str(),
        static_cast<std::size_t>(reason_size));
    handle->last_error.clear();
    return 0;
  } catch (const std::exception& exc) {
    handle->last_error = exc.what();
    return -1;
  }
}

int32_t lingtu_nav_far_last_error(
    LingtuNavFarHandle* handle,
    char* output,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || out_size == nullptr) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(handle->mutex);
  const auto required = static_cast<std::uint64_t>(handle->last_error.size() + 1U);
  *out_size = required;
  if (output == nullptr || capacity < required) {
    return 1;
  }
  std::memcpy(output, handle->last_error.c_str(), static_cast<std::size_t>(required));
  return 0;
}

}  // extern "C"
