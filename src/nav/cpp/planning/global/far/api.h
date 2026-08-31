#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(LINGTU_NAV_FAR_C_API_BUILD)
#    define LINGTU_NAV_FAR_API __declspec(dllexport)
#  else
#    define LINGTU_NAV_FAR_API __declspec(dllimport)
#  endif
#else
#  define LINGTU_NAV_FAR_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LINGTU_NAV_FAR_ABI_VERSION 1U

typedef struct LingtuNavFarHandle LingtuNavFarHandle;
typedef uint8_t (*LingtuNavFarCancelCheck)(void* user_data);

enum LingtuNavFarPlanningPhase {
  LINGTU_NAV_FAR_PHASE_NONE = 0,
  LINGTU_NAV_FAR_PHASE_KNOWN_FREE = 1,
  LINGTU_NAV_FAR_PHASE_UNKNOWN_FALLBACK = 2,
  LINGTU_NAV_FAR_PHASE_FAILED = 3,
};

enum LingtuNavFarMapUpdateMode {
  LINGTU_NAV_FAR_UPDATE_NONE = 0,
  LINGTU_NAV_FAR_UPDATE_FULL = 1,
  LINGTU_NAV_FAR_UPDATE_INCREMENTAL = 2,
  LINGTU_NAV_FAR_UPDATE_NOOP = 3,
};

typedef struct LingtuNavFarConfig {
  uint32_t struct_size;
  uint32_t abi_version;
  double robot_radius_m;
  double obstacle_clearance_m;
  double max_visibility_distance_m;
  double unknown_cost_multiplier;
  int32_t corner_separation_cells;
  int32_t snap_search_radius_cells;
  uint64_t max_graph_nodes;
  uint64_t max_visibility_pairs;
  uint64_t max_search_expansions;
  uint8_t allow_unknown_fallback;
  uint8_t simplify_path;
} LingtuNavFarConfig;

typedef struct LingtuNavFarMap {
  uint32_t struct_size;
  uint32_t abi_version;
  int32_t width;
  int32_t height;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
  const char* frame_id;
  uint64_t generation;
  const int8_t* cells;
  uint64_t cell_count;
  const char* map_id;
  int64_t map_content_epoch;
} LingtuNavFarMap;

typedef struct LingtuNavFarPlanRequest {
  uint32_t struct_size;
  uint32_t abi_version;
  double start_x;
  double start_y;
  double start_z;
  double goal_x;
  double goal_y;
  double goal_z;
  uint64_t expected_map_generation;
  int32_t max_iterations;
  double terminal_goal_tolerance_m;
  double terminal_goal_xy_tolerance_m;
  double terminal_goal_z_tolerance_m;
} LingtuNavFarPlanRequest;

typedef struct LingtuNavFarPlanResult {
  uint32_t struct_size;
  uint32_t abi_version;
  uint8_t ok;
  uint8_t reached_goal;
  uint8_t cancelled;
  uint8_t used_unknown_space;
  uint8_t start_snapped;
  uint8_t goal_snapped;
  int32_t planning_phase;
  int32_t map_update_mode;
  uint64_t map_generation;
  uint64_t changed_cells;
  uint64_t graph_nodes;
  uint64_t visibility_pairs;
  uint64_t reusable_edges;
  uint64_t recomputed_edges;
  uint64_t search_expansions;
  uint64_t unknown_cells_traversed;
  double goal_error_m;
  double goal_xy_error_m;
  double goal_z_error_m;
  double elapsed_ms;
} LingtuNavFarPlanResult;

LINGTU_NAV_FAR_API LingtuNavFarHandle* lingtu_nav_far_create(
    const LingtuNavFarConfig* config);

LINGTU_NAV_FAR_API void lingtu_nav_far_destroy(LingtuNavFarHandle* handle);

LINGTU_NAV_FAR_API int32_t lingtu_nav_far_update_map(
    LingtuNavFarHandle* handle,
    const LingtuNavFarMap* map);

// Returns 0 when all outputs fit, 1 when path/reason capacity is insufficient,
// and -1 for an invalid ABI call. A capacity probe executes the plan once and
// caches it; the matching read call retrieves that exact result.
LINGTU_NAV_FAR_API int32_t lingtu_nav_far_plan(
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
    uint64_t* out_failure_reason_size);

LINGTU_NAV_FAR_API int32_t lingtu_nav_far_last_error(
    LingtuNavFarHandle* handle,
    char* output,
    uint64_t capacity,
    uint64_t* out_size);

#ifdef __cplusplus
}
#endif
