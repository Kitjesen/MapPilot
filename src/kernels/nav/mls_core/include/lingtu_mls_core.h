#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LtMlsCell {
  int32_t x;
  int32_t y;
  int32_t z;
} LtMlsCell;

typedef struct LtMlsConfig {
  int32_t clearance_cells;
  uint32_t closing_passes;
  // Exact closing radius in map cells. Zero keeps the legacy integer radius.
  float closing_radius_cells;
} LtMlsConfig;

typedef struct LtMlsGraphConfig {
  int32_t node_spacing_cells;
  int32_t max_link_cells;
  int32_t wall_clearance_cells;
  int32_t wall_buffer_cells;
  float wall_buffer_weight;
  float step_penalty_weight;
  float max_slope;
  int32_t max_step_cells;
  int32_t horizontal_hop_cells;
} LtMlsGraphConfig;

typedef struct LtMlsGraphNode {
  uint32_t id;
  int32_t x;
  int32_t y;
  int32_t z;
} LtMlsGraphNode;

typedef struct LtMlsGraphEdge {
  // Stable for the lifetime of the endpoint node IDs. The smaller endpoint
  // occupies the high 32 bits, so an undirected edge has one identity.
  uint64_t id;
  uint32_t from;
  uint32_t to;
  float cost;
  uint64_t corridor_offset;
  uint64_t corridor_len;
} LtMlsGraphEdge;

typedef struct LtMlsGraphOutput {
  LtMlsGraphNode* nodes_ptr;
  uint64_t nodes_len;
  uint64_t nodes_cap;
  LtMlsGraphEdge* edges_ptr;
  uint64_t edges_len;
  uint64_t edges_cap;
  LtMlsCell* corridor_ptr;
  uint64_t corridor_len;
  uint64_t corridor_cap;
} LtMlsGraphOutput;

typedef struct LtMlsGraphSummary {
  uint64_t node_count;
  uint64_t edge_count;
  uint64_t component_count;
  uint64_t corridor_cells;
  uint64_t max_corridor_cells;
} LtMlsGraphSummary;

typedef struct LtMlsArtifact LtMlsArtifact;

typedef struct LtMlsArtifactSummary {
  uint64_t revision;
  uint64_t observed_surface_count;
  uint64_t inferred_surface_count;
  uint64_t node_count;
  uint64_t edge_count;
  uint64_t component_count;
} LtMlsArtifactSummary;

typedef struct LtMlsRegion {
  int32_t min_x;
  int32_t max_x;
  int32_t min_y;
  int32_t max_y;
  int32_t min_z;
  int32_t max_z;
} LtMlsRegion;

typedef struct LtMlsUpdateReport {
  uint64_t revision;
  uint64_t surface_cells_rebuilt;
  uint64_t affected_nodes;
  uint64_t affected_edges;
  uint64_t preserved_nodes;
  uint64_t preserved_edges;
  uint64_t reused_nodes;
  uint64_t new_nodes;
  uint64_t removed_nodes;
  uint64_t reused_edges;
  uint64_t rebuilt_edges;
  uint8_t route_replan_required;
  uint8_t graph_rebuilt_globally;
} LtMlsUpdateReport;

typedef struct LtMlsRouteCheckConfig {
  int32_t body_radius_cells;
  int32_t body_height_cells;
  int32_t max_step_cells;
  float max_slope;
  uint8_t has_no_go;
  LtMlsRegion no_go;
} LtMlsRouteCheckConfig;

typedef struct LtMlsRouteCheckResult {
  uint8_t valid;
  uint32_t failure_kind;
  int32_t failed_segment;
  LtMlsCell failed_cell;
} LtMlsRouteCheckResult;

enum { LT_MLS_ROUTE_OCCUPIED = 3, LT_MLS_ROUTE_NO_GO = 6, LT_MLS_ROUTE_CANCELLED = 11 };

typedef struct LtMlsPoint {
  double x;
  double y;
  double z;
} LtMlsPoint;

typedef struct LtMlsMetricRegion {
  LtMlsPoint min;
  LtMlsPoint max;
} LtMlsMetricRegion;

typedef struct LtMlsPose {
  LtMlsPoint point;
  double yaw;
  uint8_t has_yaw;
} LtMlsPose;

typedef struct LtMlsPoseBuffer {
  LtMlsPose* ptr;
  uint64_t len;
  uint64_t cap;
} LtMlsPoseBuffer;

typedef struct LtMlsMetricConfig {
  double resolution;
  double robot_radius;
  double below;
  double above;
  double support_height;
  double support_tolerance;
  double max_step;
  double max_slope;
  LtMlsPoint min;
  LtMlsPoint max;
  int32_t support_depth_cells;
  int32_t support_patch_radius_cells;
  int32_t support_patch_min_samples;
  uint8_t ground_support;
  uint8_t direct_support_only;
  uint8_t actual_start;
  double body_offset;
  LtMlsPose start;
  uint8_t start_exemption;
  double goal_tolerance;
  double goal_xy_tolerance;
  double goal_z_tolerance;
} LtMlsMetricConfig;

typedef int32_t (*LtMlsCancelCheck)(void* context);
typedef struct LtMlsGeometry {
  LtMlsMetricConfig config;
  const LtMlsMetricRegion* regions;
  uint64_t regions_len;
  const LtMlsCell* preblocked;
  uint64_t preblocked_len;
  LtMlsCancelCheck cancel;
  void* context;
} LtMlsGeometry;

typedef struct LtMlsBuffer {
  LtMlsCell* ptr;
  uint64_t len;
  uint64_t cap;
} LtMlsBuffer;

uint32_t lt_mls_abi_version(void);

int32_t lt_mls_extract_surfaces(
    const LtMlsCell* occupied,
    uint64_t occupied_len,
    const LtMlsCell* known_free,
    uint64_t known_free_len,
    const LtMlsConfig* config,
    LtMlsBuffer* observed,
    LtMlsBuffer* inferred);

int32_t lt_mls_build_graph_summary(
    const LtMlsCell* surfaces,
    uint64_t surface_len,
    const LtMlsGraphConfig* config,
    LtMlsGraphSummary* summary);

int32_t lt_mls_build_graph(
    const LtMlsCell* surfaces,
    uint64_t surface_len,
    const LtMlsGraphConfig* config,
    LtMlsGraphOutput* output);

int32_t lt_mls_artifact_create(
    const LtMlsCell* occupied,
    uint64_t occupied_len,
    const LtMlsCell* known_free,
    uint64_t known_free_len,
    const LtMlsConfig* surface_config,
    const LtMlsGraphConfig* graph_config,
    uint64_t revision,
    LtMlsArtifact** out);

int32_t lt_mls_artifact_create_from_surface(
    const LtMlsCell* observed,
    uint64_t observed_len,
    const LtMlsCell* inferred,
    uint64_t inferred_len,
    const LtMlsGraphConfig* graph_config,
    uint64_t revision,
    LtMlsArtifact** out);

int32_t lt_mls_artifact_create_from_graph(
    const LtMlsCell* observed,
    uint64_t observed_len,
    const LtMlsCell* inferred,
    uint64_t inferred_len,
    const LtMlsGraphNode* nodes,
    uint64_t nodes_len,
    const LtMlsGraphEdge* edges,
    uint64_t edges_len,
    const LtMlsCell* corridor,
    uint64_t corridor_len,
    const LtMlsGraphConfig* graph_config,
    uint64_t revision,
    LtMlsArtifact** out);

void lt_mls_artifact_free(LtMlsArtifact* handle);

// Restore raw geometry without rebuilding the prepared surface graph.
int32_t lt_mls_artifact_set_geometry(
    LtMlsArtifact* handle,
    const LtMlsCell* occupied, uint64_t occupied_len,
    const LtMlsCell* known_free, uint64_t known_free_len,
    const LtMlsConfig* config);

int32_t lt_mls_artifact_summary(
    const LtMlsArtifact* handle,
    LtMlsArtifactSummary* summary);

int32_t lt_mls_artifact_surfaces(
    const LtMlsArtifact* handle,
    LtMlsBuffer* observed,
    LtMlsBuffer* inferred);

int32_t lt_mls_artifact_below(
    const LtMlsArtifact* handle,
    int32_t x,
    int32_t y,
    int32_t z,
    int32_t* result);

int32_t lt_mls_artifact_inferred(
    const LtMlsArtifact* handle,
    int32_t x,
    int32_t y,
    int32_t min_z,
    int32_t max_z);

int32_t lt_mls_artifact_occupied(
    const LtMlsArtifact* handle,
    int32_t x,
    int32_t y,
    int32_t min_z,
    int32_t max_z);

int32_t lt_mls_artifact_occupied_below(
    const LtMlsArtifact* handle,
    int32_t x,
    int32_t y,
    int32_t z,
    int32_t* result);

int32_t lt_mls_artifact_region_occupied(
    const LtMlsArtifact* handle,
    LtMlsRegion region);

int32_t lt_mls_artifact_height(
    const LtMlsArtifact* handle,
    int32_t x,
    int32_t y,
    double expected_z,
    double tolerance,
    double resolution,
    double* result);

int32_t lt_mls_artifact_graph(
    const LtMlsArtifact* handle,
    LtMlsGraphOutput* output);

int32_t lt_mls_artifact_query(
    const LtMlsArtifact* handle,
    LtMlsCell start,
    LtMlsCell goal,
    LtMlsBuffer* route);

int32_t lt_mls_artifact_query_limited(
    const LtMlsArtifact* handle,
    LtMlsCell start,
    LtMlsCell goal,
    int32_t max_snap_cells,
    LtMlsBuffer* route);

int32_t lt_mls_artifact_query_budgeted(
    const LtMlsArtifact* handle,
    LtMlsCell start,
    LtMlsCell goal,
    int32_t max_snap_cells,
    int32_t max_iterations,
    LtMlsBuffer* route);

// Query again after a geometry gate rejects a corridor. Edges whose corridor
// contains one of the blocked cells are skipped; the map artifact itself is
// unchanged.
int32_t lt_mls_artifact_query_avoiding(
    const LtMlsArtifact* handle,
    LtMlsCell start,
    LtMlsCell goal,
    int32_t max_snap_cells,
    int32_t max_iterations,
    const LtMlsCell* blocked_cells,
    uint64_t blocked_cells_len,
    LtMlsBuffer* route);

// Kinds: segment=0, actual start connector=1, actual goal connector=2,
// cancellation poll=3. Return pass=1, blocked=0, cancelled=-1.
typedef int32_t (*LtMlsSegmentCheck)(LtMlsCell from, LtMlsCell to,
                                   uint32_t kind, void* context);

// Status: found=0, exhausted=1, budget=2, cancelled=3; negative is ABI failure.
int32_t lt_mls_artifact_query_checked(
    const LtMlsArtifact* handle,
    LtMlsCell start,
    LtMlsCell goal,
    int32_t max_snap_cells,
    int32_t max_iterations,
    LtMlsSegmentCheck check,
    void* context,
    LtMlsBuffer* route);

int32_t lt_mls_artifact_check_route(
    const LtMlsArtifact* handle,
    const LtMlsCell* route,
    uint64_t route_len,
    const LtMlsRouteCheckConfig* config,
    LtMlsRouteCheckResult* result);

// Metric PointPose route validation. Failure kinds reuse the cell checker;
// 10 denotes outside map bounds. The handle must own raw occupied evidence.
int32_t lt_mls_artifact_check_metric_route(
    const LtMlsArtifact* handle,
    const LtMlsPoint* route,
    uint64_t route_len,
    const LtMlsMetricConfig* config,
    const LtMlsMetricRegion* regions,
    uint64_t regions_len,
    LtMlsRouteCheckResult* result);

int32_t lt_mls_artifact_check_metric_poses(
    const LtMlsArtifact* handle, const LtMlsPose* route, uint64_t route_len,
    const LtMlsGeometry* geometry, LtMlsRouteCheckResult* result);

// Same query statuses as query_checked. Geometry and turns run entirely in Rust.
int32_t lt_mls_artifact_query_metric(
    const LtMlsArtifact* handle, LtMlsPose start, LtMlsPose goal,
    int32_t max_snap_cells, int32_t max_iterations,
    const LtMlsGeometry* geometry, LtMlsPoseBuffer* route);

void lt_mls_free_pose_buffer(LtMlsPoseBuffer* route);

// Replaces occupied/free source evidence. Returns -2 for handles created from
// extracted surfaces or a cached graph, leaving the handle and report unchanged.
int32_t lt_mls_artifact_update(
    LtMlsArtifact* handle,
    LtMlsRegion region,
    const LtMlsCell* replacement_occupied,
    uint64_t replacement_occupied_len,
    const LtMlsCell* replacement_known_free,
    uint64_t replacement_known_free_len,
    LtMlsUpdateReport* report);

void lt_mls_free_graph(LtMlsGraphOutput* output);

void lt_mls_free_buffer(LtMlsBuffer* buffer);

#ifdef __cplusplus
}
#endif
