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
  uint64_t max_hole_cells;
  float max_slope;
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

void lt_mls_artifact_free(LtMlsArtifact* handle);

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
