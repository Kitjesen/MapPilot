#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef LINGTU_MAPS_API
#  if defined(_WIN32)
#    if defined(LINGTU_MAPS_C_API_BUILD)
#      define LINGTU_MAPS_API __declspec(dllexport)
#    else
#      define LINGTU_MAPS_API __declspec(dllimport)
#    endif
#  else
#    define LINGTU_MAPS_API __attribute__((visibility("default")))
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LingtuMapsSemanticHandle LingtuMapsSemanticHandle;

typedef enum LingtuMapsSemanticStatus {
  LINGTU_MAPS_SEMANTIC_OK = 0,
  LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL = 1,
  LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED = 2,
  LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT = -1,
  LINGTU_MAPS_SEMANTIC_ERROR = -2,
  LINGTU_MAPS_SEMANTIC_UNKNOWN_ERROR = -3,
} LingtuMapsSemanticStatus;

typedef struct LingtuMapsSemanticConfig {
  float voxel_size_m;
  float max_range_m;
  float min_z_m;
  float max_z_m;
  float hit_log_odds;
  float miss_log_odds;
  float min_log_odds;
  float max_log_odds;
  float occupied_probability;
  uint8_t raycast_free_space;
  uint64_t max_rays_per_update;
  uint64_t max_ray_voxels_per_ray;
  uint64_t max_voxels;
  uint64_t max_query_voxel_checks;
  uint64_t max_query_results;
  uint16_t unknown_label;
} LingtuMapsSemanticConfig;

typedef struct LingtuMapsSemanticUpdateStats {
  uint64_t input_points;
  uint64_t accepted_points;
  uint64_t hit_voxels;
  uint64_t rays_traced;
  uint64_t truncated_rays;
  uint64_t free_voxel_updates;
  uint64_t pruned_voxels;
  uint64_t total_voxels;
  uint64_t generation_before;
  uint64_t generation_after;
  uint8_t replaced_full_map;
  uint8_t applied;
  uint8_t duplicate_sequence;
  uint8_t stale_sequence;
} LingtuMapsSemanticUpdateStats;

typedef struct LingtuMapsSemanticQuery {
  float center_x_m;
  float center_y_m;
  float center_z_m;
  float radius_m;
  float min_occupancy_probability;
} LingtuMapsSemanticQuery;

typedef struct LingtuMapsSemanticMetadata {
  uint64_t generation;
  uint64_t voxel_count;
  float voxel_size_m;
  uint32_t taxonomy_version;
  uint64_t frame_id_bytes;
  uint64_t taxonomy_bytes;
} LingtuMapsSemanticMetadata;

typedef struct LingtuMapsSemanticChunkBuffers {
  int32_t* index_x;
  int32_t* index_y;
  int32_t* index_z;
  float* center_x_m;
  float* center_y_m;
  float* center_z_m;
  float* occupancy_probability;
  uint32_t* hit_count;
  uint32_t* miss_count;
  uint32_t* point_count;
  float* mean_x_m;
  float* mean_y_m;
  float* mean_z_m;
  float* covariance_xx;
  float* covariance_xy;
  float* covariance_xz;
  float* covariance_yy;
  float* covariance_yz;
  float* covariance_zz;
  uint16_t* dominant_label;
  float* semantic_confidence;
} LingtuMapsSemanticChunkBuffers;

LINGTU_MAPS_API uint32_t lingtu_maps_semantic_abi_version(void);

LINGTU_MAPS_API LingtuMapsSemanticHandle* lingtu_maps_semantic_create(
    const LingtuMapsSemanticConfig* config);

LINGTU_MAPS_API LingtuMapsSemanticHandle* lingtu_maps_semantic_open_file(
    const char* path);

LINGTU_MAPS_API void lingtu_maps_semantic_destroy(LingtuMapsSemanticHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_reset(LingtuMapsSemanticHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_generation(
    const LingtuMapsSemanticHandle* handle,
    uint64_t* out_generation);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_metadata(
    const LingtuMapsSemanticHandle* handle,
    LingtuMapsSemanticMetadata* out_metadata);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_metadata_strings(
    const LingtuMapsSemanticHandle* handle,
    uint64_t expected_generation,
    char* out_frame_id,
    uint64_t frame_id_capacity,
    char* out_taxonomy,
    uint64_t taxonomy_capacity);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_load_file(
    LingtuMapsSemanticHandle* handle,
    const char* path,
    uint64_t expected_generation,
    LingtuMapsSemanticUpdateStats* out_stats);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_save_file(
    const LingtuMapsSemanticHandle* handle,
    const char* path,
    uint64_t expected_generation);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_validate_file(
    const char* path,
    char* out_error,
    uint64_t error_capacity,
    uint64_t* out_error_size);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_update_xyz_soa(
    LingtuMapsSemanticHandle* handle,
    const float* x,
    const float* y,
    const float* z,
    const uint16_t* labels,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m,
    uint64_t sequence,
    uint64_t expected_generation,
    uint8_t full_map,
    const char* taxonomy,
    uint32_t taxonomy_version,
    LingtuMapsSemanticUpdateStats* out_stats);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_query_radius_count(
    const LingtuMapsSemanticHandle* handle,
    const LingtuMapsSemanticQuery* query,
    uint64_t* out_generation,
    uint64_t* out_count);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_query_radius_fill(
    const LingtuMapsSemanticHandle* handle,
    const LingtuMapsSemanticQuery* query,
    uint64_t expected_generation,
    const LingtuMapsSemanticChunkBuffers* buffers,
    uint64_t capacity,
    uint64_t* out_count);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_snapshot_count(
    const LingtuMapsSemanticHandle* handle,
    float min_occupancy_probability,
    uint64_t* out_generation,
    uint64_t* out_count);

LINGTU_MAPS_API int32_t lingtu_maps_semantic_snapshot_fill(
    const LingtuMapsSemanticHandle* handle,
    float min_occupancy_probability,
    uint64_t expected_generation,
    uint64_t offset,
    const LingtuMapsSemanticChunkBuffers* buffers,
    uint64_t capacity,
    uint64_t* out_count,
    uint8_t* out_complete);

#ifdef __cplusplus
}
#endif
