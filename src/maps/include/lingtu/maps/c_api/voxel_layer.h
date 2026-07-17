#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(LINGTU_MAPS_C_API_BUILD)
#    define LINGTU_MAPS_API __declspec(dllexport)
#  else
#    define LINGTU_MAPS_API __declspec(dllimport)
#  endif
#else
#  define LINGTU_MAPS_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LingtuMapsVoxelHandle LingtuMapsVoxelHandle;

typedef struct LingtuMapsVoxelConfig {
  float voxel_size_m;
  float max_range_m;
  float min_z_m;
  float max_z_m;
  float decay_rate;
  float prune_below_count;
  uint8_t column_carving;
} LingtuMapsVoxelConfig;

typedef struct LingtuMapsVoxelStats {
  uint64_t input_points;
  uint64_t accepted_points;
  uint64_t input_voxels;
  uint64_t input_columns;
  uint64_t carved_columns;
  uint64_t carved_voxels;
  uint64_t total_voxels;
  uint64_t accumulated_cells;
  uint64_t accumulated_occupied;
  uint64_t accumulated_generation;
  uint64_t ray_updates;
  uint64_t free_updates;
  uint64_t hit_updates;
  uint64_t pruned_cells;
  uint8_t column_carving;
} LingtuMapsVoxelStats;

typedef struct LingtuMapsVoxelSceneStats {
  uint64_t live_voxels;
  uint64_t accumulated_cells;
  uint64_t accumulated_occupied;
  uint64_t accumulated_generation;
  uint64_t live_points;
  uint64_t frame_stamp_ns;
  float voxel_size_m;
  uint8_t localization_ok;
  uint8_t map_ok;
  uint8_t planner_ok;
  uint32_t status_bits;
} LingtuMapsVoxelSceneStats;

LINGTU_MAPS_API uint32_t lingtu_maps_abi_version(void);

LINGTU_MAPS_API LingtuMapsVoxelHandle* lingtu_maps_voxel_create(
    const LingtuMapsVoxelConfig* config);

LINGTU_MAPS_API void lingtu_maps_voxel_destroy(LingtuMapsVoxelHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_reset(LingtuMapsVoxelHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_decay(LingtuMapsVoxelHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_update_xyz_interleaved(
    LingtuMapsVoxelHandle* handle,
    const float* xyz,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_update_xyzi_interleaved(
    LingtuMapsVoxelHandle* handle,
    const float* xyzi,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_update_xyz_soa(
    LingtuMapsVoxelHandle* handle,
    const float* x,
    const float* y,
    const float* z,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float origin_x_m,
    float origin_y_m,
    float origin_z_m);

LINGTU_MAPS_API uint64_t lingtu_maps_voxel_count(
    const LingtuMapsVoxelHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_contains(
    const LingtuMapsVoxelHandle* handle,
    float x_m,
    float y_m,
    float z_m);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_query_count(
    const LingtuMapsVoxelHandle* handle,
    float x_m,
    float y_m,
    float z_m,
    float* out_count);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_stats(
    const LingtuMapsVoxelHandle* handle,
    LingtuMapsVoxelStats* out_stats);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_snapshot_size(
    const LingtuMapsVoxelHandle* handle,
    uint64_t* out_point_count);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_snapshot_xyz_soa(
    const LingtuMapsVoxelHandle* handle,
    float* out_x,
    float* out_y,
    float* out_z,
    uint64_t capacity,
    uint64_t* out_point_count);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_snapshot_occupied_xyz_soa(
    const LingtuMapsVoxelHandle* handle,
    float* out_x,
    float* out_y,
    float* out_z,
    uint64_t capacity,
    uint64_t* out_point_count);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_scene_stats(
    const LingtuMapsVoxelHandle* handle,
    LingtuMapsVoxelSceneStats* out_stats);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_save_binary(
    const LingtuMapsVoxelHandle* handle,
    const char* path);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_load_binary(
    LingtuMapsVoxelHandle* handle,
    const char* path);

LINGTU_MAPS_API int32_t lingtu_maps_voxel_validate_binary(
    const char* path);

#ifdef __cplusplus
}
#endif
