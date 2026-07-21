#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(LINGTU_MAPS_C_API_BUILD)
#    define LINGTU_MAPS_ROLLING_API __declspec(dllexport)
#  else
#    define LINGTU_MAPS_ROLLING_API __declspec(dllimport)
#  endif
#else
#  define LINGTU_MAPS_ROLLING_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum LingtuMapsOccupancyState {
  LINGTU_MAPS_OCCUPANCY_UNKNOWN = 0,
  LINGTU_MAPS_OCCUPANCY_FREE = 1,
  LINGTU_MAPS_OCCUPANCY_OCCUPIED = 2,
};

typedef struct LingtuMapsRollingOccupancyHandle LingtuMapsRollingOccupancyHandle;

typedef struct LingtuMapsRollingOccupancyConfig {
  int32_t size_x;
  int32_t size_y;
  int32_t size_z;
  float resolution_m;
  float max_ray_range_m;
  float hit_log_odds;
  float miss_log_odds;
  float min_log_odds;
  float max_log_odds;
  float occupied_probability;
  float free_probability;
  int32_t roll_margin_x;
  int32_t roll_margin_y;
  int32_t roll_margin_z;
  int64_t decay_after_ns;
  float decay_factor;
  uint8_t auto_roll;
  uint8_t reject_out_of_order;
} LingtuMapsRollingOccupancyConfig;

typedef struct LingtuMapsRollingOccupancyStats {
  uint64_t input_points;
  uint64_t accepted_points;
  uint64_t rejected_points;
  uint64_t unique_rays;
  uint64_t free_updates;
  uint64_t hit_updates;
  uint64_t rolled_out_cells;
  uint64_t decayed_cells;
  uint64_t generation;
  uint8_t rolled;
} LingtuMapsRollingOccupancyStats;

typedef struct LingtuMapsRollingOccupancySnapshotInfo {
  int64_t stamp_ns;
  uint64_t generation;
  float resolution_m;
  int32_t size_x;
  int32_t size_y;
  int32_t size_z;
  float origin_x_m;
  float origin_y_m;
  float origin_z_m;
  uint64_t cell_count;
} LingtuMapsRollingOccupancySnapshotInfo;

LINGTU_MAPS_ROLLING_API LingtuMapsRollingOccupancyHandle*
lingtu_maps_rolling_occupancy_create(
    const LingtuMapsRollingOccupancyConfig* config);

LINGTU_MAPS_ROLLING_API void lingtu_maps_rolling_occupancy_destroy(
    LingtuMapsRollingOccupancyHandle* handle);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_reset(
    LingtuMapsRollingOccupancyHandle* handle,
    const char* frame_id,
    float center_x_m,
    float center_y_m,
    float center_z_m,
    int64_t stamp_ns);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_roll_to(
    LingtuMapsRollingOccupancyHandle* handle,
    float center_x_m,
    float center_y_m,
    float center_z_m,
    int64_t stamp_ns,
    uint64_t* out_rolled_cell_count);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_update_xyz_soa(
    LingtuMapsRollingOccupancyHandle* handle,
    const float* x,
    const float* y,
    const float* z,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float sensor_origin_x_m,
    float sensor_origin_y_m,
    float sensor_origin_z_m,
    LingtuMapsRollingOccupancyStats* out_stats);

LINGTU_MAPS_ROLLING_API int32_t
lingtu_maps_rolling_occupancy_update_xyz_interleaved(
    LingtuMapsRollingOccupancyHandle* handle,
    const float* xyz,
    uint64_t point_count,
    const char* frame_id,
    int64_t stamp_ns,
    float sensor_origin_x_m,
    float sensor_origin_y_m,
    float sensor_origin_z_m,
    LingtuMapsRollingOccupancyStats* out_stats);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_decay(
    LingtuMapsRollingOccupancyHandle* handle,
    int64_t now_ns,
    uint64_t* out_decayed_cells);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_query(
    const LingtuMapsRollingOccupancyHandle* handle,
    float x_m,
    float y_m,
    float z_m,
    uint8_t* out_state,
    float* out_probability);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_snapshot_info(
    const LingtuMapsRollingOccupancyHandle* handle,
    LingtuMapsRollingOccupancySnapshotInfo* out_info);

LINGTU_MAPS_ROLLING_API int32_t lingtu_maps_rolling_occupancy_snapshot_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_cell_count);

LINGTU_MAPS_ROLLING_API int32_t
lingtu_maps_rolling_occupancy_observed_count(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint64_t* out_count);

LINGTU_MAPS_ROLLING_API int32_t
lingtu_maps_rolling_occupancy_observed_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    float* out_x_m,
    float* out_y_m,
    float* out_z_m,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_count);

LINGTU_MAPS_ROLLING_API int32_t
lingtu_maps_rolling_occupancy_rolled_out_count(
    const LingtuMapsRollingOccupancyHandle* handle,
    uint64_t* out_count);

LINGTU_MAPS_ROLLING_API int32_t
lingtu_maps_rolling_occupancy_rolled_out_copy(
    const LingtuMapsRollingOccupancyHandle* handle,
    float* out_x_m,
    float* out_y_m,
    float* out_z_m,
    uint8_t* out_state,
    int16_t* out_log_odds_q8,
    uint64_t capacity,
    uint64_t* out_count);

#ifdef __cplusplus
}
#endif
