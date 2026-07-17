#pragma once

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

typedef struct LingtuMapsGridSpec {
  int32_t rows;
  int32_t cols;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
} LingtuMapsGridSpec;

typedef struct LingtuMapsTerrainRiskParams {
  float max_slope_deg;
  float soft_slope_start_deg;
  float critical_step_m;
  float roughness_critical_m;
} LingtuMapsTerrainRiskParams;

typedef struct LingtuMapsTraversabilityParams {
  float lethal;
  float inscribed;
  float max_slope_deg;
  float soft_slope_start_deg;
  float safe_distance_m;
  float proximity_cap;
} LingtuMapsTraversabilityParams;

typedef struct LingtuMapsOccupancyCounts {
  int64_t unknown;
  int64_t free;
  int64_t occupied;
} LingtuMapsOccupancyCounts;

LINGTU_MAPS_API int32_t lingtu_maps_build_elevation_map(
    const float* xyz,
    uint64_t point_count,
    double robot_x_m,
    double robot_y_m,
    double resolution_m,
    double radius_m,
    double z_floor_m,
    double z_ceil_m,
    LingtuMapsGridSpec* out_spec,
    float* out_min_z,
    float* out_max_z,
    float* out_clearance,
    uint8_t* out_valid,
    uint64_t capacity);

LINGTU_MAPS_API int32_t lingtu_maps_compute_esdf(
    const float* occupancy,
    const LingtuMapsGridSpec* spec,
    float obstacle_threshold,
    float* out_distance,
    float* out_grad_x,
    float* out_grad_y,
    uint64_t capacity);

LINGTU_MAPS_API int32_t lingtu_maps_compute_terrain_risk(
    const float* min_z,
    const float* max_z,
    const float* clearance,
    const uint8_t* valid,
    const LingtuMapsGridSpec* spec,
    const LingtuMapsTerrainRiskParams* params,
    float* out_risk,
    float* out_slope_deg,
    float* out_step_height,
    float* out_roughness,
    uint64_t capacity);

LINGTU_MAPS_API int32_t lingtu_maps_fuse_traversability_cost(
    const float* cost,
    const float* slope_deg,
    const float* esdf_distance,
    const float* terrain_risk,
    const LingtuMapsGridSpec* spec,
    const LingtuMapsTraversabilityParams* params,
    float* out_cost,
    uint64_t capacity);

LINGTU_MAPS_API int32_t lingtu_maps_build_occupancy_grid(
    const float* xyz,
    uint64_t point_count,
    double robot_x_m,
    double robot_y_m,
    double robot_yaw_rad,
    double resolution_m,
    double radius_m,
    double z_min_m,
    double z_max_m,
    double inflation_radius_m,
    double robot_clear_radius_m,
    double robot_clear_forward_m,
    double robot_clear_backward_m,
    double robot_clear_lateral_m,
    uint8_t raycast_free_space,
    uint8_t unknown_as_obstacle_for_costmap,
    uint32_t raycast_max_rays,
    double raycast_free_inflation_radius_m,
    LingtuMapsGridSpec* out_spec,
    int8_t* out_occupancy,
    float* out_cost,
    LingtuMapsOccupancyCounts* out_counts,
    uint64_t capacity);

LINGTU_MAPS_API int32_t lingtu_maps_resample_grid_bilinear(
    const float* src,
    const LingtuMapsGridSpec* src_spec,
    int32_t dst_rows,
    int32_t dst_cols,
    double dst_resolution_m,
    double dst_origin_x_m,
    double dst_origin_y_m,
    float fill,
    float* out,
    uint64_t capacity);

#ifdef __cplusplus
}
#endif
