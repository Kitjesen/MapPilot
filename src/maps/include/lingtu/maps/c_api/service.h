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

typedef struct LingtuMapsServiceHandle LingtuMapsServiceHandle;

#define LINGTU_MAPS_SAVE_MAP_ABI_VERSION 1U
#define LINGTU_MAPS_UNITY_SEMANTIC_IMPORT_ABI_VERSION 1U

typedef struct LingtuMapsUnitySemanticImportOptions {
  uint32_t struct_size;
  uint32_t abi_version;
  const char* taxonomy_path;
  const char* frame_id;
  double voxel_size_m;
  double occupied_probability;
  double shell_thickness_voxels;
  uint64_t generation;
  uint64_t max_objects;
  uint64_t max_voxels;
  uint64_t max_voxel_checks;
  uint8_t include_unknown_geometry;
  uint8_t exclude_dynamic_classes;
} LingtuMapsUnitySemanticImportOptions;

typedef struct LingtuMapsSaveRequirements {
  uint32_t struct_size;
  uint32_t abi_version;
  uint8_t occupancy;
  uint8_t octomap;
  uint8_t esdf;
  uint8_t traversability;
  uint8_t semantic;
} LingtuMapsSaveRequirements;

typedef struct LingtuMapsSaveSourceOptions {
  uint32_t struct_size;
  uint32_t abi_version;
  double voxel_size;
  uint8_t dynamic_filter_enabled;
  uint8_t dynamic_filter_required;
  const char* dynamic_filter_command;
  double dynamic_filter_timeout_sec;
  const char* optimizer_strategy;
  uint8_t optimizer_required;
  const char* optimizer_command;
  double optimizer_timeout_sec;
} LingtuMapsSaveSourceOptions;

typedef struct LingtuMapsSaveOctomapOptions {
  uint32_t struct_size;
  uint32_t abi_version;
  const char* converter_command;
  const char* build_mode;
  double resolution;
  int32_t support_dilation_cells;
  int32_t free_layers_above;
  int32_t free_dilation_cells;
  const char* frame_id;
  const char* source_profile;
  const char* data_source;
  const char* slam_source;
  const char* localization_source;
  const char* mapping_source;
  double timeout_sec;
} LingtuMapsSaveOctomapOptions;

typedef struct LingtuMapsSaveMapRequest {
  uint32_t struct_size;
  uint32_t abi_version;
  const char* request_id;
  const char* map_id;
  LingtuMapsSaveRequirements require;
  LingtuMapsSaveSourceOptions source;
  LingtuMapsSaveOctomapOptions octomap;
  uint8_t activate_on_success;
  uint8_t require_slam_healthy;
  uint64_t minimum_point_count;
} LingtuMapsSaveMapRequest;

typedef struct LingtuMapsMapSnapshot {
  uint32_t struct_size;
  uint32_t abi_version;
  const char* snapshot_id;
  const char* source_dir;
  const char* frame_id;
  int64_t captured_at_ns;
  uint64_t first_sequence;
  uint64_t last_sequence;
  const char* source_sha256;
  uint8_t slam_healthy;
  const char* health_message;
} LingtuMapsMapSnapshot;

LINGTU_MAPS_API LingtuMapsServiceHandle* lingtu_maps_service_create(
    const char* root_dir,
    const char* active_state_filename);

LINGTU_MAPS_API void lingtu_maps_service_destroy(LingtuMapsServiceHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_maps_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_map_types_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_record_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_active_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_health_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_validate_artifacts_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t require_octomap,
    uint8_t require_occupancy,
    const char* expected_frame_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_ingest_localization_health_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    uint8_t localized,
    double position_error_m,
    double covariance_trace,
    double quality,
    const char* source,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_ingest_planning_outcome_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    uint8_t success,
    const char* planner,
    const char* reason,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_ingest_collision_event_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    double timestamp_s,
    double severity,
    const char* source,
    const char* reason,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_bundle_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* capability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_map_points_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint64_t max_points,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_save_map_begin_json(
    LingtuMapsServiceHandle* handle,
    const LingtuMapsSaveMapRequest* request,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_save_map_provide_snapshot_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    const LingtuMapsMapSnapshot* snapshot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_save_map_status_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_save_map_jobs_json(
    LingtuMapsServiceHandle* handle,
    uint64_t limit,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_save_map_cancel_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_save_map_retry_json(
    LingtuMapsServiceHandle* handle,
    const char* job_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_map_versions_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_rollback_map_version_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    int64_t version,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_set_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* name,
    double x_m,
    double y_m,
    double z_m,
    double yaw_rad,
    uint8_t has_yaw,
    const char* frame_id,
    const char* tags_json,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_delete_poi_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* name,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_map_graph_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_set_map_edge_json(
    LingtuMapsServiceHandle* handle,
    const char* from_map_id,
    const char* to_map_id,
    const char* edge_type,
    uint8_t bidirectional,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_delete_map_edge_json(
    LingtuMapsServiceHandle* handle,
    const char* from_map_id,
    const char* to_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_shortest_route_json(
    LingtuMapsServiceHandle* handle,
    const char* start_map_id,
    const char* goal_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_rollback_active_map_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_list_active_slots_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_set_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    const char* map_id,
    uint8_t strict,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_clear_active_slot_json(
    LingtuMapsServiceHandle* handle,
    const char* slot,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_build_queue_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_enqueue_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* artifact_type,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_pop_build_queue_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_cancel_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_retry_artifact_job_json(
    LingtuMapsServiceHandle* handle,
    const char* request_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_audit_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_quarantine_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_gc_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_migrate_versions_json(
    LingtuMapsServiceHandle* handle,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_export_version_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    int64_t version,
    const char* package_dir,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_import_package_json(
    LingtuMapsServiceHandle* handle,
    const char* package_dir,
    const char* requested_map_id,
    uint8_t dry_run,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_create_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_delete_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_rename_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* new_map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_retire_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_set_active_map_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t strict,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_clear_active_map_json(
    LingtuMapsServiceHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_begin_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* artifact_type,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_finish_build_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* build_id,
    uint8_t success,
    const char* message,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_build_status_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_import_pcd_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* source_path,
    double voxel_size,
    uint8_t has_bounds,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_commit_saved_source_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* source_dir,
    double voxel_size,
    uint8_t dynamic_filter_enabled,
    uint8_t dynamic_filter_required,
    const char* dynamic_filter_command,
    double dynamic_filter_timeout_sec,
    const char* optimizer_strategy,
    uint8_t optimizer_required,
    const char* optimizer_command,
    double optimizer_timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_crop_pcd_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    uint8_t invert,
    double voxel_size,
    uint8_t has_bounds,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_restore_source_backup_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_occupancy_snapshot_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_octomap_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* converter_command,
    const char* build_mode,
    double resolution,
    int32_t support_dilation_cells,
    int32_t free_layers_above,
    int32_t free_dilation_cells,
    const char* frame_id,
    const char* source_profile,
    const char* data_source,
    const char* slam_source,
    const char* localization_source,
    const char* mapping_source,
    double timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_get_voxel_edits_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_edit_octomap_voxels_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* editor_command,
    const char* state,
    const char* shape,
    double x_m,
    double y_m,
    double z_m,
    double radius_m,
    double timeout_sec,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_navigation_package_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* converter_command,
    const char* build_mode,
    double resolution,
    int32_t support_dilation_cells,
    int32_t free_layers_above,
    int32_t free_dilation_cells,
    const char* frame_id,
    const char* source_profile,
    const char* data_source,
    const char* slam_source,
    const char* localization_source,
    const char* mapping_source,
    double timeout_sec,
    uint8_t include_esdf,
    uint8_t include_traversability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_esdf_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_traversability_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_build_semantic_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_service_import_unity_semantic_artifact_json(
    LingtuMapsServiceHandle* handle,
    const char* map_id,
    const char* scene_dir,
    const LingtuMapsUnitySemanticImportOptions* options,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

#ifdef __cplusplus
}
#endif
