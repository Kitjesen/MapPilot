#pragma once

#include <stdint.h>

#if defined(_WIN32)
#define LINGTU_INSPECTION_API __declspec(dllexport)
#else
#define LINGTU_INSPECTION_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void* lingtu_inspection_store_handle;

typedef struct lingtu_inspection_point {
  const char* id;
  double x_m;
  double y_m;
  double z_m;
  double yaw_rad;
  int32_t has_yaw;
  double position_tolerance_m;
  double yaw_tolerance_rad;
  double dwell_s;
  const char* action;
  int32_t enabled;
} lingtu_inspection_point;

typedef struct lingtu_inspection_route {
  const char* id;
  const char* name;
  const char* map_id;
  int64_t map_version;
  uint64_t revision;
  uint32_t loop_count;
  int32_t failure_policy;
  uint32_t max_retries;
  const lingtu_inspection_point* points;
  uint64_t point_count;
} lingtu_inspection_route;

LINGTU_INSPECTION_API uint32_t lingtu_inspection_store_abi_version(void);
LINGTU_INSPECTION_API lingtu_inspection_store_handle
lingtu_inspection_store_create(const char* map_root);
LINGTU_INSPECTION_API void lingtu_inspection_store_destroy(
    lingtu_inspection_store_handle handle);
LINGTU_INSPECTION_API int32_t lingtu_inspection_store_put(
    lingtu_inspection_store_handle handle,
    const lingtu_inspection_route* route);
LINGTU_INSPECTION_API int32_t lingtu_inspection_store_delete(
    lingtu_inspection_store_handle handle,
    const char* map_id,
    const char* route_id);
LINGTU_INSPECTION_API char* lingtu_inspection_store_get_json(
    lingtu_inspection_store_handle handle,
    const char* map_id,
    const char* route_id);
LINGTU_INSPECTION_API char* lingtu_inspection_store_list_json(
    lingtu_inspection_store_handle handle,
    const char* map_id);
LINGTU_INSPECTION_API char* lingtu_inspection_store_status_json(
    lingtu_inspection_store_handle handle);
LINGTU_INSPECTION_API const char* lingtu_inspection_store_last_error(
    lingtu_inspection_store_handle handle);
LINGTU_INSPECTION_API void lingtu_inspection_string_free(char* value);

#ifdef __cplusplus
}
#endif
