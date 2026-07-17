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

typedef struct LingtuMapsStoreHandle LingtuMapsStoreHandle;

LINGTU_MAPS_API LingtuMapsStoreHandle* lingtu_maps_store_create(
    const char* root_dir,
    const char* active_state_filename);

LINGTU_MAPS_API void lingtu_maps_store_destroy(LingtuMapsStoreHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_store_validate_map_id(const char* map_id);

LINGTU_MAPS_API int32_t lingtu_maps_store_list_map_ids(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_active_map_id(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_list_records_json(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_record_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_active_record_json(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_bundle_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    const char* capability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_validate_artifacts_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint8_t require_octomap,
    uint8_t require_occupancy,
    const char* expected_frame_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size);

LINGTU_MAPS_API int32_t lingtu_maps_store_create_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id);

LINGTU_MAPS_API int32_t lingtu_maps_store_delete_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id);

LINGTU_MAPS_API int32_t lingtu_maps_store_rename_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    const char* new_map_id);

LINGTU_MAPS_API int32_t lingtu_maps_store_set_active_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint8_t strict);

LINGTU_MAPS_API int32_t lingtu_maps_store_clear_active_map(
    LingtuMapsStoreHandle* handle);

LINGTU_MAPS_API int32_t lingtu_maps_store_artifact_count(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint64_t* out_count);

#ifdef __cplusplus
}
#endif
