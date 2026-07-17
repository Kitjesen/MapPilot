#pragma once

#include <stdint.h>

#if defined(_WIN32)
#  if defined(LINGTU_MAPS_C_API_BUILD)
#    define LINGTU_MAPS_PCD_API __declspec(dllexport)
#  else
#    define LINGTU_MAPS_PCD_API __declspec(dllimport)
#  endif
#else
#  define LINGTU_MAPS_PCD_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Writes an interleaved float32 point array as a binary XYZ PCD.
 *
 * stride_floats must be at least 3. Non-finite points and points outside
 * max_abs_m are rejected by the native implementation. Set max_abs_m <= 0
 * to disable the absolute-coordinate bound.
 */
LINGTU_MAPS_PCD_API int32_t lingtu_maps_write_xyz_pcd(
    const char* path,
    const float* points,
    uint64_t point_count,
    uint32_t stride_floats,
    float max_abs_m,
    uint64_t* written_points);

#ifdef __cplusplus
}
#endif
