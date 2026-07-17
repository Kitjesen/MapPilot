#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#  if defined(LINGTU_POINTCLOUD_CODEC_BUILD)
#    define LINGTU_POINTCLOUD_CODEC_API __declspec(dllexport)
#  else
#    define LINGTU_POINTCLOUD_CODEC_API __declspec(dllimport)
#  endif
#else
#  define LINGTU_POINTCLOUD_CODEC_API __attribute__((visibility("default")))
#endif

extern "C" {

// Encodes contiguous float32 XYZ points into LingTu PCLD v1.
//
// xyz points are laid out as [x0,y0,z0, x1,y1,z1, ...].
// colors may be null. When present, colors are contiguous uint8 RGB triples.
// The returned buffer is allocated by the library and must be released with
// lingtu_free_buffer.
LINGTU_POINTCLOUD_CODEC_API int lingtu_encode_pointcloud_v1(
    const float* xyz,
    std::uint32_t count,
    float scale,
    const std::uint8_t* colors,
    std::uint8_t has_colors,
    std::uint8_t** out,
    std::size_t* out_len);

LINGTU_POINTCLOUD_CODEC_API void lingtu_free_buffer(std::uint8_t* ptr);

}  // extern "C"
