#define LINGTU_POINTCLOUD_CODEC_BUILD

#include "lingtu/pointcloud_codec.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

namespace {

constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kFlagHasColor = 0x01;
constexpr std::size_t kHeaderSize = 28;

void write_u16(std::uint8_t* dst, std::uint16_t value) {
  dst[0] = static_cast<std::uint8_t>(value & 0xff);
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
}

void write_u32(std::uint8_t* dst, std::uint32_t value) {
  dst[0] = static_cast<std::uint8_t>(value & 0xff);
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  dst[2] = static_cast<std::uint8_t>((value >> 16) & 0xff);
  dst[3] = static_cast<std::uint8_t>((value >> 24) & 0xff);
}

void write_f32(std::uint8_t* dst, float value) {
  static_assert(sizeof(float) == 4);
  std::uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(float));
  write_u32(dst, raw);
}

std::int16_t quantize(float value) {
  const float rounded = std::nearbyint(value);
  const float clipped = std::clamp(
      rounded,
      static_cast<float>(std::numeric_limits<std::int16_t>::min()),
      static_cast<float>(std::numeric_limits<std::int16_t>::max()));
  return static_cast<std::int16_t>(clipped);
}

void write_i16(std::uint8_t* dst, std::int16_t value) {
  const auto raw = static_cast<std::uint16_t>(value);
  write_u16(dst, raw);
}

void write_header(
    std::uint8_t* dst,
    std::uint8_t flags,
    std::uint32_t count,
    float scale,
    float ox,
    float oy,
    float oz) {
  dst[0] = 'P';
  dst[1] = 'C';
  dst[2] = 'L';
  dst[3] = 'D';
  dst[4] = kVersion;
  dst[5] = flags;
  write_u16(dst + 6, 0);
  write_u32(dst + 8, count);
  write_f32(dst + 12, scale);
  write_f32(dst + 16, ox);
  write_f32(dst + 20, oy);
  write_f32(dst + 24, oz);
}

}  // namespace

extern "C" int lingtu_encode_pointcloud_v1(
    const float* xyz,
    std::uint32_t count,
    float scale,
    const std::uint8_t* colors,
    std::uint8_t has_colors,
    std::uint8_t** out,
    std::size_t* out_len) {
  if (out == nullptr || out_len == nullptr) {
    return -1;
  }
  *out = nullptr;
  *out_len = 0;

  if (count > 0 && xyz == nullptr) {
    return -2;
  }
  if (!(scale > 0.0f) || !std::isfinite(scale)) {
    return -3;
  }

  const bool with_colors = has_colors != 0 && colors != nullptr;
  const std::size_t payload_len =
      static_cast<std::size_t>(count) * 3U * (sizeof(std::int16_t) + (with_colors ? sizeof(std::uint8_t) : 0U));
  const std::size_t total_len = kHeaderSize + payload_len;

  std::vector<std::uint8_t> frame;
  try {
    frame.resize(total_len);
  } catch (const std::bad_alloc&) {
    return -4;
  }

  if (count == 0) {
    write_header(frame.data(), 0, 0, 1.0f, 0.0f, 0.0f, 0.0f);
  } else {
    float ox = xyz[0];
    float oy = xyz[1];
    float oz = xyz[2];
    for (std::uint32_t i = 1; i < count; ++i) {
      const float x = xyz[i * 3U];
      const float y = xyz[i * 3U + 1U];
      const float z = xyz[i * 3U + 2U];
      ox = std::min(ox, x);
      oy = std::min(oy, y);
      oz = std::min(oz, z);
    }

    const std::uint8_t flags = with_colors ? kFlagHasColor : 0;
    write_header(frame.data(), flags, count, scale, ox, oy, oz);

    std::uint8_t* pos = frame.data() + kHeaderSize;
    for (std::uint32_t i = 0; i < count; ++i) {
      const float x = (xyz[i * 3U] - ox) / scale;
      const float y = (xyz[i * 3U + 1U] - oy) / scale;
      const float z = (xyz[i * 3U + 2U] - oz) / scale;
      write_i16(pos + (static_cast<std::size_t>(i) * 3U + 0U) * sizeof(std::int16_t), quantize(x));
      write_i16(pos + (static_cast<std::size_t>(i) * 3U + 1U) * sizeof(std::int16_t), quantize(y));
      write_i16(pos + (static_cast<std::size_t>(i) * 3U + 2U) * sizeof(std::int16_t), quantize(z));
    }

    if (with_colors) {
      const std::size_t color_offset = kHeaderSize + static_cast<std::size_t>(count) * 3U * sizeof(std::int16_t);
      std::memcpy(frame.data() + color_offset, colors, static_cast<std::size_t>(count) * 3U);
    }
  }

  auto* result = new (std::nothrow) std::uint8_t[frame.size()];
  if (result == nullptr) {
    return -4;
  }
  std::memcpy(result, frame.data(), frame.size());
  *out = result;
  *out_len = frame.size();
  return 0;
}

extern "C" void lingtu_free_buffer(std::uint8_t* ptr) {
  delete[] ptr;
}
