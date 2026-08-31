#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <type_traits>

namespace lingtu::drivers::camera::record {

inline constexpr std::array<char, 4> kMagic{{'L', 'T', 'O', 'B'}};
inline constexpr std::string_view kWireByteOrder{"little-endian"};
inline constexpr std::uint16_t kVersion = 2;
inline constexpr std::uint16_t kKindIntrinsics = 1;
inline constexpr std::uint16_t kKindColor = 2;
inline constexpr std::uint16_t kKindDepth = 3;
inline constexpr std::uint32_t kFormatUnknown = 0;
inline constexpr std::uint32_t kFormatRgb8 = 1;
inline constexpr std::uint32_t kFormatBgr8 = 2;
inline constexpr std::uint32_t kFormatDepthU16 = 3;
inline constexpr double kDepthScaleMetersPerMillimeter = 0.001;
inline constexpr std::uint32_t kMaxDimension = 16384;
inline constexpr std::uint32_t kMaxPayloadBytes = 128u * 1024u * 1024u;
inline constexpr double kMaxTimestampSeconds =
    static_cast<double>(std::numeric_limits<std::int32_t>::max());
inline constexpr int kMinRecordTimeoutMs = 1;
inline constexpr int kMaxRecordTimeoutMs = 60000;

using RecordClock = std::chrono::steady_clock;
using RecordDeadline = RecordClock::time_point;

#pragma pack(push, 1)
struct RecordHeader {
  char magic[4];
  std::uint16_t version;
  std::uint16_t kind;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t channels;
  std::uint32_t format;
  double timestamp_s;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scale_m;
  std::uint32_t payload_size;
  double dist_k1;
  double dist_k2;
  double dist_p1;
  double dist_p2;
  double dist_k3;
};
#pragma pack(pop)

static_assert(sizeof(RecordHeader) == 116, "unexpected camera record header size");
static_assert(offsetof(RecordHeader, version) == 4);
static_assert(offsetof(RecordHeader, kind) == 6);
static_assert(offsetof(RecordHeader, width) == 8);
static_assert(offsetof(RecordHeader, height) == 12);
static_assert(offsetof(RecordHeader, channels) == 16);
static_assert(offsetof(RecordHeader, format) == 20);
static_assert(offsetof(RecordHeader, timestamp_s) == 24);
static_assert(offsetof(RecordHeader, fx) == 32);
static_assert(offsetof(RecordHeader, fy) == 40);
static_assert(offsetof(RecordHeader, cx) == 48);
static_assert(offsetof(RecordHeader, cy) == 56);
static_assert(offsetof(RecordHeader, depth_scale_m) == 64);
static_assert(offsetof(RecordHeader, payload_size) == 72);
static_assert(offsetof(RecordHeader, dist_k1) == 76);
static_assert(offsetof(RecordHeader, dist_k2) == 84);
static_assert(offsetof(RecordHeader, dist_p1) == 92);
static_assert(offsetof(RecordHeader, dist_p2) == 100);
static_assert(offsetof(RecordHeader, dist_k3) == 108);
static_assert(std::is_standard_layout_v<RecordHeader>);
static_assert(std::is_trivially_copyable_v<RecordHeader>);

enum class RecordValidation {
  kValid,
  kUnsupportedByteOrder,
  kInvalidMagic,
  kUnsupportedVersion,
  kUnsupportedKind,
  kInvalidDimensions,
  kInvalidTimestamp,
  kInvalidDepthScale,
  kPayloadTooLarge,
  kMissingPayload,
  kIntrinsicsRequired,
  kInvalidIntrinsicsFormat,
  kInvalidIntrinsicsChannels,
  kInvalidIntrinsicsPayload,
  kInvalidIntrinsics,
  kInvalidColorFormat,
  kInvalidColorChannels,
  kInvalidDepthFormat,
  kInvalidDepthChannels,
  kPayloadSizeMismatch,
};

inline RecordHeader makeRecordHeader(std::uint16_t kind) noexcept {
  RecordHeader header{};
  std::copy(kMagic.begin(), kMagic.end(), header.magic);
  header.version = kVersion;
  header.kind = kind;
  header.depth_scale_m = kDepthScaleMetersPerMillimeter;
  return header;
}

inline bool hasMagic(const RecordHeader& header) noexcept {
  return std::equal(kMagic.begin(), kMagic.end(), header.magic);
}

inline bool hasValidPreamble(const RecordHeader& header) noexcept {
  return hasMagic(header) && header.version == kVersion;
}

inline bool nativeByteOrderIsLittleEndian() noexcept {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

inline bool checkedImagePayloadBytes(
    const RecordHeader& header,
    std::uint32_t bytes_per_pixel,
    std::uint64_t* payload_bytes) noexcept {
  if (payload_bytes == nullptr || bytes_per_pixel == 0) {
    return false;
  }
  *payload_bytes = 0;
  const auto width = static_cast<std::uint64_t>(header.width);
  const auto height = static_cast<std::uint64_t>(header.height);
  const auto max_value = std::numeric_limits<std::uint64_t>::max();
  if (width != 0 && height > max_value / width) {
    return false;
  }
  const auto pixel_count = width * height;
  if (pixel_count != 0 && bytes_per_pixel > max_value / pixel_count) {
    return false;
  }
  *payload_bytes = pixel_count * bytes_per_pixel;
  return true;
}

inline bool hasValidDimensions(const RecordHeader& header) noexcept {
  return header.width > 0 && header.height > 0 &&
      header.width <= kMaxDimension && header.height <= kMaxDimension;
}

inline bool isValidRecordTimeoutMs(int timeout_ms) noexcept {
  return timeout_ms >= kMinRecordTimeoutMs &&
      timeout_ms <= kMaxRecordTimeoutMs;
}

inline bool isValidTimestampSeconds(double timestamp_s) noexcept {
  return std::isfinite(timestamp_s) && timestamp_s > 0.0 &&
      timestamp_s <= kMaxTimestampSeconds;
}

inline bool hasCanonicalDepthScale(const RecordHeader& header) noexcept {
  return header.depth_scale_m == kDepthScaleMetersPerMillimeter;
}

inline RecordValidation validateRecordPayloadPointer(
    const RecordHeader& header,
    const void* payload) noexcept {
  return header.payload_size > 0 && payload == nullptr
      ? RecordValidation::kMissingPayload
      : RecordValidation::kValid;
}

inline RecordValidation validateRecordSequence(
    const RecordHeader& header,
    bool has_intrinsics) noexcept {
  return header.kind != kKindIntrinsics && !has_intrinsics
      ? RecordValidation::kIntrinsicsRequired
      : RecordValidation::kValid;
}

inline RecordValidation validateRecordHeader(const RecordHeader& header) noexcept {
  if (!nativeByteOrderIsLittleEndian()) {
    return RecordValidation::kUnsupportedByteOrder;
  }
  if (!hasMagic(header)) {
    return RecordValidation::kInvalidMagic;
  }
  if (header.version != kVersion) {
    return RecordValidation::kUnsupportedVersion;
  }
  if (header.kind != kKindIntrinsics &&
      header.kind != kKindColor &&
      header.kind != kKindDepth) {
    return RecordValidation::kUnsupportedKind;
  }
  if (!hasValidDimensions(header)) {
    return RecordValidation::kInvalidDimensions;
  }
  if (!isValidTimestampSeconds(header.timestamp_s)) {
    return RecordValidation::kInvalidTimestamp;
  }
  if ((header.kind == kKindIntrinsics || header.kind == kKindDepth) &&
      !hasCanonicalDepthScale(header)) {
    return RecordValidation::kInvalidDepthScale;
  }
  if (header.payload_size > kMaxPayloadBytes) {
    return RecordValidation::kPayloadTooLarge;
  }

  if (header.kind == kKindIntrinsics) {
    if (header.format != kFormatUnknown) {
      return RecordValidation::kInvalidIntrinsicsFormat;
    }
    if (header.channels != 0) {
      return RecordValidation::kInvalidIntrinsicsChannels;
    }
    if (header.payload_size != 0) {
      return RecordValidation::kInvalidIntrinsicsPayload;
    }
    if (!std::isfinite(header.fx) || header.fx <= 0.0 ||
        !std::isfinite(header.fy) || header.fy <= 0.0 ||
        !std::isfinite(header.cx) || header.cx < 0.0 ||
        header.cx >= static_cast<double>(header.width) ||
        !std::isfinite(header.cy) || header.cy < 0.0 ||
        header.cy >= static_cast<double>(header.height) ||
        !std::isfinite(header.dist_k1) || !std::isfinite(header.dist_k2) ||
        !std::isfinite(header.dist_p1) || !std::isfinite(header.dist_p2) ||
        !std::isfinite(header.dist_k3)) {
      return RecordValidation::kInvalidIntrinsics;
    }
    return RecordValidation::kValid;
  }

  if (header.kind == kKindColor) {
    if (header.format != kFormatRgb8 && header.format != kFormatBgr8) {
      return RecordValidation::kInvalidColorFormat;
    }
    if (header.channels != 3) {
      return RecordValidation::kInvalidColorChannels;
    }
    std::uint64_t expected = 0;
    if (!checkedImagePayloadBytes(header, 3, &expected) ||
        expected > kMaxPayloadBytes) {
      return RecordValidation::kPayloadTooLarge;
    }
    return header.payload_size == expected
        ? RecordValidation::kValid
        : RecordValidation::kPayloadSizeMismatch;
  }

  if (header.format != kFormatDepthU16) {
    return RecordValidation::kInvalidDepthFormat;
  }
  if (header.channels != 1) {
    return RecordValidation::kInvalidDepthChannels;
  }
  std::uint64_t expected = 0;
  if (!checkedImagePayloadBytes(header, 2, &expected) ||
      expected > kMaxPayloadBytes) {
    return RecordValidation::kPayloadTooLarge;
  }
  return header.payload_size == expected
      ? RecordValidation::kValid
      : RecordValidation::kPayloadSizeMismatch;
}

struct CanonicalDepthLayout {
  RecordValidation validation{RecordValidation::kValid};
  std::size_t pixel_count{0};
};

inline CanonicalDepthLayout validateCanonicalDepthLayout(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t channels,
    std::uint32_t format,
    std::uint32_t payload_size) noexcept {
  auto header = makeRecordHeader(kKindDepth);
  header.width = width;
  header.height = height;
  header.channels = channels;
  header.format = format;
  header.timestamp_s = 1.0;
  header.payload_size = payload_size;
  const auto validation = validateRecordHeader(header);
  if (validation != RecordValidation::kValid) {
    return {validation, 0};
  }
  return {
      RecordValidation::kValid,
      static_cast<std::size_t>(payload_size / sizeof(std::uint16_t)),
  };
}

inline bool isValidSourceDepthScale(double scale_m) noexcept {
  return std::isfinite(scale_m) && scale_m > 0.0;
}

inline std::uint16_t normalizeDepthSampleMillimeters(
    const std::uint8_t* source,
    double source_scale_m) noexcept {
  std::uint16_t raw = 0;
  std::memcpy(&raw, source, sizeof(raw));
  if (raw == 0) {
    return 0;
  }
  const double millimeters =
      static_cast<double>(raw) * source_scale_m /
      kDepthScaleMetersPerMillimeter;
  const double bounded = std::min(
      millimeters,
      static_cast<double>(std::numeric_limits<std::uint16_t>::max()));
  return static_cast<std::uint16_t>(std::lround(bounded));
}

inline const char* recordValidationReason(RecordValidation validation) noexcept {
  switch (validation) {
    case RecordValidation::kValid:
      return "";
    case RecordValidation::kUnsupportedByteOrder:
      return "camera_record_requires_little_endian_host";
    case RecordValidation::kInvalidMagic:
      return "invalid_camera_record_magic";
    case RecordValidation::kUnsupportedVersion:
      return "unsupported_camera_record_version";
    case RecordValidation::kUnsupportedKind:
      return "unsupported_camera_record_kind";
    case RecordValidation::kInvalidDimensions:
      return "camera_record_dimensions_invalid";
    case RecordValidation::kInvalidTimestamp:
      return "camera_record_timestamp_invalid";
    case RecordValidation::kInvalidDepthScale:
      return "camera_record_depth_scale_must_be_0.001";
    case RecordValidation::kPayloadTooLarge:
      return "camera_record_payload_too_large";
    case RecordValidation::kMissingPayload:
      return "camera_record_payload_pointer_missing";
    case RecordValidation::kIntrinsicsRequired:
      return "camera_intrinsics_required_before_image";
    case RecordValidation::kInvalidIntrinsicsFormat:
      return "camera_intrinsics_format_invalid";
    case RecordValidation::kInvalidIntrinsicsChannels:
      return "camera_intrinsics_channels_invalid";
    case RecordValidation::kInvalidIntrinsicsPayload:
      return "camera_intrinsics_payload_invalid";
    case RecordValidation::kInvalidIntrinsics:
      return "camera_intrinsics_values_invalid";
    case RecordValidation::kInvalidColorFormat:
      return "camera_color_format_invalid";
    case RecordValidation::kInvalidColorChannels:
      return "camera_color_channels_invalid";
    case RecordValidation::kInvalidDepthFormat:
      return "camera_depth_format_invalid";
    case RecordValidation::kInvalidDepthChannels:
      return "camera_depth_channels_invalid";
    case RecordValidation::kPayloadSizeMismatch:
      return "camera_record_payload_size_mismatch";
  }
  return "camera_record_validation_unknown";
}

inline RecordDeadline makeRecordDeadline(int timeout_ms) noexcept {
  return RecordClock::now() + std::chrono::milliseconds(timeout_ms);
}

inline int remainingRecordTimeoutMs(
    RecordDeadline deadline,
    RecordDeadline now = RecordClock::now()) noexcept {
  if (now >= deadline) {
    return 0;
  }
  const auto remaining_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
  constexpr std::int64_t kNanosecondsPerMillisecond = 1000000;
  const std::int64_t rounded_ms =
      (remaining_ns + kNanosecondsPerMillisecond - 1) /
      kNanosecondsPerMillisecond;
  return static_cast<int>(std::min<std::int64_t>(
      std::max<std::int64_t>(rounded_ms, 1),
      std::numeric_limits<int>::max()));
}

enum class RecordWaitResult {
  kReady,
  kRetry,
  kTimeout,
  kEndOfStream,
};

enum class RecordReadResult {
  kComplete,
  kTimeout,
  kEndOfStream,
};

template <typename WaitReadable, typename ReadSome>
RecordReadResult readExactUntil(
    void* output,
    std::size_t size,
    RecordDeadline deadline,
    WaitReadable&& wait_readable,
    ReadSome&& read_some) {
  auto* destination = static_cast<std::uint8_t*>(output);
  std::size_t received = 0;
  while (received < size) {
    const int timeout_ms = remainingRecordTimeoutMs(deadline);
    if (timeout_ms <= 0) {
      return RecordReadResult::kTimeout;
    }
    switch (wait_readable(timeout_ms)) {
      case RecordWaitResult::kRetry:
        continue;
      case RecordWaitResult::kTimeout:
        return RecordReadResult::kTimeout;
      case RecordWaitResult::kEndOfStream:
        return RecordReadResult::kEndOfStream;
      case RecordWaitResult::kReady:
        break;
    }
    const std::ptrdiff_t count =
        read_some(destination + received, size - received);
    if (count < 0) {
      continue;
    }
    if (count == 0) {
      return RecordReadResult::kEndOfStream;
    }
    if (static_cast<std::size_t>(count) > size - received) {
      return RecordReadResult::kEndOfStream;
    }
    received += static_cast<std::size_t>(count);
  }
  return RecordReadResult::kComplete;
}

}  // namespace lingtu::drivers::camera::record
