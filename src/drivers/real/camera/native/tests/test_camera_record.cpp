#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

#include "../camera_record.hpp"

namespace {

namespace record = lingtu::drivers::camera::record;

int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (condition) {
    return;
  }
  std::cerr << "camera record contract failed at line " << line << ": "
            << expression << '\n';
  ++failures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

record::RecordHeader validIntrinsics() {
  auto header = record::makeRecordHeader(record::kKindIntrinsics);
  header.width = 640;
  header.height = 480;
  header.channels = 0;
  header.format = record::kFormatUnknown;
  header.timestamp_s = 1.0;
  header.fx = 615.0;
  header.fy = 615.0;
  header.cx = 320.0;
  header.cy = 240.0;
  header.payload_size = 0;
  return header;
}

record::RecordHeader validColor() {
  auto header = record::makeRecordHeader(record::kKindColor);
  header.width = 2;
  header.height = 2;
  header.channels = 3;
  header.format = record::kFormatRgb8;
  header.timestamp_s = 1.0;
  header.payload_size = 12;
  return header;
}

record::RecordHeader validDepth() {
  auto header = record::makeRecordHeader(record::kKindDepth);
  header.width = 2;
  header.height = 2;
  header.channels = 1;
  header.format = record::kFormatDepthU16;
  header.timestamp_s = 1.0;
  header.payload_size = 8;
  return header;
}

}  // namespace

int main() {
  using record::RecordValidation;

  CHECK(record::kWireByteOrder == "little-endian");
  CHECK(record::nativeByteOrderIsLittleEndian());
  CHECK(sizeof(record::RecordHeader) == 116);
  CHECK(offsetof(record::RecordHeader, version) == 4);
  CHECK(offsetof(record::RecordHeader, width) == 8);
  CHECK(offsetof(record::RecordHeader, payload_size) == 72);
  CHECK(offsetof(record::RecordHeader, dist_k3) == 108);

  auto header = validIntrinsics();
  CHECK(record::hasMagic(header));
  CHECK(record::hasValidPreamble(header));
  CHECK(record::validateRecordHeader(header) == RecordValidation::kValid);

  auto color = validColor();
  CHECK(record::validateRecordHeader(color) == RecordValidation::kValid);
  color.format = record::kFormatBgr8;
  CHECK(record::validateRecordHeader(color) == RecordValidation::kValid);
  color.format = record::kFormatDepthU16;
  CHECK(record::validateRecordHeader(color) == RecordValidation::kInvalidColorFormat);
  color = validColor();
  color.channels = 4;
  CHECK(record::validateRecordHeader(color) == RecordValidation::kInvalidColorChannels);
  color = validColor();
  color.payload_size -= 1;
  CHECK(record::validateRecordHeader(color) == RecordValidation::kPayloadSizeMismatch);
  color = validColor();
  color.payload_size = record::kMaxPayloadBytes + 1;
  CHECK(record::validateRecordHeader(color) == RecordValidation::kPayloadTooLarge);

  auto depth = validDepth();
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kValid);
  depth.format = record::kFormatRgb8;
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kInvalidDepthFormat);
  depth = validDepth();
  depth.channels = 3;
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kInvalidDepthChannels);
  depth = validDepth();
  depth.payload_size += 2;
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kPayloadSizeMismatch);
  depth = validDepth();
  depth.depth_scale_m = 0.002;
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kInvalidDepthScale);
  depth = validDepth();
  depth.depth_scale_m = std::numeric_limits<double>::quiet_NaN();
  CHECK(record::validateRecordHeader(depth) == RecordValidation::kInvalidDepthScale);

  const auto valid_layout = record::validateCanonicalDepthLayout(
      2, 2, 1, record::kFormatDepthU16, 8);
  CHECK(valid_layout.validation == RecordValidation::kValid);
  CHECK(valid_layout.pixel_count == 4);
  auto overflow_header = validDepth();
  overflow_header.width = std::numeric_limits<std::uint32_t>::max();
  overflow_header.height = std::numeric_limits<std::uint32_t>::max();
  std::uint64_t checked_payload_bytes = 123;
  CHECK(!record::checkedImagePayloadBytes(
      overflow_header, 2, &checked_payload_bytes));
  CHECK(checked_payload_bytes == 0);
  const auto oversized_dimensions = record::validateCanonicalDepthLayout(
      std::numeric_limits<std::uint32_t>::max(),
      std::numeric_limits<std::uint32_t>::max(),
      1,
      record::kFormatDepthU16,
      record::kMaxPayloadBytes);
  CHECK(oversized_dimensions.validation == RecordValidation::kInvalidDimensions);
  CHECK(oversized_dimensions.pixel_count == 0);
  const auto oversized_payload = record::validateCanonicalDepthLayout(
      record::kMaxDimension,
      record::kMaxDimension,
      1,
      record::kFormatDepthU16,
      record::kMaxPayloadBytes);
  CHECK(oversized_payload.validation == RecordValidation::kPayloadTooLarge);
  CHECK(oversized_payload.pixel_count == 0);
  const auto mismatched_payload = record::validateCanonicalDepthLayout(
      2, 2, 1, record::kFormatDepthU16, 6);
  CHECK(mismatched_payload.validation == RecordValidation::kPayloadSizeMismatch);
  CHECK(mismatched_payload.pixel_count == 0);

  CHECK(record::isValidSourceDepthScale(0.001));
  CHECK(!record::isValidSourceDepthScale(0.0));
  CHECK(!record::isValidSourceDepthScale(
      std::numeric_limits<double>::quiet_NaN()));
  const std::array<std::uint8_t, 7> unaligned_depth{{
      0xff,
      0x00, 0x00,
      0xe8, 0x03,
      0x40, 0x9c,
  }};
  CHECK(record::normalizeDepthSampleMillimeters(
            unaligned_depth.data() + 1, 0.002) == 0);
  CHECK(record::normalizeDepthSampleMillimeters(
            unaligned_depth.data() + 3, 0.002) == 2000);
  CHECK(record::normalizeDepthSampleMillimeters(
            unaligned_depth.data() + 5, 0.002) ==
        std::numeric_limits<std::uint16_t>::max());

  header = validIntrinsics();
  header.kind = 99;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kUnsupportedKind);
  header = validIntrinsics();
  header.width = 0;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidDimensions);
  header = validIntrinsics();
  header.format = record::kFormatRgb8;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsicsFormat);
  header = validIntrinsics();
  header.channels = 1;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsicsChannels);
  header = validIntrinsics();
  header.payload_size = 1;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsicsPayload);
  header = validIntrinsics();
  header.fx = std::numeric_limits<double>::quiet_NaN();
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsics);
  header = validIntrinsics();
  header.cx = static_cast<double>(header.width);
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsics);
  header = validIntrinsics();
  header.dist_k1 = std::numeric_limits<double>::infinity();
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidIntrinsics);
  header = validIntrinsics();
  header.depth_scale_m = 0.002;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidDepthScale);

  header = validIntrinsics();
  header.timestamp_s = 0.0;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidTimestamp);
  header = validIntrinsics();
  header.timestamp_s = std::numeric_limits<double>::quiet_NaN();
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidTimestamp);
  header = validIntrinsics();
  header.timestamp_s = std::numeric_limits<double>::infinity();
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidTimestamp);
  header = validIntrinsics();
  header.timestamp_s = record::kMaxTimestampSeconds;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kValid);
  header = validIntrinsics();
  header.timestamp_s = record::kMaxTimestampSeconds + 1.0;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidTimestamp);

  header = validIntrinsics();
  header.magic[0] = 'X';
  CHECK(record::validateRecordHeader(header) == RecordValidation::kInvalidMagic);
  header = validIntrinsics();
  header.version += 1;
  CHECK(record::validateRecordHeader(header) == RecordValidation::kUnsupportedVersion);

  const std::uint8_t payload[12]{};
  color = validColor();
  CHECK(record::validateRecordPayloadPointer(color, payload) == RecordValidation::kValid);
  CHECK(record::validateRecordPayloadPointer(color, nullptr) == RecordValidation::kMissingPayload);
  header = validIntrinsics();
  CHECK(record::validateRecordPayloadPointer(header, nullptr) == RecordValidation::kValid);
  CHECK(std::string_view(record::recordValidationReason(RecordValidation::kMissingPayload)) ==
        "camera_record_payload_pointer_missing");
  color = validColor();
  depth = validDepth();
  header = validIntrinsics();
  CHECK(record::validateRecordSequence(header, false) == RecordValidation::kValid);
  CHECK(record::validateRecordSequence(color, false) == RecordValidation::kIntrinsicsRequired);
  CHECK(record::validateRecordSequence(depth, false) == RecordValidation::kIntrinsicsRequired);
  CHECK(record::validateRecordSequence(color, true) == RecordValidation::kValid);
  CHECK(std::string_view(record::recordValidationReason(RecordValidation::kIntrinsicsRequired)) ==
        "camera_intrinsics_required_before_image");

  auto golden = validColor();
  golden.width = 0x01020304u;
  golden.height = 1;
  golden.payload_size = 3;
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&golden);
  CHECK(bytes[0] == 'L' && bytes[1] == 'T' && bytes[2] == 'O' && bytes[3] == 'B');
  CHECK(bytes[4] == 0x02 && bytes[5] == 0x00);
  CHECK(bytes[6] == 0x02 && bytes[7] == 0x00);
  CHECK(bytes[8] == 0x04 && bytes[9] == 0x03 && bytes[10] == 0x02 && bytes[11] == 0x01);
  CHECK(bytes[72] == 0x03 && bytes[73] == 0x00 && bytes[74] == 0x00 && bytes[75] == 0x00);

  const record::RecordDeadline start{};
  const auto deadline = start + std::chrono::milliseconds(100);
  CHECK(record::remainingRecordTimeoutMs(deadline, start) == 100);
  CHECK(record::remainingRecordTimeoutMs(
            deadline, start + std::chrono::microseconds(99500)) == 1);
  CHECK(record::remainingRecordTimeoutMs(deadline, deadline) == 0);
  CHECK(record::remainingRecordTimeoutMs(
            deadline, deadline + std::chrono::milliseconds(1)) == 0);
  CHECK(!record::isValidRecordTimeoutMs(0));
  CHECK(record::isValidRecordTimeoutMs(record::kMinRecordTimeoutMs));
  CHECK(record::isValidRecordTimeoutMs(record::kMaxRecordTimeoutMs));
  CHECK(!record::isValidRecordTimeoutMs(record::kMaxRecordTimeoutMs + 1));

  return failures == 0 ? 0 : 1;
}