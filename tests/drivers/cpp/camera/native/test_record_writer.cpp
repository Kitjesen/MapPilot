#include <array>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "record_writer.hpp"

namespace record = lingtu::drivers::camera::record;

namespace {
int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "record writer failed at " << line << ": " << expression << '\n';
    ++failures;
  }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

template <typename Call>
void expectError(Call call, const char* reason) {
  try {
    call();
    CHECK(false);
  } catch (const std::runtime_error& error) {
    CHECK(std::string(error.what()) == reason);
  }
}

struct FlushFailure : std::stringbuf {
  int sync() override { return -1; }
};

record::RecordHeader header(std::uint16_t kind) {
  auto h = record::makeRecordHeader(kind);
  h.width = 2;
  h.height = 2;
  h.timestamp_s = 123.25;
  return h;
}
}  // namespace

int main() {
  std::ostringstream stream(std::ios::binary);
  record::RecordWriter writer(stream);
  auto info = header(record::kKindIntrinsics);
  info.fx = 500;
  info.fy = 501;
  info.cx = 1;
  info.cy = 1;
  const std::array<std::uint8_t, 12> color{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}};
  const std::array<std::uint16_t, 4> depth{{0, 1000, 2000, 65535}};
  auto image = header(record::kKindColor);
  image.channels = 3;
  image.format = record::kFormatBgr8;
  image.payload_size = sizeof(color);
  auto distance = header(record::kKindDepth);
  distance.channels = 1;
  distance.format = record::kFormatDepthU16;
  distance.payload_size = sizeof(depth);
  writer.write(info);
  writer.write(image, color.data());
  writer.write(distance, depth.data());
  writer.flush();

  std::string expected;
  expected.append(reinterpret_cast<const char*>(&info), sizeof(info));
  expected.append(reinterpret_cast<const char*>(&image), sizeof(image));
  expected.append(reinterpret_cast<const char*>(color.data()), sizeof(color));
  expected.append(reinterpret_cast<const char*>(&distance), sizeof(distance));
  expected.append(reinterpret_cast<const char*>(depth.data()), sizeof(depth));
  CHECK(stream.str() == expected);
  CHECK(stream.str().size() == 3 * 116 + sizeof(color) + sizeof(depth));

  const auto before = stream.str();
  auto invalid = image;
  invalid.payload_size = 1;
  expectError([&] { writer.write(invalid, color.data()); },
              record::recordValidationReason(record::RecordValidation::kPayloadSizeMismatch));
  expectError([&] { writer.write(image); },
              record::recordValidationReason(record::RecordValidation::kMissingPayload));
  CHECK(stream.str() == before);

  std::ostringstream closed;
  closed.setstate(std::ios::badbit);
  record::RecordWriter failed(closed);
  expectError([&] { failed.write(info); }, "camera_record_output_failed");
  FlushFailure failure;
  std::ostream buffered(&failure);
  record::RecordWriter flush_writer(buffered);
  flush_writer.write(info);
  expectError([&] { flush_writer.flush(); }, "camera_record_output_failed");
  return failures == 0 ? 0 : 1;
}
