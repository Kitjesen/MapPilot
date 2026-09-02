#include <chrono>
#include <cstdint>
#include <iostream>

#include "camera_record.hpp"

namespace {

namespace camera_record = lingtu::drivers::camera::record;
using camera_record::RecordHeader;

void writeRecord(std::uint16_t kind, std::uint32_t channels, std::uint32_t format,
                 const void *payload, std::uint32_t payload_size) {
  RecordHeader header = camera_record::makeRecordHeader(kind);
  header.width = 2;
  header.height = 2;
  header.channels = channels;
  header.format = format;
  header.timestamp_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  if (kind == camera_record::kKindIntrinsics) {
    header.fx = 500.0;
    header.fy = 501.0;
    header.cx = 1.0;
    header.cy = 1.5;
    header.depth_scale_m = camera_record::kDepthScaleMetersPerMillimeter;
    header.dist_k1 = 0.1;
    header.dist_k2 = 0.2;
    header.dist_p1 = 0.3;
    header.dist_p2 = 0.4;
    header.dist_k3 = 0.5;
  }
  header.payload_size = payload_size;
  std::cout.write(reinterpret_cast<const char *>(&header), sizeof(header));
  if (payload_size > 0) {
    std::cout.write(static_cast<const char *>(payload), payload_size);
  }
}

}  // namespace

int main() {
  const std::uint8_t color[] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
  };
  const std::uint16_t depth[] = {1000, 2000, 3000, 4000};
  writeRecord(camera_record::kKindIntrinsics, 0, camera_record::kFormatUnknown, nullptr, 0);
  writeRecord(camera_record::kKindColor, 3, camera_record::kFormatRgb8, color, sizeof(color));
  writeRecord(camera_record::kKindDepth, 1, camera_record::kFormatDepthU16, depth, sizeof(depth));
  std::cout.flush();
  return 0;
}
