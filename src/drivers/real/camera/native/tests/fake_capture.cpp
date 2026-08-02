#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

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

static_assert(sizeof(RecordHeader) == 116);

void writeRecord(
    std::uint16_t kind,
    std::uint32_t channels,
    std::uint32_t format,
    const void* payload,
    std::uint32_t payload_size) {
  RecordHeader header{};
  std::memcpy(header.magic, "LTOB", 4);
  header.version = 2;
  header.kind = kind;
  header.width = 2;
  header.height = 2;
  header.channels = channels;
  header.format = format;
  header.timestamp_s = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  if (kind == 1) {
    header.fx = 500.0;
    header.fy = 501.0;
    header.cx = 1.0;
    header.cy = 1.5;
    header.depth_scale_m = 0.001;
    header.dist_k1 = 0.1;
    header.dist_k2 = 0.2;
    header.dist_p1 = 0.3;
    header.dist_p2 = 0.4;
    header.dist_k3 = 0.5;
  }
  header.payload_size = payload_size;
  std::cout.write(reinterpret_cast<const char*>(&header), sizeof(header));
  if (payload_size > 0) {
    std::cout.write(static_cast<const char*>(payload), payload_size);
  }
}

}  // namespace

int main() {
  const std::uint8_t color[] = {
      0, 1, 2, 3, 4, 5,
      6, 7, 8, 9, 10, 11,
  };
  const std::uint16_t depth[] = {1000, 2000, 3000, 4000};
  writeRecord(1, 0, 0, nullptr, 0);
  writeRecord(2, 3, 1, color, sizeof(color));
  writeRecord(3, 1, 3, depth, sizeof(depth));
  std::cout.flush();
  return 0;
}
