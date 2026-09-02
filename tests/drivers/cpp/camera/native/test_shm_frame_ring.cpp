#include "shm_frame_ring.hpp"

#include <sys/mman.h>

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <string>

int main() {
  using namespace lingtu::drivers::camera::shm;
  const std::string name = "/lingtu_camera_shm_test_" + std::to_string(::getpid());
  const std::uint8_t payload[] = {
      0, 1, 2, 3, 4, 5,
      6, 7, 8, 9, 10, 11,
  };
  {
    FrameWriter writer(WriterConfig{name, 2, 128, 0600, false});
    FrameMetadata metadata;
    metadata.stream_kind = StreamKind::kColor;
    metadata.timestamp_ns = unixTimeNs();
    metadata.width = 2;
    metadata.height = 2;
    metadata.stride = 6;
    metadata.encoding = "rgb8";
    metadata.frame_id = "camera_link";
    assert(writer.publish(metadata, payload, sizeof(payload)) == 1);
    metadata.timestamp_ns = unixTimeNs();
    assert(writer.publish(metadata, payload, sizeof(payload)) == 2);
    assert(writer.lastSequence() == 2);

    bool rejected = false;
    metadata.stride = 5;
    try {
      writer.publish(metadata, payload, sizeof(payload));
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    assert(rejected);
  }
  ::shm_unlink(name.c_str());
  return 0;
}
