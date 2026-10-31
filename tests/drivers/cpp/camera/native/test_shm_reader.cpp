#include "shm_frame_ring.hpp"
#include "shm_reader.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace camera_shm = lingtu::drivers::camera::shm;

namespace {

camera_shm::FrameMetadata metadata(
    std::uint64_t timestamp_ns,
    std::string_view encoding = "rgb8",
    std::uint32_t stride = 8) {
  camera_shm::FrameMetadata value;
  value.stream_kind = camera_shm::StreamKind::kColor;
  value.timestamp_ns = timestamp_ns;
  value.width = 2;
  value.height = 2;
  value.stride = stride;
  value.encoding = encoding;
  value.frame_id = "camera_link";
  return value;
}

std::vector<std::uint8_t> payload(std::uint8_t seed, std::uint32_t stride = 8) {
  std::vector<std::uint8_t> value(static_cast<std::size_t>(stride) * 2u);
  for (std::size_t i = 0; i < value.size(); ++i) {
    value[i] = static_cast<std::uint8_t>(seed + i);
  }
  return value;
}

void overwrite(
    const std::string& name,
    const void* data,
    std::size_t size,
    std::size_t offset) {
  const int fd = ::shm_open(name.c_str(), O_RDWR | O_CLOEXEC, 0);
  assert(fd >= 0);
  const auto count = ::pwrite(fd, data, size, static_cast<off_t>(offset));
  assert(count == static_cast<ssize_t>(size));
  assert(::close(fd) == 0);
}

std::size_t slotOffset(std::uint64_t sequence, std::uint32_t capacity = 128) {
  const auto slot = static_cast<std::size_t>((sequence - 1u) % 2u);
  return sizeof(camera_shm::SharedHeader) +
      slot * (sizeof(camera_shm::SlotHeader) + capacity);
}

}  // namespace

int main() {
  const std::string name = "/lingtu_camera_reader_test_" + std::to_string(::getpid());
  ::shm_unlink(name.c_str());
  camera_shm::ColorReader reader(name);
  camera_shm::ColorFrame frame;
  assert(!reader.readLatest(frame, 1000, 100));

  {
    camera_shm::FrameWriter writer({name, 2, 128, 0600, false, 101});
    const auto first = payload(1);
    writer.publish(metadata(900), first.data(), first.size());
    const auto second = payload(20);
    writer.publish(metadata(950, "bgr8"), second.data(), second.size());
    const auto third = payload(40);
    writer.publish(metadata(990), third.data(), third.size());

    assert(reader.readLatest(frame, 1000, 100));
    assert(frame.source_generation == 101);
    assert(frame.sequence == 3);
    assert(frame.timestamp_ns == 990);
    assert(frame.width == 2 && frame.height == 2 && frame.stride == 8);
    assert(frame.encoding == "rgb8");
    assert(frame.frame_id == "camera_link");
    assert(frame.payload == third);
    assert(!reader.readLatest(frame, 1000, 100));

    const auto old = payload(60);
    writer.publish(metadata(700), old.data(), old.size());
    assert(!reader.readLatest(frame, 1000, 100));

    const auto future = payload(70);
    writer.publish(metadata(1100), future.data(), future.size());
    assert(!reader.readLatest(frame, 1000, 100));

    const auto padded = payload(80, 10);
    writer.publish(metadata(1000, "bgr8", 10), padded.data(), padded.size());
    assert(reader.readLatest(frame, 1000, 100));
    assert(frame.sequence == 6);
    assert(frame.encoding == "bgr8");
    assert(frame.stride == 10);
    assert(frame.payload == padded);
  }

  {
    camera_shm::FrameWriter regressed({name, 2, 128, 0600, false, 101});
    const auto regressed_payload = payload(90);
    regressed.publish(metadata(1050), regressed_payload.data(), regressed_payload.size());
    assert(!reader.readLatest(frame, 1050, 100));
  }

  {
    camera_shm::FrameWriter restarted({name, 2, 128, 0600, false, 102});
    const auto restarted_payload = payload(100);
    restarted.publish(metadata(1100), restarted_payload.data(), restarted_payload.size());
    assert(reader.readLatest(frame, 1100, 100));
    assert(frame.source_generation == 102);
    assert(frame.sequence == 1);
    assert(frame.payload == restarted_payload);
  }

  assert(::shm_unlink(name.c_str()) == 0);
  {
    camera_shm::FrameWriter recreated({name, 2, 128, 0600, false, 103});
    const auto recreated_payload = payload(120);
    recreated.publish(metadata(1200), recreated_payload.data(), recreated_payload.size());
    assert(reader.readLatest(frame, 1200, 100));
    assert(frame.source_generation == 103);
    assert(frame.sequence == 1);
    assert(frame.payload == recreated_payload);

    const auto corrupt_payload = payload(140);
    const auto corrupt_sequence = recreated.publish(
        metadata(1300), corrupt_payload.data(), corrupt_payload.size());
    const std::uint8_t corrupt_byte = 0xff;
    overwrite(
        name,
        &corrupt_byte,
        sizeof(corrupt_byte),
        slotOffset(corrupt_sequence) + sizeof(camera_shm::SlotHeader));
    assert(!reader.readLatest(frame, 1300, 100));

    const auto torn_payload = payload(160);
    const auto torn_sequence = recreated.publish(
        metadata(1400), torn_payload.data(), torn_payload.size());
    const std::uint64_t dirty_guard = torn_sequence * 2u + 1u;
    overwrite(
        name,
        &dirty_guard,
        sizeof(dirty_guard),
        slotOffset(torn_sequence) + offsetof(camera_shm::SlotHeader, guard_begin));
    assert(!reader.readLatest(frame, 1400, 100));
  }

  assert(::shm_unlink(name.c_str()) == 0);
  return 0;
}
