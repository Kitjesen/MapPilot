#pragma once

#include "shm_frame_ring.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::drivers::camera::shm {

struct ColorFrame {
  std::uint64_t source_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  std::string encoding;
  std::string frame_id;
  std::vector<std::uint8_t> payload;
};

class ColorReader {
 public:
  explicit ColorReader(std::string name) : name_(std::move(name)) {
    if (name_.size() < 2 || name_.front() != '/' ||
        name_.find('/', 1) != std::string::npos ||
        name_.find('\0') != std::string::npos) {
      throw std::invalid_argument(
          "camera POSIX SHM name must have exactly one leading slash");
    }
  }

  ColorReader(const ColorReader&) = delete;
  ColorReader& operator=(const ColorReader&) = delete;
  ColorReader(ColorReader&&) = delete;
  ColorReader& operator=(ColorReader&&) = delete;
  ~ColorReader() = default;

  bool readLatest(ColorFrame& out, std::uint64_t now_ns, std::uint64_t max_age_ns) {
    const int fd = ::shm_open(name_.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (fd < 0) {
      if (errno == ENOENT) {
        return false;
      }
      throwSystemError("shm_open");
    }
    File file(fd);

    SharedHeader header{};
    if (!readExact(file.fd, &header, sizeof(header), 0) || !validHeader(header)) {
      return false;
    }
    const std::size_t slot_size = sizeof(SlotHeader) + header.slot_capacity;
    if (header.slot_count >
        (std::numeric_limits<std::size_t>::max() - sizeof(SharedHeader)) / slot_size) {
      return false;
    }
    const std::size_t expected_size =
        sizeof(SharedHeader) + static_cast<std::size_t>(header.slot_count) * slot_size;
    struct stat info {};
    if (::fstat(file.fd, &info) != 0) {
      throwSystemError("fstat");
    }
    if (info.st_size < 0 || static_cast<std::uintmax_t>(info.st_size) != expected_size ||
        header.source_generation == 0 || header.published_sequence == 0) {
      return false;
    }
    if (header.source_generation == last_generation_ &&
        header.published_sequence <= last_sequence_) {
      return false;
    }

    const std::uint64_t sequence = header.published_sequence;
    const std::uint32_t slot_index =
        static_cast<std::uint32_t>((sequence - 1u) % header.slot_count);
    const std::size_t slot_offset = sizeof(SharedHeader) +
        static_cast<std::size_t>(slot_index) * slot_size;
    SlotHeader slot{};
    if (!readExact(file.fd, &slot, sizeof(slot), slot_offset)) {
      return false;
    }
    const std::uint64_t expected_guard = sequence * 2u;
    if (sequence > std::numeric_limits<std::uint64_t>::max() / 2u ||
        slot.guard_begin != expected_guard || slot.guard_end != expected_guard ||
        (slot.guard_begin & 1u) != 0 || slot.sequence != sequence ||
        slot.schema_version != kSchemaVersion || slot.header_size != sizeof(SlotHeader) ||
        slot.stream_kind != static_cast<std::uint16_t>(StreamKind::kColor) ||
        slot.payload_capacity != header.slot_capacity ||
        slot.payload_size > header.slot_capacity) {
      return false;
    }

    if (slot.encoding_size == 0 || slot.encoding_size >= sizeof(slot.encoding) ||
        slot.encoding[slot.encoding_size] != '\0') {
      return false;
    }
    const std::string encoding(slot.encoding, slot.encoding_size);
    if (encoding != "rgb8" && encoding != "bgr8") {
      return false;
    }
    const void* frame_id_end = std::memchr(slot.frame_id, '\0', sizeof(slot.frame_id));
    if (frame_id_end == nullptr || slot.timestamp_ns == 0 || slot.width == 0 ||
        slot.height == 0 || slot.width > std::numeric_limits<std::uint32_t>::max() / 3u ||
        slot.stride < slot.width * 3u ||
        slot.height > std::numeric_limits<std::size_t>::max() / slot.stride ||
        slot.payload_size != static_cast<std::size_t>(slot.stride) * slot.height) {
      return false;
    }
    if (now_ns < slot.timestamp_ns ||
        (max_age_ns != 0 && now_ns - slot.timestamp_ns > max_age_ns)) {
      return false;
    }

    std::vector<std::uint8_t> payload(slot.payload_size);
    if (!payload.empty() &&
        !readExact(file.fd, payload.data(), payload.size(), slot_offset + sizeof(SlotHeader))) {
      return false;
    }
    std::uint64_t final_begin = 0;
    std::uint64_t final_end = 0;
    SharedHeader final_header{};
    if (!readExact(file.fd, &final_begin, sizeof(final_begin),
                   slot_offset + offsetof(SlotHeader, guard_begin)) ||
        !readExact(file.fd, &final_end, sizeof(final_end),
                   slot_offset + offsetof(SlotHeader, guard_end)) ||
        final_begin != expected_guard || final_end != expected_guard ||
        crc32(payload.data(), payload.size()) != slot.payload_crc32 ||
        !readExact(file.fd, &final_header, sizeof(final_header), 0) ||
        !validHeader(final_header) ||
        final_header.source_generation != header.source_generation ||
        final_header.slot_count != header.slot_count ||
        final_header.slot_capacity != header.slot_capacity) {
      return false;
    }

    ColorFrame frame;
    frame.source_generation = header.source_generation;
    frame.sequence = sequence;
    frame.timestamp_ns = slot.timestamp_ns;
    frame.width = slot.width;
    frame.height = slot.height;
    frame.stride = slot.stride;
    frame.encoding = encoding;
    frame.frame_id.assign(
        slot.frame_id,
        static_cast<std::size_t>(
            static_cast<const char*>(frame_id_end) - slot.frame_id));
    frame.payload = std::move(payload);
    out = std::move(frame);
    last_generation_ = header.source_generation;
    last_sequence_ = sequence;
    return true;
  }

 private:
  struct File {
    explicit File(int input_fd) : fd(input_fd) {}
    File(const File&) = delete;
    File& operator=(const File&) = delete;
    ~File() {
      if (fd >= 0) {
        ::close(fd);
      }
    }
    int fd;
  };

  static bool readExact(int fd, void* destination, std::size_t size, std::size_t offset) {
    auto* output = static_cast<std::uint8_t*>(destination);
    std::size_t read_size = 0;
    while (read_size < size) {
      const auto count = ::pread(
          fd,
          output + read_size,
          size - read_size,
          static_cast<off_t>(offset + read_size));
      if (count == 0) {
        return false;
      }
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      read_size += static_cast<std::size_t>(count);
    }
    return true;
  }

  static bool validHeader(const SharedHeader& header) {
    return std::memcmp(header.magic, kMagic, sizeof(kMagic)) == 0 &&
        header.schema_version == kSchemaVersion &&
        header.header_size == sizeof(SharedHeader) &&
        header.slot_header_size == sizeof(SlotHeader) && header.slot_count >= 2 &&
        header.slot_capacity > 0 && header.active_slot < header.slot_count;
  }

  [[noreturn]] static void throwSystemError(const char* operation) {
    throw std::runtime_error(
        std::string("camera SHM reader ") + operation + " failed: " + std::strerror(errno));
  }

  std::string name_;
  std::uint64_t last_generation_{0};
  std::uint64_t last_sequence_{0};
};

}  // namespace lingtu::drivers::camera::shm
