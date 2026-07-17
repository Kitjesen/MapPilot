#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace lingtu::drivers::camera::shm {

inline constexpr char kMagic[8] = {'L', 'T', 'C', 'S', 'H', 'M', '0', '1'};
inline constexpr std::uint16_t kSchemaVersion = 1;
inline constexpr std::uint32_t kDefaultSlotCount = 2;
inline constexpr std::uint32_t kDefaultSlotCapacity = 8u * 1024u * 1024u;

enum class StreamKind : std::uint16_t {
  kInfo = 1,
  kColor = 2,
  kDepth = 3,
};

struct alignas(64) SharedHeader {
  char magic[8];
  std::uint16_t schema_version;
  std::uint16_t header_size;
  std::uint16_t slot_header_size;
  std::uint16_t slot_count;
  std::uint32_t slot_capacity;
  std::uint32_t active_slot;
  std::uint32_t flags;
  std::uint32_t reserved0;
  std::uint64_t published_sequence;
  std::uint64_t created_timestamp_ns;
  std::uint64_t writer_heartbeat_ns;
  std::uint64_t reserved1;
};

struct alignas(64) SlotHeader {
  std::uint64_t guard_begin;
  std::uint64_t sequence;
  std::uint64_t timestamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t stride;
  std::uint32_t payload_size;
  std::uint32_t payload_capacity;
  std::uint32_t payload_crc32;
  std::uint32_t flags;
  std::uint32_t reserved0;
  std::uint16_t stream_kind;
  std::uint16_t encoding_size;
  std::uint16_t schema_version;
  std::uint16_t header_size;
  char encoding[16];
  char frame_id[64];
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scale;
  double dist_k1;
  double dist_k2;
  double dist_p1;
  double dist_p2;
  double dist_k3;
  std::uint8_t reserved1[24];
  std::uint64_t guard_end;
};

static_assert(sizeof(SharedHeader) == 64, "camera SHM superblock must be 64 bytes");
static_assert(sizeof(SlotHeader) == 256, "camera SHM slot header must be 256 bytes");
static_assert(offsetof(SharedHeader, published_sequence) == 32);
static_assert(offsetof(SlotHeader, encoding) == 64);
static_assert(offsetof(SlotHeader, frame_id) == 80);
static_assert(offsetof(SlotHeader, guard_end) == 248);

struct FrameMetadata {
  StreamKind stream_kind{StreamKind::kColor};
  std::uint64_t timestamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  std::string_view encoding;
  std::string_view frame_id;
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double depth_scale{0.001};
  double dist_k1{0.0};
  double dist_k2{0.0};
  double dist_p1{0.0};
  double dist_p2{0.0};
  double dist_k3{0.0};
};

struct WriterConfig {
  std::string name;
  std::uint32_t slot_count{kDefaultSlotCount};
  std::uint32_t slot_capacity{kDefaultSlotCapacity};
  mode_t permissions{0660};
  bool unlink_on_destroy{false};
};

inline std::uint64_t unixTimeNs() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

inline std::uint32_t crc32(const void* data, std::size_t size) {
  auto crc = std::uint32_t{0xFFFFFFFFu};
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

class FrameWriter {
 public:
  explicit FrameWriter(WriterConfig config) : config_(std::move(config)) {
    validateConfig();
    open();
  }

  FrameWriter(const FrameWriter&) = delete;
  FrameWriter& operator=(const FrameWriter&) = delete;

  ~FrameWriter() {
    close();
  }

  std::uint64_t publish(
      const FrameMetadata& metadata,
      const void* payload,
      std::size_t payload_size) {
    validateFrame(metadata, payload, payload_size);
    if (last_sequence_ >= (std::numeric_limits<std::uint64_t>::max() - 1u) / 2u) {
      throw std::runtime_error("camera SHM sequence exhausted");
    }
    const std::uint64_t sequence = last_sequence_ + 1u;
    const std::uint32_t slot_index =
        static_cast<std::uint32_t>((sequence - 1u) % config_.slot_count);
    auto* slot = slotHeader(slot_index);
    auto* destination = slotPayload(slot_index);
    const std::uint64_t dirty_token = sequence * 2u + 1u;
    const std::uint64_t committed_token = sequence * 2u;

    atomicStore(&slot->guard_begin, dirty_token);
    atomicStore(&slot->guard_end, std::uint64_t{0});
    slot->sequence = sequence;
    slot->timestamp_ns = metadata.timestamp_ns == 0 ? unixTimeNs() : metadata.timestamp_ns;
    slot->width = metadata.width;
    slot->height = metadata.height;
    slot->stride = metadata.stride;
    slot->payload_size = static_cast<std::uint32_t>(payload_size);
    slot->payload_capacity = config_.slot_capacity;
    slot->payload_crc32 = payload_size == 0 ? 0u : crc32(payload, payload_size);
    slot->flags = 0;
    slot->reserved0 = 0;
    slot->stream_kind = static_cast<std::uint16_t>(metadata.stream_kind);
    slot->encoding_size = static_cast<std::uint16_t>(metadata.encoding.size());
    slot->schema_version = kSchemaVersion;
    slot->header_size = sizeof(SlotHeader);
    copyString(slot->encoding, sizeof(slot->encoding), metadata.encoding);
    copyString(slot->frame_id, sizeof(slot->frame_id), metadata.frame_id);
    slot->fx = metadata.fx;
    slot->fy = metadata.fy;
    slot->cx = metadata.cx;
    slot->cy = metadata.cy;
    slot->depth_scale = metadata.depth_scale;
    slot->dist_k1 = metadata.dist_k1;
    slot->dist_k2 = metadata.dist_k2;
    slot->dist_p1 = metadata.dist_p1;
    slot->dist_p2 = metadata.dist_p2;
    slot->dist_k3 = metadata.dist_k3;
    std::memset(slot->reserved1, 0, sizeof(slot->reserved1));
    if (payload_size > 0) {
      std::memcpy(destination, payload, payload_size);
    }

    std::atomic_thread_fence(std::memory_order_release);
    atomicStore(&slot->guard_end, committed_token);
    atomicStore(&slot->guard_begin, committed_token);
    atomicStore(&header_->writer_heartbeat_ns, unixTimeNs());
    atomicStore(&header_->active_slot, slot_index);
    atomicStore(&header_->published_sequence, sequence);
    last_sequence_ = sequence;
    return sequence;
  }

  std::uint64_t lastSequence() const noexcept {
    return last_sequence_;
  }

  const std::string& name() const noexcept {
    return config_.name;
  }

 private:
  template <typename T>
  static void atomicStore(T* address, T value) {
    __atomic_store_n(address, value, __ATOMIC_RELEASE);
  }

  template <typename T>
  static T atomicLoad(const T* address) {
    return __atomic_load_n(address, __ATOMIC_ACQUIRE);
  }

  void validateConfig() const {
    if (config_.name.size() < 2 || config_.name.front() != '/' ||
        config_.name.find('/', 1) != std::string::npos) {
      throw std::invalid_argument(
          "camera POSIX SHM name must have exactly one leading slash");
    }
    if (config_.slot_count < 2 ||
        config_.slot_count > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument("camera SHM slot_count must be in [2, 65535]");
    }
    if (config_.slot_capacity == 0) {
      throw std::invalid_argument("camera SHM slot_capacity must be positive");
    }
  }

  void open() {
    const std::size_t expected_size = mappingSize();
    fd_ = ::shm_open(config_.name.c_str(), O_CREAT | O_RDWR, config_.permissions);
    if (fd_ < 0) {
      throwSystemError("shm_open");
    }
    if (::ftruncate(fd_, static_cast<off_t>(expected_size)) != 0) {
      const int saved_errno = errno;
      close();
      errno = saved_errno;
      throwSystemError("ftruncate");
    }
    mapping_ = ::mmap(nullptr, expected_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      mapping_ = nullptr;
      const int saved_errno = errno;
      close();
      errno = saved_errno;
      throwSystemError("mmap");
    }
    mapping_size_ = expected_size;
    header_ = static_cast<SharedHeader*>(mapping_);
    if (!compatibleHeader()) {
      std::memset(mapping_, 0, mapping_size_);
      std::memcpy(header_->magic, kMagic, sizeof(kMagic));
      header_->schema_version = kSchemaVersion;
      header_->header_size = sizeof(SharedHeader);
      header_->slot_header_size = sizeof(SlotHeader);
      header_->slot_count = static_cast<std::uint16_t>(config_.slot_count);
      header_->slot_capacity = config_.slot_capacity;
      header_->created_timestamp_ns = unixTimeNs();
    }
    last_sequence_ = atomicLoad(&header_->published_sequence);
    atomicStore(&header_->writer_heartbeat_ns, unixTimeNs());
  }

  bool compatibleHeader() const {
    return std::memcmp(header_->magic, kMagic, sizeof(kMagic)) == 0 &&
        header_->schema_version == kSchemaVersion &&
        header_->header_size == sizeof(SharedHeader) &&
        header_->slot_header_size == sizeof(SlotHeader) &&
        header_->slot_count == config_.slot_count &&
        header_->slot_capacity == config_.slot_capacity;
  }

  void close() noexcept {
    if (mapping_ != nullptr) {
      ::munmap(mapping_, mapping_size_);
      mapping_ = nullptr;
      header_ = nullptr;
      mapping_size_ = 0;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    if (config_.unlink_on_destroy && !config_.name.empty()) {
      ::shm_unlink(config_.name.c_str());
    }
  }

  std::size_t mappingSize() const {
    return sizeof(SharedHeader) +
        static_cast<std::size_t>(config_.slot_count) *
            (sizeof(SlotHeader) + config_.slot_capacity);
  }

  SlotHeader* slotHeader(std::uint32_t index) const {
    auto* base = static_cast<std::uint8_t*>(mapping_) + sizeof(SharedHeader);
    return reinterpret_cast<SlotHeader*>(
        base + static_cast<std::size_t>(index) *
            (sizeof(SlotHeader) + config_.slot_capacity));
  }

  std::uint8_t* slotPayload(std::uint32_t index) const {
    return reinterpret_cast<std::uint8_t*>(slotHeader(index)) + sizeof(SlotHeader);
  }

  void validateFrame(
      const FrameMetadata& metadata,
      const void* payload,
      std::size_t payload_size) const {
    if (metadata.encoding.size() > 15 || metadata.frame_id.size() > 63) {
      throw std::invalid_argument("camera SHM encoding or frame_id is too long");
    }
    if (payload_size > config_.slot_capacity ||
        payload_size > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("camera SHM payload exceeds slot capacity");
    }
    if (payload_size > 0 && payload == nullptr) {
      throw std::invalid_argument("camera SHM payload pointer is null");
    }
    if (metadata.stream_kind == StreamKind::kInfo) {
      if (metadata.width == 0 || metadata.height == 0 || metadata.stride != 0 ||
          payload_size != 0) {
        throw std::invalid_argument("camera info SHM frame geometry is invalid");
      }
      return;
    }
    if (metadata.width == 0 || metadata.height == 0 || metadata.stride == 0 ||
        payload_size != static_cast<std::size_t>(metadata.stride) * metadata.height) {
      throw std::invalid_argument("camera image SHM payload must equal stride * height");
    }
    const std::uint32_t bytes_per_pixel = bytesPerPixel(metadata.encoding);
    if (bytes_per_pixel == 0 || metadata.stride < metadata.width * bytes_per_pixel) {
      throw std::invalid_argument("camera image SHM encoding or stride is invalid");
    }
  }

  static std::uint32_t bytesPerPixel(std::string_view encoding) {
    if (encoding == "rgb8" || encoding == "bgr8") {
      return 3;
    }
    if (encoding == "rgba8" || encoding == "32FC1") {
      return 4;
    }
    if (encoding == "16UC1") {
      return 2;
    }
    if (encoding == "mono8" || encoding == "8UC1") {
      return 1;
    }
    return 0;
  }

  static void copyString(char* destination, std::size_t capacity, std::string_view value) {
    std::memset(destination, 0, capacity);
    if (!value.empty()) {
      std::memcpy(destination, value.data(), value.size());
    }
  }

  [[noreturn]] static void throwSystemError(const char* operation) {
    throw std::runtime_error(
        std::string("camera SHM ") + operation + " failed: " + std::strerror(errno));
  }

  WriterConfig config_;
  int fd_{-1};
  void* mapping_{nullptr};
  std::size_t mapping_size_{0};
  SharedHeader* header_{nullptr};
  std::uint64_t last_sequence_{0};
};

}  // namespace lingtu::drivers::camera::shm
