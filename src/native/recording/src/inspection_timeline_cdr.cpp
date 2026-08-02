#include "lingtu/recording/inspection_timeline_cdr.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "message/cpp/dds_topics.hpp"

namespace lingtu::recording {
namespace {

constexpr std::size_t kMaximumTimelineFacts = 1'000'000U;
constexpr std::size_t kMaximumTimelineStringBytes = 64U * 1024U;

std::uint64_t observed_time(const RecordedMessage &message) {
  return message.publish_time_ns == 0 ? message.log_time_ns : message.publish_time_ns;
}

template <typename Fact>
void append_fact(std::vector<Fact> &facts, Fact fact) {
  if (facts.size() >= kMaximumTimelineFacts) {
    throw std::runtime_error("inspection timeline evidence capacity exceeded");
  }
  facts.push_back(std::move(fact));
}

class Xcdr1LittleEndianReader {
 public:
  explicit Xcdr1LittleEndianReader(const std::vector<std::byte> &payload) : payload_(payload) {
    if (payload_.size() <= kEncapsulationBytes) {
      fail("CDR payload size is invalid");
    }
    if (payload_[0] != std::byte{0x00} || payload_[1] != std::byte{0x01}) {
      fail("inspection timeline requires little-endian XCDR1 payloads");
    }
    offset_ = kEncapsulationBytes;
  }

  void finish() const {
    const std::size_t relative = offset_ - kEncapsulationBytes;
    const std::size_t maximum_padding = (4U - (relative % 4U)) % 4U;
    if (remaining() > maximum_padding) {
      fail("CDR payload has trailing bytes");
    }
    for (std::size_t index = offset_; index < payload_.size(); ++index) {
      if (payload_[index] != std::byte{0x00}) {
        fail("CDR payload has nonzero alignment padding");
      }
    }
  }

  void skip_header() {
    (void)read_int32();
    (void)read_uint32();
    (void)read_string();
  }

  bool read_bool() {
    const auto value = read_uint8();
    if (value > 1U) {
      fail("CDR boolean value is invalid");
    }
    return value != 0U;
  }

  std::int32_t read_int32() {
    return static_cast<std::int32_t>(read_little_endian<std::uint32_t>(4U));
  }

  std::uint32_t read_uint32() { return read_little_endian<std::uint32_t>(4U); }

  std::int64_t read_int64() {
    return static_cast<std::int64_t>(read_little_endian<std::uint64_t>(8U));
  }

  std::uint64_t read_uint64() { return read_little_endian<std::uint64_t>(8U); }

  double read_double() {
    const auto bits = read_little_endian<std::uint64_t>(8U);
    double value = 0.0;
    static_assert(sizeof(value) == sizeof(bits), "double must be IEEE-754 binary64 sized");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::string read_string() {
    const std::uint32_t byte_count = read_uint32();
    if (byte_count == 0U) {
      fail("CDR string length is invalid");
    }
    if (byte_count > kMaximumTimelineStringBytes) {
      fail("CDR string exceeds inspection timeline bound");
    }
    if (remaining() < byte_count) {
      fail("CDR string overruns payload");
    }
    const auto *begin = reinterpret_cast<const char *>(payload_.data() + offset_);
    if (begin[byte_count - 1U] != '\0') {
      fail("CDR string is not nul-terminated");
    }
    offset_ += byte_count;
    return std::string(begin, begin + byte_count - 1U);
  }

 private:
  static constexpr std::size_t kEncapsulationBytes = 4U;

  [[noreturn]] static void fail(const char *message) { throw std::runtime_error(message); }

  std::size_t remaining() const { return payload_.size() - offset_; }

  void align(std::size_t alignment) {
    const std::size_t relative = offset_ - kEncapsulationBytes;
    const std::size_t padding = (alignment - (relative % alignment)) % alignment;
    if (padding > remaining()) {
      fail("CDR alignment overruns payload");
    }
    offset_ += padding;
  }

  std::uint8_t read_uint8() {
    if (remaining() < 1U) {
      fail("CDR primitive overruns payload");
    }
    return std::to_integer<std::uint8_t>(payload_[offset_++]);
  }

  template <typename Integer>
  Integer read_little_endian(std::size_t alignment) {
    align(alignment);
    if (remaining() < sizeof(Integer)) {
      fail("CDR primitive overruns payload");
    }
    Integer value = 0;
    for (std::size_t index = 0; index < sizeof(Integer); ++index) {
      value |= static_cast<Integer>(std::to_integer<std::uint8_t>(payload_[offset_ + index]))
               << (index * 8U);
    }
    offset_ += sizeof(Integer);
    return value;
  }

  const std::vector<std::byte> &payload_;
  std::size_t offset_{0};
};

InspectionTaskEventFact decode_inspection_event(const RecordedMessage &message) {
  Xcdr1LittleEndianReader reader(message.payload);
  reader.skip_header();
  InspectionTaskEventFact fact;
  fact.observed_time_ns = observed_time(message);
  fact.boot_id = reader.read_string();
  fact.event_sequence = reader.read_uint64();
  fact.kind = reader.read_int32();
  fact.task_id = reader.read_string();
  fact.request_id = reader.read_string();
  (void)reader.read_string();
  fact.state = reader.read_int32();
  fact.map_id = reader.read_string();
  fact.map_version = reader.read_int64();
  fact.route_id = reader.read_string();
  fact.route_revision = reader.read_uint64();
  (void)reader.read_uint32();
  (void)reader.read_uint32();
  (void)reader.read_uint32();
  (void)reader.read_uint32();
  (void)reader.read_string();
  (void)reader.read_string();
  (void)reader.read_string();
  (void)reader.read_string();
  (void)reader.read_string();
  reader.finish();
  return fact;
}

FinalOutputFact decode_final_output(const RecordedMessage &message) {
  Xcdr1LittleEndianReader reader(message.payload);
  (void)reader.read_string();
  FinalOutputFact fact;
  fact.observed_time_ns = observed_time(message);
  fact.producer_boot_id = reader.read_string();
  fact.output_sequence = reader.read_uint64();
  (void)reader.read_uint64();
  (void)reader.read_uint64();
  fact.linear_x = reader.read_double();
  fact.linear_y = reader.read_double();
  fact.linear_z = reader.read_double();
  fact.angular_x = reader.read_double();
  fact.angular_y = reader.read_double();
  fact.angular_z = reader.read_double();
  reader.finish();
  return fact;
}

DriverControlFact decode_driver_control(const RecordedMessage &message) {
  Xcdr1LittleEndianReader reader(message.payload);
  reader.skip_header();
  DriverControlFact fact;
  fact.observed_time_ns = observed_time(message);
  fact.connected = reader.read_bool();
  (void)reader.read_bool();
  (void)reader.read_bool();
  (void)reader.read_bool();
  (void)reader.read_bool();
  (void)reader.read_uint32();
  (void)reader.read_uint64();
  fact.accepted_producer_boot_id = reader.read_string();
  fact.accepted_output_sequence = reader.read_uint64();
  fact.last_command_accepted = reader.read_bool();
  (void)reader.read_string();
  (void)reader.read_string();
  (void)reader.read_string();
  (void)reader.read_string();
  reader.finish();
  return fact;
}

}  // namespace

void InspectionTimelineCapture::observe(const TopicBinding &binding,
                                        const RecordedMessage &message) {
  if (binding.contract == nullptr) {
    return;
  }
  if (binding.contract != &lingtu::message::kNavInspectionTaskEvent &&
      binding.contract != &lingtu::message::kNavCmdVel &&
      binding.contract != &lingtu::message::kDriverControlState) {
    return;
  }

  if (binding.contract == &lingtu::message::kNavInspectionTaskEvent) {
    append_fact(events_, decode_inspection_event(message));
    return;
  }
  if (binding.contract == &lingtu::message::kNavCmdVel) {
    append_fact(outputs_, decode_final_output(message));
    return;
  }
  append_fact(driver_states_, decode_driver_control(message));
}

InspectionTimelineReport InspectionTimelineCapture::verify(const std::string &task_id) const {
  return verify_inspection_task_timeline(task_id, events_, outputs_, driver_states_);
}

}  // namespace lingtu::recording
