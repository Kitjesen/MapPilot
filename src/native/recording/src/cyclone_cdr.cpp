#include "lingtu/recording/cyclone_cdr.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include "dds/ddsi/ddsi_serdata.h"

namespace lingtu::recording {
namespace {

std::uint64_t wall_time_ns() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

struct RawCdrSerdata {
  ddsi_serdata base{};
  std::vector<std::byte> bytes;
};

const ddsi_serdata_ops &raw_serdata_ops();

const ddsi_sertype_ops &raw_sertype_ops() {
  static const ddsi_sertype_ops operations = [] {
    ddsi_sertype_ops value{};
    value.version = ddsi_sertype_v0;
    return value;
  }();
  return operations;
}

ddsi_sertype &raw_sertype() {
  static ddsi_sertype type{};
  static const bool initialized = [] {
    ddsi_sertype_init_flags(&type, "lingtu.recording.RawCdr", &raw_sertype_ops(),
                            &raw_serdata_ops(), DDSI_SERTYPE_FLAG_TOPICKIND_NO_KEY);
    return true;
  }();
  static_cast<void>(initialized);
  return type;
}

const RawCdrSerdata &as_raw(const ddsi_serdata *data) {
  return *reinterpret_cast<const RawCdrSerdata *>(data);
}

RawCdrSerdata &as_raw(ddsi_serdata *data) {
  return *reinterpret_cast<RawCdrSerdata *>(data);
}

std::uint32_t raw_size(const ddsi_serdata *data) {
  return static_cast<std::uint32_t>(as_raw(data).bytes.size());
}

void raw_to_ser(const ddsi_serdata *data, std::size_t offset, std::size_t size, void *buffer) {
  const auto &raw = as_raw(data);
  assert(offset <= raw.bytes.size());
  assert(size <= raw.bytes.size() - offset);
  std::memcpy(buffer, raw.bytes.data() + offset, size);
}

ddsi_serdata *raw_to_ser_ref(const ddsi_serdata *data, std::size_t offset, std::size_t size,
                             ddsrt_iovec_t *reference) {
  const auto &raw = as_raw(data);
  assert(offset <= raw.bytes.size());
  assert(size <= raw.bytes.size() - offset);
  reference->iov_base = const_cast<std::byte *>(raw.bytes.data() + offset);
  reference->iov_len = size;
  return ddsi_serdata_ref(data);
}

void raw_to_ser_unref(ddsi_serdata *data, const ddsrt_iovec_t *reference) {
  static_cast<void>(reference);
  ddsi_serdata_unref(data);
}

void raw_free(ddsi_serdata *data) {
  delete &as_raw(data);
}

const ddsi_serdata_ops &raw_serdata_ops() {
  static const ddsi_serdata_ops operations = [] {
    ddsi_serdata_ops value{};
    value.get_size = raw_size;
    value.to_ser = raw_to_ser;
    value.to_ser_ref = raw_to_ser_ref;
    value.to_ser_unref = raw_to_ser_unref;
    value.free = raw_free;
    return value;
  }();
  return operations;
}

ddsi_serdata *make_raw_serdata(const std::byte *payload, std::size_t payload_size,
                               dds_time_t source_timestamp) {
  auto *raw = new RawCdrSerdata;
  raw->bytes.assign(payload, payload + payload_size);
  ddsi_serdata_init(&raw->base, &raw_sertype(), SDK_DATA);
  raw->base.timestamp.v = source_timestamp;
  raw->base.statusinfo = 0;
  return &raw->base;
}

}  // namespace

std::vector<RecordedMessage> take_cdr_messages(dds_entity_t reader, std::string_view wire_topic,
                                               std::uint32_t &next_sequence,
                                               std::size_t max_samples) {
  if (max_samples == 0 || max_samples > 256) {
    throw std::invalid_argument("CDR take batch must be between 1 and 256");
  }

  std::array<ddsi_serdata *, 256> samples{};
  std::array<dds_sample_info_t, 256> infos{};
  const dds_return_t result = dds_takecdr(
      reader, samples.data(), static_cast<std::uint32_t>(max_samples), infos.data(), DDS_ANY_STATE);
  if (result < 0) {
    throw std::runtime_error("dds_takecdr failed: " + std::string(dds_strretcode(-result)));
  }

  std::vector<RecordedMessage> messages;
  messages.reserve(static_cast<std::size_t>(result));
  for (dds_return_t index = 0; index < result; ++index) {
    auto *sample = samples[static_cast<std::size_t>(index)];
    if (sample == nullptr) {
      continue;
    }
    try {
      if (infos[static_cast<std::size_t>(index)].valid_data) {
        const std::uint32_t size = ddsi_serdata_size(sample);
        RecordedMessage message;
        message.wire_topic.assign(wire_topic.data(), wire_topic.size());
        message.log_time_ns = wall_time_ns();
        const dds_time_t source_timestamp = infos[static_cast<std::size_t>(index)].source_timestamp;
        message.publish_time_ns = source_timestamp > 0
                                      ? static_cast<std::uint64_t>(source_timestamp)
                                      : message.log_time_ns;
        message.sequence = next_sequence++;
        message.payload.resize(size);
        ddsi_serdata_to_ser(sample, 0, size, message.payload.data());
        messages.push_back(std::move(message));
      }
      ddsi_serdata_unref(sample);
      samples[static_cast<std::size_t>(index)] = nullptr;
    } catch (...) {
      ddsi_serdata_unref(sample);
      samples[static_cast<std::size_t>(index)] = nullptr;
      for (dds_return_t tail = index + 1; tail < result; ++tail) {
        if (samples[static_cast<std::size_t>(tail)] != nullptr) {
          ddsi_serdata_unref(samples[static_cast<std::size_t>(tail)]);
        }
      }
      throw;
    }
  }
  return messages;
}

bool bounded_drain_cdr_messages(
    const std::vector<CdrDrainReader> &readers, std::chrono::steady_clock::duration maximum_wait,
    const std::function<bool()> &complete,
    const std::function<void(std::size_t, RecordedMessage &&)> &consume,
    const CdrTakeOne &take_one) {
  if (!complete || !consume) {
    throw std::invalid_argument("bounded CDR drain callbacks are required");
  }
  for (const auto &reader : readers) {
    if (reader.reader <= 0 || reader.wire_topic.empty() || reader.next_sequence == nullptr) {
      throw std::invalid_argument("bounded CDR drain reader is invalid");
    }
  }
  if (complete()) {
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + maximum_wait;
  while (true) {
    bool received = false;
    for (std::size_t index = 0; index < readers.size(); ++index) {
      if (complete()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return complete();
      }
      const auto &reader = readers[index];
      auto messages = take_one ? take_one(reader.reader, reader.wire_topic, *reader.next_sequence)
                               : take_cdr_messages(reader.reader, reader.wire_topic,
                                                   *reader.next_sequence, 1);
      if (messages.size() > 1) {
        throw std::logic_error("bounded CDR take-one callback returned multiple messages");
      }
      received = received || !messages.empty();
      for (auto &message : messages) {
        consume(index, std::move(message));
        if (complete()) {
          return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          return complete();
        }
      }
      if (complete()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return complete();
      }
    }
    if (complete()) {
      return true;
    }
    if (!received) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        break;
      }
      std::this_thread::sleep_for(
          std::min(remaining, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::milliseconds(2))));
      if (complete()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return complete();
      }
    }
  }
  return complete();
}

std::string validate_cdr_payload(const TopicBinding &binding, const std::byte *payload,
                                 std::size_t payload_size) {
  if (binding.descriptor == nullptr) {
    return "CDR topic descriptor is null";
  }
  if (binding.descriptor->m_nkeys != 0) {
    return "phase-0 native replay supports only unkeyed DDS sensor types";
  }
  if (payload == nullptr || payload_size <= 4 ||
      payload_size > std::numeric_limits<std::uint32_t>::max()) {
    return "CDR payload size is invalid";
  }
  if (payload[0] != std::byte{0x00} || payload[1] != std::byte{0x01}) {
    return "phase-0 native replay requires little-endian XCDR1 payloads";
  }
  return {};
}

std::string validate_cdr_payload(const TopicBinding &binding,
                                 const std::vector<std::byte> &payload) {
  return validate_cdr_payload(binding, payload.data(), payload.size());
}

std::string forward_cdr_message(dds_entity_t writer, const TopicBinding &binding,
                                const std::byte *payload, std::size_t payload_size,
                                dds_time_t source_timestamp) {
  if (writer <= 0) {
    return "CDR forward writer is invalid";
  }
  if (source_timestamp < 0) {
    return "CDR source timestamp is invalid";
  }
  const std::string validation_error = validate_cdr_payload(binding, payload, payload_size);
  if (!validation_error.empty()) {
    return validation_error;
  }

  // CycloneDDS consumes the serdata reference once it accepts a valid writer.
  // The raw type intentionally supplies only the serialization operations used
  // by ddsi_serdata_ref_as_type; Cyclone converts it to the writer's typed
  // serdata before publication.
  ddsi_serdata *raw = make_raw_serdata(payload, payload_size, source_timestamp);
  const dds_return_t result = dds_forwardcdr(writer, raw);
  if (result < 0) {
    return "dds_forwardcdr failed: " + std::string(dds_strretcode(-result));
  }
  return {};
}

std::string forward_cdr_message(dds_entity_t writer, const TopicBinding &binding,
                                const std::vector<std::byte> &payload,
                                dds_time_t source_timestamp) {
  return forward_cdr_message(writer, binding, payload.data(), payload.size(), source_timestamp);
}

}  // namespace lingtu::recording
