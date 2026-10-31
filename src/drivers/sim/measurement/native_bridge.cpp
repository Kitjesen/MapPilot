#include "drivers/sim/measurement/native_bridge.h"

#include "measurement_protocol.hpp"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#include "dds/dds.h"
#include "messages.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace a = lingtu::sim::dds_adapter;
thread_local char last_error[1024]{};
void error(const char* message) noexcept {
  std::snprintf(last_error, sizeof(last_error), "%s", message);
}
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return value;
}
std::size_t text_size(const char* value, std::size_t maximum) {
  require(value != nullptr, "measurement string is null");
  std::size_t size = 0;
  while (size <= maximum && value[size] != '\0') ++size;
  require(size > 0 && size <= maximum, "measurement string exceeds bounds or is empty");
  return size;
}

struct Loan {
  dds_entity_t reader;
  void* samples[1]{};
  dds_sample_info_t infos[1]{};
  dds_return_t count{0};
  explicit Loan(dds_entity_t value) : reader(value) {
    count = checked(dds_take(reader, samples, infos, 1, 1), "dds_take(measurement)");
  }
  ~Loan() { if (count > 0) dds_return_loan(reader, samples, count); }
  Loan(const Loan&) = delete;
  Loan& operator=(const Loan&) = delete;
};

template <typename Sequence> void array_size(const Sequence& values, std::uint64_t count) {
  require(values._length == count && values._maximum >= values._length &&
          (count == 0 || values._buffer != nullptr), "measurement DDS array size/shape mismatch");
}
template <typename T, typename Sequence> void copy(std::vector<T>& out, const Sequence& values) {
  if (values._length) out.assign(values._buffer, values._buffer + values._length);
}

// Preflight all variable-sized DDS fields before copying any array/string payload.
template <typename Message> a::MeasurementRecord record(
    const Message& m, a::MeasurementModality modality, std::uint32_t rows,
    std::uint32_t columns, std::uint32_t rate, bool secondary) {
  require(rows > 0 && columns > 0, "measurement shape must be positive");
  const std::uint64_t n = static_cast<std::uint64_t>(rows) * columns;
  const std::uint64_t stride = secondary ? 9 : 5;
  require(n <= a::kMaxMeasurementPayloadBytes / stride, "measurement shape exceeds 32 MiB");
  const char* ids[] = {m.identity.product_session_id, m.identity.session_id, m.identity.instance_id,
                       m.sensor_id, m.header.frame_id};
  std::size_t header = a::kMeasurementHeaderBytes;
  for (const auto* id : ids) header += text_size(id, 255);
  const auto metadata = text_size(m.metadata_json, a::kMaxMeasurementMetadataBytes);
  require(header + metadata + n * stride <= a::kMaxMeasurementPayloadBytes,
          "measurement payload exceeds 32 MiB");
  require(m.header.stamp.sec >= 0 && m.header.stamp.nanosec < 1000000000U &&
          static_cast<std::uint64_t>(m.header.stamp.sec) * 1000000000ULL +
              m.header.stamp.nanosec == m.identity.sim_time_ns,
          "measurement header/source time mismatch");
  array_size(m.valid_mask, n);
  a::MeasurementRecord r;
  r.modality = modality; r.rows = rows; r.columns = columns; r.sample_rate_hz = rate;
  r.product_session_id = ids[0]; r.session_id = ids[1]; r.instance_id = ids[2];
  r.sensor_id = ids[3]; r.frame_id = ids[4];
  r.model_generation = m.identity.model_generation; r.reset_generation = m.identity.reset_generation;
  r.sequence = m.identity.sample_sequence; r.sim_time_ns = m.identity.sim_time_ns;
  r.metadata_json.assign(m.metadata_json, metadata);
  copy(r.valid_mask, m.valid_mask);
  return r;
}
a::MeasurementRecord decode(const lingtu_dds_ThermalImage& m) {
  array_size(m.temperature_k, static_cast<std::uint64_t>(m.height) * m.width);
  auto r = record(m, a::MeasurementModality::Thermal, m.height, m.width, 0, false);
  copy(r.primary, m.temperature_k); return r;
}
a::MeasurementRecord decode(const lingtu_dds_AcousticBlock& m) {
  const std::uint64_t n = static_cast<std::uint64_t>(m.channels) * m.samples;
  array_size(m.pressure_pa, n); array_size(m.voltage_v, m.voltage_v._length ? n : 0);
  require(m.sample_rate_hz > 0, "measurement acoustic sample rate must be positive");
  auto r = record(m, a::MeasurementModality::Acoustic, m.channels, m.samples,
                  m.sample_rate_hz, m.voltage_v._length != 0);
  copy(r.primary, m.pressure_pa); copy(r.voltage_v, m.voltage_v); return r;
}
a::MeasurementRecord decode(const lingtu_dds_SemanticImage& m) {
  const std::uint64_t n = static_cast<std::uint64_t>(m.height) * m.width;
  array_size(m.class_ids, n); array_size(m.instance_ids, n);
  auto r = record(m, a::MeasurementModality::Semantic, m.height, m.width, 0, true);
  copy(r.class_ids, m.class_ids); copy(r.instance_ids, m.instance_ids); return r;
}

class MeasurementBridge final {
 public:
  MeasurementBridge(int domain, const char* product, const char* source)
      : product_(product, text_size(product, 255)), source_(source, text_size(source, 255)),
        validator_(product_, source_) {
    require(domain >= 0, "measurement domain must be nonnegative");
    participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr),
                           "dds_create_participant(measurement)");
    try {
      readers_[0] = reader(&lingtu_dds_ThermalImage_desc, lingtu::message::kSensorThermal);
      readers_[1] = reader(&lingtu_dds_AcousticBlock_desc, lingtu::message::kSensorAcoustic);
      readers_[2] = reader(&lingtu_dds_SemanticImage_desc, lingtu::message::kSensorSemantic);
    } catch (...) { dds_delete(participant_); throw; }
  }
  ~MeasurementBridge() { dds_delete(participant_); }
  MeasurementBridge(const MeasurementBridge&) = delete;
  MeasurementBridge& operator=(const MeasurementBridge&) = delete;

  int take(void* buffer, std::size_t capacity, std::size_t* needed) {
    // At most 16 samples per reader per call, including foreign/disposed data.
    // Round-robin start prevents a busy modality starving the other two.
    for (unsigned i = 0; pending_.empty() && i < 48; ++i) {
      const auto index = next_;
      next_ = (next_ + 1) % readers_.size();
      Loan loan(readers_[index]);
      if (!loan.count || !loan.infos[0].valid_data) continue;
      switch (index) {
        case 0: receive(*static_cast<const lingtu_dds_ThermalImage*>(loan.samples[0])); break;
        case 1: receive(*static_cast<const lingtu_dds_AcousticBlock*>(loan.samples[0])); break;
        case 2: receive(*static_cast<const lingtu_dds_SemanticImage*>(loan.samples[0])); break;
      }
    }
    if (pending_.empty()) return 0;
    *needed = pending_.size();
    if (capacity < pending_.size()) return -2;
    std::memcpy(buffer, pending_.data(), pending_.size());
    pending_.clear();
    return 1;
  }

 private:
  dds_entity_t reader(const dds_topic_descriptor_t* descriptor, const lingtu::message::TopicContract& c) {
    const auto topic = checked(dds_create_topic(participant_, descriptor, c.dds_topic.data(), nullptr, nullptr),
                               "dds_create_topic(measurement)");
    auto qos = lingtu::dds::make_qos(c.qos_profile);
    return checked(dds_create_reader(participant_, topic, qos.get(), nullptr), "dds_create_reader(measurement)");
  }
  template <typename Message> void receive(const Message& message) {
    // A foreign stream never binds a sensor or changes the current stream truth.
    if (!message.identity.product_session_id || !message.identity.session_id ||
        product_ != message.identity.product_session_id || source_ != message.identity.session_id) return;
    auto r = decode(message);
    auto bytes = a::encode_measurement_payload(r);
    std::string reason;
    if (!validator_.accept(r, reason)) throw std::runtime_error(reason);
    pending_ = std::move(bytes);
  }
  std::string product_, source_;
  a::MeasurementStreamValidator validator_;
  dds_entity_t participant_{0};
  std::array<dds_entity_t, 3> readers_{};
  std::size_t next_{0};
  std::vector<std::uint8_t> pending_;
};
}  // namespace

extern "C" void* lt_measurement_create(int domain, const char* product, const char* source) {
  error("");
  try { return new MeasurementBridge(domain, product, source); }
  catch (const std::exception& e) { error(e.what()); }
  catch (...) { error("unknown measurement create error"); }
  return nullptr;
}
extern "C" int lt_measurement_take(void* handle, void* buffer, size_t capacity, size_t* needed) {
  error("");
  if (needed) *needed = 0;
  try {
    require(handle && needed && (buffer || capacity == 0), "invalid measurement take arguments");
    return static_cast<MeasurementBridge*>(handle)->take(buffer, capacity, needed);
  } catch (const std::exception& e) { error(e.what()); }
  catch (...) { error("unknown measurement take error"); }
  return -1;
}
extern "C" void lt_measurement_destroy(void* handle) {
  delete static_cast<MeasurementBridge*>(handle);
}
extern "C" const char* lt_measurement_last_error(void) { return last_error; }
