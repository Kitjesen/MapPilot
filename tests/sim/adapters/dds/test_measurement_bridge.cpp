#include "drivers/sim/measurement/native_bridge.h"
#include "measurement_protocol.hpp"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#include "dds/dds.h"
#include "messages.h"

#include <chrono>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace a = lingtu::sim::dds_adapter;
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
dds_entity_t checked(dds_return_t value) {
  require(value >= 0, std::string("DDS: ") + dds_strretcode(-value));
  return value;
}
template <typename F> void until(F fn, const char* message) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    if (fn()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error(message);
}
using Bridge = std::unique_ptr<void, decltype(&lt_measurement_destroy)>;
constexpr int domain = 131;
const std::string metadata = u8"{\"units\":\"K\",\"note\":\"\u6e29\u5ea6\",\"model\":{\"gain\":1.25}}";

struct Peer {
  dds_entity_t participant{};
  std::array<dds_entity_t, 3> writers{};
  Peer() {
    participant = checked(dds_create_participant(domain, nullptr, nullptr));
    try {
      const lingtu::message::TopicContract contracts[] = {lingtu::message::kSensorThermal,
          lingtu::message::kSensorAcoustic, lingtu::message::kSensorSemantic};
      const dds_topic_descriptor_t* descriptors[] = {&lingtu_dds_ThermalImage_desc,
          &lingtu_dds_AcousticBlock_desc, &lingtu_dds_SemanticImage_desc};
      for (std::size_t i = 0; i < writers.size(); ++i) {
        auto qos = lingtu::dds::make_qos(contracts[i].qos_profile);
        const auto topic = checked(dds_create_topic(participant, descriptors[i],
                                    contracts[i].dds_topic.data(), nullptr, nullptr));
        writers[i] = checked(dds_create_writer(participant, topic, qos.get(), nullptr));
      }
    } catch (...) { dds_delete(participant); throw; }
  }
  ~Peer() { dds_delete(participant); }
  Peer(const Peer&) = delete;
  Peer& operator=(const Peer&) = delete;
  void matched(bool expected = true) {
    until([&] {
      for (const auto writer : writers) {
        dds_publication_matched_status_t status{};
        checked(dds_get_publication_matched_status(writer, &status));
        if ((status.current_count > 0) != expected) return false;
      }
      return true;
    }, "reader discovery/close timed out");
  }
  template <typename Sequence, typename T> static void array(Sequence& out, const std::vector<T>& values) {
    out._maximum = out._length = static_cast<std::uint32_t>(values.size());
    out._buffer = const_cast<T*>(values.data()); out._release = false;
  }
  template <typename Message> static void identity(Message& message, const a::MeasurementRecord& r,
                                                  int header_delta) {
    message.sensor_id = const_cast<char*>(r.sensor_id.c_str());
    message.header.frame_id = const_cast<char*>(r.frame_id.c_str());
    message.header.stamp.sec = static_cast<std::int32_t>(r.sim_time_ns / 1000000000ULL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(r.sim_time_ns % 1000000000ULL) + header_delta;
    message.identity.product_session_id = const_cast<char*>(r.product_session_id.c_str());
    message.identity.session_id = const_cast<char*>(r.session_id.c_str());
    message.identity.instance_id = const_cast<char*>(r.instance_id.c_str());
    message.identity.model_generation = r.model_generation; message.identity.reset_generation = r.reset_generation;
    message.identity.sample_sequence = r.sequence; message.identity.sim_time_ns = r.sim_time_ns;
    array(message.valid_mask, r.valid_mask);
    message.metadata_json = const_cast<char*>(r.metadata_json.c_str());
  }
  void write(const a::MeasurementRecord& r, int header_delta = 0) {
    switch (r.modality) {
      case a::MeasurementModality::Thermal: {
        lingtu_dds_ThermalImage m{}; identity(m, r, header_delta);
        m.height = r.rows; m.width = r.columns; array(m.temperature_k, r.primary);
        checked(dds_write(writers[0], &m)); break;
      }
      case a::MeasurementModality::Acoustic: {
        lingtu_dds_AcousticBlock m{}; identity(m, r, header_delta);
        m.channels = r.rows; m.samples = r.columns; m.sample_rate_hz = r.sample_rate_hz;
        array(m.pressure_pa, r.primary); array(m.voltage_v, r.voltage_v);
        checked(dds_write(writers[1], &m)); break;
      }
      case a::MeasurementModality::Semantic: {
        lingtu_dds_SemanticImage m{}; identity(m, r, header_delta);
        m.height = r.rows; m.width = r.columns;
        array(m.class_ids, r.class_ids); array(m.instance_ids, r.instance_ids);
        checked(dds_write(writers[2], &m)); break;
      }
    }
  }
};

a::MeasurementRecord sample(a::MeasurementModality modality = a::MeasurementModality::Thermal) {
  a::MeasurementRecord r;
  r.modality = modality;
  r.product_session_id = "product"; r.session_id = "source"; r.instance_id = u8"robot/\u96f7\u9706";
  r.sensor_id = modality == a::MeasurementModality::Thermal ? "thermal" :
                modality == a::MeasurementModality::Acoustic ? "acoustic" : "semantic";
  r.frame_id = "robot/" + r.sensor_id;
  r.model_generation = 7; r.reset_generation = 3; r.sequence = 0; r.sim_time_ns = 1234567890;
  r.rows = 1; r.columns = 2; r.metadata_json = metadata; r.valid_mask = {1, 0};
  if (modality == a::MeasurementModality::Semantic) {
    r.class_ids = {0x7fffffff, 0}; r.instance_ids = {99, 0};
  } else {
    r.primary = {321.5F, std::numeric_limits<float>::quiet_NaN()};
    if (modality == a::MeasurementModality::Acoustic) {
      r.sample_rate_hz = 48000; r.voltage_v = {0.5F, std::numeric_limits<float>::quiet_NaN()};
    }
  }
  return r;
}
int take(void* bridge, std::vector<std::uint8_t>& bytes, std::size_t& needed) {
  return lt_measurement_take(bridge, bytes.data(), bytes.size(), &needed);
}
std::vector<std::uint8_t> delivered(void* bridge) {
  std::vector<std::uint8_t> bytes(4096);
  std::size_t needed = 0;
  until([&] {
    const auto status = take(bridge, bytes, needed);
    require(status >= 0, lt_measurement_last_error());
    return status == 1;
  }, "DDS measurement delivery timed out");
  bytes.resize(needed); return bytes;
}
void expect_error(void* bridge, const char* reason) {
  std::vector<std::uint8_t> bytes(4096);
  std::size_t needed = 123;
  until([&] {
    const auto status = take(bridge, bytes, needed);
    require(status != 1 && status != -2, "invalid data was delivered");
    return status == -1;
  }, "expected malformed/stale measurement error");
  require(needed == 0 && std::string(lt_measurement_last_error()).find(reason) != std::string::npos,
          std::string("wrong error: ") + lt_measurement_last_error());
}
void expect_none(void* bridge) {
  // Poll for a bounded observation period: a single empty poll would not prove exclusion.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(60);
  std::vector<std::uint8_t> bytes(4096);
  do {
    std::size_t needed = 123;
    require(take(bridge, bytes, needed) == 0 && needed == 0 && !*lt_measurement_last_error(),
            "excluded/empty stream produced output or error");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
}

void thermal_roundtrip() {
  Peer peer;
  Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
  require(bridge != nullptr, lt_measurement_last_error());
  peer.matched();
  peer.write(sample());
  std::vector<std::uint8_t> bytes(4096);
  std::size_t needed = 0;
  until([&] {
    const int status = lt_measurement_take(bridge.get(), bytes.data(), bytes.size(), &needed);
    require(status >= 0, lt_measurement_last_error());
    return status == 1;
  }, "thermal delivery timed out");
  bytes.resize(needed);
  const auto r = a::decode_measurement_payload(bytes);
  require(r.metadata_json == metadata && r.sensor_id == "thermal" && r.sequence == 0 &&
          r.sim_time_ns == 1234567890 && r.primary[0] == 321.5F && std::isnan(r.primary[1]),
          "thermal source data changed");
  bridge.reset(); peer.matched(false);
}
void all_types_and_retry() {
  Peer peer;
  Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
  require(bridge != nullptr, lt_measurement_last_error()); peer.matched();
  for (const auto modality : {a::MeasurementModality::Thermal, a::MeasurementModality::Acoustic,
                              a::MeasurementModality::Semantic}) {
    auto r = sample(modality);
    peer.write(r);
    // Size query consumes DDS but retains the fully owned packet after returning its loan.
    std::size_t needed = 0;
    until([&] {
      const auto status = lt_measurement_take(bridge.get(), nullptr, 0, &needed);
      require(status == 0 || status == -2, lt_measurement_last_error()); return status == -2;
    }, "size query timed out");
    const auto size = needed;
    auto next = r; next.sequence = 1; next.sim_time_ns += 100000000;
    peer.write(next);
    std::vector<std::uint8_t> small(size - 1, 0xa5);
    require(take(bridge.get(), small, needed) == -2 && needed == size &&
            small == std::vector<std::uint8_t>(size - 1, 0xa5), "small buffer changed/lost pending packet");
    std::vector<std::uint8_t> exact(size);
    require(take(bridge.get(), exact, needed) == 1 && needed == size, "exact-size retry failed");
    require(exact == a::encode_measurement_payload(r), "DDS -> LTM1 bytes differ from source contract");
    const auto decoded = a::decode_measurement_payload(exact);
    require(a::encode_measurement_payload(decoded) == exact, "LTM1 decoder parity failed");
    require(delivered(bridge.get()) == a::encode_measurement_payload(next), "retry consumed next packet");
  }
  auto acoustic = sample(a::MeasurementModality::Acoustic);
  acoustic.sequence = 2; acoustic.sim_time_ns += 200000000; acoustic.voltage_v.clear();
  acoustic.rows = 2; acoustic.columns = 2;
  acoustic.primary = {1.F, 2.F, 3.F, 4.F}; acoustic.valid_mask = {1, 1, 1, 1};
  peer.write(acoustic);
  require(delivered(bridge.get()) == a::encode_measurement_payload(acoustic), "channel-major/no-volts block changed");
  expect_none(bridge.get());
  bridge.reset(); peer.matched(false);
}
void foreign_and_current_truth() {
  Peer peer;
  Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
  require(bridge != nullptr, lt_measurement_last_error()); peer.matched();
  auto r = sample();
  auto foreign = r; foreign.product_session_id = "foreign"; foreign.sequence = 100;
  foreign.instance_id = "other-robot"; foreign.primary.clear(); // Skip even malformed foreign arrays.
  peer.write(foreign); expect_none(bridge.get());
  foreign = r; foreign.session_id = "foreign"; foreign.sequence = 100;
  peer.write(foreign); expect_none(bridge.get());
  peer.write(r); require(delivered(bridge.get()) == a::encode_measurement_payload(r), "foreign sample auto-bound sensor");

  peer.write(r); expect_error(bridge.get(), "increase");
  auto next = r; next.sequence = 1; next.sim_time_ns += 100000000;
  auto invalid = next; invalid.reset_generation = 2;
  peer.write(invalid); expect_error(bridge.get(), "generation");
  invalid = next; invalid.frame_id = "wrong-frame";
  peer.write(invalid); expect_error(bridge.get(), "instance/frame/modality");
  peer.write(next); require(delivered(bridge.get()) == a::encode_measurement_payload(next), "reject changed current truth");
  r = next; r.reset_generation = 4; r.sequence = 0; r.sim_time_ns = 0;
  peer.write(r); require(delivered(bridge.get()) == a::encode_measurement_payload(r), "reset zero time was restamped");
  invalid = next; invalid.sequence = 200;
  peer.write(invalid); expect_error(bridge.get(), "generation");
  r.model_generation = 8; r.reset_generation = 0; // Existing validator treats model as outer generation.
  peer.write(r); require(delivered(bridge.get()) == a::encode_measurement_payload(r), "model generation restart rejected");
  auto other = sample(); other.sensor_id = "independent"; other.frame_id = "robot/independent";
  peer.write(other); require(delivered(bridge.get()) == a::encode_measurement_payload(other), "compared counters across sensors");
}
void malformed_then_recover() {
  Peer peer;
  Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
  require(bridge != nullptr, lt_measurement_last_error()); peer.matched();
  const auto r = sample();
  auto bad = r; bad.primary.pop_back(); peer.write(bad); expect_error(bridge.get(), "size/shape");
  bad = r; bad.valid_mask.pop_back(); peer.write(bad); expect_error(bridge.get(), "size/shape");
  peer.write(r, 1); expect_error(bridge.get(), "header/source time");
  bad = r; bad.valid_mask[0] = 2; peer.write(bad); expect_error(bridge.get(), "mask");
  bad = r; bad.primary[0] = -1; peer.write(bad); expect_error(bridge.get(), "Kelvin");
  bad = r; bad.primary[1] = 1; peer.write(bad); expect_error(bridge.get(), "NaN");
  bad = r; bad.metadata_json = "\xff"; peer.write(bad); expect_error(bridge.get(), "UTF-8");
  bad = r; bad.metadata_json.assign(a::kMaxMeasurementMetadataBytes + 1, 'x');
  peer.write(bad); expect_error(bridge.get(), "bounds");
  bad = r; bad.sequence = 0x100000000ULL; peer.write(bad); expect_error(bridge.get(), "uint32");
  peer.write(r); require(delivered(bridge.get()) == a::encode_measurement_payload(r), "malformed sample poisoned sensor state");
  auto semantic = sample(a::MeasurementModality::Semantic);
  bad = semantic; bad.instance_ids.clear(); peer.write(bad); expect_error(bridge.get(), "size/shape");
  bad = semantic; bad.class_ids[0] = 0x80000000; peer.write(bad); expect_error(bridge.get(), "labels");
  peer.write(semantic); require(delivered(bridge.get()) == a::encode_measurement_payload(semantic), "semantic recovery failed");
  auto acoustic = sample(a::MeasurementModality::Acoustic);
  bad = acoustic; bad.voltage_v.pop_back(); peer.write(bad); expect_error(bridge.get(), "size/shape");
  bad = acoustic; bad.sample_rate_hz = 0; peer.write(bad); expect_error(bridge.get(), "sample rate");
  bad = acoustic; bad.voltage_v[1] = 1; peer.write(bad); expect_error(bridge.get(), "voltage");
  peer.write(acoustic); require(delivered(bridge.get()) == a::encode_measurement_payload(acoustic), "acoustic recovery failed");
  bad = acoustic; bad.sequence = 1; ++bad.sim_time_ns;
  peer.write(bad); expect_error(bridge.get(), "overlap");
  acoustic.sequence = 1; acoustic.sim_time_ns += 41667;
  peer.write(acoustic); require(delivered(bridge.get()) == a::encode_measurement_payload(acoustic), "acoustic overlap changed truth");
}
void abi_errors_and_close() {
  require(!lt_measurement_create(domain, nullptr, "source") && *lt_measurement_last_error(), "null product accepted");
  require(!lt_measurement_create(domain, "product", "") && *lt_measurement_last_error(), "empty source auto-bound");
  require(!lt_measurement_create(-1, "product", "source") && *lt_measurement_last_error(), "negative domain accepted");
  std::size_t needed = 99;
  require(lt_measurement_take(nullptr, nullptr, 0, &needed) == -1 && needed == 0, "null handle accepted");
  lt_measurement_destroy(nullptr);
  Peer peer;
  for (int i = 0; i < 3; ++i) {
    Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
    require(bridge != nullptr, lt_measurement_last_error()); peer.matched();
    require(lt_measurement_take(bridge.get(), nullptr, 1, &needed) == -1, "null output accepted");
    require(lt_measurement_take(bridge.get(), nullptr, 0, nullptr) == -1, "null needed accepted");
    peer.write(sample());
    until([&] { return lt_measurement_take(bridge.get(), nullptr, 0, &needed) == -2; }, "pending close sample timed out");
    bridge.reset(); peer.matched(false); // Close releases participant and owned pending bytes.
  }
}
void packet_cap_and_bounded_batch() {
  Peer peer;
  Bridge bridge(lt_measurement_create(domain, "product", "source"), lt_measurement_destroy);
  require(bridge != nullptr, lt_measurement_last_error()); peer.matched();
  {
    auto large = sample();
    // Arrays alone fit; fixed header + IDs + metadata push the complete packet over 32 MiB.
    large.columns = a::kMaxMeasurementPayloadBytes / 5;
    large.primary.assign(large.columns, 300.F); large.valid_mask.assign(large.columns, 1);
    peer.write(large); expect_error(bridge.get(), "32 MiB");
    bool rejected = false;
    try { (void)a::encode_measurement_payload(large); }
    catch (const std::exception& e) { rejected = std::string(e.what()).find("32 MiB") != std::string::npos; }
    require(rejected, "encoder allocated an oversized complete packet");
  }
  auto valid = sample();
  peer.write(valid);
  require(delivered(bridge.get()) == a::encode_measurement_payload(valid), "oversize input changed sensor truth");

  auto foreign = sample(a::MeasurementModality::Acoustic);
  foreign.product_session_id = "foreign";
  // SensorStream keeps 256 samples. A call may consume only 16 from this modality.
  for (std::uint64_t i = 0; i < 64; ++i) {
    foreign.sequence = i; foreign.sim_time_ns += 100000;
    peer.write(foreign);
  }
  std::vector<std::uint8_t> bytes(4096);
  std::size_t needed = 123;
  require(take(bridge.get(), bytes, needed) == 0 && needed == 0, "foreign batch produced output");
  auto acoustic = sample(a::MeasurementModality::Acoustic);
  peer.write(acoustic);
  // If the first take drained an unbounded batch, this one would already deliver the valid tail.
  require(take(bridge.get(), bytes, needed) == 0 && needed == 0, "take drained more than its bounded batch");
  require(delivered(bridge.get()) == a::encode_measurement_payload(acoustic), "valid tail lost behind foreign batch");
}
}  // namespace
int main() {
  try {
    thermal_roundtrip();
    std::puts("PASS thermal DDS -> C ABI -> LTM1 and close");
    all_types_and_retry(); std::puts("PASS all three typed arrays, UTF-8 metadata, small-buffer retry");
    foreign_and_current_truth(); std::puts("PASS foreign exclusion, per-sensor stale/generation truth, zero source time");
    malformed_then_recover(); std::puts("PASS malformed arrays/metadata/time/values and recovery, acoustic duration");
    abi_errors_and_close(); std::puts("PASS ABI errors, pending close and repeated create/destroy");
    packet_cap_and_bounded_batch(); std::puts("PASS complete 32 MiB cap before copy and finite foreign batch");
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1;
  }
}
