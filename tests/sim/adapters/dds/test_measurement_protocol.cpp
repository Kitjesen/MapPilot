#include "sensor_record_protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef LINGTU_MEASUREMENT_DDS_TEST
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <thread>
#include "dds/dds.h"
#include "messages.h"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#ifndef _WIN32
#include <sys/wait.h>
#endif
#endif

namespace {
namespace a = lingtu::sim::dds_adapter;
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}
void integer(std::string &out, std::uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; ++i) out.push_back(static_cast<char>(value >> (8 * i)));
}
void overwrite(std::string &out, std::size_t offset, std::uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; ++i) out.at(offset + i) = static_cast<char>(value >> (8 * i));
}
void floating(std::string &out, float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  integer(out, bits, 4);
}
const std::string metadata = u8"{\"units\":\"K\",\"note\":\"\u6e29\u5ea6\"}";
std::string inner(std::uint16_t modality = 1, bool voltage = false) {
  const std::string ids[] = {"product", "source", "robot", "sensor", "robot/sensor"};
  std::size_t header = 76;
  for (const auto &id : ids) header += id.size();
  std::string out = "LTM1";
  integer(out, 1, 2); integer(out, modality, 2);
  integer(out, header, 4); integer(out, metadata.size(), 4);
  integer(out, 1, 4); integer(out, 2, 4);
  integer(out, modality == 2 ? 48000 : 0, 4); integer(out, voltage ? 1 : 0, 4);
  integer(out, 7, 8); integer(out, 3, 8); integer(out, 0, 8); integer(out, 1234567890, 8);
  for (const auto &id : ids) integer(out, id.size(), 2);
  integer(out, 0, 2);
  for (const auto &id : ids) out += id;
  out += metadata;
  if (modality == 3) { integer(out, 42, 4); integer(out, 0, 4); }
  else { floating(out, 321.5F); floating(out, std::numeric_limits<float>::quiet_NaN()); }
  out.push_back(1); out.push_back(0);
  if (modality == 3) { integer(out, 99, 4); integer(out, 0, 4); }
  else if (voltage) { floating(out, 0.5F); floating(out, std::numeric_limits<float>::quiet_NaN()); }
  return out;
}
std::string envelope(const std::string &payload) {
  std::string out("LTU1\x09\0\0\0", 8);
  integer(out, 1234567890, 8); integer(out, 0, 4);
  integer(out, 2, 4); integer(out, payload.size(), 4);
  return out + payload;
}
a::SensorRecord read(const std::string &wire) {
  a::SensorRecord record;
  std::string error;
  std::istringstream input(wire);
  const auto status = a::read_sensor_record(input, record, error);
  require(status == a::SensorRecordReadStatus::Ok, "valid LTM1 must parse: " + error);
  return record;
}
void thermal_envelope_preserves_bytes() {
  const auto payload = inner();
  const auto record = read(envelope(payload));
  require(std::string(record.payload.begin(), record.payload.end()) == payload,
          "LTM1 bytes changed inside LTU1");
}
void typed_values_and_rejections() {
  for (const auto modality : {1, 2, 3}) {
    const auto r = a::decode_measurement_payload(read(envelope(inner(static_cast<std::uint16_t>(modality), modality == 2))));
    require(r.product_session_id == "product" && r.session_id == "source" &&
            r.instance_id == "robot" && r.sensor_id == "sensor" && r.frame_id == "robot/sensor" &&
            r.model_generation == 7 && r.reset_generation == 3 && r.sequence == 0 &&
            r.sim_time_ns == 1234567890 && r.rows == 1 && r.columns == 2 &&
            r.metadata_json == metadata && r.valid_mask == std::vector<std::uint8_t>({1, 0}),
            "typed identity, shape, time, UTF-8 metadata changed");
    if (modality == 3)
      require(r.class_ids == std::vector<std::uint32_t>({42, 0}) &&
              r.instance_ids == std::vector<std::uint32_t>({99, 0}), "uint32 labels changed");
    else {
      require(r.primary.size() == 2 && r.primary[0] == 321.5F && std::isnan(r.primary[1]),
              "float/NaN primary changed");
      if (modality == 2)
        require(r.sample_rate_hz == 48000 && r.voltage_v[0] == 0.5F && std::isnan(r.voltage_v[1]),
                "acoustic rate or volts changed");
    }
  }
  auto reject = [](const std::string &wire) {
    a::SensorRecord record; std::string error; std::istringstream input(wire);
    require(a::read_sensor_record(input, record, error) == a::SensorRecordReadStatus::Error && !error.empty(),
            "malformed measurement accepted");
  };
  for (const auto field : {8U, 16U, 20U}) {
    auto wire = envelope(inner()); overwrite(wire, field, 99, field == 8 ? 8 : 4); reject(wire);
  }
  for (const auto &change : std::vector<std::pair<std::size_t, std::uint64_t>>{
           {4, 2}, {6, 0}, {8, 76}, {12, 0}, {12, 1048577}, {16, 0}, {20, 0},
           {16, 0xffffffff}, {24, 1}, {28, 1}, {48, 0x100000000ULL},
           {56, 2147483648000000000ULL}, {64, 0}, {64, 256}, {74, 1}}) {
    auto payload = inner();
    const unsigned size = change.first >= 48 && change.first <= 56 ? 8 :
                          change.first < 8 || change.first >= 64 ? 2 : 4;
    overwrite(payload, change.first, change.second, size); reject(envelope(payload));
  }
  auto payload = inner(); payload[0] = 'X'; reject(envelope(payload));
  payload = inner(); payload[76] = ' '; reject(envelope(payload));
  payload = inner(); payload[76] = '\0'; reject(envelope(payload));
  payload = inner(); payload[76] = static_cast<char>(0xff); reject(envelope(payload));
  // Metadata follows 76 prefix bytes plus 36 identifier bytes in this fixture.
  payload = inner(); payload[112] = static_cast<char>(0xff); reject(envelope(payload));
  payload = inner(); payload[112] = '\0'; reject(envelope(payload));
  payload = inner(); payload.pop_back(); reject(envelope(payload));
  reject(envelope(inner() + "x"));
  payload = inner(); overwrite(payload, payload.size() - 2, 2, 1); reject(envelope(payload));
  payload = inner(); overwrite(payload, payload.size() - 6, 0, 4); reject(envelope(payload));
  payload = inner(); overwrite(payload, payload.size() - 10, 0x7f800000, 4); reject(envelope(payload));
  payload = inner(); overwrite(payload, payload.size() - 10, 0, 4); reject(envelope(payload));
  payload = inner(2, true); overwrite(payload, payload.size() - 4, 0, 4); reject(envelope(payload));
  payload = inner(2); overwrite(payload, 24, 0, 4); reject(envelope(payload));
  payload = inner(2); overwrite(payload, 28, 2, 4); reject(envelope(payload));
  for (const auto value : {0U, 0x80000000U, 0xffffffffU}) {
    payload = inner(3); overwrite(payload, payload.size() - 18, value, 4); reject(envelope(payload));
    payload = inner(3); overwrite(payload, payload.size() - 8, value, 4); reject(envelope(payload));
  }
  payload = inner(3); overwrite(payload, payload.size() - 4, 1, 4); reject(envelope(payload));
  payload = inner(3); overwrite(payload, payload.size() - 18, 0x7fffffff, 4);
  overwrite(payload, payload.size() - 8, 0x7fffffff, 4);
  const auto boundary = a::decode_measurement_payload(read(envelope(payload)));
  require(boundary.class_ids[0] == 0x7fffffff && boundary.instance_ids[0] == 0x7fffffff,
          "maximum Python-compatible uint32 label was not preserved");
  auto wire = envelope(inner()); overwrite(wire, 24, a::kMaxMeasurementPayloadBytes + 1ULL, 4); reject(wire);
  // Bytes beyond the declared type9 cap must never be allocated/read.
  require(a::decode_measurement_payload(read(envelope(inner(2)))).voltage_v.empty(), "optional volts not empty");
}
void independent_streams_and_generations() {
  auto r = a::decode_measurement_payload(read(envelope(inner())));
  a::MeasurementStreamValidator validator("product", "source");
  std::string error;
  require(validator.accept(r, error), "first sensor rejected: " + error);
  auto other = r; other.sensor_id = "second"; other.sequence = 57; other.sim_time_ns = 2;
  require(validator.accept(other, error), "independent sensor sequence compared globally");
  require(!validator.accept(r, error), "duplicate measurement accepted");
  r.sequence = 2; r.sim_time_ns += 10;
  require(validator.accept(r, error), "sequence gaps must be allowed for best effort");
  auto bad = r; bad.sequence = 3; bad.sim_time_ns = 1;
  require(!validator.accept(bad, error), "backward time accepted");
  for (int field = 0; field < 6; ++field) {
    bad = r; ++bad.sequence; ++bad.sim_time_ns;
    if (field == 0) bad.product_session_id = "wrong";
    if (field == 1) bad.session_id = "wrong";
    if (field == 2) bad.instance_id = "wrong";
    if (field == 3) bad.frame_id = "wrong";
    if (field == 4) --bad.model_generation;
    if (field == 5) --bad.reset_generation;
    require(!validator.accept(bad, error), "wrong identity/generation accepted");
  }
  ++r.reset_generation; r.sequence = 0; r.sim_time_ns = 0;
  require(validator.accept(r, error), "reset did not permit source clock restart");
  --r.reset_generation; r.sequence = 100; r.sim_time_ns = 100;
  require(!validator.accept(r, error), "old reset accepted");
  ++r.model_generation; r.reset_generation = 0; r.sequence = 0; r.sim_time_ns = 0;
  require(validator.accept(r, error), "new model did not permit reset restart");
  require(!a::MeasurementStreamValidator("", "").accept(r, error), "unbound identity accepted");
  auto acoustic = a::decode_measurement_payload(read(envelope(inner(2))));
  acoustic.sensor_id = "microphone";
  require(validator.accept(acoustic, error), "first acoustic block rejected");
  ++acoustic.sequence; acoustic.sim_time_ns += 1;
  require(!validator.accept(acoustic, error), "overlapping acoustic blocks accepted");
  acoustic.sim_time_ns = 1234567890 + 41667;
  require(validator.accept(acoustic, error), "nonoverlapping acoustic block rejected");
}
#ifdef LINGTU_MEASUREMENT_DDS_TEST
using Clock = std::chrono::steady_clock;
class Process final {
 public:
  Process(const std::filesystem::path &executable, const std::string &options) {
    const auto command = "\"" + executable.string() + "\" --stdin-records " + options;
#ifdef _WIN32
    pipe_ = _popen(command.c_str(), "wb");
#else
    pipe_ = popen(command.c_str(), "w");
#endif
    require(pipe_ != nullptr, "cannot start measurement publisher");
  }
  Process(const Process &) = delete;
  Process &operator=(const Process &) = delete;
  ~Process() { if (pipe_) close(); }
  void write(const std::string &wire) {
    require(std::fwrite(wire.data(), 1, wire.size(), pipe_) == wire.size() && std::fflush(pipe_) == 0,
            "publisher stdin write failed");
  }
  int close() {
#ifdef _WIN32
    const auto result = _pclose(pipe_);
#else
    const auto status = pclose(pipe_);
    const auto result = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
    pipe_ = nullptr; return result;
  }
 private:
  std::FILE *pipe_{nullptr};
};
void publisher_validation(const std::filesystem::path &executable) {
  const std::string binding = " --measurement-product-session-id product --measurement-session-id source";
  for (const auto &stream : {" --stream camera", ""}) {
    Process process(executable, "--validate-records" + binding + stream);
    auto thermal = inner(); auto acoustic = inner(2, true); auto semantic = inner(3);
    // Distinct sensors are independent even at the same timestamp and sequence.
    acoustic[76 + 7 + 6 + 5] = 'a'; semantic[76 + 7 + 6 + 5] = 'm';
    process.write(envelope(thermal) + envelope(acoustic) + envelope(semantic));
    require(process.close() == 0, "camera/all publisher must accept typed measurements");
  }
  for (const auto &options : std::vector<std::string>{"--stream imu" + binding, "--stream lidar" + binding,
           "--stream camera --measurement-product-session-id wrong --measurement-session-id source",
           "--stream camera --measurement-product-session-id product --measurement-session-id wrong",
           "--stream camera"}) {
    Process process(executable, "--validate-records " + options);
    process.write(envelope(inner()));
    require(process.close() != 0, "wrong stream/unbound measurement publisher accepted input");
  }
  {
    Process process(executable, "--validate-records --stream camera" + binding);
    process.write(envelope(inner()) + envelope(inner()));
    require(process.close() != 0, "publisher accepted duplicate source measurements");
  }
}
dds_entity_t checked(dds_return_t result, const char *operation) {
  require(result >= 0, std::string(operation) + ": " + dds_strretcode(-result));
  return static_cast<dds_entity_t>(result);
}
template <typename Predicate> void until(Predicate predicate, const char *message, int seconds = 8) {
  const auto deadline = Clock::now() + std::chrono::seconds(seconds);
  while (!predicate()) {
    require(Clock::now() < deadline, message);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
class Readers final {
 public:
  explicit Readers(int domain) {
    participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr), "participant");
    try {
      const dds_topic_descriptor_t *types[] = {&lingtu_dds_ThermalImage_desc,
          &lingtu_dds_AcousticBlock_desc, &lingtu_dds_SemanticImage_desc};
      const lingtu::message::TopicContract contracts[] = {lingtu::message::kSensorThermal,
          lingtu::message::kSensorAcoustic, lingtu::message::kSensorSemantic};
      for (std::size_t i = 0; i < 3; ++i) {
        const auto topic = checked(dds_create_topic(participant_, types[i], contracts[i].dds_topic.data(), nullptr, nullptr), "topic");
        auto qos = lingtu::dds::make_qos(contracts[i].qos_profile);
        readers_[i] = checked(dds_create_reader(participant_, topic, qos.get(), nullptr), "reader");
      }
    } catch (...) { dds_delete(participant_); participant_ = 0; throw; }
  }
  ~Readers() { if (participant_ > 0) dds_delete(participant_); }
  Readers(const Readers &) = delete;
  Readers &operator=(const Readers &) = delete;
  bool matched() const {
    for (const auto reader : readers_)
      if (checked(dds_get_matched_publications(reader, nullptr, 0), "matched publications") == 0) return false;
    return true;
  }
  void poll() {
    take<lingtu_dds_ThermalImage>(readers_[0]);
    take<lingtu_dds_AcousticBlock>(readers_[1]);
    take<lingtu_dds_SemanticImage>(readers_[2]);
  }
  std::vector<a::MeasurementRecord> records;
 private:
  template <typename T> static a::MeasurementRecord common(const T &m) {
    a::MeasurementRecord r;
    r.product_session_id = m.identity.product_session_id;
    r.session_id = m.identity.session_id; r.instance_id = m.identity.instance_id;
    r.sensor_id = m.sensor_id; r.frame_id = m.header.frame_id;
    r.model_generation = m.identity.model_generation; r.reset_generation = m.identity.reset_generation;
    r.sequence = m.identity.sample_sequence; r.sim_time_ns = m.identity.sim_time_ns;
    require(m.header.stamp.sec >= 0 &&
        static_cast<std::uint64_t>(m.header.stamp.sec) * 1000000000ULL + m.header.stamp.nanosec == r.sim_time_ns,
        "DDS header stamp does not preserve exact source time");
    r.metadata_json = m.metadata_json;
    r.valid_mask.assign(m.valid_mask._buffer, m.valid_mask._buffer + m.valid_mask._length);
    return r;
  }
  static a::MeasurementRecord convert(const lingtu_dds_ThermalImage &m) {
    auto r = common(m); r.modality = a::MeasurementModality::Thermal;
    r.rows = m.height; r.columns = m.width;
    r.primary.assign(m.temperature_k._buffer, m.temperature_k._buffer + m.temperature_k._length);
    return r;
  }
  static a::MeasurementRecord convert(const lingtu_dds_AcousticBlock &m) {
    auto r = common(m); r.modality = a::MeasurementModality::Acoustic;
    r.rows = m.channels; r.columns = m.samples; r.sample_rate_hz = m.sample_rate_hz;
    r.primary.assign(m.pressure_pa._buffer, m.pressure_pa._buffer + m.pressure_pa._length);
    if (m.voltage_v._length) r.voltage_v.assign(m.voltage_v._buffer, m.voltage_v._buffer + m.voltage_v._length);
    return r;
  }
  static a::MeasurementRecord convert(const lingtu_dds_SemanticImage &m) {
    auto r = common(m); r.modality = a::MeasurementModality::Semantic;
    r.rows = m.height; r.columns = m.width;
    r.class_ids.assign(m.class_ids._buffer, m.class_ids._buffer + m.class_ids._length);
    r.instance_ids.assign(m.instance_ids._buffer, m.instance_ids._buffer + m.instance_ids._length);
    return r;
  }
  template <typename T> void take(dds_entity_t reader) {
    void *samples[32]{}; dds_sample_info_t info[32]{};
    const auto count = checked(dds_take(reader, samples, info, 32, 32), "take");
    if (count == 0) return;
    try {
      for (dds_return_t i = 0; i < count; ++i)
        if (info[i].valid_data) records.push_back(convert(*static_cast<const T *>(samples[i])));
    } catch (...) { dds_return_loan(reader, samples, count); throw; }
    checked(dds_return_loan(reader, samples, count), "return loan");
  }
  dds_entity_t participant_{0}, readers_[3]{};
};
std::string sample(unsigned modality, std::uint32_t seq, std::uint64_t time, std::uint64_t reset = 3) {
  auto payload = inner(static_cast<std::uint16_t>(modality), modality == 2);
  payload[76 + 7 + 6 + 5] = static_cast<char>('a' + modality);
  overwrite(payload, 48, seq, 8); overwrite(payload, 56, time, 8); overwrite(payload, 40, reset, 8);
  auto wire = envelope(payload); overwrite(wire, 8, time, 8); overwrite(wire, 16, seq, 4);
  return wire;
}
void verify_dds(const Readers &readers, std::uint32_t seq, std::uint64_t time, std::uint64_t reset) {
  require(readers.records.size() == 3, "expected exactly one typed measurement per modality");
  unsigned modalities = 0;
  for (const auto &r : readers.records) {
    const auto modality = static_cast<unsigned>(r.modality); modalities |= 1U << modality;
    auto expected = a::decode_measurement_payload(read(sample(modality, seq, time, reset)));
    require(r.product_session_id == expected.product_session_id && r.session_id == expected.session_id &&
        r.instance_id == expected.instance_id && r.sensor_id == expected.sensor_id && r.frame_id == expected.frame_id &&
        r.model_generation == expected.model_generation && r.reset_generation == reset && r.sequence == seq &&
        r.sim_time_ns == time && r.rows == expected.rows && r.columns == expected.columns &&
        r.sample_rate_hz == expected.sample_rate_hz && r.metadata_json == metadata && r.valid_mask == expected.valid_mask &&
        r.class_ids == expected.class_ids && r.instance_ids == expected.instance_ids,
        "typed DDS identity/shape/labels/metadata changed");
    if (modality != 3) require(r.primary.size() == 2 && r.primary[0] == 321.5F && std::isnan(r.primary[1]),
                               "DDS finite/NaN values changed");
    if (modality == 2) require(r.voltage_v.size() == 2 && r.voltage_v[0] == 0.5F && std::isnan(r.voltage_v[1]),
                              "DDS electrical calibration lost");
  }
  require(modalities == 14, "missing typed DDS modality");
}
void publisher_loopback(const std::filesystem::path &executable) {
  constexpr int domain = 6;  // Existing adapter low-domain slots start at 7; Product uses 17.
  Readers readers(domain);
  for (const auto &stream : {" --stream camera", ""}) {
    Process process(executable, std::string("--dds --domain-id 6 --restamp-stdin-records ") +
        "--measurement-product-session-id product --measurement-session-id source" + stream);
    readers.records.clear();
    std::uint32_t seq = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(8);
    unsigned seen = 0;
    do {
      const auto time = 1234567890ULL + static_cast<std::uint64_t>(seq) * 100000000ULL;
      for (unsigned m = 1; m <= 3; ++m) process.write(sample(m, seq, time));
      ++seq;
      const auto retry = Clock::now() + std::chrono::milliseconds(50);
      do {
        readers.poll();
        for (const auto &r : readers.records) seen |= 1U << static_cast<unsigned>(r.modality);
        readers.records.clear();
        if (seen == 14 && readers.matched()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } while (Clock::now() < retry);
      require(Clock::now() < deadline, "measurement DDS best-effort discovery/warmup timed out");
    } while (seen != 14 || !readers.matched());
    readers.poll(); readers.records.clear();
    // Far-apart simulation timestamps prove bypass of both replay pacing and restamping.
    for (int stage = 0; stage < 3; ++stage) {
      const auto time = stage == 2 ? 0ULL : 50000000000ULL + static_cast<std::uint64_t>(stage) * 10000000000ULL;
      const auto current_seq = stage == 2 ? 0 : seq++;
      const auto reset = stage == 2 ? 4ULL : 3ULL;
      for (unsigned m = 1; m <= 3; ++m) process.write(sample(m, current_seq, time, reset));
      until([&] { readers.poll(); return readers.records.size() >= 3; }, "DDS publication was paced/lost", 2);
      verify_dds(readers, current_seq, time, reset); readers.records.clear();
    }
    require(process.close() == 0, "DDS measurement publisher failed at EOF");
    until([&] { return !readers.matched(); }, "old DDS writer not removed");
  }
}

void quote(std::ostream &out, const std::string &value) {
  out << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
    else if (c < 0x20) {
      out << "\\u00" << "0123456789abcdef"[c >> 4] << "0123456789abcdef"[c & 15];
    } else out << static_cast<char>(c);
  }
  out << '"';
}
template <typename T> void array(std::ostream &out, const std::vector<T> &values) {
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) out << ',';
    if (std::isfinite(static_cast<double>(values[i]))) out << std::setprecision(9) << +values[i];
    else out << "null";
  }
  out << ']';
}
void observation(const std::filesystem::path &path, const Readers &readers) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "{\"measurements\":[";
  for (std::size_t i = 0; i < readers.records.size(); ++i) {
    if (i) out << ',';
    const auto &r = readers.records[i];
    out << "{\"modality\":" << static_cast<unsigned>(r.modality);
    const std::pair<const char *, const std::string *> fields[] = {
        {"product_session_id", &r.product_session_id}, {"session_id", &r.session_id},
        {"instance_id", &r.instance_id}, {"sensor_id", &r.sensor_id}, {"frame_id", &r.frame_id},
        {"metadata_json", &r.metadata_json}};
    for (const auto &field : fields) { out << ','; quote(out, field.first); out << ':'; quote(out, *field.second); }
    out << ",\"model_generation\":" << r.model_generation << ",\"reset_generation\":" << r.reset_generation
        << ",\"sequence\":" << r.sequence << ",\"sim_time_ns\":" << r.sim_time_ns
        << ",\"rows\":" << r.rows << ",\"columns\":" << r.columns << ",\"sample_rate_hz\":" << r.sample_rate_hz;
    out << ",\"primary\":"; array(out, r.primary);
    out << ",\"voltage_v\":"; array(out, r.voltage_v);
    out << ",\"class_ids\":"; array(out, r.class_ids);
    out << ",\"instance_ids\":"; array(out, r.instance_ids);
    out << ",\"valid_mask\":"; array(out, r.valid_mask); out << '}';
  }
  out << "]}\n"; out.close(); require(static_cast<bool>(out), "cannot write measurement observation");
}
int observe(int argc, const char *const argv[]) {
  std::filesystem::path output, ready, matched;
  int domain = -1; std::size_t count = 0; bool all_modalities = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    require(i + 1 < argc, "observer argument needs value"); const std::string value = argv[++i];
    if (arg == "--observe-json" && output.empty()) output = value;
    else if (arg == "--ready-file" && ready.empty()) ready = value;
    else if (arg == "--matched-file" && matched.empty()) matched = value;
    else if (arg == "--require-all-modalities" && value == "1") all_modalities = true;
    else if (arg == "--domain-id" && domain == -1) {
      std::size_t end = 0; domain = std::stoi(value, &end);
      require(end == value.size() && domain >= 0 && domain <= 232, "invalid observer domain");
    } else if (arg == "--count" && count == 0) {
      std::size_t end = 0; const auto parsed = std::stoll(value, &end);
      require(end == value.size() && parsed > 0 && parsed <= 10000, "observer count outside 1..10000");
      count = static_cast<std::size_t>(parsed);
    } else throw std::runtime_error("unexpected observer argument: " + arg);
  }
  require(!output.empty() && !ready.empty() && domain >= 0 && count > 0, "observer requires output/ready/domain/count");
  Readers readers(domain);
  auto mark = [](const std::filesystem::path &path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc); out << "{\"ready\":true}\n";
    out.close(); require(static_cast<bool>(out), "cannot write observer readiness");
  };
  mark(ready);
  try {
    if (!matched.empty()) { until([&] { return readers.matched(); }, "measurement DDS match timed out"); mark(matched); }
    until([&] {
      readers.poll();
      unsigned seen = 0;
      for (const auto &record : readers.records) seen |= 1U << static_cast<unsigned>(record.modality);
      return readers.records.size() >= count && (!all_modalities || seen == 14);
    }, "measurement observation timed out");
    observation(output, readers);
  } catch (...) {
    std::error_code error; std::filesystem::remove(ready, error);
    if (!matched.empty()) std::filesystem::remove(matched, error);
    throw;
  }
  std::error_code error; std::filesystem::remove(ready, error);
  if (!matched.empty()) std::filesystem::remove(matched, error);
  return 0;
}
#endif
}  // namespace

int main(int argc, const char *const argv[]) {
  try {
#ifdef LINGTU_MEASUREMENT_DDS_TEST
    if (argc > 1 && std::string(argv[1]) == "--observe-json") return observe(argc, argv);
    require(argc == 2, "expected publisher executable path");
    thermal_envelope_preserves_bytes();
    typed_values_and_rejections();
    independent_streams_and_generations();
    publisher_validation(argv[1]);
    publisher_loopback(argv[1]);
#else
    (void)argc; (void)argv;
    thermal_envelope_preserves_bytes();
    typed_values_and_rejections();
    independent_streams_and_generations();
#endif
    std::puts("measurement protocol passed");
    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "measurement protocol failed: %s\n", e.what());
    return 1;
  }
}
