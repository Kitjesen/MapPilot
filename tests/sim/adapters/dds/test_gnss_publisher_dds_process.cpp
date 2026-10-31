#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <wincrypt.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "dds/dds.h"
#include "dds_domain.hpp"
#include "gnss_protocol.hpp"
#include "messages.h"
#include "transport/dds/qos.hpp"
#include "message/generated/topics.hpp"
#include "test_dds_domain.hpp"

#ifndef _WIN32
#include <sys/wait.h>
#endif

namespace {
namespace a = lingtu::sim::dds_adapter;
using Clock = std::chrono::steady_clock;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return static_cast<dds_entity_t>(value);
}
void number(std::ostream& out, double value) {
  if (std::isfinite(value)) out << std::setprecision(17) << value;
  else out << "null";
}
void header(std::ostream& out, const lingtu_dds_Header& h) {
  out << "\"time_ns\":" << static_cast<std::uint64_t>(h.stamp.sec)*1000000000ULL+h.stamp.nanosec
      << ",\"frame_id\":" << std::quoted(h.frame_id ? h.frame_id : "");
}
template <typename T> void quality(std::ostream& out, const T& m) {
  out << ",\"fix_type\":" << m.fix_type << ",\"num_sat\":" << m.num_sat
      << ",\"num_sat_used\":" << m.num_sat_used << ",\"hdop\":";
  number(out, m.hdop); out << ",\"rtcm_age_s\":"; number(out, m.rtcm_age_s);
}
std::string json(const lingtu_dds_GnssFix& m) {
  std::ostringstream out;
  out << '{'; header(out, m.header); quality(out, m);
  out << ",\"lat\":"; number(out, m.latitude);
  out << ",\"lon\":"; number(out, m.longitude);
  out << ",\"alt\":"; number(out, m.altitude);
  out << ",\"cov\":[";
  for (std::size_t i = 0; i < 9; ++i) { if (i) out << ','; number(out, m.position_covariance[i]); }
  out << "]}"; return out.str();
}
std::string json(const lingtu_dds_GnssStatus& m) {
  std::ostringstream out;
  out << '{'; header(out, m.header); quality(out, m);
  out << ",\"device\":" << std::quoted(m.device ? m.device : "")
      << ",\"link_ok\":" << (m.link_ok ? "true" : "false")
      << ",\"rtk\":" << (m.rtk ? "true" : "false")
      << ",\"error\":" << std::quoted(m.error ? m.error : "") << '}';
  return out.str();
}
class Readers final {
 public:
  explicit Readers(int domain) {
    participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr), "participant");
    try {
      const auto fix_topic = checked(dds_create_topic(participant_, &lingtu_dds_GnssFix_desc,
          lingtu::message::kGnssFix.dds_topic.data(), nullptr, nullptr), "fix topic");
      const auto status_topic = checked(dds_create_topic(participant_, &lingtu_dds_GnssStatus_desc,
          lingtu::message::kGnssStatus.dds_topic.data(), nullptr, nullptr), "status topic");
      auto fix_qos = lingtu::dds::make_qos(lingtu::message::kGnssFix.qos_profile);
      auto status_qos = lingtu::dds::make_qos(lingtu::message::kGnssStatus.qos_profile);
      fix_ = checked(dds_create_reader(participant_, fix_topic, fix_qos.get(), nullptr), "fix reader");
      status_ = checked(dds_create_reader(participant_, status_topic, status_qos.get(), nullptr), "status reader");
    } catch (...) { dds_delete(participant_); participant_ = 0; throw; }
  }
  Readers(const Readers&) = delete;
  Readers& operator=(const Readers&) = delete;
  ~Readers() { if (participant_ > 0) dds_delete(participant_); }
  void poll() {
    take<lingtu_dds_GnssFix>(fix_, fixes);
    take<lingtu_dds_GnssStatus>(status_, statuses);
  }
  bool matched() const {
    return checked(dds_get_matched_publications(fix_, nullptr, 0), "fix match") > 0 &&
        checked(dds_get_matched_publications(status_, nullptr, 0), "status match") > 0;
  }
  void clear() { fixes.clear(); statuses.clear(); }
  std::vector<std::string> fixes, statuses;
 private:
  template <typename T> void take(dds_entity_t reader, std::vector<std::string>& output) {
    for (;;) {
      void* samples[16]{}; dds_sample_info_t info[16]{};
      const auto count = checked(dds_take(reader, samples, info, 16, 16), "take");
      if (count == 0) return;
      try {
        for (dds_return_t i = 0; i < count; ++i)
          if (info[i].valid_data && samples[i]) output.push_back(json(*static_cast<const T*>(samples[i])));
      } catch (...) { dds_return_loan(reader, samples, count); throw; }
      checked(dds_return_loan(reader, samples, count), "loan");
    }
  }
  dds_entity_t participant_{0}, fix_{0}, status_{0};
};
template <typename Predicate> void until(Predicate predicate, Clock::time_point deadline, const char* message) {
  while (!predicate()) {
    if (Clock::now() >= deadline) throw std::runtime_error(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
void output_json(const std::filesystem::path& path, const Readers& r) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "{\"fixes\":[";
  for (std::size_t i = 0; i < r.fixes.size(); ++i) { if (i) out << ','; out << r.fixes[i]; }
  out << "],\"statuses\":[";
  for (std::size_t i = 0; i < r.statuses.size(); ++i) { if (i) out << ','; out << r.statuses[i]; }
  out << "]}\n"; out.close(); require(static_cast<bool>(out), "cannot write observation JSON");
}
int observe(int argc, const char* const argv[]) {
  std::filesystem::path output, ready, matched;
  int domain = -1, count = 0;
  bool minimum_count = false, count_mode_set = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    require(i+1 < argc, "missing observer argument value");
    const std::string value = argv[++i];
    if (arg == "--observe-json" && output.empty()) output = value;
    else if (arg == "--ready-file" && ready.empty()) ready = value;
    else if (arg == "--matched-file" && matched.empty()) matched = value;
    else if (arg == "--domain-id" && domain == -1) domain = a::parse_supported_dds_domain_id(value);
    else if (arg == "--count-mode" && !count_mode_set) {
      require(value == "exact" || value == "minimum", "count mode must be exact or minimum");
      count_mode_set = true;
      minimum_count = value == "minimum";
    }
    else if (arg == "--count" && count == 0) {
      std::size_t consumed = 0; count = std::stoi(value, &consumed);
      require(consumed == value.size() && count > 0, "count must be positive integer");
    } else throw std::runtime_error("unexpected observer argument: " + arg);
  }
  require(!output.empty() && !ready.empty() && domain >= 0 && count > 0, "observer requires output/domain/count/ready");
  Readers readers(domain);
  {
    std::ofstream out(ready, std::ios::binary | std::ios::trunc);
    out << "{\"ready\":true,\"dds_domain\":" << domain << "}\n";
    out.close(); require(static_cast<bool>(out), "cannot write observer readiness");
  }
  try {
    const auto deadline = Clock::now()+std::chrono::seconds(10);
    if (!matched.empty()) {
      until([&] { return readers.matched(); }, deadline, "GNSS publication matching timed out (10s)");
      // Reader-side discovery is not a data-plane delivery acknowledgement.
      std::ofstream out(matched, std::ios::binary | std::ios::trunc);
      out << "{\"matched\":true,\"dds_domain\":" << domain << "}\n";
      out.close(); require(static_cast<bool>(out), "cannot write observer matched file");
    }
    const auto expected = static_cast<std::size_t>(count);
    until([&] { readers.poll(); return readers.fixes.size() >= expected && readers.statuses.size() >= expected; },
          deadline, "GNSS observation timed out (10s)");
    if (!minimum_count) {
      require(readers.fixes.size() == expected && readers.statuses.size() == expected, "unexpected extra GNSS samples");
    }
    output_json(output, readers);
  } catch (...) {
    std::error_code error; std::filesystem::remove(ready, error);
    if (!matched.empty()) std::filesystem::remove(matched, error);
    throw;
  }
  std::error_code error; std::filesystem::remove(ready, error);
  if (!matched.empty()) std::filesystem::remove(matched, error);
  return 0;
}

void integer(std::string& out, std::uint64_t value, unsigned size) {
  for (unsigned i = 0; i < size; ++i) out.push_back(static_cast<char>(value >> (i*8)));
}
double from_bits(std::uint64_t bits) { double value; std::memcpy(&value, &bits, sizeof(value)); return value; }
std::string record(std::uint64_t sequence, std::uint64_t timestamp, bool fix = true,
                   std::uint64_t reset = 0, std::uint64_t model = 7) {
  std::string out("LTGN", 4);
  integer(out, 1, 2); integer(out, 7, 2); integer(out, 176, 4); integer(out, 124, 4);
  out += "sim-session"; out.append(64-11, '\0');
  integer(out, model, 8); integer(out, reset, 8); integer(out, sequence, 8); integer(out, timestamp, 8);
  out += "robot/gnss"; out.append(64-10, '\0');
  integer(out, fix ? 8 : 0, 4); integer(out, 0, 4); integer(out, 0, 4);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // Exact finite values carrying SUB and CRLF prove Windows binary stdin.
  const double values[14] = {fix ? from_bits(0x3FF000000000001AULL) : nan,
      fix ? from_bits(0x3FF0000000000A0DULL) : nan, fix ? 15.0 : nan,
      fix ? 1.0 : nan, fix ? 0.1 : nan, fix ? 0.0 : nan,
      fix ? 0.1 : nan, fix ? 2.0 : nan, fix ? 0.0 : nan,
      fix ? 0.0 : nan, fix ? 0.0 : nan, fix ? 3.0 : nan, nan, nan};
  for (double value : values) { std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); integer(out, bits, 8); }
  return out;
}
std::string quote(const std::filesystem::path& path) {
#ifdef _WIN32
  return "\"" + path.string() + "\"";
#else
  return "'" + path.string() + "'";
#endif
}
class Process final {
 public:
  Process(const std::filesystem::path& executable, const std::filesystem::path& ready, int domain,
          const std::string& sensor_options = {}, bool endpoint = false) {
    const auto command = quote(executable) + (sensor_options.empty()
        ? " --dds-domain "+std::to_string(domain)+" --session-id sim-session"
        : (endpoint ? " --local-endpoint --dds --domain-id " : " --stdin-records --dds --domain-id ")+
            std::to_string(domain)+" "+sensor_options) +
        (endpoint ? " 2>"+quote(ready.string()+".stderr") : " --ready-file "+quote(ready));
#ifdef _WIN32
    pipe_ = _popen(("\""+command+"\"").c_str(), "wb");
#else
    pipe_ = popen(command.c_str(), "w");
#endif
    require(pipe_ != nullptr, "cannot launch GNSS publisher");
  }
  Process(const Process&) = delete;
  Process& operator=(const Process&) = delete;
  ~Process() { if (pipe_) close(); }
  void write(const std::string& bytes) {
    require(std::fwrite(bytes.data(), 1, bytes.size(), pipe_) == bytes.size(), "pipe write failed");
    require(std::fflush(pipe_) == 0, "pipe flush failed");
  }
  int close() {
#ifdef _WIN32
    const int result = _pclose(pipe_);
#else
    const int status = pclose(pipe_); const int result = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
    pipe_ = nullptr; return result;
  }
 private:
  std::FILE* pipe_{nullptr};
};
void ready(const std::filesystem::path& file, int domain) {
  until([&] {
    if (!std::filesystem::is_regular_file(file)) return false;
    std::ifstream input(file); const std::string text{std::istreambuf_iterator<char>(input), {}};
    return text.find("\"schema\":\"lingtu.sim.gnss-publisher.ready.v1\"") != std::string::npos &&
        text.find("\"ready\":true") != std::string::npos &&
        text.find("\"session_id\":\"sim-session\"") != std::string::npos &&
        text.find("\"dds_domain\":"+std::to_string(domain)) != std::string::npos &&
        text.find("\"topic\":\"rt/gnss/fix\"") != std::string::npos &&
        text.find("\"status_topic\":\"rt/gnss/status\"") != std::string::npos;
  }, Clock::now()+std::chrono::seconds(8), "GNSS readiness invalid or timed out");
}
void verify(const Readers& readers, std::uint64_t stamp, bool fix) {
  const auto stamp_json = "\"time_ns\":"+std::to_string(stamp)+",";
  const auto fix_json = "\"fix_type\":"+std::to_string(fix ? 8 : 0)+",";
  for (const auto* list : {&readers.fixes, &readers.statuses}) {
    require(list->size() == 1, "expected exactly one sample per topic");
    const auto& sample = list->front();
    require(sample.find(stamp_json) != std::string::npos && sample.find(fix_json) != std::string::npos &&
        sample.find("\"frame_id\":\"robot/gnss\"") != std::string::npos &&
        sample.find("\"num_sat\":0,\"num_sat_used\":0,\"hdop\":null,\"rtcm_age_s\":null") != std::string::npos,
        "GNSS shared payload mismatch");
  }
  const auto& status = readers.statuses.front();
  require(status.find("\"device\":\"simulation\",\"link_ok\":true,\"rtk\":false") != std::string::npos &&
      status.find(fix ? "\"error\":\"\"" : "\"error\":\"simulation_no_fix\"") != std::string::npos,
      "GNSS status mismatch");
  const auto& sample = readers.fixes.front();
  if (fix) {
    std::ostringstream expected;
    expected << "\"lat\":"; number(expected, from_bits(0x3FF000000000001AULL));
    expected << ",\"lon\":"; number(expected, from_bits(0x3FF0000000000A0DULL));
    expected << ",\"alt\":15,\"cov\":[1,"; number(expected, 0.1); expected << ",0,";
    number(expected, 0.1); expected << ",2,0,0,0,3]";
    require(sample.find(expected.str()) != std::string::npos, "SIMULATION coordinates/covariance mismatch");
  } else require(sample.find("\"lat\":null,\"lon\":null,\"alt\":null,\"cov\":[null,null,null,null,null,null,null,null,null]") != std::string::npos,
                 "NO_FIX leaked finite coordinates/covariance");
}
int loopback(const std::filesystem::path& executable) {
  const int domain = a::test::domain_id_from_environment();
  Readers readers(domain);
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-gnss-test-"+std::to_string(Clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(directory);
  const auto file = directory / "publisher.ready.json";
  try {
    {
      Process process(executable, file, domain);
      ready(file, domain);
      until([&] { return readers.matched(); }, Clock::now()+std::chrono::seconds(8), "DDS discovery timed out");
      // Best-effort fix delivery needs a bidirectional discovery warm-up.
      std::uint64_t sequence = 0;
      bool fix_seen = false, status_seen = false;
      const auto deadline = Clock::now()+std::chrono::seconds(8);
      while (!fix_seen || !status_seen) {
        process.write(record(sequence, sequence+1)); ++sequence;
        const auto retry = Clock::now()+std::chrono::milliseconds(50);
        do {
          readers.poll(); fix_seen = fix_seen || !readers.fixes.empty(); status_seen = status_seen || !readers.statuses.empty();
          if (fix_seen && status_seen) break;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (Clock::now() < retry);
        require(Clock::now() < deadline, "DDS data plane warm-up timed out");
      }
      readers.poll(); readers.clear();
      for (int stage = 0; stage < 3; ++stage) {
        const auto stamp = stage == 2 ? 0ULL : 1234567890ULL+static_cast<std::uint64_t>(stage)*200000000ULL;
        const auto bytes = record(stage == 2 ? 0 : sequence++, stamp, stage != 1, stage == 2 ? 1 : 0);
        require(bytes.size() == 300, "fixture is not a 300-byte LTGN record");
        if (stage != 1) {
          const auto payload = bytes.substr(176);
          require(payload.find(static_cast<char>(0x1a)) != std::string::npos && payload.find("\r\n") != std::string::npos,
                  "binary stdin fixture lacks SUB/CRLF");
        }
        process.write(bytes);
        until([&] { readers.poll(); return !readers.fixes.empty() && !readers.statuses.empty(); },
              Clock::now()+std::chrono::seconds(8), "GNSS fix/status delivery timed out");
        verify(readers, stamp, stage != 1); readers.clear();
      }
      require(process.close() == 0, "publisher failed at EOF");
      require(!std::filesystem::exists(file), "readiness survived EOF");
    }
    std::vector<std::string> invalid;
    invalid.push_back(record(0, 1).substr(0, 299));
    auto bad = record(0, 1); bad[0] = 'x'; invalid.push_back(bad);
    bad = record(0, 1); bad[16] = 'x'; invalid.push_back(bad);
    bad = record(0, 1); bad[176] = 4; invalid.push_back(bad);
    invalid.push_back(record(1, 1));
    invalid.push_back(record(0, 1)+record(0, 1));
    invalid.push_back(record(0, 2)+record(1, 1));
    invalid.push_back(record(0, 2, true, 1)+record(0, 1));
    invalid.push_back(record(0, 2)+record(0, 1, true, 0, 6));
    for (const auto& bytes : invalid) {
      Process process(executable, file, domain); ready(file, domain); process.write(bytes);
      require(process.close() != 0, "publisher accepted invalid stream");
      require(!std::filesystem::exists(file), "readiness survived input failure");
    }
    { Process process(executable, file, domain); ready(file, domain); require(process.close() == 0, "empty EOF failed"); }
    require(!std::filesystem::exists(file), "readiness survived empty EOF");
  } catch (...) {
    std::error_code error; std::filesystem::remove(file, error); std::filesystem::remove(directory, error); throw;
  }
  std::filesystem::remove(directory);
  std::puts("GNSS DDS process loopback passed: FIX, NO_FIX, reset, invalid input, EOF, binary stdin"); return 0;
}

std::string envelope(const std::string& inner, std::uint64_t stamp, std::uint32_t sequence) {
  std::string out("LTU1\x08\x00\x00\x00", 8);
  integer(out, stamp, 8); integer(out, sequence, 4);
  integer(out, 1, 4); integer(out, inner.size(), 4);
  return out + inner;
}

int sensor_loopback(const std::filesystem::path& executable) {
  constexpr int domain = 134;
  Readers readers(domain);
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-ltu1-gnss-"+std::to_string(Clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(directory);
  const auto file = directory / "sensor.ready.json";
  for (const auto& options : {"--stream imu --restamp-stdin-records", "--restamp-stdin-records"}) {
    std::filesystem::remove(file);
    Process process(executable, file, domain, options);
    until([&] { return std::filesystem::is_regular_file(file); },
          Clock::now()+std::chrono::seconds(8), "sensor readiness timed out");
    until([&] { return !readers.matched(); }, Clock::now()+std::chrono::seconds(2),
          "GNSS writers must stay absent before first GNSS input");
    std::uint32_t sequence = 0;
    const auto deadline = Clock::now()+std::chrono::seconds(8);
    while (readers.fixes.empty() || readers.statuses.empty()) {
      const auto stamp = static_cast<std::uint64_t>(sequence)+1;
      process.write(envelope(record(sequence, stamp), stamp, sequence)); ++sequence;
      const auto retry = Clock::now()+std::chrono::milliseconds(50);
      do { readers.poll(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
      while ((readers.fixes.empty() || readers.statuses.empty()) && Clock::now() < retry);
      require(Clock::now() < deadline, "LTU1 GNSS DDS warm-up timed out");
    }
    readers.poll(); readers.clear();
    for (int stage = 0; stage < 3; ++stage) {
      const auto stamp = stage == 2 ? 0ULL : 1234567890ULL+static_cast<std::uint64_t>(stage)*4000000000ULL;
      const auto seq = stage == 2 ? 0 : sequence++;
      const auto begin = Clock::now();
      process.write(envelope(record(seq, stamp, stage != 1, stage == 2 ? 1 : 0), stamp, seq));
      until([&] { readers.poll(); return !readers.fixes.empty() && !readers.statuses.empty(); },
            begin+std::chrono::seconds(2), "GNSS must bypass wall-clock replay pacing");
      verify(readers, stamp, stage != 1); readers.clear();
    }
    require(process.close() == 0, "LTU1 GNSS publisher failed at EOF");
  }
  const auto good = envelope(record(0, 1), 1, 0);
  for (const auto& options : {"--stream lidar", "--stream camera", "--stream imu"}) {
    std::vector<std::string> invalid;
    if (std::string(options) != "--stream imu") invalid.push_back(good);
    else {
      invalid.push_back(envelope(record(0, 1), 2, 0));
      invalid.push_back(envelope(record(0, 1), 1, 1));
      auto bad = good; bad[28] = 'x'; invalid.push_back(bad);
      invalid.push_back(envelope(record(1, 1), 1, 1));
      invalid.push_back(good + good);
      invalid.push_back(envelope(record(0, 2), 2, 0)+envelope(record(1, 1), 1, 1));
      auto other_session = record(1, 2); other_session[16] = 'x';
      invalid.push_back(good+envelope(other_session, 2, 1));
    }
    for (const auto& bytes : invalid) {
      Process process(executable, file, domain, options);
      process.write(bytes);
      require(process.close() != 0, "sensor publisher accepted invalid GNSS/stream");
    }
  }
  std::filesystem::remove(file);
  std::filesystem::remove(directory);
  std::puts("LTU1 GNSS DDS passed: lazy writer, simulation time, NO_FIX, reset, no pacing/restamp");
  return 0;
}

std::string file_text(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "cannot read endpoint fixture file");
  return {std::istreambuf_iterator<char>(input), {}};
}

class Environment final {
 public:
  Environment(const char* name, const std::string& value) : name_(name) {
#ifdef _WIN32
    char* old = nullptr; std::size_t size = 0;
    require(_dupenv_s(&old, &size, name) == 0, "cannot read test environment");
    if (old) previous_ = std::string(old);
    std::free(old);
#else
    if (const auto* old = std::getenv(name)) previous_ = old;
#endif
    set(value.c_str());
  }
  ~Environment() { set(previous_ ? previous_->c_str() : nullptr); }
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
 private:
  void set(const char* value) const {
#ifdef _WIN32
    _putenv_s(name_, value ? value : "");
#else
    if (value) setenv(name_, value, 1); else unsetenv(name_);
#endif
  }
  const char* name_;
  std::optional<std::string> previous_;
};

class EndpointClient final {
 public:
  explicit EndpointClient(const std::filesystem::path& ready) {
    const auto description = file_text(ready);
    require(description.find("\"role\":\"imu_publisher\"") != std::string::npos &&
        description.find("\"protocol\":\"ltu1-v1\"") != std::string::npos,
        "GNSS must reuse the existing IMU endpoint contract");
    const auto port_at = description.find("\"port\":");
    require(port_at != std::string::npos, "endpoint port missing");
    const auto port = std::stoul(description.substr(port_at+7));
    auto secret = file_text(ready.parent_path()/"imu.auth");
#ifdef _WIN32
    WSADATA data{};
    require(WSAStartup(MAKEWORD(2, 2), &data) == 0, "WSAStartup failed");
    DATA_BLOB protected_secret{static_cast<DWORD>(secret.size()), reinterpret_cast<BYTE*>(secret.data())};
    DATA_BLOB plain{};
    require(CryptUnprotectData(&protected_secret, nullptr, nullptr, nullptr, nullptr,
        CRYPTPROTECT_UI_FORBIDDEN, &plain) != FALSE, "endpoint auth decrypt failed");
    secret.assign(reinterpret_cast<const char*>(plain.pbData), plain.cbData);
    LocalFree(plain.pbData);
#endif
    require(secret.size() == 32, "endpoint secret must be 32 bytes");
    std::string nonce;
    for (const unsigned char byte : secret) {
      nonce += "0123456789abcdef"[byte >> 4]; nonce += "0123456789abcdef"[byte & 15];
    }
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef _WIN32
    require(socket_ != INVALID_SOCKET, "endpoint socket failed");
    const DWORD timeout = 2000;
#else
    require(socket_ >= 0, "endpoint socket failed");
    const timeval timeout{2, 0};
#endif
    require(setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout),
        sizeof(timeout)) == 0, "endpoint receive timeout failed");
    sockaddr_in address{};
    address.sin_family = AF_INET; address.sin_port = htons(static_cast<std::uint16_t>(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    require(connect(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
        "endpoint connect failed");
    const std::string request = "{\"nonce\":\""+nonce+"\",\"product_session_id\":\"sim-session\","
        "\"protocol\":\"ltu1-v1\",\"role\":\"imu_publisher\","
        "\"schema\":\"lingtu.sim.local_endpoint.connect.v1\"}";
    std::string frame;
    for (int shift : {24, 16, 8, 0}) frame += static_cast<char>(request.size() >> shift);
    send_all(frame+request);
    const auto size_bytes = receive(4);
    std::uint32_t size = 0;
    for (const unsigned char byte : size_bytes) size = (size << 8) | byte;
    require(size > 0 && size < 4096, "endpoint ACK frame size invalid");
    const auto ack = receive(size);
    require(ack.find("\"ok\":true") != std::string::npos &&
        ack.find("\"product_session_id\":\"sim-session\"") != std::string::npos,
        "IMU endpoint authentication failed");
  }
  EndpointClient(const EndpointClient&) = delete;
  EndpointClient& operator=(const EndpointClient&) = delete;
  ~EndpointClient() {
#ifdef _WIN32
    closesocket(socket_); WSACleanup();
#else
    close(socket_);
#endif
  }
  void send_all(const std::string& bytes) const {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto count = send(socket_, bytes.data()+offset, static_cast<int>(bytes.size()-offset), 0);
      require(count > 0, "endpoint send failed"); offset += static_cast<std::size_t>(count);
    }
  }
 private:
  std::string receive(std::size_t size) const {
    std::string result(size, '\0');
    std::size_t offset = 0;
    while (offset < size) {
      const auto count = recv(socket_, result.data()+offset, static_cast<int>(size-offset), 0);
      require(count > 0, "endpoint receive failed"); offset += static_cast<std::size_t>(count);
    }
    return result;
  }
#ifdef _WIN32
  SOCKET socket_{INVALID_SOCKET};
#else
  int socket_{-1};
#endif
};

int sensor_endpoint(const std::filesystem::path& executable) {
  constexpr int domain = 134;
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-gnss-endpoint-"+std::to_string(Clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(directory);
  const auto plan = directory/"plan.json";
  { std::ofstream output(plan); output << "{}\n"; }
  Environment run_plan("LINGTU_RUN_PLAN", plan.string());
  Environment session("LINGTU_PRODUCT_SESSION_ID", "sim-session");
  const auto ready_file = directory/"imu.ready.json";
  const auto error_file = ready_file.string()+".stderr";
  Readers readers(domain);
  for (const bool wrong_session : {true, false}) {
    Process process(executable, ready_file, domain, "--stream imu", true);
    until([&] { return std::filesystem::is_regular_file(ready_file); },
          Clock::now()+std::chrono::seconds(8), "IMU endpoint readiness timed out");
    {
      EndpointClient client(ready_file);
      if (wrong_session) {
        auto inner = record(0, 1); inner[16] = 'x';
        client.send_all(envelope(inner, 1, 0));
      } else {
        std::uint32_t sequence = 0;
        const auto deadline = Clock::now()+std::chrono::seconds(8);
        readers.clear();
        while (readers.fixes.empty() || readers.statuses.empty()) {
          const auto stamp = static_cast<std::uint64_t>(sequence)+1;
          client.send_all(envelope(record(sequence, stamp), stamp, sequence)); ++sequence;
          const auto retry = Clock::now()+std::chrono::milliseconds(50);
          do { readers.poll(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
          while ((readers.fixes.empty() || readers.statuses.empty()) && Clock::now() < retry);
          require(Clock::now() < deadline, "formal IMU GNSS DDS warm-up timed out");
        }
        readers.poll(); readers.clear();
        client.send_all(envelope(record(sequence, 9876543210ULL, false), 9876543210ULL, sequence));
        until([&] { readers.poll(); return !readers.fixes.empty() && !readers.statuses.empty(); },
              Clock::now()+std::chrono::seconds(2), "formal endpoint GNSS delivery timed out");
        verify(readers, 9876543210ULL, false);
        readers.clear();
      }
    }
    // Deliberately close the feeder: the existing formal contract rejects premature EOF.
    require(process.close() != 0, "formal endpoint must reject wrong session or early EOF");
    const auto error = file_text(error_file);
    require(error.find(wrong_session ? "GNSS session id mismatch" :
        "sensor endpoint closed before stop was requested") != std::string::npos,
        "formal GNSS rejection reason mismatch");
    require(!std::filesystem::exists(ready_file) && !std::filesystem::exists(directory/"imu.auth"),
        "formal endpoint artifacts survived exit");
  }
  std::filesystem::remove(error_file); std::filesystem::remove(plan);
  std::filesystem::remove(directory);
  std::puts("Formal IMU GNSS endpoint passed: authentication, Product session, native DDS, EOF cleanup");
  return 0;
}
} // namespace
int main(int argc, const char* const argv[]) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--observe-json") return observe(argc, argv);
    if (argc == 3 && std::string(argv[1]) == "--sensor-publisher") return sensor_loopback(argv[2]);
    if (argc == 3 && std::string(argv[1]) == "--sensor-endpoint") return sensor_endpoint(argv[2]);
    require(argc == 2, "expected publisher path or --observe-json OUT --domain-id N --count N --ready-file READY [--matched-file MATCHED] [--count-mode exact|minimum]");
    return loopback(argv[1]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "test_gnss_publisher_dds_process failed: %s\n", e.what()); return 1;
  }
}
