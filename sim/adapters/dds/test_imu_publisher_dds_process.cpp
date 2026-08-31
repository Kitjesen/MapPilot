#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "imu_protocol.hpp"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "test_dds_domain.hpp"

#ifndef _WIN32
#include <sys/wait.h>
#endif

namespace {
namespace adapter = lingtu::sim::dds_adapter;
constexpr char kSessionId[] = "sim-session";

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return static_cast<dds_entity_t>(value);
}
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
int domain() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}
void u16(std::string& out, std::uint16_t value) { out.push_back(static_cast<char>(value)); out.push_back(static_cast<char>(value >> 8U)); }
void u32(std::string& out, std::uint32_t value) { for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>(value >> (i * 8))); }
void u64(std::string& out, std::uint64_t value) { for (int i = 0; i < 8; ++i) out.push_back(static_cast<char>(value >> (i * 8))); }
void dbl(std::string& out, double value) { std::uint64_t bits = 0; std::memcpy(&bits, &value, 8); u64(out, bits); }
// Finite doubles whose exact little-endian bytes inside the LTIM payload are
//   1A 00 00 00 00 00 F0 3F   (contains the SUB byte 0x1A)
//   0D 0A 00 00 00 00 F0 3F   (contains contiguous CRLF)
constexpr std::uint64_t kPayloadSubBits = 0x3FF000000000001AULL;
constexpr std::uint64_t kPayloadCrlfBits = 0x3FF0000000000A0DULL;
double from_bits(std::uint64_t bits) {
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
std::string record(std::uint64_t sequence, std::uint64_t timestamp_ns) {
  std::string out("LTIM", 4);
  u16(out, adapter::kImuProtocolVersion); u16(out, adapter::kImuAllFlags);
  u32(out, adapter::kImuHeaderBytes); u32(out, adapter::kImuPayloadBytes);
  out.append(kSessionId); out.append(64 - std::strlen(kSessionId), '\0');
  u64(out, 7); u64(out, 0); u64(out, sequence); u64(out, timestamp_ns);
  out.append("thunder_01/imu"); out.append(adapter::kImuFrameBytes - std::strlen("thunder_01/imu"), '\0');
  for (double value : std::array<double, 10>{1.0, from_bits(kPayloadSubBits), from_bits(kPayloadCrlfBits),
                                             0.3, 0.01, 0.02, 0.03, 1.0, 2.0, 9.81}) dbl(out, value);
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
  Process(const std::filesystem::path& executable, const std::filesystem::path& ready, int dds_domain) {
    const std::string command = quote(executable) + " --dds-domain " + std::to_string(dds_domain) +
                                " --session-id " + kSessionId + " --ready-file " + quote(ready);
#ifdef _WIN32
    pipe_ = _popen(("\"" + command + "\"").c_str(), "wb");
#else
    pipe_ = popen(command.c_str(), "w");
#endif
    if (pipe_ == nullptr) throw std::runtime_error("failed to launch IMU publisher");
  }
  ~Process() { if (pipe_ != nullptr) close(); }
  void write(const std::string& bytes) { require(std::fwrite(bytes.data(), 1, bytes.size(), pipe_) == bytes.size(), "write failed"); require(std::fflush(pipe_) == 0, "flush failed"); }
  int close() {
    if (pipe_ == nullptr) return 0;
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
void ready(const std::filesystem::path& file, int dds_domain) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::is_regular_file(file)) {
      std::ifstream input(file); const std::string text{std::istreambuf_iterator<char>(input), {}};
      require(text.find("lingtu.sim.imu-publisher.ready.v1") != std::string::npos, "bad readiness schema");
      require(text.find("rt/sim/imu") != std::string::npos, "bad readiness topic");
      require(text.find("\"dds_domain\":" + std::to_string(dds_domain)) != std::string::npos, "bad readiness domain");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("IMU publisher readiness timed out");
}
void wait_for_match(dds_entity_t reader) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto count = dds_get_matched_publications(reader, nullptr, 0);
    if (count < 0) throw std::runtime_error(dds_strretcode(-count));
    if (count > 0) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("IMU DDS discovery timed out");
}
dds_duration_t remaining_dds_time(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) return 0;
  const auto remaining =
      std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
  return static_cast<dds_duration_t>(std::max<std::int64_t>(1, remaining));
}
}  // namespace

int main(int argc, const char* const argv[]) {
  if (argc != 2) return 2;
  dds_entity_t participant = 0;
  try {
    const int dds_domain = domain();
    participant = checked(dds_create_participant(static_cast<dds_domainid_t>(dds_domain), nullptr, nullptr), "participant");
    const auto topic = checked(dds_create_topic(participant, &lingtu_dds_Imu_desc,
                                                lingtu::message::kSimImu.dds_topic.data(), nullptr, nullptr), "topic");
    auto qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::SensorStream);
    const auto reader = checked(dds_create_reader(participant, topic, qos.get(), nullptr), "reader");
    const auto ready_file = std::filesystem::temp_directory_path() / "lingtu_imu_publisher.ready.json";
    std::filesystem::remove(ready_file);
    Process publisher(argv[1], ready_file, dds_domain);
    ready(ready_file, dds_domain);
    wait_for_match(reader);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    const auto condition = checked(dds_create_readcondition(reader, DDS_ANY_STATE), "read condition");
    const auto waitset = checked(dds_create_waitset(participant), "waitset");
    checked(dds_waitset_attach(waitset, condition, static_cast<dds_attach_t>(1)), "waitset attach");

    // SensorStream is best-effort. A reader can discover the remote writer just before the
    // writer discovers this reader, so prove delivery with sequential warm-up records first.
    std::uint64_t next_sequence = 0;
    bool data_plane_ready = false;
    while (!data_plane_ready) {
      const auto retry_deadline = std::min(
          deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(50));
      publisher.write(record(next_sequence, next_sequence + 1));
      ++next_sequence;
      while (!data_plane_ready) {
        void* samples[1]{};
        dds_sample_info_t infos[1]{};
        const auto count = dds_take(reader, samples, infos, 1, 1);
        if (count < 0) throw std::runtime_error(dds_strretcode(-count));
        if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
          const auto* message = static_cast<const lingtu_dds_Imu*>(samples[0]);
          require(message->header.frame_id != nullptr &&
                      std::string(message->header.frame_id) == "thunder_01/imu",
                  "received IMU warm-up payload mismatch");
          data_plane_ready = true;
        }
        if (count > 0) checked(dds_return_loan(reader, samples, count), "warm-up loan");
        if (data_plane_ready) break;
        if (std::chrono::steady_clock::now() >= retry_deadline) break;
        dds_attach_t triggered{};
        const auto triggered_count =
            dds_waitset_wait(waitset, &triggered, 1, remaining_dds_time(retry_deadline));
        if (triggered_count < 0) {
          throw std::runtime_error(std::string("DDS warm-up wait failed: ") +
                                   dds_strretcode(-triggered_count));
        }
      }
      if (!data_plane_ready && std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("timed out establishing IMU DDS delivery");
      }
    }

    // Two real 256-byte LTIM records whose payload region must carry the exact
    // little-endian bytes 1A 00 00 00 00 00 F0 3F and 0D 0A 00 00 00 00 F0 3F.
    const std::string frame0 = record(next_sequence, 123456789);
    const std::string frame1 = record(next_sequence + 1, 125456789);
    for (const std::string& frame : {frame0, frame1}) {
      require(frame.size() == static_cast<std::size_t>(adapter::kImuHeaderBytes) +
                                  static_cast<std::size_t>(adapter::kImuPayloadBytes),
              "fixture must be a real 256-byte LTIM record");
      const std::string payload = frame.substr(adapter::kImuHeaderBytes, adapter::kImuPayloadBytes);
      require(payload.find(static_cast<char>(0x1A)) != std::string::npos,
              "fixture payload must contain byte 0x1A");
      require(payload.find(std::string("\r\n", 2)) != std::string::npos,
              "fixture payload must contain contiguous CRLF");
    }
    publisher.write(frame0);
    publisher.write(frame1);
    const std::uint64_t expected_stamps[2] = {123456789, 125456789};
    const double expected_orientation_x = from_bits(kPayloadSubBits);
    const double expected_orientation_y = from_bits(kPayloadCrlfBits);
    std::size_t seen = 0;
    while (true) {
      void* samples[2]{}; dds_sample_info_t infos[2]{};
      const auto count = dds_take(reader, samples, infos, 2, 2);
      if (count < 0) throw std::runtime_error(dds_strretcode(-count));
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data || samples[i] == nullptr) continue;
        require(seen < 2, "more than two IMU DDS samples");
        const auto* message = static_cast<const lingtu_dds_Imu*>(samples[i]);
        const bool matches = message->header.frame_id != nullptr && std::string(message->header.frame_id) == "thunder_01/imu" &&
                             message->header.stamp.nanosec == expected_stamps[seen] &&
                             std::abs(message->orientation.w - 1.0) < 1e-9 &&
                             std::abs(message->orientation.x - expected_orientation_x) < 1e-9 &&
                             std::abs(message->orientation.y - expected_orientation_y) < 1e-9 &&
                             std::abs(message->angular_velocity.z - 0.03) < 1e-9 &&
                             std::abs(message->linear_acceleration.z - 9.81) < 1e-9;
        require(matches, "received IMU payload mismatch");
        ++seen;
      }
      if (count > 0) checked(dds_return_loan(reader, samples, count), "loan");
      if (seen == 2) {
        require(publisher.close() == 0, "publisher did not exit cleanly");
        dds_delete(participant); std::printf("IMU DDS process loopback passed: topic=%s\n", lingtu::message::kSimImu.dds_topic.data());
        return 0;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("timed out waiting for two IMU DDS samples");
      }
      dds_attach_t triggered{};
      const auto triggered_count =
          dds_waitset_wait(waitset, &triggered, 1, remaining_dds_time(deadline));
      if (triggered_count < 0) {
        throw std::runtime_error(std::string("DDS sample wait failed: ") +
                                 dds_strretcode(-triggered_count));
      }
    }
  } catch (const std::exception& error) {
    if (participant > 0) dds_delete(participant);
    std::fprintf(stderr, "test_imu_publisher_dds_process failed: %s\n", error.what());
    return 1;
  }
}
