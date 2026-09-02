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
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "test_dds_domain.hpp"
#include "truth_odom_protocol.hpp"

#ifndef _WIN32
#include <sys/wait.h>
#endif

namespace {

namespace adapter = lingtu::sim::dds_adapter;

constexpr char kSessionId[] = "sim-session";
constexpr std::uint64_t kFirstTimestampNs = 123456789;
constexpr std::uint64_t kPublishPeriodNs = 2000000;

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int test_domain_id() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}

void append_u16(std::string &output, std::uint16_t value) {
  for (int index = 0; index < 2; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_u32(std::string &output, std::uint32_t value) {
  for (int index = 0; index < 4; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_u64(std::string &output, std::uint64_t value) {
  for (int index = 0; index < 8; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_double(std::string &output, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u64(output, bits);
}

std::string make_record(std::uint64_t sequence) {
  std::array<double, adapter::kTruthOdometryPayloadDoubleCount> values{};
  values[0] = 1.0;
  values[1] = 2.0;
  values[2] = 3.0;
  values[3] = 1.0;
  values[7] = 4.0;
  values[8] = 5.0;
  values[9] = 6.0;
  values[10] = 0.1;
  values[11] = 0.2;
  values[12] = 0.3;
  values[13] = 0.01;
  values[48] = 0.015;
  values[49] = 0.02;
  values[84] = 0.025;

  std::string output("LTOD", 4);
  append_u16(output, adapter::kTruthOdometryProtocolVersion);
  append_u16(output, adapter::kTruthOdometryAllFlags);
  append_u32(output, adapter::kTruthOdometryHeaderBytes);
  append_u32(output, adapter::kTruthOdometryPayloadBytes);
  output.append(kSessionId);
  output.append(64 - std::strlen(kSessionId), '\0');
  append_u64(output, 7);
  append_u64(output, 3);
  append_u64(output, sequence);
  append_u64(output, kFirstTimestampNs + sequence * kPublishPeriodNs);
  for (const double value : values) {
    append_double(output, value);
  }
  return output;
}

std::string shell_quote(const std::filesystem::path &path) {
#ifdef _WIN32
  std::string value = path.string();
  std::string quoted = "\"";
  for (const char character : value) {
    quoted += character == '"' ? "\\\"" : std::string(1, character);
  }
  return quoted + "\"";
#else
  const std::string value = path.string();
  std::string quoted = "'";
  for (const char character : value) {
    quoted += character == '\'' ? "'\\''" : std::string(1, character);
  }
  return quoted + "'";
#endif
}

class PublisherProcess final {
 public:
  PublisherProcess(const std::filesystem::path &executable, const std::filesystem::path &ready_file,
                   int domain_id) {
    const std::string command = shell_quote(executable) + " --dds-domain " +
                                std::to_string(domain_id) + " --session-id " + kSessionId +
                                " --parent-frame odom_truth" +
                                " --child-frame base_truth --ready-file " + shell_quote(ready_file);
#ifdef _WIN32
    const std::string wrapped_command = "\"" + command + "\"";
    pipe_ = _popen(wrapped_command.c_str(), "wb");
#else
    pipe_ = popen(command.c_str(), "w");
#endif
    if (pipe_ == nullptr) {
      throw std::runtime_error("failed to launch truth odometry publisher");
    }
  }

  PublisherProcess(const PublisherProcess &) = delete;
  PublisherProcess &operator=(const PublisherProcess &) = delete;

  ~PublisherProcess() {
    if (pipe_ != nullptr) {
      (void)close();
    }
  }

  void write_record(const std::string &record) {
    if (pipe_ == nullptr) {
      throw std::runtime_error("truth odometry publisher stdin is closed");
    }
    const std::size_t written = std::fwrite(record.data(), 1, record.size(), pipe_);
    if (written != record.size() || std::fflush(pipe_) != 0) {
      throw std::runtime_error("failed to write truth odometry record");
    }
  }

  int close() {
    if (pipe_ == nullptr) {
      return exit_code_.value_or(0);
    }
#ifdef _WIN32
    const int status = _pclose(pipe_);
    exit_code_ = status;
#else
    const int status = pclose(pipe_);
    exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
#endif
    pipe_ = nullptr;
    return *exit_code_;
  }

 private:
  std::FILE *pipe_{nullptr};
  std::optional<int> exit_code_;
};

void wait_for_readiness(const std::filesystem::path &ready_file, int domain_id) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  std::string last_document;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::is_regular_file(ready_file)) {
      std::ifstream input(ready_file, std::ios::binary);
      const std::string document{std::istreambuf_iterator<char>(input),
                                 std::istreambuf_iterator<char>()};
      last_document = document;
      const bool complete =
          document.find("lingtu.sim.truth-odom-publisher.ready.v1") != std::string::npos &&
          document.find("\"ready\":true") != std::string::npos &&
          document.find("\"dds_domain\":" + std::to_string(domain_id)) != std::string::npos &&
          document.find(kSessionId) != std::string::npos &&
          document.find(lingtu::message::kSimTruthOdometry.dds_topic) != std::string::npos;
      if (complete) {
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("truth odometry readiness timed out; last document: " +
                           last_document);
}

dds_entity_t create_reader(dds_entity_t participant) {
  const dds_entity_t topic = checked(
      dds_create_topic(participant, &lingtu_dds_Odometry_desc,
                       lingtu::message::kSimTruthOdometry.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(truth_odometry_reader)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::SensorStream);
  return checked(dds_create_reader(participant, topic, qos.get(), nullptr),
                 "dds_create_reader(truth_odometry)");
}

void wait_for_match(dds_entity_t reader) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    const dds_return_t matches = dds_get_matched_publications(reader, nullptr, 0);
    if (matches < 0) {
      throw std::runtime_error(std::string("dds_get_matched_publications: ") +
                               dds_strretcode(-matches));
    }
    if (matches > 0) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("truth odometry DDS discovery timed out");
}

bool take_matching_odometry(dds_entity_t reader) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take: ") + dds_strretcode(-count));
  }
  if (count != 1 || !infos[0].valid_data || samples[0] == nullptr) {
    if (count > 0) {
      checked(dds_return_loan(reader, samples, count), "dds_return_loan");
    }
    return false;
  }
  const auto *message = static_cast<const lingtu_dds_Odometry *>(samples[0]);
  const std::uint64_t timestamp_ns =
      static_cast<std::uint64_t>(message->header.stamp.sec) * 1000000000ULL +
      message->header.stamp.nanosec;
  const bool matches =
      message->header.frame_id != nullptr &&
      std::string(message->header.frame_id) == "odom_truth" && message->child_frame_id != nullptr &&
      std::string(message->child_frame_id) == "base_truth" &&
      timestamp_ns >= kFirstTimestampNs &&
      (timestamp_ns - kFirstTimestampNs) % kPublishPeriodNs == 0 &&
      std::abs(message->pose.pose.position.x - 1.0) < 1e-9 &&
      std::abs(message->pose.pose.position.y - 2.0) < 1e-9 &&
      std::abs(message->pose.pose.position.z - 3.0) < 1e-9 &&
      std::abs(message->pose.pose.orientation.w - 1.0) < 1e-9 &&
      std::abs(message->twist.twist.linear.x - 4.0) < 1e-9 &&
      std::abs(message->twist.twist.angular.z - 0.3) < 1e-9 &&
      std::abs(message->pose.covariance[0] - 0.01) < 1e-9 &&
      std::abs(message->pose.covariance[35] - 0.015) < 1e-9 &&
      std::abs(message->twist.covariance[0] - 0.02) < 1e-9 &&
      std::abs(message->twist.covariance[35] - 0.025) < 1e-9;
  checked(dds_return_loan(reader, samples, count), "dds_return_loan");
  return matches;
}

}  // namespace

int main(int argc, const char *const argv[]) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: test_truth_odom_publisher_dds_process PUBLISHER_EXE\n");
    return 2;
  }

  dds_entity_t participant = 0;
  try {
    const int domain_id = test_domain_id();
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant");
    const dds_entity_t reader = create_reader(participant);
    const auto nonce = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto ready_file =
        std::filesystem::temp_directory_path() / ("lingtu_truth_odom_" + nonce + ".ready.json");
    std::filesystem::remove(ready_file);

    PublisherProcess publisher(argv[1], ready_file, domain_id);
    wait_for_readiness(ready_file, domain_id);
    wait_for_match(reader);

    bool received = false;
    std::uint64_t sequence = 0;
    auto next_publish = std::chrono::steady_clock::now();
    const auto sample_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (!received && std::chrono::steady_clock::now() < sample_deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_publish) {
        publisher.write_record(make_record(sequence++));
        next_publish = now + std::chrono::milliseconds(20);
      }
      received = take_matching_odometry(reader);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(received, "truth odometry DDS sample was not received");
    require(publisher.close() == 0, "valid publisher process must exit cleanly");
    std::filesystem::remove(ready_file);

    const auto invalid_ready_file = std::filesystem::temp_directory_path() /
                                    ("lingtu_truth_odom_invalid_" + nonce + ".ready.json");
    std::filesystem::remove(invalid_ready_file);
    PublisherProcess invalid(argv[1], invalid_ready_file, domain_id);
    wait_for_readiness(invalid_ready_file, domain_id);
    invalid.write_record(make_record(1));
    require(invalid.close() != 0, "first non-zero sequence must fail the publisher process closed");
    std::filesystem::remove(invalid_ready_file);

    dds_delete(participant);
    std::printf("truth odometry DDS process passed: domain=%d topic=%s\n", domain_id,
                lingtu::message::kSimTruthOdometry.dds_topic.data());
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0) {
      dds_delete(participant);
    }
    std::fprintf(stderr, "test_truth_odom_publisher_dds_process failed: %s\n", error.what());
    return 1;
  }
}
