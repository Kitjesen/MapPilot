#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "dds/dds.h"
#include "dds_domain.hpp"
#include "imu_protocol.hpp"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

namespace {
namespace adapter = lingtu::sim::dds_adapter;
constexpr char kReadySchema[] = "lingtu.sim.imu-publisher.ready.v1";

struct Config {
  int dds_domain{-1};
  std::string session_id;
  std::filesystem::path ready_file;
};

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return static_cast<dds_entity_t>(value);
}

std::string next_arg(int& index, int argc, const char* const argv[]) {
  if (index + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[index]);
  return argv[++index];
}

bool valid_id(const std::string& value) {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char c = value[index];
    const bool alphanumeric =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9');
    if (!alphanumeric && (index == 0 || (c != '_' && c != '.' && c != '-'))) return false;
  }
  return true;
}

Config parse(int argc, const char* const argv[]) {
  Config config;
  bool domain = false;
  bool session = false;
  bool ready = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--dds-domain" && !domain) {
      config.dds_domain = adapter::parse_supported_dds_domain_id(next_arg(i, argc, argv));
      domain = true;
    } else if (argument == "--session-id" && !session) {
      config.session_id = next_arg(i, argc, argv);
      session = true;
    } else if (argument == "--ready-file" && !ready) {
      config.ready_file = next_arg(i, argc, argv);
      ready = true;
    } else if (argument == "--help" && argc == 2) {
      std::cout << "usage: lingtu_imu_publisher --dds-domain N --session-id ID --ready-file PATH\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unexpected or duplicate argument: " + argument);
    }
  }
  if (!domain || !session || !ready || !valid_id(config.session_id) || config.ready_file.empty()) {
    throw std::runtime_error("all IMU publisher arguments are required and valid");
  }
  return config;
}

void publish_ready(const Config& config) {
  if (config.ready_file.has_parent_path()) std::filesystem::create_directories(config.ready_file.parent_path());
  std::ofstream output(config.ready_file, std::ios::binary | std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create IMU readiness file");
  output << "{\"schema\":\"" << kReadySchema << "\",\"ready\":true,\"dds_domain\":"
         << config.dds_domain << ",\"session_id\":\"" << config.session_id
         << "\",\"topic\":\"" << lingtu::message::kSimImu.dds_topic << "\"}\n";
}

class Writer final {
 public:
  explicit Writer(int domain) {
    participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr), "participant");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr), "publisher");
    topic_ = checked(dds_create_topic(participant_, &lingtu_dds_Imu_desc,
                                      lingtu::message::kSimImu.dds_topic.data(), nullptr, nullptr), "topic");
    auto qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::SensorStream);
    writer_ = checked(dds_create_writer(publisher_, topic_, qos.get(), nullptr), "writer");
  }
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;
  ~Writer() { if (participant_ > 0) dds_delete(participant_); }

  void write(const adapter::ImuRecord& record) {
    lingtu_dds_Imu message{};
    message.header.stamp.sec = static_cast<std::int32_t>(record.timestamp_ns / 1000000000ULL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(record.timestamp_ns % 1000000000ULL);
    message.header.frame_id = const_cast<char*>(record.frame_id.c_str());
    message.orientation.x = record.orientation_wxyz[1];
    message.orientation.y = record.orientation_wxyz[2];
    message.orientation.z = record.orientation_wxyz[3];
    message.orientation.w = record.orientation_wxyz[0];
    message.angular_velocity.x = record.angular_velocity_rps[0];
    message.angular_velocity.y = record.angular_velocity_rps[1];
    message.angular_velocity.z = record.angular_velocity_rps[2];
    message.linear_acceleration.x = record.linear_acceleration_mps2[0];
    message.linear_acceleration.y = record.linear_acceleration_mps2[1];
    message.linear_acceleration.z = record.linear_acceleration_mps2[2];
    checked(dds_write(writer_, &message), "dds_write(imu)");
  }

 private:
  dds_entity_t participant_{0};
  dds_entity_t publisher_{0};
  dds_entity_t topic_{0};
  dds_entity_t writer_{0};
};

int run(const Config& config) {
  Writer writer(config.dds_domain);
  publish_ready(config);
  adapter::ImuStreamValidator validator(config.session_id);
  while (true) {
    adapter::ImuRecord record;
    std::string error;
    const auto status = adapter::read_imu_record(std::cin, record, error);
    if (status == adapter::ImuReadStatus::Eof) return 0;
    if (status == adapter::ImuReadStatus::Error) throw std::runtime_error("IMU input rejected: " + error);
    if (!validator.accept(record, error)) throw std::runtime_error("IMU input rejected: " + error);
    writer.write(record);
  }
}
}  // namespace

int main(int argc, const char* const argv[]) {
  try {
#ifdef _WIN32
    // LTIM frames are binary; without this, text-mode stdin treats payload
    // byte 0x1A as EOF and rewrites CRLF, truncating the record.
    if (_setmode(_fileno(stdin), _O_BINARY) == -1) {
      throw std::runtime_error("failed to switch IMU stdin to binary mode");
    }
#endif
    return run(parse(argc, argv));
  } catch (const std::exception& error) {
    std::fprintf(stderr, "lingtu_imu_publisher failed: %s\n", error.what());
    return 1;
  }
}
