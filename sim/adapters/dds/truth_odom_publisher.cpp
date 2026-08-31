#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "dds/dds.h"
#include "dds_domain.hpp"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "truth_odom_protocol.hpp"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

namespace adapter = lingtu::sim::dds_adapter;

constexpr char kReadySchema[] = "lingtu.sim.truth-odom-publisher.ready.v1";

struct CliConfig {
  bool show_help{false};
  int dds_domain{-1};
  std::string session_id;
  std::string parent_frame;
  std::string child_frame;
  std::filesystem::path ready_file;
};

constexpr char kUsage[] =
    "usage: lingtu_truth_odom_publisher --dds-domain N "
    "--session-id ID --parent-frame FRAME --child-frame FRAME "
    "--ready-file PATH\n";

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::string next_argument(int &index, int argc, const char *const argv[]) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  return argv[++index];
}

bool valid_id(const std::string &value) noexcept {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char character = value[index];
    const bool alphanumeric =
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
    if (!alphanumeric &&
        (index == 0 || (character != '_' && character != '.' && character != '-'))) return false;
  }
  return true;
}

void validate_frame(const std::string &value, const char *option) {
  if (value.empty() || std::any_of(value.begin(), value.end(), [](char character) {
        return std::isspace(static_cast<unsigned char>(character)) != 0;
      })) {
    throw std::runtime_error(std::string(option) + " must be one non-empty frame token");
  }
}

CliConfig parse_arguments(int argc, const char *const argv[]) {
  CliConfig config;
  bool saw_domain = false;
  bool saw_session = false;
  bool saw_parent = false;
  bool saw_child = false;
  bool saw_ready = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      config.show_help = true;
    } else if (argument == "--dds-domain" && !saw_domain) {
      config.dds_domain = adapter::parse_supported_dds_domain_id(next_argument(index, argc, argv));
      saw_domain = true;
    } else if (argument == "--session-id" && !saw_session) {
      config.session_id = next_argument(index, argc, argv);
      saw_session = true;
    } else if (argument == "--parent-frame" && !saw_parent) {
      config.parent_frame = next_argument(index, argc, argv);
      saw_parent = true;
    } else if (argument == "--child-frame" && !saw_child) {
      config.child_frame = next_argument(index, argc, argv);
      saw_child = true;
    } else if (argument == "--ready-file" && !saw_ready) {
      config.ready_file = next_argument(index, argc, argv);
      saw_ready = true;
    } else {
      throw std::runtime_error("unexpected or duplicate argument: " + argument);
    }
  }
  if (config.show_help) {
    if (argc != 2) {
      throw std::runtime_error("--help cannot be combined with runtime arguments");
    }
    return config;
  }
  if (!saw_domain || !saw_session || !saw_parent || !saw_child || !saw_ready) {
    throw std::runtime_error("all truth odometry publisher arguments are required");
  }
  if (!valid_id(config.session_id)) {
    throw std::runtime_error("--session-id must be a 1-63 character ASCII slug");
  }
  validate_frame(config.parent_frame, "--parent-frame");
  validate_frame(config.child_frame, "--child-frame");
  if (config.parent_frame == config.child_frame) {
    throw std::runtime_error("parent and child frames must differ");
  }
  if (config.ready_file.empty()) {
    throw std::runtime_error("--ready-file must not be empty");
  }
  return config;
}

class ReadyFile final {
 public:
  ReadyFile(std::filesystem::path path, int dds_domain, std::string session_id)
      : path_(std::move(path)),
        dds_domain_(dds_domain),
        session_id_(std::move(session_id)) {
    std::error_code error;
    std::filesystem::remove(path_, error);
    std::filesystem::remove(temporary_path(), error);
  }

  void publish() const {
    if (path_.has_parent_path()) {
      std::filesystem::create_directories(path_.parent_path());
    }
    const auto temporary = temporary_path();
    {
      std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
      if (!output) {
        throw std::runtime_error("cannot create truth odometry readiness file");
      }
      output << "{\"schema\":\"" << kReadySchema
             << "\",\"ready\":true,\"dds_domain\":" << dds_domain_ << ",\"session_id\":\""
             << session_id_ << "\",\"topic\":\"" << lingtu::message::kSimTruthOdometry.dds_topic
             << "\"}\n";
      if (!output) {
        throw std::runtime_error("cannot write truth odometry readiness file");
      }
    }
    std::error_code error;
    std::filesystem::rename(temporary, path_, error);
    if (!error) {
      return;
    }
    std::filesystem::remove(path_, error);
    error.clear();
    std::filesystem::rename(temporary, path_, error);
    if (error) {
      throw std::runtime_error("cannot publish truth odometry readiness file: " + error.message());
    }
  }

 private:
  std::filesystem::path temporary_path() const {
    auto temporary = path_;
    temporary += ".tmp";
    return temporary;
  }

  std::filesystem::path path_;
  int dds_domain_;
  std::string session_id_;
};

class TruthOdometryWriter final {
 public:
  TruthOdometryWriter(int dds_domain, std::string parent_frame, std::string child_frame)
      : parent_frame_(std::move(parent_frame)), child_frame_(std::move(child_frame)) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(dds_domain), nullptr, nullptr),
                "dds_create_participant(truth_odometry)");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher(truth_odometry)");
    topic_ = checked(dds_create_topic(participant_, &lingtu_dds_Odometry_desc,
                                      lingtu::message::kSimTruthOdometry.dds_topic.data(), nullptr,
                                      nullptr),
                     "dds_create_topic(truth_odometry)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::SensorStream);
    writer_ = checked(dds_create_writer(publisher_, topic_, qos.get(), nullptr),
                      "dds_create_writer(truth_odometry)");
  }

  TruthOdometryWriter(const TruthOdometryWriter &) = delete;
  TruthOdometryWriter &operator=(const TruthOdometryWriter &) = delete;

  ~TruthOdometryWriter() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  void publish(const adapter::TruthOdometryRecord &record) const {
    lingtu_dds_Odometry message{};
    message.header.stamp.sec = static_cast<std::int32_t>(record.timestamp_ns / 1000000000ULL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(record.timestamp_ns % 1000000000ULL);
    message.header.frame_id = const_cast<char *>(parent_frame_.c_str());
    message.child_frame_id = const_cast<char *>(child_frame_.c_str());
    message.pose.pose.position.x = record.position_m[0];
    message.pose.pose.position.y = record.position_m[1];
    message.pose.pose.position.z = record.position_m[2];
    message.pose.pose.orientation.w = record.orientation_wxyz[0];
    message.pose.pose.orientation.x = record.orientation_wxyz[1];
    message.pose.pose.orientation.y = record.orientation_wxyz[2];
    message.pose.pose.orientation.z = record.orientation_wxyz[3];
    message.twist.twist.linear.x = record.linear_velocity_mps[0];
    message.twist.twist.linear.y = record.linear_velocity_mps[1];
    message.twist.twist.linear.z = record.linear_velocity_mps[2];
    message.twist.twist.angular.x = record.angular_velocity_rps[0];
    message.twist.twist.angular.y = record.angular_velocity_rps[1];
    message.twist.twist.angular.z = record.angular_velocity_rps[2];
    std::copy(record.pose_covariance.begin(), record.pose_covariance.end(),
              message.pose.covariance);
    std::copy(record.twist_covariance.begin(), record.twist_covariance.end(),
              message.twist.covariance);
    checked(dds_write(writer_, &message), "dds_write(truth_odometry)");
  }

 private:
  std::string parent_frame_;
  std::string child_frame_;
  dds_entity_t participant_{0};
  dds_entity_t publisher_{0};
  dds_entity_t topic_{0};
  dds_entity_t writer_{0};
};

int run(const CliConfig &config) {
  TruthOdometryWriter writer(config.dds_domain, config.parent_frame, config.child_frame);
  ReadyFile readiness(config.ready_file, config.dds_domain, config.session_id);
  readiness.publish();

  adapter::TruthOdometryStreamValidator validator(config.session_id);
  while (true) {
    adapter::TruthOdometryRecord record;
    std::string error;
    const auto status = adapter::read_truth_odometry_record(std::cin, record, error);
    if (status == adapter::TruthOdometryReadStatus::Eof) {
      return 0;
    }
    if (status == adapter::TruthOdometryReadStatus::Error) {
      throw std::runtime_error("truth odometry input rejected: " + error);
    }
    if (!validator.accept(record, error)) {
      throw std::runtime_error("truth odometry input rejected: " + error);
    }
    writer.publish(record);
  }
}

}  // namespace

int main(int argc, const char *const argv[]) {
  CliConfig config;
  try {
    config = parse_arguments(argc, argv);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 2;
  }
  if (config.show_help) {
    std::fputs(kUsage, stdout);
    return 0;
  }

#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
#endif

  try {
    return run(config);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "truth_odom_publisher failed: %s\n", error.what());
    return 1;
  }
}
