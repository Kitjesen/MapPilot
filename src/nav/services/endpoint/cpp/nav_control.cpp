#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

struct CliConfig {
  std::string command;
  std::string text;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  int domain_id = 0;
};

CliConfig parseArgs(int argc, char** argv) {
  if (argc < 2) {
    throw std::runtime_error(
        "usage: lingtu_nav_control goal X Y [Z YAW] [--domain-id N] | "
        "cancel [REASON] [--domain-id N] | instruction TEXT [--domain-id N]");
  }
  CliConfig cfg;
  cfg.command = argv[1];
  int i = 2;
  if (cfg.command == "goal") {
    if (argc < 4) {
      throw std::runtime_error("goal requires X and Y");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.z = std::stod(argv[i++]);
    }
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.yaw = std::stod(argv[i++]);
    }
  } else if (cfg.command == "cancel") {
    cfg.text = "dds_cancel";
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (cfg.command == "instruction") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("instruction requires text");
    }
    cfg.text = argv[i++];
  } else {
    throw std::runtime_error("unsupported command: " + cfg.command);
  }
  while (i < argc) {
    const std::string arg = argv[i++];
    if (arg == "--domain-id") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --domain-id");
      }
      cfg.domain_id = std::stoi(argv[i++]);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return cfg;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    const dds_entity_t participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
        "dds_create_participant");
    const dds_entity_t publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr),
        "dds_create_publisher");

    if (cfg.command == "goal") {
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_PoseStamped_desc,
              lingtu::message::kNavGoalPose.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(goal_pose)");
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, nullptr, nullptr),
          "dds_create_writer(goal_pose)");
      lingtu_dds_PoseStamped msg{};
      fillHeader(msg.header, nowSeconds(), "map");
      msg.pose.position.x = cfg.x;
      msg.pose.position.y = cfg.y;
      msg.pose.position.z = cfg.z;
      msg.pose.orientation = quaternionFromYaw(cfg.yaw);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      checked(dds_write(writer, &msg), "dds_write(goal_pose)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published goal %.3f %.3f %.3f yaw=%.3f\n", cfg.x, cfg.y, cfg.z, cfg.yaw);
    } else {
      const auto& contract = cfg.command == "cancel"
          ? lingtu::message::kNavCancel
          : lingtu::message::kNavSemanticInstruction;
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_Text_desc,
              contract.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(text)");
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, nullptr, nullptr),
          "dds_create_writer(text)");
      lingtu_dds_Text msg{};
      msg.data = const_cast<char*>(cfg.text.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      checked(dds_write(writer, &msg), "dds_write(text)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published %s: %s\n", cfg.command.c_str(), cfg.text.c_str());
    }
    dds_delete(participant);
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_nav_control failed: %s\n", exc.what());
    return 1;
  }
}
