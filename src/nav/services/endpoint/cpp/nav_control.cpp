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
#include <vector>

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

dds_qos_t* reliableQos() {
  dds_qos_t* qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  return qos;
}

void waitForMatchedReader(dds_entity_t writer, const char* label, double timeout_s = 3.0) {
  const double deadline = nowSeconds() + timeout_s;
  while (nowSeconds() < deadline) {
    const dds_return_t count = dds_get_matched_subscriptions(writer, nullptr, 0);
    if (count > 0) {
      return;
    }
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_get_matched_subscriptions(") + label + "): " +
          dds_strretcode(-count));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  throw std::runtime_error(std::string("no matched DDS reader for ") + label);
}

struct CliConfig {
  std::string command;
  std::string text;
  std::vector<lingtu_dds_Point> path;
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
        "path X1 Y1 Z1 X2 Y2 Z2 [X Y Z ...] [--domain-id N] | "
        "clear <map|cloud|all> [--domain-id N] | cancel [REASON] [--domain-id N] | "
        "instruction TEXT [--domain-id N]");
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
  } else if (cfg.command == "path") {
    while (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      lingtu_dds_Point point{};
      point.x = std::stod(argv[i++]);
      if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
        throw std::runtime_error("path points must be X Y Z triples");
      }
      point.y = std::stod(argv[i++]);
      if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
        throw std::runtime_error("path points must be X Y Z triples");
      }
      point.z = std::stod(argv[i++]);
      cfg.path.push_back(point);
    }
    if (cfg.path.size() < 2) {
      throw std::runtime_error("path requires at least two X Y Z points");
    }
  } else if (cfg.command == "instruction") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("instruction requires text");
    }
    cfg.text = argv[i++];
  } else if (cfg.command == "clear") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("clear requires map, cloud, or all");
    }
    cfg.text = argv[i++];
    if (cfg.text != "map" && cfg.text != "cloud" && cfg.text != "all") {
      throw std::runtime_error("clear target must be map, cloud, or all");
    }
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
      dds_qos_t* qos = reliableQos();
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos, nullptr),
          "dds_create_writer(goal_pose)");
      dds_delete_qos(qos);
      lingtu_dds_PoseStamped msg{};
      fillHeader(msg.header, nowSeconds(), "map");
      msg.pose.position.x = cfg.x;
      msg.pose.position.y = cfg.y;
      msg.pose.position.z = cfg.z;
      msg.pose.orientation = quaternionFromYaw(cfg.yaw);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "goal_pose");
      checked(dds_write(writer, &msg), "dds_write(goal_pose)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(goal_pose)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published goal %.3f %.3f %.3f yaw=%.3f\n", cfg.x, cfg.y, cfg.z, cfg.yaw);
    } else if (cfg.command == "path") {
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_Path_desc,
              lingtu::message::kNavGlobalPath.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(global_path)");
      dds_qos_t* qos = reliableQos();
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos, nullptr),
          "dds_create_writer(global_path)");
      dds_delete_qos(qos);
      const double stamp_s = nowSeconds();
      std::vector<lingtu_dds_PoseStamped> poses(cfg.path.size());
      for (std::size_t i = 0; i < cfg.path.size(); ++i) {
        fillHeader(poses[i].header, stamp_s, "map");
        poses[i].pose.position = cfg.path[i];
        poses[i].pose.orientation = quaternionFromYaw(0.0);
      }
      lingtu_dds_Path msg{};
      fillHeader(msg.header, stamp_s, "map");
      msg.poses._maximum = static_cast<std::uint32_t>(poses.size());
      msg.poses._length = static_cast<std::uint32_t>(poses.size());
      msg.poses._buffer = poses.data();
      msg.poses._release = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "global_path");
      checked(dds_write(writer, &msg), "dds_write(global_path)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(global_path)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published path %zu waypoints\n", cfg.path.size());
    } else if (cfg.command == "clear") {
      auto publish_clear = [&](const lingtu::message::TopicContract& contract, const char* label) {
        const dds_entity_t topic = checked(
            dds_create_topic(
                participant,
                &lingtu_dds_Bool_desc,
                contract.dds_topic.data(),
                nullptr,
                nullptr),
            (std::string("dds_create_topic(") + label + ")").c_str());
        dds_qos_t* qos = reliableQos();
        const dds_entity_t writer = checked(
            dds_create_writer(publisher, topic, qos, nullptr),
            (std::string("dds_create_writer(") + label + ")").c_str());
        dds_delete_qos(qos);
        lingtu_dds_Bool msg{};
        msg.data = true;
        waitForMatchedReader(writer, label);
        checked(dds_write(writer, &msg), (std::string("dds_write(") + label + ")").c_str());
        checked(
            dds_wait_for_acks(writer, DDS_SECS(2)),
            (std::string("dds_wait_for_acks(") + label + ")").c_str());
      };
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (cfg.text == "map" || cfg.text == "all") {
        publish_clear(lingtu::message::kNavMapClearing, "map_clearing");
      }
      if (cfg.text == "cloud" || cfg.text == "all") {
        publish_clear(lingtu::message::kNavCloudClearing, "cloud_clearing");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published clear %s\n", cfg.text.c_str());
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
      dds_qos_t* qos = reliableQos();
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos, nullptr),
          "dds_create_writer(text)");
      dds_delete_qos(qos);
      lingtu_dds_Text msg{};
      msg.data = const_cast<char*>(cfg.text.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, cfg.command.c_str());
      checked(dds_write(writer, &msg), "dds_write(text)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(text)");
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
