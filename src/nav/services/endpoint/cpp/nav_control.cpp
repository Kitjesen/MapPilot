#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "nav/commands/cpp/client.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

lingtu::dds::UniqueQos qosFor(const lingtu::message::TopicContract& contract) {
  return lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
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
  double cost = 100.0;
  double resolution = 0.1;
  double origin_x = -2.0;
  double origin_y = -2.0;
  double duration_s = 0.0;
  double rate_hz = 20.0;
  int timeout_ms = 3000;
  std::uint32_t width = 50;
  std::uint32_t height = 50;
  int domain_id = 0;
};

CliConfig parseArgs(int argc, char** argv) {
  if (argc < 2) {
    throw std::runtime_error(
        "usage: lingtu_nav_control goal X Y [Z YAW] [--domain-id N] [--timeout-ms N] | "
        "teleop VX VY WZ [--duration-s S] [--rate-hz HZ] [--domain-id N] | "
        "cloud X Y Z [HEIGHT] [--domain-id N] | "
        "trav COST [--domain-id N] | "
        "path X1 Y1 Z1 X2 Y2 Z2 [X Y Z ...] [--domain-id N] | "
        "clear <map|cloud|all> [--domain-id N] | cancel [REASON] [--domain-id N] | "
        "stop [REASON] [--domain-id N] | estop [REASON] [--domain-id N] | "
        "clear-estop [REASON] [--domain-id N] | resume [REASON] [--domain-id N] | "
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
  } else if (cfg.command == "teleop") {
    if (argc < 5) {
      throw std::runtime_error("teleop requires VX VY WZ");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    cfg.yaw = std::stod(argv[i++]);
  } else if (cfg.command == "cloud") {
    if (argc < 5) {
      throw std::runtime_error("cloud requires X Y Z");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    cfg.z = std::stod(argv[i++]);
    cfg.cost = cfg.z;
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.cost = std::stod(argv[i++]);
    }
  } else if (cfg.command == "trav") {
    if (argc < 3) {
      throw std::runtime_error("trav requires COST");
    }
    cfg.cost = std::stod(argv[i++]);
  } else if (cfg.command == "cancel") {
    cfg.text = "dds_cancel";
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (
      cfg.command == "stop" || cfg.command == "estop" ||
      cfg.command == "clear-estop" || cfg.command == "resume") {
    cfg.text = cfg.command;
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
    } else if (arg == "--duration-s") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --duration-s");
      }
      cfg.duration_s = std::stod(argv[i++]);
    } else if (arg == "--rate-hz") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --rate-hz");
      }
      cfg.rate_hz = std::stod(argv[i++]);
    } else if (arg == "--timeout-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --timeout-ms");
      }
      cfg.timeout_ms = std::stoi(argv[i++]);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.duration_s = std::max(0.0, cfg.duration_s);
  cfg.rate_hz = std::max(1.0, cfg.rate_hz);
  cfg.timeout_ms = std::max(1, cfg.timeout_ms);
  return cfg;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    if (
        cfg.command == "goal" || cfg.command == "cancel" ||
        cfg.command == "teleop" || cfg.command == "stop" ||
        cfg.command == "estop" || cfg.command == "clear-estop" ||
        cfg.command == "resume") {
      lingtu::nav::commands::Client client(cfg.domain_id);
      if (cfg.command == "goal") {
        client.navigation().sendGoal(
            cfg.x, cfg.y, cfg.z, cfg.yaw, cfg.timeout_ms);
        std::printf(
            "accepted goal %.3f %.3f %.3f yaw=%.3f\n",
            cfg.x,
            cfg.y,
            cfg.z,
            cfg.yaw);
      } else if (cfg.command == "cancel") {
        client.navigation().cancel(cfg.text, cfg.timeout_ms);
        std::printf("accepted cancel: %s\n", cfg.text.c_str());
      } else if (cfg.command == "teleop") {
        const double deadline = nowSeconds() + cfg.duration_s;
        std::uint64_t count = 0;
        do {
          client.navigation().sendTeleop(
              cfg.x, cfg.y, cfg.yaw, cfg.timeout_ms);
          ++count;
          if (cfg.duration_s <= 0.0) {
            break;
          }
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.rate_hz)));
        } while (nowSeconds() < deadline);
        std::printf(
            "accepted teleop vx=%.3f vy=%.3f wz=%.3f samples=%llu\n",
            cfg.x,
            cfg.y,
            cfg.yaw,
            static_cast<unsigned long long>(count));
      } else if (cfg.command == "stop") {
        client.navigation().stop(cfg.text, cfg.timeout_ms);
        std::printf("accepted stop: %s\n", cfg.text.c_str());
      } else if (cfg.command == "estop") {
        client.navigation().estop(cfg.text, cfg.timeout_ms);
        std::printf("accepted estop: %s\n", cfg.text.c_str());
      } else if (cfg.command == "clear-estop") {
        client.navigation().clearEstop(cfg.text, cfg.timeout_ms);
        std::printf("accepted clear-estop: %s\n", cfg.text.c_str());
      } else {
        client.navigation().resumeAutonomy(cfg.text, cfg.timeout_ms);
        std::printf("accepted resume: %s\n", cfg.text.c_str());
      }
      return 0;
    }
    const dds_entity_t participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
        "dds_create_participant");
    const dds_entity_t publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr),
        "dds_create_publisher");

    if (cfg.command == "cloud") {
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_PointCloud2_desc,
              lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(registered_cloud)");
      auto qos = qosFor(lingtu::message::kSlamRegisteredCloud);
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos.get(), nullptr),
          "dds_create_writer(registered_cloud)");

      lingtu_dds_PointField fields[4]{};
      fields[0].name = const_cast<char*>("x");
      fields[0].offset = 0;
      fields[0].datatype = 7;
      fields[0].count = 1;
      fields[1].name = const_cast<char*>("y");
      fields[1].offset = 4;
      fields[1].datatype = 7;
      fields[1].count = 1;
      fields[2].name = const_cast<char*>("z");
      fields[2].offset = 8;
      fields[2].datatype = 7;
      fields[2].count = 1;
      fields[3].name = const_cast<char*>("height");
      fields[3].offset = 12;
      fields[3].datatype = 7;
      fields[3].count = 1;
      std::vector<std::uint8_t> data(16);
      const float values[4] = {
          static_cast<float>(cfg.x),
          static_cast<float>(cfg.y),
          static_cast<float>(cfg.z),
          static_cast<float>(cfg.cost),
      };
      std::memcpy(data.data(), values, sizeof(values));

      lingtu_dds_PointCloud2 msg{};
      fillHeader(msg.header, nowSeconds(), "body");
      msg.height = 1;
      msg.width = 1;
      msg.fields._maximum = 4;
      msg.fields._length = 4;
      msg.fields._buffer = fields;
      msg.fields._release = false;
      msg.point_step = 16;
      msg.row_step = 16;
      msg.data._maximum = static_cast<std::uint32_t>(data.size());
      msg.data._length = static_cast<std::uint32_t>(data.size());
      msg.data._buffer = data.data();
      msg.data._release = false;
      msg.is_dense = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "registered_cloud");
      checked(dds_write(writer, &msg), "dds_write(registered_cloud)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(registered_cloud)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf(
          "published cloud point x=%.3f y=%.3f z=%.3f height=%.3f\n",
          cfg.x,
          cfg.y,
          cfg.z,
          cfg.cost);
    } else if (cfg.command == "trav") {
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_OccupancyGrid_desc,
              lingtu::message::kNavTraversability.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(traversability)");
      auto qos = qosFor(lingtu::message::kNavTraversability);
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos.get(), nullptr),
          "dds_create_writer(traversability)");

      std::vector<std::uint8_t> data(
          static_cast<std::size_t>(cfg.width) * static_cast<std::size_t>(cfg.height),
          static_cast<std::uint8_t>(std::clamp(cfg.cost, 0.0, 100.0)));
      lingtu_dds_OccupancyGrid msg{};
      fillHeader(msg.header, nowSeconds(), "map");
      msg.info.resolution = static_cast<float>(cfg.resolution);
      msg.info.width = cfg.width;
      msg.info.height = cfg.height;
      msg.info.origin.position.x = cfg.origin_x;
      msg.info.origin.position.y = cfg.origin_y;
      msg.info.origin.orientation.w = 1.0;
      msg.data._maximum = static_cast<std::uint32_t>(data.size());
      msg.data._length = static_cast<std::uint32_t>(data.size());
      msg.data._buffer = data.data();
      msg.data._release = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "traversability");
      checked(dds_write(writer, &msg), "dds_write(traversability)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(traversability)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published traversability cost=%.1f cells=%ux%u\n",
                  cfg.cost,
                  cfg.width,
                  cfg.height);
    } else if (cfg.command == "path") {
      std::fprintf(
          stderr,
          "nav_control: legacy path injection requires endpoint "
          "--allow-legacy-motion-inputs true; product control uses typed Goal\n");
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_Path_desc,
              lingtu::message::kNavGlobalPath.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(global_path)");
      auto qos = qosFor(lingtu::message::kNavGlobalPath);
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos.get(), nullptr),
          "dds_create_writer(global_path)");
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
        auto qos = qosFor(contract);
        const dds_entity_t writer = checked(
            dds_create_writer(publisher, topic, qos.get(), nullptr),
            (std::string("dds_create_writer(") + label + ")").c_str());
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
      const auto& contract = lingtu::message::kNavSemanticInstruction;
      const dds_entity_t topic = checked(
          dds_create_topic(
              participant,
              &lingtu_dds_Text_desc,
              contract.dds_topic.data(),
              nullptr,
              nullptr),
          "dds_create_topic(text)");
      auto qos = qosFor(contract);
      const dds_entity_t writer = checked(
          dds_create_writer(publisher, topic, qos.get(), nullptr),
          "dds_create_writer(text)");
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
