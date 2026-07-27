#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header &header, double stamp_s, const char *frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char *>(frame_id);
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

lingtu::dds::UniqueQos qosFor(const lingtu::message::TopicContract &contract) {
  return lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
}

void logDdsError(dds_return_t value, const char *what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

template <typename T, typename Handler>
void drainReader(dds_entity_t reader, const dds_topic_descriptor_t &descriptor, Handler &&handler) {
  constexpr std::size_t kMaxSamples = 16;
  void *samples[kMaxSamples];
  dds_sample_info_t infos[kMaxSamples];
  for (auto &sample : samples) {
    sample = dds_alloc(sizeof(T));
    std::memset(sample, 0, sizeof(T));
  }
  const dds_return_t count = dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count >= 0) {
    for (dds_return_t i = 0; i < count; ++i) {
      if (infos[i].valid_data) {
        handler(*static_cast<T *>(samples[i]));
      }
    }
  } else {
    logDdsError(count, "dds_take");
  }
  for (auto &sample : samples) {
    dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
  }
}

struct CliConfig {
  int domain_id{0};
  double publish_hz{50.0};
  double command_timeout_s{0.5};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double max_linear{1.0};
  double max_angular{1.5};
  std::string status_file;
};

std::string envOrEmpty(const char *name) {
  const char *value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

CliConfig parseArgs(int argc, char **argv) {
  CliConfig cfg;
  cfg.status_file = envOrEmpty("LINGTU_MOTION_MOCK_STATUS_FILE");
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--publish-hz") {
      cfg.publish_hz = std::stod(next());
    } else if (arg == "--command-timeout-s") {
      cfg.command_timeout_s = std::stod(next());
    } else if (arg == "--x") {
      cfg.x = std::stod(next());
    } else if (arg == "--y") {
      cfg.y = std::stod(next());
    } else if (arg == "--z") {
      cfg.z = std::stod(next());
    } else if (arg == "--yaw") {
      cfg.yaw = std::stod(next());
    } else if (arg == "--max-linear") {
      cfg.max_linear = std::stod(next());
    } else if (arg == "--max-angular") {
      cfg.max_angular = std::stod(next());
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_motion_mock_dds [--domain-id N] [--publish-hz HZ] "
          "[--x X] [--y Y] [--z Z] [--yaw RAD] [--status-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.publish_hz = std::max(1.0, cfg.publish_hz);
  cfg.command_timeout_s = std::max(0.0, cfg.command_timeout_s);
  cfg.max_linear = std::max(0.0, cfg.max_linear);
  cfg.max_angular = std::max(0.0, cfg.max_angular);
  return cfg;
}

struct TwistCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double stamp_s{-1.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct TfMessage {
  lingtu_dds_TFMessage msg{};
  lingtu_dds_TransformStamped transform{};
};

lingtu_dds_Odometry toOdometry(const Pose2D &pose, const TwistCommand &cmd, double stamp_s) {
  lingtu_dds_Odometry out{};
  fillHeader(out.header, stamp_s, "odom");
  out.child_frame_id = const_cast<char *>("body");
  out.pose.pose.position.x = pose.x;
  out.pose.pose.position.y = pose.y;
  out.pose.pose.position.z = pose.z;
  out.pose.pose.orientation = quaternionFromYaw(pose.yaw);
  out.twist.twist.linear.x = cmd.vx;
  out.twist.twist.linear.y = cmd.vy;
  out.twist.twist.angular.z = cmd.wz;
  return out;
}

TfMessage toMapOdomTf(double stamp_s) {
  TfMessage out;
  fillHeader(out.transform.header, stamp_s, "map");
  out.transform.child_frame_id = const_cast<char *>("odom");
  out.transform.transform.rotation.w = 1.0;
  out.msg.transforms._maximum = 1;
  out.msg.transforms._length = 1;
  out.msg.transforms._buffer = &out.transform;
  out.msg.transforms._release = false;
  return out;
}

void writeStatus(const CliConfig &cfg, const Pose2D &pose, const TwistCommand &cmd,
                 std::uint64_t cmd_count, std::uint64_t odom_count, bool active_cmd) {
  if (cfg.status_file.empty()) {
    return;
  }
  const std::filesystem::path path(cfg.status_file);
  if (!path.parent_path().empty()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
  }
  const std::filesystem::path tmp = path.string() + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    return;
  }
  out << "{\n"
      << "  \"schema_version\": \"lingtu.motion_mock.status.v1\",\n"
      << "  \"endpoint\": \"lingtu_motion_mock_dds\",\n"
      << "  \"stamp_s\": " << nowSeconds() << ",\n"
      << "  \"domain_id\": " << cfg.domain_id << ",\n"
      << "  \"publish_hz\": " << cfg.publish_hz << ",\n"
      << "  \"active_cmd\": " << (active_cmd ? "true" : "false") << ",\n"
      << "  \"pose\": {\"x\": " << pose.x << ", \"y\": " << pose.y << ", \"z\": " << pose.z
      << ", \"yaw\": " << pose.yaw << "},\n"
      << "  \"cmd_vel\": {\"vx\": " << cmd.vx << ", \"vy\": " << cmd.vy << ", \"wz\": " << cmd.wz
      << "},\n"
      << "  \"counters\": {\"cmd_vel\": " << cmd_count << ", \"odometry\": " << odom_count << "}\n"
      << "}\n";
  out.close();
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
}

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant");
    subscriber_ =
        checked(dds_create_subscriber(participant_, nullptr, nullptr), "dds_create_subscriber");
    publisher_ =
        checked(dds_create_publisher(participant_, nullptr, nullptr), "dds_create_publisher");
    cmd_vel_reader_ =
        reader(lingtu::message::kNavCmdVel, &lingtu_dds_FinalVelocityCommand_desc, "cmd_vel");
    odom_writer_ = writer(lingtu::message::kSlamOdometry, &lingtu_dds_Odometry_desc, "odom");
    tf_writer_ = writer(lingtu::message::kTf, &lingtu_dds_TFMessage_desc, "tf");
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  template <typename Handler>
  void drainCmdVel(Handler &&handler) {
    drainReader<lingtu_dds_FinalVelocityCommand>(
        cmd_vel_reader_, lingtu_dds_FinalVelocityCommand_desc, std::forward<Handler>(handler));
  }

  void writeOdom(const lingtu_dds_Odometry &msg) {
    logDdsError(dds_write(odom_writer_, &msg), "dds_write(odom)");
  }

  void writeTf(const lingtu_dds_TFMessage &msg) {
    logDdsError(dds_write(tf_writer_, &msg), "dds_write(tf)");
  }

 private:
  dds_entity_t reader(const lingtu::message::TopicContract &contract,
                      const dds_topic_descriptor_t *desc, const char *label) {
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, desc, contract.dds_topic.data(), nullptr, nullptr),
                (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = qosFor(contract);
    return checked(dds_create_reader(subscriber_, topic, qos.get(), nullptr),
                   (std::string("dds_create_reader(") + label + ")").c_str());
  }

  dds_entity_t writer(const lingtu::message::TopicContract &contract,
                      const dds_topic_descriptor_t *desc, const char *label) {
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, desc, contract.dds_topic.data(), nullptr, nullptr),
                (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = qosFor(contract);
    return checked(dds_create_writer(publisher_, topic, qos.get(), nullptr),
                   (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t cmd_vel_reader_{0};
  dds_entity_t odom_writer_{0};
  dds_entity_t tf_writer_{0};
};

}  // namespace

int main(int argc, char **argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cfg = parseArgs(argc, argv);
    DdsRuntime dds(cfg.domain_id);
    Pose2D pose{cfg.x, cfg.y, cfg.z, cfg.yaw};
    TwistCommand cmd;
    std::uint64_t cmd_count = 0;
    std::uint64_t odom_count = 0;
    double last_s = nowSeconds();
    double next_status_s = last_s + 1.0;

    std::fprintf(stderr,
                 "lingtu_motion_mock_dds: domain=%d publish_hz=%.2f pose=(%.3f %.3f %.3f %.3f)\n",
                 cfg.domain_id, cfg.publish_hz, pose.x, pose.y, pose.z, pose.yaw);

    while (g_running) {
      dds.drainCmdVel([&](const lingtu_dds_FinalVelocityCommand &msg) {
        cmd.vx = std::clamp(msg.twist.linear.x, -cfg.max_linear, cfg.max_linear);
        cmd.vy = std::clamp(msg.twist.linear.y, -cfg.max_linear, cfg.max_linear);
        cmd.wz = std::clamp(msg.twist.angular.z, -cfg.max_angular, cfg.max_angular);
        cmd.stamp_s = nowSeconds();
        ++cmd_count;
      });

      const double now = nowSeconds();
      const double dt = std::clamp(now - last_s, 0.0, 0.1);
      last_s = now;
      const bool active_cmd = cmd.stamp_s > 0.0 && (cfg.command_timeout_s <= 0.0 ||
                                                    now - cmd.stamp_s <= cfg.command_timeout_s);
      if (active_cmd) {
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);
        pose.x += (cmd.vx * c - cmd.vy * s) * dt;
        pose.y += (cmd.vx * s + cmd.vy * c) * dt;
        pose.yaw += cmd.wz * dt;
      }

      const auto odom = toOdometry(pose, active_cmd ? cmd : TwistCommand{}, now);
      const auto tf = toMapOdomTf(now);
      dds.writeOdom(odom);
      dds.writeTf(tf.msg);
      ++odom_count;

      if (now >= next_status_s) {
        next_status_s = now + 1.0;
        writeStatus(cfg, pose, cmd, cmd_count, odom_count, active_cmd);
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.publish_hz)));
    }
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "lingtu_motion_mock_dds failed: %s\n", exc.what());
    return 1;
  }
}
