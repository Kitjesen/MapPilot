#include "slam.hpp"
#include "message/cpp/dds_topics.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using lingtu::slam::Cloud;
using lingtu::slam::GnssSample;
using lingtu::slam::ISlamBackend;
using lingtu::slam::ImuSample;
using lingtu::slam::LidarFrame;
using lingtu::slam::PointXYZIT;
using lingtu::slam::Pose3d;
using lingtu::slam::SlamConfig;
using lingtu::slam::SlamMode;
using lingtu::slam::SlamOutputs;
using lingtu::slam::Status;
using lingtu::slam::makeContractBackend;
using lingtu::slam::makeFastLioBackend;
using lingtu::slam::makePointLioBackend;
using lingtu::slam::modeFromString;
using lingtu::slam::toString;

double stampSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time toRosTime(double stamp_s) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = rclcpp::Clock().now().seconds();
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_s);
  stamp.nanosec = static_cast<uint32_t>((stamp_s - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}

std::string normalizedBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (backend == "fastlio" || backend == "fastlio2" || backend == "localizer") {
    return "fastlio2";
  }
  if (backend.empty()) {
    return "fastlio2";
  }
  return backend;
}

std::unique_ptr<ISlamBackend> createBackend(const std::string& backend) {
  const std::string normalized = normalizedBackend(backend);
  if (normalized == "fastlio2") {
    return makeFastLioBackend();
  }
  if (normalized == "pointlio") {
    return makePointLioBackend();
  }
  return makeContractBackend(normalized);
}

LidarFrame toLidarFrame(const lingtu::message::LivoxCustomMsg& msg) {
  LidarFrame frame;
  frame.stamp_s =
      msg.timebase > 0 ? static_cast<double>(msg.timebase) * 1e-9 : stampSeconds(msg.header.stamp);
  frame.frame_id = msg.header.frame_id.empty() ? "livox_frame" : msg.header.frame_id;
  frame.points.reserve(msg.points.size());
  for (const auto& src : msg.points) {
    PointXYZIT point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.intensity = static_cast<float>(src.reflectivity);
    point.offset_time_ns = static_cast<std::int64_t>(src.offset_time);
    point.line = src.line;
    point.tag = src.tag;
    frame.points.push_back(point);
  }
  return frame;
}

ImuSample toImuSample(const lingtu::message::Imu& msg) {
  ImuSample sample;
  sample.stamp_s = stampSeconds(msg.header.stamp);
  sample.qx = msg.orientation.x;
  sample.qy = msg.orientation.y;
  sample.qz = msg.orientation.z;
  sample.qw = msg.orientation.w;
  sample.gx = msg.angular_velocity.x;
  sample.gy = msg.angular_velocity.y;
  sample.gz = msg.angular_velocity.z;
  sample.ax = msg.linear_acceleration.x;
  sample.ay = msg.linear_acceleration.y;
  sample.az = msg.linear_acceleration.z;
  return sample;
}

geometry_msgs::msg::Pose toRosPose(const Pose3d& pose) {
  geometry_msgs::msg::Pose out;
  out.position.x = pose.x;
  out.position.y = pose.y;
  out.position.z = pose.z;
  out.orientation.x = pose.qx;
  out.orientation.y = pose.qy;
  out.orientation.z = pose.qz;
  out.orientation.w = pose.qw;
  return out;
}

nav_msgs::msg::Odometry toRosOdom(
    const Pose3d& pose,
    double stamp_s,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  nav_msgs::msg::Odometry out;
  out.header.stamp = toRosTime(stamp_s);
  out.header.frame_id = frame_id;
  out.child_frame_id = child_frame_id;
  out.pose.pose = toRosPose(pose);
  return out;
}

void writeFloat(std::vector<std::uint8_t>& data, std::size_t offset, float value) {
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

sensor_msgs::msg::PointCloud2 toRosCloud(const Cloud& cloud) {
  sensor_msgs::msg::PointCloud2 out;
  out.header.stamp = toRosTime(cloud.stamp_s);
  out.header.frame_id = cloud.frame_id;
  out.height = 1;
  out.width = static_cast<std::uint32_t>(cloud.points.size());
  out.is_bigendian = false;
  out.is_dense = false;
  out.point_step = 16;
  out.row_step = out.point_step * out.width;
  out.fields.resize(4);
  const char* names[] = {"x", "y", "z", "intensity"};
  for (std::size_t i = 0; i < out.fields.size(); ++i) {
    out.fields[i].name = names[i];
    out.fields[i].offset = static_cast<std::uint32_t>(i * 4);
    out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[i].count = 1;
  }
  out.data.resize(static_cast<std::size_t>(out.row_step));
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const std::size_t base = i * out.point_step;
    writeFloat(out.data, base + 0, cloud.points[i].x);
    writeFloat(out.data, base + 4, cloud.points[i].y);
    writeFloat(out.data, base + 8, cloud.points[i].z);
    writeFloat(out.data, base + 12, cloud.points[i].intensity);
  }
  return out;
}

std_msgs::msg::Float32 toQuality(float value) {
  std_msgs::msg::Float32 out;
  out.data = value;
  return out;
}

std_msgs::msg::String toHealthJson(const SlamOutputs& out) {
  std_msgs::msg::String msg;
  msg.data = std::string("{\"state\":\"") + toString(out.state) +
             "\",\"confidence\":" + std::to_string(out.confidence) +
             ",\"backend\":\"cpp_dds_slam\",\"reason\":\"" + out.reason + "\"}";
  return msg;
}

struct CliConfig {
  std::string backend = "fastlio2";
  std::string mode = "mapping";
  std::string map_path;
  std::string config_path;
  double tick_hz = 50.0;
};

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--backend") {
      cfg.backend = next();
    } else if (arg == "--mode") {
      cfg.mode = next();
    } else if (arg == "--map") {
      cfg.map_path = next();
    } else if (arg == "--config") {
      cfg.config_path = next();
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_slam_ros2_dds_runtime [--backend fastlio2] [--mode mapping|localization] "
          "[--map PATH] [--config PATH] [--tick-hz HZ]");
    }
  }
  return cfg;
}

class SlamRos2DdsRuntime final : public rclcpp::Node {
 public:
  explicit SlamRos2DdsRuntime(const CliConfig& cli)
      : Node("lingtu_slam_ros2_dds_runtime"),
        cli_(cli),
        backend_(createBackend(cli.backend)) {
    SlamConfig config;
    config.backend = normalizedBackend(cli.backend);
    config.config_path = cli.config_path;
    Status status = backend_->configure(config);
    if (!status.ok) {
      throw std::runtime_error("SLAM configure failed: " + status.message);
    }
    status = backend_->setMode(modeFromString(cli.mode), cli.map_path);
    if (!status.ok) {
      RCLCPP_WARN(get_logger(), "SLAM mode set returned: %s", status.message.c_str());
    }

    lidar_sub_ = create_subscription<lingtu::message::LivoxCustomMsg>(
        std::string(lingtu::message::kLidarRawFrame.topic),
        rclcpp::SensorDataQoS(),
        [this](lingtu::message::LivoxCustomMsg::SharedPtr msg) {
          const Status s = backend_->feedLidar(toLidarFrame(*msg));
          if (!s.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "feedLidar: %s", s.message.c_str());
          }
        });
    imu_sub_ = create_subscription<lingtu::message::Imu>(
        std::string(lingtu::message::kImuRaw.topic),
        rclcpp::SensorDataQoS(),
        [this](lingtu::message::Imu::SharedPtr msg) {
          const Status s = backend_->feedImu(toImuSample(*msg));
          if (!s.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "feedImu: %s", s.message.c_str());
          }
        });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        std::string(lingtu::message::kSlamOdometry.topic), 10);
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        std::string(lingtu::message::kSlamStateAtScan.topic), 10);
    registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        std::string(lingtu::message::kSlamRegisteredCloud.topic), 5);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        std::string(lingtu::message::kSlamMapCloud.topic), 5);
    quality_pub_ = create_publisher<std_msgs::msg::Float32>(
        std::string(lingtu::message::kSlamLocalizationQuality.topic), 10);
    health_pub_ = create_publisher<std_msgs::msg::String>(
        std::string(lingtu::message::kSlamLocalizationHealth.topic), 10);

    const double hz = std::max(1.0, cli.tick_hz);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / hz),
        [this]() { tick(); });
  }

 private:
  void tick() {
    const Status status = backend_->tick();
    const SlamOutputs out = backend_->outputs();
    if (!status.ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tick: %s", status.message.c_str());
    }
    if (out.odometry_odom_body.has_value()) {
      odom_pub_->publish(toRosOdom(*out.odometry_odom_body, out.stamp_s, "odom", "body"));
    }
    if (out.state_estimation_at_scan.has_value()) {
      state_pub_->publish(toRosOdom(*out.state_estimation_at_scan, out.stamp_s, "odom", "body"));
    }
    if (out.registered_cloud_body.has_value()) {
      registered_pub_->publish(toRosCloud(*out.registered_cloud_body));
    }
    if (out.map_cloud_map.has_value()) {
      map_pub_->publish(toRosCloud(*out.map_cloud_map));
    }
    quality_pub_->publish(toQuality(static_cast<float>(out.localization_quality)));
    health_pub_->publish(toHealthJson(out));
  }

  CliConfig cli_;
  std::unique_ptr<ISlamBackend> backend_;
  rclcpp::Subscription<lingtu::message::LivoxCustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<lingtu::message::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliConfig cli = parseArgs(argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRos2DdsRuntime>(cli));
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
}
