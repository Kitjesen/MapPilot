#include "slam.hpp"
#include "message/cpp/dds_topics.hpp"

#include "dds/dds.hpp"
#include "lingtu_slam.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace lt = lingtu::dds;

using lingtu::slam::Cloud;
using lingtu::slam::ISlamBackend;
using lingtu::slam::ImuSample;
using lingtu::slam::LidarFrame;
using lingtu::slam::PointXYZIT;
using lingtu::slam::Pose3d;
using lingtu::slam::SlamConfig;
using lingtu::slam::SlamOutputs;
using lingtu::slam::Status;
using lingtu::slam::makeContractBackend;
using lingtu::slam::makeFastLioBackend;
using lingtu::slam::makePointLioBackend;
using lingtu::slam::modeFromString;
using lingtu::slam::toString;

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double stampSeconds(const lt::Time& stamp) {
  return static_cast<double>(stamp.sec()) + static_cast<double>(stamp.nanosec()) * 1e-9;
}

lt::Time toDdsTime(double stamp_s) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  const auto sec = static_cast<std::int32_t>(stamp_s);
  const auto nsec = static_cast<std::uint32_t>((stamp_s - static_cast<double>(sec)) * 1e9);
  return lt::Time(sec, nsec);
}

lt::Header makeHeader(double stamp_s, const std::string& frame_id) {
  return lt::Header(toDdsTime(stamp_s), frame_id);
}

std::string normalizedBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (backend.empty() || backend == "fastlio" || backend == "fastlio2" || backend == "localizer") {
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

LidarFrame toLidarFrame(const lt::LivoxFrame& msg) {
  LidarFrame frame;
  frame.stamp_s = msg.timebase() > 0
      ? static_cast<double>(msg.timebase()) * 1e-9
      : stampSeconds(msg.header().stamp());
  frame.frame_id = msg.header().frame_id().empty() ? "livox_frame" : msg.header().frame_id();
  frame.points.reserve(msg.points().size());
  for (const auto& src : msg.points()) {
    PointXYZIT point;
    point.x = src.x();
    point.y = src.y();
    point.z = src.z();
    point.intensity = static_cast<float>(src.reflectivity());
    point.offset_time_ns = static_cast<std::int64_t>(src.offset_time());
    point.line = src.line();
    point.tag = src.tag();
    frame.points.push_back(point);
  }
  return frame;
}

ImuSample toImuSample(const lt::Imu& msg) {
  ImuSample sample;
  sample.stamp_s = stampSeconds(msg.header().stamp());
  sample.qx = msg.orientation().x();
  sample.qy = msg.orientation().y();
  sample.qz = msg.orientation().z();
  sample.qw = msg.orientation().w();
  sample.gx = msg.angular_velocity().x();
  sample.gy = msg.angular_velocity().y();
  sample.gz = msg.angular_velocity().z();
  sample.ax = msg.linear_acceleration().x();
  sample.ay = msg.linear_acceleration().y();
  sample.az = msg.linear_acceleration().z();
  return sample;
}

lt::Pose toDdsPose(const Pose3d& pose) {
  return lt::Pose(
      lt::Point(pose.x, pose.y, pose.z),
      lt::Quaternion(pose.qx, pose.qy, pose.qz, pose.qw));
}

template <std::size_t N>
std::array<double, N> zeros() {
  std::array<double, N> values{};
  values.fill(0.0);
  return values;
}

lt::Odometry toDdsOdom(
    const Pose3d& pose,
    double stamp_s,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  return lt::Odometry(
      makeHeader(stamp_s, frame_id),
      child_frame_id,
      lt::PoseWithCovariance(toDdsPose(pose), zeros<36>()),
      lt::TwistWithCovariance(
          lt::Twist(lt::Vector3(0.0, 0.0, 0.0), lt::Vector3(0.0, 0.0, 0.0)),
          zeros<36>()));
}

void writeFloat(std::vector<std::uint8_t>& data, std::size_t offset, float value) {
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

lt::PointCloud2 toDdsCloud(const Cloud& cloud) {
  std::vector<lt::PointField> fields{
      lt::PointField("x", 0, 7, 1),
      lt::PointField("y", 4, 7, 1),
      lt::PointField("z", 8, 7, 1),
      lt::PointField("intensity", 12, 7, 1),
  };
  constexpr std::uint32_t point_step = 16;
  const auto width = static_cast<std::uint32_t>(cloud.points.size());
  const auto row_step = point_step * width;
  std::vector<std::uint8_t> data(static_cast<std::size_t>(row_step));
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const std::size_t base = i * point_step;
    writeFloat(data, base + 0, cloud.points[i].x);
    writeFloat(data, base + 4, cloud.points[i].y);
    writeFloat(data, base + 8, cloud.points[i].z);
    writeFloat(data, base + 12, cloud.points[i].intensity);
  }
  return lt::PointCloud2(
      makeHeader(cloud.stamp_s, cloud.frame_id),
      1,
      width,
      fields,
      false,
      point_step,
      row_step,
      data,
      false);
}

lt::Text toHealthJson(const SlamOutputs& out) {
  return lt::Text(
      std::string("{\"state\":\"") + toString(out.state) +
      "\",\"confidence\":" + std::to_string(out.confidence) +
      ",\"backend\":\"cpp_cyclone_slam\",\"reason\":\"" + out.reason + "\"}");
}

struct CliConfig {
  std::string backend = "fastlio2";
  std::string mode = "mapping";
  std::string map_path;
  std::string config_path;
  int domain_id = 0;
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
    } else if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_slam_cyclone_runtime [--backend fastlio2] "
          "[--mode mapping|localization] [--map PATH] [--config PATH] "
          "[--domain-id N] [--tick-hz HZ]");
    }
  }
  return cfg;
}

template <typename Reader, typename Handler>
void drainReader(Reader& reader, Handler&& handler) {
  auto samples = reader.take();
  for (auto it = samples.begin(); it < samples.end(); ++it) {
    if (it->info().valid()) {
      handler(it->data());
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cli = parseArgs(argc, argv);
    auto backend = createBackend(cli.backend);
    SlamConfig config;
    config.backend = normalizedBackend(cli.backend);
    config.config_path = cli.config_path;
    Status status = backend->configure(config);
    if (!status.ok) {
      throw std::runtime_error("SLAM configure failed: " + status.message);
    }
    status = backend->setMode(modeFromString(cli.mode), cli.map_path);
    if (!status.ok) {
      std::fprintf(stderr, "SLAM mode set returned: %s\n", status.message.c_str());
    }

    dds::domain::DomainParticipant participant(cli.domain_id);
    dds::sub::Subscriber subscriber(participant);
    dds::pub::Publisher publisher(participant);

    dds::topic::Topic<lt::LivoxFrame> lidar_topic(
        participant, std::string(lingtu::message::kLidarRawFrame.dds_topic));
    dds::topic::Topic<lt::Imu> imu_topic(
        participant, std::string(lingtu::message::kImuRaw.dds_topic));
    dds::topic::Topic<lt::Odometry> odom_topic(
        participant, std::string(lingtu::message::kSlamOdometry.dds_topic));
    dds::topic::Topic<lt::Odometry> state_topic(
        participant, std::string(lingtu::message::kSlamStateAtScan.dds_topic));
    dds::topic::Topic<lt::PointCloud2> registered_topic(
        participant, std::string(lingtu::message::kSlamRegisteredCloud.dds_topic));
    dds::topic::Topic<lt::PointCloud2> map_topic(
        participant, std::string(lingtu::message::kSlamMapCloud.dds_topic));
    dds::topic::Topic<lt::Float32> quality_topic(
        participant, std::string(lingtu::message::kSlamLocalizationQuality.dds_topic));
    dds::topic::Topic<lt::Text> health_topic(
        participant, std::string(lingtu::message::kSlamLocalizationHealth.dds_topic));

    dds::sub::DataReader<lt::LivoxFrame> lidar_reader(subscriber, lidar_topic);
    dds::sub::DataReader<lt::Imu> imu_reader(subscriber, imu_topic);
    dds::pub::DataWriter<lt::Odometry> odom_writer(publisher, odom_topic);
    dds::pub::DataWriter<lt::Odometry> state_writer(publisher, state_topic);
    dds::pub::DataWriter<lt::PointCloud2> registered_writer(publisher, registered_topic);
    dds::pub::DataWriter<lt::PointCloud2> map_writer(publisher, map_topic);
    dds::pub::DataWriter<lt::Float32> quality_writer(publisher, quality_topic);
    dds::pub::DataWriter<lt::Text> health_writer(publisher, health_topic);

    const double hz = std::max(1.0, cli.tick_hz);
    const auto period = std::chrono::duration<double>(1.0 / hz);
    while (g_running) {
      drainReader(imu_reader, [&](const lt::Imu& msg) {
        const Status s = backend->feedImu(toImuSample(msg));
        if (!s.ok) {
          std::fprintf(stderr, "feedImu: %s\n", s.message.c_str());
        }
      });
      drainReader(lidar_reader, [&](const lt::LivoxFrame& msg) {
        const Status s = backend->feedLidar(toLidarFrame(msg));
        if (!s.ok) {
          std::fprintf(stderr, "feedLidar: %s\n", s.message.c_str());
        }
      });

      status = backend->tick();
      if (!status.ok) {
        std::fprintf(stderr, "tick: %s\n", status.message.c_str());
      }
      const SlamOutputs out = backend->outputs();
      if (out.odometry_odom_body.has_value()) {
        odom_writer.write(toDdsOdom(*out.odometry_odom_body, out.stamp_s, "odom", "body"));
      }
      if (out.state_estimation_at_scan.has_value()) {
        state_writer.write(toDdsOdom(*out.state_estimation_at_scan, out.stamp_s, "odom", "body"));
      }
      if (out.registered_cloud_body.has_value()) {
        registered_writer.write(toDdsCloud(*out.registered_cloud_body));
      }
      if (out.map_cloud_map.has_value()) {
        map_writer.write(toDdsCloud(*out.map_cloud_map));
      }
      quality_writer.write(lt::Float32(static_cast<float>(out.localization_quality)));
      health_writer.write(toHealthJson(out));
      std::this_thread::sleep_for(period);
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
}
