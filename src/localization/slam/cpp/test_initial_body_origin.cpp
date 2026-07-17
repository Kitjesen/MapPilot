#include "slam.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace lingtu::slam;

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ImuSample stationaryImu(double stamp_s) {
  ImuSample sample;
  sample.stamp_s = stamp_s;
  sample.qw = 1.0;
  sample.az = 1.0;
  return sample;
}

LidarFrame structuredScan(double stamp_s, float dx) {
  LidarFrame frame;
  frame.stamp_s = stamp_s;
  frame.frame_id = "lidar";
  for (int i = 0; i < 240; ++i) {
    const int sample = i / 3;
    const float a = static_cast<float>((sample % 10) - 5) * 0.20F;
    const float b = static_cast<float>((sample / 10) - 4) * 0.20F;
    PointXYZIT point;
    if (i % 3 == 0) {
      point.x = 3.0F + dx;
      point.y = a;
      point.z = b;
    } else if (i % 3 == 1) {
      point.x = 2.0F + dx + a;
      point.y = 2.0F;
      point.z = b;
    } else {
      point.x = 2.0F + dx + a;
      point.y = b;
      point.z = 1.5F;
    }
    point.offset_time_ns = static_cast<std::int64_t>((i % 10) * 5'000'000);
    frame.points.push_back(point);
  }
  return frame;
}

double yawFromPose(const Pose3d& pose) {
  return std::atan2(
      2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
      1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz));
}

}  // namespace

int main() {
  const auto config_path =
      std::filesystem::temp_directory_path() / "lingtu_initial_body_origin.yaml";
  {
    std::ofstream config(config_path);
    config << "reject_nonconverged_update: false\n";
    config << "reject_degenerate_nonconverged_update: true\n";
    config << "max_update_translation_m: 10.0\n";
    config << "max_update_rotation_rad: 3.14\n";
    config << "max_update_velocity_delta_mps: 100.0\n";
    config << "navigation_body_from_imu_translation: [0.38, -0.02, 0.10]\n";
    config << "navigation_body_from_imu_rotation: "
              "[0.707106781186548, 0.0, 0.707106781186548, "
              "0.0, -1.0, 0.0, "
              "0.707106781186548, 0.0, -0.707106781186548]\n";
  }

  auto backend = makeFastLioBackend();
  require(backend != nullptr, "fastlio backend missing");
  SlamConfig slam_config;
  slam_config.backend = "fastlio2";
  slam_config.config_path = config_path.string();
  require(backend->configure(slam_config).ok, "fastlio configure failed");
  require(backend->setMode(SlamMode::Mapping, "").ok, "mapping mode failed");

  for (int i = 0; i <= 60; ++i) {
    require(
        backend->feedImu(stationaryImu(static_cast<double>(i) * 0.01)).ok,
        "stationary IMU feed failed");
  }
  require(backend->feedLidar(structuredScan(0.20, 0.0F)).ok, "first scan failed");
  require(backend->feedLidar(structuredScan(0.30, 0.02F)).ok, "second scan failed");
  for (int i = 0; i < 4; ++i) {
    require(backend->tick().ok, "initialization tick failed");
  }

  const SlamOutputs outputs = backend->outputs();
  require(outputs.odometry_odom_body.has_value(), "initial body odometry missing");
  const Pose3d& pose = *outputs.odometry_odom_body;
  const double position_norm =
      std::sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
  std::cout << "initial body pose: x=" << pose.x << " y=" << pose.y
            << " z=" << pose.z << " yaw=" << yawFromPose(pose) << '\n';
  require(position_norm < 1e-4, "initial base_link position is not the odom origin");
  require(std::abs(yawFromPose(pose)) < 1e-4, "initial base_link yaw is not zero");
  return 0;
}
