#include "slam.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace lingtu::slam;

namespace {

ImuSample imu(double stamp_s) {
  ImuSample sample;
  sample.stamp_s = stamp_s;
  sample.qw = 1.0;
  sample.az = 1.0;
  return sample;
}

LidarFrame lidar(double stamp_s, float dx) {
  LidarFrame frame;
  frame.stamp_s = stamp_s;
  frame.frame_id = "lidar";
  for (int i = 0; i < 240; ++i) {
    const float y = static_cast<float>((i % 20) - 10) * 0.12F;
    const float z = static_cast<float>((i / 20) - 6) * 0.08F;
    PointXYZIT point;
    point.x = 3.0F + dx + static_cast<float>(i % 5) * 0.03F;
    point.y = y;
    point.z = z;
    point.intensity = 20.0F;
    point.offset_time_ns = static_cast<std::int64_t>((i % 10) * 5'000'000);
    point.line = static_cast<std::uint8_t>(i % 4);
    point.tag = 0;
    frame.points.push_back(point);
  }
  return frame;
}

void check(bool ok, const char* message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  auto backend = makeFastLioBackend();
  check(backend != nullptr, "missing_backend");

  SlamConfig config;
  config.backend = "fastlio2";
  check(backend->configure(config).ok, "configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "set_mode_failed");

  for (int i = 0; i <= 60; ++i) {
    check(backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok, "feed_imu_failed");
  }
  check(backend->feedLidar(lidar(0.20, 0.0F)).ok, "feed_lidar_1_failed");
  check(backend->feedLidar(lidar(0.30, 0.02F)).ok, "feed_lidar_2_failed");

  for (int i = 0; i < 4; ++i) {
    check(backend->tick().ok, "tick_failed");
  }

  const auto outputs = backend->outputs();
  if (!outputs.odometry_odom_body.has_value()) {
    std::cerr << "missing_odometry state=" << toString(outputs.state)
              << " reason=" << outputs.reason
              << " imu_batch=" << outputs.imu_batch
              << " waits=" << outputs.sync_wait_count << "\n";
    return 1;
  }
  if (!outputs.registered_cloud_body.has_value() || !outputs.map_cloud_map.has_value()) {
    std::cerr << "missing_cloud_outputs\n";
    return 1;
  }

  std::cout << "fastlio2_mock_flow odometry stamp=" << outputs.stamp_s
            << " points=" << outputs.registered_cloud_body->points.size() << "\n";
  return 0;
}
