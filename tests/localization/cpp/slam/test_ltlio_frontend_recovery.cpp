#include "ltlio/frontend.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
namespace lio = lingtu::ltlio;
constexpr double kGravity = 9.80665;
constexpr double kDt = 0.005;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

lio::Config config() {
  lio::Config result;
  result.elevator_enabled = false;
  result.automatic_door = false;
  result.imu_init_samples = 20;
  result.point_filter_num = 1;
  result.blind_m = 0.1;
  result.scan_resolution = 0.1;
  return result;
}

lio::ImuSample sample(int index, double gyro_z = 0.0, double acc_x = 0.0) {
  lio::ImuSample result;
  result.stamp_s = index * kDt;
  result.angular_velocity_rps = {0.0, 0.0, gyro_z};
  result.acceleration = {acc_x, 0.0, kGravity};
  return result;
}

lio::TimedCloud room(double end) {
  lio::TimedCloud cloud;
  cloud.stamp_s = end - 0.02;
  cloud.end_stamp_s = end;
  for (int plane = 0; plane < 3; ++plane)
    for (int a = -6; a <= 6; ++a)
      for (int b = -6; b <= 6; ++b) {
        lio::TimedPoint point;
        if (plane == 0) { point.x = 1.1F; point.y = a * 0.1F; point.z = b * 0.1F; }
        else if (plane == 1) { point.x = a * 0.1F; point.y = 1.4F; point.z = b * 0.1F; }
        else { point.x = a * 0.1F; point.y = b * 0.1F; point.z = -0.8F; }
        cloud.points.push_back(point);
      }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return cloud;
}

lio::Result observe(lio::Frontend& frontend, double end) {
  frontend.feedCloud(room(end));
  const auto result = frontend.processNext();
  require(result.has_value(), "covered scan did not return a state");
  return *result;
}

void checkStartupRecovery() {
  for (bool stationary_checks : {false, true}) {
    for (int spike_index : {0, 10}) {
      auto settings = config();
      if (stationary_checks) {
        settings.imu_static_acc_thresh = 0.30;
        settings.imu_static_gyro_thresh = 0.025;
      }
      lio::Frontend frontend(settings);
      for (int i = 0; i < 60; ++i)
        frontend.feedImu(sample(i, i == spike_index ? 11.0 : 0.0));
      const auto initial = frontend.initialState();
      require(initial.has_value(), "normal IMU did not recover initialization after a gyro spike");
      require(initial->gyro_bias.norm() < 1e-12,
              "startup gyro spike contaminated the initialized bias");
      require(initial->stamp_s >= (spike_index + 20) * kDt - 1e-12,
              "initialization did not collect a complete new window after the gyro discontinuity");
    }
  }
  std::cout << "startup gyro recovery passed with and without stationary thresholds\n";
}

void checkIsolatedSpikePreservesAcceleration() {
  lio::Frontend normal(config()), spiked(config());
  for (int i = 0; i <= 20; ++i) {
    normal.feedImu(sample(i));
    spiked.feedImu(sample(i));
  }
  for (int i = 21; i <= 30; ++i) {
    normal.feedImu(sample(i, 0.0, 1.0));
    spiked.feedImu(sample(i, i == 21 ? 11.0 : 0.0, 1.0));
  }
  const auto expected = observe(normal, 0.15);
  const auto actual = observe(spiked, 0.15);
  require(!actual.attempted && !expected.attempted,
          "gyro isolation fixture unexpectedly applied a LiDAR observation");
  require(actual.state.orientation.angularDistance(expected.state.orientation) < 1e-12,
          "isolated gyro spike entered IMU orientation integration");
  require((actual.world_velocity - expected.world_velocity).norm() < 1e-12 &&
              (actual.world_position - expected.world_position).norm() < 1e-12,
          "gyro rejection replaced valid current acceleration with an older sample");
  std::cout << "isolated gyro rejection retained the acceleration observation\n";
}

void checkRunningReferenceRecovery() {
  lio::Frontend frontend(config());
  for (int i = 0; i <= 20; ++i) frontend.feedImu(sample(i));
  for (int i = 21; i <= 30; ++i) frontend.feedImu(sample(i, 11.0));
  auto result = observe(frontend, 0.15);
  require(std::abs(result.causal_gyro_rps.z() - 11.0) < 1e-12,
          "consistent new raw gyro samples could not replace a stale reference");
  for (int i = 31; i <= 40; ++i) frontend.feedImu(sample(i));
  result = observe(frontend, 0.20);
  require(result.causal_gyro_rps.norm() < 1e-12,
          "normal gyro samples could not recover after a sustained excursion");
  frontend.reset();
  for (int i = 0; i <= 20; ++i) frontend.feedImu(sample(i));
  require(frontend.initialized() && frontend.initialState()->gyro_bias.norm() < 1e-12,
          "reset retained a gyro rejection reference");
  std::cout << "running gyro reference recovered in both directions and across reset\n";
}

void checkThreeDimensionalMotion() {
  for (double vertical_speed : {0.25, -0.25}) {
    auto settings = config();
    settings.imu_static_acc_thresh = 0.22;
    settings.imu_static_gyro_thresh = 0.022;
    lio::Frontend frontend(settings);
    int next_imu = 0;
    lio::Result last;
    for (int frame = 0; frame < 8; ++frame) {
      const double end = 0.22 + frame * 0.05;
      while (next_imu * kDt <= end + kDt)
        frontend.feedImu(sample(next_imu++));
      last = observe(frontend, end);
    }
    require(last.accepted, "3D motion fixture did not establish LiDAR tracking");
    const Eigen::Vector3d origin = last.world_position;
    const Eigen::Vector3d velocity(0.4, 0.0, vertical_speed);
    const double start = last.scan_end_s;
    require(frontend.correctState(start, last.state.orientation, origin, velocity),
            "3D motion fixture could not establish initial velocity");
    double previous_z = origin.z();
    for (int frame = 1; frame <= 40; ++frame) {
      const double end = start + frame * 0.05;
      auto cloud = room(end);
      for (auto& point : cloud.points) {
        const double elapsed = cloud.stamp_s + point.offset_ns * 1e-9 - start;
        point.x -= static_cast<float>(velocity.x() * elapsed);
        point.z -= static_cast<float>(velocity.z() * elapsed);
      }
      while (next_imu * kDt <= end + kDt)
        frontend.feedImu(sample(next_imu++));
      frontend.feedCloud(cloud);
      const auto result = frontend.processNext();
      require(result && result->accepted, "3D motion lost a real LiDAR match");
      last = *result;
      require((last.world_position.z() - previous_z) * vertical_speed > 0.0,
              "accepted height stopped following the ascending or descending motion");
      previous_z = last.world_position.z();
    }
    const Eigen::Vector3d expected = origin + velocity * (last.scan_end_s - start);
    require((last.world_position - expected).norm() < 0.001 &&
                (last.world_velocity - velocity).norm() < 0.005,
            "ordinary motion was biased toward a horizontal or zero-velocity assumption");
    std::cout << "3D motion expected_vz=" << vertical_speed
              << " actual_vz=" << last.world_velocity.z()
              << " position_error=" << (last.world_position - expected).norm() << '\n';
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string selected = argc > 1 ? argv[1] : "";
    if (selected.empty() || selected == "--startup") checkStartupRecovery();
    if (selected.empty() || selected == "--isolated") checkIsolatedSpikePreservesAcceleration();
    if (selected.empty() || selected == "--running") checkRunningReferenceRecovery();
    if (selected.empty() || selected == "--3d-motion") checkThreeDimensionalMotion();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
