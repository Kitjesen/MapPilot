#include "ltlio/frontend.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
namespace lio = lingtu::ltlio;
constexpr double gravity = 9.80665;
constexpr double dt = 0.005;

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

lio::Config settings() {
  lio::Config value;
  value.elevator_enabled = false;
  value.imu_init_samples = 200;
  value.point_filter_num = 1;
  value.blind_m = 0.1;
  value.scan_resolution = 0.1;
  value.imu_static_acc_thresh = 0.22;
  value.imu_static_gyro_thresh = 0.022;
  value.zupt_min_static_frames = 6;
  value.zupt_sigma_v = 0.015;
  return value;
}

lio::ImuSample imu(double stamp, const Eigen::Vector3d& acc,
                   const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
  lio::ImuSample value;
  value.stamp_s = stamp;
  value.acceleration = acc;
  value.angular_velocity_rps = gyro;
  return value;
}

lio::TimedCloud scan(double stamp) {
  lio::TimedCloud cloud;
  cloud.stamp_s = stamp;
  cloud.end_stamp_s = stamp + 0.02;
  for (int surface = 0; surface < 3; ++surface)
    for (int a = -8; a <= 8; ++a)
      for (int b = -8; b <= 8; ++b) {
        lio::TimedPoint p;
        if (surface == 0) { p.x = 3; p.y = a * 0.1F; p.z = b * 0.1F; }
        else if (surface == 1) { p.x = a * 0.1F; p.y = 4; p.z = b * 0.1F; }
        else { p.x = a * 0.1F; p.y = b * 0.1F; p.z = -2; }
        cloud.points.push_back(p);
      }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return cloud;
}

void checkStartupImpactAndStationaryRetry() {
  lio::Frontend frontend(settings());
  for (int i = 0; i < 200; ++i)
    frontend.feedImu(imu(i * dt, {0.0, 0.0, i < 8 ? 60.0 : gravity}));
  require(!frontend.initialized() && !frontend.initialState(),
          "MuJoCo startup contact calibrated the IMU before settling");
  for (int i = 200; i < 400; ++i)
    frontend.feedImu(imu(i * dt, {0.0, 0.0, gravity}));
  const auto initial = frontend.initialState();
  require(frontend.initialized() && initial,
          "a stable window did not recover from the rejected impact window");
  require(std::abs(initial->gravity.norm() - gravity) < 1e-10 &&
              initial->velocity.norm() < 1e-12,
          "startup impact contaminated the replacement stable initialization");

  int next_imu = 400;
  lio::Result last;
  for (int frame = 0; frame < 100; ++frame) {
    const auto cloud = scan(2.10 + frame * 0.025);
    while (next_imu * dt <= cloud.end_stamp_s + dt)
      frontend.feedImu(imu(next_imu++ * dt, {0.0, 0.0, gravity}));
    frontend.feedCloud(cloud);
    const auto result = frontend.processNext();
    require(result.has_value(), "stationary native frontend stalled");
    last = *result;
    require(last.state.covariance.allFinite() &&
                last.state.covariance.llt().info() == Eigen::Success,
            "stationary constraints destroyed positive-definite covariance");
  }
  require(last.accepted && last.world_position.norm() < 1e-5 &&
              last.world_velocity.norm() < 1e-5,
          "stationary real LiDAR/IMU accumulated position or velocity drift");
}

void checkVaryingWindowIsRejected() {
  for (const bool gyro_motion : {false, true}) {
    lio::Frontend frontend(settings());
    for (int i = 0; i < 200; ++i) {
      const double sign = i % 2 == 0 ? 1.0 : -1.0;
      frontend.feedImu(imu(i * dt,
          {gyro_motion ? 0.0 : sign * 0.3, 0.0, gravity},
          {gyro_motion ? sign * 0.03 : 0.0, 0.0, 0.0}));
    }
    require(!frontend.initialized(),
            "moving IMU window passed its configured stationary RMS threshold");
    for (int i = 200; i < 400; ++i)
      frontend.feedImu(imu(i * dt, {0.0, 0.0, gravity}));
    require(frontend.initialized(), "stationary retry retained the rejected motion samples");
  }
}

void checkExplicitSensorUnits() {
  auto si_config = settings();
  auto g_config = si_config;
  g_config.acc_scale = gravity;
  lio::Frontend si(si_config), raw_g(g_config);
  const Eigen::Vector3d measured(0.02, -0.01, 10.005);
  const Eigen::Vector3d bias(0.002, -0.001, 0.003);
  // Check units before voxel/nearest-neighbor choices: a 0.1 m regular room
  // grid lies on voxel boundaries, so sub-ulp IMU rounding can change selected
  // centroids without indicating an acceleration-unit error.
  for (int index = 0; index <= 200; ++index) {
    si.feedImu(imu(index * dt, measured, bias));
    raw_g.feedImu(imu(index * dt, measured / gravity, bias));
  }
  const auto si_initial = si.initialState();
  const auto g_initial = raw_g.initialState();
  require(si_initial && g_initial, "unit fixtures did not initialize");
  for (const auto& initial : {*si_initial, *g_initial}) {
    require(std::abs(initial.gravity.norm() - measured.norm()) < 1e-10 &&
                (initial.orientation * measured + initial.gravity).norm() < 1e-10 &&
                (initial.gyro_bias - bias).norm() < 1e-12,
            "input units changed initialized gravity, tilt or gyro bias");
  }
  constexpr double acceleration = 0.4;
  const Eigen::Vector3d body_acceleration =
      si_initial->orientation.conjugate() * Eigen::Vector3d(acceleration, 0.0, 0.0);
  int next_imu = 201;
  for (int frame = 0; frame < 4; ++frame) {
    const double end = 1.25 + frame * 0.25;
    const auto cloud = scan(end - 0.02);
    while (next_imu * dt <= cloud.end_stamp_s + dt) {
      const double stamp = next_imu++ * dt;
      si.feedImu(imu(stamp, measured + body_acceleration, bias));
      raw_g.feedImu(imu(stamp, (measured + body_acceleration) / gravity, bias));
    }
    si.feedCloud(cloud);
    raw_g.feedCloud(cloud);
    const auto si_result = si.processNext();
    const auto g_result = raw_g.processNext();
    require(si_result && g_result && !si_result->attempted && !g_result->attempted,
            "unit fixture unexpectedly applied LiDAR or stationary observations");
    const double duration = cloud.end_stamp_s - 1.0;
    // The first 5 ms interval averages the initial zero-acceleration sample
    // with the new acceleration. All remaining intervals have constant a.
    const Eigen::Vector3d expected_velocity(acceleration * (duration - 0.5 * dt), 0.0, 0.0);
    const Eigen::Vector3d expected_position(
        acceleration * (0.5 * duration * duration - 0.5 * duration * dt + 0.25 * dt * dt),
        0.0, 0.0);
    for (const auto* result : {&*si_result, &*g_result}) {
      require((result->world_position - expected_position).norm() < 1e-10 &&
                  (result->world_velocity - expected_velocity).norm() < 1e-10,
              "acc_scale was omitted or applied more than once during propagation");
    }
    require((si_result->world_position - g_result->world_position).norm() < 1e-10 &&
                (si_result->world_velocity - g_result->world_velocity).norm() < 1e-10 &&
                si_result->state.orientation.angularDistance(g_result->state.orientation) < 1e-10,
            "SI and scaled-g propagation disagree");
    require(si_result->state.covariance.llt().info() == Eigen::Success &&
                g_result->state.covariance.llt().info() == Eigen::Success,
            "scaled stationary IMU produced an invalid covariance");
  }
}
} // namespace

int main() {
  try {
    checkStartupImpactAndStationaryRetry();
    checkVaryingWindowIsRejected();
    checkExplicitSensorUnits();
    std::cout << "LTLIO IMU units, startup-impact rejection, stationary retry and covariance passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
