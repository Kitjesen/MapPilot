#include "imu_processor.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  Config config;
  config.imu_init_num = 200;
  auto filter = std::make_shared<IESKF>();
  IMUProcessor processor(config, filter);
  SyncPackage package;
  package.cloud_end_time = 1.0;
  for (int i = 0; i < config.imu_init_num; ++i) {
    IMUData sample;
    sample.acc = V3D(0.02, -0.01, 10.005);
    sample.gyro = V3D::Zero();
    sample.time = static_cast<double>(i) * 0.005;
    package.imus.push_back(sample);
  }

  require(processor.initialize(package), "stationary IMU initialization failed");
  const double normalized_gravity =
      package.imus.front().acc.norm() * processor.accelerationScale();
  require(
      std::abs(normalized_gravity - State::gravity) < 1e-9,
      "accelerometer magnitude was not normalized to gravity");
  require(
      processor.accelerationScale() > 0.97 && processor.accelerationScale() < 1.0,
      "unexpected accelerometer initialization scale");

  double package_start_s = 1.0;
  for (int frame = 0; frame < 100; ++frame) {
    SyncPackage next;
    next.cloud_start_time = package_start_s;
    next.cloud_end_time = package_start_s + 0.1;
    next.cloud = std::make_shared<CloudType>();
    PointType first;
    first.curvature = 0.0F;
    PointType last;
    last.curvature = 100.0F;
    next.cloud->push_back(first);
    next.cloud->push_back(last);
    for (int i = 0; i <= 20; ++i) {
      IMUData sample;
      sample.acc = V3D(0.02, -0.01, 10.005);
      sample.gyro = V3D::Zero();
      sample.time = package_start_s + static_cast<double>(i) * 0.005;
      next.imus.push_back(sample);
    }
    processor.undistort(next);
    package_start_s += 0.1;
  }
  require(
      filter->x().v.norm() < 1e-6,
      "normalized stationary IMU accumulated velocity");
  require(
      filter->x().t_wi.norm() < 1e-6,
      "normalized stationary IMU accumulated position drift");

  Config biased_config;
  biased_config.imu_init_num = 400;
  biased_config.imu_static_acc_thresh = 0.25;
  biased_config.imu_static_gyro_thresh = 0.02;
  biased_config.zupt_min_static_frames = 3;
  biased_config.zupt_sigma_v = 0.01;
  biased_config.zupt_sigma_pos = 0.08;
  auto biased_filter = std::make_shared<IESKF>();
  IMUProcessor biased_processor(biased_config, biased_filter);
  SyncPackage biased_init;
  biased_init.cloud_end_time = 2.0;
  const V3D live_gyro_bias(-0.03521, -0.00226, -0.00204);
  for (int i = 0; i < biased_config.imu_init_num; ++i) {
    IMUData sample;
    sample.acc = V3D(6.79172, -0.14845, -7.21200);
    sample.gyro = live_gyro_bias;
    sample.time = static_cast<double>(i) * 0.005;
    biased_init.imus.push_back(sample);
  }
  require(
      biased_processor.initialize(biased_init),
      "biased stationary IMU initialization failed");
  biased_filter->x().v = V3D(0.2, -0.1, 0.05);

  double biased_package_start_s = 2.0;
  for (int frame = 0; frame < 8; ++frame) {
    SyncPackage next;
    next.cloud_start_time = biased_package_start_s;
    next.cloud_end_time = biased_package_start_s + 0.1;
    next.cloud = std::make_shared<CloudType>();
    PointType first;
    first.curvature = 0.0F;
    PointType last;
    last.curvature = 100.0F;
    next.cloud->push_back(first);
    next.cloud->push_back(last);
    for (int i = 0; i <= 20; ++i) {
      IMUData sample;
      sample.acc = V3D(6.79172, -0.14845, -7.21200);
      sample.gyro = live_gyro_bias;
      sample.time = biased_package_start_s + static_cast<double>(i) * 0.005;
      next.imus.push_back(sample);
    }
    biased_processor.undistort(next);
    biased_package_start_s += 0.1;
  }
  require(
      biased_filter->x().v.norm() < 1e-3,
      "gyro-bias-compensated stationary detection did not inject ZUPT");

  Config thermal_config = biased_config;
  auto thermal_filter = std::make_shared<IESKF>();
  IMUProcessor thermal_processor(thermal_config, thermal_filter);
  SyncPackage thermal_init;
  thermal_init.cloud_end_time = 2.0;
  for (int i = 0; i < thermal_config.imu_init_num; ++i) {
    IMUData sample;
    sample.acc = V3D(0.0, 0.0, State::gravity);
    sample.gyro = live_gyro_bias;
    sample.time = static_cast<double>(i) * 0.005;
    thermal_init.imus.push_back(sample);
  }
  require(
      thermal_processor.initialize(thermal_init),
      "thermal-drift IMU initialization failed");
  const M3D rotation_before_thermal_drift = thermal_filter->x().r_wi;
  // The field failure was about 0.72 deg/min, i.e. roughly 2e-4 rad/s.
  // Keep the replay slightly above that while staying below deliberate motion.
  const V3D thermal_bias_step(0.0, 0.0, 0.0003);
  const V3D warmed_gyro_bias = live_gyro_bias + thermal_bias_step;

  double thermal_package_start_s = 2.0;
  constexpr int thermal_frame_count = 600;
  constexpr double thermal_frame_duration_s = 0.1;
  for (int frame = 0; frame < thermal_frame_count; ++frame) {
    SyncPackage next;
    next.cloud_start_time = thermal_package_start_s;
    next.cloud_end_time = thermal_package_start_s + thermal_frame_duration_s;
    next.cloud = std::make_shared<CloudType>();
    PointType first;
    first.curvature = 0.0F;
    PointType last;
    last.curvature = 100.0F;
    next.cloud->push_back(first);
    next.cloud->push_back(last);
    for (int i = 0; i <= 20; ++i) {
      IMUData sample;
      sample.acc = V3D(0.0, 0.0, State::gravity);
      sample.gyro = warmed_gyro_bias;
      sample.time =
          thermal_package_start_s + static_cast<double>(i) * 0.005;
      next.imus.push_back(sample);
    }
    thermal_processor.undistort(next);
    thermal_package_start_s += thermal_frame_duration_s;
  }

  const double thermal_duration_s =
      thermal_frame_count * thermal_frame_duration_s;
  const double uncompensated_yaw_rad = thermal_bias_step.z() * thermal_duration_s;
  const double gyro_bias_error =
      (thermal_filter->x().bg - warmed_gyro_bias).norm();
  const M3D relative_rotation =
      rotation_before_thermal_drift.transpose() * thermal_filter->x().r_wi;
  const double yaw_drift_rad =
      std::abs(std::atan2(relative_rotation(1, 0), relative_rotation(0, 0)));
  const double max_bias_error = thermal_bias_step.norm() * 0.2;
  const double max_yaw_drift_rad = uncompensated_yaw_rad * 0.25;
  if (gyro_bias_error >= max_bias_error || yaw_drift_rad >= max_yaw_drift_rad) {
    std::cerr << "stationary thermal gyro-bias adaptation: bias_error="
              << gyro_bias_error << " max_bias_error=" << max_bias_error
              << " yaw_drift_rad=" << yaw_drift_rad
              << " max_yaw_drift_rad=" << max_yaw_drift_rad
              << " uncompensated_yaw_rad=" << uncompensated_yaw_rad << '\n';
  }
  require(
      gyro_bias_error < max_bias_error && yaw_drift_rad < max_yaw_drift_rad,
      "stationary thermal drift did not adapt gyro bias and suppress yaw integration");

  Config gap_config = biased_config;
  auto gap_filter = std::make_shared<IESKF>();
  IMUProcessor gap_processor(gap_config, gap_filter);
  SyncPackage gap_init;
  gap_init.cloud_end_time = 2.0;
  for (int i = 0; i < gap_config.imu_init_num; ++i) {
    IMUData sample;
    sample.acc = V3D(0.0, 0.0, State::gravity);
    sample.gyro = live_gyro_bias;
    sample.time = static_cast<double>(i) * 0.005;
    gap_init.imus.push_back(sample);
  }
  require(gap_processor.initialize(gap_init), "gap-reset IMU initialization failed");
  const V3D gap_initial_bias = gap_filter->x().bg;
  double gap_package_start_s = 2.0;
  auto feed_gap_frame = [&](double start_s) {
    SyncPackage next;
    next.cloud_start_time = start_s;
    next.cloud_end_time = start_s + 0.1;
    next.cloud = std::make_shared<CloudType>();
    PointType first;
    first.curvature = 0.0F;
    PointType last;
    last.curvature = 100.0F;
    next.cloud->push_back(first);
    next.cloud->push_back(last);
    for (int i = 0; i <= 20; ++i) {
      IMUData sample;
      sample.acc = V3D(0.0, 0.0, State::gravity);
      sample.gyro = live_gyro_bias + thermal_bias_step;
      sample.time = start_s + static_cast<double>(i) * 0.005;
      next.imus.push_back(sample);
    }
    gap_processor.undistort(next);
  };
  for (int frame = 0; frame < 49; ++frame) {
    feed_gap_frame(gap_package_start_s);
    gap_package_start_s += 0.1;
  }
  require(
      (gap_filter->x().bg - gap_initial_bias).norm() < 1e-12,
      "gyro bias adapted before the 50-frame stationary gate");
  gap_package_start_s += 5.0;
  feed_gap_frame(gap_package_start_s);
  require(
      (gap_filter->x().bg - gap_initial_bias).norm() < 1e-12,
      "gyro bias stationary gate survived a long input gap");

  Config rotating_config = biased_config;
  auto rotating_filter = std::make_shared<IESKF>();
  IMUProcessor rotating_processor(rotating_config, rotating_filter);
  SyncPackage rotating_init;
  rotating_init.cloud_end_time = 2.0;
  for (int i = 0; i < rotating_config.imu_init_num; ++i) {
    IMUData sample;
    sample.acc = V3D(0.0, 0.0, State::gravity);
    sample.gyro = live_gyro_bias;
    sample.time = static_cast<double>(i) * 0.005;
    rotating_init.imus.push_back(sample);
  }
  require(
      rotating_processor.initialize(rotating_init),
      "real-yaw-rate IMU initialization failed");
  const V3D gyro_bias_before_rotation = rotating_filter->x().bg;
  const M3D rotation_before_real_yaw = rotating_filter->x().r_wi;
  constexpr double real_yaw_rate_rad_s = 0.004;
  const V3D rotating_gyro =
      live_gyro_bias + V3D(0.0, 0.0, real_yaw_rate_rad_s);

  double rotating_package_start_s = 2.0;
  constexpr int rotating_frame_count = 600;
  constexpr double rotating_frame_duration_s = 0.1;
  for (int frame = 0; frame < rotating_frame_count; ++frame) {
    SyncPackage next;
    next.cloud_start_time = rotating_package_start_s;
    next.cloud_end_time =
        rotating_package_start_s + rotating_frame_duration_s;
    next.cloud = std::make_shared<CloudType>();
    PointType first;
    first.curvature = 0.0F;
    PointType last;
    last.curvature = 100.0F;
    next.cloud->push_back(first);
    next.cloud->push_back(last);
    for (int i = 0; i <= 20; ++i) {
      IMUData sample;
      sample.acc = V3D(0.0, 0.0, State::gravity);
      sample.gyro = rotating_gyro;
      sample.time =
          rotating_package_start_s + static_cast<double>(i) * 0.005;
      next.imus.push_back(sample);
    }
    rotating_processor.undistort(next);
    rotating_package_start_s += rotating_frame_duration_s;
  }

  const double rotating_duration_s =
      rotating_frame_count * rotating_frame_duration_s;
  const double theoretical_yaw_rad =
      real_yaw_rate_rad_s * rotating_duration_s;
  const double learned_rotation_bias_rad_s =
      (rotating_filter->x().bg - gyro_bias_before_rotation).norm();
  const M3D relative_real_yaw =
      rotation_before_real_yaw.transpose() * rotating_filter->x().r_wi;
  const double integrated_real_yaw_rad =
      std::abs(std::atan2(relative_real_yaw(1, 0), relative_real_yaw(0, 0)));
  std::cout << "real yaw rotation guard: learned_bias_rad_s="
            << learned_rotation_bias_rad_s
            << " integrated_yaw_rad=" << integrated_real_yaw_rad
            << " theoretical_yaw_rad=" << theoretical_yaw_rad << '\n';
  require(
      learned_rotation_bias_rad_s < 1e-4,
      "ZARU learned a constant real yaw rate into gyro bias");
  require(
      integrated_real_yaw_rad > theoretical_yaw_rad * 0.8,
      "ZARU suppressed too much constant real yaw rotation");

  auto correlated_filter = std::make_shared<IESKF>();
  correlated_filter->x().t_wi = V3D(1.0, -2.0, 0.5);
  correlated_filter->x().v = V3D(0.3, -0.2, 0.1);
  correlated_filter->P().setIdentity();
  correlated_filter->P().block<3, 3>(3, 12) = M3D::Identity() * 0.8;
  correlated_filter->P().block<3, 3>(12, 3) = M3D::Identity() * 0.8;
  const V3D position_before_zupt = correlated_filter->x().t_wi;
  correlated_filter->injectZUPT(0.01, 0.08);
  require(
      (correlated_filter->x().t_wi - position_before_zupt).norm() < 1e-12,
      "zero-velocity update changed absolute position");
  require(
      correlated_filter->x().v.norm() < 1e-3,
      "zero-velocity update did not constrain velocity");

  auto configure_bounded_update = [](IESKF& candidate, bool reject_nonconverged) {
    candidate.P().setIdentity();
    candidate.setMaxIter(1);
    candidate.setDegeneracyGuard(
        2, 50000.0, 0.5, 0.35, 3.0, 1.0,
        reject_nonconverged, reject_nonconverged);
    candidate.setLossFunction([](State&, SharedState& shared) {
      shared.H.setIdentity();
      shared.b.setZero();
      shared.valid = true;
    });
    candidate.setStopFunction([](const V21D&) { return false; });
  };

  Config field_defaults;
  require(
      !field_defaults.reject_nonconverged_update,
      "field default rejects every bounded LiDAR update that reaches the iteration limit");
  require(
      field_defaults.reject_degenerate_nonconverged_update,
      "field default must still reject non-converged updates in degenerate geometry");

  IESKF strict_update;
  configure_bounded_update(strict_update, true);
  require(
      !strict_update.update(),
      "non-converged rejected update was reported as map-integrable");

  IESKF bounded_update;
  configure_bounded_update(bounded_update, false);
  require(
      bounded_update.update(),
      "bounded non-converged update was not reported as accepted");

  IESKF invalid_update;
  invalid_update.P().setIdentity();
  invalid_update.setLossFunction([](State&, SharedState& shared) {
    shared.valid = false;
  });
  invalid_update.setStopFunction([](const V21D&) { return true; });
  require(
      !invalid_update.update(),
      "update without a valid LiDAR residual was reported as map-integrable");
  return 0;
}
