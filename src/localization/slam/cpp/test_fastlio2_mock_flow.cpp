#include "slam.hpp"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace lingtu::slam;

namespace {

ImuSample imu(double stamp_s) {
  ImuSample sample;
  sample.stamp_s = stamp_s;
  sample.qw = 1.0;
  sample.az = 1.0;
  return sample;
}

ImuSample acceleratedImu(double stamp_s) {
  ImuSample sample = imu(stamp_s);
  sample.ax = 100.0;
  return sample;
}

LidarFrame lidar(double stamp_s, float dx) {
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

void processScan(
    ISlamBackend& backend,
    double scan_stamp_s,
    float dx,
    double& next_imu_stamp_s) {
  while (next_imu_stamp_s <= scan_stamp_s + 0.06) {
    check(backend.feedImu(imu(next_imu_stamp_s)).ok, "feed_health_imu_failed");
    next_imu_stamp_s += 0.005;
  }
  check(backend.feedLidar(lidar(scan_stamp_s, dx)).ok, "feed_health_lidar_failed");
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend.tick().ok, "health_tick_failed");
  }
}

void processAcceleratedScan(
    ISlamBackend& backend,
    double scan_stamp_s,
    double& next_imu_stamp_s) {
  while (next_imu_stamp_s <= scan_stamp_s + 0.06) {
    check(
        backend.feedImu(acceleratedImu(next_imu_stamp_s)).ok,
        "feed_accelerated_imu_failed");
    next_imu_stamp_s += 0.005;
  }
  check(backend.feedLidar(lidar(scan_stamp_s, 0.0F)).ok, "feed_accelerated_lidar_failed");
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend.tick().ok, "accelerated_tick_failed");
  }
}

std::unique_ptr<ISlamBackend> initializedMappingBackend(
    const std::filesystem::path& config_path,
    double& next_imu_stamp_s,
    const std::optional<OdomSample>& initialization_prior = std::nullopt,
    bool expect_tracking = true) {
  auto backend = makeFastLioBackend();
  check(backend != nullptr, "health_backend_missing");

  SlamConfig config;
  config.backend = "fastlio2";
  config.config_path = config_path.string();
  check(backend->configure(config).ok, "health_configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "health_set_mode_failed");

  for (int i = 0; i <= 60; ++i) {
    check(backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok, "health_init_imu_failed");
  }
  check(backend->feedLidar(lidar(0.20, 0.0F)).ok, "health_init_lidar_1_failed");
  check(backend->feedLidar(lidar(0.30, 0.02F)).ok, "health_init_lidar_2_failed");
  if (initialization_prior.has_value()) {
    check(
        backend->feedVisualOdom(*initialization_prior).ok,
        "health_initialization_prior_failed");
  }
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend->tick().ok, "health_init_tick_failed");
  }
  if (expect_tracking) {
    check(
        backend->outputs().state == SlamState::Tracking,
        "health_baseline_not_tracking");
  }
  next_imu_stamp_s = 0.61;
  return backend;
}

void checkRejectedUpdateHealthGate(const std::filesystem::path& config_path) {
  double next_imu_stamp_s = 0.0;
  auto backend = initializedMappingBackend(config_path, next_imu_stamp_s);

  processScan(*backend, 0.40, 10.0F, next_imu_stamp_s);
  const auto one_rejection = backend->outputs();
  check(one_rejection.state == SlamState::Degraded, "single_rejection_not_degraded");
  check(
      one_rejection.reason == "fastlio_lidar_update_rejected",
      "single_rejection_reason_mismatch");
  check(one_rejection.confidence == 0.0, "single_rejection_confidence_not_zero");
  check(!one_rejection.odometry_odom_body.has_value(), "single_rejection_published_odom");
  check(!one_rejection.registered_cloud_body.has_value(), "single_rejection_published_cloud");
  check(!one_rejection.map_cloud_map.has_value(), "single_rejection_published_map_cloud");
  check(one_rejection.fastlio_lidar_update.attempted, "single_rejection_attempt_missing");
  check(!one_rejection.fastlio_lidar_update.accepted, "single_rejection_marked_accepted");
  check(
      one_rejection.fastlio_lidar_update.rejection_reason != "none",
      "single_rejection_detail_missing");
  check(
      one_rejection.fastlio_lidar_update.consecutive_rejections == 1U,
      "single_rejection_detail_streak_mismatch");
  check(
      one_rejection.fastlio_lidar_update.downsampled_points > 0U,
      "single_rejection_downsampled_points_missing");
  const std::uint64_t rejected_attempt_sequence =
      one_rejection.fastlio_lidar_update.attempt_sequence;
  const std::string rejected_detail_reason =
      one_rejection.fastlio_lidar_update.rejection_reason;

  processScan(*backend, 0.50, 0.0F, next_imu_stamp_s);
  const auto recovered = backend->outputs();
  check(recovered.state == SlamState::Tracking, "single_rejection_did_not_recover");
  check(recovered.odometry_odom_body.has_value(), "recovery_odometry_missing");
  check(recovered.registered_cloud_body.has_value(), "recovery_cloud_missing");
  check(recovered.fastlio_lidar_update.accepted, "recovery_update_not_marked_accepted");
  check(
      recovered.fastlio_lidar_update.rejection_reason == "none",
      "recovery_retained_current_rejection_detail");
  check(
      recovered.fastlio_lidar_update.previous_rejection_reason ==
          rejected_detail_reason,
      "recovery_lost_previous_rejection_detail");
  check(
      recovered.fastlio_lidar_update.attempt_sequence ==
          rejected_attempt_sequence + 1U,
      "recovery_attempt_sequence_mismatch");
  check(
      recovered.fastlio_lidar_update.consecutive_rejections == 0U,
      "recovery_did_not_clear_detail_streak");

  processScan(*backend, 0.60, 10.0F, next_imu_stamp_s);
  processScan(*backend, 0.70, 10.0F, next_imu_stamp_s);
  const auto repeated_rejection = backend->outputs();
  check(repeated_rejection.state == SlamState::Lost, "repeated_rejection_not_lost");
  check(
      repeated_rejection.reason == "fastlio_lidar_update_rejected_streak",
      "repeated_rejection_reason_mismatch");
  check(repeated_rejection.confidence == 0.0, "repeated_rejection_confidence_not_zero");
  check(!repeated_rejection.odometry_odom_body.has_value(), "repeated_rejection_published_odom");
  check(!repeated_rejection.registered_cloud_body.has_value(), "repeated_rejection_published_cloud");
  check(!repeated_rejection.map_cloud_map.has_value(), "repeated_rejection_published_map_cloud");
  check(
      repeated_rejection.fastlio_lidar_update.consecutive_rejections == 2U,
      "repeated_rejection_detail_streak_mismatch");
  check(
      repeated_rejection.fastlio_lidar_update.rejection_reason != "none",
      "repeated_rejection_detail_missing");
  check(
      repeated_rejection.fastlio_lidar_update.previous_rejection_reason != "none",
      "repeated_rejection_previous_detail_missing");
}

void checkPositionCovarianceHealthGate(const std::filesystem::path& config_path) {
  double next_imu_stamp_s = 0.0;
  auto backend = initializedMappingBackend(config_path, next_imu_stamp_s, std::nullopt, false);
  const auto outputs = backend->outputs();
  check(outputs.state == SlamState::Lost, "unsafe_position_covariance_not_lost");
  check(
      outputs.reason == "fastlio_position_covariance_out_of_bounds",
      "unsafe_position_covariance_reason_mismatch");
  check(outputs.confidence == 0.0, "unsafe_position_covariance_confidence_not_zero");
  check(!outputs.odometry_odom_body.has_value(), "unsafe_position_covariance_published_odom");
  check(!outputs.map_cloud_map.has_value(), "unsafe_position_covariance_published_map_cloud");
}

void checkVelocityHealthGate(const std::filesystem::path& config_path) {
  OdomSample unsafe_prior;
  unsafe_prior.stamp_s = 0.30;
  unsafe_prior.has_velocity = true;
  unsafe_prior.vx = 3.5;
  double unsafe_imu_stamp_s = 0.0;
  auto backend = initializedMappingBackend(
      config_path, unsafe_imu_stamp_s, unsafe_prior, false);

  const auto unsafe = backend->outputs();
  check(unsafe.state == SlamState::Lost, "unsafe_velocity_not_lost");
  check(
      unsafe.reason == "fastlio_velocity_out_of_bounds",
      "unsafe_velocity_reason_mismatch");
  check(unsafe.confidence == 0.0, "unsafe_velocity_confidence_not_zero");
  check(!unsafe.odometry_odom_body.has_value(), "unsafe_velocity_published_odom");
  check(!unsafe.registered_cloud_body.has_value(), "unsafe_velocity_published_cloud");
  check(!unsafe.map_cloud_map.has_value(), "unsafe_velocity_published_map_cloud");

  OdomSample safe_prior;
  safe_prior.stamp_s = 0.40;
  safe_prior.has_velocity = true;
  check(backend->feedVisualOdom(safe_prior).ok, "safe_prior_after_fault_rejected");
  processScan(*backend, 0.40, 0.0F, unsafe_imu_stamp_s);
  const auto mapping_fault_latched = backend->outputs();
  check(
      mapping_fault_latched.state == SlamState::Lost,
      "mapping_catastrophic_fault_recovered_without_reset");
  check(
      mapping_fault_latched.reason == "fastlio_velocity_out_of_bounds",
      "mapping_catastrophic_fault_reason_not_latched");
  check(
      !mapping_fault_latched.odometry_odom_body.has_value(),
      "mapping_catastrophic_fault_republished_odom");
  check(
      !mapping_fault_latched.map_cloud_map.has_value(),
      "mapping_catastrophic_fault_republished_map_cloud");

  check(backend->reset().ok, "mapping_catastrophic_reset_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "mapping_mode_after_reset_failed");
  for (int i = 0; i <= 60; ++i) {
    check(
        backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok,
        "mapping_reset_imu_failed");
  }
  check(backend->feedLidar(lidar(0.20, 0.0F)).ok, "mapping_reset_lidar_1_failed");
  check(backend->feedLidar(lidar(0.30, 0.02F)).ok, "mapping_reset_lidar_2_failed");
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend->tick().ok, "mapping_reset_tick_failed");
  }
  const auto mapping_after_reset = backend->outputs();
  check(mapping_after_reset.state == SlamState::Tracking, "mapping_reset_did_not_clear_fault");
  check(mapping_after_reset.odometry_odom_body.has_value(), "mapping_reset_odometry_missing");

  OdomSample large_origin_prior;
  large_origin_prior.stamp_s = 0.30;
  large_origin_prior.odom_body.x = 1000.0;
  large_origin_prior.odom_body.y = -1000.0;
  large_origin_prior.has_velocity = true;
  double large_origin_imu_stamp_s = 0.0;
  auto large_origin_backend =
      initializedMappingBackend(config_path, large_origin_imu_stamp_s, large_origin_prior);

  const auto safe_large_origin = large_origin_backend->outputs();
  check(
      safe_large_origin.state == SlamState::Tracking,
      "large_origin_zero_velocity_misclassified");
  check(safe_large_origin.odometry_odom_body.has_value(), "large_origin_odometry_missing");
  check(safe_large_origin.map_cloud_map.has_value(), "large_origin_map_cloud_missing");

  double localization_imu_stamp_s = 0.0;
  auto localization_backend =
      initializedMappingBackend(config_path, localization_imu_stamp_s);
  const auto localization_map_path = config_path.parent_path() / "catastrophic_latch_map.pcd";
  check(
      localization_backend->saveMap(localization_map_path.string()).ok,
      "catastrophic_latch_map_save_failed");
  check(
      localization_backend->setMode(
          SlamMode::Localization, localization_map_path.string()).ok,
      "catastrophic_latch_localization_mode_failed");

  OdomSample localization_unsafe_prior;
  localization_unsafe_prior.stamp_s = 0.40;
  localization_unsafe_prior.has_velocity = true;
  localization_unsafe_prior.vx = 3.5;
  check(
      localization_backend->feedVisualOdom(localization_unsafe_prior).ok,
      "localization_unsafe_prior_rejected");
  processScan(*localization_backend, 0.40, 0.0F, localization_imu_stamp_s);
  check(
      localization_backend->outputs().state == SlamState::Lost,
      "localization_unsafe_velocity_not_lost");

  OdomSample localization_safe_prior;
  localization_safe_prior.stamp_s = 0.50;
  localization_safe_prior.has_velocity = true;
  check(
      localization_backend->feedVisualOdom(localization_safe_prior).ok,
      "localization_safe_prior_rejected");
  processScan(*localization_backend, 0.50, 0.0F, localization_imu_stamp_s);
  const auto localization_fault_latched = localization_backend->outputs();
  check(
      localization_fault_latched.state == SlamState::Lost,
      "localization_catastrophic_fault_recovered_without_relocalization");
  check(
      !localization_fault_latched.odometry_odom_body.has_value(),
      "localization_catastrophic_fault_republished_odom");
  check(
      localization_backend->relocalize(Pose3d{}).ok,
      "localization_catastrophic_relocalization_failed");
  const auto localization_after_relocalization = localization_backend->outputs();
  check(
      localization_after_relocalization.state == SlamState::Tracking,
      "successful_relocalization_did_not_clear_catastrophic_fault");
  check(
      localization_after_relocalization.odometry_odom_body.has_value(),
      "relocalized_odometry_missing_after_catastrophic_fault");
}

void checkRelocalizationCannotClearUnhealthyNumericState(
    const std::filesystem::path& config_path) {
  double next_imu_stamp_s = 0.0;
  auto backend = initializedMappingBackend(config_path, next_imu_stamp_s);
  const auto map_path = config_path.parent_path() / "unhealthy_numeric_state_map.pcd";
  check(backend->saveMap(map_path.string()).ok, "unhealthy_numeric_map_save_failed");
  check(
      backend->setMode(SlamMode::Localization, map_path.string()).ok,
      "unhealthy_numeric_localization_mode_failed");

  processAcceleratedScan(*backend, 0.80, next_imu_stamp_s);
  const auto fault = backend->outputs();
  check(fault.state == SlamState::Lost, "accelerated_fastlio_state_not_lost");
  check(
      fault.reason == "fastlio_velocity_out_of_bounds",
      "accelerated_fastlio_fault_reason_mismatch");

  const Status sync_relocalization = backend->relocalize(Pose3d{});
  check(
      !sync_relocalization.ok,
      "sync_relocalization_cleared_unhealthy_numeric_state");
  check(
      sync_relocalization.message == "fastlio_numeric_health_unrecovered",
      "sync_unhealthy_numeric_relocalization_reason_mismatch");
  const auto after_sync = backend->outputs();
  check(after_sync.state == SlamState::Lost, "sync_relocalization_cleared_fault_latch");
  check(
      after_sync.reason == "fastlio_velocity_out_of_bounds",
      "sync_relocalization_overwrote_fault_reason");
  check(!after_sync.odometry_odom_body.has_value(), "sync_relocalization_republished_bad_odom");

  check(
      backend->startRelocalizeAsync(Pose3d{}).ok,
      "async_unhealthy_numeric_relocalization_start_failed");
  std::optional<Status> async_completion;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    async_completion = backend->pollRelocalizeAsync();
    if (async_completion.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(async_completion.has_value(), "async_unhealthy_numeric_relocalization_timeout");
  check(
      !async_completion->ok,
      "async_relocalization_cleared_unhealthy_numeric_state");
  check(
      async_completion->message == "fastlio_numeric_health_unrecovered",
      "async_unhealthy_numeric_relocalization_reason_mismatch");
  const auto after_async = backend->outputs();
  check(after_async.state == SlamState::Lost, "async_relocalization_cleared_fault_latch");
  check(
      after_async.reason == "fastlio_velocity_out_of_bounds",
      "async_relocalization_overwrote_fault_reason");
  check(!after_async.odometry_odom_body.has_value(), "async_relocalization_republished_bad_odom");
}

void checkFailedRelocalizationPreservesCatastrophicFault(
    const std::filesystem::path& config_path,
    const Pose3d& guess,
    const std::string& expected_failure_message,
    const std::string& expected_relocalization_state,
    const std::string& case_name) {
  for (const bool async : {true, false}) {
    const std::string invocation = async ? "async" : "sync";
    const std::string prefix = case_name + "_" + invocation;
    double next_imu_stamp_s = 0.0;
    auto backend = initializedMappingBackend(config_path, next_imu_stamp_s);
    const auto map_path =
        config_path.parent_path() / (prefix + "_catastrophic_latch_map.pcd");
    check(
        backend->saveMap(map_path.string()).ok,
        (prefix + "_map_save_failed").c_str());
    check(
        backend->setMode(SlamMode::Localization, map_path.string()).ok,
        (prefix + "_localization_mode_failed").c_str());

    processAcceleratedScan(*backend, 0.80, next_imu_stamp_s);
    const auto fault = backend->outputs();
    check(fault.state == SlamState::Lost, (prefix + "_fault_not_lost").c_str());
    check(
        fault.reason == "fastlio_velocity_out_of_bounds",
        (prefix + "_fault_reason_mismatch").c_str());

    Status relocalization;
    if (async) {
      check(
          backend->startRelocalizeAsync(guess).ok,
          (prefix + "_start_failed").c_str());
      std::optional<Status> completion;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (std::chrono::steady_clock::now() < deadline) {
        completion = backend->pollRelocalizeAsync();
        if (completion.has_value()) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      check(completion.has_value(), (prefix + "_timeout").c_str());
      relocalization = *completion;
    } else {
      relocalization = backend->relocalize(guess);
    }

    if (relocalization.message != expected_failure_message) {
      std::cerr << prefix << "_unexpected_failure=" << relocalization.message << "\n";
    }
    check(!relocalization.ok, (prefix + "_unexpected_success").c_str());
    check(
        relocalization.message == expected_failure_message,
        (prefix + "_failure_message_mismatch").c_str());
    const auto after_failure = backend->outputs();
    check(
        after_failure.state == SlamState::Lost,
        (prefix + "_cleared_fault_latch").c_str());
    check(
        after_failure.reason == "fastlio_velocity_out_of_bounds",
        (prefix + "_overwrote_fault_reason").c_str());
    check(
        after_failure.relocalization_state == expected_relocalization_state,
        (prefix + "_relocalization_state_mismatch").c_str());
    check(
        after_failure.last_relocalization_message == expected_failure_message,
        (prefix + "_last_message_mismatch").c_str());
  }
}

void checkSensorTimeJumpGate(const std::filesystem::path& config_path) {
  double next_imu_stamp_s = 0.0;
  auto mapping_backend = initializedMappingBackend(config_path, next_imu_stamp_s);
  const auto mapping_before_jump = mapping_backend->outputs();
  check(mapping_backend->feedImu(imu(100.0)).ok, "mapping_time_jump_imu_rejected");
  const auto mapping_jump = mapping_backend->outputs();
  check(
      mapping_jump.source_epoch == mapping_before_jump.source_epoch + 1U,
      "mapping_time_jump_source_epoch_not_advanced");
  check(
      mapping_jump.state == SlamState::Initializing,
      "mapping_time_jump_did_not_reinitialize");
  check(
      mapping_jump.reason == "sensor_time_jump_reset_mapping",
      "mapping_time_jump_reason_mismatch");
  check(mapping_jump.confidence == 0.0, "mapping_time_jump_confidence_not_zero");
  check(!mapping_jump.odometry_odom_body.has_value(), "mapping_time_jump_retained_odom");
  check(!mapping_jump.registered_cloud_body.has_value(), "mapping_time_jump_retained_cloud");
  check(!mapping_jump.map_cloud_map.has_value(), "mapping_time_jump_retained_map_cloud");

  auto localization_backend = initializedMappingBackend(config_path, next_imu_stamp_s);
  check(
      localization_backend->setMode(SlamMode::Localization, "").ok,
      "time_jump_localization_mode_failed");
  const auto localization_before_jump = localization_backend->outputs();
  check(
      localization_backend->feedLidar(lidar(100.0, 0.0F)).ok,
      "localization_time_jump_lidar_rejected");
  const auto localization_jump = localization_backend->outputs();
  check(
      localization_jump.source_epoch == localization_before_jump.source_epoch + 1U,
      "localization_time_jump_source_epoch_not_advanced");
  check(
      localization_jump.state == SlamState::Lost,
      "localization_time_jump_not_lost");
  check(
      localization_jump.reason == "sensor_time_jump_relocalization_required",
      "localization_time_jump_reason_mismatch");
  check(
      localization_jump.confidence == 0.0,
      "localization_time_jump_confidence_not_zero");
  check(
      !localization_jump.odometry_odom_body.has_value(),
      "localization_time_jump_retained_odom");
  check(
      !localization_jump.registered_cloud_body.has_value(),
      "localization_time_jump_retained_cloud");
  check(
      !localization_jump.map_cloud_map.has_value(),
      "localization_time_jump_retained_map_cloud");

  for (int i = 0; i <= 60; ++i) {
    check(
        localization_backend->feedImu(imu(100.0 + static_cast<double>(i) * 0.01)).ok,
        "localization_new_epoch_imu_failed");
  }
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(localization_backend->tick().ok, "localization_new_epoch_tick_failed");
  }
  const auto localization_reinitializing = localization_backend->outputs();
  check(
      localization_reinitializing.state == SlamState::Lost,
      "localization_time_jump_left_lost_before_relocalization");
  check(
      localization_reinitializing.reason == "sensor_time_jump_relocalization_required",
      "localization_reinitializing_reason_mismatch");
  check(
      !localization_reinitializing.odometry_odom_body.has_value(),
      "localization_reinitializing_published_odom");
  check(
      !localization_reinitializing.map_cloud_map.has_value(),
      "localization_reinitializing_published_map_cloud");
}

void checkOdomPriorBypass(const std::filesystem::path& config_path) {
  auto backend = makeFastLioBackend();
  check(backend != nullptr, "bypass_backend_missing");
  SlamConfig config;
  config.backend = "fastlio2";
  config.config_path = config_path.string();
  check(backend->configure(config).ok, "bypass_configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "bypass_set_mode_failed");

  OdomSample prior;
  prior.stamp_s = 0.30;
  prior.odom_body.x = 5.0;
  prior.odom_body.y = -2.0;
  prior.has_velocity = true;
  prior.vx = 0.2;
  check(backend->feedVisualOdom(prior).ok, "bypass_prior_rejected");
  for (int i = 0; i <= 50; ++i) {
    check(
        backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok,
        "bypass_imu_rejected");
  }
  check(backend->feedLidar(lidar(0.30, 0.0F)).ok, "bypass_lidar_rejected");
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend->tick().ok, "bypass_tick_failed");
  }

  const auto outputs = backend->outputs();
  check(outputs.state == SlamState::Tracking, "bypass_not_tracking");
  check(
      outputs.reason == "tracking_with_odom_prior_bypass",
      "bypass_reason_mismatch");
  check(outputs.odom_prior_active, "bypass_prior_not_active");
  check(outputs.odometry_odom_body.has_value(), "bypass_odometry_missing");
  check(
      std::abs(outputs.odometry_odom_body->x - 5.0) < 1e-9 &&
          std::abs(outputs.odometry_odom_body->y + 2.0) < 1e-9,
      "bypass_odometry_mismatch");
  check(outputs.registered_cloud_body.has_value(), "bypass_registered_cloud_missing");
  check(outputs.map_cloud_map.has_value(), "bypass_map_cloud_missing");
}

void checkOdomPriorHistoryBypass(const std::filesystem::path& config_path) {
  auto backend = makeFastLioBackend();
  check(backend != nullptr, "bypass_history_backend_missing");
  SlamConfig config;
  config.backend = "fastlio2";
  config.config_path = config_path.string();
  check(backend->configure(config).ok, "bypass_history_configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "bypass_history_set_mode_failed");

  OdomSample scan_prior;
  scan_prior.stamp_s = 0.34;
  scan_prior.odom_body.x = 5.0;
  scan_prior.odom_body.y = -2.0;
  scan_prior.has_velocity = true;
  scan_prior.vx = 0.2;
  check(backend->feedVisualOdom(scan_prior).ok, "bypass_history_scan_prior_rejected");

  OdomSample future_prior = scan_prior;
  future_prior.stamp_s = 1.00;
  future_prior.odom_body.x = 9.0;
  check(backend->feedVisualOdom(future_prior).ok, "bypass_history_future_prior_rejected");
  for (int i = 0; i <= 110; ++i) {
    check(
        backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok,
        "bypass_history_imu_rejected");
  }
  check(backend->feedLidar(lidar(0.30, 0.0F)).ok, "bypass_history_lidar_rejected");
  for (int tick_index = 0; tick_index < 4; ++tick_index) {
    check(backend->tick().ok, "bypass_history_tick_failed");
  }

  const auto outputs = backend->outputs();
  check(outputs.state == SlamState::Tracking, "bypass_history_not_tracking");
  check(
      outputs.reason == "tracking_with_odom_prior_bypass",
      "bypass_history_reason_mismatch");
  check(outputs.odom_prior_active, "bypass_history_prior_not_active");
  check(outputs.odometry_odom_body.has_value(), "bypass_history_odometry_missing");
  check(
      std::abs(outputs.odometry_odom_body->x - 5.0) < 1e-9,
      "bypass_history_selected_latest_instead_of_scan_prior");

  const std::uint64_t observation_sequence = outputs.observation_sequence;
  for (int i = 111; i <= 150; ++i) {
    check(
        backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok,
        "bypass_history_late_imu_rejected");
  }
  check(backend->feedLidar(lidar(1.20, 0.0F)).ok, "bypass_history_gap_lidar_rejected");
  check(backend->tick().ok, "bypass_history_gap_tick_failed");
  const auto waiting = backend->outputs();
  check(waiting.state == SlamState::Tracking, "bypass_history_gap_lost_tracking");
  check(
      waiting.reason == "waiting_for_time_aligned_odom_prior",
      "bypass_history_gap_fell_back_to_fastlio");
  check(
      waiting.observation_sequence == observation_sequence,
      "bypass_history_gap_published_observation");

  OdomSample recovery_prior = scan_prior;
  recovery_prior.stamp_s = 1.34;
  recovery_prior.odom_body.x = 11.0;
  check(
      backend->feedVisualOdom(recovery_prior).ok,
      "bypass_history_recovery_prior_rejected");
  check(
      backend->feedLidar(lidar(1.30, 0.0F)).ok,
      "bypass_history_recovery_lidar_rejected");
  check(backend->tick().ok, "bypass_history_recovery_tick_failed");
  const auto recovered = backend->outputs();
  check(recovered.state == SlamState::Tracking, "bypass_history_did_not_recover");
  check(
      recovered.reason == "tracking_with_odom_prior_bypass",
      "bypass_history_recovery_reason_mismatch");
  check(
      recovered.observation_sequence == observation_sequence + 1U,
      "bypass_history_recovery_observation_missing");
  check(
      recovered.odometry_odom_body.has_value() &&
          std::abs(recovered.odometry_odom_body->x - 11.0) < 1e-9,
      "bypass_history_recovery_pose_mismatch");
}

void checkOdomPriorBypassDoesNotFallback(const std::filesystem::path& config_path) {
  auto backend = makeFastLioBackend();
  check(backend != nullptr, "bypass_no_fallback_backend_missing");
  SlamConfig config;
  config.backend = "fastlio2";
  config.config_path = config_path.string();
  check(backend->configure(config).ok, "bypass_no_fallback_configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "bypass_no_fallback_set_mode_failed");
  for (int i = 0; i <= 50; ++i) {
    check(
        backend->feedImu(imu(static_cast<double>(i) * 0.01)).ok,
        "bypass_no_fallback_imu_rejected");
  }
  check(backend->feedLidar(lidar(0.30, 0.0F)).ok, "bypass_no_fallback_lidar_rejected");
  check(backend->tick().ok, "bypass_no_fallback_tick_failed");

  const auto outputs = backend->outputs();
  check(
      outputs.reason == "waiting_for_time_aligned_odom_prior",
      "bypass_missing_prior_fell_back_to_fastlio");
  check(!outputs.odometry_odom_body.has_value(), "bypass_missing_prior_published_odometry");
  check(!outputs.registered_cloud_body.has_value(), "bypass_missing_prior_published_cloud");
}

}  // namespace

int main() {
  const auto map_dir =
      std::filesystem::temp_directory_path() / "lingtu_fastlio2_mock_flow_map";
  std::filesystem::remove_all(map_dir);
  std::filesystem::create_directories(map_dir);
  const auto config_path = map_dir / "fastlio_test.yaml";
  {
    std::ofstream config_out(config_path);
    config_out << "relocalization_map_bounds_margin_m: 10.0\n";
    config_out << "relocalization_min_inliers: 10\n";
    config_out << "relocalization_max_pos_cov_trace: 100.0\n";
    config_out << "track_against_map_require_degeneracy_metrics: false\n";
    config_out << "odom_prior_enabled: true\n";
    config_out << "max_update_velocity_mps: 3.0\n";
    config_out << "max_sensor_time_jump_s: 1.0\n";
    config_out << "reject_nonconverged_update: false\n";
    config_out << "reject_degenerate_nonconverged_update: false\n";
    config_out << "degeneracy_max_update_dof: 5\n";
    config_out << "degeneracy_max_condition: 1000000000000.0\n";
    config_out << "max_update_translation_m: 10.0\n";
    config_out << "max_update_rotation_rad: 3.14\n";
    config_out << "max_update_velocity_delta_mps: 100.0\n";
  }

  const auto bypass_config_path = map_dir / "fastlio_bypass_test.yaml";
  {
    std::ofstream config_out(bypass_config_path);
    config_out << "odom_prior_enabled: true\n";
    config_out << "odom_prior_bypass_fastlio: true\n";
    config_out << "odom_prior_max_age_s: 0.20\n";
    config_out << "max_update_velocity_mps: 3.0\n";
  }

  const auto covariance_gate_config_path = map_dir / "fastlio_covariance_gate_test.yaml";
  {
    std::ofstream config_out(covariance_gate_config_path);
    config_out << "fastlio_max_pos_cov_trace: 0.000000001\n";
    config_out << "odom_prior_enabled: false\n";
    config_out << "max_update_velocity_mps: 3.0\n";
    config_out << "reject_nonconverged_update: false\n";
    config_out << "reject_degenerate_nonconverged_update: false\n";
    config_out << "degeneracy_max_update_dof: 5\n";
    config_out << "degeneracy_max_condition: 1000000000000.0\n";
    config_out << "max_update_translation_m: 10.0\n";
    config_out << "max_update_rotation_rad: 3.14\n";
    config_out << "max_update_velocity_delta_mps: 100.0\n";
  }

  const auto relocalization_gate_failure_config_path =
      map_dir / "fastlio_relocalization_gate_failure_test.yaml";
  {
    std::ofstream config_out(relocalization_gate_failure_config_path);
    config_out << "relocalization_map_bounds_margin_m: 10.0\n";
    config_out << "relocalization_min_inliers: 1000000\n";
    config_out << "relocalization_max_pos_cov_trace: 100.0\n";
    config_out << "track_against_map_require_degeneracy_metrics: false\n";
    config_out << "odom_prior_enabled: true\n";
    config_out << "max_update_velocity_mps: 3.0\n";
    config_out << "reject_nonconverged_update: false\n";
    config_out << "reject_degenerate_nonconverged_update: false\n";
    config_out << "degeneracy_max_update_dof: 5\n";
    config_out << "degeneracy_max_condition: 1000000000000.0\n";
    config_out << "max_update_translation_m: 10.0\n";
    config_out << "max_update_rotation_rad: 3.14\n";
    config_out << "max_update_velocity_delta_mps: 100.0\n";
  }

  checkOdomPriorBypass(bypass_config_path);
  checkOdomPriorHistoryBypass(bypass_config_path);
  checkOdomPriorBypassDoesNotFallback(bypass_config_path);
  checkPositionCovarianceHealthGate(covariance_gate_config_path);
  checkSensorTimeJumpGate(config_path);
  checkRejectedUpdateHealthGate(config_path);
  checkVelocityHealthGate(config_path);
  checkRelocalizationCannotClearUnhealthyNumericState(config_path);
  Pose3d result_failure_guess;
  result_failure_guess.x = 1000.0;
  result_failure_guess.y = 1000.0;
  result_failure_guess.z = 1000.0;
  checkFailedRelocalizationPreservesCatastrophicFault(
      config_path,
      result_failure_guess,
      "native_relocalizer_icp_failed",
      "failed",
      "result_failure");
  checkFailedRelocalizationPreservesCatastrophicFault(
      relocalization_gate_failure_config_path,
      Pose3d{},
      "relocalization_inliers_rejected",
      "rejected",
      "gate_failure");

  auto backend = makeFastLioBackend();
  check(backend != nullptr, "missing_backend");

  SlamConfig config;
  config.backend = "fastlio2";
  config.config_path = config_path.string();
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
  double imu_stamp_s = 0.61;
  for (int frame_index = 0; frame_index < 100; ++frame_index) {
    const double scan_stamp_s = 0.40 + static_cast<double>(frame_index) * 0.10;
    while (imu_stamp_s <= scan_stamp_s + 0.06) {
      check(backend->feedImu(imu(imu_stamp_s)).ok, "feed_static_imu_failed");
      imu_stamp_s += 0.005;
    }
    check(
        backend->feedLidar(lidar(scan_stamp_s, 0.0F)).ok,
        "feed_static_lidar_failed");
    for (int tick_index = 0; tick_index < 4; ++tick_index) {
      check(backend->tick().ok, "static_tick_failed");
    }
  }
  const auto static_outputs = backend->outputs();
  check(static_outputs.odometry_odom_body.has_value(), "static_odometry_missing");
  const auto& static_pose = *static_outputs.odometry_odom_body;
  const double static_position_norm =
      std::sqrt(static_pose.x * static_pose.x + static_pose.y * static_pose.y +
                static_pose.z * static_pose.z);
  check(std::isfinite(static_position_norm), "static_odometry_non_finite");
  if (static_position_norm > 0.25) {
    std::cerr << "static_scan_drift=" << static_position_norm << " pose="
              << static_pose.x << "," << static_pose.y << "," << static_pose.z
              << "\n";
    return 1;
  }

  const auto save_status = backend->saveMap((map_dir / "map.pcd").string());
  check(save_status.ok, "save_map_failed");
  check(std::filesystem::exists(map_dir / "map.pcd"), "map_pcd_missing");
  check(std::filesystem::exists(map_dir / "map.raw.pcd"), "raw_map_pcd_missing");
  check(
      std::filesystem::exists(map_dir / "map_optimization.json"),
      "map_optimization_metadata_missing");

  check(
      backend->setMode(SlamMode::Localization, (map_dir / "map.pcd").string()).ok,
      "localization_mode_failed");
  check(
      !backend->outputs().map_odom_tf.has_value(),
      "localization_must_not_publish_map_odom_before_relocalization");

  const auto async_start_time = std::chrono::steady_clock::now();
  check(
      backend->startRelocalizeAsync(static_pose).ok,
      "async_relocalization_start_failed");
  const double async_start_elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - async_start_time).count();
  check(async_start_elapsed_s < 0.20, "async_relocalization_start_blocked");
  check(backend->relocalizeAsyncInFlight(), "async_relocalization_not_in_flight");

  const double async_scan_stamp_s = 10.40;
  while (imu_stamp_s <= async_scan_stamp_s + 0.06) {
    check(backend->feedImu(imu(imu_stamp_s)).ok, "feed_async_imu_failed");
    imu_stamp_s += 0.005;
  }
  check(
      backend->feedLidar(lidar(async_scan_stamp_s, 0.0F)).ok,
      "feed_async_lidar_failed");
  const auto async_tick_start = std::chrono::steady_clock::now();
  check(backend->tick().ok, "async_tick_failed");
  const double async_tick_elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - async_tick_start).count();
  check(async_tick_elapsed_s < 0.50, "async_relocalization_blocked_tick");

  std::optional<Status> async_completion;
  const auto async_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < async_deadline) {
    async_completion = backend->pollRelocalizeAsync();
    if (async_completion.has_value()) {
      break;
    }
    check(backend->tick().ok, "tick_while_async_relocalizing_failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(async_completion.has_value(), "async_relocalization_timeout");
  if (!async_completion->ok) {
    std::cerr << "async_relocalization_failed=" << async_completion->message << "\n";
    return 1;
  }
  const auto localized_outputs = backend->outputs();
  check(localized_outputs.map_odom_tf.has_value(), "async_map_odom_missing");
  check(localized_outputs.map_frame_jump, "async_map_frame_jump_missing");
  check(
      localized_outputs.map_frame_jump_sequence > 0U,
      "async_map_frame_jump_sequence_missing");

  check(backend->relocalize(static_pose).ok, "manual_relocalization_regressed");

  std::cout << "fastlio2_mock_flow odometry stamp=" << outputs.stamp_s
            << " points=" << outputs.registered_cloud_body->points.size() << "\n";
  return 0;
}
