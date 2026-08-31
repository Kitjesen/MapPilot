#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "config.hpp"
#include "core.hpp"
#include "status.hpp"

namespace {

void check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

void testPhysicalVelocityAndOdometryEvidence() {
  lingtu::driver::RuntimeStats stats;
  stats.body.fresh = true;
  stats.body.posture = lingtu::driver::Posture::Standing;
  stats.body.velocity = {0.25, -0.1, 0.4};
  stats.body.velocity_available = true;
  stats.body.odometry_position_m = {1.25, -0.5, 0.1};
  stats.body.odometry_position_available = true;
  stats.last_output_kind = "motion_command";
  lingtu::driver::recordVelocityTracking(stats, {0.3, 0.0, 0.0}, stats.body);

  lingtu::driver::Config config;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  config.status_file =
      "/tmp/lingtu_driver_status_test_" + std::to_string(suffix) + ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  const lingtu::driver::AdapterDiagnostics adapter{
      "go2", "unitree_sdk2", "dds://eth0/rt/api/sport/request",
      "none", "", false, 0};

  lingtu::driver::writeStatus(config, core, stats, adapter, 123.0);
  const std::string status = readText(config.status_file);

  check(status.find("\"odometry_position_available\": true") != std::string::npos,
        "status must expose odometry availability");
  check(status.find("\"x_m\": 1.25") != std::string::npos,
        "status must expose odometry position in metres");
  check(status.find("\"last_output_kind\": \"motion_command\"") !=
            std::string::npos,
        "status must distinguish motion from zero outputs");
  check(status.find("\"tracking_samples\": 1") != std::string::npos,
        "status must count command/observation comparisons");
  check(status.find("\"mean_commanded_linear_mps\": 0.3") !=
            std::string::npos,
        "status must retain commanded speed");
  check(status.find("\"mean_observed_linear_mps\": 0.269258") !=
            std::string::npos,
        "status must retain observed speed");

  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

void testBoundedMotionRunEvidence() {
  lingtu::driver::RuntimeStats stats;
  lingtu::driver::BodyState body;
  body.fresh = true;
  body.velocity_available = true;
  body.odometry_position_available = true;
  body.velocity = {0.10, 0.0, 0.0};
  body.odometry_position_m = {0.0, 0.0, 0.0};
  lingtu::driver::recordMotionOutput(
      stats, {0.3, 0.0, 0.0}, body, "motion_command", 100.0);

  body.velocity = {0.20, 0.0, 0.0};
  body.odometry_position_m = {0.30, 0.0, 0.0};
  lingtu::driver::recordMotionOutput(
      stats, {0.3, 0.0, 0.0}, body, "motion_command", 102.0);

  body.velocity = {};
  body.odometry_position_m = {0.42, 0.0, 0.0};
  lingtu::driver::recordMotionOutput(
      stats, {}, body, "commanded_zero", 103.0);
  stats.body = body;

  lingtu::driver::Config config;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  config.status_file =
      "/tmp/lingtu_driver_motion_run_test_" + std::to_string(suffix) + ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  const lingtu::driver::AdapterDiagnostics adapter{
      "go2", "unitree_sdk2", "dds://eth0/rt/api/sport/request",
      "none", "", false, 0};

  lingtu::driver::writeStatus(config, core, stats, adapter, 104.0);
  const std::string status = readText(config.status_file);

  check(status.find("\"last_completed_motion_run\": {\"available\": true") !=
            std::string::npos,
        "status must retain the last completed bounded motion run");
  check(status.find("\"duration_s\": 3") != std::string::npos,
        "status must expose bounded-run elapsed time");
  check(status.find("\"commanded_distance_m\": 0.9") != std::string::npos,
        "status must integrate the commanded physical distance");
  check(status.find("\"odometry_displacement_m\": 0.42") !=
            std::string::npos,
        "status must expose start/end odometry displacement");
  check(status.find("\"odometry_to_command_ratio\": 0.466667") !=
            std::string::npos,
        "status must quantify an under-speed run");
  check(status.find("\"end_output_kind\": \"commanded_zero\"") !=
            std::string::npos,
        "status must expose how the bounded run ended");

  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

void testEpochTimestampKeepsSubsecondPrecision() {
  lingtu::driver::RuntimeStats stats;
  lingtu::driver::Config config;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  config.status_file =
      "/tmp/lingtu_driver_timestamp_test_" + std::to_string(suffix) + ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  const lingtu::driver::AdapterDiagnostics adapter{
      "go2", "unitree_sdk2", "dds://eth0/rt/api/sport/request",
      "none", "", false, 0};

  lingtu::driver::writeStatus(config, core, stats, adapter, 1787560707.125);
  const std::string status = readText(config.status_file);

  check(status.find("\"stamp_s\": 1787560707.125000") != std::string::npos,
        "status must not quantize Unix epoch timestamps");

  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

}  // namespace

int main() {
  try {
    testPhysicalVelocityAndOdometryEvidence();
    testBoundedMotionRunEvidence();
    testEpochTimestampKeepsSubsecondPrecision();
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_driver_status: FAIL: %s\n", exc.what());
    return 1;
  }
}
