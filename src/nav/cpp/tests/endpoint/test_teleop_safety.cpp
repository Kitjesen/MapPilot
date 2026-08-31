#include <cmath>
#include <iostream>
#include <stdexcept>

#include "safety/command.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testDirectTeleopLimitsVelocityWithoutPoseOrMap() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.max_speed_mps = 0.4;
  cfg.max_yaw_rate = 1.0;

  const auto decision =
      lingtu::nav::endpoint::evaluateCommandSafety(cfg, {1.0, 0.0, 2.0}, 0.0);

  require(decision.should_publish, "direct teleop must produce a command");
  require(!decision.stopped, "direct teleop must not require pose or map data");
  require(decision.limited, "configured velocity limits must remain active");
  require(std::abs(decision.cmd.vx - 0.4) < 1e-9, "linear speed must be clamped");
  require(std::abs(decision.cmd.wz - 1.0) < 1e-9, "yaw rate must be clamped");
}

void testStaleCommandStops() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.cmd_max_age_s = 0.35;

  const auto decision =
      lingtu::nav::endpoint::evaluateCommandSafety(cfg, {0.2, 0.0, 0.0}, 0.36);

  require(decision.stopped, "stale teleop command must stop");
  require(decision.reason == "stale", "stale reason must remain explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "stale command must publish zero translation");
}

void testSubthresholdTranslationBecomesZero() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.min_motion_speed_mps = 0.03;

  const auto decision =
      lingtu::nav::endpoint::evaluateCommandSafety(cfg, {0.02, 0.0, 0.0}, 0.0);

  require(decision.stopped, "subthreshold translation must be explicit zero");
  require(decision.reason == "below_min_motion", "deadband reason must remain explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "deadband must not leak nonzero translation");
}

}  // namespace

int main() {
  try {
    testDirectTeleopLimitsVelocityWithoutPoseOrMap();
    testStaleCommandStops();
    testSubthresholdTranslationBecomesZero();
  } catch (const std::exception &error) {
    std::cerr << "test_teleop_safety: FAIL: " << error.what() << '\n';
    return 1;
  }
  std::cout << "test_teleop_safety: PASS\n";
  return 0;
}
