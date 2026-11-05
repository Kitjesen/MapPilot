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

void testOrdinaryDefaultAllowsOneMeterPerSecondAndStillLimitsExcess() {
  using namespace lingtu::nav::endpoint;
  const CommandSafetyConfig cfg;
  for (const auto request : {nav_kernel::Twist{1,0,0}, nav_kernel::Twist{0,1,0}}) {
    const auto decision=evaluateCommandSafety(cfg,request,0);
    require(!decision.stopped && !decision.limited && linearSpeed(decision.cmd) == 1.0,
            "ordinary command default must permit one meter per second on either axis");
  }
  const auto limited=evaluateCommandSafety(cfg,{2,0,0},0);
  require(limited.limited && linearSpeed(limited.cmd) == 1.0,
          "ordinary command default must still enforce its speed limit");
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

void testIndependentAxesPreserveDiagonalAndRespectTotalOverride() {
  using namespace lingtu::nav::endpoint;
  CommandSafetyConfig cfg;
  cfg.max_axis_speed_mps = 1.0;
  cfg.max_speed_mps = std::hypot(1.0, 1.0);
  for (const auto request : {nav_kernel::Twist{1.0, 1.0, 0.0},
                             nav_kernel::Twist{-1.0, -1.0, 0.0}}) {
    const auto decision = evaluateCommandSafety(cfg, request, 0.0);
    require(!decision.limited && !decision.stopped,
            "in-range diagonal must not be reduced to a single-axis speed");
    require(decision.cmd.vx == request.vx && decision.cmd.vy == request.vy,
            "both independent axes must survive the command boundary");
  }
  for (const auto request : {nav_kernel::Twist{2.0, 0.0, 0.0},
                             nav_kernel::Twist{0.0, -2.0, 0.0},
                             nav_kernel::Twist{2.0, -2.0, 0.0}}) {
    const auto decision = evaluateCommandSafety(cfg, request, 0.0);
    require(decision.limited && std::abs(decision.cmd.vx) <= 1.0 &&
                std::abs(decision.cmd.vy) <= 1.0,
            "higher diagonal budget must not raise either body-axis bound");
  }
  cfg.max_speed_mps = 0.6;
  const auto reduced = evaluateCommandSafety(cfg, {1.0, 1.0, 0.0}, 0.0);
  require(std::abs(linearSpeed(reduced.cmd) - 0.6) < 1e-9,
          "explicit smaller total speed must remain binding");
  const auto held = evaluateCommandSafety(cfg, {}, 0.0);
  require(linearSpeed(held.cmd) == 0.0, "zero input must remain zero");
}

void testDefaultScalarLimitStillBoundsDiagonal() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.max_speed_mps = 0.6;
  const auto decision =
      lingtu::nav::endpoint::evaluateCommandSafety(cfg, {1.0, 1.0, 0.0}, 0.0);
  require(std::abs(lingtu::nav::endpoint::linearSpeed(decision.cmd) - 0.6) < 1e-9,
          "existing scalar speed limits must still bound the vector norm");
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

void testDeadbandCannotSuppressTheWholeMotionBudget() {
  using namespace lingtu::nav::endpoint;
  CommandSafetyConfig cfg;
  cfg.max_axis_speed_mps = 0.005;
  cfg.max_speed_mps = std::hypot(0.005, 0.005);
  cfg.min_motion_speed_mps = 0.03;
  const auto slow = evaluateCommandSafety(cfg, {0.002, 0.0, 0.0}, 0.0);
  require(!slow.stopped && slow.cmd.vx == 0.002,
          "a smaller axis budget must preserve a legal slow command");
  const auto diagonal = evaluateCommandSafety(cfg, {0.02, -0.02, 0.0}, 0.0);
  require(diagonal.cmd.vx == 0.005 && diagonal.cmd.vy == -0.005,
          "physical body axes must still bound low-speed diagonal commands");
  cfg.max_axis_speed_mps = 1.0;
  cfg.max_speed_mps = 0.02;
  const auto low_norm = evaluateCommandSafety(cfg, {0.2, 0.0, 0.0}, 0.0);
  require(!low_norm.stopped && std::abs(low_norm.cmd.vx - 0.02) < 1e-9,
          "a smaller path budget must retain its capped nonzero command");
  require(cfg.min_motion_speed_mps == 0.03,
          "a per-command budget must preserve the configured deadband");
  cfg.min_motion_speed_mps = 0.001;
  require(evaluateCommandSafety(cfg, {0.0005, 0.0, 0.0}, 0.0).stopped,
          "a smaller configured deadband must still suppress creep");
}

}  // namespace

int main() {
  try {
    testDirectTeleopLimitsVelocityWithoutPoseOrMap();
    testOrdinaryDefaultAllowsOneMeterPerSecondAndStillLimitsExcess();
    testIndependentAxesPreserveDiagonalAndRespectTotalOverride();
    testDefaultScalarLimitStillBoundsDiagonal();
    testStaleCommandStops();
    testSubthresholdTranslationBecomesZero();
    testDeadbandCannotSuppressTheWholeMotionBudget();
  } catch (const std::exception &error) {
    std::cerr << "test_teleop_safety: FAIL: " << error.what() << '\n';
    return 1;
  }
  std::cout << "test_teleop_safety: PASS\n";
  return 0;
}
