#include "nav_endpoint_config.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::inputGateConfig;
using lingtu::nav::endpoint::parseArgs;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

CliConfig parse(std::vector<std::string> args) {
  std::vector<char*> argv;
  argv.reserve(args.size());
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }
  return parseArgs(static_cast<int>(argv.size()), argv.data());
}

void testNavigationRateAndAcceleration() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--tick-hz", "20",
      "--max-speed-mps", "0.6",
      "--max-accel-mps2", "1.5",
      "--corridor-lookahead-m", "1.2",
  });
  require(std::abs(cfg.tick_hz - 20.0) < 1e-12, "tick_hz must parse");
  require(std::abs(1.0 / cfg.tick_hz - 0.05) < 1e-12, "20 Hz must be 50 ms");
  require(std::abs(cfg.nav_max_speed_mps - 0.6) < 1e-12, "max speed must parse");
  require(std::abs(cfg.nav_max_accel_mps2 - 1.5) < 1e-12, "max accel must parse");
  require(
      std::abs(cfg.corridor_lookahead_m - 1.2) < 1e-12,
      "corridor lookahead must parse");
}

void testUnsafeNegativeValuesClampToZero() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--tick-hz", "0",
      "--max-speed-mps", "-0.6",
      "--max-accel-mps2", "-1.5",
  });
  require(cfg.tick_hz == 1.0, "tick_hz must clamp to at least 1 Hz");
  require(cfg.nav_max_speed_mps == 0.0, "negative max speed must clamp to zero");
  require(cfg.nav_max_accel_mps2 == 0.0, "negative max accel must clamp to zero");
}

void testInputFutureToleranceIsProfileConfigurable() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--input-future-tolerance-s", "0.4",
  });
  require(
      std::abs(cfg.input_future_tolerance_s - 0.4) < 1e-12,
      "input future tolerance must parse");
}

void testEstopLatchFileIsConfigurable() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--estop-latch-file", "/tmp/lingtu-estop-latched",
  });
  require(
      cfg.estop_latch_file == "/tmp/lingtu-estop-latched",
      "estop latch path must parse");
}

void testMapRootAndArtifactPathAreSeparateInputs() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--map-root", "/tmp/lingtu/maps",
      "--map", "/tmp/lingtu/maps/active/octomap.ot",
  });
  require(cfg.map_root == "/tmp/lingtu/maps", "native Maps root must parse separately");
  require(
      cfg.map_path == "/tmp/lingtu/maps/active/octomap.ot",
      "configured OctoMap artifact path must remain explicit");
}

void testOctoPlanner3DPhysicalContractIsConfigurable() {
  const auto cfg = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--octo-robot-radius-m", "0.25",
      "--octo-body-clearance-below-m", "0.18",
      "--octo-body-clearance-above-m", "0.30",
      "--octo-max-iterations", "240000",
      "--octo-snap-radius-cells", "16",
      "--octo-require-ground-support", "true",
      "--octo-strict-ground-support", "false",
      "--octo-ground-support-xy-radius-cells", "1",
      "--octo-ground-support-depth-cells", "8",
      "--octo-support-height-m", "0.55",
      "--octo-support-height-tolerance-m", "0.05",
      "--octo-support-patch-radius-cells", "0",
      "--octo-support-patch-min-samples", "0",
      "--octo-enable-preblocked-costmap", "false",
      "--octo-preblocked-radius-cells", "0",
      "--octo-preblocked-weight", "0",
      "--octo-lowest-traversable-only", "false",
      "--octo-floor-change-penalty", "4",
      "--octo-max-step-height-m", "0.23",
      "--octo-max-slope", "0.65",
      "--octo-same-floor-preference", "true",
      "--octo-same-floor-z-tolerance-m", "0.75",
      "--octo-max-same-floor-z-excursion-m", "2.0",
      "--octo-obstacle-clearance-radius-cells", "0",
      "--octo-obstacle-clearance-weight", "0",
      "--octo-terminal-goal-tolerance-m", "0.35",
      "--octo-terminal-goal-xy-tolerance-m", "0.35",
      "--octo-terminal-goal-z-tolerance-m", "0.18",
  });
  const auto& octo = cfg.octoplanner_options;
  require(std::abs(octo.robot_radius - 0.25) < 1e-12, "OctoPlanner3D radius must parse");
  require(std::abs(octo.body_clearance_below_m - 0.18) < 1e-12, "body lower envelope must parse");
  require(std::abs(octo.body_clearance_above_m - 0.30) < 1e-12, "body upper envelope must parse");
  require(octo.max_iterations == 240000, "A* budget must parse");
  require(octo.snap_search_radius_cells == 16, "snap radius must parse");
  require(octo.require_ground_support, "ground support must stay enabled");
  require(!octo.strict_direct_ground_support, "nearby support must remain allowed");
  require(octo.ground_support_xy_radius_cells == 1, "support XY radius must parse");
  require(octo.ground_support_depth_cells == 8, "support depth must parse");
  require(std::abs(octo.support_height_m - 0.55) < 1e-12, "support height must parse");
  require(std::abs(octo.support_height_tolerance_m - 0.05) < 1e-12, "support tolerance must parse");
  require(!octo.enable_preblocked_costmap, "preblocked soft cost must be configurable");
  require(!octo.lowest_traversable_only, "stacked floors must keep every support layer");
  require(std::abs(octo.max_step_height - 0.23) < 1e-12, "23 cm step limit must parse");
  require(std::abs(octo.max_slope - 0.65) < 1e-12, "slope limit must parse");
  require(octo.obstacle_clearance_radius_cells == 0, "clearance radius must parse");
  require(std::abs(octo.terminal_goal_z_tolerance_m - 0.18) < 1e-12, "goal Z tolerance must parse");
}

void testNonFiniteMotionLimitsFailClosed() {
  bool rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--path-library", "fixture-paths",
        "--max-accel-mps2", "inf",
    });
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "non-finite max accel must be rejected");

  bool planner_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--path-library", "fixture-paths",
        "--teleop-planner-max-deviation-deg", "nan",
    });
  } catch (const std::runtime_error&) {
    planner_rejected = true;
  }
  require(planner_rejected, "non-finite teleop planner limits must be rejected");
}

void testSafetyThresholdOrderingFailsClosed() {
  bool distance_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--control-mode", "teleop_avoid",
        "--teleop-slow-distance-m", "0.5",
        "--teleop-stop-distance-m", "0.8",
    });
  } catch (const std::runtime_error&) {
    distance_rejected = true;
  }
  require(distance_rejected, "stop distance beyond slow distance must be rejected");

  bool cost_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--control-mode", "teleop_avoid",
        "--teleop-traversability-soft-cost", "90",
        "--teleop-traversability-hard-cost", "80",
    });
  } catch (const std::runtime_error&) {
    cost_rejected = true;
  }
  require(cost_rejected, "soft terrain cost beyond hard cost must be rejected");

  bool assisted_cost_mismatch_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--path-library", "fixture-paths",
        "--control-mode", "teleop_avoid",
        "--teleop-local-planner", "true",
        "--traversability-hard-cost", "90",
        "--teleop-traversability-hard-cost", "80",
    });
  } catch (const std::runtime_error&) {
    assisted_cost_mismatch_rejected = true;
  }
  require(
      assisted_cost_mismatch_rejected,
      "assisted planner and final safety hard terrain costs must match");
}

void testControlModesLockTheirSafetyContract() {
  const auto teleop = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--control-mode", "teleop",
      "--check-obstacle", "true",
      "--use-traversability-cost", "true",
  });
  require(teleop.control_mode == ControlMode::Teleop, "teleop mode must parse");
  require(!teleop.check_obstacle, "pure teleop must not depend on obstacle input");
  require(!teleop.use_traversability_cost, "pure teleop must not depend on terrain input");

  const auto teleop_avoid = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--control-mode", "teleop_avoid",
      "--check-obstacle", "false",
      "--use-traversability-cost", "false",
  });
  require(
      teleop_avoid.control_mode == ControlMode::TeleopAvoid,
      "teleop_avoid mode must parse");
  require(teleop_avoid.check_obstacle, "teleop_avoid must enforce obstacle checks");
  require(
      teleop_avoid.use_traversability_cost,
      "teleop_avoid must enforce traversability checks");

  const auto teleop_gate = inputGateConfig(teleop);
  require(!teleop_gate.require_odom, "pure teleop must not require odometry");
  require(!teleop_gate.require_cloud, "pure teleop must not require obstacle cloud");
  require(!teleop_gate.require_traversability, "pure teleop must not require traversability");
  require(!teleop_gate.require_localization_health, "pure teleop must not require localization health");
  require(
      teleop_gate.require_driver_control,
      "all hardware motion modes must require LingTu's Brainstem lease");

  const auto avoid_gate = inputGateConfig(teleop_avoid);
  require(avoid_gate.require_odom, "teleop_avoid needs pose for spatial obstacle safety");
  require(avoid_gate.require_cloud, "teleop_avoid must require obstacle cloud");
  require(avoid_gate.require_traversability, "teleop_avoid must require traversability");
  require(avoid_gate.require_localization_health, "teleop_avoid must require healthy localization");

  const auto autonomy = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--control-mode", "autonomy",
      "--allow-teleop-takeover", "true",
      "--teleop-local-planner", "true",
      "--teleop-planner-horizon-m", "2.4",
      "--teleop-planner-max-deviation-deg", "48.0",
      "--check-obstacle", "false",
      "--use-traversability-cost", "false",
      "--localization-health-max-age-s", "0.4",
      "--driver-control-max-age-s", "0.3",
  });
  require(
      autonomy.allow_teleop_takeover,
      "autonomy must parse the explicit operator takeover gate");
  require(
      autonomy.teleop_local_planner,
      "autonomy must parse assisted operator local planning");
  require(
      std::abs(autonomy.teleop_planner_horizon_m - 2.4) < 1e-12,
      "teleop planner horizon must parse");
  require(
      std::abs(autonomy.teleop_planner_max_deviation_deg - 48.0) < 1e-12,
      "teleop planner deviation must parse");
  const auto autonomy_gate = inputGateConfig(autonomy);
  require(autonomy_gate.require_odom, "autonomy must require odometry");
  require(!autonomy_gate.require_cloud, "disabled autonomy obstacle input must remain optional");
  require(!autonomy_gate.require_traversability, "disabled autonomy traversability must remain optional");
  require(autonomy_gate.require_localization_health, "autonomy must require healthy localization");
  require(
      autonomy_gate.require_driver_control,
      "autonomy must require fresh lingtu-driver control readiness");
  require(
      std::abs(autonomy_gate.localization_health_max_age_s - 0.4) < 1e-12,
      "localization health max age must parse into the gate contract");
  require(
      std::abs(autonomy_gate.driver_control_max_age_s - 0.3) < 1e-12,
      "driver control max age must parse into the gate contract");
}

void testPureTeleopDoesNotRequirePlannerAssets() {
  const auto teleop = parse({
      "lingtu_nav_native_endpoint",
      "--control-mode", "teleop",
      "--path-library", "",
  });
  require(
      teleop.path_library_dir.empty(),
      "pure teleop must start without a local planner path library");
  const auto avoid = parse({
      "lingtu_nav_native_endpoint",
      "--control-mode", "teleop_avoid",
      "--path-library", "",
  });
  require(
      avoid.path_library_dir.empty(),
      "stop-only teleop_avoid must not require planner assets");

  bool assisted_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--control-mode", "teleop_avoid",
        "--teleop-local-planner", "true",
        "--path-library", "",
    });
  } catch (const std::runtime_error&) {
    assisted_rejected = true;
  }
  require(
      assisted_rejected,
      "assisted teleop must require local planner assets");

  bool autonomy_rejected = false;
  try {
    (void)parse({
        "lingtu_nav_native_endpoint",
        "--control-mode", "autonomy",
        "--path-library", "",
    });
  } catch (const std::runtime_error&) {
    autonomy_rejected = true;
  }
  require(autonomy_rejected, "autonomy must still require planner assets");
}

void testLegacyMotionInputsAreExplicitCompatibilityOnly() {
  const auto production = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
  });
  require(
      !production.allow_legacy_motion_inputs,
      "typed command request must be the production motion boundary");
  const auto compatibility = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--allow-legacy-motion-inputs", "true",
  });
  require(
      compatibility.allow_legacy_motion_inputs,
      "legacy motion readers must require an explicit compatibility flag");
}

void testLocalPlannerVisualizationIsExplicitAndBounded() {
  const auto defaults = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
  });
  require(
      defaults.local_planner_debug_candidate_limit == 0,
      "candidate debug snapshots must be disabled by default");
  require(
      defaults.local_map_debug_point_limit == 0,
      "local-map debug snapshots must be disabled by default");

  const auto enabled = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--local-planner-debug-candidates", "99",
      "--local-map-debug-points", "640",
  });
  require(
      enabled.local_planner_debug_candidate_limit == 36,
      "candidate debug snapshots must clamp to the 36 library rotations");
  require(
      enabled.local_map_debug_point_limit == 640,
      "local-map debug point limit must parse");
}

void testLocalPlannerOverheadHeightLimitIsConfigurable() {
  const auto defaults = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
  });
  require(
      std::abs(defaults.local_planner_obstacle_height_max_m - 1.2) < 1e-12,
      "local planner must ignore points above the default body envelope");

  const auto configured = parse({
      "lingtu_nav_native_endpoint",
      "--path-library", "fixture-paths",
      "--local-planner-obstacle-height-max-m", "1.65",
  });
  require(
      std::abs(configured.local_planner_obstacle_height_max_m - 1.65) < 1e-12,
      "local planner relative obstacle height maximum must parse");
}

void testDriverControlFreshnessCannotBeDisabled() {
  for (const std::string& value : {"0", "-1", "nan", "inf"}) {
    bool rejected = false;
    try {
      (void)parse({
          "lingtu_nav_native_endpoint",
          "--path-library", "fixture-paths",
          "--driver-control-max-age-s", value,
      });
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    require(
        rejected,
        "driver control freshness must remain finite and strictly positive");
  }
}

}  // namespace

int main() {
  try {
    testNavigationRateAndAcceleration();
    testUnsafeNegativeValuesClampToZero();
  testSafetyThresholdOrderingFailsClosed();
  testInputFutureToleranceIsProfileConfigurable();
  testEstopLatchFileIsConfigurable();
    testMapRootAndArtifactPathAreSeparateInputs();
    testOctoPlanner3DPhysicalContractIsConfigurable();
    testNonFiniteMotionLimitsFailClosed();
    testControlModesLockTheirSafetyContract();
    testPureTeleopDoesNotRequirePlannerAssets();
    testLegacyMotionInputsAreExplicitCompatibilityOnly();
    testLocalPlannerVisualizationIsExplicitAndBounded();
    testLocalPlannerOverheadHeightLimitIsConfigurable();
    testDriverControlFreshnessCannotBeDisabled();
    std::puts("test_nav_endpoint_config: PASS");
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_nav_endpoint_config: FAIL: %s\n", exc.what());
    return 1;
  }
}
