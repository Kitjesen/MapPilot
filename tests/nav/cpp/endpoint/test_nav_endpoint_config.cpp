#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "runtime/config/config.hpp"
#include "planning/rolling/segment.hpp"

namespace {

using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::GlobalPlannerBackend;
using lingtu::nav::endpoint::buildExecutorConfig;
using lingtu::nav::endpoint::buildLocalPlannerParams;
using lingtu::nav::endpoint::buildStatusWriterConfig;
using lingtu::nav::endpoint::commandSafetyConfig;
using lingtu::nav::endpoint::inputGateConfig;
using lingtu::nav::endpoint::parseArgs;
using lingtu::nav::endpoint::rollingSegmentExecutorConfig;

class ScopedEnvironment {
 public:
  ScopedEnvironment(std::string name, const std::string &value) : name_(std::move(name)) {
    if (const char *current = std::getenv(name_.c_str()); current != nullptr) {
      previous_ = std::string(current);
    }
    set(value);
  }

  ~ScopedEnvironment() noexcept {
    if (previous_.has_value()) {
      set(*previous_);
    } else {
      clear();
    }
  }

  ScopedEnvironment(const ScopedEnvironment &) = delete;
  ScopedEnvironment &operator=(const ScopedEnvironment &) = delete;

 private:
  void set(const std::string &value) noexcept {
#ifdef _WIN32
    (void)_putenv_s(name_.c_str(), value.c_str());
#else
    (void)setenv(name_.c_str(), value.c_str(), 1);
#endif
  }

  void clear() noexcept {
#ifdef _WIN32
    (void)_putenv_s(name_.c_str(), "");
#else
    (void)unsetenv(name_.c_str());
#endif
  }

  std::string name_;
  std::optional<std::string> previous_;
};

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

CliConfig parse(std::vector<std::string> args) {
  std::vector<char *> argv;
  argv.reserve(args.size());
  for (auto &arg : args) {
    argv.push_back(arg.data());
  }
  return parseArgs(static_cast<int>(argv.size()), argv.data());
}

void testNavigationRateAndAcceleration() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--tick-hz",
      "20",
      "--max-speed-mps",
      "0.6",
      "--max-accel-mps2",
      "1.5",
      "--corridor-lookahead-m",
      "1.2",
  });
  require(std::abs(cfg.tick_hz - 20.0) < 1e-12, "tick_hz must parse");
  require(std::abs(1.0 / cfg.tick_hz - 0.05) < 1e-12, "20 Hz must be 50 ms");
  require(std::abs(cfg.nav_max_speed_mps - 0.6) < 1e-12, "max speed must parse");
  require(std::abs(cfg.nav_max_accel_mps2 - 1.5) < 1e-12, "max accel must parse");
  require(std::abs(cfg.corridor_lookahead_m - 1.2) < 1e-12, "corridor lookahead must parse");
}

void testCompiledProductMotionContractParses() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--product",
      "nav",
      "--waypoint-reached-m",
      "0.20",
      "--goal-reached-m",
      "0.10",
      "--path-follower-goal-tolerance-m",
      "0.05",
      "--path-follower-lookahead-m",
      "0.35",
      "--path-follower-max-speed-mps",
      "0.20",
      "--path-follower-min-speed-mps",
      "0.08",
      "--path-follower-max-accel-mps2",
      "1.0",
      "--path-follower-max-yaw-rate-rad-s",
      "0.70",
      "--path-follower-heading-align-enter-rad",
      "0.80",
      "--path-follower-heading-align-exit-rad",
      "0.30",
      "--recovery-order",
      "rotate,translate",
      "--recovery-blocked-interval-s",
      "1.25",
      "--recovery-rotation-timeout-s",
      "2.75",
      "--recovery-translation-timeout-s",
      "1.75",
      "--recovery-max-attempts",
      "4",
      "--recovery-translation-speed-mps",
      "0.12",
      "--recovery-rotation-rate-rad-s",
      "0.40",
      "--recovery-min-rotation-rad",
      "0.25",
      "--recovery-max-rotation-rad",
      "1.10",
      "--recovery-rotation-candidate-step-rad",
      "0.15",
      "--recovery-rotation-sample-step-rad",
      "0.05",
  });
  require(cfg.product == "nav", "compiled product identity must parse");
  require(std::abs(cfg.waypoint_reached_m - 0.20) < 1e-12, "waypoint threshold must parse");
  require(std::abs(cfg.goal_reached_m - 0.10) < 1e-12, "goal threshold must parse");
  require(std::abs(cfg.path_follower_goal_tolerance_m - 0.05) < 1e-12,
          "path follower goal tolerance must parse");
  require(std::abs(cfg.path_follower_lookahead_m - 0.35) < 1e-12,
          "path follower lookahead must parse");
  require(std::abs(cfg.path_follower_max_speed_mps - 0.20) < 1e-12,
          "path follower maximum speed must parse");
  require(std::abs(cfg.path_follower_min_speed_mps - 0.08) < 1e-12,
          "path follower minimum speed must parse");
  require(std::abs(cfg.path_follower_max_accel_mps2 - 1.0) < 1e-12,
          "path follower acceleration must parse");
  require(std::abs(cfg.path_follower_max_yaw_rate_rad_s - 0.70) < 1e-12,
          "path follower yaw-rate limit must parse in rad/s");
  require(std::abs(cfg.path_follower_heading_align_enter_rad - 0.80) < 1e-12 &&
              std::abs(cfg.path_follower_heading_align_exit_rad - 0.30) < 1e-12,
          "path follower heading-alignment hysteresis must parse");
  const auto executor_config = buildExecutorConfig(cfg);
  require(std::abs(executor_config.follower.maxYawRateRadS - 0.70) < 1e-12 &&
              std::abs(executor_config.follower.headingAlignEnterRad - 0.40) < 1e-12 &&
              std::abs(executor_config.follower.headingAlignExitRad - 0.40) < 1e-12 &&
              std::abs(executor_config.follower.baseLookAheadDis - 0.50) < 1e-12 &&
              std::abs(executor_config.follower.maxAccel - 2.0) < 1e-12,
          "CMU config must apply the official Go2 follower profile");
  const auto status = buildStatusWriterConfig(cfg, inputGateConfig(cfg));
  require(std::abs(status.follower_lookahead_m - 0.50) < 1e-12 &&
              std::abs(status.max_accel_mps2 - 2.0) < 1e-12 &&
              std::abs(status.follower_goal_tolerance_m - 0.30) < 1e-12 &&
              std::abs(status.nominal_dt_s - 0.01) < 1e-12,
          "CMU status must report the effective Go2 follower profile");
  require(executor_config.recovery.behavior_order.size() == 2U &&
              executor_config.recovery.behavior_order[0] ==
                  nav_kernel::RecoveryAction::Rotate &&
              executor_config.recovery.behavior_order[1] ==
                  nav_kernel::RecoveryAction::Translate,
          "compiled ExecutorConfig must retain recovery action order");
  require(std::abs(executor_config.recovery.blocked_interval_s - 1.25) < 1e-12 &&
              std::abs(executor_config.recovery.rotation_timeout_s - 2.75) < 1e-12 &&
              std::abs(executor_config.recovery.translation_timeout_s - 1.75) < 1e-12 &&
              executor_config.recovery.max_attempts == 4 &&
              std::abs(executor_config.recovery.translation_speed_mps - 0.12) < 1e-12 &&
              std::abs(executor_config.recovery.rotation_rate_rad_s - 0.40) < 1e-12 &&
              std::abs(executor_config.recovery.min_rotation_rad - 0.25) < 1e-12 &&
              std::abs(executor_config.recovery.max_rotation_rad - 1.10) < 1e-12 &&
              std::abs(executor_config.recovery.rotation_candidate_step_rad - 0.15) < 1e-12 &&
              std::abs(executor_config.recovery.rotation_sample_step_rad - 0.05) < 1e-12,
          "compiled ExecutorConfig must retain bounded recovery motion settings");
}

void testDynamicObstacleThresholdsParse() {
  ScopedEnvironment min_cells("LINGTU_NAV_DYNAMIC_MIN_CELLS", "5");
  ScopedEnvironment min_speed("LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS", "0.18");
  ScopedEnvironment confirm_frames("LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES", "3");

  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});

  require(cfg.dynamic_min_cells == 5U, "dynamic obstacle minimum cells must parse");
  require(std::abs(cfg.dynamic_min_speed_mps - 0.18) < 1e-12,
          "dynamic obstacle minimum speed must parse");
  require(cfg.dynamic_confirm_frames == 3U,
          "dynamic obstacle confirmation frames must parse");
}

void testLegacyNativeProfileSelectorIsRejected() {
  bool rejected = false;
  try {
    (void)parse({"navd", "--path-library", "fixture-paths", "--profile", "nav"});
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "legacy --profile selector must be rejected");
}
void testCompiledProductMotionContractRejectsMinimumAboveMaximum() {
  bool rejected = false;
  try {
    (void)parse({"navd", "--path-library", "fixture-paths", "--path-follower-max-speed-mps", "0.2",
                 "--path-follower-min-speed-mps", "0.3"});
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "minimum native path follower speed must not exceed maximum");
}

void testRecoveryConfigurationRejectsInvalidOrderAndAngles() {
  bool duplicate_rejected = false;
  try {
    (void)parse({"navd", "--path-library", "fixture-paths", "--recovery-order",
                 "rotate,rotate"});
  } catch (const std::runtime_error &) {
    duplicate_rejected = true;
  }
  require(duplicate_rejected, "recovery order must not repeat an action");

  bool angle_range_rejected = false;
  try {
    (void)parse({"navd", "--path-library", "fixture-paths", "--recovery-min-rotation-rad",
                 "1.0", "--recovery-max-rotation-rad", "0.5"});
  } catch (const std::runtime_error &) {
    angle_range_rejected = true;
  }
  require(angle_range_rejected, "recovery minimum angle must not exceed maximum");
}

void testUnsafeNegativeValuesClampToZero() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--tick-hz",
      "0",
      "--max-speed-mps",
      "-0.6",
      "--max-accel-mps2",
      "-1.5",
  });
  require(cfg.tick_hz == 1.0, "tick_hz must clamp to at least 1 Hz");
  require(cfg.nav_max_speed_mps == 0.0, "negative max speed must clamp to zero");
  require(cfg.nav_max_accel_mps2 == 0.0, "negative max accel must clamp to zero");
}

void testInputFutureToleranceIsProfileConfigurable() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--input-future-tolerance-s",
      "0.4",
  });
  require(std::abs(cfg.input_future_tolerance_s - 0.4) < 1e-12,
          "input future tolerance must parse");
}

void testEstopLatchFileIsConfigurable() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--estop-latch-file",
      "/tmp/lingtu-estop-latched",
  });
  require(cfg.estop_latch_file == "/tmp/lingtu-estop-latched", "estop latch path must parse");
}

void testPlannerArtifactCarriesRuntimeMapIdentity() {
  ScopedEnvironment planner_map("OCTOPLANNER_MAP_PATH",
                                "/tmp/lingtu/maps/active/octomap.ot");
  ScopedEnvironment map_id("LINGTU_MAP_ID", "active");
  ScopedEnvironment map_epoch("LINGTU_MAP_CONTENT_EPOCH", "7");
  ScopedEnvironment map_frame("LINGTU_MAP_FRAME", "map");
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(cfg.map_path == "/tmp/lingtu/maps/active/octomap.ot",
          "configured OctoMap artifact path must remain explicit");
  require(cfg.map_identity.map_id == "active" && cfg.map_identity.content_epoch == 7 &&
              cfg.map_identity.frame_id == "map",
          "planner artifact must carry the Product-bound map identity");
}

void testOctoPlanner3DPhysicalContractIsConfigurable() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--octo-robot-radius-m",
      "0.25",
      "--octo-body-clearance-below-m",
      "0.18",
      "--octo-body-clearance-above-m",
      "0.30",
      "--octo-max-iterations",
      "240000",
      "--octo-snap-radius-cells",
      "16",
      "--octo-require-ground-support",
      "true",
      "--octo-strict-ground-support",
      "false",
      "--octo-ground-support-xy-radius-cells",
      "1",
      "--octo-ground-support-depth-cells",
      "8",
      "--octo-support-height-m",
      "0.55",
      "--octo-support-height-tolerance-m",
      "0.05",
      "--octo-support-patch-radius-cells",
      "0",
      "--octo-support-patch-min-samples",
      "0",
      "--octo-enable-preblocked-costmap",
      "false",
      "--octo-preblocked-radius-cells",
      "0",
      "--octo-preblocked-weight",
      "0",
      "--octo-lowest-traversable-only",
      "false",
      "--octo-floor-change-penalty",
      "4",
      "--octo-max-step-height-m",
      "0.23",
      "--octo-max-slope",
      "0.65",
      "--octo-same-floor-preference",
      "true",
      "--octo-same-floor-z-tolerance-m",
      "0.75",
      "--octo-max-same-floor-z-excursion-m",
      "2.0",
      "--octo-obstacle-clearance-radius-cells",
      "0",
      "--octo-obstacle-clearance-weight",
      "0",
      "--octo-terminal-goal-tolerance-m",
      "0.35",
      "--octo-terminal-goal-xy-tolerance-m",
      "0.35",
      "--octo-terminal-goal-z-tolerance-m",
      "0.18",
  });
  const auto &octo = cfg.octoplanner_options;
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

void testFarPlannerIsAnExplicitOptionalBackend() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(defaults.global_planner == GlobalPlannerBackend::OctoPlanner3D,
          "OctoPlanner3D must remain the product default");

  ScopedEnvironment map_id("LINGTU_MAP_ID", "active");
  ScopedEnvironment map_epoch("LINGTU_MAP_CONTENT_EPOCH", "7");
  ScopedEnvironment map_frame("LINGTU_MAP_FRAME", "map");
  ScopedEnvironment octomap("OCTOPLANNER_MAP_PATH",
                            "/tmp/lingtu/maps/active/octomap.ot");
  ScopedEnvironment occupancy("FAR_OCCUPANCY_PATH",
                              "/tmp/lingtu/maps/active/occupancy.npz");
  const auto far = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--global-planner",
      "far",
      "--far-robot-radius-m",
      "0.28",
      "--far-obstacle-clearance-m",
      "0.12",
      "--far-max-visibility-distance-m",
      "24.0",
      "--far-unknown-cost-multiplier",
      "8.0",
      "--far-corner-separation-cells",
      "2",
      "--far-snap-radius-cells",
      "9",
      "--far-max-graph-nodes",
      "2048",
      "--far-max-visibility-pairs",
      "120000",
      "--far-max-search-expansions",
      "180000",
      "--far-simplify-path",
      "false",
  });
  require(far.global_planner == GlobalPlannerBackend::Far, "FAR backend must parse");
  require(far.map_path == "/tmp/lingtu/maps/active/occupancy.npz",
          "FAR must receive an explicit occupancy artifact");
  require(std::abs(far.far_options.robot_radius_m - 0.28) < 1e-12, "FAR radius must parse");
  require(std::abs(far.far_options.obstacle_clearance_m - 0.12) < 1e-12,
          "FAR obstacle clearance must parse");
  require(std::abs(far.far_options.max_visibility_distance_m - 24.0) < 1e-12,
          "FAR visibility limit must parse");
  require(far.far_options.corner_separation_cells == 2, "FAR corner spacing must parse");
  require(far.far_options.snap_search_radius_cells == 9, "FAR snap radius must parse");
  require(far.far_options.max_graph_nodes == 2048U, "FAR graph limit must parse");
  require(!far.far_options.allow_unknown_fallback,
          "Product FAR endpoint must never search unknown space");
  require(!far.far_options.simplify_path, "FAR path simplification must be configurable");
}

void testFarPlannerRejectsInvalidConfiguration() {
  bool backend_rejected = false;
  try {
    (void)parse({
        "navd",
        "--path-library",
        "fixture-paths",
        "--global-planner",
        "unsupported",
    });
  } catch (const std::runtime_error &) {
    backend_rejected = true;
  }
  require(backend_rejected, "unknown global planner backend must fail startup");

  bool unknown_fallback_rejected = false;
  try {
    (void)parse({
        "navd",
        "--path-library",
        "fixture-paths",
        "--global-planner",
        "far",
        "--far-allow-unknown-fallback",
        "true",
    });
  } catch (const std::runtime_error &) {
    unknown_fallback_rejected = true;
  }
  require(unknown_fallback_rejected,
          "Product endpoint must reject FAR unknown fallback configuration");

  bool limit_rejected = false;
  try {
    (void)parse({
        "navd",
        "--path-library",
        "fixture-paths",
        "--global-planner",
        "far",
        "--far-max-graph-nodes",
        "1",
    });
  } catch (const std::runtime_error &) {
    limit_rejected = true;
  }
  require(limit_rejected, "unsafe FAR graph limits must fail startup");
}

void testNonFiniteMotionLimitsFailClosed() {
  bool rejected = false;
  try {
    (void)parse({
        "navd",
        "--path-library",
        "fixture-paths",
        "--max-accel-mps2",
        "inf",
    });
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "non-finite max accel must be rejected");

  bool planner_rejected = false;
  try {
    (void)parse({
        "navd",
        "--path-library",
        "fixture-paths",
        "--teleop-planner-max-deviation-deg",
        "nan",
    });
  } catch (const std::runtime_error &) {
    planner_rejected = true;
  }
  require(planner_rejected, "non-finite teleop planner limits must be rejected");
}

void testSafetyThresholdOrderingFailsClosed() {
  bool distance_rejected = false;
  try {
    (void)parse({
        "navd",
        "--control-mode",
        "teleop_avoid",
        "--teleop-slow-distance-m",
        "0.5",
        "--teleop-stop-distance-m",
        "0.8",
    });
  } catch (const std::runtime_error &) {
    distance_rejected = true;
  }
  require(distance_rejected, "stop distance beyond slow distance must be rejected");

  bool cost_rejected = false;
  try {
    (void)parse({
        "navd",
        "--control-mode",
        "teleop_avoid",
        "--teleop-traversability-soft-cost",
        "90",
        "--teleop-traversability-hard-cost",
        "80",
    });
  } catch (const std::runtime_error &) {
    cost_rejected = true;
  }
  require(cost_rejected, "soft terrain cost beyond hard cost must be rejected");

  const auto distinct_costs = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--control-mode",
      "teleop_avoid",
      "--teleop-local-planner",
      "true",
      "--traversability-hard-cost",
      "90",
      "--teleop-traversability-hard-cost",
      "80",
  });
  require(std::abs(distinct_costs.traversability_hard_cost - 90.0) < 1e-12 &&
              std::abs(distinct_costs.teleop_traversability_hard_cost - 80.0) < 1e-12,
          "planned-path and direct-command terrain policies must remain independent");
}

void testControlModesLockTheirSafetyContract() {
  const auto teleop = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--control-mode",
      "teleop",
      "--check-obstacle",
      "true",
      "--use-traversability-cost",
      "true",
  });
  require(teleop.control_mode == ControlMode::Teleop, "teleop mode must parse");
  require(!teleop.check_obstacle, "pure teleop must not depend on obstacle input");
  require(!teleop.use_traversability_cost, "pure teleop must not depend on terrain input");

  const auto teleop_avoid = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--control-mode",
      "teleop_avoid",
      "--check-obstacle",
      "false",
      "--use-traversability-cost",
      "false",
  });
  require(teleop_avoid.control_mode == ControlMode::TeleopAvoid, "teleop_avoid mode must parse");
  require(teleop_avoid.check_obstacle, "teleop_avoid must enforce obstacle checks");
  require(!teleop_avoid.use_traversability_cost,
          "teleop_avoid must allow CMU scan-only operation without traversability");

  const auto teleop_gate = inputGateConfig(teleop);
  require(!teleop_gate.require_odom, "pure teleop must not require odometry");
  require(!teleop_gate.require_cloud, "pure teleop must not require obstacle cloud");
  require(!teleop_gate.require_traversability, "pure teleop must not require traversability");
  require(!teleop_gate.require_localization_health,
          "pure teleop must not require localization health");
  require(teleop_gate.require_driver_control,
          "all hardware motion modes must require LingTu's Brainstem lease");
  require(buildStatusWriterConfig(teleop, teleop_gate).stop_confirmation_evidence == "driver_ack",
          "pure teleop status must disclose Brainstem ACK-only stop confirmation");

  const auto avoid_gate = inputGateConfig(teleop_avoid);
  require(avoid_gate.require_odom, "teleop_avoid needs pose for spatial obstacle safety");
  require(avoid_gate.require_cloud, "teleop_avoid must require obstacle cloud");
  require(!avoid_gate.require_traversability,
          "scan-only teleop_avoid must not wait for traversability");
  require(buildStatusWriterConfig(teleop_avoid, avoid_gate).stop_confirmation_evidence ==
              "driver_ack",
          "operator-assisted control must not block on quiet odometry after driver ACK");
  require(!avoid_gate.require_localization_health,
          "teleop_avoid must use odometry freshness without a second localization-health hard gate");

  const auto autonomy = parse({
      "navd",          "--path-library",
      "fixture-paths", "--control-mode",
      "autonomy",      "--allow-teleop-takeover",
      "true",          "--teleop-local-planner",
      "true",          "--teleop-planner-horizon-m",
      "2.4",           "--teleop-planner-max-deviation-deg",
      "48.0",          "--check-obstacle",
      "false",         "--use-traversability-cost",
      "false",         "--localization-health-max-age-s",
      "0.4",           "--driver-control-max-age-s",
      "0.3",
  });
  require(autonomy.allow_teleop_takeover,
          "autonomy must parse the explicit operator takeover gate");
  require(autonomy.teleop_local_planner, "autonomy must parse assisted operator local planning");
  require(std::abs(autonomy.teleop_planner_horizon_m - 2.4) < 1e-12,
          "teleop planner horizon must parse");
  require(std::abs(autonomy.teleop_planner_max_deviation_deg - 48.0) < 1e-12,
          "teleop planner deviation must parse");
  const auto autonomy_gate = inputGateConfig(autonomy);
  require(autonomy_gate.require_odom, "autonomy must require odometry");
  require(!autonomy_gate.require_cloud, "disabled autonomy obstacle input must remain optional");
  require(!autonomy_gate.require_traversability,
          "disabled autonomy traversability must remain optional");
  require(autonomy_gate.require_localization_health, "autonomy must require healthy localization");
  require(autonomy_gate.require_driver_control,
          "autonomy must require fresh lingtu-driver control readiness");
  require(std::abs(autonomy_gate.localization_health_max_age_s - 0.4) < 1e-12,
          "localization health max age must parse into the gate contract");
  require(std::abs(autonomy_gate.driver_control_max_age_s - 0.3) < 1e-12,
          "driver control max age must parse into the gate contract");
}

void testPureTeleopDoesNotRequirePlannerAssets() {
  const auto teleop = parse({
      "navd",
      "--control-mode",
      "teleop",
      "--path-library",
      "",
  });
  require(teleop.path_library_dir.empty(),
          "pure teleop must start without a local planner path library");
  const auto avoid = parse({
      "navd",
      "--control-mode",
      "teleop_avoid",
      "--path-library",
      "",
  });
  require(avoid.path_library_dir.empty(), "stop-only teleop_avoid must not require planner assets");

  bool assisted_rejected = false;
  try {
    (void)parse({
        "navd",
        "--control-mode",
        "teleop_avoid",
        "--teleop-local-planner",
        "true",
        "--path-library",
        "",
    });
  } catch (const std::runtime_error &) {
    assisted_rejected = true;
  }
  require(assisted_rejected, "assisted teleop must require local planner assets");

  bool autonomy_rejected = false;
  try {
    (void)parse({
        "navd",
        "--control-mode",
        "autonomy",
        "--path-library",
        "",
    });
  } catch (const std::runtime_error &) {
    autonomy_rejected = true;
  }
  require(autonomy_rejected, "autonomy must still require planner assets");
}

void testLocalPlannerVisualizationIsExplicitAndBounded() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(defaults.local_planner_debug_candidate_limit == 0,
          "candidate debug snapshots must be disabled by default");
  require(defaults.local_map_debug_point_limit == 0,
          "local-map debug snapshots must be disabled by default");

  const auto enabled = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--local-planner-debug-candidates",
      "99",
      "--local-map-debug-points",
      "640",
  });
  require(enabled.local_planner_debug_candidate_limit == 36,
          "candidate debug snapshots must clamp to the 36 library rotations");
  require(enabled.local_map_debug_point_limit == 640, "local-map debug point limit must parse");
}

void testLocalPlannerScoringThreadsAreExplicitAndBounded() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(defaults.local_planner_threads == 2,
          "local planner must use a bounded two-thread scoring team by default");

  const auto configured = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--local-planner-threads",
      "3",
  });
  require(configured.local_planner_threads == 3, "local planner thread count must parse");

  const auto clamped = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--local-planner-threads",
      "99",
  });
  require(clamped.local_planner_threads == 4,
          "local planner thread count must remain bounded under invalid product input");
}

void testStopConfirmationTimeoutIsExplicitAndFailClosed() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(std::abs(defaults.stop_confirmation_timeout_s - 4.0) < 1e-12,
          "stop confirmation must allow a bounded SLAM publication gap by default");

  const auto configured = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--stop-confirmation-timeout-s",
      "5.5",
  });
  require(std::abs(configured.stop_confirmation_timeout_s - 5.5) < 1e-12,
          "stop confirmation timeout must parse from the product command");

  for (const char *value : {"0.49", "30.01", "nan", "inf"}) {
    bool rejected = false;
    try {
      (void)parse({
          "navd",
          "--path-library",
          "fixture-paths",
          "--stop-confirmation-timeout-s",
          value,
      });
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "unsafe stop confirmation timeouts must fail closed");
  }
}

void testLocalPlannerOverheadHeightLimitIsConfigurable() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(std::abs(defaults.local_planner_obstacle_height_max_m - 1.2) < 1e-12,
          "local planner must ignore points above the default body envelope");

  const auto configured = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--local-planner-obstacle-height-max-m",
      "1.65",
  });
  require(std::abs(configured.local_planner_obstacle_height_max_m - 1.65) < 1e-12,
          "local planner relative obstacle height maximum must parse");
}

void testRuntimeEnvironmentPrecedesDefaultsAndCliPrecedesEnvironment() {
  ScopedEnvironment domain("LINGTU_DDS_DOMAIN_ID", "23");
  ScopedEnvironment tick("LINGTU_NAV_DDS_TICK_HZ", "17.5");
  ScopedEnvironment status("LINGTU_NAV_STATUS_S", "0.25");

  const auto from_env = parse({"navd", "--path-library", "fixture-paths"});
  require(from_env.domain_id == 23, "DDS domain must parse from the runtime environment");
  require(std::abs(from_env.tick_hz - 17.5) < 1e-12,
          "navigation tick rate must parse from the runtime environment");
  require(std::abs(from_env.status_s - 0.25) < 1e-12,
          "status period must parse from the runtime environment");

  const auto from_cli = parse({"navd", "--path-library", "fixture-paths", "--domain-id", "24",
                               "--tick-hz", "30", "--status-s", "0.4"});
  require(from_cli.domain_id == 24, "CLI DDS domain must override the environment");
  require(std::abs(from_cli.tick_hz - 30.0) < 1e-12,
          "CLI navigation tick rate must override the environment");
  require(std::abs(from_cli.status_s - 0.4) < 1e-12,
          "CLI status period must override the environment");
}

void testGo2RunPlanGeometryEnvironmentParsesExactly() {
  ScopedEnvironment length("LINGTU_NAV_VEHICLE_LENGTH_M", "0.76");
  ScopedEnvironment width("LINGTU_NAV_VEHICLE_WIDTH_M", "0.31");
  ScopedEnvironment hard_margin("LINGTU_TELEOP_OBSTACLE_MARGIN_M", "0.10");
  ScopedEnvironment cylinder_radius("LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M", "0.25");
  ScopedEnvironment cylinder_offset("LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M", "0.18");
  ScopedEnvironment follower_speed("LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS", "0.50");
  ScopedEnvironment planning_range("LINGTU_NAV_CORRIDOR_LOOKAHEAD_M", "3.0");
  ScopedEnvironment obstacle_voxel("LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M", "0.05");

  const auto cmu_cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(std::abs(cmu_cfg.vehicle_length_m - 0.76) < 1e-12 &&
              std::abs(cmu_cfg.vehicle_width_m - 0.31) < 1e-12,
          "Go2 physical dimensions must parse from the compiled RunPlan environment");
  require(std::abs(cmu_cfg.collision_cylinder_radius_m - 0.25) < 1e-12 &&
              std::abs(cmu_cfg.collision_cylinder_offset_m - 0.18) < 1e-12,
          "Go2 double-cylinder geometry must parse from the compiled RunPlan environment");
  require(std::abs(cmu_cfg.obstacle_voxel_size_m - 0.05) < 1e-12,
          "Go2 CMU obstacle fusion must retain the upstream five-centimeter density");
  const auto cmu = buildLocalPlannerParams(cmu_cfg);
  require(std::abs(cmu.footprintPadding - 0.10) < 1e-12,
          "Go2 CMU must receive the one configured hard margin");
  require(std::abs(cmu.autonomySpeed - 0.50) < 1e-12 &&
              std::abs(cmu.maxSpeed - 0.50) < 1e-12,
          "CMU speed scaling must normalize against the configured follower speed");
  require(std::abs(cmu.adjacentRange - 3.0) < 1e-12,
          "CMU planning range must follow the compiled local corridor range");
  require(cmu.useTerrainAnalysis,
          "CMU must use the official terrain-height obstacle semantics");
  require(!cmu.twoWayDrive,
          "autonomous CMU planning must match the forward-only Go2 follower contract");
  require(std::abs(cmu.nearFieldStopDis) < 1e-12,
          "CMU candidate collision checks must not be followed by a duplicate near-field stop");

  ScopedEnvironment planner_backend("LINGTU_NAV_LOCAL_PLANNER_BACKEND", "scan");
  const auto scan_cfg = parse({"navd"});
  const auto scan = buildLocalPlannerParams(scan_cfg);
  require(std::abs(scan.scan.cylinderOffset - 0.18) < 1e-12 &&
              std::abs(scan.footprintPadding) < 1e-12,
          "Go2 SCAN must use the complete double-cylinder envelope without extra padding");
  require(!scan.useTerrainAnalysis,
          "SCAN collision input must not inherit the CMU terrain-cloud interpretation");
}

void testScanFollowerEnvironmentReachesExecutor() {
  ScopedEnvironment time_forward("LINGTU_NAV_SCAN_TIME_FORWARD_S", "0.65");
  ScopedEnvironment heading_error("LINGTU_NAV_SCAN_HEADING_ERROR_RAD", "0.70");
  ScopedEnvironment position_gain("LINGTU_NAV_SCAN_POSITION_GAIN", "0.90");
  ScopedEnvironment yaw_gain("LINGTU_NAV_SCAN_YAW_GAIN", "1.40");
  ScopedEnvironment max_vx("LINGTU_NAV_SCAN_MAX_VX_MPS", "0.68");
  ScopedEnvironment max_vy("LINGTU_NAV_SCAN_MAX_VY_MPS", "0.32");
  ScopedEnvironment max_yaw("LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S", "0.95");
  ScopedEnvironment shared_finish_distance(
      "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M", "0.07");

  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  const auto executor_config = buildExecutorConfig(cfg);
  require(std::abs(executor_config.follower.spline.timeForward - 0.65) < 1e-12,
          "SCAN look-ahead time must reach Executor");
  require(std::abs(executor_config.follower.spline.headingErrorThreshold - 0.70) < 1e-12,
          "SCAN heading gate must reach Executor");
  require(std::abs(executor_config.follower.spline.positionGain - 0.90) < 1e-12 &&
              std::abs(executor_config.follower.spline.yawGain - 1.40) < 1e-12,
          "SCAN feedback gains must reach Executor");
  require(std::abs(executor_config.follower.spline.maxVx - 0.68) < 1e-12 &&
              std::abs(executor_config.follower.spline.maxVy - 0.32) < 1e-12 &&
              std::abs(executor_config.follower.spline.maxYawRateRadS - 0.95) < 1e-12,
          "SCAN body velocity limits must reach Executor");
  require(std::abs(executor_config.follower.spline.finishDistance - 0.07) < 1e-12,
          "CMU and SCAN must share the local tracking finish distance");
}

void testControlLoopDeadlineMissRatioIsExplicitAndBounded() {
  const auto defaults = parse({
      "navd",
      "--path-library",
      "fixture-paths",
  });
  require(std::abs(defaults.control_loop_deadline_miss_ratio_limit - 0.05) < 1e-12,
          "field deadline-miss ratio limit must remain five percent by default");
  require(std::abs(defaults.control_loop_p95_utilization_limit - 0.90) < 1e-12,
          "field P95 utilization limit must remain ninety percent by default");

  const auto configured = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--control-loop-deadline-miss-ratio-limit",
      "0.10",
      "--control-loop-p95-utilization-limit",
      "1.50",
  });
  require(std::abs(configured.control_loop_deadline_miss_ratio_limit - 0.10) < 1e-12,
          "non-real-time simulation must be able to declare its measured jitter budget");
  require(std::abs(configured.control_loop_p95_utilization_limit - 1.50) < 1e-12,
          "non-real-time simulation must be able to declare its P95 work budget");

  for (const char *value : {"-0.01", "1.01", "nan", "inf"}) {
    bool rejected = false;
    try {
      (void)parse({
          "navd",
          "--path-library",
          "fixture-paths",
          "--control-loop-deadline-miss-ratio-limit",
          value,
      });
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid deadline-miss ratio limit must fail closed");
  }

  for (const char *value : {"0", "-0.01", "nan", "inf"}) {
    bool rejected = false;
    try {
      (void)parse({
          "navd",
          "--path-library",
          "fixture-paths",
          "--control-loop-p95-utilization-limit",
          value,
      });
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid P95 utilization limit must fail closed");
  }
}

void testScanIsAnExplicitSecondLocalBackend() {
  const auto defaults = parse({
      "navd", "--path-library", "fixture-paths"});
  require(
      defaults.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu,
      "CMU must remain the default local backend");
  const auto scan = parse({"navd", "--local-planner", "scan"});
  require(
      scan.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan,
      "SCAN backend must parse explicitly");
  require(scan.path_library_dir.empty(),
          "SCAN startup must not require or synthesize a CMU path library");
  require(
      buildLocalPlannerParams(scan).backend ==
          nav_kernel::LocalPlannerBackend::Scan,
      "endpoint config must forward the selected local backend");

  bool rejected = false;
  try {
    (void)parse({
        "navd", "--path-library", "fixture-paths", "--local-planner", "unknown"});
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "unknown local planner backend must fail startup");
}

void testScanDoesNotDuplicateLiveObstacleInflation() {
  CliConfig cfg;
  cfg.teleop_obstacle_margin_m = 0.15;
  cfg.live_obstacle_inflation_radius_m = 0.12;
  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
  const auto scan = buildLocalPlannerParams(cfg);
  require(std::abs(scan.footprintPadding) < 1e-12,
          "SCAN cylinder geometry already owns the complete hard envelope");

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Cmu;
  const auto cmu = buildLocalPlannerParams(cfg);
  require(std::abs(cmu.footprintPadding - 0.15) < 1e-12,
          "CMU must use exactly one configured hard margin");
}

void testLocalPlannerUsesConfiguredTraversabilityPolicy() {
  CliConfig cfg;
  cfg.use_traversability_cost = true;

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Cmu;
  const auto cmu = buildLocalPlannerParams(cfg);
  require(cmu.useTraversabilityCost,
          "CMU must consume the configured planner traversability policy");

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
  const auto scan = buildLocalPlannerParams(cfg);
  require(scan.useTraversabilityCost,
          "SCAN must consume the configured planner traversability policy");

  cfg.use_traversability_cost = false;
  const auto disabled = buildLocalPlannerParams(cfg);
  require(!disabled.useTraversabilityCost,
          "local planning must not synthesize a traversability dependency when it is disabled");
}

void testCommandBoundaryUsesActiveControlModeMotionLimits() {
  CliConfig cfg;
  cfg.path_follower_max_speed_mps = 0.6;
  cfg.path_follower_max_yaw_rate_rad_s = 0.8;
  cfg.teleop_max_speed_mps = 0.4;
  cfg.teleop_max_yaw_rate = 1.0;

  cfg.control_mode = ControlMode::Autonomy;
  auto safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - 0.6) < 1e-12,
          "autonomy command boundary must preserve the configured follower speed limit");
  require(std::abs(safety.max_yaw_rate - 0.8) < 1e-12,
          "autonomy command boundary must preserve the configured follower yaw limit");

  cfg.control_mode = ControlMode::Teleop;
  safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - 0.4) < 1e-12,
          "teleop command boundary must use the teleop speed limit");
  require(std::abs(safety.max_yaw_rate - 1.0) < 1e-12,
          "teleop command boundary must use the teleop yaw limit");

  cfg.control_mode = ControlMode::TeleopAvoid;
  safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - 0.4) < 1e-12,
          "teleop_avoid command boundary must use the teleop speed limit");
  require(std::abs(safety.max_yaw_rate - 1.0) < 1e-12,
          "teleop_avoid command boundary must use the teleop yaw limit");
}

void testScanCollisionEnvelopeIsConfigurable() {
  const auto cfg = parse({
      "navd",
      "--local-planner",
      "scan",
      "--collision-cylinder-radius-m",
      "0.41",
      "--collision-cylinder-offset-m",
      "0.24",
      "--collision-clearance-below-m",
      "0.22",
      "--collision-clearance-above-m",
      "0.36",
  });
  const auto planner = buildLocalPlannerParams(cfg);
  require(std::abs(cfg.collision_cylinder_radius_m - 0.41) < 1e-12,
          "SCAN collision radius must remain available to the Mapd profile");
  require(std::abs(planner.scan.cylinderOffset - 0.24) < 1e-12,
          "SCAN collision offset must reach the planner");
  require(std::abs(planner.scan.bodyClearanceBelow - 0.22) < 1e-12,
          "SCAN lower body clearance must reach the planner");
  require(std::abs(planner.scan.bodyClearanceAbove - 0.36) < 1e-12,
          "SCAN upper body clearance must reach the planner");
}

void testVelocitySmootherEnvironmentParsesExactly() {
  ScopedEnvironment enabled("LINGTU_NAV_SMOOTHER_ENABLED", "true");
  ScopedEnvironment feedback_mode("LINGTU_NAV_SMOOTHER_FEEDBACK_MODE", "open");
  ScopedEnvironment target_timeout("LINGTU_NAV_SMOOTHER_TARGET_TIMEOUT_S", "0.4");
  ScopedEnvironment feedback_timeout("LINGTU_NAV_SMOOTHER_FEEDBACK_TIMEOUT_S", "0.2");
  ScopedEnvironment max_step("LINGTU_NAV_SMOOTHER_MAX_STEP_S", "0.1");
  ScopedEnvironment future_tolerance("LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S", "0.03");
  ScopedEnvironment scale("LINGTU_NAV_SMOOTHER_SCALE_VELOCITIES", "true");
  ScopedEnvironment x_min("LINGTU_NAV_SMOOTHER_X_MIN_MPS", "-0.4");
  ScopedEnvironment x_max("LINGTU_NAV_SMOOTHER_X_MAX_MPS", "0.4");
  ScopedEnvironment x_accel("LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2", "0.8");
  ScopedEnvironment x_decel("LINGTU_NAV_SMOOTHER_X_DECEL_MPS2", "1.2");
  ScopedEnvironment x_deadband("LINGTU_NAV_SMOOTHER_X_DEADBAND_MPS", "0.01");
  ScopedEnvironment y_min("LINGTU_NAV_SMOOTHER_Y_MIN_MPS", "-0.2");
  ScopedEnvironment y_max("LINGTU_NAV_SMOOTHER_Y_MAX_MPS", "0.2");
  ScopedEnvironment y_accel("LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2", "0.9");
  ScopedEnvironment y_decel("LINGTU_NAV_SMOOTHER_Y_DECEL_MPS2", "1.3");
  ScopedEnvironment y_deadband("LINGTU_NAV_SMOOTHER_Y_DEADBAND_MPS", "0.02");
  ScopedEnvironment yaw_min("LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS", "-0.8");
  ScopedEnvironment yaw_max("LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS", "0.8");
  ScopedEnvironment yaw_accel("LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2", "1.5");
  ScopedEnvironment yaw_decel("LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2", "2.5");
  ScopedEnvironment yaw_deadband("LINGTU_NAV_SMOOTHER_YAW_DEADBAND_RADPS", "0.03");

  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(cfg.velocity_smoother_enabled, "velocity smoother enabled flag must parse");
  require(cfg.velocity_smoother.feedback_mode == nav_kernel::VelocityFeedbackMode::kOpenLoop,
          "velocity smoother feedback mode must parse");
  require(std::abs(cfg.velocity_smoother.target_timeout_s - 0.4) < 1e-12,
          "velocity smoother target timeout must parse");
  require(std::abs(cfg.velocity_smoother.feedback_timeout_s - 0.2) < 1e-12,
          "velocity smoother feedback timeout must parse");
  require(std::abs(cfg.velocity_smoother.max_step_s - 0.1) < 1e-12,
          "velocity smoother maximum step must parse");
  require(std::abs(cfg.velocity_smoother.future_tolerance_s - 0.03) < 1e-12,
          "velocity smoother future tolerance must parse");
  require(cfg.velocity_smoother.scale_velocities,
          "velocity smoother scaling flag must parse");
  require(std::abs(cfg.velocity_smoother.x.minimum + 0.4) < 1e-12 &&
              std::abs(cfg.velocity_smoother.x.maximum - 0.4) < 1e-12 &&
              std::abs(cfg.velocity_smoother.x.acceleration - 0.8) < 1e-12 &&
              std::abs(cfg.velocity_smoother.x.deceleration - 1.2) < 1e-12 &&
              std::abs(cfg.velocity_smoother.x.deadband - 0.01) < 1e-12,
          "velocity smoother x-axis limits must parse");
  require(std::abs(cfg.velocity_smoother.y.minimum + 0.2) < 1e-12 &&
              std::abs(cfg.velocity_smoother.y.maximum - 0.2) < 1e-12 &&
              std::abs(cfg.velocity_smoother.y.acceleration - 0.9) < 1e-12 &&
              std::abs(cfg.velocity_smoother.y.deceleration - 1.3) < 1e-12 &&
              std::abs(cfg.velocity_smoother.y.deadband - 0.02) < 1e-12,
          "velocity smoother y-axis limits must parse");
  require(std::abs(cfg.velocity_smoother.yaw.minimum + 0.8) < 1e-12 &&
              std::abs(cfg.velocity_smoother.yaw.maximum - 0.8) < 1e-12 &&
              std::abs(cfg.velocity_smoother.yaw.acceleration - 1.5) < 1e-12 &&
              std::abs(cfg.velocity_smoother.yaw.deceleration - 2.5) < 1e-12 &&
              std::abs(cfg.velocity_smoother.yaw.deadband - 0.03) < 1e-12,
          "velocity smoother yaw-axis limits must parse");
}

void testVelocitySmootherRejectsUnavailableClosedLoopOnlyWhenEnabled() {
  ScopedEnvironment feedback_mode("LINGTU_NAV_SMOOTHER_FEEDBACK_MODE", "closed");
  {
    ScopedEnvironment enabled("LINGTU_NAV_SMOOTHER_ENABLED", "true");
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "enabled closed-loop smoothing must fail startup without feedback wiring");
  }
  ScopedEnvironment disabled("LINGTU_NAV_SMOOTHER_ENABLED", "false");
  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(!cfg.velocity_smoother_enabled,
          "disabled closed-loop smoother config must preserve legacy runtime behavior");
  require(cfg.velocity_smoother.feedback_mode == nav_kernel::VelocityFeedbackMode::kClosedLoop,
          "disabled closed-loop mode must still parse deterministically");
}

void testVelocitySmootherConfigurationFailsClosed() {
  for (const auto &invalid : std::vector<std::pair<std::string, std::string>>{
           {"LINGTU_NAV_SMOOTHER_FEEDBACK_MODE", "pid"},
           {"LINGTU_NAV_SMOOTHER_X_MIN_MPS", "0.1"},
           {"LINGTU_NAV_SMOOTHER_Y_MAX_MPS", "-0.1"},
           {"LINGTU_NAV_SMOOTHER_X_MIN_MPS", "nan"},
           {"LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S", "inf"},
           {"LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S", "-0.01"},
       }) {
    ScopedEnvironment value(invalid.first, invalid.second);
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid velocity smoother configuration must fail startup");
  }
}

void testEnvironmentNumbersRejectTrailingJunk() {
  for (const auto &invalid : std::vector<std::pair<std::string, std::string>>{
           {"LINGTU_NAV_MAX_SPEED_MPS", "0.5junk"},
           {"LINGTU_NAV_LIVE_OBSTACLE_MIN_HITS", "1junk"},
           {"LINGTU_NAV_SEGMENT_MAX_WAYPOINTS", "4junk"},
           {"LINGTU_NAV_SMOOTHER_TARGET_TIMEOUT_S", "0.5junk"},
       }) {
    ScopedEnvironment value(invalid.first, invalid.second);
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "environment numbers with trailing junk must be rejected");
  }

  ScopedEnvironment padded("LINGTU_NAV_MAX_SPEED_MPS", " 0.5 \t");
  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(std::abs(cfg.nav_max_speed_mps - 0.5) < 1e-12,
          "environment numbers may contain surrounding whitespace");
}

void testVelocitySmootherAcceptsCoreBoundaryValues() {
  ScopedEnvironment future_tolerance("LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S", "0");
  ScopedEnvironment x_min("LINGTU_NAV_SMOOTHER_X_MIN_MPS", "-0.8");
  ScopedEnvironment x_max("LINGTU_NAV_SMOOTHER_X_MAX_MPS", "0.2");
  ScopedEnvironment x_deadband("LINGTU_NAV_SMOOTHER_X_DEADBAND_MPS", "0.8");
  ScopedEnvironment y_min("LINGTU_NAV_SMOOTHER_Y_MIN_MPS", "0");
  ScopedEnvironment y_max("LINGTU_NAV_SMOOTHER_Y_MAX_MPS", "0");
  ScopedEnvironment y_deadband("LINGTU_NAV_SMOOTHER_Y_DEADBAND_MPS", "0");

  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(cfg.velocity_smoother.future_tolerance_s == 0.0,
          "zero future tolerance must remain valid");
  require(cfg.velocity_smoother.x.deadband == 0.8,
          "deadband may equal the largest configured speed magnitude");
  require(cfg.velocity_smoother.y.minimum == 0.0 &&
              cfg.velocity_smoother.y.maximum == 0.0,
          "an axis may be explicitly disabled with a zero-only range");
}

void testSimulationRunPlanIdentityDerivesRuntimePaths() {
  const std::string product_session_id = "product-session-test";
  const auto session_root =
      (std::filesystem::current_path() / "nav-endpoint-session").lexically_normal();
  ScopedEnvironment runtime_env("LINGTU_ENV", "sim");
  ScopedEnvironment product_session_environment(
      "LINGTU_PRODUCT_SESSION_ID", product_session_id);
  ScopedEnvironment product("LINGTU_PRODUCT", "teleop");
  ScopedEnvironment session("LINGTU_SESSION_ROOT", session_root.string());
  ScopedEnvironment status("LINGTU_NAV_STATUS_FILE", "");
  ScopedEnvironment inspection("LINGTU_INSPECTION_DIR", "");

  const auto cfg = parse({"navd", "--control-mode", "teleop"});
  require(cfg.product_session_id == product_session_id,
          "simulation navd must preserve the Product session id");
  require(std::filesystem::path(cfg.status_file) == session_root / "nav.status.json",
          "simulation navd must publish status inside the Product session root");
  require(std::filesystem::path(cfg.inspection_dir) == session_root / "inspection",
          "simulation navd must keep inspection data inside the Product session root");
  const auto status_cfg = buildStatusWriterConfig(cfg, inputGateConfig(cfg));
  require(status_cfg.product_session_id == product_session_id,
          "status writer must receive the Product session id");
}

void testSimulationRunPlanIdentityFailsClosed() {
  const auto session_root =
      (std::filesystem::current_path() / "nav-endpoint-session").lexically_normal();
  ScopedEnvironment runtime_env("LINGTU_ENV", "sim");
  ScopedEnvironment product("LINGTU_PRODUCT", "teleop");
  ScopedEnvironment session("LINGTU_SESSION_ROOT", session_root.string());
  ScopedEnvironment status("LINGTU_NAV_STATUS_FILE", "");

  ScopedEnvironment product_session_environment("LINGTU_PRODUCT_SESSION_ID", "");
  bool rejected = false;
  try {
    (void)parse({"navd", "--control-mode", "teleop"});
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "simulation navd must require a Product session id");
}

void testLocalPlannerUsesConfiguredObstacleEnvelope() {
  CliConfig cfg;
  cfg.teleop_obstacle_height_min_m = 0.10;
  cfg.teleop_obstacle_height_max_m = 1.40;
  cfg.local_planner_obstacle_height_max_m = 1.20;

  const auto planner = buildLocalPlannerParams(cfg);
  require(std::abs(planner.obstacleHeightThre - 0.10) < 1e-12,
          "local planner must use the configured obstacle minimum");
  require(std::abs(planner.obstacleHeightMax - 1.40) < 1e-12,
          "local planner must use the configured obstacle maximum");
}

void testDriverControlFreshnessCannotBeDisabled() {
  for (const char *value : {"0", "-1", "nan", "inf"}) {
    bool rejected = false;
    try {
      (void)parse({
          "navd",
          "--path-library",
          "fixture-paths",
          "--driver-control-max-age-s",
          value,
      });
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "driver control freshness must remain finite and strictly positive");
  }
}

void testRollingSegmentPoliciesAreConfigurableAndBounded() {
  const auto cfg = parse({
      "navd",
      "--path-library",
      "fixture-paths",
      "--segment-max-distance-m",
      "6.5",
      "--segment-max-waypoints",
      "48",
      "--segment-max-grid-cells",
      "524288",
      "--segment-risk-stop",
      "60",
      "--segment-risk-resume",
      "45",
      "--segment-map-max-age-s",
      "0.8",
  });
  const auto policy = rollingSegmentExecutorConfig(cfg);
  require(std::abs(policy.segment.max_distance_m - 6.5) < 1e-12,
          "segment maximum distance must parse");
  require(policy.segment.max_waypoints == 48U, "segment waypoint limit must parse");
  require(policy.map_input.max_grid_cells == 524288U, "segment grid limit must parse");
  require(std::abs(policy.risk.stop_threshold - 60.0F) < 1e-6F,
          "segment risk stop threshold must parse");
  require(std::abs(policy.risk.resume_threshold - 45.0F) < 1e-6F,
          "segment risk resume threshold must parse");
  require(std::abs(policy.map_input.max_age_s - 0.8) < 1e-12,
          "segment map maximum age must parse");

  for (const std::vector<std::string> &args : {
           std::vector<std::string>{"navd", "--path-library", "fixture-paths",
                                    "--segment-risk-stop", "39", "--segment-risk-resume", "40"},
           std::vector<std::string>{"navd", "--path-library", "fixture-paths",
                                    "--segment-max-distance-m", "0"},
           std::vector<std::string>{"navd", "--path-library", "fixture-paths",
                                    "--segment-max-waypoints", "1"},
           std::vector<std::string>{"navd", "--path-library", "fixture-paths",
                                    "--segment-max-grid-cells", "1048577"},
           std::vector<std::string>{"navd", "--path-library", "fixture-paths",
                                    "--segment-map-max-age-s", "10.1"},
       }) {
    bool rejected = false;
    try {
      (void)parse(args);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid rolling segment policy must fail closed");
  }
}

}  // namespace

int main() {
  try {
    ScopedEnvironment runtime_env("LINGTU_ENV", "");
    testNavigationRateAndAcceleration();
    testDynamicObstacleThresholdsParse();
    testRuntimeEnvironmentPrecedesDefaultsAndCliPrecedesEnvironment();
    testGo2RunPlanGeometryEnvironmentParsesExactly();
    testScanFollowerEnvironmentReachesExecutor();
    testCompiledProductMotionContractParses();
    testVelocitySmootherEnvironmentParsesExactly();
    testVelocitySmootherRejectsUnavailableClosedLoopOnlyWhenEnabled();
    testVelocitySmootherConfigurationFailsClosed();
    testEnvironmentNumbersRejectTrailingJunk();
    testVelocitySmootherAcceptsCoreBoundaryValues();
    testSimulationRunPlanIdentityDerivesRuntimePaths();
    testSimulationRunPlanIdentityFailsClosed();
    testLegacyNativeProfileSelectorIsRejected();
    testCompiledProductMotionContractRejectsMinimumAboveMaximum();
    testRecoveryConfigurationRejectsInvalidOrderAndAngles();
    testUnsafeNegativeValuesClampToZero();
    testSafetyThresholdOrderingFailsClosed();
    testInputFutureToleranceIsProfileConfigurable();
    testEstopLatchFileIsConfigurable();
    testPlannerArtifactCarriesRuntimeMapIdentity();
    testOctoPlanner3DPhysicalContractIsConfigurable();
    testFarPlannerIsAnExplicitOptionalBackend();
    testScanIsAnExplicitSecondLocalBackend();
    testScanDoesNotDuplicateLiveObstacleInflation();
    testLocalPlannerUsesConfiguredTraversabilityPolicy();
    testCommandBoundaryUsesActiveControlModeMotionLimits();
    testScanCollisionEnvelopeIsConfigurable();
    testFarPlannerRejectsInvalidConfiguration();
    testNonFiniteMotionLimitsFailClosed();
    testControlModesLockTheirSafetyContract();
    testPureTeleopDoesNotRequirePlannerAssets();
    testLocalPlannerVisualizationIsExplicitAndBounded();
    testLocalPlannerScoringThreadsAreExplicitAndBounded();
    testControlLoopDeadlineMissRatioIsExplicitAndBounded();
    testStopConfirmationTimeoutIsExplicitAndFailClosed();
    testLocalPlannerOverheadHeightLimitIsConfigurable();
    testLocalPlannerUsesConfiguredObstacleEnvelope();
    testDriverControlFreshnessCannotBeDisabled();
    testRollingSegmentPoliciesAreConfigurableAndBounded();
    std::puts("test_nav_endpoint_config: PASS");
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_nav_endpoint_config: FAIL: %s\n", exc.what());
    return 1;
  }
}
