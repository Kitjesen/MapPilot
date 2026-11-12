#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "app/config/config.hpp"
#include "planning/rolling/segment.hpp"
#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"

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
  require(std::abs(cfg.path_follower_max_speed_mps - 0.6) < 1e-12, "max speed must parse");
  require(std::abs(cfg.path_follower_max_accel_mps2 - 1.5) < 1e-12, "max accel must parse");
  require(std::abs(cfg.corridor_lookahead_m - 1.2) < 1e-12, "corridor lookahead must parse");
}

void testOrdinarySpeedDefaultsAgreeAcrossTheNativeBoundary() {
  ScopedEnvironment nav_speed("LINGTU_NAV_MAX_SPEED_MPS", "");
  ScopedEnvironment follower_speed("LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS", "");
  ScopedEnvironment planner_speed("LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS", "");
  ScopedEnvironment forward_speed("LINGTU_NAV_SCAN_MAX_VX_MPS", "");
  ScopedEnvironment lateral_speed("LINGTU_NAV_SCAN_MAX_VY_MPS", "");
  ScopedEnvironment teleop_speed("LINGTU_TELEOP_MAX_SPEED_MPS", "");
  const auto cfg=parse({"navd", "--local-planner", "scan"});
  const auto planner=buildLocalPlannerParams(cfg);
  const auto executor=buildExecutorConfig(cfg);
  const double path_speed = std::sqrt(2.0);
  require(nav_kernel::ScanPlannerParams{}.maxVelocity == path_speed &&
              nav_kernel::SplineFollowerParams{}.maxVx == 1.0 &&
              nav_kernel::SplineFollowerParams{}.maxVy == 1.0 &&
              nav_kernel::local::scan::upstream::ClosedLoopControllerParams{}.maxLinearSpeed == path_speed &&
              nav_kernel::local::scan::upstream::ClosedLoopControllerParams{}.maxVx == 1.0 &&
              nav_kernel::local::scan::upstream::ClosedLoopControllerParams{}.maxVy == 1.0,
          "SCAN library defaults must preserve shared path and body-axis limits");
  require(cfg.path_follower_max_speed_mps == path_speed &&
              cfg.teleop_max_speed_mps == 1.0 && planner.scan.maxVelocity == path_speed &&
              planner.scan.maxBodyVx == 1.0 && planner.scan.maxBodyVy == 1.0 &&
              executor.max_speed == path_speed && executor.follower.spline.maxVx == 1.0 &&
              executor.follower.spline.maxVy == 1.0 &&
              lingtu::nav::navigation::ExecutorConfig{}.max_speed == path_speed &&
              lingtu::nav::endpoint::CommandSafetyConfig{}.max_speed_mps == 1.0,
          "bare endpoint parsing and planner/follower wiring must preserve the shared limits");
}

void testBackendSettingsReachPlannerAndFollower() {
  ScopedEnvironment scale("LINGTU_NAV_CMU_PATH_SCALE", "0.9");
  ScopedEnvironment lookahead("LINGTU_NAV_CMU_FOLLOWER_LOOKAHEAD_M", "0.7");
  ScopedEnvironment cmu_voxel("LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M", "");
  ScopedEnvironment scan_voxel("LINGTU_NAV_SCAN_MOTION_VOXEL_SIZE_M", "0.09");
  ScopedEnvironment scan_resolution("LINGTU_NAV_SCAN_VOXEL_RESOLUTION_M", "0.10");
  ScopedEnvironment operator_age("LINGTU_TELEOP_CMD_MAX_AGE_S", "0.5");
  ScopedEnvironment input_future("LINGTU_NAV_INPUT_FUTURE_TOLERANCE_S", "0.07");
  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});
  require(buildLocalPlannerParams(cfg).pathScale == 0.9, "CMU path scale must reach the planner");
  require(cfg.obstacle_voxel_size_m == 0.05, "SCAN voxel tuning must not change CMU defaults");
  const auto follower = buildExecutorConfig(cfg).follower;
  require(nav_kernel::cmuFollowerParams(follower).baseLookAheadDis == 0.7,
          "CMU lookahead must survive follower profile selection");
  const auto scan = parse({"navd", "--local-planner", "scan"});
  require(scan.obstacle_voxel_size_m == 0.09, "SCAN voxel tuning must reach the motion layer");
  require(buildLocalPlannerParams(scan).scan.voxelResolution == 0.10,
          "the map resolution must reach the SCAN planner");
  const auto operator_options = lingtu::nav::endpoint::operatorMotionAuthorityOptions(scan);
  require(operator_options.max_sample_age_s == 0.5 && operator_options.future_tolerance_s == 0.07,
          "operator freshness settings must reach the authority");
}

void testOrdinaryAccelerationDefaultsAndExplicitOverrides() {
  ScopedEnvironment nav_accel("LINGTU_NAV_MAX_ACCEL_MPS2", "");
  ScopedEnvironment follower_accel("LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2", "");
  ScopedEnvironment scan_accel("LINGTU_NAV_SCAN_PLANNER_MAX_ACCELERATION_MPS2", "");
  ScopedEnvironment x_accel("LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2", "");
  ScopedEnvironment y_accel("LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2", "");
  require(nav_kernel::ScanPlannerParams{}.maxAcceleration == 0.5 &&
              nav_kernel::FollowerParams{}.maxAccel == 0.5 &&
              nav_kernel::VelocitySmootherConfig{}.x.acceleration == 0.5 &&
              nav_kernel::VelocitySmootherConfig{}.y.acceleration == 0.5,
          "native library ordinary acceleration defaults must agree");
  for (const char *backend : {"cmu", "scan"}) {
    const auto cfg = parse({"navd", "--path-library", "fixture-paths", "--local-planner", backend});
    require(cfg.path_follower_max_accel_mps2 == 0.5 &&
                cfg.scan_planner.maxAcceleration == 0.5 &&
                cfg.velocity_smoother.x.acceleration == 0.5 &&
                cfg.velocity_smoother.y.acceleration == 0.5,
            "ordinary endpoint acceleration defaults must agree");
    require(buildExecutorConfig(cfg).follower.maxAccel == 0.5 &&
                buildLocalPlannerParams(cfg).scan.maxAcceleration == 0.5,
            "ordinary defaults must reach planner and follower");

    const auto lower = parse({"navd", "--path-library", "fixture-paths", "--local-planner", backend,
                              "--max-accel-mps2", "1.3", "--path-follower-max-accel-mps2", "0.4"});
    require(lower.path_follower_max_accel_mps2 == 0.4 &&
                lower.scan_planner.maxAcceleration == 0.4 &&
                lower.velocity_smoother.x.acceleration == 0.4 &&
                lower.velocity_smoother.y.acceleration == 0.4,
            "dependent acceleration defaults must follow the final CLI follower override");
    require(buildExecutorConfig(lower).follower.maxAccel == 0.4 &&
                buildLocalPlannerParams(lower).scan.maxAcceleration == 0.4,
            "explicit lower acceleration must survive CMU and SCAN composition");
    require(lower.velocity_smoother.x.deceleration == 4.0 &&
                lower.velocity_smoother.y.deceleration == 4.0 &&
                lower.velocity_smoother.yaw.acceleration == 4.0 &&
                lower.velocity_smoother.yaw.deceleration == 4.0,
            "ordinary translation acceleration must not alter braking or yaw defaults");
  }
  ScopedEnvironment explicit_follower("LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2", "0.8");
  ScopedEnvironment explicit_scan("LINGTU_NAV_SCAN_PLANNER_MAX_ACCELERATION_MPS2", "0.25");
  ScopedEnvironment explicit_x("LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2", "0.3");
  ScopedEnvironment explicit_y("LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2", "0.35");
  const auto cfg = parse({"navd", "--local-planner", "scan", "--path-follower-max-accel-mps2", "0.4"});
  require(cfg.path_follower_max_accel_mps2 == 0.4 && cfg.scan_planner.maxAcceleration == 0.25 &&
              cfg.velocity_smoother.x.acceleration == 0.3 &&
              cfg.velocity_smoother.y.acceleration == 0.35,
          "CLI must override follower environment while explicit SCAN and smoother tuning survive");
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
              std::abs(executor_config.follower.maxAccel - 1.0) < 1e-12,
          "CMU config must apply the official Go2 follower profile");
  const auto status = buildStatusWriterConfig(cfg, inputGateConfig(cfg));
  require(std::abs(status.follower_lookahead_m - 0.50) < 1e-12 &&
              std::abs(status.max_accel_mps2 - 1.0) < 1e-12 &&
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
  require(cfg.path_follower_max_speed_mps == 0.0, "negative max speed must clamp to zero");
  require(cfg.path_follower_max_accel_mps2 == 0.0, "negative max accel must clamp to zero");
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
      "--octo-ground-support",
      "true",
      "--octo-direct-support-only",
      "false",
      "--octo-support-depth-cells",
      "8",
      "--octo-search-resolution-m",
      "0.15",
      "--octo-surface-closing-radius-m",
      "0.4",
      "--octo-surface-max-hole-area-m2",
      "0.12",
      "--octo-surface-max-slope",
      "0.5",
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
  require(octo.ground_support, "ground support must stay enabled");
  require(!octo.direct_support_only, "nearby support must remain allowed");
  require(octo.support_depth_cells == 8, "support depth must parse");
  require(std::abs(octo.search_resolution_m - 0.15) < 1e-12, "search resolution must parse");
  require(std::abs(octo.surface_closing_radius_m - 0.4) < 1e-12, "surface closing radius must parse");
  require(std::abs(octo.surface_max_hole_area_m2 - 0.12) < 1e-12, "surface hole area must parse");
  require(std::abs(octo.surface_max_slope - 0.5) < 1e-12, "surface fit slope must parse");
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
  require(buildStatusWriterConfig(teleop, teleop_gate).stop_confirmation_evidence == "zero_output",
          "pure teleop status must disclose driver zero-output confirmation");

  const auto avoid_gate = inputGateConfig(teleop_avoid);
  require(avoid_gate.require_odom, "teleop_avoid needs pose for spatial obstacle safety");
  require(avoid_gate.require_cloud, "teleop_avoid must require obstacle cloud");
  require(!avoid_gate.require_traversability,
          "scan-only teleop_avoid must not wait for traversability");
  require(buildStatusWriterConfig(teleop_avoid, avoid_gate).stop_confirmation_evidence ==
              "zero_output",
          "operator-assisted control must not block on quiet odometry after zero output");
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
      "--path-follower-lookahead-m", "0.3",
      "--path-follower-goal-tolerance-m", "0.2",
      "--path-follower-max-accel-mps2", "1.0",
  });
  require(teleop.path_library_dir.empty(),
          "pure teleop must start without a local planner path library");
  const auto status = buildStatusWriterConfig(teleop, inputGateConfig(teleop));
  require(std::abs(status.follower_lookahead_m - teleop.path_follower_lookahead_m) < 1e-12 &&
              std::abs(status.follower_goal_tolerance_m - teleop.path_follower_goal_tolerance_m) < 1e-12 &&
              std::abs(status.max_accel_mps2 - teleop.path_follower_max_accel_mps2) < 1e-12,
          "unused CMU follower defaults must not override the teleop/map RunPlan status");
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
  ScopedEnvironment hard_margin("LINGTU_TELEOP_OBSTACLE_MARGIN_M", "0.05");
  ScopedEnvironment clearance_below("LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M", "0.10");
  ScopedEnvironment clearance_above("LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M", "0.24");
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
  require(std::abs(cmu.footprintPadding - 0.05) < 1e-12,
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
  const auto body = buildExecutorConfig(scan_cfg).body;
  require(!body, "Go2 final control must not replace the SCAN cylinders with a box");
  require(std::abs(scan.scan.bodyClearanceBelow - .10) < 1e-12 &&
              std::abs(scan.scan.bodyClearanceAbove - .24) < 1e-12,
          "Go2 must retain the configured cylinder height");
}

void testScanPlannerEnvironmentReachesCore() {
  struct Binding {
    const char *name;
    double nav_kernel::ScanPlannerParams::*field;
    double value;
  };
  using Params = nav_kernel::ScanPlannerParams;
  const Binding bindings[] = {
      {"LINGTU_NAV_SCAN_CONTROL_POINT_SPACING_M", &Params::controlPointSpacing, 0.16},
      {"LINGTU_NAV_SCAN_REPLAN_DISTANCE_M", &Params::replanDistance, 0.8},
      {"LINGTU_NAV_SCAN_NO_REPLAN_DISTANCE_M", &Params::noReplanDistance, 0.08},
      {"LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS", &Params::maxVelocity, 0.42},
      {"LINGTU_NAV_SCAN_PLANNER_MAX_ACCELERATION_MPS2", &Params::maxAcceleration, 0.3},
      {"LINGTU_NAV_SCAN_PLANNING_HORIZON_M", &Params::planningHorizon, 2.5},
      {"LINGTU_NAV_SCAN_SMOOTH_WEIGHT", &Params::smoothWeight, 1.2},
      {"LINGTU_NAV_SCAN_COLLISION_WEIGHT", &Params::collisionWeight, 1.3},
      {"LINGTU_NAV_SCAN_FEASIBILITY_WEIGHT", &Params::feasibilityWeight, 0.2},
      {"LINGTU_NAV_SCAN_FITNESS_WEIGHT", &Params::fitnessWeight, 0.9},
      {"LINGTU_NAV_SCAN_FEASIBILITY_TOLERANCE", &Params::feasibilityTolerance, 0.4},
      {"LINGTU_NAV_SCAN_VELOCITY_TOLERANCE", &Params::velocityTolerance, 0.8},
      {"LINGTU_NAV_SCAN_ACCELERATION_TOLERANCE", &Params::accelerationTolerance, 0.7},
      {"LINGTU_NAV_SCAN_COLLISION_DISTANCE_M", &Params::collisionDistance, 0.18},
  };
  for (const auto &binding : bindings) {
    ScopedEnvironment value(binding.name, std::to_string(binding.value));
    const auto cfg = parse({"navd", "--local-planner", "scan"});
    const auto planner = buildLocalPlannerParams(cfg);
    require(std::abs(planner.scan.*(binding.field) - binding.value) < 1e-12,
            binding.name);
  }
  ScopedEnvironment age("LINGTU_NAV_LOCAL_COLLISION_MAX_AGE_S", "0.75");
  const auto cfg = parse({"navd", "--local-planner", "scan"});
  require(inputGateConfig(cfg).local_collision_max_age_s == 0.75 &&
              buildLocalPlannerParams(cfg).localCollisionMaxAge == 0.75,
          "input gate and recovery must consume the same collision freshness policy");
  const auto status = buildStatusWriterConfig(cfg, inputGateConfig(cfg));
  require(status.input_require_local_collision && !status.input_require_cloud &&
              status.local_collision_max_age_s == 0.75,
          "autonomous SCAN requires Mapd collision without a redundant raw-cloud gate");
  const auto avoid = parse({"navd", "--local-planner", "scan", "--control-mode", "teleop_avoid"});
  require(inputGateConfig(avoid).require_local_collision && !inputGateConfig(avoid).require_cloud,
          "assisted teleop keeps its Mapd-only observation contract");
}

void testScanPlannerRejectsInvalidRuntimeParameters() {
  for (const auto *value : {"nan", "-1", "0.04"}) {
    ScopedEnvironment speed("LINGTU_NAV_SCAN_PLANNER_MAX_VELOCITY_MPS", value);
    bool rejected = false;
    try { (void)parse({"navd", "--local-planner", "scan"}); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "invalid SCAN velocity must not reach the optimizer");
  }
  {
    ScopedEnvironment age("LINGTU_NAV_LOCAL_COLLISION_MAX_AGE_S", "0.09");
    bool rejected = false;
    try { (void)parse({"navd", "--local-planner", "scan"}); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "collision age below the recovery minimum must be rejected");
  }
  ScopedEnvironment distance("LINGTU_NAV_SCAN_REPLAN_DISTANCE_M", "0.05");
  bool rejected = false;
  try { (void)parse({"navd", "--local-planner", "scan"}); }
  catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "inverted SCAN replan thresholds must be rejected");
}

void testScanFollowerEnvironmentReachesExecutor() {
  ScopedEnvironment time_forward("LINGTU_NAV_SCAN_TIME_FORWARD_S", "0.65");
  ScopedEnvironment heading_error("LINGTU_NAV_SCAN_HEADING_ERROR_RAD", "0.70");
  ScopedEnvironment position_gain("LINGTU_NAV_SCAN_POSITION_GAIN", "0.90");
  ScopedEnvironment yaw_gain("LINGTU_NAV_SCAN_YAW_GAIN", "1.40");
  ScopedEnvironment max_vx("LINGTU_NAV_SCAN_MAX_VX_MPS", "0.68");
  ScopedEnvironment max_vy("LINGTU_NAV_SCAN_MAX_VY_MPS", "0.32");
  ScopedEnvironment min_vy("LINGTU_NAV_SCAN_MIN_VY_MPS", "0.25");
  ScopedEnvironment max_yaw("LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S", "0.95");
  ScopedEnvironment finish_distance("LINGTU_NAV_SCAN_FINISH_DISTANCE_M", "0.07");

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
          "SCAN finish distance must reach Executor independently");
  require(std::abs(executor_config.follower.spline.minVy - 0.25) < 1e-12,
          "lateral minimum must reach the follower before final safety");
  for (const char *value : {"-0.1", "nan", "inf"}) {
    ScopedEnvironment invalid_minimum("LINGTU_NAV_SCAN_MIN_VY_MPS", value);
    bool rejected = false;
    try {
      (void)parse({"navd", "--local-planner", "scan"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid lateral minimum must fail at the native boundary");
  }
}

void testGoalHeightToleranceIsIndependentOfLocalPlanner() {
  ScopedEnvironment height("LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M", "0.27");
  ScopedEnvironment scan_tuning("LINGTU_NAV_SCAN_COLLISION_WEIGHT", "4.2");

  const auto scan = parse({"navd", "--local-planner", "scan"});
  const auto cmu = parse({"navd", "--local-planner", "cmu", "--path-library", "fixture-paths"});
  require(std::abs(buildExecutorConfig(scan).goal_height_tolerance_m - 0.27) < 1e-12 &&
              std::abs(buildExecutorConfig(cmu).goal_height_tolerance_m - 0.27) < 1e-12,
          "Product arrival height must be independent of local planner and SCAN tuning");
  require(std::abs(buildStatusWriterConfig(scan, inputGateConfig(scan))
                       .goal_height_tolerance_m -
                   0.27) < 1e-12,
          "status must report the actual Product arrival height tolerance");
}

void testGoalHeightToleranceRejectsInvalidEnvironment() {
  for (const char *value : {"0", "-0.1", "nan", "inf"}) {
    ScopedEnvironment height("LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M", value);
    bool rejected = false;
    try {
      (void)parse({"navd", "--local-planner", "scan"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "goal height tolerance must be finite and strictly positive");
  }
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
  require(buildExecutorConfig(defaults).planning_frame ==
              lingtu::nav::navigation::PlanningFrame::Odom,
          "CMU must retain odom-frame planning");
  require(buildExecutorConfig(scan).planning_frame ==
              lingtu::nav::navigation::PlanningFrame::Map,
          "SCAN references and tracking must share the collision map frame");
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

void testScanHasSingleCollisionEnvelope() {
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
  const auto parsed_scan = parse({"navd", "--local-planner", "scan",
                                  "--use-traversability-cost", "true"});
  require(!parsed_scan.use_traversability_cost &&
              !buildStatusWriterConfig(parsed_scan, inputGateConfig(parsed_scan))
                   .use_traversability_cost,
          "SCAN startup and status must agree that the 2D terrain policy is unused");
  CliConfig cfg;
  cfg.use_traversability_cost = true;

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Cmu;
  const auto cmu = buildLocalPlannerParams(cfg);
  require(cmu.useTraversabilityCost,
          "CMU must consume the configured planner traversability policy");

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
  const auto scan = buildLocalPlannerParams(cfg);
  require(!scan.useTraversabilityCost,
          "SCAN must use its 3D collision volume without the CMU terrain-cost grid");
  const auto scan_gate = inputGateConfig(cfg);
  require(!scan_gate.require_traversability && !scan_gate.require_cloud &&
              scan_gate.require_odom && scan_gate.require_local_collision &&
              scan_gate.require_localization_health,
          "SCAN must require authoritative collision and retain pose, localization and driver gates");

  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Cmu;
  require(inputGateConfig(cfg).require_traversability && inputGateConfig(cfg).require_cloud,
          "CMU must retain its consumed cloud and configured terrain input requirements");

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
  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Cmu;
  auto safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - 0.6) < 1e-12,
          "autonomy command boundary must preserve the configured follower speed limit");
  require(safety.max_vx_mps == 0.4 && safety.max_vy_mps == 0.4,
          "autonomy command boundary must preserve the shared body-axis speed budget");
  require(std::abs(safety.max_yaw_rate - 0.8) < 1e-12,
          "autonomy command boundary must preserve the configured follower yaw limit");

  cfg.control_mode = ControlMode::Teleop;
  safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - std::hypot(0.4, 0.4)) < 1e-12 &&
              safety.max_vx_mps == 0.4 && safety.max_vy_mps == 0.4,
          "teleop command boundary must allow the full per-axis diagonal budget");
  require(std::abs(safety.max_yaw_rate - 1.0) < 1e-12,
          "teleop command boundary must use the teleop yaw limit");

  cfg.control_mode = ControlMode::TeleopAvoid;
  safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - std::hypot(0.4, 0.4)) < 1e-12,
          "teleop_avoid total speed must allow both body axes together");
  require(safety.max_vx_mps == 0.4 && safety.max_vy_mps == 0.4,
          "teleop_avoid must preserve its separate body-axis bound");
  require(std::abs(safety.max_yaw_rate - 1.0) < 1e-12,
          "teleop_avoid command boundary must use the teleop yaw limit");
  cfg.teleop_max_speed_mps = 1.0;
  cfg.path_follower_max_speed_mps = std::hypot(1.0, 1.0);
  safety = commandSafetyConfig(cfg);
  require(safety.max_vx_mps == 1.0 && safety.max_vy_mps == 1.0 &&
              std::abs(safety.max_speed_mps - std::hypot(1.0, 1.0)) < 1e-12,
          "resolved assisted profile must admit the full diagonal budget");
  cfg.path_follower_max_speed_mps = 1.2;
  safety = commandSafetyConfig(cfg);
  require(std::abs(safety.max_speed_mps - 1.2) < 1e-12,
          "explicit lower assisted total-speed override must remain binding");

  cfg.control_mode = ControlMode::Autonomy;
  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
  cfg.scan_follower.maxVx = 1.2;
  cfg.scan_follower.maxVy = 1.0;
  cfg.teleop_max_speed_mps = 0.4;
  safety = commandSafetyConfig(cfg);
  require(safety.max_vx_mps == 1.2 && safety.max_vy_mps == 1.0,
          "autonomous SCAN must use its own body-axis budgets");
}

void testScanCollisionEnvelopeIsConfigurable() {
  ScopedEnvironment global_radius("LINGTU_NAV_OCTO_ROBOT_RADIUS_M", "0.155");
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
  require(std::abs(planner.scan.cylinderRadius - 0.41) < 1e-12,
          "SCAN prediction envelope must use the same radius as Mapd");
  require(std::abs(planner.scan.cylinderOffset - 0.24) < 1e-12,
          "SCAN collision offset must reach the planner");
  require(std::abs(cfg.octoplanner_options.robot_radius - 0.41) < 1e-12,
          "global XYZ search must use corridor width without adding body length");
  require(std::abs(cfg.octoplanner_options.body_clearance_below_m - 0.25) < 1e-12 &&
              std::abs(cfg.octoplanner_options.body_clearance_above_m - 0.40) < 1e-12,
          "global clearance must cover SCAN's discretely inflated vertical extent");
  require(std::abs(planner.scan.bodyClearanceBelow - 0.22) < 1e-12,
          "SCAN lower body clearance must reach the planner");
  require(std::abs(planner.scan.bodyClearanceAbove - 0.36) < 1e-12,
          "SCAN upper body clearance must reach the planner");
}

void testScanGlobalCorridorEnvironmentAndLargerOverrides() {
  ScopedEnvironment planner_backend("LINGTU_NAV_LOCAL_PLANNER_BACKEND", "scan");
  ScopedEnvironment radius("LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M", "0.25");
  ScopedEnvironment offset("LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M", "0.19");
  ScopedEnvironment below("LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M", "0.10");
  ScopedEnvironment above("LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M", "0.24");
  ScopedEnvironment global_radius("LINGTU_NAV_OCTO_ROBOT_RADIUS_M", "0.10");
  ScopedEnvironment global_below("LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M", "0.01");
  ScopedEnvironment global_above("LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M", "0.01");
  const auto cfg = parse({"navd"});
  require(std::abs(cfg.octoplanner_options.robot_radius - 0.25) < 1e-12 &&
              std::abs(cfg.octoplanner_options.body_clearance_below_m - 0.10) < 1e-12 &&
              std::abs(cfg.octoplanner_options.body_clearance_above_m - 0.25) < 1e-12,
          "undersized environment values must not shrink corridor width or height");

  const auto larger = parse({"navd", "--octo-robot-radius-m", "0.70",
                            "--octo-body-clearance-below-m", "0.40",
                            "--octo-body-clearance-above-m", "0.50"});
  require(std::abs(larger.octoplanner_options.robot_radius - 0.70) < 1e-12 &&
              std::abs(larger.octoplanner_options.body_clearance_below_m - 0.40) < 1e-12 &&
              std::abs(larger.octoplanner_options.body_clearance_above_m - 0.50) < 1e-12,
          "larger explicit global clearances must remain available");

  const auto cmu = parse({"navd", "--local-planner", "cmu", "--path-library", "fixture-paths"});
  require(std::abs(cmu.octoplanner_options.robot_radius - 0.10) < 1e-12 &&
              std::abs(cmu.octoplanner_options.body_clearance_below_m - 0.01) < 1e-12 &&
              std::abs(cmu.octoplanner_options.body_clearance_above_m - 0.01) < 1e-12,
          "SCAN geometry normalization must not change the CMU contract");
}

void testScanGlobalCorridorUsesThunderGeometry() {
  const auto cfg = parse({"navd", "--local-planner", "scan",
                          "--collision-cylinder-radius-m", "0.40",
                          "--collision-cylinder-offset-m", "0.25",
                          "--collision-clearance-below-m", "0.25",
                          "--collision-clearance-above-m", "0.35"});
  require(std::abs(cfg.octoplanner_options.robot_radius - 0.40) < 1e-12 &&
              std::abs(cfg.octoplanner_options.body_clearance_below_m - 0.25) < 1e-12 &&
              std::abs(cfg.octoplanner_options.body_clearance_above_m - 0.35) < 1e-12,
          "the global corridor must derive from the selected platform geometry");
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
  require(std::abs(cfg.path_follower_max_speed_mps - 0.5) < 1e-12,
          "environment numbers may contain surrounding whitespace");
}

void testMotionLimitOverrides() {
  ScopedEnvironment speed("LINGTU_NAV_MAX_SPEED_MPS", "0.6");
  ScopedEnvironment follower_speed("LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS", "0.7");
  ScopedEnvironment accel("LINGTU_NAV_MAX_ACCEL_MPS2", "1.1");
  ScopedEnvironment follower_accel("LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2", "1.2");
  const auto env = parse({"navd", "--path-library", "fixture-paths"});
  require(env.path_follower_max_speed_mps == 0.7 && env.path_follower_max_accel_mps2 == 1.2,
          "follower environment limits must override the general limits");
  const auto cli = parse({"navd", "--path-library", "fixture-paths",
                          "--max-speed-mps", "0.9", "--path-follower-max-speed-mps", "0.8",
                          "--path-follower-max-accel-mps2", "0.4", "--max-accel-mps2", "1.3"});
  require(cli.path_follower_max_speed_mps == 0.8 && cli.path_follower_max_accel_mps2 == 1.3,
          "later CLI limits must override earlier limits");

  {
    ScopedEnvironment forward("LINGTU_NAV_SCAN_MAX_VX_MPS", "0.005");
    ScopedEnvironment lateral("LINGTU_NAV_SCAN_MAX_VY_MPS", "0.005");
    const auto low = parse({"navd", "--local-planner", "scan"});
    require(low.scan_follower.maxVx == 0.005 && low.scan_follower.maxVy == 0.005,
            "positive low hardware axis limits must reach the SCAN follower");
  }

  for (const auto &option : {"--max-speed-mps", "--max-accel-mps2"}) {
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths", option, "nan",
                   "--path-follower-max-speed-mps", "0.8",
                   "--path-follower-max-accel-mps2", "1.0"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid general limits must be rejected even when later overridden");
  }
  for (const auto &name : {"LINGTU_NAV_MAX_SPEED_MPS", "LINGTU_NAV_MAX_ACCEL_MPS2"}) {
    ScopedEnvironment invalid(name, "nan");
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid general environment limits must not be hidden by overrides");
  }
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

  ScopedEnvironment clock("LINGTU_NAV_EXECUTION_CLOCK", "simulation");

  const auto cfg = parse({"navd", "--control-mode", "teleop"});
  require(cfg.use_simulation_clock, "sim RunPlan must select the physical execution clock");
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

void testExecutionClockConfiguration() {
  ScopedEnvironment runtime_env("LINGTU_ENV", "real");
  {
    ScopedEnvironment clock("LINGTU_NAV_EXECUTION_CLOCK", "");
    require(!parse({"navd", "--control-mode", "teleop"}).use_simulation_clock,
            "real execution must default to steady time");
  }
  for (const char *value : {"simulation", "unknown"}) {
    ScopedEnvironment clock("LINGTU_NAV_EXECUTION_CLOCK", value);
    bool rejected = false;
    try {
      (void)parse({"navd", "--control-mode", "teleop"});
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "real must reject simulation or unknown execution clocks");
  }
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
  ScopedEnvironment minimum("LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MIN_M", "0.10");
  ScopedEnvironment legacy_maximum("LINGTU_TELEOP_OBSTACLE_HEIGHT_MAX_M", "1.20");
  ScopedEnvironment maximum("LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M", "0.80");
  const auto cfg = parse({"navd", "--path-library", "fixture-paths"});

  const auto planner = buildLocalPlannerParams(cfg);
  require(std::abs(planner.obstacleHeightThre - 0.10) < 1e-12,
          "local planner must use the configured obstacle minimum");
  require(std::abs(planner.obstacleHeightMax - 0.80) < 1e-12,
          "the local setting must own the planner obstacle maximum");
  const auto status = buildStatusWriterConfig(cfg, inputGateConfig(cfg));
  require(status.planner_obstacle_height_min_m == planner.obstacleHeightThre &&
              status.planner_obstacle_height_max_m == planner.obstacleHeightMax,
          "the status preview must use the same obstacle interval as the planner");

  const auto low = buildLocalPlannerParams(parse({
      "navd", "--path-library", "fixture-paths",
      "--local-planner-obstacle-height-min-m", "0.05",
      "--local-planner-obstacle-height-max-m", "0.10",
  }));
  require(low.obstacleHeightThre == 0.05 && low.obstacleHeightMax == 0.10,
          "a valid low obstacle interval must reach the planner unchanged");

  for (const auto &[value, expected] :
       {std::pair{"-0.10", "local planner obstacle height minimum must be finite and non-negative"},
        std::pair{"0.90", "local planner obstacle height maximum must be at least the minimum"}}) {
    bool rejected = false;
    try {
      (void)parse({"navd", "--path-library", "fixture-paths",
                   "--local-planner-obstacle-height-min-m", value});
    } catch (const std::runtime_error &error) {
      rejected = std::string(error.what()) == expected;
    }
    require(rejected, "an invalid local obstacle height interval must fail during config parsing");
  }
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
  ScopedEnvironment future_tolerance("LINGTU_NAV_SEGMENT_FUTURE_TOLERANCE_S", "0.7");
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
  require(std::abs(policy.map_input.future_tolerance_s - 0.7) < 1e-12,
          "segment map future tolerance must reach the planner");

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

void testScanUsesConfiguredCylindersWithoutBodyFallback() {
  CliConfig cfg;
  cfg.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
  cfg.collision_cylinder_radius_m = .32;
  cfg.collision_cylinder_offset_m = .23;
  cfg.collision_clearance_below_m = .05;
  cfg.collision_clearance_above_m = .35;
  cfg.teleop_obstacle_margin_m = .08;
  require(!buildExecutorConfig(cfg).body,
          "SCAN final control must use the inflated twin cylinders, not a fallback box");
  const auto local = buildLocalPlannerParams(cfg);
  require(std::abs(local.scan.cylinderRadius - .32) < 1e-12 &&
              std::abs(local.scan.cylinderOffset - .23) < 1e-12,
          "the same configured radius and offset must reach SCAN and final control");
  require(std::abs(local.scan.bodyClearanceBelow - .05) < 1e-12 &&
              std::abs(local.scan.bodyClearanceAbove - .35) < 1e-12,
          "body-relative cylinder height must not receive another margin");
}

void testStairLandingUsesPhysicalFootprint() {
  CliConfig cfg;
  cfg.vehicle_length_m = 1.0;
  cfg.vehicle_width_m = 0.6;
  cfg.collision_cylinder_radius_m = 0.4;
  cfg.collision_cylinder_offset_m = 0.25;
  const auto executor = buildExecutorConfig(cfg);
  require(std::abs(executor.stairs.bodyLength - 1.0) < 1e-12 &&
              std::abs(executor.stairs.bodyWidth - 0.6) < 1e-12,
          "stair landing must use the physical vehicle footprint, not SCAN circles");
  const auto local = buildLocalPlannerParams(cfg);
  require(std::abs(local.scan.cylinderRadius - 0.4) < 1e-12 &&
              std::abs(local.scan.cylinderOffset - 0.25) < 1e-12,
          "stair footprint mapping must retain the local collision envelope");
}


}  // namespace

int main() {
  try {
    ScopedEnvironment runtime_env("LINGTU_ENV", "");
    testNavigationRateAndAcceleration();
    testScanUsesConfiguredCylindersWithoutBodyFallback();
    testOrdinaryAccelerationDefaultsAndExplicitOverrides();
    testOrdinarySpeedDefaultsAgreeAcrossTheNativeBoundary();
    testBackendSettingsReachPlannerAndFollower();
    testStairLandingUsesPhysicalFootprint();
    testDynamicObstacleThresholdsParse();
    testRuntimeEnvironmentPrecedesDefaultsAndCliPrecedesEnvironment();
    testGo2RunPlanGeometryEnvironmentParsesExactly();
    testScanFollowerEnvironmentReachesExecutor();
    testGoalHeightToleranceIsIndependentOfLocalPlanner();
    testGoalHeightToleranceRejectsInvalidEnvironment();
    testScanPlannerEnvironmentReachesCore();
    testScanPlannerRejectsInvalidRuntimeParameters();
    testCompiledProductMotionContractParses();
    testVelocitySmootherEnvironmentParsesExactly();
    testVelocitySmootherRejectsUnavailableClosedLoopOnlyWhenEnabled();
    testVelocitySmootherConfigurationFailsClosed();
    testEnvironmentNumbersRejectTrailingJunk();
    testMotionLimitOverrides();
    testVelocitySmootherAcceptsCoreBoundaryValues();
    testSimulationRunPlanIdentityDerivesRuntimePaths();
    testExecutionClockConfiguration();
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
    testScanHasSingleCollisionEnvelope();
    testLocalPlannerUsesConfiguredTraversabilityPolicy();
    testCommandBoundaryUsesActiveControlModeMotionLimits();
    testScanCollisionEnvelopeIsConfigurable();
    testScanGlobalCorridorEnvironmentAndLargerOverrides();
    testScanGlobalCorridorUsesThunderGeometry();
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
