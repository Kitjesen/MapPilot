#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "control/teleop.hpp"
#include "runtime/loop.hpp"
#include "tracking/smoother.cpp"

namespace {

using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::CommandSafetyConfig;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::FinalControl;
using lingtu::nav::endpoint::FinalActions;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::enforcePostPlanningInputReadiness;
using lingtu::nav::endpoint::TeleopDiagnostics;
using lingtu::nav::endpoint::TeleopTickActions;
using lingtu::nav::endpoint::TeleopTickController;
using lingtu::nav::endpoint::TeleopTickInput;
using lingtu::nav::endpoint::PlanView;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  CliConfig config;
  CommandSafetyConfig safety;
  std::optional<nav_kernel::Pose> map_body{nav_kernel::Pose{}};
  InputGateState gate;
  nav_kernel::Twist request{0.2, 0.0, 0.0};
  std::vector<float> obstacles;
  TraversabilityGrid traversability;
  TeleopDiagnostics previous;
  TimingDiagnostics timing;
  PlanView planner_inputs;
  lingtu::nav::navigation::ExecutionOutput planner_output;
  std::vector<double> ages{0.1, 0.1};
  std::size_t age_index{0};
  std::vector<double> now_values{10.0};
  std::size_t now_index{0};
  int compute_calls{0};
  int planner_calls{0};
  int pause_calls{0};
  int replan_calls{0};
  int stop_calls{0};
  int now_calls{0};
  int shape_calls{0};
  int commit_calls{0};
  int velocity_stop_calls{0};
  nav_kernel::Twist shaped_command{};
  nav_kernel::Twist shape_input{};
  nav_kernel::Twist committed_command{};
  nav_kernel::Twist smoother_state{};
  double shape_now_s{0.0};
  double commit_now_s{0.0};
  double velocity_stop_now_s{0.0};
  std::string velocity_stop_reason;
  bool override_shaped_command{false};
  bool shape_from_mock_state{false};
  bool shape_valid{true};
  bool commit_succeeds{true};
  TeleopTickActions actions;
  FinalActions final_actions;
  std::optional<FinalControl> final_control;

  Fixture() {
    gate.ready = true;
    previous.request = {0.7, 0.1, 0.2};
    previous.age_s = 4.0;
    actions.teleop_receive_age_s = [&] {
      const auto index = std::min(age_index++, ages.size() - 1);
      return ages[index];
    };
    actions.steady_now_s = [&] {
      ++now_calls;
      const auto index = std::min(now_index++, now_values.size() - 1);
      return now_values[index];
    };
    actions.read_plan = [&](double, TimingDiagnostics &) {
      ++compute_calls;
      return planner_inputs;
    };
    actions.tick_teleop_intent = [&](const nav_kernel::Pose &, const nav_kernel::Twist &,
                                     const float *, int, double,
                                     lingtu::nav::navigation::TraversabilityGridView) {
      ++planner_calls;
      return planner_output;
    };
    actions.pause_linear_motion = [&] { ++pause_calls; };
    actions.replan_motion = [&] { ++replan_calls; };
    actions.stop_linear_motion = [&] { ++stop_calls; };
    final_actions.command_safety = lingtu::nav::endpoint::evaluateCommandSafety;
    final_actions.shape = [&](const nav_kernel::Twist &raw, double now_s) {
      ++shape_calls;
      shape_input = raw;
      shape_now_s = now_s;
      nav_kernel::VelocitySmootherOutput output;
      output.command = shape_from_mock_state
                           ? smoother_state
                           : (override_shaped_command ? shaped_command : raw);
      output.valid = shape_valid;
      output.reason = shape_valid ? "smoothed" : "velocity_smoother_invalid";
      return output;
    };
    final_actions.commit = [&](const nav_kernel::Twist &command, double now_s) {
      ++commit_calls;
      committed_command = command;
      commit_now_s = now_s;
      return commit_succeeds;
    };
    final_actions.stop = [&](double now_s, const std::string &reason) {
      ++velocity_stop_calls;
      velocity_stop_now_s = now_s;
      velocity_stop_reason = reason;
      smoother_state = {};
    };
  }

  FinalControl &control() {
    final_control.emplace(final_actions);
    return *final_control;
  }

  TeleopTickInput input(bool path_active = false, bool request_present = true) {
    return {
        config,    safety,         map_body, gate,     request_present ? &request : nullptr, path_active,
        obstacles, traversability, 9.5,      previous, timing,
    };
  }
};

void testIdleTeleopPublishesAZeroHeartbeatForLateJoiningDriver() {
  Fixture fixture;
  fixture.config.control_mode = ControlMode::TeleopAvoid;
  fixture.config.publish_cmd_vel = true;
  fixture.previous.output = {0.2, 0.1, 0.3};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input(false, false));

  require(result.handled, "idle teleop mode must keep the final command boundary active");
  require(result.publish.cmd_vel && result.publish.command.vx == 0.0 &&
              result.publish.command.vy == 0.0 && result.publish.command.wz == 0.0,
          "idle teleop mode must publish only a zero heartbeat");
  require(result.teleop.stopped && result.teleop.published && result.teleop.reason == "idle",
          "idle teleop diagnostics must report a published stop");
  require(result.teleop.output.vx == 0.0 && result.teleop.output.vy == 0.0 &&
              result.teleop.output.wz == 0.0,
          "idle diagnostics must not retain a previous nonzero output");
  require(result.delta.cmd_vel_count == 1 && result.delta.teleop_output_count == 0,
          "idle heartbeat must count only the final command publication");
  require(fixture.compute_calls == 0 && fixture.planner_calls == 0 && fixture.stop_calls == 0,
          "idle heartbeat must not enter local planning");
}

void testIdleAutonomyDoesNotPublishTeleopHeartbeat() {
  Fixture fixture;
  fixture.config.control_mode = ControlMode::Autonomy;
  fixture.config.publish_cmd_vel = true;
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input(false, false));

  require(!result.handled && !result.publish.cmd_vel,
          "idle autonomy must remain owned by the autonomy runtime");
}

void testBlockedGatePausesPlannerWithoutCancellingIntent() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.gate.ready = false;
  fixture.gate.reason = "input_cloud_stale";
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.handled, "active teleop must be handled");
  require(result.teleop.seen && !result.teleop.fresh, "blocked input must be visible and stale");
  require(result.teleop.stopped && result.teleop.limited && !result.teleop.slowed,
          "blocked input must fail closed");
  require(result.teleop.reason == "input_cloud_stale", "gate reason must be retained");
  require(result.teleop.request.vx == fixture.previous.request.vx &&
              result.teleop.age_s == fixture.previous.age_s,
          "untouched diagnostic fields must retain their previous values");
  require(result.publish.cmd_vel, "blocked input must request a zero publish");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "controller must never turn a blocked input into nonzero output");
  require(result.delta.cmd_vel_count == 1 && result.delta.teleop_output_count == 1 &&
              result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "blocked counters must match endpoint behavior");
  require(fixture.pause_calls == 1 && fixture.replan_calls == 0 && fixture.stop_calls == 0 &&
              fixture.compute_calls == 0 && fixture.planner_calls == 0,
          "a transient input outage must pause output without cancelling the teleop target");
  require(fixture.velocity_stop_calls == 1 &&
              fixture.velocity_stop_reason == "input_cloud_stale",
          "blocked input must hard-stop smoother state");
}

void testDirectCommandIsAcceptedOrLimitedWithoutPlannerSideEffects() {
  for (const auto &sample : std::vector<std::pair<double, bool>>{{0.2, false}, {1.0, true}}) {
    Fixture fixture;
    fixture.config.teleop_local_planner = false;
    fixture.config.publish_cmd_vel = true;
    fixture.safety.max_speed_mps = 0.4;
    fixture.request.vx = sample.first;
    TeleopTickController controller(fixture.actions, fixture.control());

    const auto result = controller.tick(fixture.input());

    require(result.publish.cmd_vel, "direct command must return a publish intent");
    require(result.teleop.published && result.teleop.fresh,
            "fresh direct command diagnostics mismatch");
    require(!result.teleop.stopped && result.teleop.limited == sample.second,
            "direct command limit disposition mismatch");
    require(result.teleop.reason == (sample.second ? "limited" : "accepted"),
            "direct command reason mismatch");
    require(result.publish.command.vx == (sample.second ? 0.4 : 0.2),
            "direct command output mismatch");
    require(result.delta.cmd_vel_count == 1 && result.delta.teleop_output_count == 1 &&
                result.delta.teleop_limited_count == (sample.second ? 1U : 0U),
            "direct command counter mismatch");
    require(fixture.compute_calls == 0 && fixture.planner_calls == 0 && fixture.stop_calls == 0,
            "direct command must not enter or stop an unused planner");
  }
}

void testPublishTimeStaleOverridesAnEarlierAcceptedDecision() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.config.teleop_cmd_max_age_s = 0.35;
  fixture.ages = {0.1, 0.5};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.publish.cmd_vel, "stale transition must publish a zero intent");
  require(!result.teleop.fresh && result.teleop.stopped && result.teleop.limited,
          "publish-time stale decision must fail closed");
  require(result.teleop.reason == "stale" && result.teleop.age_s == 0.5,
          "publish-time age and reason mismatch");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "stale transition must overwrite the accepted nonzero command");
  require(result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "stale counters mismatch");
}

void testAssistedPathReturnsPlannerArtifactsAndFinalSafetyIntent() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assisted";
  fixture.planner_output.slow_down = 1;
  fixture.planner_output.target_distance_m = 1.0;
  fixture.planner_output.target = {1.0, 0.0, 0.0};
  fixture.planner_output.cmd_vel = {0.25, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.compute_calls == 1 && fixture.planner_calls == 1 && fixture.stop_calls == 0,
          "assisted path action sequence mismatch");
  require(result.local.has_value() && result.local->path_found,
          "assisted local diagnostics missing");
  require(result.local->reason == "teleop_assisted" &&
              result.local->final_safety_reason == "teleop_assisted",
          "assisted diagnostics reason mismatch");
  require(result.local_path.size() == 2 && result.publish.local_path && result.publish.waypoint,
          "assisted planner artifacts must be returned as publish intents");
  require(result.publish.cmd_vel && result.publish.command.vx == 0.25,
          "assisted command intent mismatch");
  require(result.teleop.reason == "teleop_assisted" && result.delta.output_count == 1,
          "assisted teleop result mismatch");
}

void testManualModeBypassesUnavailablePlanningInputs() {
  Fixture fixture;
  fixture.config.control_mode = ControlMode::TeleopAvoid;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.gate.ready = false;
  fixture.gate.reason = "localization_health_stale";
  fixture.map_body.reset();
  fixture.obstacles = {0.10F, 0.0F, 0.30F, 0.30F};
  TeleopTickController controller(fixture.actions, fixture.control());
  auto input = fixture.input();
  input.manual_mode = true;

  const auto result = controller.tick(input);

  require(result.handled && result.publish.cmd_vel,
          "manual mode must keep the teleop command boundary active");
  require(result.publish.command.vx > 0.0 && !result.teleop.stopped,
          "manual mode must allow a bounded nonzero command without planning inputs");
  require(result.teleop.manual_mode && result.teleop.reason == "manual_mode",
          "manual mode must be explicit in native diagnostics");
  require(fixture.compute_calls == 0 && fixture.planner_calls == 0,
          "manual mode must not enter the local planner");
}

void testManualModeDoesNotBypassDriverReadiness() {
  Fixture fixture;
  fixture.config.control_mode = ControlMode::TeleopAvoid;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.gate.ready = false;
  fixture.gate.reason = "driver_control_stale";
  TeleopTickController controller(fixture.actions, fixture.control());
  auto input = fixture.input();
  input.manual_mode = true;

  const auto result = controller.tick(input);

  require(result.teleop.manual_mode && result.teleop.stopped,
          "manual request must remain visible when driver readiness blocks it");
  require(result.teleop.reason == "driver_control_stale" &&
              result.publish.command.vx == 0.0,
          "manual mode must never bypass the driver-control gate");
  require(fixture.planner_calls == 0 && fixture.velocity_stop_calls == 1,
          "driver-control failure must stop without entering the planner");
}

void testCmuDetourPublishesTranslationAndYawWithoutRecovery() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.obstacles = {0.20F, 0.0F, 0.30F, 0.30F};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assist_control_ready";
  fixture.planner_output.trajectory_frozen = false;
  fixture.planner_output.cmd_vel = {0.15, -0.10, -0.25};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {0.5, -0.25, 0.0},
      {1.0, -0.5, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.publish.cmd_vel &&
              std::abs(result.publish.command.vx - 0.15) < 1e-9 &&
              std::abs(result.publish.command.vy + 0.10) < 1e-9 &&
              std::abs(result.publish.command.wz + 0.25) < 1e-9,
          "CMU detour translation and yaw must reach the final command boundary");
  require(!result.teleop.stopped &&
              result.teleop.reason == "teleop_assist_control_ready",
          "normal CMU detour must remain a path-following state");
  require(result.local.has_value() && result.local->path_found &&
              result.local->recovery_state == 0 &&
              result.local->recovery_action ==
                  static_cast<int>(nav_kernel::RecoveryAction::None),
          "CMU detour must not be reported as Recovery");
  require(fixture.pause_calls == 0 && fixture.replan_calls == 0 &&
              fixture.stop_calls == 0,
          "a safe selected CMU path must not trigger endpoint recovery side effects");
}

void testVerifiedTeleopRotationPublishesWithoutPath() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.obstacles = {0.20F, 0.0F, 0.30F, 0.30F};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = false;
  fixture.planner_output.near_field_stop = true;
  fixture.planner_output.reason = "recovery_rotation_active";
  fixture.planner_output.recovery_state = 1;
  fixture.planner_output.recovery_action =
      static_cast<int>(nav_kernel::RecoveryAction::Rotate);
  fixture.planner_output.recovery_verified = true;
  fixture.planner_output.recovery_reason = "recovery_rotation_active";
  fixture.planner_output.cmd_vel = {0.0, 0.0, 0.25};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.publish.cmd_vel && result.publish.command.vx == 0.0 &&
              result.publish.command.vy == 0.0 &&
              std::abs(result.publish.command.wz - 0.25) < 1e-9,
          "a verified rotate-first recovery must reach the final command boundary");
  require(!result.teleop.stopped &&
              result.teleop.reason == "recovery_rotation_active",
          "verified rotation must not be downgraded to a no-path stop");
  require(result.local.has_value() && result.local->recovery_state == 1 &&
              result.local->recovery_action ==
                  static_cast<int>(nav_kernel::RecoveryAction::Rotate) &&
              result.local->recovery_verified,
          "teleop diagnostics must expose the active verified rotation");
  require(fixture.replan_calls == 0 && fixture.pause_calls == 0 &&
              fixture.stop_calls == 0,
          "an accepted rotation must not reset or cancel its yaw target");
}

void testPlannerSlowdownIsNotAppliedTwice() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.request = {0.20, 0.0, 0.0};
  fixture.obstacles = {0.90F, 0.0F, 0.30F, 0.30F};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assist_detour";
  fixture.planner_output.slow_down = 1;
  fixture.planner_output.cmd_vel = {0.11, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {0.6, 0.0, 0.0},
      {1.2, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.teleop.slowed && !result.teleop.stopped,
          "assisted obstacle path must remain a slowdown, not a stop");
  require(result.teleop.reason == "teleop_assist_detour",
          "the accepted planner reason must reach teleop diagnostics");
  require(std::abs(result.publish.command.vx - 0.11) < 1e-9,
          "planner slowdown must survive the command boundary without being multiplied again");
  require(std::abs(result.publish.command.vx - 0.0385) > 1e-3,
          "the duplicated planner-times-safety slowdown must not return");
}

void testPlannerTerrainSlowdownDoesNotResetSmootherRamp() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.max_speed_mps = 0.5;
  fixture.safety.min_motion_speed_mps = 0.03;
  fixture.request = {0.50, 0.0, 0.0};
  fixture.override_shaped_command = true;
  fixture.shaped_command = {0.05, 0.0, 0.0};
  fixture.traversability.rows = 80;
  fixture.traversability.cols = 80;
  fixture.traversability.resolution = 0.2;
  fixture.traversability.origin_x = -8.0;
  fixture.traversability.origin_y = -8.0;
  fixture.traversability.values.assign(80U * 80U, 44.0F);
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "terrain_slow";
  fixture.planner_output.slow_down = 1;
  fixture.planner_output.cmd_vel = {0.125, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(!result.teleop.stopped && result.teleop.slowed,
          "soft terrain must not turn a valid assisted ramp into a hard stop");
  require(result.teleop.reason == "terrain_slow",
          "the planner terrain reason must reach teleop diagnostics");
  require(std::abs(result.publish.command.vx - 0.05) < 1e-9,
          "the smoother ramp already below the safety cap must pass unchanged");
  require(fixture.velocity_stop_calls == 0 && fixture.commit_calls == 1,
          "a valid composed ramp must advance instead of resetting smoother state");
}

void testPlannerRampBelowOperatorDeadbandKeepsAdvancing() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.min_motion_speed_mps = 0.03;
  fixture.request = {0.50, 0.50, 0.0};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assist_spline_ready";
  fixture.planner_output.cmd_vel = {0.02, 0.01, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(!result.teleop.stopped && result.teleop.reason == "teleop_assist_spline_ready",
          "a valid follower ramp must not be stopped by the raw operator deadband");
  require(std::abs(result.publish.command.vx - 0.02) < 1e-9 &&
              std::abs(result.publish.command.vy - 0.01) < 1e-9,
          "the follower ramp must reach the command boundary unchanged");
  require(fixture.replan_calls == 0 && fixture.velocity_stop_calls == 0,
          "a valid follower ramp must keep its planner and acceleration state");
}

void testPlannerTerrainScalePassesThroughUnchanged() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.max_speed_mps = 0.5;
  fixture.request = {0.50, 0.0, 0.0};
  fixture.override_shaped_command = true;
  fixture.shaped_command = {0.41875, 0.0, 0.0};
  fixture.traversability.rows = 80;
  fixture.traversability.cols = 80;
  fixture.traversability.resolution = 0.2;
  fixture.traversability.origin_x = -8.0;
  fixture.traversability.origin_y = -8.0;
  fixture.traversability.values.assign(80U * 80U, 50.0F);
  fixture.planner_inputs.traversability = {
      fixture.traversability.values.data(), fixture.traversability.rows,
      fixture.traversability.cols, fixture.traversability.resolution,
      fixture.traversability.origin_x, fixture.traversability.origin_y, 1,
  };
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "terrain_slow";
  fixture.planner_output.slow_down = 1;
  fixture.planner_output.cmd_vel = {0.41875, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(!result.teleop.stopped && result.teleop.slowed,
          "soft terrain must remain a continuous slowdown");
  require(std::abs(result.publish.command.vx - 0.41875) < 1e-9,
          "assisted control must keep the measured soft-risk scale instead of the worst case");
}

void testAssistedNoPathFailsClosedAndReplansOriginalIntent() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = false;
  fixture.planner_output.reason.clear();
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.compute_calls == 1 && fixture.planner_calls == 1 &&
              fixture.replan_calls == 1 && fixture.pause_calls == 0 && fixture.stop_calls == 0,
          "no-path assist must stop output and retry the original teleop target");
  require(result.teleop.stopped && result.teleop.limited &&
              result.teleop.reason == "teleop_assist_no_path",
          "no-path assist must fail closed with a stable reason");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "no-path assist must only return a zero command intent");
  require(result.local.has_value() && !result.local->path_found &&
              result.local->final_safety_reason == "teleop_assist_no_path",
          "no-path local diagnostics mismatch");
  require(result.publish.local_path && result.publish.waypoint && result.delta.output_count == 1 &&
              result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "no-path publish intents and counters mismatch");
}

void testPlannerAcceptedPathIsNotVetoedByDuplicateFinalSweep() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.obstacles = {0.40F, 0.0F, 0.30F, 0.30F};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assist_control_ready";
  fixture.planner_output.hold_body_heading = true;
  fixture.planner_output.cmd_vel = {0.20, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(!result.teleop.stopped && result.teleop.reason == "teleop_assist_control_ready",
          "an accepted planner path must not be rejected by a second collision sweep");
  require(std::abs(result.publish.command.vx - 0.20) < 1e-9,
          "the planner-approved command must reach the final command boundary");
  require(fixture.replan_calls == 0 && fixture.pause_calls == 0 && fixture.stop_calls == 0,
          "the final command boundary must not replan a planner-approved path");
}

void testActiveAutonomyPathSuppressesTeleopPublishing() {
  Fixture fixture;
  fixture.previous.stopped = true;
  fixture.previous.limited = true;
  fixture.previous.output = {0.1, 0.2, 0.3};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input(true));

  require(result.handled && result.teleop.seen && result.teleop.fresh,
          "auto-active teleop diagnostics mismatch");
  require(!result.teleop.published && result.teleop.reason == "auto_active",
          "active autonomy path must suppress teleop publication");
  require(result.teleop.stopped && result.teleop.limited && result.teleop.output.vy == 0.2,
          "auto-active must preserve fields the endpoint did not overwrite");
  require(!result.publish.cmd_vel && !result.publish.local_path && !result.publish.waypoint,
          "auto-active branch must return no publish intents");
  require(fixture.compute_calls == 0 && fixture.planner_calls == 0 && fixture.stop_calls == 0,
          "auto-active branch must not touch teleop planner state");
}

void testAssistedPublishTimeStaleUpdatesLocalFinalSafety() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.config.teleop_cmd_max_age_s = 0.35;
  fixture.ages = {0.1, 0.5};
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assisted";
  fixture.planner_output.cmd_vel = {0.25, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.planner_calls == 1 && fixture.stop_calls == 1,
          "publish-time stale must stop an already-ticked assisted planner");
  require(result.teleop.reason == "stale" && !result.teleop.fresh && result.teleop.stopped,
          "assisted stale teleop diagnostics mismatch");
  require(result.local.has_value() && result.local->near_field_stop &&
              result.local->cmd_vel.vx == 0.0,
          "assisted stale local command must be zeroed");
  require(result.local->final_safety_stopped && !result.local->final_safety_slowed &&
              result.local->final_safety_limited && result.local->final_safety_reason == "stale",
          "assisted stale final-safety diagnostics mismatch");
  require(result.local->reason == "teleop_assisted" && result.local_path.size() == 2,
          "stale override must retain the planner trace for diagnostics");
}

void testDirectSafetyEvaluatesShapedCommandAndCommitsAppliedValue() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.max_speed_mps = 1.0;
  fixture.request = {0.6, 0.0, 0.0};
  fixture.override_shaped_command = true;
  fixture.shaped_command = {0.18, 0.0, 0.0};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.publish.command.vx == 0.18,
          "direct command safety must evaluate the shaped command");
  require(fixture.shape_calls == 1 && fixture.shape_input.vx == 0.6,
          "direct flow must shape the prechecked command once");
  require(fixture.commit_calls == 1 && fixture.committed_command.vx == 0.18,
          "direct flow must commit the safety-applied command");
  require(fixture.now_calls == 1 && fixture.shape_now_s == fixture.commit_now_s,
          "direct shape and commit must share one steady timestamp");
}

void testAssistedSafetyEvaluatesShapedPlannerCommand() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.planner_inputs.obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assisted";
  fixture.planner_output.cmd_vel = {0.3, 0.0, 0.0};
  fixture.planner_output.local_path_map = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  fixture.override_shaped_command = true;
  fixture.shaped_command = {0.12, 0.0, 0.0};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.shape_calls == 1 && fixture.shape_input.vx == 0.3,
          "assisted flow must shape the planner command");
  require(result.publish.command.vx == 0.12 && result.local->cmd_vel.vx == 0.12,
          "the assisted planner command must pass through the limits-only command boundary");
  require(fixture.commit_calls == 1 && fixture.committed_command.vx == 0.12,
          "assisted flow must commit the bounded planner command");
  require(fixture.now_calls == 1 && fixture.shape_now_s == fixture.commit_now_s,
          "assisted shape and commit must share one steady timestamp");
}

void testCommitFailureFailsClosedAndStopsSmoother() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.commit_succeeds = false;
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.publish.cmd_vel && result.publish.command.vx == 0.0,
          "commit failure must replace motion with a zero publish");
  require(result.teleop.stopped && result.teleop.limited &&
              result.teleop.reason == "velocity_smoother_commit_failed",
          "commit failure must expose a stable fail-closed reason");
  require(fixture.commit_calls == 1 && fixture.velocity_stop_calls == 1 &&
              fixture.velocity_stop_reason == "velocity_smoother_commit_failed",
          "commit failure must hard-stop smoother state");
  require(fixture.now_calls == 1 && fixture.commit_now_s == fixture.velocity_stop_now_s,
          "commit failure must reuse the tick timestamp when stopping");
}

void testHardZeroResetsSmootherBeforeNextCommand() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.config.teleop_cmd_max_age_s = 0.35;
  fixture.ages = {0.1, 0.5};
  fixture.shape_from_mock_state = true;
  fixture.smoother_state = {0.4, 0.0, 0.0};
  TeleopTickController controller(fixture.actions, fixture.control());

  const auto stopped = controller.tick(fixture.input());
  require(stopped.teleop.reason == "stale" && fixture.velocity_stop_calls == 1,
          "stale hard zero must reset smoother state");

  fixture.ages = {0.1, 0.1};
  fixture.age_index = 0;
  const auto resumed = controller.tick(fixture.input());
  require(resumed.publish.command.vx == 0.0 && fixture.smoother_state.vx == 0.0,
          "the next target must be shaped from the mock's reset zero state");
}

void testRealSmootherTransitionalZeroRetainsTargetAcrossTicks() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.max_speed_mps = 1.0;
  fixture.safety.min_motion_speed_mps = 0.0;
  fixture.request = {0.5, 0.0, 0.0};
  fixture.now_values = {0.0, 0.05, 0.10, 0.15, 0.20, 0.25};
  nav_kernel::VelocitySmootherConfig smoother_config;
  smoother_config.x.acceleration = 0.1;
  smoother_config.x.deadband = 0.02;
  nav_kernel::VelocitySmoother smoother(smoother_config);
  int applied_commits = 0;
  fixture.final_actions.shape = [&](const nav_kernel::Twist &raw, double now_s) {
    require(smoother.SetTarget(raw, now_s), "real smoother target must be accepted");
    return smoother.Step(now_s);
  };
  fixture.final_actions.commit =
      [&](const nav_kernel::Twist &applied, double now_s) {
        ++applied_commits;
        return smoother.CommitApplied(applied, now_s);
      };
  fixture.final_actions.stop = [&](double now_s, const std::string &reason) {
    ++fixture.velocity_stop_calls;
    (void)smoother.Stop(now_s, reason);
  };
  TeleopTickController controller(fixture.actions, fixture.control());

  std::vector<double> outputs;
  std::vector<bool> published;
  std::vector<int> commit_counts;
  for (std::size_t tick = 0; tick < fixture.now_values.size(); ++tick) {
    const auto result = controller.tick(fixture.input());
    outputs.push_back(result.publish.command.vx);
    published.push_back(result.publish.cmd_vel);
    commit_counts.push_back(applied_commits);
  }

  require(outputs[0] == 0.0 && outputs[1] == 0.0 && outputs[2] == 0.0 &&
              outputs[3] == 0.0,
          "low-rate acceleration below deadband must publish multiple transitional zeros");
  require(published[0] && published[1] && published[2] && published[3],
          "each transitional zero must remain an explicit publish intent");
  require(commit_counts[0] == 0 && commit_counts[1] == 0 && commit_counts[2] == 0 &&
              commit_counts[3] == 0,
          "transitional zeros must not be committed over the smoother's latent ramp");
  require(outputs.back() > 0.0 && applied_commits > 0,
          "the latent ramp must cross deadband and only then commit a nonzero command");
  require(fixture.velocity_stop_calls == 0,
          "a shaped transitional zero must not hard-stop the real smoother");
}

void testRawZeroStillHardStopsSmoother() {
  Fixture raw_zero;
  raw_zero.config.publish_cmd_vel = true;
  raw_zero.request = {};
  TeleopTickController raw_zero_controller(raw_zero.actions, raw_zero.control());

  const auto raw_zero_result = raw_zero_controller.tick(raw_zero.input());

  require(raw_zero_result.publish.command.vx == 0.0 &&
              raw_zero.velocity_stop_calls == 1,
          "an explicit raw zero must hard-stop smoother state");

}

void testPostPlanningStaleInputStopsInsteadOfPublishingNonzeroCommand() {
  InputGateState fresh_gate;
  fresh_gate.ready = true;
  fresh_gate.reason = "ready";
  bool stop_called = false;

  const auto fresh = enforcePostPlanningInputReadiness(
      {0.2, 0.0, 0.0}, fresh_gate, [&](const std::string &) {
        stop_called = true;
        return true;
      });
  require(fresh.allow_publish && !fresh.stop_required && !stop_called,
          "fresh post-planning input must preserve the nonzero publish");

  InputGateState stale_gate;
  stale_gate.ready = false;
  stale_gate.reason = "cloud_stale";
  std::string stop_reason;
  const auto stale = enforcePostPlanningInputReadiness(
      {0.2, 0.0, 0.0}, stale_gate, [&](const std::string &reason) {
        stop_called = true;
        stop_reason = reason;
        return true;
      });
  require(!stale.allow_publish && stale.stop_required && stale.stop_succeeded,
          "stale post-planning input must suppress the nonzero publish through the stop path");
  require(stop_reason == "input_gate_cloud_stale",
          "post-planning stop must preserve the current input-gate reason");

  stop_called = false;
  const auto manual = enforcePostPlanningInputReadiness(
      {0.2, 0.0, 0.0}, stale_gate,
      [&](const std::string &) {
        stop_called = true;
        return true;
      },
      true);
  require(manual.allow_publish && !manual.stop_required && !stop_called,
          "manual mode must not be re-blocked by the post-planning input gate");

  InputGateState driver_stale_gate;
  driver_stale_gate.ready = false;
  driver_stale_gate.reason = "driver_control_stale";
  const auto driver_stale = enforcePostPlanningInputReadiness(
      {0.2, 0.0, 0.0}, driver_stale_gate,
      [&](const std::string &) {
        stop_called = true;
        return true;
      },
      true);
  require(!driver_stale.allow_publish && driver_stale.stop_required,
          "manual mode must still fail closed when driver control is stale");
}

}  // namespace

int main() {
  try {
    testIdleTeleopPublishesAZeroHeartbeatForLateJoiningDriver();
    testIdleAutonomyDoesNotPublishTeleopHeartbeat();
    testBlockedGatePausesPlannerWithoutCancellingIntent();
    testManualModeBypassesUnavailablePlanningInputs();
    testManualModeDoesNotBypassDriverReadiness();
    testDirectCommandIsAcceptedOrLimitedWithoutPlannerSideEffects();
    testPublishTimeStaleOverridesAnEarlierAcceptedDecision();
    testAssistedPathReturnsPlannerArtifactsAndFinalSafetyIntent();
    testCmuDetourPublishesTranslationAndYawWithoutRecovery();
    testVerifiedTeleopRotationPublishesWithoutPath();
    testPlannerSlowdownIsNotAppliedTwice();
    testPlannerTerrainSlowdownDoesNotResetSmootherRamp();
    testPlannerRampBelowOperatorDeadbandKeepsAdvancing();
    testPlannerTerrainScalePassesThroughUnchanged();
    testPlannerAcceptedPathIsNotVetoedByDuplicateFinalSweep();
    testAssistedNoPathFailsClosedAndReplansOriginalIntent();
    testActiveAutonomyPathSuppressesTeleopPublishing();
    testAssistedPublishTimeStaleUpdatesLocalFinalSafety();
    testDirectSafetyEvaluatesShapedCommandAndCommitsAppliedValue();
    testAssistedSafetyEvaluatesShapedPlannerCommand();
    testCommitFailureFailsClosedAndStopsSmoother();
    testHardZeroResetsSmootherBeforeNextCommand();
    testRealSmootherTransitionalZeroRetainsTargetAcrossTicks();
    testRawZeroStillHardStopsSmoother();
    testPostPlanningStaleInputStopsInsteadOfPublishingNonzeroCommand();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_teleop_tick_controller: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}
