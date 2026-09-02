#include "control/teleop.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

using Clock = std::chrono::steady_clock;

const std::vector<float> kEmptyObstacles;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool isZeroCommand(const nav_kernel::Twist &command) {
  return command.vx == 0.0 && command.vy == 0.0 && command.wz == 0.0;
}

bool isVerifiedRotation(
    const lingtu::nav::navigation::ExecutionOutput &assisted) {
  return assisted.recovery_state == 1 && assisted.recovery_verified &&
         assisted.recovery_action ==
             static_cast<int>(nav_kernel::RecoveryAction::Rotate) &&
         linearSpeed(assisted.cmd_vel) <= 1e-6 &&
         std::abs(assisted.cmd_vel.wz) > 1e-6;
}

CommandSafetyDecision failClosedDecision(const std::string &reason) {
  CommandSafetyDecision decision;
  decision.should_publish = true;
  decision.stopped = true;
  decision.limited = true;
  decision.reason = reason;
  return decision;
}

}  // namespace

TeleopTickController::TeleopTickController(TeleopTickActions actions,
                                           FinalControl &final_control)
    : actions_(std::move(actions)), final_control_(final_control) {
  if (!actions_.teleop_receive_age_s || !actions_.steady_now_s ||
      !actions_.read_plan || !actions_.tick_teleop_intent ||
      !actions_.pause_linear_motion || !actions_.replan_motion ||
      !actions_.stop_linear_motion) {
    throw std::invalid_argument("teleop tick actions must all be configured");
  }
}

TeleopTickResult TeleopTickController::tick(const TeleopTickInput &input) {
  const auto tick_start = Clock::now();
  TeleopTickResult result;
  if (input.active_request == nullptr) {
    const bool idle_teleop =
        !input.path_active &&
        (input.config.control_mode == ControlMode::Teleop ||
         input.config.control_mode == ControlMode::TeleopAvoid);
    if (idle_teleop) {
      result.handled = true;
      result.teleop = input.previous_teleop;
      result.teleop.fresh = false;
      result.teleop.published = input.config.publish_cmd_vel;
      result.teleop.stopped = true;
      result.teleop.slowed = false;
      result.teleop.limited = false;
      result.teleop.reason = "idle";
      result.teleop.output = {};
      result.publish.cmd_vel = input.config.publish_cmd_vel;
      result.publish.command = {};
      result.delta.cmd_vel_count = input.config.publish_cmd_vel ? 1U : 0U;
    }
    result.timing.teleop_gate_ms = elapsedMs(tick_start);
    return result;
  }

  result.handled = true;
  const double tick_now = actions_.steady_now_s();
  bool velocity_stopped = false;
  bool linear_motion_paused = false;
  bool motion_replan_requested = false;
  bool linear_motion_stopped = false;
  const auto stop_velocity = [&](const std::string &reason) {
    if (!velocity_stopped) {
      final_control_.stop(tick_now, reason);
      velocity_stopped = true;
    }
  };
  const auto stop_linear_motion = [&] {
    if (!linear_motion_stopped) {
      actions_.stop_linear_motion();
      linear_motion_paused = true;
      linear_motion_stopped = true;
    }
  };
  const auto pause_linear_motion = [&] {
    if (!linear_motion_paused && !linear_motion_stopped) {
      actions_.pause_linear_motion();
      linear_motion_paused = true;
    }
  };
  const auto replan_motion = [&] {
    if (!motion_replan_requested && !linear_motion_stopped) {
      actions_.replan_motion();
      linear_motion_paused = true;
      motion_replan_requested = true;
    }
  };
  const bool manual_input_bypass =
      input.manual_mode && manualModeMayBypassInputGate(input.input_gate);
  if (!input.path_active && !input.input_gate.ready && !manual_input_bypass) {
    result.teleop = input.previous_teleop;
    if (input.config.teleop_local_planner) {
      pause_linear_motion();
    }
    result.teleop.seen = true;
    result.teleop.manual_mode = input.manual_mode;
    result.teleop.fresh = false;
    result.teleop.published = input.config.publish_cmd_vel;
    result.teleop.stopped = true;
    result.teleop.slowed = false;
    result.teleop.limited = true;
    result.teleop.reason = input.input_gate.reason;
    result.teleop.output = {};
    result.publish.cmd_vel = input.config.publish_cmd_vel;
    result.delta.cmd_vel_count = input.config.publish_cmd_vel ? 1U : 0U;
    result.delta.teleop_output_count = input.config.publish_cmd_vel ? 1U : 0U;
    result.delta.teleop_stop_count = 1U;
    result.delta.teleop_limited_count = 1U;
    stop_velocity(result.teleop.reason);
  } else if (!input.path_active) {
    const double age_s = actions_.teleop_receive_age_s();
    double publish_age_s = age_s;
    bool publish_age_sampled = false;
    const auto sample_publish_age = [&]() {
      if (!publish_age_sampled) {
        publish_age_s = actions_.teleop_receive_age_s();
        publish_age_sampled = true;
      }
      return publish_age_s;
    };
    const auto precheck =
        arbitrateTeleopCommand(input.safety, *input.active_request, age_s);
    auto decision = precheck;
    bool hard_zero_requested = isZeroCommand(precheck.cmd);
    const bool use_assisted_planner =
        !input.manual_mode && input.config.teleop_local_planner && input.map_body &&
        precheck.should_publish &&
        !precheck.stopped && linearSpeed(precheck.cmd) >= input.config.teleop_min_motion_speed_mps;
    if (use_assisted_planner) {
      const auto planner_inputs = actions_.read_plan(tick_now, input.timing);
      const auto *planner_obstacles = planner_inputs.obstacles;
      const auto nav_tick_start = Clock::now();
      auto assisted = actions_.tick_teleop_intent(
          *input.map_body, precheck.cmd,
          planner_obstacles != nullptr ? planner_obstacles->data() : nullptr,
          planner_obstacles != nullptr ? static_cast<int>(planner_obstacles->size() / 4) : 0,
          tick_now, planner_inputs.traversability);
      result.timing.nav_tick_measured = true;
      result.timing.nav_tick_ms = elapsedMs(nav_tick_start);

      const bool path_ready =
          assisted.path_found && assisted.local_path_map.size() >= 2;
      const bool rotation_ready = isVerifiedRotation(assisted);
      if (path_ready || rotation_ready) {
        hard_zero_requested = isZeroCommand(assisted.cmd_vel);
        CommandSafetyConfig path_safety = input.safety;
        path_safety.min_motion_speed_mps = 0.0;
        const auto final = final_control_.finalize(FinalInput{
            FinalMode::kTeleopPath,
            path_safety,
            assisted.cmd_vel,
            sample_publish_age(),
            input.config.teleop_cmd_max_age_s,
            precheck.limited,
            input.config.publish_cmd_vel,
            tick_now,
        });
        decision = final.decision;
        velocity_stopped = velocity_stopped || final.smoother_stopped;
        if (decision.stopped) {
          if (path_ready && std::abs(decision.cmd.wz) > 1e-6) {
            pause_linear_motion();
          } else {
            replan_motion();
          }
        }
        if (!decision.stopped && decision.reason == "accepted") {
          decision.reason = assisted.reason;
        }
        if (!decision.stopped && assisted.slow_down > 0) {
          decision.slowed = true;
        }
      } else {
        decision = failClosedDecision("teleop_assist_no_path");
        decision.reason = assisted.reason.empty() ? "teleop_assist_no_path" : assisted.reason;
        replan_motion();
      }

      LocalDiagnostics local;
      local.seen = true;
      local.active = assisted.active;
      local.goal_reached = false;
      local.path_found = assisted.path_found;
      local.near_field_stop = assisted.near_field_stop || decision.stopped;
      local.reason = assisted.reason;
      local.slow_down = assisted.slow_down;
      local.recovery_state = assisted.recovery_state;
      local.recovery_action = assisted.recovery_action;
      local.recovery_attempt = assisted.recovery_attempt;
      local.recovery_candidate_count = assisted.recovery_candidate_count;
      local.recovery_rotation_target_rad =
          assisted.recovery_rotation_target_rad;
      local.recovery_verified = assisted.recovery_verified;
      local.recovery_progress = assisted.recovery_progress;
      local.recovery_trigger = assisted.recovery_trigger;
      local.recovery_reason = assisted.recovery_reason;
      local.recovery_exhausted = assisted.recovery_exhausted;
      local.target_index = 0;
      local.target_distance_m = assisted.target_distance_m;
      local.target = assisted.target;
      local.local_path_points = assisted.local_path_map.size();
      local.path_follower_cmd_vel = assisted.cmd_vel;
      local.cmd_vel = decision.cmd;
      local.final_safety_applied = true;
      local.final_safety_stopped = decision.stopped;
      local.final_safety_slowed = decision.slowed;
      local.final_safety_limited = decision.limited;
      local.final_safety_reason = decision.reason;
      local.final_safety_obstacle_distance_m = decision.obstacle_distance_m;
      local.final_safety_traversability_cost = decision.traversability_cost;
      result.local = std::move(local);
      result.local_path = std::move(assisted.local_path_map);
      result.local_planner_debug = std::move(assisted.local_planner_debug);
      result.waypoint = assisted.target;
      result.publish.local_path = true;
      result.publish.waypoint = true;
      result.delta.output_count = 1U;
    } else {
      if (precheck.stopped || !precheck.should_publish) {
        decision = precheck;
      } else {
        const auto final = final_control_.finalize(FinalInput{
            FinalMode::kTeleopCommand,
            input.safety,
            precheck.cmd,
            sample_publish_age(),
            input.config.teleop_cmd_max_age_s,
            precheck.limited,
            input.config.publish_cmd_vel,
            tick_now,
        });
        decision = final.decision;
        velocity_stopped = velocity_stopped || final.smoother_stopped;
      }
      if (input.manual_mode && decision.should_publish && !decision.stopped) {
        decision.reason = "manual_mode";
      }
      if (input.config.teleop_local_planner) {
        stop_linear_motion();
      }
    }

    (void)sample_publish_age();
    if (input.config.teleop_cmd_max_age_s > 0.0 &&
        publish_age_s > input.config.teleop_cmd_max_age_s) {
      decision = {};
      decision.should_publish = true;
      decision.stopped = true;
      decision.limited = true;
      decision.reason = "stale";
      if (input.config.teleop_local_planner) {
        stop_linear_motion();
      }
      if (use_assisted_planner && result.local) {
        result.local->near_field_stop = true;
        result.local->cmd_vel = {};
        result.local->final_safety_stopped = true;
        result.local->final_safety_slowed = false;
        result.local->final_safety_limited = true;
        result.local->final_safety_reason = "stale";
      }
    }

    if (decision.stopped || hard_zero_requested) {
      stop_velocity(decision.reason);
    }

    result.teleop.seen = true;
    result.teleop.manual_mode = input.manual_mode;
    result.teleop.fresh = input.config.teleop_cmd_max_age_s <= 0.0 ||
                          publish_age_s <= input.config.teleop_cmd_max_age_s;
    result.teleop.age_s = publish_age_s;
    result.teleop.request = *input.active_request;
    result.teleop.output = decision.cmd;
    result.teleop.published = input.config.publish_cmd_vel && decision.should_publish;
    result.teleop.stopped = decision.stopped;
    result.teleop.slowed = decision.slowed;
    result.teleop.limited = decision.limited;
    result.teleop.reason = decision.reason;
    result.teleop.obstacle_distance_m = decision.obstacle_distance_m;
    result.teleop.traversability_cost = decision.traversability_cost;
    result.publish.cmd_vel = result.teleop.published;
    result.publish.command = decision.cmd;
    result.delta.cmd_vel_count = result.publish.cmd_vel ? 1U : 0U;
    result.delta.teleop_output_count = result.publish.cmd_vel ? 1U : 0U;
    result.delta.teleop_stop_count = decision.stopped ? 1U : 0U;
    result.delta.teleop_slow_count = decision.slowed ? 1U : 0U;
    result.delta.teleop_limited_count = decision.limited ? 1U : 0U;
  } else {
    result.teleop = input.previous_teleop;
    result.teleop.seen = true;
    result.teleop.fresh = true;
    result.teleop.published = false;
    result.teleop.reason = "auto_active";
  }

  result.timing.teleop_gate_ms = elapsedMs(tick_start);
  return result;
}

}  // namespace lingtu::nav::endpoint
