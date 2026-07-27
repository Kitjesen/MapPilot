#include "motion/autonomy_tick_controller.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

const std::vector<float> &obstaclesOrEmpty(const AutonomyTickPlannerInputs &inputs) {
  static const std::vector<float> empty;
  return inputs.planner_obstacles != nullptr ? *inputs.planner_obstacles : empty;
}

LocalDiagnostics activeDiagnostics(const lingtu::nav::plan::NavLoopOutput &output,
                                   const CommandSafetyDecision *final_safety) {
  LocalDiagnostics local;
  local.seen = true;
  local.active = output.active;
  local.goal_reached = output.goal_reached;
  local.path_found = output.path_found;
  local.near_field_stop = output.near_field_stop;
  local.reason = output.reason;
  local.slow_down = output.slow_down;
  local.recovery_state = output.recovery_state;
  local.recovery_action = output.recovery_action;
  local.recovery_attempt = output.recovery_attempt;
  local.recovery_candidate_count = output.recovery_candidate_count;
  local.recovery_verified = output.recovery_verified;
  local.recovery_progress = output.recovery_progress;
  local.recovery_reason = output.recovery_reason;
  local.recovery_exhausted = output.recovery_exhausted;
  local.target_index = output.target_index;
  local.target_distance_m = output.target_distance_m;
  local.target = output.target;
  local.local_path_points = output.local_path_map.size();
  local.path_follower_cmd_vel = output.cmd_vel;
  local.cmd_vel = output.cmd_vel;
  local.final_safety_applied = final_safety != nullptr;
  local.final_safety_stopped = final_safety != nullptr && final_safety->stopped;
  local.final_safety_slowed = final_safety != nullptr && final_safety->slowed;
  local.final_safety_limited = final_safety != nullptr && final_safety->limited;
  local.final_safety_reason = final_safety != nullptr ? final_safety->reason : "zero_command";
  local.final_safety_obstacle_distance_m =
      final_safety != nullptr ? final_safety->obstacle_distance_m : -1.0;
  local.final_safety_traversability_cost =
      final_safety != nullptr ? final_safety->traversability_cost : -1.0;
  return local;
}

}  // namespace

AutonomyTickController::AutonomyTickController(AutonomyTickActions actions)
    : actions_(std::move(actions)) {
  if (!actions_.steady_now_s || !actions_.compute_planner_inputs || !actions_.tick_autonomy ||
      !actions_.evaluate_path_safety || !actions_.evaluate_command_safety ||
      !actions_.stop_linear_motion) {
    throw std::invalid_argument("autonomy tick actions must all be configured");
  }
}

AutonomyTickResult AutonomyTickController::tick(const AutonomyTickInput &input) {
  AutonomyTickResult result;

  if (input.map_body && input.path_active && input.input_gate.ready && input.motion_allowed) {
    result.handled = true;
    const auto planner_inputs = actions_.compute_planner_inputs(input.timing);
    const bool has_obstacles =
        planner_inputs.planner_obstacles != nullptr && !planner_inputs.planner_obstacles->empty();
    const auto nav_tick_start = Clock::now();
    auto output = actions_.tick_autonomy(
        *input.map_body, has_obstacles ? planner_inputs.planner_obstacles->data() : nullptr,
        has_obstacles ? static_cast<int>(planner_inputs.planner_obstacles->size() / 4U) : 0,
        actions_.steady_now_s(), planner_inputs.traversability_view);
    result.timing.nav_tick_measured = true;
    result.timing.nav_tick_ms = elapsedMs(nav_tick_start);

    const nav_kernel::Twist path_follower_cmd = output.cmd_vel;
    std::optional<CommandSafetyDecision> final_safety;
    std::string rolling_final_safety_reason;
    const bool has_command =
        linearSpeed(output.cmd_vel) > 1e-6 || std::abs(output.cmd_vel.wz) > 1e-6;
    if (has_command) {
      final_safety =
          actions_.evaluate_path_safety(input.safety, output.cmd_vel, input.map_body,
                                        output.local_path_map, obstaclesOrEmpty(planner_inputs),
                                        input.traversability, planner_inputs.traversability_fresh);
      if (final_safety->stopped) {
        actions_.stop_linear_motion();
        if (std::abs(output.cmd_vel.wz) > 1e-6) {
          const nav_kernel::Twist rotation_only{0.0, 0.0, output.cmd_vel.wz};
          const auto rotation_safety = actions_.evaluate_command_safety(
              input.safety, rotation_only, 0.0, input.map_body, obstaclesOrEmpty(planner_inputs),
              input.traversability, planner_inputs.traversability_fresh);
          if (!rotation_safety.stopped && std::abs(rotation_safety.cmd.wz) > 1e-6) {
            final_safety->cmd = rotation_safety.cmd;
            final_safety->limited = true;
            final_safety->reason += "_rotation_only";
          }
        }
      }
      output.cmd_vel = final_safety->cmd;
      if (final_safety->stopped) {
        output.reason = std::string("final_safety_") + final_safety->reason;
        if (input.rolling_segment_active) {
          rolling_final_safety_reason = std::string("segment_final_safety_") + final_safety->reason;
          output.cmd_vel = {};
          output.reason = rolling_final_safety_reason;
        }
      }
    }

    auto local = activeDiagnostics(output, final_safety.has_value() ? &*final_safety : nullptr);
    local.path_follower_cmd_vel = path_follower_cmd;
    local.cmd_vel = output.cmd_vel;
    local.reason = output.reason;
    result.local = std::move(local);
    result.publish.local_path = true;
    result.publish.waypoint = true;
    result.publish.cmd_vel = input.publish_cmd_vel;
    result.publish.command = output.cmd_vel;
    result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
    result.delta.output_count = 1U;

    if (!rolling_final_safety_reason.empty()) {
      result.outcome.kind = AutonomyTickOutcomeKind::kRollingFinalSafetyStopped;
      result.outcome.reason = rolling_final_safety_reason;
    } else if (output.recovery_exhausted) {
      result.outcome.reason = output.reason.empty() ? "local_recovery_exhausted" : output.reason;
      result.outcome.kind = input.rolling_segment_active
                                ? AutonomyTickOutcomeKind::kRollingRecoveryExhausted
                                : AutonomyTickOutcomeKind::kGoalFailed;
    } else if (output.goal_reached) {
      if (input.rolling_segment_active) {
        result.outcome.kind = AutonomyTickOutcomeKind::kRollingReached;
        result.outcome.reason = "segment_reached";
      } else {
        result.outcome.kind = AutonomyTickOutcomeKind::kGoalReached;
        result.outcome.reason = "goal_reached";
        result.outcome.inspection_arrival_intent = true;
      }
    }

    result.output = std::move(output);
    return result;
  }

  if (input.path_active && !input.input_gate.ready) {
    result.handled = true;
    result.clear_local_path = true;
    result.clear_local_planner_debug = true;
    auto local = input.previous_local;
    local.seen = true;
    local.active = false;
    local.path_found = false;
    local.near_field_stop = true;
    local.reason = input.input_gate.reason;
    local.final_safety_applied = false;
    local.final_safety_stopped = true;
    local.final_safety_slowed = false;
    local.final_safety_limited = false;
    local.final_safety_reason = std::string("input_gate_") + input.input_gate.reason;
    local.final_safety_obstacle_distance_m = -1.0;
    local.final_safety_traversability_cost = -1.0;
    local.path_follower_cmd_vel = {};
    local.cmd_vel = {};
    result.local = std::move(local);
    result.publish.cmd_vel = input.publish_cmd_vel;
    result.publish.command = {};
    result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
  }

  return result;
}

}  // namespace lingtu::nav::endpoint
