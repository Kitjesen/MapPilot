#include "control/autonomy.hpp"

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

const std::vector<float> &obstaclesOrEmpty(const PlanView &inputs) {
  static const std::vector<float> empty;
  return inputs.obstacles != nullptr ? *inputs.obstacles : empty;
}

LocalDiagnostics activeDiagnostics(const lingtu::nav::navigation::ExecutionOutput &output,
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
  local.recovery_rotation_target_rad = output.recovery_rotation_target_rad;
  local.recovery_verified = output.recovery_verified;
  local.recovery_progress = output.recovery_progress;
  local.recovery_trigger = output.recovery_trigger;
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

std::string activeMapIdentityBlocker(const std::optional<lingtu::nav::plan::MapIdentity> &expected,
                                     const GoalPlanMapIdentityResult &current) {
  if (!expected || !expected->valid()) {
    return "active_path_map_identity_missing";
  }
  if (!current.identity || !current.identity->valid()) {
    return "active_map_unavailable_during_navigation";
  }
  if (!lingtu::nav::plan::sameMapIdentity(*expected, *current.identity)) {
    return "active_map_changed_during_navigation";
  }
  return {};
}

bool isZeroCommand(const nav_kernel::Twist &command) {
  return linearSpeed(command) <= 1e-6 && std::abs(command.wz) <= 1e-6;
}
}  // namespace

AutonomyTickController::AutonomyTickController(AutonomyTickActions actions,
                                               FinalControl &final_control)
    : actions_(std::move(actions)), final_control_(final_control) {
  if (!actions_.steady_now_s || !actions_.read_plan ||
      !actions_.current_map_identity || !actions_.tick_autonomy || !actions_.stop_linear_motion) {
    throw std::invalid_argument("autonomy tick actions must all be configured");
  }
}

AutonomyTickResult AutonomyTickController::tick(const AutonomyTickInput &input) {
  AutonomyTickResult result;
  const double now_s = actions_.steady_now_s();

  if (input.map_body && input.path_active && input.input_gate.ready && input.motion_allowed) {
    result.handled = true;
    const auto current_map = actions_.current_map_identity();
    const std::string map_blocker =
        activeMapIdentityBlocker(input.active_path_map_identity, current_map);
    if (!map_blocker.empty()) {
      final_control_.stop(now_s, map_blocker);
      result.clear_local_path = true;
      result.clear_local_planner_debug = true;
      auto local = input.previous_local;
      local.seen = true;
      local.active = false;
      local.path_found = false;
      local.near_field_stop = true;
      local.reason = map_blocker;
      local.final_safety_applied = false;
      local.final_safety_stopped = true;
      local.final_safety_slowed = false;
      local.final_safety_limited = false;
      local.final_safety_reason = map_blocker;
      local.final_safety_obstacle_distance_m = -1.0;
      local.final_safety_traversability_cost = -1.0;
      local.path_follower_cmd_vel = {};
      local.cmd_vel = {};
      result.local = std::move(local);
      result.publish.cmd_vel = input.publish_cmd_vel;
      result.publish.command = {};
      result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
      result.outcome.kind = AutonomyTickOutcomeKind::kGoalFailed;
      result.outcome.reason = map_blocker;
      return result;
    }
    if (input.precomputed_replan_trigger) {
      const GoalReplanTrigger &trigger = *input.precomputed_replan_trigger;
      const std::string reason =
          trigger.reason.empty() ? "persistent_path_obstruction" : trigger.reason;
      actions_.stop_linear_motion();
      final_control_.stop(now_s, reason);
      result.clear_local_path = true;
      result.clear_local_planner_debug = true;
      auto local = input.previous_local;
      local.seen = true;
      local.active = false;
      local.path_found = false;
      local.near_field_stop = true;
      local.reason = reason;
      local.final_safety_applied = false;
      local.final_safety_stopped = true;
      local.final_safety_slowed = false;
      local.final_safety_limited = false;
      local.final_safety_reason = reason;
      local.final_safety_obstacle_distance_m = -1.0;
      local.final_safety_traversability_cost = -1.0;
      local.path_follower_cmd_vel = {};
      local.cmd_vel = {};
      result.local = std::move(local);
      result.publish.cmd_vel = input.publish_cmd_vel;
      result.publish.command = {};
      result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
      result.outcome.kind = AutonomyTickOutcomeKind::kGoalFailed;
      result.outcome.reason = reason;
      result.outcome.replan_trigger = trigger;
      return result;
    }
    const auto planner_inputs = actions_.read_plan(now_s, input.timing);
    const bool has_obstacles =
        planner_inputs.obstacles != nullptr && !planner_inputs.obstacles->empty();
    const auto nav_tick_start = Clock::now();
    auto output = actions_.tick_autonomy(
        *input.map_body, has_obstacles ? planner_inputs.obstacles->data() : nullptr,
        has_obstacles ? static_cast<int>(planner_inputs.obstacles->size() / 4U) : 0,
        now_s, planner_inputs.traversability);
    result.timing.nav_tick_measured = true;
    result.timing.nav_tick_ms = elapsedMs(nav_tick_start);

    const nav_kernel::Twist path_follower_cmd = output.cmd_vel;
    std::optional<CommandSafetyDecision> final_safety;
    const bool raw_command_zero = isZeroCommand(output.cmd_vel);
    if (output.goal_reached || raw_command_zero) {
      final_control_.stop(now_s, output.goal_reached ? "goal_reached" : "zero_command");
      output.cmd_vel = {};
    } else {
      const auto final = final_control_.finalize(FinalInput{
          FinalMode::kAutonomyPath,
          input.safety,
          path_follower_cmd,
          0.0,
          0.0,
          false,
          input.publish_cmd_vel,
          now_s,
      });
      output.cmd_vel = final.decision.cmd;
      if (!final.reason.empty()) {
        output.reason = final.reason;
      }
      if (final.safety_applied) {
        final_safety = final.decision;
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

    if (output.recovery_exhausted) {
      result.outcome.reason = output.reason.empty() ? "local_recovery_exhausted" : output.reason;
      result.outcome.kind = input.rolling_segment_active
                                ? AutonomyTickOutcomeKind::kRollingRecoveryExhausted
                                : AutonomyTickOutcomeKind::kGoalFailed;
      if (!input.rolling_segment_active) {
        GoalReplanTrigger trigger;
        trigger.kind = GoalReplanTriggerKind::kLocalRecoveryExhausted;
        trigger.reason = result.outcome.reason;
        if (input.active_goal_identity) {
          trigger.goal = *input.active_goal_identity;
        }
        result.outcome.replan_trigger = std::move(trigger);
      }
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
    final_control_.stop(now_s, std::string("input_gate_") + input.input_gate.reason);
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
