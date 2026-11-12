#include "control/autonomy.hpp"
#include "control/diagnostics.hpp"

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
      !actions_.current_map_identity || !actions_.tick_autonomy ||
      !actions_.pause_linear_motion) {
    throw std::invalid_argument("autonomy tick actions must all be configured");
  }
}

AutonomyTickResult AutonomyTickController::tick(const AutonomyTickInput &input) {
  AutonomyTickResult result;
  const double now_s = actions_.steady_now_s();

  if (input.path_active && (!input.input_gate.ready || !input.motion_allowed))
    actions_.pause_linear_motion();

  if (input.map_body && input.path_active && input.input_gate.ready && input.motion_allowed) {
    result.handled = true;
    const auto current_map = actions_.current_map_identity();
    const std::string map_blocker =
        activeMapIdentityBlocker(input.active_path_map_identity, current_map);
    if (!map_blocker.empty()) {
      const auto final_start = Clock::now();
      final_control_.stop(now_s, map_blocker);
      result.timing.final_control_ms += elapsedMs(final_start);
      result.clear_local_path = true;
      result.clear_local_planner_debug = true;
      auto local = makeStoppedLocalDiagnostics(input.previous_local, map_blocker, map_blocker);
      local.tracking = {};
      result.local = std::move(local);
      result.publish.cmd_vel = input.publish_cmd_vel;
      result.publish.command = {};
      result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
      // A map service lookup can be temporarily unavailable during a reload or
      // DDS reconnect. Hold zero and keep the goal so the next valid identity
      // can resume it; other map identity mismatches still fail closed.
      if (map_blocker != "active_map_unavailable_during_navigation") {
        result.outcome.kind = AutonomyTickOutcomeKind::kGoalFailed;
        result.outcome.reason = map_blocker;
      }
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
    bool safety_applied = false;
    const bool raw_command_zero = isZeroCommand(output.cmd_vel);
    if (output.goal_reached || raw_command_zero) {
      const auto final_start = Clock::now();
      final_control_.stop(now_s, output.goal_reached ? "goal_reached" : "zero_command");
      result.timing.final_control_ms += elapsedMs(final_start);
      output.cmd_vel = {};
    } else {
      CommandSafetyConfig path_safety = input.safety;
      path_safety.min_motion_speed_mps = 0.0;
      path_safety.verified_recovery_translation =
          output.recovery_verified && output.recovery_action ==
              static_cast<int>(nav_kernel::RecoveryAction::Translate);
      const auto final_start = Clock::now();
      const auto final = final_control_.finalize(FinalInput{
          FinalMode::kAutonomyPath,
          path_safety,
          path_follower_cmd,
          false,
          input.publish_cmd_vel,
          now_s,
      });
      result.timing.final_control_ms += elapsedMs(final_start);
      output.cmd_vel = final.decision.cmd;
      // CMU recovery retains final-motion blockage across pending replans.
      const bool motion_blocked = final.decision.reason == "brake_blocked" ||
                                  final.decision.reason == "cmd_blocked";
      if (actions_.report_final_motion_blocked &&
          (motion_blocked ||
           linearSpeed(output.cmd_vel) > 1e-6 || std::abs(output.cmd_vel.wz) > 1e-6)) {
        actions_.report_final_motion_blocked(motion_blocked, now_s);
      }
      if (final.decision.stopped ||
          (linearSpeed(path_follower_cmd) > 1e-6 && linearSpeed(output.cmd_vel) <= 1e-6)) {
        actions_.pause_linear_motion();
        output.trajectory_frozen = true;
        output.tracking.executionFrozen = true;
      }
      if (!final.reason.empty()) {
        output.reason = final.reason;
      }
      final_safety = final.decision;
      safety_applied = final.safety_applied;
    }

    auto local = makeLocalDiagnostics(
        output, final_safety.has_value() ? &*final_safety : nullptr,
        safety_applied);
    local.path_follower_cmd_vel = path_follower_cmd;
    result.local = std::move(local);
    result.publish.local_path = true;
    result.publish.waypoint = true;
    result.publish.cmd_vel = input.publish_cmd_vel;
    result.publish.command = output.cmd_vel;
    result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
    result.delta.output_count = 1U;

    if (output.planning_failed) {
      result.outcome.kind = input.rolling_segment_active
                                ? AutonomyTickOutcomeKind::kRollingRecoveryExhausted
                                : AutonomyTickOutcomeKind::kGoalFailed;
      result.outcome.reason = output.reason;
      result.outcome.terminal_failure_intent = true;
    } else if (output.recovery_exhausted) {
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
    const std::string reason = input.input_gate.reason;
    const std::string stop_reason = inputGateStopReason(reason);
    const auto final_start = Clock::now();
    final_control_.stop(now_s, stop_reason);
    result.timing.final_control_ms += elapsedMs(final_start);
    result.handled = true;
    result.clear_local_path = true;
    result.clear_local_planner_debug = true;
    auto local = makeStoppedLocalDiagnostics(input.previous_local, reason, stop_reason);
    local.tracking.executionFrozen = local.tracking.active;
    result.local = std::move(local);
    result.publish.cmd_vel = input.publish_cmd_vel;
    result.publish.command = {};
    result.delta.cmd_vel_count = input.publish_cmd_vel ? 1U : 0U;
  }

  return result;
}

}  // namespace lingtu::nav::endpoint
