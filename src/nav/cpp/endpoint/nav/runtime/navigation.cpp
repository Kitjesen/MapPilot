#include "runtime/navigation.hpp"

#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

NavigationRuntimeController::NavigationRuntimeController(
    GoalPlanController &goal_plan, GoalReplanRuntimeCoordinator &goal_replan_runtime,
    GoalTerminalTransaction &goal_terminal_transaction)
    : goal_plan_(goal_plan),
      goal_replan_runtime_(goal_replan_runtime),
      goal_terminal_transaction_(goal_terminal_transaction) {}

NavigationRuntimeFrameResult
NavigationRuntimeController::advanceFrame(const GoalReplanRuntimeFrameInput &frame,
                                          const NavigationRuntimeFrameActions &actions) {
  if (!actions.complete_endpoint_work_before_autonomy || !actions.run_autonomy ||
      !actions.apply_autonomy_outputs) {
    throw std::invalid_argument("NavigationRuntimeController requires complete frame actions");
  }

  NavigationRuntimeFrameResult result;
  result.planning_result = goal_replan_runtime_.advancePlanningCycle(frame);
  const GoalTerminalSchedulingDecision planning_schedule =
      decideGoalTerminalScheduling(result.planning_result, goal_replan_runtime_.terminalPending());
  if (planning_schedule.service_terminal) {
    const GoalTerminalTransactionResult terminal = completeTerminal(result.planning_result);
    result.terminal_delivery_acknowledged |= terminal.delivery_acknowledged;
  }

  actions.complete_endpoint_work_before_autonomy(result.planning_result);

  if (planning_schedule.run_autonomy_tick) {
    const GoalPlanSnapshot pre_autonomy_goal_snapshot = goal_plan_.snapshot();
    const NavigationRuntimeAutonomyObservation observation =
        actions.run_autonomy(pre_autonomy_goal_snapshot);
    GoalReplanRuntimeResult runtime_outcome = goal_replan_runtime_.handleAutonomyOutcome(
        observation.updated_frame,
        GoalReplanRuntimeAutonomyEvent{observation.outcome, pre_autonomy_goal_snapshot,
                                       observation.inspection_active,
                                       observation.rolling_segment_active});
    result.autonomy_result = runtime_outcome;

    const NavigationRuntimePostAutonomyState post_state =
        actions.apply_autonomy_outputs(runtime_outcome);
    std::optional<GoalReplanRuntimeResult> inspection_fallback_terminal;
    if (observation.autonomy_tick_handled && !deferred_inspection_completion_ &&
        shouldDeferInspectionCompletion(observation.outcome, runtime_outcome, post_state)) {
      deferred_inspection_completion_ = observation.outcome;
      inspection_fallback_terminal = goal_replan_runtime_.interrupt(
          GoalReplanRuntimeInterruption::kControlHold, observation.updated_frame.steady_now_s);
    }

    const GoalReplanRuntimeResult &terminal_candidate =
        inspection_fallback_terminal ? *inspection_fallback_terminal : runtime_outcome;
    const GoalTerminalSchedulingDecision outcome_schedule =
        decideGoalTerminalScheduling(terminal_candidate, goal_replan_runtime_.terminalPending());
    if (outcome_schedule.service_terminal) {
      const GoalTerminalTransactionResult terminal = completeTerminal(terminal_candidate);
      result.terminal_delivery_acknowledged |= terminal.delivery_acknowledged;
    }
  }

  if (result.terminal_delivery_acknowledged && deferred_inspection_completion_) {
    result.inspection_completion = std::move(deferred_inspection_completion_);
    deferred_inspection_completion_.reset();
  }

  if (!goal_replan_runtime_.terminalPending()) {
    result.pending_result = goal_replan_runtime_.drainPendingCycle(frame);
    result.pending_cycle_advanced = true;
  }
  return result;
}

bool NavigationRuntimeController::terminalPending() const {
  return goal_replan_runtime_.terminalPending();
}

NavigationRuntimeInterruptionResult NavigationRuntimeController::interrupt(
    GoalReplanRuntimeInterruption interruption, double steady_now_s,
    const std::string &estop_reason) {
  NavigationRuntimeInterruptionResult result;
  result.runtime_result = goal_replan_runtime_.interrupt(interruption, steady_now_s);
  if (result.runtime_result.terminal_after_stop &&
      result.runtime_result.terminal_intent_id != 0U) {
    result.terminal_transaction = completeTerminal(result.runtime_result, estop_reason);
  }
  return result;
}

NavigationRuntimeGoalSubmissionResult NavigationRuntimeController::submitGoal(
    const GoalPlanRequest &request, const GoalPlanAdmissionContext &admission,
    double steady_now_s) {
  NavigationRuntimeGoalSubmissionResult result;

  // A terminal is a hard admission barrier.  The caller must replay and
  // acknowledge the exact terminal intent before any new request can enter
  // GoalPlanController.
  if (terminalPending()) {
    result.plan_result = {false, "goal_terminal_pending", false, false, false, std::nullopt};
    return result;
  }

  // External goals replace the current autonomous goal.  Keep this
  // preemption and the subsequent submit in one controller transaction so a
  // surfaced terminal cannot be accidentally followed by a same-frame goal.
  if (request.origin == GoalPlanOrigin::kExternal) {
    NavigationRuntimeInterruptionResult preemption =
        interrupt(GoalReplanRuntimeInterruption::kNewGoal, steady_now_s);
    result.preemption_terminal = preemption.terminal_transaction;
    if (preemption.runtime_result.terminal_after_stop || terminalPending()) {
      result.plan_result = {false, "goal_terminal_pending", false, false, false, std::nullopt};
      return result;
    }
  }

  result.plan_result = goal_plan_.submit(request, admission);
  return result;
}

GoalTerminalTransactionResult
NavigationRuntimeController::completeTerminal(const GoalReplanRuntimeResult &runtime_result,
                                              const std::string &estop_reason) {
  return goal_terminal_transaction_.advance(runtime_result, estop_reason);
}

MotionStopTerminalBarrierResult NavigationRuntimeController::stopWhileTerminalPending() {
  return goal_terminal_transaction_.stopWhileTerminalPending();
}

MotionStopTerminalBarrierResult
NavigationRuntimeController::estopWhileTerminalPending(const std::string &estop_reason) {
  return goal_terminal_transaction_.estopWhileTerminalPending(estop_reason);
}

bool NavigationRuntimeController::shouldDeferInspectionCompletion(
    const AutonomyTickOutcome &outcome, const GoalReplanRuntimeResult &runtime_result,
    const NavigationRuntimePostAutonomyState &post_state) {
  if (runtime_result.handled) {
    return false;
  }
  if (outcome.kind == AutonomyTickOutcomeKind::kGoalFailed) {
    return post_state.inspection_navigation_active;
  }
  return outcome.kind == AutonomyTickOutcomeKind::kGoalReached &&
         outcome.inspection_arrival_intent && post_state.inspection_active;
}

}  // namespace lingtu::nav::endpoint
