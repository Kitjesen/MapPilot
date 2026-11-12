#include "app/navigation.hpp"

#include <stdexcept>
#include <algorithm>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

std::optional<AutonomyTickOutcome>
inspectionCompletion(const GoalReplanRuntimeResult &result) {
  if (!result.terminal_after_stop) {
    return std::nullopt;
  }
  for (const auto &status : result.terminal_after_stop->delivery_ticket.statuses) {
    if (status.origin != GoalPlanOrigin::kInspection) {
      continue;
    }
    if (status.state == lingtu::message::NavigationGoalState::Reached) {
      return AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalReached, status.reason,
                                 false, std::nullopt};
    }
    if (status.state == lingtu::message::NavigationGoalState::Failed) {
      return AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, status.reason,
                                 false, std::nullopt};
    }
  }
  return std::nullopt;
}

}  // namespace

NavigationRuntimeController::NavigationRuntimeController(
    GoalPlanController &goal_plan, GoalReplanRuntimeCoordinator &goal_replan_runtime,
    GoalTerminal &goal_terminal, MotionStopBarrier &motion_stop)
    : goal_plan_(goal_plan),
      goal_replan_runtime_(goal_replan_runtime),
      goal_terminal_(goal_terminal), motion_stop_(motion_stop) {}

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
    const GoalTerminalResult terminal = completeTerminal(result.planning_result);
    result.terminal_delivery_acknowledged |= terminal.delivery_acknowledged;
    if (terminal.delivery_acknowledged) {
      result.inspection_completion = inspectionCompletion(result.planning_result);
    }
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

    actions.apply_autonomy_outputs(runtime_outcome);
    const GoalTerminalSchedulingDecision outcome_schedule =
        decideGoalTerminalScheduling(runtime_outcome, goal_replan_runtime_.terminalPending());
    if (outcome_schedule.service_terminal) {
      const GoalTerminalResult terminal = completeTerminal(runtime_outcome);
      result.terminal_delivery_acknowledged |= terminal.delivery_acknowledged;
      if (terminal.delivery_acknowledged) {
        result.inspection_completion = inspectionCompletion(runtime_outcome);
      }
    }
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
  cancelResume();
  NavigationRuntimeInterruptionResult result;
  result.runtime_result = goal_replan_runtime_.interrupt(interruption, steady_now_s);
  if (result.runtime_result.terminal_after_stop &&
      result.runtime_result.terminal_intent_id != 0U) {
    result.terminal = completeTerminal(result.runtime_result, estop_reason);
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

  result.plan_result = goal_plan_.submit(request, admission);
  if (!result.plan_result.accepted) {
    return result;
  }

  // Only an admitted external goal supersedes the current retry. A terminal
  // from an older deferred replacement still blocks activation, but does not
  // change acceptance of the newly submitted goal.
  if (request.origin == GoalPlanOrigin::kExternal) {
    NavigationRuntimeInterruptionResult preemption =
        interrupt(GoalReplanRuntimeInterruption::kNewGoal, steady_now_s);
    result.preemption_terminal = preemption.terminal;
  }

  return result;
}

GoalTerminalResult
NavigationRuntimeController::completeTerminal(const GoalReplanRuntimeResult &runtime_result,
                                              const std::string &estop_reason) {
  return goal_terminal_.advance(runtime_result, estop_reason);
}

MotionStopTerminalBarrierResult NavigationRuntimeController::stopWhileTerminalPending() {
  return goal_terminal_.stopPending();
}

MotionStopTerminalBarrierResult
NavigationRuntimeController::estopWhileTerminalPending(const std::string &estop_reason) {
  return goal_terminal_.estopPending(estop_reason);
}

CommandAck NavigationRuntimeController::cancelTask(const GoalTaskCancelRequest &request) {
  if (pending_resume_ && pending_resume_->task_id == request.task_id) cancelResume();
  GoalTaskCancelRouter cancel(goal_plan_, goal_replan_runtime_,
      [this](const GoalReplanRuntimeResult &result) {
        const auto serviced = completeTerminal(result);
        return GoalTaskCancelTerminalServiceResult{
            serviced.action_committed, serviced.reason, motion_stop_.confirmationPending()};
      }, [] {});
  return cancel.handle(request);
}

std::pair<bool, std::string> NavigationRuntimeController::pauseTask(
    const std::string &task_id, const std::string &request_id, const std::string &reason) {
  auto transition = goal_plan_.deferPause(task_id, request_id, reason);
  if (!transition.accepted) return {false, transition.reason};
  cancelResume();
  const auto result = motion_stop_.pauseTask(std::move(transition.commit), task_id + ":" + request_id);
  return {result.accepted || result.pending, result.pending ? "pause_requested" : result.reason};
}

std::pair<bool, std::string> NavigationRuntimeController::resumeTask(
    const std::string &task_id, const std::string &request_id, double source_stamp_s,
    GoalPlanAdmissionContext context, GoalResumeActions actions, const std::string &blocker) {
  if (!blocker.empty()) return {false, blocker};
  const auto paused = goal_plan_.snapshot();
  if (paused.paused_task_id != task_id) return {false, "task_not_active"};
  if (!paused.pause_pending && !paused.active_paused) return {false, "task_not_paused"};
  if (paused.pause_pending || motion_stop_.confirmationPending()) {
    pending_resume_ = PendingResume{task_id, request_id, source_stamp_s};
    return {true, "resume_requested"};
  }
  context.hold_reason.clear();
  context.autonomy_request_not_before_s = std::max(context.autonomy_request_not_before_s, source_stamp_s);
  auto transition = goal_plan_.deferResume(task_id, request_id, context);
  if (!transition.accepted) return {false, transition.reason};
  const auto result = motion_stop_.resumeTask(
      [this, commit = std::move(transition.commit), task_id, actions = std::move(actions)] {
        actions.release_operator();
        commit();
        if (goal_plan_.snapshot().active_task_id == task_id && !goal_replan_runtime_.replanPending())
          actions.activate_route();
        actions.changed();
      }, source_stamp_s, task_id + ":" + request_id);
  return {result.accepted || result.pending, result.pending ? "resume_requested" : result.reason};
}

void NavigationRuntimeController::retryResume(
    const std::function<GoalPlanAdmissionContext()> &context, const GoalResumeActions &actions,
    const std::function<std::string()> &blocker) {
  if (!pending_resume_ || motion_stop_.confirmationPending()) return;
  const auto resume = std::move(*pending_resume_);
  pending_resume_.reset();
  (void)resumeTask(resume.task_id, resume.request_id, resume.source_stamp_s,
                   context(), actions, blocker());
}

void NavigationRuntimeController::cancelResume() { pending_resume_.reset(); }

MotionStopResult NavigationRuntimeController::interruptRelated(
    GoalReplanRuntimeInterruption interruption, double now_s, const std::string &reason,
    std::function<bool()> commit) {
  cancelResume();
  return goal_terminal_.commitRelated(goal_replan_runtime_.interrupt(interruption, now_s),
                                      reason, std::move(commit));
}

}  // namespace lingtu::nav::endpoint
