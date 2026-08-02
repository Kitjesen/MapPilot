#include "plan/goal_replan_runtime_coordinator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "motion/motion_stop_coordinator.hpp"

namespace lingtu::nav::endpoint {
namespace {

constexpr const char *kGoalReachedReason = "goal_reached";

bool sameMap(const std::optional<lingtu::nav::plan::MapIdentity> &lhs,
             const std::optional<lingtu::nav::plan::MapIdentity> &rhs) {
  return lhs && rhs && lhs->valid() && rhs->valid() &&
         lingtu::nav::plan::sameMapIdentity(*lhs, *rhs);
}

bool triggerMatchesSnapshot(const GoalReplanTrigger &trigger, const GoalPlanSnapshot &snapshot) {
  return trigger.goal.valid() && trigger.goal.task_id == snapshot.active_task_id &&
         trigger.goal.request_id == snapshot.active_request_id &&
         trigger.goal.goal_epoch == snapshot.active_goal_epoch && snapshot.active_map_identity &&
         snapshot.active_map_identity->valid() &&
         lingtu::nav::plan::sameMapIdentity(trigger.goal.map_identity,
                                            *snapshot.active_map_identity);
}

bool validPersistentOverlay(const GoalReplanTrigger &trigger, std::uint64_t frame_epoch) {
  const auto &overlay = trigger.temporary_overlay;
  return trigger.kind == GoalReplanTriggerKind::kPersistentPathObstruction && !overlay.empty() &&
         overlay.revision != 0U && overlay.frame_epoch != 0U &&
         overlay.frame_epoch == frame_epoch && overlay.obstacle_generation != 0U &&
         overlay.traversability_generation != 0U;
}

bool validLocalRecoveryTrigger(const GoalReplanTrigger &trigger) {
  const auto &overlay = trigger.temporary_overlay;
  return trigger.kind == GoalReplanTriggerKind::kLocalRecoveryExhausted && overlay.empty() &&
         overlay.revision == 0U && overlay.frame_epoch == 0U && overlay.obstacle_generation == 0U &&
         overlay.traversability_generation == 0U;
}

const char *replanStopReason(GoalReplanTriggerKind kind) {
  return kind == GoalReplanTriggerKind::kPersistentPathObstruction
             ? "persistent_path_obstruction_replan"
             : "local_recovery_replan";
}

lingtu::message::NavigationGoalState deferredReplacementAdmissionState(const std::string &reason) {
  if (reason == "map_drift" || reason == "invalid_replan_admission" ||
      reason == "map_odom_tf_not_ready" || reason == "odometry_not_ready" ||
      reason.rfind("input_gate_", 0) == 0 || reason.find("map") != std::string::npos ||
      reason.find("odom") != std::string::npos) {
    return lingtu::message::NavigationGoalState::Failed;
  }
  return lingtu::message::NavigationGoalState::Cancelled;
}

std::string terminalTaskId(const GoalPlanTerminalAfterStop &terminal) {
  if (terminal.delivery_ticket.statuses.empty()) {
    return {};
  }
  return terminal.delivery_ticket.statuses.front().task_id;
}
std::string interruptionReason(GoalReplanRuntimeInterruption interruption) {
  switch (interruption) {
    case GoalReplanRuntimeInterruption::kNewGoal:
      return "superseded_by_new_goal";
    case GoalReplanRuntimeInterruption::kCancel:
      return "cancelled";
    case GoalReplanRuntimeInterruption::kStop:
      return "stopped";
    case GoalReplanRuntimeInterruption::kEstop:
      return "estop_latched";
    case GoalReplanRuntimeInterruption::kShutdown:
      return "navd_shutdown";
    case GoalReplanRuntimeInterruption::kOperatorTakeover:
      return "operator_takeover_resume_required";
    case GoalReplanRuntimeInterruption::kDriverAuthorityLost:
      return "driver_authority_lost";
    case GoalReplanRuntimeInterruption::kControlHold:
      return "control_hold";
    case GoalReplanRuntimeInterruption::kInspectionPause:
      return "inspection_pause_requested";
    case GoalReplanRuntimeInterruption::kInspectionCancel:
      return "inspection_cancel_requested";
    case GoalReplanRuntimeInterruption::kMapDrift:
      return "map_drift";
  }
  return "control_hold";
}

lingtu::message::NavigationGoalState
deferredReplacementInterruptionState(GoalReplanRuntimeInterruption interruption) {
  if (interruption == GoalReplanRuntimeInterruption::kMapDrift) {
    return lingtu::message::NavigationGoalState::Failed;
  }
  return lingtu::message::NavigationGoalState::Cancelled;
}

TerminalStopPolicy interruptionStopPolicy(GoalReplanRuntimeInterruption interruption) {
  switch (interruption) {
    case GoalReplanRuntimeInterruption::kCancel:
      return TerminalStopPolicy::kCancel;
    case GoalReplanRuntimeInterruption::kStop:
      return TerminalStopPolicy::kStop;
    case GoalReplanRuntimeInterruption::kEstop:
      return TerminalStopPolicy::kEstop;
    case GoalReplanRuntimeInterruption::kShutdown:
      return TerminalStopPolicy::kShutdown;
    case GoalReplanRuntimeInterruption::kNewGoal:
    case GoalReplanRuntimeInterruption::kOperatorTakeover:
    case GoalReplanRuntimeInterruption::kDriverAuthorityLost:
    case GoalReplanRuntimeInterruption::kControlHold:
    case GoalReplanRuntimeInterruption::kInspectionPause:
    case GoalReplanRuntimeInterruption::kInspectionCancel:
    case GoalReplanRuntimeInterruption::kMapDrift:
      return TerminalStopPolicy::kGenericStop;
  }
  return TerminalStopPolicy::kGenericStop;
}

}  // namespace

GoalReplanRuntimeCoordinator::GoalReplanRuntimeCoordinator(GoalPlanController &goal_plan,
                                                           MotionStopCoordinator &motion_stop,
                                                           BoundedGoalReplanConfig config)
    : goal_plan_(goal_plan), motion_stop_(motion_stop), bounded_(config) {}

GoalTerminalSchedulingDecision decideGoalTerminalScheduling(const GoalReplanRuntimeResult &result,
                                                            bool terminal_pending) {
  const bool surfaced_terminal =
      result.terminal_intent_id != 0U && result.terminal_after_stop.has_value();
  return {surfaced_terminal, !surfaced_terminal && !terminal_pending};
}

ShutdownExitDecision decideShutdownExit(bool stop_confirmed, bool terminal_required,
                                        bool terminal_pending, bool delivery_acknowledged) {
  return {stop_confirmed && !terminal_pending && (!terminal_required || delivery_acknowledged)};
}

bool GoalReplanRuntimeCoordinator::attemptActive(BoundedGoalReplanState state) {
  return state == BoundedGoalReplanState::kBackoffPending ||
         state == BoundedGoalReplanState::kReplanInFlight;
}

bool GoalReplanRuntimeCoordinator::validTime(double time_s) {
  return std::isfinite(time_s) && time_s >= 0.0;
}

bool GoalReplanRuntimeCoordinator::validPosition(const nav_kernel::Vec3 &position) {
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

bool GoalReplanRuntimeCoordinator::sameActiveGoal(const GoalPlanSnapshot &lhs,
                                                  const GoalPlanSnapshot &rhs) {
  return !lhs.active_task_id.empty() && !lhs.active_request_id.empty() &&
         lhs.active_task_id == rhs.active_task_id &&
         lhs.active_request_id == rhs.active_request_id && lhs.active_goal_epoch != 0U &&
         lhs.active_goal_epoch == rhs.active_goal_epoch && lhs.active_origin && rhs.active_origin &&
         lhs.active_origin == rhs.active_origin &&
         sameMap(lhs.active_map_identity, rhs.active_map_identity);
}

bool GoalReplanRuntimeCoordinator::directReplacementPlanning(const GoalPlanSnapshot &snapshot) {
  if (!snapshot.busy || snapshot.replan_in_progress || snapshot.planning_task_id.empty() ||
      snapshot.planning_request_id.empty() || snapshot.active_task_id.empty() ||
      snapshot.active_request_id.empty()) {
    return false;
  }
  return snapshot.planning_task_id != snapshot.active_task_id ||
         snapshot.planning_request_id != snapshot.active_request_id;
}

std::optional<BoundedGoalReplanGoal>
GoalReplanRuntimeCoordinator::activeGoal(const GoalPlanSnapshot &snapshot) {
  if (snapshot.active_task_id.empty() || snapshot.active_request_id.empty() ||
      snapshot.active_goal_epoch == 0U || !snapshot.active_map_identity ||
      !snapshot.active_map_identity->valid()) {
    return std::nullopt;
  }
  // The bounded controller keys by task+request. Supplying task_id as both is
  // deliberate: command request IDs change on pause/resume, but the one retry
  // belongs to the stable task. The real request ID was validated above and is
  // still used for stale-event comparison in sameActiveGoal().
  return BoundedGoalReplanGoal{snapshot.active_task_id, snapshot.active_task_id,
                               snapshot.active_goal_epoch, *snapshot.active_map_identity};
}

std::optional<BoundedGoalReplanGoal> GoalReplanRuntimeCoordinator::trackedGoal() const {
  const auto tracked = bounded_.snapshot();
  if (tracked.task_id.empty() || tracked.request_id.empty() || tracked.active_goal_epoch == 0U ||
      !tracked.map_identity || !tracked.map_identity->valid()) {
    return std::nullopt;
  }
  return BoundedGoalReplanGoal{tracked.task_id, tracked.request_id, tracked.active_goal_epoch,
                               *tracked.map_identity};
}

GoalPlanAdmissionContext
GoalReplanRuntimeCoordinator::normalizedAdmission(const GoalPlanAdmissionContext &context) {
  GoalPlanAdmissionContext result = context;
  // replanActive/resumePending predate InputGate. Project it onto their existing
  // blocker contract so a fresh blocked input cannot start a planner task.
  if (!result.input_ready && result.driver_control_blocker.empty()) {
    result.driver_control_blocker =
        std::string{"input_gate_"} +
        (result.input_gate_reason.empty() ? "blocked" : result.input_gate_reason);
  }
  return result;
}

std::string
GoalReplanRuntimeCoordinator::admissionFailure(const GoalReplanRuntimeFrameInput &input) {
  const auto &context = input.fresh_admission;
  if (!std::isfinite(context.autonomy_request_not_before_s) ||
      (context.map_position && !validPosition(*context.map_position))) {
    return "invalid_replan_admission";
  }
  if (input.map_drift) {
    return "map_drift";
  }
  if (input.inspection_active) {
    return "inspection_replan_not_supported";
  }
  if (input.rolling_segment_active || context.rolling_segment_active) {
    return "rolling_segment_replan_not_supported";
  }
  if (input.control_hold) {
    return "control_hold";
  }
  if (!context.motion_allowed) {
    return "estop_latched";
  }
  if (context.operator_takeover_latched) {
    return "operator_takeover_resume_required";
  }
  if (!context.autonomy_mode) {
    return std::string{"goal_not_allowed_in_"} + context.control_mode_name;
  }
  if (!context.driver_control_blocker.empty()) {
    return context.driver_control_blocker;
  }
  if (!context.input_ready) {
    return std::string{"input_gate_"} +
           (context.input_gate_reason.empty() ? "blocked" : context.input_gate_reason);
  }
  if (!context.map_position) {
    return context.odometry_ready ? "map_odom_tf_not_ready" : "odometry_not_ready";
  }
  if (!context.planner_map_configured) {
    return context.planner_map_missing_reason;
  }
  return {};
}

void GoalReplanRuntimeCoordinator::consumeInvalidTimeFor(const BoundedGoalReplanGoal &goal,
                                                         double steady_now_s) {
  pending_replan_trigger_.reset();
  bounded_.reset();
  (void)bounded_.armAfterConfirmedStop(goal, steady_now_s, false);
}

void GoalReplanRuntimeCoordinator::observeAndCancel(const BoundedGoalReplanGoal &goal,
                                                    double steady_now_s,
                                                    const std::string &reason) {
  pending_replan_trigger_.reset();
  // observe is required before cancel: cancel alone ignores an unbound goal and
  // would leave its one retry budget available after an identity failure.
  (void)bounded_.observeGoal(goal, steady_now_s);
  (void)bounded_.cancel(goal, steady_now_s, reason);
}

void GoalReplanRuntimeCoordinator::attachDeferredTerminal(
    GoalReplanRuntimeResult &result, lingtu::message::NavigationGoalState state,
    const std::string &reason, bool invalidate_planning, TerminalStopPolicy stop_policy) {
  pending_replan_trigger_.reset();
  const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
  const bool has_active_identity = !snapshot.active_task_id.empty() &&
                                   !snapshot.active_request_id.empty() &&
                                   snapshot.active_goal_epoch != 0U;
  const std::string task_id =
      !snapshot.active_task_id.empty() ? snapshot.active_task_id : snapshot.planning_task_id;
  result.handled = true;
  result.reason = reason;
  if (task_id.empty()) {
    return;
  }
  if (pending_terminal_) {
    (void)surfacePendingTerminal(result);
    return;
  }

  GoalPlanTerminalCommitWithTicket terminal;
  if (!invalidate_planning) {
    terminal = goal_plan_.deferActiveTerminalWithTicket(state, reason);
  } else if (state == lingtu::message::NavigationGoalState::Failed) {
    terminal = goal_plan_.deferFailureWithTicket(reason);
  } else {
    const bool project_planning_to_navigation_state =
        (stop_policy != TerminalStopPolicy::kStop && stop_policy != TerminalStopPolicy::kEstop &&
         stop_policy != TerminalStopPolicy::kShutdown) ||
        has_active_identity;
    terminal = goal_plan_.deferAbortWithTicket(reason, project_planning_to_navigation_state);
  }
  pending_terminal_intent_id_ = allocateTerminalIntentId();
  pending_terminal_task_id_ = task_id;
  pending_terminal_stop_policy_ = stop_policy;
  pending_terminal_ =
      GoalPlanTerminalAfterStop{reason, std::move(terminal.ticket), std::move(terminal.commit)};
  (void)surfacePendingTerminal(result);
}

void GoalReplanRuntimeCoordinator::attachExistingTerminal(GoalReplanRuntimeResult &result,
                                                          GoalPlanTerminalAfterStop terminal,
                                                          const std::string &task_id,
                                                          TerminalStopPolicy stop_policy) {
  pending_replan_trigger_.reset();
  result.handled = true;
  result.reason = terminal.reason;
  if (task_id.empty()) {
    return;
  }
  if (!pending_terminal_) {
    pending_terminal_intent_id_ = allocateTerminalIntentId();
    pending_terminal_task_id_ = task_id;
    pending_terminal_stop_policy_ = stop_policy;
    pending_terminal_ = std::move(terminal);
  }
  (void)surfacePendingTerminal(result);
}

bool GoalReplanRuntimeCoordinator::surfacePendingTerminal(GoalReplanRuntimeResult &result) const {
  if (!pending_terminal_ || pending_terminal_intent_id_ == 0U) {
    return false;
  }
  result.handled = true;
  result.reason = pending_terminal_->reason;
  result.terminal_intent_id = pending_terminal_intent_id_;
  result.terminal_task_id = pending_terminal_task_id_;
  result.terminal_stop_policy = pending_terminal_stop_policy_;
  result.terminal_after_stop = *pending_terminal_;
  return true;
}

std::uint64_t GoalReplanRuntimeCoordinator::allocateTerminalIntentId() {
  if (next_terminal_intent_id_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("terminal intent identity exhausted");
  }
  return next_terminal_intent_id_++;
}

bool GoalReplanRuntimeCoordinator::acknowledgeTerminal(std::uint64_t terminal_intent_id) {
  if (!pending_terminal_ || terminal_intent_id == 0U ||
      terminal_intent_id != pending_terminal_intent_id_) {
    return false;
  }
  pending_terminal_.reset();
  pending_terminal_task_id_.clear();
  pending_terminal_stop_policy_ = TerminalStopPolicy::kGenericStop;
  pending_terminal_intent_id_ = 0U;
  return true;
}

GoalReplanRuntimeResult GoalReplanRuntimeCoordinator::replayPendingTerminal() const {
  GoalReplanRuntimeResult result;
  (void)surfacePendingTerminal(result);
  return result;
}

bool GoalReplanRuntimeCoordinator::terminalPending() const {
  return pending_terminal_.has_value();
}

GoalReplanRuntimeResult
GoalReplanRuntimeCoordinator::interrupt(GoalReplanRuntimeInterruption interruption,
                                        double steady_now_s) {
  GoalReplanRuntimeResult result;
  if (surfacePendingTerminal(result)) {
    return result;
  }
  result.handled = true;
  result.interrupted = true;
  result.reason = interruptionReason(interruption);
  pending_replan_trigger_.reset();

  if (replacement_plan_in_progress_) {
    GoalPlanAdvanceResult interrupted_deferred = goal_plan_.failDeferredReplacement(
        deferredReplacementInterruptionState(interruption), result.reason);
    if (interrupted_deferred.terminal_after_stop) {
      GoalPlanTerminalAfterStop terminal = std::move(*interrupted_deferred.terminal_after_stop);
      interrupted_deferred.terminal_after_stop.reset();
      const std::string task_id = terminalTaskId(terminal);
      result.plan_advance = std::move(interrupted_deferred);
      attachExistingTerminal(result, std::move(terminal), task_id,
                             interruptionStopPolicy(interruption));
      replacement_plan_in_progress_ = false;
      return result;
    }
  }

  const auto current = activeGoal(goal_plan_.snapshot());
  const auto tracked = trackedGoal();
  if (!validTime(steady_now_s)) {
    if (current) {
      consumeInvalidTimeFor(*current, steady_now_s);
    } else if (tracked) {
      consumeInvalidTimeFor(*tracked, steady_now_s);
    }
  } else if (tracked) {
    (void)bounded_.cancel(*tracked, steady_now_s, result.reason);
  } else if (current) {
    observeAndCancel(*current, steady_now_s, result.reason);
  }
  replacement_plan_in_progress_ = false;

  switch (interruption) {
    case GoalReplanRuntimeInterruption::kNewGoal:
      // GoalPlan::submit owns queueing/cancelling the replan. Calling
      // invalidateForHold here would erase its pending B request.
      break;
    case GoalReplanRuntimeInterruption::kCancel:
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Cancelled, result.reason,
                             true, TerminalStopPolicy::kCancel);
      break;
    case GoalReplanRuntimeInterruption::kStop: {
      const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
      const bool has_active_identity = !snapshot.active_task_id.empty() &&
                                       !snapshot.active_request_id.empty() &&
                                       snapshot.active_goal_epoch != 0U;
      const bool has_planning_identity = !snapshot.planning_task_id.empty() &&
                                         !snapshot.planning_request_id.empty() &&
                                         snapshot.planning_goal_epoch != 0U;
      if (has_active_identity || has_planning_identity) {
        attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Cancelled,
                               result.reason, true, TerminalStopPolicy::kStop);
      } else {
        goal_plan_.invalidateForHold(result.reason);
      }
    } break;
    case GoalReplanRuntimeInterruption::kEstop: {
      const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
      const bool has_active_identity = !snapshot.active_task_id.empty() &&
                                       !snapshot.active_request_id.empty() &&
                                       snapshot.active_goal_epoch != 0U;
      const bool has_planning_identity = !snapshot.planning_task_id.empty() &&
                                         !snapshot.planning_request_id.empty() &&
                                         snapshot.planning_goal_epoch != 0U;
      if (has_active_identity || has_planning_identity) {
        attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Cancelled,
                               "estop_latched", true, TerminalStopPolicy::kEstop);
      } else {
        goal_plan_.invalidateForHold(result.reason);
      }
    } break;
    case GoalReplanRuntimeInterruption::kShutdown: {
      const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
      const bool has_active_identity = !snapshot.active_task_id.empty() &&
                                       !snapshot.active_request_id.empty() &&
                                       snapshot.active_goal_epoch != 0U;
      const bool has_planning_identity = !snapshot.planning_task_id.empty() &&
                                         !snapshot.planning_request_id.empty() &&
                                         snapshot.planning_goal_epoch != 0U;
      if (has_active_identity || has_planning_identity) {
        attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Cancelled,
                               "navd_shutdown", true, TerminalStopPolicy::kShutdown);
      } else {
        goal_plan_.invalidateForHold(result.reason);
      }
    } break;
    case GoalReplanRuntimeInterruption::kOperatorTakeover:
    case GoalReplanRuntimeInterruption::kDriverAuthorityLost:
    case GoalReplanRuntimeInterruption::kControlHold:
    case GoalReplanRuntimeInterruption::kInspectionPause:
    case GoalReplanRuntimeInterruption::kInspectionCancel:
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Cancelled, result.reason,
                             true);
      break;
    case GoalReplanRuntimeInterruption::kMapDrift:
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      break;
  }
  return result;
}
std::optional<GoalReplanRuntimeResult>
GoalReplanRuntimeCoordinator::handleGoalReached(const GoalReplanRuntimeFrameInput &input,
                                                const GoalReplanRuntimeAutonomyEvent &event,
                                                const GoalPlanSnapshot &current_snapshot) {
  if (event.outcome.kind != AutonomyTickOutcomeKind::kGoalReached) {
    return std::nullopt;
  }
  if (event.rolling_segment_active || input.rolling_segment_active ||
      input.fresh_admission.rolling_segment_active) {
    return std::nullopt;
  }
  if (!event.goal_snapshot.active_origin || !current_snapshot.active_origin ||
      *event.goal_snapshot.active_origin != *current_snapshot.active_origin) {
    return std::nullopt;
  }
  const bool supported_origin = *current_snapshot.active_origin == GoalPlanOrigin::kExternal ||
                                *current_snapshot.active_origin == GoalPlanOrigin::kInspection;
  if (!supported_origin) {
    return std::nullopt;
  }
  if (*current_snapshot.active_origin == GoalPlanOrigin::kExternal &&
      (event.inspection_active || input.inspection_active)) {
    return std::nullopt;
  }

  GoalReplanRuntimeResult result;
  const bool same_ids =
      !event.goal_snapshot.active_task_id.empty() &&
      !event.goal_snapshot.active_request_id.empty() &&
      event.goal_snapshot.active_goal_epoch != 0U &&
      event.goal_snapshot.active_task_id == current_snapshot.active_task_id &&
      event.goal_snapshot.active_request_id == current_snapshot.active_request_id &&
      event.goal_snapshot.active_goal_epoch == current_snapshot.active_goal_epoch;
  if (!same_ids) {
    result.handled = true;
    result.reason = "stale_autonomy_goal_reached_ignored";
    return result;
  }
  if (event.goal_snapshot.busy || event.goal_snapshot.replan_in_progress ||
      event.goal_snapshot.replacement_plan_in_progress || current_snapshot.busy ||
      current_snapshot.replan_in_progress || current_snapshot.replacement_plan_in_progress ||
      replacement_plan_in_progress_) {
    result.handled = true;
    result.reason = "goal_reached_plan_unstable";
    return result;
  }
  if (input.map_drift || !sameActiveGoal(event.goal_snapshot, current_snapshot)) {
    result.handled = true;
    result.reason = "active_goal_map_identity_changed";
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  if (input.control_hold) {
    result.handled = true;
    result.reason = "goal_reached_control_hold_ignored";
    return result;
  }

  attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Reached, kGoalReachedReason,
                         false);
  return result;
}

std::optional<GoalReplanRuntimeResult>
GoalReplanRuntimeCoordinator::handleInspectionTerminalOutcome(
    const GoalReplanRuntimeFrameInput &input, const GoalReplanRuntimeAutonomyEvent &event,
    const GoalPlanSnapshot &current_snapshot) {
  if (event.outcome.kind != AutonomyTickOutcomeKind::kGoalFailed || event.rolling_segment_active ||
      input.rolling_segment_active || input.fresh_admission.rolling_segment_active) {
    return std::nullopt;
  }
  if (!event.goal_snapshot.active_origin || !current_snapshot.active_origin ||
      *event.goal_snapshot.active_origin != GoalPlanOrigin::kInspection ||
      *current_snapshot.active_origin != GoalPlanOrigin::kInspection) {
    return std::nullopt;
  }

  GoalReplanRuntimeResult result;
  const bool same_ids =
      !event.goal_snapshot.active_task_id.empty() &&
      !event.goal_snapshot.active_request_id.empty() &&
      event.goal_snapshot.active_goal_epoch != 0U &&
      event.goal_snapshot.active_task_id == current_snapshot.active_task_id &&
      event.goal_snapshot.active_request_id == current_snapshot.active_request_id &&
      event.goal_snapshot.active_goal_epoch == current_snapshot.active_goal_epoch;
  if (!same_ids) {
    result.handled = true;
    result.reason = "stale_autonomy_failure_ignored";
    return result;
  }
  if (event.goal_snapshot.busy || event.goal_snapshot.replan_in_progress ||
      event.goal_snapshot.replacement_plan_in_progress || current_snapshot.busy ||
      current_snapshot.replan_in_progress || current_snapshot.replacement_plan_in_progress ||
      replacement_plan_in_progress_) {
    result.handled = true;
    result.reason = "goal_failed_plan_unstable";
    return result;
  }
  if (input.map_drift || !sameActiveGoal(event.goal_snapshot, current_snapshot)) {
    result.handled = true;
    result.reason = "active_goal_map_identity_changed";
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  result.reason = event.outcome.reason.empty() ? "goal_failed" : event.outcome.reason;
  attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                         false);
  return result;
}

GoalReplanRuntimeResult
GoalReplanRuntimeCoordinator::handleAutonomyOutcome(const GoalReplanRuntimeFrameInput &frame,
                                                    const GoalReplanRuntimeAutonomyEvent &event) {
  GoalReplanRuntimeResult result;
  if (surfacePendingTerminal(result)) {
    return result;
  }

  const GoalPlanSnapshot current_snapshot = goal_plan_.snapshot();
  const auto current = activeGoal(current_snapshot);
  if (!validTime(frame.steady_now_s)) {
    result.handled = true;
    result.reason = "invalid_time_budget_consumed";
    if (current) {
      consumeInvalidTimeFor(*current, frame.steady_now_s);
    } else if (const auto tracked = trackedGoal()) {
      consumeInvalidTimeFor(*tracked, frame.steady_now_s);
    }
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  if ((!current_snapshot.active_task_id.empty() || !current_snapshot.active_request_id.empty() ||
       current_snapshot.active_goal_epoch != 0U || current_snapshot.active_map_identity) &&
      !current) {
    result.handled = true;
    result.reason = "active_goal_identity_missing";
    if (const auto tracked = trackedGoal()) {
      observeAndCancel(*tracked, frame.steady_now_s, result.reason);
    }
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  if (current) {
    const auto observation = bounded_.observeGoal(*current, frame.steady_now_s);
    if (observation.action == BoundedGoalReplanAction::kCancel &&
        observation.reason == "superseded_by_new_goal") {
      pending_replan_trigger_.reset();
    }
    if (observation.action == BoundedGoalReplanAction::kReject ||
        (observation.action == BoundedGoalReplanAction::kCancel &&
         observation.reason != "superseded_by_new_goal")) {
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed,
                             observation.reason, true);
      return result;
    }
  }

  if (auto reached = handleGoalReached(frame, event, current_snapshot)) {
    return std::move(*reached);
  }
  if (auto inspection_terminal = handleInspectionTerminalOutcome(frame, event, current_snapshot)) {
    return std::move(*inspection_terminal);
  }
  if (replacement_plan_in_progress_) {
    result.handled = true;
    result.reason = "replacement_plan_in_flight";
    return result;
  }

  const bool typed_replan_request = event.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
                                    event.outcome.replan_trigger.has_value() &&
                                    !event.outcome.inspection_arrival_intent;
  if (!typed_replan_request || event.inspection_active || event.rolling_segment_active ||
      frame.inspection_active || frame.rolling_segment_active ||
      frame.fresh_admission.rolling_segment_active || frame.control_hold || frame.map_drift) {
    return result;
  }
  const GoalReplanTrigger &trigger = *event.outcome.replan_trigger;
  const char *stop_reason = replanStopReason(trigger.kind);
  if (!event.goal_snapshot.active_origin || !current_snapshot.active_origin) {
    if (current) {
      const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
      observeAndCancel(*current, frame.steady_now_s, "active_goal_identity_missing");
      result.handled = true;
      result.reason = stopped.accepted ? "active_goal_identity_missing" : stopped.reason;
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
    }
    return result;
  }
  if (*event.goal_snapshot.active_origin != GoalPlanOrigin::kExternal ||
      *current_snapshot.active_origin != GoalPlanOrigin::kExternal) {
    return result;
  }
  if (!current) {
    result.handled = true;
    result.reason = "active_goal_identity_missing";
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  const auto captured = activeGoal(event.goal_snapshot);
  if (!captured) {
    const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
    observeAndCancel(*current, frame.steady_now_s, "active_goal_identity_missing");
    result.handled = true;
    result.reason = stopped.accepted ? "active_goal_identity_missing" : stopped.reason;
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  if (!sameActiveGoal(event.goal_snapshot, current_snapshot)) {
    const bool same_ids =
        event.goal_snapshot.active_task_id == current_snapshot.active_task_id &&
        event.goal_snapshot.active_request_id == current_snapshot.active_request_id &&
        event.goal_snapshot.active_goal_epoch == current_snapshot.active_goal_epoch;
    if (!same_ids) {
      result.handled = true;
      result.reason = "stale_autonomy_failure_ignored";
      return result;
    }
    const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
    observeAndCancel(*current, frame.steady_now_s, "active_goal_map_identity_changed");
    result.handled = true;
    result.reason = stopped.accepted ? "active_goal_map_identity_changed" : stopped.reason;
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  if (!triggerMatchesSnapshot(trigger, event.goal_snapshot) ||
      !triggerMatchesSnapshot(trigger, current_snapshot)) {
    const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
    observeAndCancel(*current, frame.steady_now_s, "replan_trigger_identity_mismatch");
    result.handled = true;
    result.reason = stopped.accepted ? "replan_trigger_identity_mismatch" : stopped.reason;
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  const bool valid_trigger = validLocalRecoveryTrigger(trigger) ||
                             validPersistentOverlay(trigger, frame.fresh_admission.frame_epoch);
  if (!valid_trigger) {
    const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
    observeAndCancel(*current, frame.steady_now_s, "replan_trigger_payload_invalid");
    result.handled = true;
    result.reason = stopped.accepted ? "replan_trigger_payload_invalid" : stopped.reason;
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  if (!goalPlanAcceptsReplanTrigger(current_snapshot) || !frame.fresh_admission.motion_allowed ||
      frame.fresh_admission.operator_takeover_latched || !frame.fresh_admission.autonomy_mode ||
      !frame.fresh_admission.driver_control_blocker.empty() || !frame.fresh_admission.input_ready) {
    return result;
  }

  const auto state = bounded_.snapshot().state;
  if (attemptActive(state)) {
    result.handled = true;
    result.reason =
        state == BoundedGoalReplanState::kBackoffPending ? "backoff_pending" : "replan_in_flight";
    return result;
  }
  const MotionStopResult stopped = motion_stop_.confirmGoalReplanStop(stop_reason);
  const auto armed = bounded_.armAfterConfirmedStop(*current, frame.steady_now_s, stopped.accepted);
  result.handled = true;
  if (!stopped.accepted) {
    result.reason = stopped.reason;
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }
  if (armed.action == BoundedGoalReplanAction::kHold) {
    pending_replan_trigger_ = trigger;
    result.reason = "backoff_pending";
    return result;
  }
  result.reason = armed.reason;
  attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason, true);
  return result;
}

GoalReplanRuntimeResult
GoalReplanRuntimeCoordinator::advancePlanningCycle(const GoalReplanRuntimeFrameInput &input) {
  GoalReplanRuntimeResult result;
  if (surfacePendingTerminal(result)) {
    return result;
  }
  GoalPlanSnapshot before = goal_plan_.snapshot();
  auto current = activeGoal(before);

  if (!validTime(input.steady_now_s)) {
    result.handled = true;
    result.reason = "invalid_time_budget_consumed";
    if (current) {
      consumeInvalidTimeFor(*current, input.steady_now_s);
    } else if (const auto tracked = trackedGoal()) {
      consumeInvalidTimeFor(*tracked, input.steady_now_s);
    }
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  if (!validTime(input.wall_now_s)) {
    result.handled = true;
    result.reason = "invalid_wall_time";
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  if (replacement_plan_in_progress_) {
    const std::string admission_error = admissionFailure(input);
    GoalPlanAdvanceResult activated =
        admission_error.empty()
            ? goal_plan_.activateDeferredReplacement(input.wall_now_s,
                                                     normalizedAdmission(input.fresh_admission))
            : goal_plan_.failDeferredReplacement(deferredReplacementAdmissionState(admission_error),
                                                 admission_error);
    if (activated.path_activated) {
      replacement_plan_in_progress_ = false;
      result.handled = true;
      result.reason = "replacement_plan_completed";
      result.plan_advance = std::move(activated);
      return result;
    }
    if (activated.terminal_after_stop) {
      replacement_plan_in_progress_ = false;
      GoalPlanTerminalAfterStop terminal = std::move(*activated.terminal_after_stop);
      activated.terminal_after_stop.reset();
      const std::string task_id = terminalTaskId(terminal);
      result.plan_advance = std::move(activated);
      attachExistingTerminal(result, std::move(terminal), task_id);
      return result;
    }
  }

  if ((!before.active_task_id.empty() || !before.active_request_id.empty() ||
       before.active_goal_epoch != 0U || before.active_map_identity) &&
      !current) {
    result.handled = true;
    result.reason = "active_goal_identity_missing";
    if (const auto tracked = trackedGoal()) {
      observeAndCancel(*tracked, input.steady_now_s, result.reason);
    }
    attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                           true);
    return result;
  }

  if (current) {
    const auto observation = bounded_.observeGoal(*current, input.steady_now_s);
    if (observation.action == BoundedGoalReplanAction::kCancel &&
        observation.reason == "superseded_by_new_goal") {
      pending_replan_trigger_.reset();
    }
    if (observation.action == BoundedGoalReplanAction::kReject ||
        (observation.action == BoundedGoalReplanAction::kCancel &&
         observation.reason != "superseded_by_new_goal")) {
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed,
                             observation.reason, true);
      return result;
    }
  }

  auto bounded_snapshot = bounded_.snapshot();
  const bool bounded_attempt_active = attemptActive(bounded_snapshot.state);
  if (current && directReplacementPlanning(before)) {
    replacement_plan_in_progress_ = true;
    if (bounded_attempt_active) {
      (void)bounded_.cancel(*current, input.steady_now_s, "superseded_by_new_goal");
      pending_replan_trigger_.reset();
      result.interrupted = true;
    }
  } else if (current && bounded_attempt_active && before.pending_plan_queued) {
    (void)bounded_.cancel(*current, input.steady_now_s, "superseded_by_new_goal");
    pending_replan_trigger_.reset();
    result.interrupted = true;
  }

  bounded_snapshot = bounded_.snapshot();
  const bool retry_runtime_active = attemptActive(bounded_snapshot.state) ||
                                    before.pending_plan_queued || replacement_plan_in_progress_;
  if (retry_runtime_active) {
    if (input.map_drift) {
      return interrupt(GoalReplanRuntimeInterruption::kMapDrift, input.steady_now_s);
    }
    if (input.fresh_admission.operator_takeover_latched) {
      return interrupt(GoalReplanRuntimeInterruption::kOperatorTakeover, input.steady_now_s);
    }
    if (!input.fresh_admission.driver_control_blocker.empty()) {
      return interrupt(GoalReplanRuntimeInterruption::kDriverAuthorityLost, input.steady_now_s);
    }
    if (!input.fresh_admission.motion_allowed) {
      return interrupt(GoalReplanRuntimeInterruption::kEstop, input.steady_now_s);
    }
    if (input.control_hold || !input.fresh_admission.autonomy_mode ||
        !input.fresh_admission.input_ready || input.inspection_active ||
        input.rolling_segment_active || input.fresh_admission.rolling_segment_active) {
      return interrupt(GoalReplanRuntimeInterruption::kControlHold, input.steady_now_s);
    }
    if (!motion_stop_.keepZeroFresh()) {
      result.handled = true;
      result.reason = "zero_refresh_failed";
      if (const auto tracked = trackedGoal()) {
        (void)bounded_.cancel(*tracked, input.steady_now_s, result.reason);
      }
      replacement_plan_in_progress_ = false;
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }
    result.zero_kept_fresh = true;
  }

  const bool completing_replan = before.replan_in_progress &&
                                 bounded_snapshot.state == BoundedGoalReplanState::kReplanInFlight;
  const bool completing_replacement =
      replacement_plan_in_progress_ && before.busy && !before.replan_in_progress;
  const auto completion_goal = trackedGoal();
  result.plan_advance = goal_plan_.advance(
      GoalPlanAdvanceContext{input.fresh_admission.frame_epoch,
                             input.fresh_admission.operator_takeover_latched, input.wall_now_s});
  GoalPlanSnapshot after = goal_plan_.snapshot();

  if (completing_replan && result.plan_advance.completion_consumed && completion_goal) {
    (void)bounded_.completeReplan(*completion_goal, input.steady_now_s,
                                  result.plan_advance.path_activated);
    pending_replan_trigger_.reset();
    result.handled = true;
  }

  if (result.plan_advance.terminal_after_stop) {
    GoalPlanTerminalAfterStop terminal = std::move(*result.plan_advance.terminal_after_stop);
    result.plan_advance.terminal_after_stop.reset();
    attachExistingTerminal(result, std::move(terminal), before.active_task_id);
    return result;
  }

  if (completing_replacement && result.plan_advance.completion_consumed) {
    replacement_plan_in_progress_ = false;
    result.handled = true;
    if (result.plan_advance.path_activated) {
      result.reason = "replacement_plan_completed";
      return result;
    }
    const std::string reason =
        after.diagnostics.reason.empty() ? "replacement_plan_failed" : after.diagnostics.reason;
    attachDeferredTerminal(result,
                           result.plan_advance.counted_failure
                               ? lingtu::message::NavigationGoalState::Failed
                               : lingtu::message::NavigationGoalState::Cancelled,
                           reason, false);
    return result;
  }

  if (completing_replan && result.plan_advance.completion_consumed &&
      result.plan_advance.path_activated) {
    result.reason = "replan_completed";
    return result;
  }

  if (after.pending_plan_queued) {
    result.handled = true;
    result.reason = after.busy ? "pending_plan_draining" : "pending_plan_ready";
    return result;
  }

  if (replacement_plan_in_progress_) {
    result.handled = true;
    result.reason = "replacement_plan_in_flight";
    return result;
  }

  const GoalPlanSnapshot replan_snapshot = goal_plan_.snapshot();
  current = activeGoal(replan_snapshot);
  const auto retry = bounded_.snapshot();
  if (retry.state == BoundedGoalReplanState::kBackoffPending && current) {
    const auto decision = bounded_.tick(*current, input.steady_now_s);
    result.handled = true;
    if (decision.action == BoundedGoalReplanAction::kHold) {
      result.reason = "backoff_pending";
      return result;
    }
    if (decision.action != BoundedGoalReplanAction::kStart) {
      result.reason = decision.reason;
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }

    if (!pending_replan_trigger_ ||
        !triggerMatchesSnapshot(*pending_replan_trigger_, replan_snapshot)) {
      (void)bounded_.completeReplan(*current, input.steady_now_s, false);
      result.reason = "replan_trigger_missing_or_stale";
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }
    const bool valid_pending_trigger =
        validLocalRecoveryTrigger(*pending_replan_trigger_) ||
        validPersistentOverlay(*pending_replan_trigger_, input.fresh_admission.frame_epoch);
    if (!valid_pending_trigger) {
      (void)bounded_.completeReplan(*current, input.steady_now_s, false);
      result.reason = "replan_trigger_payload_stale";
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }

    const std::string admission_error = admissionFailure(input);
    if (!admission_error.empty()) {
      (void)bounded_.completeReplan(*current, input.steady_now_s, false);
      result.reason = admission_error;
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }
    GoalPlanAdmissionContext replan_admission = normalizedAdmission(input.fresh_admission);
    replan_admission.temporary_overlay = {};
    if (pending_replan_trigger_->kind == GoalReplanTriggerKind::kPersistentPathObstruction) {
      replan_admission.temporary_overlay = pending_replan_trigger_->temporary_overlay;
    }
    const GoalPlanSubmitResult started = goal_plan_.replanActive(replan_admission);
    if (!started.accepted) {
      (void)bounded_.completeReplan(*current, input.steady_now_s, false);
      result.reason = started.reason;
      attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                             true);
      return result;
    }
    pending_replan_trigger_.reset();
    result.replan_started = true;
    result.reason = "replan_started";
    return result;
  }

  if (retry.state == BoundedGoalReplanState::kReplanInFlight) {
    result.handled = true;
    result.reason = "replan_in_flight";
  } else if (result.plan_advance.completion_consumed || result.plan_advance.path_activated) {
    result.handled = true;
    result.reason = "plan_advanced";
  }
  return result;
}
GoalReplanRuntimeResult
GoalReplanRuntimeCoordinator::drainPendingCycle(const GoalReplanRuntimeFrameInput &frame) {
  GoalReplanRuntimeResult result;
  if (surfacePendingTerminal(result)) {
    return result;
  }

  const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
  if (!snapshot.pending_plan_queued) {
    if (replacement_plan_in_progress_) {
      result.handled = true;
      result.reason = "replacement_plan_in_flight";
    }
    return result;
  }

  result.handled = true;
  if (snapshot.busy) {
    result.reason = "pending_plan_draining";
    return result;
  }
  const GoalPlanSubmitResult resumed =
      goal_plan_.resumePending(normalizedAdmission(frame.fresh_admission));
  if (resumed.accepted) {
    replacement_plan_in_progress_ = true;
    result.pending_resumed = true;
    result.reason = "pending_plan_resumed";
    return result;
  }
  result.reason = resumed.reason;
  if (resumed.active_terminal_after_stop) {
    attachDeferredTerminal(result, resumed.active_terminal_after_stop->state,
                           resumed.active_terminal_after_stop->reason, false);
    return result;
  }
  attachDeferredTerminal(result, lingtu::message::NavigationGoalState::Failed, result.reason,
                         false);
  return result;
}

BoundedGoalReplanSnapshot GoalReplanRuntimeCoordinator::snapshot() const {
  return bounded_.snapshot();
}

}  // namespace lingtu::nav::endpoint
