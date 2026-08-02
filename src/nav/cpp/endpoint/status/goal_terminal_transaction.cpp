#include "status/goal_terminal_transaction.hpp"

#include <stdexcept>
#include <utility>

#include "motion/motion_stop_coordinator.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"
#include "status/goal_terminal_status_delivery.hpp"

namespace lingtu::nav::endpoint {

GoalTerminalTransaction::GoalTerminalTransaction(GoalReplanRuntimeCoordinator &goal_replan_runtime,
                                                 MotionStopCoordinator &motion_stop,
                                                 GoalTerminalStatusDelivery &goal_terminal_delivery,
                                                 GoalTerminalTransactionActions actions)
    : goal_replan_runtime_(goal_replan_runtime),
      motion_stop_(motion_stop),
      goal_terminal_delivery_(goal_terminal_delivery),
      actions_(std::move(actions)) {
  if (!actions_.report_error || !actions_.pause_inspection || !actions_.sync_goal_diagnostics) {
    throw std::invalid_argument("GoalTerminalTransaction requires complete actions");
  }
}

GoalTerminalTransactionResult
GoalTerminalTransaction::advance(const GoalReplanRuntimeResult &runtime_result,
                                 const std::string &estop_reason) {
  if (!runtime_result.terminal_after_stop || runtime_result.terminal_intent_id == 0U) {
    return {false, false, "goal_terminal_missing"};
  }

  const GoalPlanTerminalAfterStop &terminal = *runtime_result.terminal_after_stop;
  const GoalTerminalStatusDelivery::StageResult stage =
      goal_terminal_delivery_.stage(runtime_result.terminal_intent_id, terminal.delivery_ticket);
  if (stage != GoalTerminalStatusDelivery::StageResult::kStaged &&
      stage != GoalTerminalStatusDelivery::StageResult::kReplay) {
    actions_.report_error(stage == GoalTerminalStatusDelivery::StageResult::kConflict
                              ? "goal_terminal_delivery_ticket_conflict"
                              : "goal_terminal_delivery_ticket_invalid");
    return {false, false, "goal_terminal_delivery_ticket_invalid"};
  }

  if (!goal_terminal_delivery_.isCommitted(runtime_result.terminal_intent_id)) {
    MotionStopTerminalBarrierResult terminal_result;
    switch (runtime_result.terminal_stop_policy) {
      case TerminalStopPolicy::kStop:
        terminal_result = motion_stop_.stopPreservingGoalTerminal(terminal.commit);
        break;
      case TerminalStopPolicy::kCancel:
        terminal_result = motion_stop_.cancelPreservingGoalTerminal(terminal.commit);
        break;
      case TerminalStopPolicy::kEstop:
        terminal_result = motion_stop_.estopPreservingGoalTerminal(estop_reason, terminal.commit);
        break;
      case TerminalStopPolicy::kShutdown:
        return {false, false, "shutdown_terminal_requires_transaction"};
      case TerminalStopPolicy::kGenericStop: {
        const MotionStopResult stopped =
            motion_stop_.commitGoalTerminalAfterStop(terminal.reason, terminal.commit);
        terminal_result =
            MotionStopTerminalBarrierResult{stopped.accepted, stopped.reason, stopped.accepted};
        break;
      }
    }

    if (!terminal_result.accepted) {
      const std::string failure = "goal_terminal_stop_unconfirmed:" + terminal_result.reason;
      actions_.pause_inspection(failure);
      actions_.report_error(failure);
      return {false, false, terminal_result.reason};
    }
    if (!terminal_result.terminal_committed) {
      actions_.report_error("goal_terminal_stop_unconfirmed:" + terminal_result.reason);
      return {false, false, terminal_result.reason};
    }
    if (!goal_terminal_delivery_.markCommitted(runtime_result.terminal_intent_id)) {
      actions_.report_error("goal_terminal_delivery_commit_identity_rejected");
      return {false, false, "goal_terminal_delivery_commit_identity_rejected"};
    }
    actions_.sync_goal_diagnostics();
  }

  const GoalTerminalStatusDelivery::FlushResult delivery =
      goal_terminal_delivery_.flushAndAcknowledge(goal_replan_runtime_);
  if (delivery == GoalTerminalStatusDelivery::FlushResult::kAcknowledged) {
    return {true, true, "goal_terminal_acknowledged"};
  }
  if (delivery == GoalTerminalStatusDelivery::FlushResult::kAcknowledgementRejected) {
    actions_.report_error("goal_terminal_delivery_acknowledgement_rejected");
    return {true, false, "goal_terminal_delivery_acknowledgement_rejected"};
  }
  return {true, false, "goal_terminal_delivery_pending"};
}

MotionStopTerminalBarrierResult GoalTerminalTransaction::stopWhileTerminalPending() {
  return runWhileTerminalPending(PendingTerminalSafetyAction::kStop, {});
}

MotionStopTerminalBarrierResult
GoalTerminalTransaction::estopWhileTerminalPending(const std::string &estop_reason) {
  return runWhileTerminalPending(PendingTerminalSafetyAction::kEstop, estop_reason);
}

MotionStopTerminalBarrierResult
GoalTerminalTransaction::runWhileTerminalPending(PendingTerminalSafetyAction safety_action,
                                                 const std::string &estop_reason) {
  const GoalReplanRuntimeResult pending = goal_replan_runtime_.replayPendingTerminal();
  if (!pending.terminal_after_stop || pending.terminal_intent_id == 0U) {
    const MotionStopResult physical_stop = motion_stop_.stopWithoutTerminalCommit();
    actions_.report_error("goal_terminal_surface_missing");
    return {
        false,
        physical_stop.accepted ? "goal_terminal_surface_missing" : physical_stop.reason,
        false,
    };
  }

  const GoalPlanTerminalAfterStop &terminal = *pending.terminal_after_stop;
  const GoalTerminalStatusDelivery::StageResult stage =
      goal_terminal_delivery_.stage(pending.terminal_intent_id, terminal.delivery_ticket);
  const bool exact_stage = stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                           stage == GoalTerminalStatusDelivery::StageResult::kReplay;

  auto preserve_terminal = [&] {
    if (safety_action == PendingTerminalSafetyAction::kEstop) {
      return motion_stop_.estopPreservingGoalTerminal(estop_reason, terminal.commit);
    }
    return motion_stop_.stopPreservingGoalTerminal(terminal.commit);
  };

  if (!exact_stage) {
    const std::string failure_reason = stage == GoalTerminalStatusDelivery::StageResult::kConflict
                                           ? "goal_terminal_delivery_ticket_conflict"
                                           : "goal_terminal_delivery_ticket_invalid";
    actions_.report_error(failure_reason);
    const MotionStopTerminalBarrierResult preserved = preserve_terminal();
    if (preserved.terminal_committed) {
      actions_.sync_goal_diagnostics();
    }
    return {
        false,
        preserved.accepted ? failure_reason : preserved.reason,
        preserved.terminal_committed,
    };
  }

  const MotionStopTerminalBarrierResult preserved = preserve_terminal();
  if (!preserved.terminal_committed) {
    return preserved;
  }
  if (!goal_terminal_delivery_.markCommitted(pending.terminal_intent_id)) {
    actions_.report_error("goal_terminal_delivery_commit_identity_rejected");
    return {
        false,
        "goal_terminal_delivery_commit_identity_rejected",
        true,
    };
  }
  actions_.sync_goal_diagnostics();
  return preserved;
}

}  // namespace lingtu::nav::endpoint
