#include "app/goal/terminal.hpp"

#include <stdexcept>
#include <utility>
#include <memory>

#include "app/goal/runtime.hpp"
#include "status/goal_terminal_status_delivery.hpp"

namespace lingtu::nav::endpoint {

GoalTerminal::GoalTerminal(GoalReplanRuntimeCoordinator &runtime,
                           MotionStopBarrier &stop,
                           GoalTerminalStatusDelivery &delivery,
                           GoalTerminalActions actions)
    : runtime_(runtime),
      stop_(stop),
      delivery_(delivery),
      actions_(std::move(actions)) {
  if (!actions_.report_error || !actions_.pause_inspection || !actions_.sync_goal_diagnostics) {
    throw std::invalid_argument("GoalTerminal requires complete actions");
  }
}

GoalTerminalResult GoalTerminal::advance(const GoalReplanRuntimeResult &runtime_result,
                                         const std::string &estop_reason) {
  if (!runtime_result.terminal_after_stop || runtime_result.terminal_intent_id == 0U) {
    return {false, false, "goal_terminal_missing"};
  }

  const GoalPlanTerminalAfterStop &terminal = *runtime_result.terminal_after_stop;
  const GoalTerminalStatusDelivery::StageResult stage =
      delivery_.stage(runtime_result.terminal_intent_id, terminal.delivery_ticket);
  if (stage != GoalTerminalStatusDelivery::StageResult::kStaged &&
      stage != GoalTerminalStatusDelivery::StageResult::kReplay) {
    actions_.report_error(stage == GoalTerminalStatusDelivery::StageResult::kConflict
                              ? "goal_terminal_delivery_ticket_conflict"
                              : "goal_terminal_delivery_ticket_invalid");
    return {false, false, "goal_terminal_delivery_ticket_invalid"};
  }

  if (!delivery_.isCommitted(runtime_result.terminal_intent_id)) {
    MotionStopTerminalBarrierResult terminal_result;
    switch (runtime_result.terminal_stop_policy) {
      case TerminalStopPolicy::kReached:
        // FinalControl already published the zero command for a normal goal
        // reached outcome. Do not turn ordinary arrival into a stop transaction.
        terminal.commit();
        if (!delivery_.markCommitted(runtime_result.terminal_intent_id)) {
          actions_.report_error("goal_terminal_delivery_commit_identity_rejected");
          return {false, false, "goal_terminal_delivery_commit_identity_rejected"};
        }
        actions_.sync_goal_diagnostics();
        break;
      case TerminalStopPolicy::kStop:
        terminal_result = stop_.stopPreservingGoalTerminal(terminal.commit);
        break;
      case TerminalStopPolicy::kCancel:
        terminal_result = stop_.cancelPreservingGoalTerminal(terminal.commit);
        break;
      case TerminalStopPolicy::kEstop:
        terminal_result = stop_.estopPreservingGoalTerminal(estop_reason, terminal.commit);
        break;
      case TerminalStopPolicy::kShutdown:
        return {false, false, "shutdown_terminal_requires_transaction"};
      case TerminalStopPolicy::kGenericStop: {
        const MotionStopResult stopped =
            stop_.commitGoalTerminalAfterStop(terminal.reason, terminal.commit);
        terminal_result =
            MotionStopTerminalBarrierResult{stopped.accepted, stopped.reason, stopped.accepted,
                                             stopped.pending};
        break;
      }
    }

    if (runtime_result.terminal_stop_policy != TerminalStopPolicy::kReached) {
      if (terminal_result.pending) return {false, false, "stop_confirmation_pending"};
      if (!terminal_result.accepted || !terminal_result.terminal_committed) {
        const std::string failure = "goal_terminal_stop_unconfirmed:" + terminal_result.reason;
        if (!terminal_result.accepted) actions_.pause_inspection(failure);
        actions_.report_error(failure);
        return {false, false, terminal_result.reason};
      }
      if (!delivery_.markCommitted(runtime_result.terminal_intent_id)) {
        actions_.report_error("goal_terminal_delivery_commit_identity_rejected");
        return {false, false, "goal_terminal_delivery_commit_identity_rejected"};
      }
      actions_.sync_goal_diagnostics();
    }
  }

  const GoalTerminalStatusDelivery::FlushResult delivery =
      delivery_.flushAndAcknowledge(runtime_);
  if (delivery == GoalTerminalStatusDelivery::FlushResult::kAcknowledged) {
    return {true, true, "goal_terminal_acknowledged"};
  }
  if (delivery == GoalTerminalStatusDelivery::FlushResult::kAcknowledgementRejected) {
    actions_.report_error("goal_terminal_delivery_acknowledgement_rejected");
    return {true, false, "goal_terminal_delivery_acknowledgement_rejected"};
  }
  return {true, false, "goal_terminal_delivery_pending"};
}

MotionStopResult GoalTerminal::commitRelated(
    const GoalReplanRuntimeResult &result, const std::string &reason,
    std::function<bool()> commit) {
  const auto &terminal = result.terminal_after_stop;
  if (terminal) {
    const auto stage = delivery_.stage(result.terminal_intent_id, terminal->delivery_ticket);
    if (stage != GoalTerminalStatusDelivery::StageResult::kStaged &&
        stage != GoalTerminalStatusDelivery::StageResult::kReplay) {
      return {false, stage == GoalTerminalStatusDelivery::StageResult::kConflict
          ? "goal_terminal_delivery_ticket_conflict" : "goal_terminal_delivery_ticket_invalid"};
    }
    if (delivery_.isCommitted(result.terminal_intent_id))
      return {true, "inspection_goal_terminal_committed"};
  }
  auto committed = std::make_shared<bool>(false);
  auto stopped = stop_.commitGoalTerminalAfterStop(terminal ? terminal->reason : reason,
      [this, terminal, intent = result.terminal_intent_id, commit = std::move(commit), committed] {
        if (terminal) terminal->commit();
        *committed = commit();
        if (*committed && terminal) {
          *committed = delivery_.markCommitted(intent);
          actions_.sync_goal_diagnostics();
        }
      });
  if (!stopped.pending && stopped.accepted && !*committed)
    return {false, "inspection_state_commit_failed"};
  return stopped;
}

MotionStopTerminalBarrierResult GoalTerminal::stopPending() {
  return stopPending(StopAction::kStop, {});
}

MotionStopTerminalBarrierResult
GoalTerminal::estopPending(const std::string &estop_reason) {
  return stopPending(StopAction::kEstop, estop_reason);
}

MotionStopTerminalBarrierResult
GoalTerminal::stopPending(StopAction safety_action,
                          const std::string &estop_reason) {
  runtime_.requirePendingTerminalOdometry();
  const GoalReplanRuntimeResult pending = runtime_.replayPendingTerminal();
  if (!pending.terminal_after_stop || pending.terminal_intent_id == 0U) {
    const MotionStopResult physical_stop = stop_.stopWithoutTerminalCommit();
    actions_.report_error("goal_terminal_surface_missing");
    return {
        false,
        physical_stop.accepted ? "goal_terminal_surface_missing" : physical_stop.reason,
        false,
    };
  }

  const GoalPlanTerminalAfterStop &terminal = *pending.terminal_after_stop;
  const GoalTerminalStatusDelivery::StageResult stage =
      delivery_.stage(pending.terminal_intent_id, terminal.delivery_ticket);
  const bool exact_stage = stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                           stage == GoalTerminalStatusDelivery::StageResult::kReplay;

  std::string stage_error;
  if (!exact_stage) {
    stage_error = stage == GoalTerminalStatusDelivery::StageResult::kConflict
                      ? "goal_terminal_delivery_ticket_conflict"
                      : "goal_terminal_delivery_ticket_invalid";
    actions_.report_error(stage_error);
  }

  // A delivery conflict must not suppress an operator's stop or replace its pending result.
  const auto preserved = safety_action == StopAction::kEstop
      ? stop_.estopPreservingGoalTerminal(estop_reason, terminal.commit)
      : stop_.stopPreservingGoalTerminal(terminal.commit);
  if (!exact_stage) {
    if (preserved.terminal_committed) actions_.sync_goal_diagnostics();
    return {
        false,
        preserved.accepted ? stage_error : preserved.reason,
        preserved.terminal_committed,
    };
  }

  if (!preserved.terminal_committed) {
    return preserved;
  }
  if (!delivery_.markCommitted(pending.terminal_intent_id)) {
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
