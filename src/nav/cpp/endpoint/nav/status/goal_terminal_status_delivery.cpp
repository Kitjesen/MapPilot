#include "status/goal_terminal_status_delivery.hpp"

#include <cstddef>

#include "safety/stop.hpp"
#include "runtime/goal/runtime.hpp"

namespace lingtu::nav::endpoint {

namespace {

std::string shutdownTransactionReason(const ShutdownTransactionResult &result) {
  if (result.decision.allow_exit) {
    return "shutdown_complete";
  }
  if (!result.stop_confirmed) {
    return "shutdown_zero_confirm_pending";
  }
  return "shutdown_terminal_delivery_pending";
}

}  // namespace

GoalTerminalStatusDelivery::GoalTerminalStatusDelivery(NavigationGoalStatusOutbox &outbox)
    : outbox_(outbox) {}

GoalTerminalStatusDelivery::StageResult
GoalTerminalStatusDelivery::stage(std::uint64_t intent_id,
                                  const GoalPlanTerminalDeliveryTicket &ticket) {
  if (intent_id == 0U) {
    return StageResult::kInvalidIntent;
  }
  if (!staged_terminal_) {
    staged_terminal_ = StagedTerminal{intent_id, ticket, false};
    return StageResult::kStaged;
  }
  if (staged_terminal_->intent_id == intent_id && sameTicket(staged_terminal_->ticket, ticket)) {
    return StageResult::kReplay;
  }
  return StageResult::kConflict;
}

bool GoalTerminalStatusDelivery::markCommitted(std::uint64_t intent_id) {
  if (!staged_terminal_ || intent_id == 0U || staged_terminal_->intent_id != intent_id) {
    return false;
  }
  staged_terminal_->committed = true;
  return true;
}

bool GoalTerminalStatusDelivery::isCommitted(std::uint64_t intent_id) const {
  return staged_terminal_ && intent_id != 0U && staged_terminal_->intent_id == intent_id &&
      staged_terminal_->committed;
}

GoalTerminalStatusDelivery::FlushResult
GoalTerminalStatusDelivery::flushAndAcknowledge(GoalReplanRuntimeCoordinator &coordinator) {
  if (!staged_terminal_) {
    return FlushResult::kNoStagedTerminal;
  }
  if (!staged_terminal_->committed) {
    return FlushResult::kCommitPending;
  }

  (void)outbox_.flush();
  if (!ticketDelivered(staged_terminal_->ticket)) {
    return FlushResult::kDeliveryPending;
  }
  if (!coordinator.acknowledgeTerminal(staged_terminal_->intent_id)) {
    return FlushResult::kAcknowledgementRejected;
  }

  staged_terminal_.reset();
  return FlushResult::kAcknowledged;
}

bool GoalTerminalStatusDelivery::sameTicket(const GoalPlanTerminalDeliveryTicket &left,
                                            const GoalPlanTerminalDeliveryTicket &right) {
  if (left.statuses.size() != right.statuses.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < left.statuses.size(); ++index) {
    if (!sameStatus(left.statuses[index], right.statuses[index])) {
      return false;
    }
  }
  return true;
}

bool GoalTerminalStatusDelivery::sameStatus(const GoalPlanStatus &left,
                                            const GoalPlanStatus &right) {
  return left.task_id == right.task_id && left.request_id == right.request_id &&
         left.goal_epoch == right.goal_epoch && left.state == right.state &&
         left.reason == right.reason &&
         left.project_to_navigation_state == right.project_to_navigation_state;
}

bool GoalTerminalStatusDelivery::ticketDelivered(
    const GoalPlanTerminalDeliveryTicket &ticket) const {
  for (const auto &status : ticket.statuses) {
    if (!outbox_.delivered(status)) {
      return false;
    }
  }
  return true;
}

ShutdownTransactionResult
advanceShutdownTransaction(GoalReplanRuntimeCoordinator &goal_replan_runtime,
                           MotionStopBarrier &motion_stop,
                           GoalTerminalStatusDelivery &goal_terminal_delivery,
                           double steady_now_s) {
  ShutdownTransactionResult result;
  result.runtime_result = goal_replan_runtime.interrupt(
      GoalReplanRuntimeInterruption::kShutdown, steady_now_s);
  result.terminal_required = result.runtime_result.terminal_intent_id != 0U &&
                             result.runtime_result.terminal_after_stop.has_value();

  if (!result.terminal_required) {
    const FinalShutdownResult stopped = motion_stop.finalShutdownWithoutTerminalCommit();
    result.stop_confirmed = stopped.success;
    result.decision = decideShutdownExit(result.stop_confirmed, false,
                                         goal_replan_runtime.terminalPending(), false);
    result.reason = shutdownTransactionReason(result);
    return result;
  }

  const std::uint64_t intent_id = result.runtime_result.terminal_intent_id;
  const GoalPlanTerminalAfterStop &terminal = *result.runtime_result.terminal_after_stop;
  const GoalTerminalStatusDelivery::StageResult stage =
      goal_terminal_delivery.stage(intent_id, terminal.delivery_ticket);
  const bool exact_stage = stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                           stage == GoalTerminalStatusDelivery::StageResult::kReplay;

  if (!exact_stage) {
    const FinalShutdownResult stopped = motion_stop.finalShutdownWithoutTerminalCommit();
    result.stop_confirmed = stopped.success;
  } else if (goal_terminal_delivery.isCommitted(intent_id)) {
    result.stop_confirmed = true;
  } else {
    const FinalShutdownResult stopped =
        motion_stop.finalShutdownPreservingGoalTerminal(terminal.commit);
    result.stop_confirmed = stopped.success;
    if (result.stop_confirmed) {
      (void)goal_terminal_delivery.markCommitted(intent_id);
    }
  }

  result.terminal_flush =
      goal_terminal_delivery.flushAndAcknowledge(goal_replan_runtime);
  result.delivery_acknowledged =
      result.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged;
  result.decision = decideShutdownExit(
      result.stop_confirmed, true, goal_replan_runtime.terminalPending(),
      result.delivery_acknowledged);
  result.reason = shutdownTransactionReason(result);
  return result;
}

}  // namespace lingtu::nav::endpoint
