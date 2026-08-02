#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "plan/goal_replan_runtime_coordinator.hpp"
#include "status/navigation_goal_status_outbox.hpp"

namespace lingtu::nav::endpoint {

class MotionStopCoordinator;

// Transport-free bridge between a stop-confirmed GoalPlan terminal commit and
// the coordinator acknowledgement. GoalPlan remains responsible for recording
// the ticket statuses in the outbox when its commit closure runs.
class GoalTerminalStatusDelivery {
 public:
  enum class StageResult {
    kStaged,
    kReplay,
    kConflict,
    kInvalidIntent,
  };

  enum class FlushResult {
    kNoStagedTerminal,
    kCommitPending,
    kDeliveryPending,
    kAcknowledged,
    kAcknowledgementRejected,
  };

  explicit GoalTerminalStatusDelivery(NavigationGoalStatusOutbox &outbox);

  // Copies one terminal ticket. Replaying the exact same intent and ticket is
  // idempotent; no other intent or semantic ticket may replace it.
  [[nodiscard]] StageResult stage(std::uint64_t intent_id,
                                  const GoalPlanTerminalDeliveryTicket &ticket);

  // The caller may invoke this only after the corresponding stop-confirmed
  // GoalPlan commit closure has returned successfully. Repeated matching calls
  // are idempotent.
  [[nodiscard]] bool markCommitted(std::uint64_t intent_id);

  // The terminal delivery bridge is the sole source of truth for whether the
  // staged intent has already crossed the stop-confirmed GoalPlan commit.
  [[nodiscard]] bool isCommitted(std::uint64_t intent_id) const;

  // Does not flush before markCommitted(). Once committed, flushes the outbox
  // and acknowledges only after every staged ticket status is delivered.
  [[nodiscard]] FlushResult flushAndAcknowledge(GoalReplanRuntimeCoordinator &coordinator);

 private:
  struct StagedTerminal {
    std::uint64_t intent_id{0U};
    GoalPlanTerminalDeliveryTicket ticket;
    bool committed{false};
  };

  [[nodiscard]] static bool sameTicket(const GoalPlanTerminalDeliveryTicket &left,
                                       const GoalPlanTerminalDeliveryTicket &right);
  [[nodiscard]] static bool sameStatus(const GoalPlanStatus &left, const GoalPlanStatus &right);
  [[nodiscard]] bool ticketDelivered(const GoalPlanTerminalDeliveryTicket &ticket) const;

  NavigationGoalStatusOutbox &outbox_;
  std::optional<StagedTerminal> staged_terminal_;
};

struct ShutdownTransactionResult {
  GoalReplanRuntimeResult runtime_result;
  bool stop_confirmed{false};
  bool terminal_required{false};
  GoalTerminalStatusDelivery::FlushResult terminal_flush{
      GoalTerminalStatusDelivery::FlushResult::kNoStagedTerminal};
  bool delivery_acknowledged{false};
  ShutdownExitDecision decision;
  std::string reason;
};

[[nodiscard]] ShutdownTransactionResult
advanceShutdownTransaction(GoalReplanRuntimeCoordinator &goal_replan_runtime,
                           MotionStopCoordinator &motion_stop,
                           GoalTerminalStatusDelivery &goal_terminal_delivery,
                           double steady_now_s);

}  // namespace lingtu::nav::endpoint
