#pragma once

#include <functional>
#include <string>

#include "safety/stop.hpp"

namespace lingtu::nav::endpoint {

class GoalReplanRuntimeCoordinator;
struct GoalReplanRuntimeResult;
class GoalTerminalStatusDelivery;

struct GoalTerminalTransactionActions {
  std::function<void(const std::string &)> report_error;
  std::function<void(const std::string &)> pause_inspection;
  std::function<void()> sync_goal_diagnostics;
};

struct GoalTerminalTransactionResult {
  bool action_committed{false};
  bool delivery_acknowledged{false};
  std::string reason;
};

// Owns the ordinary Goal terminal transaction from ticket staging through
// stop-confirmed commit, durable status delivery, and exact-intent acknowledgement.
// Shutdown uses its dedicated final-stop transaction instead of this module.
class GoalTerminalTransaction {
 public:
  GoalTerminalTransaction(GoalReplanRuntimeCoordinator &goal_replan_runtime,
                          MotionStopBarrier &motion_stop,
                          GoalTerminalStatusDelivery &goal_terminal_delivery,
                          GoalTerminalTransactionActions actions);

  [[nodiscard]] GoalTerminalTransactionResult
  advance(const GoalReplanRuntimeResult &runtime_result,
          const std::string &estop_reason = "estop_latched");

  // Executes a safety stop without replacing the already-pending Goal terminal.
  // The exact pending terminal is staged and committed only after stop confirmation;
  // delivery and acknowledgement remain owned by advance().
  [[nodiscard]] MotionStopTerminalBarrierResult stopWhileTerminalPending();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopWhileTerminalPending(const std::string &estop_reason);

 private:
  enum class PendingTerminalSafetyAction {
    kStop,
    kEstop,
  };

  [[nodiscard]] MotionStopTerminalBarrierResult
  runWhileTerminalPending(PendingTerminalSafetyAction safety_action,
                          const std::string &estop_reason);

  GoalReplanRuntimeCoordinator &goal_replan_runtime_;
  MotionStopBarrier &motion_stop_;
  GoalTerminalStatusDelivery &goal_terminal_delivery_;
  GoalTerminalTransactionActions actions_;
};

}  // namespace lingtu::nav::endpoint
