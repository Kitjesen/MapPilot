#pragma once

#include <functional>
#include <string>

#include "safety/stop.hpp"

namespace lingtu::nav::endpoint {

class GoalReplanRuntimeCoordinator;
struct GoalReplanRuntimeResult;
class GoalTerminalStatusDelivery;

struct GoalTerminalActions {
  std::function<void(const std::string &)> report_error;
  std::function<void(const std::string &)> pause_inspection;
  std::function<void()> sync_goal_diagnostics;
};

struct GoalTerminalResult {
  bool action_committed{false};
  bool delivery_acknowledged{false};
  std::string reason;
};

// Completes an accepted task outcome: confirm stop, commit, then deliver its status.
// Arrival geometry belongs to execution; shutdown keeps its separate final-stop flow.
class GoalTerminal {
 public:
  GoalTerminal(GoalReplanRuntimeCoordinator &runtime,
               MotionStopBarrier &stop,
               GoalTerminalStatusDelivery &delivery,
               GoalTerminalActions actions);

  [[nodiscard]] GoalTerminalResult
  advance(const GoalReplanRuntimeResult &runtime_result,
          const std::string &estop_reason = "estop_latched");

  [[nodiscard]] MotionStopResult commitRelated(
      const GoalReplanRuntimeResult &result, const std::string &reason,
      std::function<bool()> commit);

  // Executes a safety stop without replacing the already-pending Goal terminal.
  // The exact pending terminal is staged and committed only after stop confirmation;
  // delivery and acknowledgement remain owned by advance().
  [[nodiscard]] MotionStopTerminalBarrierResult stopPending();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopPending(const std::string &estop_reason);

 private:
  enum class StopAction {
    kStop,
    kEstop,
  };

  [[nodiscard]] MotionStopTerminalBarrierResult
  stopPending(StopAction safety_action, const std::string &estop_reason);

  GoalReplanRuntimeCoordinator &runtime_;
  MotionStopBarrier &stop_;
  GoalTerminalStatusDelivery &delivery_;
  GoalTerminalActions actions_;
};

}  // namespace lingtu::nav::endpoint
