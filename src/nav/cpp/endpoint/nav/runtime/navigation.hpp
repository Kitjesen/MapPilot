#pragma once

#include <functional>
#include <optional>
#include <string>

#include "control/autonomy.hpp"
#include "runtime/goal/runtime.hpp"
#include "status/goal_terminal_transaction.hpp"

namespace lingtu::nav::endpoint {

// Transport-free observation produced by the endpoint's autonomy work. The
// endpoint retains ownership of the concrete AutonomyTickResult and DDS effects.
struct NavigationRuntimeAutonomyObservation {
  GoalReplanRuntimeFrameInput updated_frame;
  AutonomyTickOutcome outcome;
  bool autonomy_tick_handled{false};
  bool inspection_active{false};
  bool rolling_segment_active{false};
};

// Inspection state sampled after the endpoint has applied autonomy outputs.
// This preserves the production fallback decision point without moving
// inspection execution into the navigation runtime controller.
struct NavigationRuntimePostAutonomyState {
  bool inspection_active{false};
  bool inspection_navigation_active{false};
};

struct NavigationRuntimeFrameActions {
  std::function<void(const GoalReplanRuntimeResult &)> complete_endpoint_work_before_autonomy;
  std::function<NavigationRuntimeAutonomyObservation(const GoalPlanSnapshot &)> run_autonomy;
  std::function<NavigationRuntimePostAutonomyState(const GoalReplanRuntimeResult &)>
      apply_autonomy_outputs;
};

struct NavigationRuntimeFrameResult {
  GoalReplanRuntimeResult planning_result;
  std::optional<GoalReplanRuntimeResult> autonomy_result;
  GoalReplanRuntimeResult pending_result;
  bool pending_cycle_advanced{false};
  bool terminal_delivery_acknowledged{false};
  std::optional<AutonomyTickOutcome> inspection_completion;
};

// Result of a lifecycle interruption routed through the controller.  The
// endpoint may still need to perform a physical-only stop when no GoalPlan
// terminal exists, but any surfaced terminal is completed here so callers do
// not have to reach into GoalReplanRuntimeCoordinator directly.
struct NavigationRuntimeInterruptionResult {
  GoalReplanRuntimeResult runtime_result;
  std::optional<GoalTerminalTransactionResult> terminal_transaction;
};

// Result of the single production goal-admission path.  A preemption terminal
// is retained for diagnostics/replay visibility; the plan result is the only
// value the endpoint needs to publish as the command receipt.
struct NavigationRuntimeGoalSubmissionResult {
  GoalPlanSubmitResult plan_result;
  std::optional<GoalTerminalTransactionResult> preemption_terminal;
};

// Owns the strict, single-frame ordering of the native navigation lifecycle.
// Endpoint callbacks perform transport and subsystem effects, but cannot
// reorder planning, terminal delivery, autonomy outcome handling, or pending
// GoalPlan admission.
class NavigationRuntimeController {
 public:
  NavigationRuntimeController(GoalPlanController &goal_plan,
                              GoalReplanRuntimeCoordinator &goal_replan_runtime,
                              GoalTerminalTransaction &goal_terminal_transaction);

  [[nodiscard]] NavigationRuntimeFrameResult
  advanceFrame(const GoalReplanRuntimeFrameInput &frame,
               const NavigationRuntimeFrameActions &actions);

  [[nodiscard]] bool terminalPending() const;
  [[nodiscard]] NavigationRuntimeInterruptionResult
  interrupt(GoalReplanRuntimeInterruption interruption, double steady_now_s,
            const std::string &estop_reason = "estop_latched");
  [[nodiscard]] NavigationRuntimeGoalSubmissionResult
  submitGoal(const GoalPlanRequest &request, const GoalPlanAdmissionContext &admission,
             double steady_now_s);
  [[nodiscard]] GoalTerminalTransactionResult
  completeTerminal(const GoalReplanRuntimeResult &runtime_result,
                   const std::string &estop_reason = "estop_latched");
  [[nodiscard]] MotionStopTerminalBarrierResult stopWhileTerminalPending();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopWhileTerminalPending(const std::string &estop_reason);

 private:
  [[nodiscard]] static bool
  shouldDeferInspectionCompletion(const AutonomyTickOutcome &outcome,
                                  const GoalReplanRuntimeResult &runtime_result,
                                  const NavigationRuntimePostAutonomyState &post_state);

  GoalPlanController &goal_plan_;
  GoalReplanRuntimeCoordinator &goal_replan_runtime_;
  GoalTerminalTransaction &goal_terminal_transaction_;
  std::optional<AutonomyTickOutcome> deferred_inspection_completion_;
};

}  // namespace lingtu::nav::endpoint
