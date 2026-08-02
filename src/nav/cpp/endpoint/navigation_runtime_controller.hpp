#pragma once

#include <functional>
#include <optional>
#include <string>

#include "motion/autonomy_tick_outcome.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"
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
