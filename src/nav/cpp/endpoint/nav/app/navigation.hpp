#pragma once

#include <functional>
#include <optional>
#include <string>

#include "control/autonomy.hpp"
#include "command/cancel.hpp"
#include "app/goal/runtime.hpp"
#include "app/goal/terminal.hpp"

namespace lingtu::nav::endpoint {

// Transport-free observation produced by the endpoint's autonomy work. The
// endpoint retains ownership of the concrete AutonomyTickResult and DDS effects.
struct NavigationRuntimeAutonomyObservation {
  GoalReplanRuntimeFrameInput updated_frame;
  AutonomyTickOutcome outcome;
  bool inspection_active{false};
  bool rolling_segment_active{false};
};

struct NavigationRuntimeFrameActions {
  std::function<void(const GoalReplanRuntimeResult &)> complete_endpoint_work_before_autonomy;
  std::function<NavigationRuntimeAutonomyObservation(const GoalPlanSnapshot &)> run_autonomy;
  std::function<void(const GoalReplanRuntimeResult &)> apply_autonomy_outputs;
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
  std::optional<GoalTerminalResult> terminal;
};

// Result of the single production goal-admission path.  A preemption terminal
// is retained for diagnostics/replay visibility; the plan result is the only
// value the endpoint needs to publish as the command receipt.
struct NavigationRuntimeGoalSubmissionResult {
  GoalPlanSubmitResult plan_result;
  std::optional<GoalTerminalResult> preemption_terminal;
};

struct GoalResumeActions {
  std::function<void()> release_operator;
  std::function<void()> activate_route;
  std::function<void()> changed;
};

// Owns the strict, single-frame ordering of the native navigation lifecycle.
// Endpoint callbacks perform transport and subsystem effects, but cannot
// reorder planning, terminal delivery, autonomy outcome handling, or pending
// GoalPlan admission.
class NavigationRuntimeController {
 public:
  NavigationRuntimeController(GoalPlanController &goal_plan,
                              GoalReplanRuntimeCoordinator &goal_replan_runtime,
                              GoalTerminal &goal_terminal, MotionStopBarrier &motion_stop);

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
  [[nodiscard]] GoalTerminalResult
  completeTerminal(const GoalReplanRuntimeResult &runtime_result,
                   const std::string &estop_reason = "estop_latched");
  [[nodiscard]] MotionStopTerminalBarrierResult stopWhileTerminalPending();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopWhileTerminalPending(const std::string &estop_reason);

  [[nodiscard]] CommandAck cancelTask(const GoalTaskCancelRequest &request);
  [[nodiscard]] std::pair<bool, std::string> pauseTask(
      const std::string &task_id, const std::string &request_id, const std::string &reason);
  [[nodiscard]] std::pair<bool, std::string> resumeTask(
      const std::string &task_id, const std::string &request_id, double source_stamp_s,
      GoalPlanAdmissionContext context, GoalResumeActions actions, const std::string &blocker = {});
  void retryResume(const std::function<GoalPlanAdmissionContext()> &context,
                   const GoalResumeActions &actions, const std::function<std::string()> &blocker);
  void cancelResume();
  [[nodiscard]] MotionStopResult interruptRelated(
      GoalReplanRuntimeInterruption interruption, double now_s,
      const std::string &reason, std::function<bool()> commit);

 private:
  GoalPlanController &goal_plan_;
  GoalReplanRuntimeCoordinator &goal_replan_runtime_;
  GoalTerminal &goal_terminal_;
  MotionStopBarrier &motion_stop_;
  struct PendingResume {
    std::string task_id;
    std::string request_id;
    double source_stamp_s;
  };
  std::optional<PendingResume> pending_resume_;

};

}  // namespace lingtu::nav::endpoint
