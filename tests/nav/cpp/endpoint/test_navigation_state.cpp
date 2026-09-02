#include <stdexcept>
#include <string>

#include "runtime/goal/plan.hpp"
#include "status/navigation_state.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::NavigationControlState;
using lingtu::nav::endpoint::NavigationExecutionState;
using lingtu::nav::endpoint::NavigationLifecycleState;
using lingtu::nav::endpoint::NavigationMapIdentity;
using lingtu::nav::endpoint::NavigationPlanningState;
using lingtu::nav::endpoint::NavigationRecoveryState;
using lingtu::nav::endpoint::NavigationStateContext;
using lingtu::nav::endpoint::NavigationStateTracker;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testLifecycleAndTransientHold() {
  NavigationStateTracker tracker(NavigationControlState::kAutonomy);
  tracker.observe(GoalPlanStatus{
      "navigation-task-7",
      "goal-7",
      7U,
      NavigationGoalState::Planning,
      "planning",
  });

  NavigationStateContext context;
  context.map = NavigationMapIdentity{"factory", 12};
  context.authority = "autonomy";
  auto state = tracker.sample(context);
  require(state.active_task_id == "navigation-task-7", "planning task identity missing");
  require(state.active_request_id == "goal-7", "planning request identity missing");
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kPlanning),
          "planning lifecycle missing");
  require(state.planning_state == static_cast<int>(NavigationPlanningState::kPlanning),
          "planning state missing");
  require(state.map_id == "factory" && state.map_content_epoch == 12, "map identity missing");

  context.input_ready = false;
  context.input_gate_reason = "localization_stale";
  state = tracker.sample(context);
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kPaused),
          "active goal must pause while input gate is blocked");
  require(state.hold_reason == "localization_stale", "hold reason missing");

  context.input_ready = true;
  state = tracker.sample(context);
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kPlanning),
          "transient hold must not destroy the underlying lifecycle");
  require(state.hold_reason.empty(), "transient hold leaked");
}

void testExecutionRecoveryAndTerminalState() {
  NavigationStateTracker tracker(NavigationControlState::kAutonomy);
  tracker.observe(GoalPlanStatus{
      "navigation-task-9",
      "goal-9",
      9U,
      NavigationGoalState::PathActive,
      "path_active",
  });

  NavigationStateContext context;
  context.path_active = true;
  context.authority = "autonomy";
  auto state = tracker.sample(context);
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kExecuting),
          "path-active lifecycle missing");
  require(state.execution_state == static_cast<int>(NavigationExecutionState::kFollowing),
          "following state missing");

  context.recovery_active = true;
  state = tracker.sample(context);
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kRecovering),
          "recovery lifecycle missing");
  require(state.recovery_state == static_cast<int>(NavigationRecoveryState::kActive),
          "recovery state missing");

  tracker.observe(GoalPlanStatus{
      "navigation-task-9",
      "goal-9",
      9U,
      NavigationGoalState::Reached,
      "goal_reached",
  });
  context = {};
  state = tracker.sample(context);
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kSuccess),
          "success lifecycle missing");
  require(state.execution_state == static_cast<int>(NavigationExecutionState::kReached),
          "reached execution state missing");
  require(state.recovery_state == static_cast<int>(NavigationRecoveryState::kIdle),
          "ordinary goal success must not be reported as recovery success");
  require(state.progress == 1.0F, "terminal progress missing");
}

void testFailureCarriesStableCode() {
  NavigationStateTracker tracker(NavigationControlState::kTeleopAvoid);
  tracker.observe(GoalPlanStatus{
      "navigation-task-11",
      "goal-11",
      11U,
      NavigationGoalState::Failed,
      "local_recovery_exhausted",
  });
  const auto state = tracker.sample({});
  require(state.lifecycle_state == static_cast<int>(NavigationLifecycleState::kFailed),
          "failure lifecycle missing");
  require(state.execution_state == static_cast<int>(NavigationExecutionState::kBlocked),
          "failed execution must be blocked");
  require(state.failure_code == "local_recovery_exhausted", "failure code missing");
  require(state.recovery_state == static_cast<int>(NavigationRecoveryState::kFailed),
          "local recovery exhaustion must remain a recovery failure");

  tracker.observe(GoalPlanStatus{
      "navigation-task-12",
      "goal-12",
      12U,
      NavigationGoalState::Failed,
      "global_planner_failed",
  });
  const auto ordinary_failure = tracker.sample({});
  require(ordinary_failure.failure_code == "global_planner_failed",
          "ordinary failure code missing");
  require(ordinary_failure.recovery_state == static_cast<int>(NavigationRecoveryState::kIdle),
          "ordinary goal failure must not invent a recovery terminal state");
}
void testExplicitTaskPausePersistsUntilPathResumes() {
  NavigationStateTracker tracker(NavigationControlState::kAutonomy);
  tracker.observe(GoalPlanStatus{
      "navigation-task-pause",
      "pause-request-1",
      21U,
      NavigationGoalState::Paused,
      "operator_pause",
  });

  const auto paused = tracker.sample({});
  require(paused.lifecycle_state == static_cast<int>(NavigationLifecycleState::kPaused),
          "explicit task pause did not project to PAUSED lifecycle");
  require(paused.active_task_id == "navigation-task-pause" &&
              paused.active_request_id == "pause-request-1",
          "explicit task pause lost task or request identity");
  require(paused.hold_reason == "operator_pause", "explicit task pause reason missing");

  tracker.observe(GoalPlanStatus{
      "navigation-task-pause", "resume-request-1", 21U,
      NavigationGoalState::PathActive, "path_resumed"});
  const auto resumed = tracker.sample({});
  require(resumed.lifecycle_state == static_cast<int>(NavigationLifecycleState::kExecuting),
          "task resume did not restore executing lifecycle");
  require(resumed.hold_reason.empty(), "task resume retained the pause reason");
}

void testPendingTaskStatusesDoNotReplaceActiveRuntimeProjection() {
  NavigationStateTracker tracker(NavigationControlState::kAutonomy);
  tracker.observe(GoalPlanStatus{"navigation-task-active", "goal-active", 31U,
                                 NavigationGoalState::PathActive, "path_active"});

  GoalPlanStatus queued{"navigation-task-pending", "goal-pending", 32U,
                        NavigationGoalState::Planning, "planning_queued"};
  queued.project_to_navigation_state = false;
  tracker.observe(queued);

  const auto after_queued = tracker.sample({});
  require(after_queued.active_task_id == "navigation-task-active" &&
              after_queued.active_request_id == "goal-active" &&
              after_queued.lifecycle_state ==
                  static_cast<int>(NavigationLifecycleState::kExecuting),
          "queued task replaced the active runtime projection");

  GoalPlanStatus pending_cancelled{"navigation-task-pending", "cancel-pending", 32U,
                                   NavigationGoalState::Cancelled, "operator_cancel"};
  pending_cancelled.project_to_navigation_state = false;
  tracker.observe(pending_cancelled);

  const auto after_cancelled = tracker.sample({});
  require(after_cancelled.active_task_id == "navigation-task-active" &&
              after_cancelled.active_request_id == "goal-active" &&
              after_cancelled.lifecycle_state ==
                  static_cast<int>(NavigationLifecycleState::kExecuting),
          "pending cancellation replaced the active runtime projection");
}
}  // namespace

int main() {
  testLifecycleAndTransientHold();
  testExecutionRecoveryAndTerminalState();
  testFailureCarriesStableCode();
  testExplicitTaskPausePersistsUntilPathResumes();
  testPendingTaskStatusesDoNotReplaceActiveRuntimeProjection();
  return 0;
}
