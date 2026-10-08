#include "status/navigation_state.hpp"

#include <utility>
#include <tuple>

#include "runtime/goal/plan.hpp"
#include "nav/inspection/inspection.hpp"

namespace lingtu::nav::endpoint {
namespace {
auto stateFields(const NavigationStateSample &state) {
  return std::tie(state.control_mode, state.lifecycle_state, state.active_task_id,
                  state.active_request_id, state.goal_epoch, state.map_id,
                  state.map_content_epoch, state.planning_state, state.execution_state,
                  state.recovery_state, state.progress, state.authority,
                  state.hold_reason, state.failure_code);
}
}  // namespace

NavigationStateTracker::NavigationStateTracker(NavigationControlState control_mode) {
  state_.control_mode = static_cast<std::int32_t>(control_mode);
}

void NavigationStateTracker::observe(const GoalPlanStatus &status) {
  if (!status.project_to_navigation_state || status.origin == GoalPlanOrigin::kInspection) {
    return;
  }
  state_.active_task_id = status.task_id;
  state_.active_request_id = status.request_id;
  state_.goal_epoch = status.goal_epoch;
  state_.hold_reason.clear();

  switch (status.state) {
    case lingtu::message::NavigationGoalState::Planning:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kPlanning);
      state_.planning_state = static_cast<std::int32_t>(NavigationPlanningState::kPlanning);
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kIdle);
      state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kIdle);
      state_.progress = 0.0F;
      state_.failure_code.clear();
      break;
    case lingtu::message::NavigationGoalState::PathActive:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kExecuting);
      state_.planning_state = static_cast<std::int32_t>(NavigationPlanningState::kReady);
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kFollowing);
      state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kIdle);
      state_.progress = -1.0F;
      state_.failure_code.clear();
      break;
    case lingtu::message::NavigationGoalState::Paused:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kPaused);
      state_.planning_state = static_cast<std::int32_t>(NavigationPlanningState::kReady);
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kBlocked);
      state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kIdle);
      state_.progress = -1.0F;
      state_.hold_reason = status.reason.empty() ? "task_paused" : status.reason;
      state_.failure_code.clear();
      break;
    case lingtu::message::NavigationGoalState::Failed:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kFailed);
      if (state_.planning_state == static_cast<std::int32_t>(NavigationPlanningState::kPlanning)) {
        state_.planning_state = static_cast<std::int32_t>(NavigationPlanningState::kFailed);
      }
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kBlocked);
      state_.recovery_state = static_cast<std::int32_t>(status.reason == "local_recovery_exhausted"
                                                            ? NavigationRecoveryState::kFailed
                                                            : NavigationRecoveryState::kIdle);
      state_.progress = -1.0F;
      state_.failure_code = status.reason.empty() ? "navigation_failed" : status.reason;
      break;
    case lingtu::message::NavigationGoalState::Reached:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kSuccess);
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kReached);
      state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kIdle);
      state_.progress = 1.0F;
      state_.failure_code.clear();
      break;
    case lingtu::message::NavigationGoalState::Cancelled:
      state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kCancelled);
      state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kIdle);
      state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kIdle);
      state_.progress = -1.0F;
      state_.failure_code.clear();
      break;
  }
}

void NavigationStateTracker::observeInspection(const inspection::RunStatus &status) {
  using inspection::RunState;
  using lingtu::message::NavigationGoalState;
  NavigationGoalState goal_state = NavigationGoalState::PathActive;
  switch (status.state) {
    case RunState::kIdle:
      return;
    case RunState::kValidating:
    case RunState::kPlanning:
      goal_state = NavigationGoalState::Planning;
      break;
    case RunState::kPaused:
      goal_state = NavigationGoalState::Paused;
      break;
    case RunState::kSucceeded:
      goal_state = NavigationGoalState::Reached;
      break;
    case RunState::kFailed:
      goal_state = NavigationGoalState::Failed;
      break;
    case RunState::kCancelled:
      goal_state = NavigationGoalState::Cancelled;
      break;
    case RunState::kNavigating:
    case RunState::kSettling:
    case RunState::kActionPending:
    case RunState::kDwelling:
    case RunState::kRecovering:
    case RunState::kPausing:
    case RunState::kCancelling:
      break;
  }
  observe(GoalPlanStatus{status.task_id, status.request_id, 0U, goal_state, status.reason});
  state_.map_id = status.map_id;
  state_.map_content_epoch = status.map_content_epoch;
  if (status.state == RunState::kSettling || status.state == RunState::kActionPending ||
      status.state == RunState::kDwelling || status.state == RunState::kPausing ||
      status.state == RunState::kCancelling) {
    state_.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kIdle);
  }
  if (status.state == RunState::kRecovering) {
    state_.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kRecovering);
    state_.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kActive);
  }
}

NavigationStateSample NavigationStateTracker::sample(const NavigationStateContext &context) const {
  NavigationStateSample out = state_;
  out.authority = context.authority.empty() ? "none" : context.authority;
  if (context.map.has_value()) {
    out.map_id = context.map->map_id;
    out.map_content_epoch = context.map->version;
  }

  if (context.path_active &&
      out.lifecycle_state == static_cast<std::int32_t>(NavigationLifecycleState::kIdle)) {
    out.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kExecuting);
    out.planning_state = static_cast<std::int32_t>(NavigationPlanningState::kReady);
    out.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kFollowing);
  }
  if (context.recovery_active && isActiveLifecycle(out.lifecycle_state)) {
    out.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kRecovering);
    out.execution_state = static_cast<std::int32_t>(NavigationExecutionState::kBlocked);
    out.recovery_state = static_cast<std::int32_t>(NavigationRecoveryState::kActive);
  }

  std::string hold_reason;
  if (context.estop_latched) {
    hold_reason = context.estop_reason.empty() ? "estop_latched" : context.estop_reason;
  } else if (context.operator_takeover) {
    hold_reason = "operator_takeover";
  } else if (!context.input_ready) {
    hold_reason =
        context.input_gate_reason.empty() ? "input_gate_blocked" : context.input_gate_reason;
  }
  if (!hold_reason.empty()) {
    out.hold_reason = std::move(hold_reason);
    if (isActiveLifecycle(out.lifecycle_state)) {
      out.lifecycle_state = static_cast<std::int32_t>(NavigationLifecycleState::kPaused);
    }
  }
  return out;
}

bool NavigationStateTracker::publishIfDue(
    const NavigationStateSample &sample, double now_s,
    const std::function<bool(const NavigationStateSample &)> &publish) {
  // Changes are immediate; an unchanged heartbeat keeps Host freshness valid.
  if (last_published_ && stateFields(*last_published_) == stateFields(sample) &&
      now_s >= last_published_s_ && now_s - last_published_s_ < 0.2) {
    return false;
  }
  if (!publish(sample)) return false;  // Retry on the next tick after a failed write.
  last_published_ = sample;
  last_published_s_ = now_s;
  return true;
}

bool NavigationStateTracker::isActiveLifecycle(std::int32_t lifecycle) {
  return lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kPlanning) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kExecuting) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kRecovering) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kPaused);
}

}  // namespace lingtu::nav::endpoint
