#include "status/navigation_state.hpp"

#include <utility>

namespace lingtu::nav::endpoint {
NavigationStateTracker::NavigationStateTracker(NavigationControlState control_mode) {
  state_.control_mode = static_cast<std::int32_t>(control_mode);
}

void NavigationStateTracker::observe(const GoalPlanStatus &status) {
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

NavigationStateSample NavigationStateTracker::sample(const NavigationStateContext &context) const {
  NavigationStateSample out = state_;
  out.authority = context.authority.empty() ? "none" : context.authority;
  if (context.map.has_value()) {
    out.map_id = context.map->map_id;
    out.map_version = context.map->version;
    out.map_hash = context.map->artifact_sha256;
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

bool NavigationStateTracker::isActiveLifecycle(std::int32_t lifecycle) {
  return lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kPlanning) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kExecuting) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kRecovering) ||
         lifecycle == static_cast<std::int32_t>(NavigationLifecycleState::kPaused);
}

}  // namespace lingtu::nav::endpoint
