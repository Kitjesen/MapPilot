#include "plan/goal_terminal_ingress_policy.hpp"

#include <type_traits>

namespace lingtu::nav::endpoint {
namespace {

bool isKnownIngress(GoalTerminalIngressKind kind) noexcept {
  using RawKind = std::underlying_type_t<GoalTerminalIngressKind>;
  const auto raw = static_cast<RawKind>(kind);
  return raw > static_cast<RawKind>(GoalTerminalIngressKind::kUnknown) &&
      raw < static_cast<RawKind>(GoalTerminalIngressKind::kCount);
}

bool isKnownBarrierState(GoalTerminalBarrierState state) noexcept {
  switch (state) {
    case GoalTerminalBarrierState::kOpen:
    case GoalTerminalBarrierState::kTerminalPending:
      return true;
  }
  return false;
}

}  // namespace

GoalTerminalIngressVerdict evaluateGoalTerminalIngress(
    GoalTerminalBarrierState state, GoalTerminalIngressKind kind) noexcept {
  if (!isKnownIngress(kind)) {
    return {GoalTerminalIngressDecision::kReject, "goal_terminal_ingress_unknown"};
  }
  if (!isKnownBarrierState(state)) {
    return {GoalTerminalIngressDecision::kReject, "goal_terminal_barrier_unknown"};
  }
  if (state == GoalTerminalBarrierState::kOpen) {
    return {GoalTerminalIngressDecision::kAllow, "goal_terminal_barrier_open"};
  }

  switch (kind) {
    case GoalTerminalIngressKind::kTypedClientClockSync:
      return {GoalTerminalIngressDecision::kAllow, "clock_synchronized"};
    case GoalTerminalIngressKind::kTypedStop:
      return {GoalTerminalIngressDecision::kSafetyStop,
              "safety_stop_preserve_goal_terminal"};
    case GoalTerminalIngressKind::kTypedEstop:
      return {GoalTerminalIngressDecision::kSafetyEstop,
              "safety_estop_preserve_goal_terminal"};
    case GoalTerminalIngressKind::kOperatorMotionSample:
      return {GoalTerminalIngressDecision::kReject,
              "operator_motion_sample_goal_terminal_pending"};
    case GoalTerminalIngressKind::kTypedGoal:
    case GoalTerminalIngressKind::kTypedTaskCancel:
    case GoalTerminalIngressKind::kTypedTaskPause:
    case GoalTerminalIngressKind::kTypedTaskResume:
    case GoalTerminalIngressKind::kTypedClearEstop:
    case GoalTerminalIngressKind::kTypedResumeAutonomy:
    case GoalTerminalIngressKind::kOperatorClaim:
    case GoalTerminalIngressKind::kOperatorHold:
    case GoalTerminalIngressKind::kOperatorRelease:
    case GoalTerminalIngressKind::kInspectionCommand:
    case GoalTerminalIngressKind::kInspectionGoalDispatch:
    case GoalTerminalIngressKind::kRollingCommand:
      return {GoalTerminalIngressDecision::kReject, "goal_terminal_pending"};
    case GoalTerminalIngressKind::kUnknown:
    case GoalTerminalIngressKind::kCount:
      break;
  }

  return {GoalTerminalIngressDecision::kReject, "goal_terminal_ingress_unknown"};
}

}  // namespace lingtu::nav::endpoint
