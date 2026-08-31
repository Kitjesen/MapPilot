#pragma once

#include <string_view>

namespace lingtu::nav::endpoint {

enum class GoalTerminalBarrierState {
  kOpen,
  kTerminalPending,
};

enum class GoalTerminalIngressKind {
  kUnknown,
  kTypedGoal,
  kTypedTaskCancel,
  kTypedTaskPause,
  kTypedTaskResume,
  kTypedClearEstop,
  kTypedResumeAutonomy,
  kTypedClientClockSync,
  kTypedStop,
  kTypedEstop,
  kOperatorClaim,
  kOperatorHold,
  kOperatorRelease,
  kOperatorMotionSample,
  kInspectionCommand,
  kInspectionGoalDispatch,
  kRollingCommand,
  kCount,
};

enum class GoalTerminalIngressDecision {
  kAllow,
  kReject,
  kSafetyStop,
  kSafetyEstop,
};

struct GoalTerminalIngressVerdict {
  GoalTerminalIngressDecision decision{GoalTerminalIngressDecision::kReject};
  std::string_view reason{"goal_terminal_pending"};
};

[[nodiscard]] GoalTerminalIngressVerdict evaluateGoalTerminalIngress(
    GoalTerminalBarrierState state, GoalTerminalIngressKind kind) noexcept;

}  // namespace lingtu::nav::endpoint
