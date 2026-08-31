#include <array>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "command/terminal.hpp"

namespace {

using lingtu::nav::endpoint::GoalTerminalBarrierState;
using lingtu::nav::endpoint::GoalTerminalIngressDecision;
using lingtu::nav::endpoint::GoalTerminalIngressKind;
using lingtu::nav::endpoint::GoalTerminalIngressVerdict;
using lingtu::nav::endpoint::evaluateGoalTerminalIngress;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_goal_terminal_ingress_policy: FAIL: %s\n", message);
    std::exit(1);
  }
}

constexpr std::array kKnownIngressKinds{
    GoalTerminalIngressKind::kTypedGoal,
    GoalTerminalIngressKind::kTypedTaskCancel,
    GoalTerminalIngressKind::kTypedTaskPause,
    GoalTerminalIngressKind::kTypedTaskResume,
    GoalTerminalIngressKind::kTypedClearEstop,
    GoalTerminalIngressKind::kTypedResumeAutonomy,
    GoalTerminalIngressKind::kTypedClientClockSync,
    GoalTerminalIngressKind::kTypedStop,
    GoalTerminalIngressKind::kTypedEstop,
    GoalTerminalIngressKind::kOperatorClaim,
    GoalTerminalIngressKind::kOperatorHold,
    GoalTerminalIngressKind::kOperatorRelease,
    GoalTerminalIngressKind::kOperatorMotionSample,
    GoalTerminalIngressKind::kInspectionCommand,
    GoalTerminalIngressKind::kInspectionGoalDispatch,
    GoalTerminalIngressKind::kRollingCommand,
};
static_assert(
    kKnownIngressKinds.size() ==
        static_cast<std::size_t>(GoalTerminalIngressKind::kCount) - 1U,
    "the known ingress matrix must name every production ingress kind");

constexpr std::array kPendingGenericRejectKinds{
    GoalTerminalIngressKind::kTypedGoal,
    GoalTerminalIngressKind::kTypedTaskCancel,
    GoalTerminalIngressKind::kTypedTaskPause,
    GoalTerminalIngressKind::kTypedTaskResume,
    GoalTerminalIngressKind::kTypedClearEstop,
    GoalTerminalIngressKind::kTypedResumeAutonomy,
    GoalTerminalIngressKind::kOperatorClaim,
    GoalTerminalIngressKind::kOperatorHold,
    GoalTerminalIngressKind::kOperatorRelease,
    GoalTerminalIngressKind::kInspectionCommand,
    GoalTerminalIngressKind::kInspectionGoalDispatch,
    GoalTerminalIngressKind::kRollingCommand,
};

void expect(GoalTerminalIngressKind kind, GoalTerminalBarrierState state,
            GoalTerminalIngressDecision decision, std::string_view reason) {
  const GoalTerminalIngressVerdict verdict = evaluateGoalTerminalIngress(state, kind);
  require(verdict.decision == decision, "decision mismatch");
  require(verdict.reason == reason, "reason mismatch");
}

void testOpenBarrierAllowsEveryIngress() {
  for (const auto kind : kKnownIngressKinds) {
    expect(kind, GoalTerminalBarrierState::kOpen, GoalTerminalIngressDecision::kAllow,
           "goal_terminal_barrier_open");
  }
}

void testPendingBarrierRejectsMutatingIngress() {
  for (const auto kind : kPendingGenericRejectKinds) {
    expect(kind, GoalTerminalBarrierState::kTerminalPending,
           GoalTerminalIngressDecision::kReject, "goal_terminal_pending");
  }
}

void testPendingBarrierPreservesSpecialCases() {
  expect(GoalTerminalIngressKind::kOperatorMotionSample,
         GoalTerminalBarrierState::kTerminalPending, GoalTerminalIngressDecision::kReject,
         "operator_motion_sample_goal_terminal_pending");
  expect(GoalTerminalIngressKind::kTypedClientClockSync,
         GoalTerminalBarrierState::kTerminalPending, GoalTerminalIngressDecision::kAllow,
         "clock_synchronized");
  expect(GoalTerminalIngressKind::kTypedStop, GoalTerminalBarrierState::kTerminalPending,
         GoalTerminalIngressDecision::kSafetyStop,
         "safety_stop_preserve_goal_terminal");
  expect(GoalTerminalIngressKind::kTypedEstop, GoalTerminalBarrierState::kTerminalPending,
         GoalTerminalIngressDecision::kSafetyEstop,
         "safety_estop_preserve_goal_terminal");
}

void testUnknownIngressAlwaysFailsClosed() {
  constexpr std::array unknown_kinds{
      GoalTerminalIngressKind::kUnknown,
      GoalTerminalIngressKind::kCount,
      static_cast<GoalTerminalIngressKind>(-1),
      static_cast<GoalTerminalIngressKind>(999),
  };
  for (const auto state :
       {GoalTerminalBarrierState::kOpen, GoalTerminalBarrierState::kTerminalPending}) {
    for (const auto kind : unknown_kinds) {
      expect(kind, state, GoalTerminalIngressDecision::kReject,
             "goal_terminal_ingress_unknown");
    }
  }
  for (const auto state :
       {static_cast<GoalTerminalBarrierState>(-1),
        static_cast<GoalTerminalBarrierState>(999)}) {
    for (const auto kind : unknown_kinds) {
      expect(kind, state, GoalTerminalIngressDecision::kReject,
             "goal_terminal_ingress_unknown");
    }
  }
}

void testUnknownBarrierAlwaysFailsClosed() {
  constexpr std::array unknown_states{
      static_cast<GoalTerminalBarrierState>(-1),
      static_cast<GoalTerminalBarrierState>(999),
  };
  for (const auto state : unknown_states) {
    for (const auto kind : kKnownIngressKinds) {
      expect(kind, state, GoalTerminalIngressDecision::kReject,
             "goal_terminal_barrier_unknown");
    }
  }
}

}  // namespace

int main() {
  testOpenBarrierAllowsEveryIngress();
  testPendingBarrierRejectsMutatingIngress();
  testPendingBarrierPreservesSpecialCases();
  testUnknownIngressAlwaysFailsClosed();
  testUnknownBarrierAlwaysFailsClosed();
  return 0;
}
