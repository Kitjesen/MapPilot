#pragma once

#include <optional>
#include <string>

#include "plan/goal_replan_trigger.hpp"

namespace lingtu::nav::endpoint {

enum class AutonomyTickOutcomeKind {
  kNone,
  kRollingFinalSafetyStopped,
  kRollingRecoveryExhausted,
  kRollingReached,
  kGoalFailed,
  kGoalReached,
};

struct AutonomyTickOutcome {
  AutonomyTickOutcomeKind kind{AutonomyTickOutcomeKind::kNone};
  std::string reason;
  bool inspection_arrival_intent{false};
  std::optional<GoalReplanTrigger> replan_trigger;
};

}  // namespace lingtu::nav::endpoint
