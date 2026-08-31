#pragma once

#include <cstdint>
#include <string>

#include "planning/global/contract.hpp"

namespace lingtu::nav::endpoint {

// Typed causes which may request the one bounded replacement plan owned by the
// GoalPlan runtime. Absence of a trigger is represented by std::nullopt at the
// producer boundary, so this enum intentionally has no untyped/unknown value.
enum class GoalReplanTriggerKind {
  kLocalRecoveryExhausted,
  kPersistentPathObstruction,
};

// Stable identity of the active external goal for which a replan is requested.
// The immutable map identity is part of the key so evidence cannot cross a map
// activation boundary.
struct GoalReplanIdentity {
  std::string task_id;
  std::string request_id;
  std::uint64_t goal_epoch{0U};
  lingtu::nav::plan::MapIdentity map_identity{};

  [[nodiscard]] bool valid() const {
    return !task_id.empty() && !request_id.empty() && goal_epoch != 0U && map_identity.valid();
  }
};

inline bool sameGoalReplanIdentity(const GoalReplanIdentity &left,
                                   const GoalReplanIdentity &right) {
  return left.task_id == right.task_id && left.request_id == right.request_id &&
         left.goal_epoch == right.goal_epoch && left.valid() && right.valid() &&
         lingtu::nav::plan::sameMapIdentity(left.map_identity, right.map_identity);
}

struct GoalReplanTrigger {
  GoalReplanTriggerKind kind{GoalReplanTriggerKind::kLocalRecoveryExhausted};
  std::string reason;
  GoalReplanIdentity goal{};
  lingtu::nav::plan::GlobalPlanTemporaryOverlay temporary_overlay{};
};

}  // namespace lingtu::nav::endpoint
