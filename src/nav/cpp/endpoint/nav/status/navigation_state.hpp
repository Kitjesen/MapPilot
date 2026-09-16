#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "message/protocol/navigation.hpp"

namespace lingtu::nav::endpoint {

struct GoalPlanStatus;

using lingtu::message::NavigationControlState;

using lingtu::message::NavigationLifecycleState;

using lingtu::message::NavigationPlanningState;

using lingtu::message::NavigationExecutionState;

using lingtu::message::NavigationRecoveryState;

struct NavigationMapIdentity {
  std::string map_id;
  std::int64_t version{0};
};

struct NavigationStateContext {
  std::optional<NavigationMapIdentity> map;
  bool path_active{false};
  bool input_ready{true};
  std::string input_gate_reason;
  bool estop_latched{false};
  std::string estop_reason;
  bool operator_takeover{false};
  bool recovery_active{false};
  std::string authority{"none"};
};

struct NavigationStateSample {
  std::int32_t control_mode{static_cast<std::int32_t>(NavigationControlState::kUnknown)};
  std::int32_t lifecycle_state{static_cast<std::int32_t>(NavigationLifecycleState::kIdle)};
  std::string active_task_id;
  std::string active_request_id;
  std::uint64_t goal_epoch{0U};
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::int32_t planning_state{static_cast<std::int32_t>(NavigationPlanningState::kIdle)};
  std::int32_t execution_state{static_cast<std::int32_t>(NavigationExecutionState::kIdle)};
  std::int32_t recovery_state{static_cast<std::int32_t>(NavigationRecoveryState::kIdle)};
  float progress{-1.0F};
  std::string authority{"none"};
  std::string hold_reason;
  std::string failure_code;
};

class NavigationStateTracker {
 public:
  explicit NavigationStateTracker(NavigationControlState control_mode);

  void observe(const GoalPlanStatus &status);
  [[nodiscard]] NavigationStateSample sample(const NavigationStateContext &context) const;

 private:
  static bool isActiveLifecycle(std::int32_t lifecycle);

  NavigationStateSample state_;
};

}  // namespace lingtu::nav::endpoint
