#pragma once

#include "message/generated/enums.hpp"

namespace lingtu::message {

// Stable values carried by NavigationCommandRequest.kind and
// NavigationCommandAck.kind. The protocol belongs to the message module so
// producers and the authoritative endpoint do not depend on one another.

inline bool isKnownNavigationCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(NavigationCommandKind::Goal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskCancel) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Stop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Estop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ClearEstop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ResumeAutonomy) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskPause) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskResume) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskSpeedLimit) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Stand) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Sit) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::EnableMotors) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::DisableMotors) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::SwitchProfile) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::EnterExternal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ExitExternal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::EnableExternal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::SelectPlanner);
}

inline const char* navigationCommandKindName(
    NavigationCommandKind kind) noexcept {
  switch (kind) {
    case NavigationCommandKind::Goal:
      return "goal";
    case NavigationCommandKind::TaskCancel:
      return "task_cancel";
    case NavigationCommandKind::Stop:
      return "stop";
    case NavigationCommandKind::Estop:
      return "estop";
    case NavigationCommandKind::ClearEstop:
      return "clear_estop";
    case NavigationCommandKind::ResumeAutonomy:
      return "resume_autonomy";
    case NavigationCommandKind::TaskPause:
      return "task_pause";
    case NavigationCommandKind::TaskResume:
      return "task_resume";
    case NavigationCommandKind::TaskSpeedLimit:
      return "task_speed_limit";
    case NavigationCommandKind::Stand:
      return "stand";
    case NavigationCommandKind::Sit:
      return "sit";
    case NavigationCommandKind::EnableMotors:
      return "enable_motors";
    case NavigationCommandKind::DisableMotors:
      return "disable_motors";
    case NavigationCommandKind::SwitchProfile:
      return "switch_profile";
    case NavigationCommandKind::EnterExternal:
      return "enter_external";
    case NavigationCommandKind::ExitExternal:
      return "exit_external";
    case NavigationCommandKind::EnableExternal:
      return "enable_external";
    case NavigationCommandKind::SelectPlanner:
      return "select_planner";
  }
  return "unknown";
}

// Stable values carried by NavigationGoalStatus.state. Command acceptance is
// deliberately separate from execution: a goal may be accepted while its
// asynchronous global plan is still pending or may later fail.

inline bool isKnownNavigationGoalState(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(NavigationGoalState::Planning) ||
      value == static_cast<std::int32_t>(NavigationGoalState::PathActive) ||
      value == static_cast<std::int32_t>(NavigationGoalState::Failed) ||
      value == static_cast<std::int32_t>(NavigationGoalState::Reached) ||
      value == static_cast<std::int32_t>(NavigationGoalState::Cancelled) ||
      value == static_cast<std::int32_t>(NavigationGoalState::Paused);
}

inline bool isTerminalNavigationGoalState(NavigationGoalState state) noexcept {
  return state == NavigationGoalState::Failed ||
      state == NavigationGoalState::Reached ||
      state == NavigationGoalState::Cancelled;
}

inline const char* navigationGoalStateName(NavigationGoalState state) noexcept {
  switch (state) {
    case NavigationGoalState::Planning:
      return "planning";
    case NavigationGoalState::PathActive:
      return "path_active";
    case NavigationGoalState::Failed:
      return "failed";
    case NavigationGoalState::Reached:
      return "reached";
    case NavigationGoalState::Cancelled:
      return "cancelled";
    case NavigationGoalState::Paused:
      return "paused";
  }
  return "unknown";
}

}  // namespace lingtu::message
