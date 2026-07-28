#pragma once

#include <cstdint>

namespace lingtu::message {

// Stable values carried by NavigationCommandRequest.kind and
// NavigationCommandAck.kind. The protocol belongs to the message module so
// producers and the authoritative endpoint do not depend on one another.
enum class NavigationCommandKind : std::int32_t {
  Goal = 1,
  Cancel = 2,
  Teleop = 3,
  Stop = 4,
  Estop = 5,
  ClearEstop = 6,
  ResumeAutonomy = 7,
  TaskPause = 8,
  TaskResume = 9,
};

inline bool isKnownNavigationCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(NavigationCommandKind::Goal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Cancel) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Teleop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Stop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Estop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ClearEstop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ResumeAutonomy) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskPause) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::TaskResume);
}

inline const char* navigationCommandKindName(
    NavigationCommandKind kind) noexcept {
  switch (kind) {
    case NavigationCommandKind::Goal:
      return "goal";
    case NavigationCommandKind::Cancel:
      return "cancel";
    case NavigationCommandKind::Teleop:
      return "teleop";
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
  }
  return "unknown";
}

// Stable values carried by NavigationGoalStatus.state. Command acceptance is
// deliberately separate from execution: a goal may be accepted while its
// asynchronous global plan is still pending or may later fail.
enum class NavigationGoalState : std::int32_t {
  Planning = 1,
  PathActive = 2,
  Failed = 3,
  Reached = 4,
  Cancelled = 5,
  Paused = 6,
};

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
