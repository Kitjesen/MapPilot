#pragma once

#include "message/generated/enums.hpp"
#include <string_view>

namespace lingtu::message {

// Stable values carried by ExplorationCommandRequest.kind and
// ExplorationCommandAck.kind. Exploration session control is separate from
// NavigationCommandRequest, which carries the goals produced by exploration.

// Stable values carried by ExplorationRunEvent.kind.

// Stable values carried by ExplorationRunEvent.state.

inline bool isValidExplorationRunId(std::string_view value) noexcept {
  constexpr std::string_view kUlidAlphabet{"0123456789ABCDEFGHJKMNPQRSTVWXYZ"};
  if (value.size() != 26U || value.front() > '7') {
    return false;
  }
  for (const char character : value) {
    if (kUlidAlphabet.find(character) == std::string_view::npos) {
      return false;
    }
  }
  return true;
}

inline bool isKnownExplorationRunEventKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(ExplorationRunEventKind::kAdmitted) ||
      value == static_cast<std::int32_t>(ExplorationRunEventKind::kStateChanged) ||
      value == static_cast<std::int32_t>(ExplorationRunEventKind::kStopConfirmationFailed);
}

inline bool isKnownExplorationRunState(std::int32_t value) noexcept {
  return value >= static_cast<std::int32_t>(ExplorationRunState::kAdmitted) &&
      value <= static_cast<std::int32_t>(ExplorationRunState::kFailed);
}

inline bool explorationRunStateRequiresMotionStop(ExplorationRunState state) noexcept {
  return state == ExplorationRunState::kPaused || state == ExplorationRunState::kCompleted ||
      state == ExplorationRunState::kCancelled || state == ExplorationRunState::kFailed;
}

inline bool isTerminalExplorationRunState(ExplorationRunState state) noexcept {
  return state == ExplorationRunState::kCompleted || state == ExplorationRunState::kCancelled ||
      state == ExplorationRunState::kFailed;
}

inline bool isKnownExplorationCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(ExplorationCommandKind::kStart) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kPause) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kResume) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kStop) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kSetDirectedTarget) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kClearDirectedTarget);
}

inline const char* explorationCommandKindName(
    ExplorationCommandKind kind) noexcept {
  switch (kind) {
    case ExplorationCommandKind::kStart:
      return "start";
    case ExplorationCommandKind::kPause:
      return "pause";
    case ExplorationCommandKind::kResume:
      return "resume";
    case ExplorationCommandKind::kStop:
      return "stop";
    case ExplorationCommandKind::kSetDirectedTarget:
      return "set_directed_target";
    case ExplorationCommandKind::kClearDirectedTarget:
      return "clear_directed_target";
  }
  return "unknown";
}

}  // namespace lingtu::message
