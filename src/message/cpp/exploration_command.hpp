#pragma once

#include <cstdint>

namespace lingtu::message {

// Stable values carried by ExplorationCommandRequest.kind and
// ExplorationCommandAck.kind. Exploration session control is separate from
// NavigationCommandRequest, which carries the goals produced by exploration.
enum class ExplorationCommandKind : std::int32_t {
  kStart = 1,
  kPause = 2,
  kResume = 3,
  kStop = 4,
};

inline bool isKnownExplorationCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(ExplorationCommandKind::kStart) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kPause) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kResume) ||
      value == static_cast<std::int32_t>(ExplorationCommandKind::kStop);
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
  }
  return "unknown";
}

}  // namespace lingtu::message
