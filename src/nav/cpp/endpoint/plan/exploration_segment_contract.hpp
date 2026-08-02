#pragma once

#include <cstdint>

namespace lingtu::nav::endpoint {

// P3 is deliberately a separate contract from generic NavigationCommandRequest.
// These values are sent on DDS, so keep them stable and validate every raw value
// at the native process boundary.
enum class ExplorationSegmentCommandKind : std::int32_t {
  kExecute = 1,
  kCancel = 2,
};

enum class ExplorationSegmentState : std::int32_t {
  kAccepted = 1,
  kExecuting = 2,
  kReached = 3,
  kFailed = 4,
  kCancelled = 5,
  kStaleBinding = 6,
};

constexpr bool isKnownExplorationSegmentCommandKind(std::int32_t raw) noexcept {
  return raw == static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute) ||
         raw == static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
}

constexpr bool isKnownExplorationSegmentState(std::int32_t raw) noexcept {
  return raw >= static_cast<std::int32_t>(ExplorationSegmentState::kAccepted) &&
         raw <= static_cast<std::int32_t>(ExplorationSegmentState::kStaleBinding);
}

constexpr bool isTerminalExplorationSegmentState(ExplorationSegmentState state) noexcept {
  return state == ExplorationSegmentState::kReached || state == ExplorationSegmentState::kFailed ||
         state == ExplorationSegmentState::kCancelled ||
         state == ExplorationSegmentState::kStaleBinding;
}

}  // namespace lingtu::nav::endpoint
