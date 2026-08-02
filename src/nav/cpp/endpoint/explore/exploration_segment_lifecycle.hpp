#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>

#include "plan/exploration_segment_contract.hpp"

namespace lingtu::nav::endpoint {

inline constexpr std::string_view kGoalOutsideStaticMapReason{"goal_outside_static_map"};

[[nodiscard]] constexpr bool isExplorationSegmentFallbackReason(std::string_view reason) noexcept {
  return reason == kGoalOutsideStaticMapReason;
}

// DDS-free correlation and release rules for a live rolling-map segment.
// The full map identity remains owned by the caller; this seam uses the fields
// carried by the segment wire protocol.
struct ExplorationSegmentRequestBinding {
  std::string request_id;
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t minimum_generation{0U};

  [[nodiscard]] bool valid() const noexcept {
    return !request_id.empty() && !session_id.empty() && reset_epoch != 0U &&
           minimum_generation != 0U;
  }
};

struct ExplorationSegmentIdentity {
  std::string frame_id;
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};

  [[nodiscard]] bool valid() const noexcept {
    return frame_id == "map" && !session_id.empty() && reset_epoch != 0U && generation != 0U &&
           live;
  }
};

struct ExplorationSegmentAckEvent {
  std::string request_id;
  std::int32_t kind{0};
  bool accepted{false};
  ExplorationSegmentIdentity identity;
  std::string reason;
};

struct ExplorationSegmentStatusEvent {
  std::string request_id;
  std::int32_t state{0};
  ExplorationSegmentIdentity identity;
  std::string reason;
};

struct ExplorationSegmentLifecycleReaction {
  bool matched{false};
  bool terminal{false};
  std::uint64_t observed_generation{0U};
  std::uint64_t terminal_generation{0U};
  std::string reason;
};

struct ExplorationSegmentReplanBarrier {
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t terminal_generation{0U};

  [[nodiscard]] bool valid() const noexcept {
    return !session_id.empty() && reset_epoch != 0U && terminal_generation != 0U;
  }
};

[[nodiscard]] inline bool
matchesExplorationSegmentBinding(const ExplorationSegmentRequestBinding &expected,
                                 const std::string &request_id,
                                 const ExplorationSegmentIdentity &actual,
                                 std::uint64_t expected_execution_generation = 0U) noexcept {
  return expected.valid() && actual.valid() && expected.request_id == request_id &&
         expected.session_id == actual.session_id && expected.reset_epoch == actual.reset_epoch &&
         actual.generation >= expected.minimum_generation &&
         (expected_execution_generation == 0U ||
          actual.generation == expected_execution_generation);
}

// A native owner may reject execute before it has a live execution grid. That
// narrow pre-bind path still requires the immutable request binding exactly.
[[nodiscard]] inline bool
matchesUnboundExecuteRejection(const ExplorationSegmentRequestBinding &expected,
                               const ExplorationSegmentAckEvent &event) noexcept {
  return expected.valid() &&
         event.kind == static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute) &&
         !event.accepted && event.request_id == expected.request_id &&
         event.identity.frame_id == "map" && event.identity.session_id == expected.session_id &&
         event.identity.reset_epoch == expected.reset_epoch &&
         (event.identity.generation == 0U ||
          event.identity.generation >= expected.minimum_generation);
}

[[nodiscard]] inline ExplorationSegmentLifecycleReaction reactToExplorationSegmentAck(
    const ExplorationSegmentRequestBinding *pending, ExplorationSegmentCommandKind expected_kind,
    std::uint64_t expected_execution_generation, const ExplorationSegmentAckEvent &event) {
  ExplorationSegmentLifecycleReaction reaction;
  if (pending == nullptr || event.kind != static_cast<std::int32_t>(expected_kind)) {
    return reaction;
  }
  if (!event.accepted && expected_kind == ExplorationSegmentCommandKind::kExecute &&
      expected_execution_generation == 0U && matchesUnboundExecuteRejection(*pending, event)) {
    reaction.matched = true;
    reaction.terminal = true;
    reaction.reason = event.reason;
    reaction.observed_generation = event.identity.generation;
    reaction.terminal_generation =
        event.identity.generation == 0U ? pending->minimum_generation : event.identity.generation;
    return reaction;
  }
  if (!matchesExplorationSegmentBinding(*pending, event.request_id, event.identity,
                                        expected_execution_generation)) {
    return reaction;
  }

  reaction.matched = true;
  reaction.reason = event.reason;
  reaction.observed_generation = event.identity.generation;
  if (!event.accepted && expected_kind == ExplorationSegmentCommandKind::kExecute) {
    reaction.terminal = true;
    reaction.terminal_generation = event.identity.generation;
  }
  return reaction;
}

[[nodiscard]] inline ExplorationSegmentLifecycleReaction
reactToExplorationSegmentStatus(const ExplorationSegmentRequestBinding *pending,
                                std::uint64_t expected_execution_generation,
                                const ExplorationSegmentStatusEvent &event) {
  ExplorationSegmentLifecycleReaction reaction;
  if (pending == nullptr || !isKnownExplorationSegmentState(event.state) ||
      !matchesExplorationSegmentBinding(*pending, event.request_id, event.identity,
                                        expected_execution_generation)) {
    return reaction;
  }

  reaction.matched = true;
  reaction.reason = event.reason;
  reaction.observed_generation = event.identity.generation;
  const auto state = static_cast<ExplorationSegmentState>(event.state);
  if (isTerminalExplorationSegmentState(state)) {
    reaction.terminal = true;
    reaction.terminal_generation = event.identity.generation;
  }
  return reaction;
}

[[nodiscard]] inline ExplorationSegmentReplanBarrier
makeExplorationSegmentReplanBarrier(const ExplorationSegmentRequestBinding &binding,
                                    std::uint64_t terminal_generation,
                                    std::uint64_t observed_snapshot_generation = 0U) {
  return {
      binding.session_id,
      binding.reset_epoch,
      std::max(std::max(binding.minimum_generation, terminal_generation),
               observed_snapshot_generation),
  };
}

[[nodiscard]] inline bool
hasNewerExplorationSegmentSnapshot(const ExplorationSegmentReplanBarrier &barrier,
                                   const ExplorationSegmentIdentity &snapshot) noexcept {
  return barrier.valid() && snapshot.valid() && barrier.session_id == snapshot.session_id &&
         barrier.reset_epoch == snapshot.reset_epoch &&
         snapshot.generation > barrier.terminal_generation;
}

}  // namespace lingtu::nav::endpoint
