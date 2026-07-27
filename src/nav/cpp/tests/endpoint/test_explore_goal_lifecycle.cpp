#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include "explore/exploration_segment_lifecycle.hpp"
#include "explore/explore_goal_lifecycle.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::ExplorationSegmentAckEvent;
using lingtu::nav::endpoint::ExplorationSegmentCommandKind;
using lingtu::nav::endpoint::ExplorationSegmentIdentity;
using lingtu::nav::endpoint::ExplorationSegmentRequestBinding;
using lingtu::nav::endpoint::ExplorationSegmentState;
using lingtu::nav::endpoint::ExplorationSegmentStatusEvent;
using lingtu::nav::endpoint::hasNewerExplorationSegmentSnapshot;
using lingtu::nav::endpoint::isExplorationSegmentFallbackReason;
using lingtu::nav::endpoint::makeExplorationSegmentReplanBarrier;
using lingtu::nav::endpoint::NavigationGoalLifecycleEvent;
using lingtu::nav::endpoint::PendingExploreGoalLifecycle;
using lingtu::nav::endpoint::reactToExplorationSegmentAck;
using lingtu::nav::endpoint::reactToExplorationSegmentStatus;
using lingtu::nav::endpoint::reactToNavigationGoalLifecycle;
using lingtu::nav::endpoint::shouldMarkExploreGoalVisited;

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

NavigationGoalLifecycleEvent event(const std::string &request_id, NavigationGoalState state,
                                   const std::string &reason = {}) {
  NavigationGoalLifecycleEvent value;
  value.request_id = request_id;
  value.state = state;
  value.reason = reason;
  return value;
}

ExplorationSegmentRequestBinding segmentBinding() {
  return {
      "segment-1",
      "rolling-session",
      7U,
      11U,
  };
}

ExplorationSegmentIdentity segmentIdentity(std::uint64_t generation = 11U) {
  return {
      "map", "rolling-session", 7U, generation, true,
  };
}

void testMismatchedFailureIsIgnored() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("other-goal", NavigationGoalState::Failed, "goal_outside_map"));
  require(!reaction.matched, "a different goal must not affect TARE state");
  require(!reaction.clear_pending, "a different goal must remain pending");
  require(!reaction.request_replan, "a different goal must not trigger replanning");
}

void testMatchingFailureRequestsReplan() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("tare-goal-1", NavigationGoalState::Failed, "goal_outside_map"));
  require(reaction.matched, "matching failure must be consumed");
  require(reaction.clear_pending, "matching failure must clear pending goal");
  require(reaction.exclude_target, "failed target must be excluded from next TARE plan");
  require(reaction.request_replan, "matching failure must request a new TARE plan");
  require(reaction.reason == "goal_outside_map", "failure reason must survive handoff");
}

void testTerminalStatusCannotRestoreOldMapVisitedTarget() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("tare-goal-1", NavigationGoalState::Failed, "goal_outside_map"));
  require(shouldMarkExploreGoalVisited(reaction, true),
          "a failed target from the current map must be excluded");
  require(!shouldMarkExploreGoalVisited(reaction, false),
          "a terminal status from an old map must not restore visited history");
}

void testPathActiveRetainsPendingGoal() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("tare-goal-1", NavigationGoalState::PathActive));
  require(reaction.matched, "matching active event must be observed");
  require(!reaction.clear_pending, "active path must retain pending target");
  require(!reaction.exclude_target, "active path must not exclude target");
  require(!reaction.request_replan, "active path must not replan");
}

void testMatchingReachedExcludesWithoutReplan() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("tare-goal-1", NavigationGoalState::Reached, "goal_reached"));
  require(reaction.matched, "matching arrival must be consumed");
  require(reaction.clear_pending, "arrival must clear the pending target");
  require(reaction.exclude_target, "arrival must mark the target visited");
  require(!reaction.request_replan, "arrival must not force an immediate replan");
}

void testMatchingCancelledClearsWithoutExcluding() {
  const PendingExploreGoalLifecycle pending{"tare-goal-1"};
  const auto reaction = reactToNavigationGoalLifecycle(
      &pending, event("tare-goal-1", NavigationGoalState::Cancelled, "operator_stop"));
  require(reaction.matched, "matching cancellation must be consumed");
  require(reaction.clear_pending, "cancellation must clear pending state");
  require(!reaction.exclude_target, "cancellation must not poison a valid frontier");
  require(!reaction.request_replan, "cancellation must wait for a fresh policy cycle");
}

void testTerminalEventAfterClearIsIgnored() {
  const auto reaction =
      reactToNavigationGoalLifecycle(nullptr, event("tare-goal-1", NavigationGoalState::Reached));
  require(!reaction.matched, "duplicate terminal event must be harmless after clear");
}

void testOnlyExactStaticMapBoundaryFailureUsesSegmentFallback() {
  require(isExplorationSegmentFallbackReason("goal_outside_static_map"),
          "the exact static-map boundary failure must select the fallback");
  require(!isExplorationSegmentFallbackReason("goal_outside_static_map "),
          "a suffixed static-map failure must not select the fallback");
  require(!isExplorationSegmentFallbackReason("goal_snap_exhausted"),
          "a different goal failure must not select the fallback");
}

void testSegmentAckAndStatusRequireExactBinding() {
  const auto binding = segmentBinding();
  const ExplorationSegmentAckEvent accepted{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute),
      true,
      segmentIdentity(12U),
      "segment_accepted",
  };
  const auto ack =
      reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, accepted);
  require(ack.matched, "matching execute ACK must be consumed");
  require(!ack.terminal, "accepted execute ACK must not release replanning");
  require(ack.observed_generation == 12U, "accepted ACK must pin its execution generation");

  const ExplorationSegmentStatusEvent executing{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentState::kExecuting),
      segmentIdentity(12U),
      "segment_executing",
  };
  const auto status = reactToExplorationSegmentStatus(&binding, ack.observed_generation, executing);
  require(status.matched, "matching executing status must be consumed");
  require(!status.terminal, "executing status must keep the segment active");

  auto wrong_execution_generation = executing;
  wrong_execution_generation.identity.generation = 13U;
  require(!reactToExplorationSegmentStatus(&binding, ack.observed_generation,
                                           wrong_execution_generation)
               .matched,
          "status must retain the accepted execution generation");

  auto mismatch = accepted;
  mismatch.request_id = "other-segment";
  require(
      !reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, mismatch)
           .matched,
      "a foreign request id must be ignored");
  mismatch = accepted;
  mismatch.identity.session_id = "other-session";
  require(
      !reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, mismatch)
           .matched,
      "a foreign session must be ignored");
  mismatch = accepted;
  mismatch.identity.generation = 10U;
  require(
      !reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, mismatch)
           .matched,
      "a stale generation must be ignored");
  mismatch = accepted;
  mismatch.identity.live = false;
  require(
      !reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, mismatch)
           .matched,
      "a non-live binding must be ignored");
}

void testUnboundExecuteRejectionIsTerminal() {
  const auto binding = segmentBinding();
  ExplorationSegmentIdentity no_execution_identity{
      "map", "rolling-session", 7U, 0U, false,
  };
  const ExplorationSegmentAckEvent rejected{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute),
      false,
      no_execution_identity,
      "execution_grid_missing",
  };
  const auto reaction =
      reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, rejected);
  require(reaction.matched && reaction.terminal,
          "a matching pre-bind execute rejection must be terminal");
  require(reaction.terminal_generation == binding.minimum_generation,
          "an unbound rejection must use the request minimum generation");

  auto min_generation_rejection = rejected;
  min_generation_rejection.identity.generation = binding.minimum_generation;
  const auto min_generation_reaction = reactToExplorationSegmentAck(
      &binding, ExplorationSegmentCommandKind::kExecute, 0U, min_generation_rejection);
  require(min_generation_reaction.matched && min_generation_reaction.terminal,
          "a non-live rejection may carry the request minimum generation");

  auto mismatched = rejected;
  mismatched.identity.session_id = "other-session";
  require(!reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U,
                                        mismatched)
               .matched,
          "an unbound rejection still requires its exact session binding");
  require(!reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 12U,
                                        rejected)
               .matched,
          "a pre-bind rejection must not replace a pinned execution generation");
}

void testSegmentStatusMayArriveBeforeExecuteAck() {
  const auto binding = segmentBinding();
  const ExplorationSegmentStatusEvent executing{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentState::kExecuting),
      segmentIdentity(12U),
      "segment_executing",
  };
  const auto status = reactToExplorationSegmentStatus(&binding, 0U, executing);
  require(status.matched, "a valid status may arrive before its execute ACK");
  require(status.observed_generation == 12U,
          "the early status must establish the execution generation");

  const ExplorationSegmentAckEvent accepted{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute),
      true,
      segmentIdentity(12U),
      "segment_accepted",
  };
  require(reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute,
                                       status.observed_generation, accepted)
              .matched,
          "the delayed ACK must match the generation pinned by status");
}

void testSegmentTerminalOutcomeNeedsNewerSnapshotBeforeReplan() {
  const auto binding = segmentBinding();
  const ExplorationSegmentAckEvent rejected{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute),
      false,
      segmentIdentity(12U),
      "segment_target_rejected",
  };
  const auto reaction =
      reactToExplorationSegmentAck(&binding, ExplorationSegmentCommandKind::kExecute, 0U, rejected);
  require(reaction.matched && reaction.terminal, "a matching execute rejection is terminal");
  const auto barrier = makeExplorationSegmentReplanBarrier(binding, reaction.terminal_generation);
  require(!hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(12U)),
          "the terminal generation must not replan immediately");
  require(!hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(11U)),
          "an older generation must not release replanning");
  require(hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(13U)),
          "a newer same-epoch snapshot must release replanning");
}

void testSegmentBarrierWaitsForSnapshotObservedAfterTerminal() {
  const auto binding = segmentBinding();
  const auto barrier = makeExplorationSegmentReplanBarrier(binding, 12U, 13U);
  require(!hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(13U)),
          "a snapshot already observed before terminal handling must not replan");
  require(hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(14U)),
          "the first later same-epoch snapshot must release replanning");
}

void testSegmentCancelWaitsForTerminalStatus() {
  const auto binding = segmentBinding();
  const ExplorationSegmentAckEvent rejected_cancel{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel),
      false,
      segmentIdentity(12U),
      "segment_cancel_rejected",
  };
  const auto cancel_ack = reactToExplorationSegmentAck(
      &binding, ExplorationSegmentCommandKind::kCancel, 12U, rejected_cancel);
  require(cancel_ack.matched, "matching cancel ACK must be observed");
  require(!cancel_ack.terminal, "a rejected cancel must not release the active segment");

  const ExplorationSegmentStatusEvent cancelled{
      "segment-1",
      static_cast<std::int32_t>(ExplorationSegmentState::kCancelled),
      segmentIdentity(12U),
      "segment_cancelled",
  };
  const auto status = reactToExplorationSegmentStatus(&binding, 12U, cancelled);
  require(status.matched && status.terminal, "matching cancelled status must be terminal");
  const auto barrier = makeExplorationSegmentReplanBarrier(binding, status.terminal_generation);
  require(!hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(12U)),
          "terminal cancellation still needs a newer snapshot");
  require(hasNewerExplorationSegmentSnapshot(barrier, segmentIdentity(13U)),
          "a newer snapshot releases a terminal cancellation barrier");
}

}  // namespace

int main() {
  testMismatchedFailureIsIgnored();
  testMatchingFailureRequestsReplan();
  testTerminalStatusCannotRestoreOldMapVisitedTarget();
  testPathActiveRetainsPendingGoal();
  testMatchingReachedExcludesWithoutReplan();
  testMatchingCancelledClearsWithoutExcluding();
  testTerminalEventAfterClearIsIgnored();
  testOnlyExactStaticMapBoundaryFailureUsesSegmentFallback();
  testSegmentAckAndStatusRequireExactBinding();
  testUnboundExecuteRejectionIsTerminal();
  testSegmentStatusMayArriveBeforeExecuteAck();
  testSegmentTerminalOutcomeNeedsNewerSnapshotBeforeReplan();
  testSegmentBarrierWaitsForSnapshotObservedAfterTerminal();
  testSegmentCancelWaitsForTerminalStatus();
  std::cout << "test_explore_goal_lifecycle passed\n";
  return 0;
}
