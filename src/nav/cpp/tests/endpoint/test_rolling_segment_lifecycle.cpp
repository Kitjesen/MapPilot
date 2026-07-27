#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "plan/rolling_segment_lifecycle.hpp"

namespace {

using lingtu::nav::endpoint::ExplorationSegmentCommandKind;
using lingtu::nav::endpoint::ExplorationSegmentState;
using lingtu::nav::endpoint::RollingSegmentAckEffect;
using lingtu::nav::endpoint::RollingSegmentActivateAuthorityEffect;
using lingtu::nav::endpoint::RollingSegmentBeginTick;
using lingtu::nav::endpoint::RollingSegmentClearMotionEffect;
using lingtu::nav::endpoint::RollingSegmentCommand;
using lingtu::nav::endpoint::RollingSegmentCommandEvent;
using lingtu::nav::endpoint::RollingSegmentEffectFailurePolicy;
using lingtu::nav::endpoint::RollingSegmentEffectFeedback;
using lingtu::nav::endpoint::RollingSegmentExecutionGrid;
using lingtu::nav::endpoint::RollingSegmentGenericPreempt;
using lingtu::nav::endpoint::RollingSegmentInstallPathEffect;
using lingtu::nav::endpoint::RollingSegmentLifecycle;
using lingtu::nav::endpoint::RollingSegmentMotionOutcome;
using lingtu::nav::endpoint::RollingSegmentMotionOutcomeKind;
using lingtu::nav::endpoint::RollingSegmentObserveExecutionGrid;
using lingtu::nav::endpoint::RollingSegmentObserveInvalidInput;
using lingtu::nav::endpoint::RollingSegmentPublishPathEffect;
using lingtu::nav::endpoint::RollingSegmentRevalidate;
using lingtu::nav::endpoint::RollingSegmentRuntimeContext;
using lingtu::nav::endpoint::RollingSegmentShutdown;
using lingtu::nav::endpoint::RollingSegmentStatusEffect;
using lingtu::nav::endpoint::RollingSegmentStopAuthorityEffect;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

RollingSegmentExecutionGrid freeGrid(std::uint64_t generation = 3U, double stamp_s = 10.0) {
  RollingSegmentExecutionGrid grid;
  grid.stamp_s = stamp_s;
  grid.frame_id = "map";
  grid.width = 10U;
  grid.height = 3U;
  grid.resolution = 1.0;
  grid.origin_qw = 1.0;
  grid.occupancy.assign(30U, 0U);
  grid.terrain_cost.assign(30U, 0U);
  grid.session_id = "rolling-session";
  grid.reset_epoch = 7U;
  grid.generation = generation;
  grid.live = true;
  grid.terrain_risk_stamp_s = stamp_s;
  grid.terrain_risk_ready = true;
  grid.payload_complete = true;
  return grid;
}

RollingSegmentRuntimeContext readyContext(double now_s = 10.0) {
  RollingSegmentRuntimeContext context;
  context.now_s = now_s;
  context.input_ready = true;
  context.autonomy_mode = true;
  context.motion_allowed = true;
  context.driver_control_ready = true;
  context.robot_pose = {0.5, 1.5, 0.0};
  context.map_z = 0.25;
  return context;
}

RollingSegmentCommand executeCommand(double stamp_s = 10.0) {
  RollingSegmentCommand command;
  command.stamp_s = stamp_s;
  command.frame_id = "map";
  command.request_id = "segment-1";
  command.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute);
  command.session_id = "rolling-session";
  command.reset_epoch = 7U;
  command.minimum_generation = 2U;
  command.target.x = 8.5;
  command.target.y = 1.5;
  command.target.qw = 1.0;
  return command;
}

void testFreshExecuteFromCurrentMapProducesOrderedAdmissionEffects() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  const auto observed = lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  require(observed.effects.empty(), "a valid grid must not emit motion effects");

  const auto result = lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  require(result.effects.size() == 6U, "accepted execute effect count changed");
  require(std::holds_alternative<RollingSegmentActivateAuthorityEffect>(result.effects[0]),
          "path authority must be acquired before installing a segment");
  require(std::get<RollingSegmentActivateAuthorityEffect>(result.effects[0]).failure_policy ==
              RollingSegmentEffectFailurePolicy::kAbortBatch,
          "authority failure must abort admission before path installation");
  require(std::holds_alternative<RollingSegmentInstallPathEffect>(result.effects[1]),
          "segment path must be installed after authority");
  require(std::holds_alternative<RollingSegmentPublishPathEffect>(result.effects[2]),
          "path telemetry must follow path installation");
  require(std::holds_alternative<RollingSegmentAckEffect>(result.effects[3]),
          "accepted ACK must follow path activation");
  require(std::holds_alternative<RollingSegmentStatusEffect>(result.effects[4]) &&
              std::holds_alternative<RollingSegmentStatusEffect>(result.effects[5]),
          "accepted and executing statuses must close admission");

  const auto &install = std::get<RollingSegmentInstallPathEffect>(result.effects[1]);
  require(install.path.size() >= 2U, "accepted execute must contain a non-trivial safe prefix");
  require(install.map_z == 0.25, "the adapter must receive the current map height with the path");
  require(install.stamp_s == std::get<RollingSegmentPublishPathEffect>(result.effects[2]).stamp_s,
          "path echo and path telemetry must share one source timestamp");

  const auto &ack = std::get<RollingSegmentAckEffect>(result.effects[3]).ack;
  require(std::get<RollingSegmentAckEffect>(result.effects[3]).failure_policy ==
              RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment,
          "accepted ACK failure must fail the active segment closed");
  require(ack.accepted, "fresh current-map execute must be accepted");
  require(ack.generation == 3U && ack.live,
          "accepted ACK must pin the exact live execution generation");

  const auto &accepted = std::get<RollingSegmentStatusEffect>(result.effects[4]).status;
  const auto &executing = std::get<RollingSegmentStatusEffect>(result.effects[5]).status;
  require(std::get<RollingSegmentStatusEffect>(result.effects[4]).failure_policy ==
                  RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment &&
              std::get<RollingSegmentStatusEffect>(result.effects[5]).failure_policy ==
                  RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment,
          "admission status failure must fail the active segment closed");
  require(accepted.state == ExplorationSegmentState::kAccepted &&
              executing.state == ExplorationSegmentState::kExecuting,
          "accepted execute status ordering changed");
  require(lifecycle.snapshot().active, "the lifecycle must own the active segment after admission");
}

void testTerminalPrebindRejectionIsReplayableAfterRequestExpires() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  const auto first = lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  require(first.effects.size() == 1U, "missing grid must only reject admission");
  const auto &rejected = std::get<RollingSegmentAckEffect>(first.effects.front()).ack;
  require(!rejected.accepted && rejected.reason == "execution_grid_missing",
          "missing execution grid must be a terminal pre-bind rejection");

  auto delayed = executeCommand(10.0);
  auto late_context = readyContext(30.0);
  const auto replay = lifecycle.step(RollingSegmentCommandEvent{std::move(delayed), late_context});
  require(replay.effects.size() == 1U, "terminal pre-bind replay must not authorize motion");
  const auto &replayed = std::get<RollingSegmentAckEffect>(replay.effects.front()).ack;
  require(!replayed.accepted && replayed.reason == "segment_terminal_replayed" &&
              replayed.generation == 2U && !replayed.live,
          "cached terminal rejection must replay despite delayed delivery");
}

void testDuplicateActiveExecuteIsIdempotent() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  const auto admitted =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  require(admitted.effects.size() == 6U, "test setup must admit a segment");

  const auto duplicate =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(10.1)});
  require(duplicate.effects.size() == 2U,
          "idempotent execute must not plan or install a second path");
  const auto &ack = std::get<RollingSegmentAckEffect>(duplicate.effects[0]).ack;
  const auto &status = std::get<RollingSegmentStatusEffect>(duplicate.effects[1]).status;
  require(ack.accepted && ack.reason == "segment_execute_idempotent" && ack.generation == 3U,
          "duplicate active execute must replay the pinned admission");
  require(status.state == ExplorationSegmentState::kExecuting && status.generation == 3U,
          "duplicate active execute must replay executing status");
}

void testStaleExactBindingRemainsIdempotentAndCancellable() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto delayed_execute =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(30.0)});
  require(delayed_execute.effects.size() == 2U,
          "an exact active execute retry must stay idempotent after expiry");
  require(std::get<RollingSegmentAckEffect>(delayed_execute.effects[0]).ack.reason ==
              "segment_execute_idempotent",
          "a delayed exact execute must replay its pinned active admission");

  auto delayed_cancel = executeCommand();
  delayed_cancel.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
  const auto cancelled =
      lifecycle.step(RollingSegmentCommandEvent{delayed_cancel, readyContext(30.0)});
  require(cancelled.effects.size() == 4U,
          "an exact active cancel must remain fail-safe after expiry");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(cancelled.effects[1]) &&
              std::holds_alternative<RollingSegmentStatusEffect>(cancelled.effects[2]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(cancelled.effects[3]) &&
              !lifecycle.snapshot().active,
          "a delayed exact cancel must stop, terminalize, and clear motion");
}

void testCancelRequiresExactActiveBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto mismatch = executeCommand();
  mismatch.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
  mismatch.minimum_generation = 3U;
  const auto rejected = lifecycle.step(RollingSegmentCommandEvent{mismatch, readyContext()});
  require(rejected.effects.size() == 1U &&
              std::get<RollingSegmentAckEffect>(rejected.effects[0]).ack.reason ==
                  "segment_cancel_binding_mismatch",
          "cancel with a different immutable binding must be rejected");
  require(lifecycle.snapshot().active, "mismatched cancel must not release the active segment");

  auto cancel = executeCommand();
  cancel.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
  const auto accepted = lifecycle.step(RollingSegmentCommandEvent{cancel, readyContext()});
  require(accepted.effects.size() == 4U, "accepted cancel must ACK, stop, terminalize, and clear");
  require(std::holds_alternative<RollingSegmentAckEffect>(accepted.effects[0]) &&
              std::get<RollingSegmentAckEffect>(accepted.effects[0]).ack.accepted,
          "matching cancel must ACK before changing motion state");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(accepted.effects[1]),
          "matching cancel must stop path authority");
  require(std::holds_alternative<RollingSegmentStatusEffect>(accepted.effects[2]) &&
              std::get<RollingSegmentStatusEffect>(accepted.effects[2]).status.state ==
                  ExplorationSegmentState::kCancelled,
          "matching cancel must publish a cancelled terminal status");
  require(std::holds_alternative<RollingSegmentClearMotionEffect>(accepted.effects[3]),
          "matching cancel must finish with the fail-closed motion clear");
  require(!lifecycle.snapshot().active, "matching cancel must release ownership");
}

void testInvalidExecutionGridFailsClosedForActiveSegment() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto invalid = freeGrid(4U, 10.1);
  invalid.payload_complete = false;
  const auto result = lifecycle.step(RollingSegmentObserveExecutionGrid{std::move(invalid)});
  require(result.effects.size() == 3U,
          "invalid grid must stop, terminalize, and clear an active segment");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(result.effects[0]) &&
              std::holds_alternative<RollingSegmentStatusEffect>(result.effects[1]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(result.effects[2]),
          "invalid-grid fail-closed effect order changed");
  const auto &status = std::get<RollingSegmentStatusEffect>(result.effects[1]).status;
  require(status.state == ExplorationSegmentState::kStaleBinding &&
              status.reason == "execution_grid_payload_incomplete",
          "invalid grid must report a stale execution binding");
  require(!lifecycle.snapshot().active && lifecycle.snapshot().input_invalidated_this_tick &&
              lifecycle.snapshot().input_error == "execution_grid_payload_incomplete",
          "invalid grid must latch until the next tick");
}

void testExecutionGridDecoderRejectsInvalidBoundaryValues() {
  using Case = std::pair<RollingSegmentExecutionGrid, std::string>;
  std::vector<Case> cases;

  auto grid = freeGrid();
  grid.frame_id = "odom";
  cases.emplace_back(std::move(grid), "execution_grid_frame_invalid");

  grid = freeGrid();
  grid.reset_epoch = 0U;
  cases.emplace_back(std::move(grid), "execution_grid_identity_invalid");

  grid = freeGrid();
  grid.width = 0U;
  cases.emplace_back(std::move(grid), "execution_grid_dimensions_invalid");

  grid = freeGrid();
  grid.occupancy.pop_back();
  cases.emplace_back(std::move(grid), "execution_grid_payload_size_invalid");

  grid = freeGrid();
  grid.occupancy[0] = 42U;
  cases.emplace_back(std::move(grid), "execution_grid_occupancy_encoding_invalid");

  grid = freeGrid();
  grid.terrain_cost[0] = 101U;
  cases.emplace_back(std::move(grid), "execution_grid_terrain_cost_invalid");

  grid = freeGrid();
  grid.origin_z = 0.1;
  cases.emplace_back(std::move(grid), "execution_grid_origin_non_planar");

  grid = freeGrid();
  grid.terrain_risk_stamp_s = 0.0;
  cases.emplace_back(std::move(grid), "execution_grid_terrain_risk_stamp_invalid");

  for (auto &[invalid_grid, expected_reason] : cases) {
    RollingSegmentLifecycle lifecycle;
    (void)lifecycle.step(RollingSegmentBeginTick{10.0});
    const auto result = lifecycle.step(RollingSegmentObserveExecutionGrid{std::move(invalid_grid)});
    const auto state = lifecycle.snapshot();
    require(result.effects.empty() && !state.has_execution_grid &&
                state.input_invalidated_this_tick && state.input_error == expected_reason,
            "invalid execution-grid boundary value was not rejected exactly");
  }
}

void testUnsafeForwardSuffixFailsBeforeMotionTick() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto changed = freeGrid(4U, 10.1);
  changed.occupancy[12U] = 255U;
  const auto observed = lifecycle.step(RollingSegmentObserveExecutionGrid{std::move(changed)});
  require(observed.effects.empty(),
          "a newer grid is observed before the dedicated revalidation phase");

  auto context = readyContext(10.1);
  const auto result = lifecycle.step(RollingSegmentRevalidate{context});
  require(result.effects.size() == 3U,
          "unsafe forward suffix must stop, fail, and clear before NavLoop");
  const auto &status = std::get<RollingSegmentStatusEffect>(result.effects[1]).status;
  require(status.state == ExplorationSegmentState::kFailed &&
              status.reason == "segment_path_no_longer_safe",
          "unsafe forward suffix must be a failed segment, not stale binding");
  require(!lifecycle.snapshot().active, "unsafe segment must not reach NavLoop");
}

void testGenericNavigationPreemptionProducesReplayableCancellation() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto preempted = lifecycle.step(RollingSegmentGenericPreempt{"superseded_by_generic_goal"});
  require(preempted.effects.size() == 3U,
          "generic navigation must stop, terminalize, and clear the segment");
  const auto &terminal = std::get<RollingSegmentStatusEffect>(preempted.effects[1]).status;
  require(terminal.state == ExplorationSegmentState::kCancelled &&
              terminal.reason == "superseded_by_generic_goal",
          "generic goal must retain the segment cancellation reason");

  const auto replay =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(10.1)});
  require(replay.effects.size() == 2U,
          "a terminal active execution must replay ACK and terminal status");
  require(std::get<RollingSegmentAckEffect>(replay.effects[0]).ack.reason ==
              "segment_terminal_replayed",
          "terminal replay ACK reason changed");
  require(std::get<RollingSegmentStatusEffect>(replay.effects[1]).status.state ==
              ExplorationSegmentState::kCancelled,
          "terminal replay must preserve the cancellation outcome");
}

void testReachedMotionOutcomeTerminatesSegment() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto reached = lifecycle.step(RollingSegmentMotionOutcome{
      RollingSegmentMotionOutcomeKind::kReached,
      {},
  });
  require(reached.effects.size() == 3U, "reached segment must stop, terminalize, and clear");
  const auto &status = std::get<RollingSegmentStatusEffect>(reached.effects[1]).status;
  require(status.state == ExplorationSegmentState::kReached && status.reason == "segment_reached",
          "reached motion outcome must use the product terminal contract");
}

void testFailureMotionOutcomesTerminateSegment() {
  RollingSegmentLifecycle recovery_lifecycle;
  (void)recovery_lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)recovery_lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)recovery_lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto recovery = recovery_lifecycle.step(RollingSegmentMotionOutcome{
      RollingSegmentMotionOutcomeKind::kRecoveryExhausted,
      "oscillation",
  });
  require(recovery.effects.size() == 3U &&
              std::get<RollingSegmentStatusEffect>(recovery.effects[1]).status.state ==
                  ExplorationSegmentState::kFailed &&
              std::get<RollingSegmentStatusEffect>(recovery.effects[1]).status.reason ==
                  "segment_local_recovery_exhausted:oscillation" &&
              !recovery_lifecycle.snapshot().active,
          "recovery exhaustion must fail and release the active segment");

  RollingSegmentLifecycle safety_lifecycle;
  (void)safety_lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)safety_lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)safety_lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto safety = safety_lifecycle.step(RollingSegmentMotionOutcome{
      RollingSegmentMotionOutcomeKind::kFinalSafetyStopped,
      {},
  });
  require(safety.effects.size() == 3U &&
              std::get<RollingSegmentStatusEffect>(safety.effects[1]).status.state ==
                  ExplorationSegmentState::kFailed &&
              std::get<RollingSegmentStatusEffect>(safety.effects[1]).status.reason ==
                  "segment_final_safety_stopped" &&
              !safety_lifecycle.snapshot().active,
          "final-safety stop must fail and release the active segment");
}

void testShutdownCancellationRetriesTerminalStatusUntilDelivered() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto shutdown = lifecycle.step(RollingSegmentShutdown{});
  require(shutdown.effects.size() == 3U,
          "shutdown must terminalize before the endpoint final-zero barrier");
  const auto &terminal = std::get<RollingSegmentStatusEffect>(shutdown.effects[1]);
  require(terminal.status.state == ExplorationSegmentState::kCancelled &&
              terminal.status.reason == "navd_shutdown" &&
              terminal.failure_policy == RollingSegmentEffectFailurePolicy::kRetryTerminalStatus &&
              std::get<RollingSegmentClearMotionEffect>(shutdown.effects[2]).failure_policy ==
                  RollingSegmentEffectFailurePolicy::kAbortBatch,
          "shutdown must publish the product cancellation outcome");

  (void)lifecycle.step(RollingSegmentEffectFeedback{
      terminal.effect_id,
      false,
  });
  require(lifecycle.snapshot().terminal_delivery_pending,
          "failed terminal delivery must remain visible to shutdown orchestration");
  const auto retry = lifecycle.step(RollingSegmentBeginTick{10.1});
  require(retry.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentStatusEffect>(retry.effects.front()),
          "undelivered terminal status must retry at the next lifecycle boundary");
  const auto retry_id = std::get<RollingSegmentStatusEffect>(retry.effects.front()).effect_id;
  (void)lifecycle.step(RollingSegmentEffectFeedback{retry_id, true});
  require(!lifecycle.snapshot().terminal_delivery_pending,
          "successful terminal delivery must clear the observable retry state");
  require(lifecycle.step(RollingSegmentBeginTick{10.2}).effects.empty(),
          "delivered terminal status must not be published again");
}

void testAcceptedAckFailureFailsClosed() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  const auto admission =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto ack_id = std::get<RollingSegmentAckEffect>(admission.effects[3]).effect_id;

  const auto compensation = lifecycle.step(RollingSegmentEffectFeedback{ack_id, false});
  require(compensation.effects.size() == 3U,
          "failed accepted ACK must stop, terminalize, and clear");
  const auto &terminal = std::get<RollingSegmentStatusEffect>(compensation.effects[1]).status;
  require(terminal.state == ExplorationSegmentState::kFailed &&
              terminal.reason == "segment_ack_publish_failed",
          "ACK publication failure must become a correlated failed terminal");
  require(!lifecycle.snapshot().active,
          "ACK publication failure must revoke segment motion ownership");
}

void testExecutionGridProvenanceCannotRegress() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid(4U, 10.0)});

  const auto result = lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid(3U, 10.1)});
  require(result.effects.empty(), "inactive provenance regression must not emit motion effects");
  const auto state = lifecycle.snapshot();
  require(!state.has_execution_grid && state.input_invalidated_this_tick &&
              state.input_error == "execution_grid_provenance_regressed",
          "older same-source execution grid must invalidate the cached input");
}

void testExternalMapInvalidationRevokesActiveBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto result = lifecycle.step(RollingSegmentObserveInvalidInput{
      "execution_grid_epoch_reset:tf_jump",
      {},
  });
  require(result.effects.size() == 3U, "map epoch invalidation must revoke active segment motion");
  const auto &terminal = std::get<RollingSegmentStatusEffect>(result.effects[1]).status;
  require(terminal.state == ExplorationSegmentState::kStaleBinding &&
              terminal.reason == "execution_grid_epoch_reset:tf_jump",
          "map epoch invalidation must preserve its stale-binding evidence");
}

void testAuthorityActivationFailureRejectsBeforeMotion() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  const auto admission =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto activation_id =
      std::get<RollingSegmentActivateAuthorityEffect>(admission.effects.front()).effect_id;

  const auto rejected = lifecycle.step(RollingSegmentEffectFeedback{activation_id, false});
  require(rejected.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(rejected.effects.front()),
          "authority rejection must stop the admission batch before path install");
  const auto &ack = std::get<RollingSegmentAckEffect>(rejected.effects.front()).ack;
  require(!ack.accepted && ack.reason == "segment_control_authority_rejected" && !ack.live &&
              ack.generation == 2U,
          "authority rejection must remain a non-live pre-bind outcome");
  require(!lifecycle.snapshot().active,
          "authority rejection must release the provisional lifecycle binding");
}

void testAdmissionStatusFailureFailsClosed() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  const auto admission =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto accepted_status_id =
      std::get<RollingSegmentStatusEffect>(admission.effects[4]).effect_id;

  const auto compensation = lifecycle.step(RollingSegmentEffectFeedback{accepted_status_id, false});
  require(compensation.effects.size() == 3U,
          "failed admission status must stop, terminalize, and clear");
  const auto &terminal = std::get<RollingSegmentStatusEffect>(compensation.effects[1]).status;
  require(terminal.state == ExplorationSegmentState::kFailed &&
              terminal.reason == "segment_status_publish_failed",
          "admission status failure must become a failed terminal");
}

}  // namespace

int main() {
  testFreshExecuteFromCurrentMapProducesOrderedAdmissionEffects();
  testTerminalPrebindRejectionIsReplayableAfterRequestExpires();
  testDuplicateActiveExecuteIsIdempotent();
  testStaleExactBindingRemainsIdempotentAndCancellable();
  testCancelRequiresExactActiveBinding();
  testInvalidExecutionGridFailsClosedForActiveSegment();
  testExecutionGridDecoderRejectsInvalidBoundaryValues();
  testUnsafeForwardSuffixFailsBeforeMotionTick();
  testGenericNavigationPreemptionProducesReplayableCancellation();
  testReachedMotionOutcomeTerminatesSegment();
  testFailureMotionOutcomesTerminateSegment();
  testShutdownCancellationRetriesTerminalStatusUntilDelivered();
  testAcceptedAckFailureFailsClosed();
  testExecutionGridProvenanceCannotRegress();
  testExternalMapInvalidationRevokesActiveBinding();
  testAuthorityActivationFailureRejectsBeforeMotion();
  testAdmissionStatusFailureFailsClosed();
  std::cout << "test_rolling_segment_lifecycle passed\n";
  return 0;
}
