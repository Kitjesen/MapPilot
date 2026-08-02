#include <cstdint>
#include <iostream>
#include <optional>
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
using lingtu::nav::endpoint::RollingSegmentIngressRejected;
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

RollingSegmentStatusEffect
confirmSafeStop(RollingSegmentLifecycle &lifecycle,
                const lingtu::nav::endpoint::RollingSegmentStepResult &result) {
  std::optional<std::size_t> stop_index;
  std::optional<std::size_t> clear_index;
  for (std::size_t index = 0U; index < result.effects.size(); ++index) {
    require(!std::holds_alternative<RollingSegmentStatusEffect>(result.effects[index]),
            "terminal status must not be present before successful motion clear feedback");
    if (std::holds_alternative<RollingSegmentStopAuthorityEffect>(result.effects[index])) {
      stop_index = index;
    }
    if (std::holds_alternative<RollingSegmentClearMotionEffect>(result.effects[index])) {
      clear_index = index;
    }
  }
  require(stop_index.has_value() && clear_index.has_value() && *stop_index < *clear_index,
          "safe-stop effects must revoke authority before clearing motion");
  const auto clear_id =
      std::get<RollingSegmentClearMotionEffect>(result.effects[*clear_index]).effect_id;
  const auto terminal = lifecycle.step(RollingSegmentEffectFeedback{clear_id, true});
  require(terminal.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentStatusEffect>(terminal.effects.front()),
          "successful motion clear must release exactly one terminal status");
  return std::get<RollingSegmentStatusEffect>(terminal.effects.front());
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

void testTerminalBarrierRejectsFreshExecuteWithoutMotionEffects() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});

  const auto result = lifecycle.step(RollingSegmentIngressRejected{
      executeCommand(),
      readyContext(),
      "goal_terminal_pending",
  });

  require(result.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(result.effects.front()),
          "terminal barrier must only emit a negative rolling ACK");
  const auto &ack = std::get<RollingSegmentAckEffect>(result.effects.front()).ack;
  require(!ack.accepted && ack.reason == "goal_terminal_pending" && ack.request_id == "segment-1" &&
              ack.session_id == "rolling-session" && ack.reset_epoch == 7U &&
              ack.generation == 2U && !ack.live,
          "terminal barrier execute ACK must retain the requested immutable binding");
  const auto state = lifecycle.snapshot();
  require(!state.active && state.has_execution_grid && !state.terminal_delivery_pending,
          "terminal barrier must not acquire authority, execute, or create terminal status");
  require(lifecycle.step(RollingSegmentBeginTick{10.1}).effects.empty(),
          "terminal barrier rejection must not schedule deferred rolling status");
}

void testTerminalBarrierExecuteReceiptSurvivesAckFailureAndExpiry() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});

  const auto rejected = lifecycle.step(RollingSegmentIngressRejected{
      executeCommand(),
      readyContext(),
      "goal_terminal_pending",
  });
  const auto &first_effect = std::get<RollingSegmentAckEffect>(rejected.effects.front());
  require(lifecycle
              .step(RollingSegmentEffectFeedback{
                  first_effect.effect_id,
                  false,
              })
              .effects.empty(),
          "failed barrier ACK feedback must not create terminal compensation");

  const auto replay =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(30.0)});
  require(replay.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(replay.effects.front()),
          "expired barrier receipt must replay only its negative ACK");
  const auto &ack = std::get<RollingSegmentAckEffect>(replay.effects.front()).ack;
  require(!ack.accepted && ack.reason == "goal_terminal_pending" &&
              ack.request_id == first_effect.ack.request_id &&
              ack.session_id == first_effect.ack.session_id &&
              ack.reset_epoch == first_effect.ack.reset_epoch &&
              ack.generation == first_effect.ack.generation && ack.live == first_effect.ack.live,
          "barrier retry must preserve the exact first negative ACK");
  require(!lifecycle.snapshot().active && !lifecycle.snapshot().terminal_delivery_pending,
          "barrier retry must not plan, activate, or create rolling terminal state");
}

void testTerminalBarrierRejectsExactActiveCancelWithPinnedBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto cancel = executeCommand();
  cancel.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
  const auto rejected = lifecycle.step(RollingSegmentIngressRejected{
      cancel,
      readyContext(),
      "goal_terminal_pending",
  });
  require(rejected.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(rejected.effects.front()),
          "terminal barrier exact cancel must only emit a negative ACK");
  const auto &first_effect = std::get<RollingSegmentAckEffect>(rejected.effects.front());
  require(!first_effect.ack.accepted && first_effect.ack.reason == "goal_terminal_pending" &&
              first_effect.ack.session_id == "rolling-session" &&
              first_effect.ack.reset_epoch == 7U && first_effect.ack.generation == 3U &&
              first_effect.ack.live,
          "terminal barrier cancel ACK must expose the real active execution binding");
  require(lifecycle.snapshot().active,
          "terminal barrier cancel must preserve the active rolling segment");

  (void)lifecycle.step(RollingSegmentEffectFeedback{first_effect.effect_id, false});
  const auto replay = lifecycle.step(RollingSegmentCommandEvent{cancel, readyContext(30.0)});
  require(replay.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(replay.effects.front()),
          "exact cancel retry must only replay its barrier receipt");
  const auto &ack = std::get<RollingSegmentAckEffect>(replay.effects.front()).ack;
  require(!ack.accepted && ack.reason == first_effect.ack.reason &&
              ack.generation == first_effect.ack.generation && ack.live == first_effect.ack.live,
          "exact cancel retry must preserve the original negative ACK");
  require(lifecycle.snapshot().active &&
              lifecycle.step(RollingSegmentBeginTick{30.1}).effects.empty(),
          "barrier-cached cancel must not terminate or schedule terminal status");
}

void testTerminalBarrierReceiptCannotMaskCancelBindingMismatch() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto exact_cancel = executeCommand();
  exact_cancel.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
  (void)lifecycle.step(RollingSegmentIngressRejected{
      exact_cancel,
      readyContext(),
      "goal_terminal_pending",
  });

  auto mismatch = exact_cancel;
  mismatch.minimum_generation = 3U;
  const auto rejected = lifecycle.step(RollingSegmentIngressRejected{
      mismatch,
      readyContext(10.1),
      "goal_terminal_pending",
  });
  require(rejected.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(rejected.effects.front()),
          "mismatched cancel must only emit its normal rejection ACK");
  const auto &ack = std::get<RollingSegmentAckEffect>(rejected.effects.front()).ack;
  require(!ack.accepted && ack.reason == "segment_cancel_binding_mismatch",
          "cancel binding mismatch must take priority over the terminal barrier receipt");
  require(lifecycle.snapshot().active,
          "mismatched cancel under the terminal barrier must preserve the active segment");
}

void testRollingTerminalReplayTakesPriorityOverGoalTerminalBarrier() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto stop = lifecycle.step(RollingSegmentMotionOutcome{
      RollingSegmentMotionOutcomeKind::kReached,
      "segment_reached_for_replay",
  });
  const auto delivered_after_stop = confirmSafeStop(lifecycle, stop);
  (void)lifecycle.step(RollingSegmentEffectFeedback{delivered_after_stop.effect_id, true});

  const auto replay = lifecycle.step(RollingSegmentIngressRejected{
      executeCommand(),
      readyContext(30.0),
      "goal_terminal_pending",
  });
  require(replay.effects.size() == 2U &&
              std::holds_alternative<RollingSegmentAckEffect>(replay.effects[0]) &&
              std::holds_alternative<RollingSegmentStatusEffect>(replay.effects[1]),
          "rolling terminal replay must retain its ACK and status effects");
  const auto &ack = std::get<RollingSegmentAckEffect>(replay.effects[0]).ack;
  const auto &status = std::get<RollingSegmentStatusEffect>(replay.effects[1]).status;
  require(!ack.accepted && ack.reason == "segment_terminal_replayed" &&
              status.state == ExplorationSegmentState::kReached &&
              status.reason == "segment_reached_for_replay",
          "real rolling terminal must take priority over the generic goal terminal barrier");
}

void testGoalTerminalBarrierPreservesRollingValidationPriority() {
  struct Case {
    RollingSegmentCommand command;
    RollingSegmentRuntimeContext context;
    std::string expected_reason;
  };

  std::vector<Case> cases;
  auto command = executeCommand();
  command.request_id.clear();
  cases.push_back({command, readyContext(), "segment_request_id_empty"});

  command = executeCommand();
  command.kind = 99;
  cases.push_back({command, readyContext(), "unknown_segment_command_kind"});

  command = executeCommand();
  command.frame_id = "odom";
  cases.push_back({command, readyContext(), "segment_request_binding_invalid"});

  command = executeCommand();
  cases.push_back({command, readyContext(30.0), "segment_request_stale_or_invalid"});

  command = executeCommand();
  command.target.qx = 0.25;
  cases.push_back({command, readyContext(), "segment_target_orientation_invalid"});

  command = executeCommand();
  command.session_id = "different-session";
  cases.push_back({command, readyContext(), "segment_request_binding_mismatch"});

  for (const auto &test_case : cases) {
    RollingSegmentLifecycle lifecycle;
    (void)lifecycle.step(RollingSegmentBeginTick{10.0});
    (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
    const auto result = lifecycle.step(RollingSegmentIngressRejected{
        test_case.command,
        test_case.context,
        "goal_terminal_pending",
    });
    require(result.effects.size() == 1U &&
                std::holds_alternative<RollingSegmentAckEffect>(result.effects.front()),
            "barrier validation rejection must only emit an ACK");
    const auto &ack = std::get<RollingSegmentAckEffect>(result.effects.front()).ack;
    require(!ack.accepted && ack.reason == test_case.expected_reason,
            "rolling validation must take priority over the goal terminal barrier");
    require(!lifecycle.snapshot().active,
            "barrier validation rejection must not activate a rolling segment");
  }
}

void testGoalTerminalBarrierEmptyReasonFailsClosedAndReplays() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});

  const auto rejected = lifecycle.step(RollingSegmentIngressRejected{
      executeCommand(),
      readyContext(),
      {},
  });
  require(rejected.effects.size() == 1U,
          "empty barrier reason must still produce one negative ACK");
  const auto &first = std::get<RollingSegmentAckEffect>(rejected.effects.front()).ack;
  require(!first.accepted && first.reason == "goal_terminal_pending",
          "empty barrier reason must use the stable fail-closed fallback");

  const auto replay =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(10.1)});
  require(replay.effects.size() == 1U,
          "fallback barrier receipt must replay without rolling admission effects");
  const auto &second = std::get<RollingSegmentAckEffect>(replay.effects.front()).ack;
  require(!second.accepted && second.reason == first.reason &&
              second.request_id == first.request_id && second.generation == first.generation,
          "fallback barrier receipt must remain exact across normal retries");
  require(!lifecycle.snapshot().active,
          "fallback barrier receipt must prevent later execution of the same binding");
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
  require(cancelled.effects.size() == 3U,
          "an exact active cancel must remain fail-safe after expiry");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(cancelled.effects[1]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(cancelled.effects[2]) &&
              !lifecycle.snapshot().active,
          "a delayed exact cancel must stop and clear before terminalizing");
  require(confirmSafeStop(lifecycle, cancelled).status.state == ExplorationSegmentState::kCancelled,
          "a delayed exact cancel must terminalize only after motion clear succeeds");
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
  require(accepted.effects.size() == 3U, "accepted cancel must ACK, stop, then clear");
  require(std::holds_alternative<RollingSegmentAckEffect>(accepted.effects[0]) &&
              std::get<RollingSegmentAckEffect>(accepted.effects[0]).ack.accepted,
          "matching cancel must ACK before changing motion state");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(accepted.effects[1]),
          "matching cancel must stop path authority");
  require(std::holds_alternative<RollingSegmentClearMotionEffect>(accepted.effects[2]),
          "matching cancel must clear motion after stopping authority");
  require(confirmSafeStop(lifecycle, accepted).status.state == ExplorationSegmentState::kCancelled,
          "matching cancel must publish its terminal only after motion clear succeeds");
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
  require(result.effects.size() == 2U,
          "invalid grid must stop and clear an active segment before terminalizing");
  require(std::holds_alternative<RollingSegmentStopAuthorityEffect>(result.effects[0]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(result.effects[1]),
          "invalid-grid fail-closed effect order changed");
  const auto status = confirmSafeStop(lifecycle, result).status;
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
  require(result.effects.size() == 2U, "unsafe forward suffix must stop and clear before NavLoop");
  const auto status = confirmSafeStop(lifecycle, result).status;
  require(status.state == ExplorationSegmentState::kFailed &&
              status.reason == "segment_path_no_longer_safe",
          "unsafe forward suffix must be a failed segment, not stale binding");
  require(!lifecycle.snapshot().active, "unsafe segment must not reach NavLoop");
}

void testNewerSameEpochRevisionRevalidatesActiveBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto observed = lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid(4U, 10.1)});
  require(observed.effects.empty() && lifecycle.snapshot().active,
          "newer same-epoch map must await corridor revalidation without invalidation");
  const auto revalidated = lifecycle.step(RollingSegmentRevalidate{readyContext(10.1)});
  require(revalidated.effects.empty() && lifecycle.snapshot().active,
          "safe corridor must remain active on a newer same-epoch revision");

  const auto replay =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(10.2)});
  const auto &ack = std::get<RollingSegmentAckEffect>(replay.effects[0]).ack;
  const auto &status = std::get<RollingSegmentStatusEffect>(replay.effects[1]).status;
  require(ack.generation == 4U && status.generation == 4U,
          "successful revalidation must advance the active execution revision");
}

void testEqualStampDifferentPayloadRevokesActiveBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto conflict = freeGrid();
  conflict.occupancy[0U] = 255U;
  const auto result = lifecycle.step(RollingSegmentObserveExecutionGrid{std::move(conflict)});
  require(result.effects.size() == 2U,
          "equal stamp with conflicting payload must stop and clear before terminalizing");
  const auto terminal = confirmSafeStop(lifecycle, result).status;
  require(terminal.state == ExplorationSegmentState::kStaleBinding &&
              terminal.reason == "execution_grid_stamp_payload_conflict" &&
              !lifecycle.snapshot().active && !lifecycle.snapshot().has_execution_grid,
          "conflicting duplicate map stamp must invalidate the cached binding");
}

void testEpochMismatchImmediatelyRevokesActiveBinding() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  auto next_epoch = freeGrid(1U, 10.1);
  next_epoch.reset_epoch = 8U;
  const auto result = lifecycle.step(RollingSegmentObserveExecutionGrid{std::move(next_epoch)});
  require(result.effects.size() == 2U,
          "map epoch mismatch must immediately stop and clear before terminalizing");
  const auto terminal = confirmSafeStop(lifecycle, result).status;
  require(terminal.state == ExplorationSegmentState::kStaleBinding &&
              terminal.reason == "segment_map_epoch_changed" && !lifecycle.snapshot().active &&
              lifecycle.snapshot().has_execution_grid,
          "new epoch must replace cached input while revoking the old active binding");
}

void testGenericNavigationPreemptionProducesReplayableCancellation() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto preempted = lifecycle.step(RollingSegmentGenericPreempt{"superseded_by_generic_goal"});
  require(preempted.effects.size() == 2U,
          "generic navigation must stop and clear the segment before terminalizing");
  const auto terminal = confirmSafeStop(lifecycle, preempted).status;
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
  require(reached.effects.size() == 2U, "reached segment must stop and clear before terminalizing");
  const auto status = confirmSafeStop(lifecycle, reached).status;
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
  const auto recovery_terminal = confirmSafeStop(recovery_lifecycle, recovery).status;
  require(recovery.effects.size() == 2U &&
              recovery_terminal.state == ExplorationSegmentState::kFailed &&
              recovery_terminal.reason == "segment_local_recovery_exhausted:oscillation" &&
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
  const auto safety_terminal = confirmSafeStop(safety_lifecycle, safety).status;
  require(safety.effects.size() == 2U &&
              safety_terminal.state == ExplorationSegmentState::kFailed &&
              safety_terminal.reason == "segment_final_safety_stopped" &&
              !safety_lifecycle.snapshot().active,
          "final-safety stop must fail and release the active segment");
}

void testShutdownCancellationRetriesTerminalStatusUntilDelivered() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto shutdown = lifecycle.step(RollingSegmentShutdown{});
  require(shutdown.effects.size() == 2U,
          "shutdown must stop and clear before publishing its terminal");
  require(std::get<RollingSegmentClearMotionEffect>(shutdown.effects[1]).failure_policy ==
              RollingSegmentEffectFailurePolicy::kAbortBatch,
          "shutdown clear failure must abort terminal delivery");
  const auto terminal = confirmSafeStop(lifecycle, shutdown);
  require(terminal.status.state == ExplorationSegmentState::kCancelled &&
              terminal.status.reason == "navd_shutdown" &&
              terminal.failure_policy == RollingSegmentEffectFailurePolicy::kRetryTerminalStatus,
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

void testClearFailureRetriesSafeStopBeforeTerminalDelivery() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  (void)lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});

  const auto terminal_intent = lifecycle.step(RollingSegmentMotionOutcome{
      RollingSegmentMotionOutcomeKind::kReached,
      "segment_reached_after_retry",
  });
  require(
      terminal_intent.effects.size() == 2U &&
          std::holds_alternative<RollingSegmentStopAuthorityEffect>(terminal_intent.effects[0]) &&
          std::holds_alternative<RollingSegmentClearMotionEffect>(terminal_intent.effects[1]),
      "terminal intent must contain only stop and clear effects");
  const auto first_clear_id =
      std::get<RollingSegmentClearMotionEffect>(terminal_intent.effects[1]).effect_id;
  require(lifecycle.step(RollingSegmentEffectFeedback{first_clear_id, false}).effects.empty(),
          "failed motion clear must not release a terminal status");

  const auto exact_retry =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext(30.0)});
  require(exact_retry.effects.size() == 3U &&
              std::holds_alternative<RollingSegmentAckEffect>(exact_retry.effects[0]) &&
              std::get<RollingSegmentAckEffect>(exact_retry.effects[0]).ack.reason ==
                  "segment_terminal_stop_pending" &&
              std::holds_alternative<RollingSegmentStopAuthorityEffect>(exact_retry.effects[1]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(exact_retry.effects[2]),
          "exact retries must report stop pending and retry safe-stop effects, not fake terminal");

  auto next = executeCommand(10.1);
  next.request_id = "segment-2";
  const auto blocked = lifecycle.step(RollingSegmentCommandEvent{next, readyContext(10.1)});
  require(blocked.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentAckEffect>(blocked.effects.front()) &&
              std::get<RollingSegmentAckEffect>(blocked.effects.front()).ack.reason ==
                  "segment_motion_clear_pending",
          "a failed clear must block new segment motion ownership");

  const auto retry = lifecycle.step(RollingSegmentBeginTick{10.2});
  require(retry.effects.size() == 2U &&
              std::holds_alternative<RollingSegmentStopAuthorityEffect>(retry.effects[0]) &&
              std::holds_alternative<RollingSegmentClearMotionEffect>(retry.effects[1]),
          "the next tick must retry authority stop and motion clear without a terminal status");
  const auto retry_clear_id = std::get<RollingSegmentClearMotionEffect>(retry.effects[1]).effect_id;
  const auto terminal = lifecycle.step(RollingSegmentEffectFeedback{retry_clear_id, true});
  require(terminal.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentStatusEffect>(terminal.effects.front()) &&
              std::get<RollingSegmentStatusEffect>(terminal.effects.front()).status.state ==
                  ExplorationSegmentState::kReached,
          "only successful retry clear may release the retained terminal outcome");

  const auto terminal_id = std::get<RollingSegmentStatusEffect>(terminal.effects.front()).effect_id;
  (void)lifecycle.step(RollingSegmentEffectFeedback{terminal_id, false});
  const auto dds_retry = lifecycle.step(RollingSegmentBeginTick{10.3});
  require(dds_retry.effects.size() == 1U &&
              std::holds_alternative<RollingSegmentStatusEffect>(dds_retry.effects.front()),
          "terminal DDS failure must retry status without repeating the successful safe stop");
}

void testAcceptedAckFailureFailsClosed() {
  RollingSegmentLifecycle lifecycle;
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  const auto admission =
      lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  const auto ack_id = std::get<RollingSegmentAckEffect>(admission.effects[3]).effect_id;

  const auto compensation = lifecycle.step(RollingSegmentEffectFeedback{ack_id, false});
  require(compensation.effects.size() == 2U,
          "failed accepted ACK must stop and clear before terminalizing");
  const auto terminal = confirmSafeStop(lifecycle, compensation).status;
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
  require(result.effects.size() == 2U, "map epoch invalidation must revoke active segment motion");
  const auto terminal = confirmSafeStop(lifecycle, result).status;
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
  require(compensation.effects.size() == 2U,
          "failed admission status must stop and clear before terminalizing");
  const auto terminal = confirmSafeStop(lifecycle, compensation).status;
  require(terminal.state == ExplorationSegmentState::kFailed &&
              terminal.reason == "segment_status_publish_failed",
          "admission status failure must become a failed terminal");
}

}  // namespace

int main() {
  testFreshExecuteFromCurrentMapProducesOrderedAdmissionEffects();
  testTerminalPrebindRejectionIsReplayableAfterRequestExpires();
  testTerminalBarrierRejectsFreshExecuteWithoutMotionEffects();
  testTerminalBarrierExecuteReceiptSurvivesAckFailureAndExpiry();
  testTerminalBarrierRejectsExactActiveCancelWithPinnedBinding();
  testTerminalBarrierReceiptCannotMaskCancelBindingMismatch();
  testRollingTerminalReplayTakesPriorityOverGoalTerminalBarrier();
  testGoalTerminalBarrierPreservesRollingValidationPriority();
  testGoalTerminalBarrierEmptyReasonFailsClosedAndReplays();
  testDuplicateActiveExecuteIsIdempotent();
  testStaleExactBindingRemainsIdempotentAndCancellable();
  testCancelRequiresExactActiveBinding();
  testInvalidExecutionGridFailsClosedForActiveSegment();
  testExecutionGridDecoderRejectsInvalidBoundaryValues();
  testUnsafeForwardSuffixFailsBeforeMotionTick();
  testNewerSameEpochRevisionRevalidatesActiveBinding();
  testEqualStampDifferentPayloadRevokesActiveBinding();
  testEpochMismatchImmediatelyRevokesActiveBinding();
  testGenericNavigationPreemptionProducesReplayableCancellation();
  testReachedMotionOutcomeTerminatesSegment();
  testFailureMotionOutcomesTerminateSegment();
  testShutdownCancellationRetriesTerminalStatusUntilDelivered();
  testClearFailureRetriesSafeStopBeforeTerminalDelivery();
  testAcceptedAckFailureFailsClosed();
  testExecutionGridProvenanceCannotRegress();
  testExternalMapInvalidationRevokesActiveBinding();
  testAuthorityActivationFailureRejectsBeforeMotion();
  testAdmissionStatusFailureFailsClosed();
  std::cout << "test_rolling_segment_lifecycle passed\n";
  return 0;
}
