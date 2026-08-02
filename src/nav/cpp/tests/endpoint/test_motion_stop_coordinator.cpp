#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion/motion_stop_coordinator.hpp"

namespace {
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::MotionStopTerminalCommit;
using lingtu::nav::endpoint::ResumeAutonomyRequest;
using lingtu::nav::endpoint::StopConfirmationState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  std::vector<std::string> order;
  bool rolling_active{false};
  bool rolling_preempt_ok{true};
  bool clear_outputs_ok{true};
  bool suspend_outputs_ok{true};
  bool persist_estop_ok{true};
  bool clear_persisted_estop_ok{true};
  bool clear_control_estop_ok{true};
  bool resume_autonomy_ok{true};
  bool publish_zero_ok{true};
  bool stop_evidence_failure_latched{false};
  std::uint64_t last_output_sequence{17U};
  std::optional<std::uint64_t> sequenced_zero{23U};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  bool estop_latched{false};
  bool takeover_latched{true};
  MotionStopActions actions;

  Fixture() {
    actions.defer_goal_abort = [&](const std::string &reason) {
      order.emplace_back("defer_abort:" + reason);
      return [&, reason] { order.emplace_back("abort:" + reason); };
    };
    actions.record_stop_evidence_failure = [&](const std::string &reason) {
      order.emplace_back("stop_failure:" + reason);
      stop_evidence_failure_latched = true;
    };
    actions.sync_goal_diagnostics = [&] { order.emplace_back("sync"); };
    actions.rolling_segment_active = [&] {
      order.emplace_back("rolling_active");
      return rolling_active;
    };
    actions.preempt_rolling_segment = [&](const std::string &reason) {
      order.emplace_back("preempt:" + reason);
      return rolling_preempt_ok;
    };
    actions.clear_motion_outputs = [&](const std::string &reason) {
      order.emplace_back("clear_outputs:" + reason);
      return clear_outputs_ok;
    };
    actions.suspend_motion_outputs = [&](const std::string &reason) {
      order.emplace_back("suspend_outputs:" + reason);
      return suspend_outputs_ok;
    };
    actions.cancel_control = [&] { order.emplace_back("cancel_control"); };
    actions.stop_control = [&] { order.emplace_back("stop_control"); };
    actions.latch_estop = [&](const std::string &reason) {
      order.emplace_back("latch_estop:" + reason);
      estop_latched = true;
      takeover_latched = false;
    };
    actions.clear_control_estop = [&] {
      order.emplace_back("clear_control_estop");
      if (clear_control_estop_ok) {
        estop_latched = false;
      }
      return clear_control_estop_ok;
    };
    actions.resume_autonomy = [&] {
      order.emplace_back("resume_autonomy");
      if (resume_autonomy_ok) {
        takeover_latched = false;
      }
      return resume_autonomy_ok;
    };
    actions.cancel_inspection = [&](const std::string &reason) {
      order.emplace_back("cancel_inspection:" + reason);
    };
    actions.clear_operator_resume_required = [&] { order.emplace_back("clear_resume_required"); };
    actions.set_autonomy_request_not_before = [&](double stamp_s) {
      order.emplace_back("set_resume_stamp:" + std::to_string(stamp_s));
    };
    actions.persist_estop_latch = [&](const std::string &reason) {
      order.emplace_back("persist_estop:" + reason);
      return persist_estop_ok;
    };
    actions.clear_persisted_estop_latch = [&] {
      order.emplace_back("clear_persisted_estop");
      return clear_persisted_estop_ok;
    };
    actions.publish_zero = [&] {
      order.emplace_back("publish_zero");
      return publish_zero_ok;
    };
    actions.last_output_sequence = [&] {
      order.emplace_back("last_sequence");
      return last_output_sequence;
    };
    actions.publish_sequenced_zero = [&] {
      order.emplace_back("publish_sequenced_zero");
      return sequenced_zero;
    };
    actions.confirm_zero = [&](std::uint64_t sequence) {
      order.emplace_back("confirm_zero:" + std::to_string(sequence));
      return confirmation;
    };
    actions.clear_global_path = [&] { order.emplace_back("clear_global_path"); };
  }
};

void orderIs(const Fixture &fixture, std::initializer_list<const char *> expected) {
  std::vector<std::string> wanted;
  for (const char *item : expected) {
    wanted.emplace_back(item);
  }
  require(fixture.order == wanted, "action order mismatch");
}

std::optional<std::size_t> actionIndex(const Fixture &fixture, const std::string &action) {
  for (std::size_t index = 0; index < fixture.order.size(); ++index) {
    if (fixture.order[index] == action) {
      return index;
    }
  }
  return std::nullopt;
}

void testClearEndpointMotionBranches() {
  Fixture inactive;
  MotionStopCoordinator inactive_coordinator(true, inactive.actions);
  require(inactive_coordinator.clearEndpointMotion("stopped"), "inactive clear must succeed");
  orderIs(inactive, {"rolling_active", "clear_outputs:stopped"});

  Fixture active;
  active.rolling_active = true;
  MotionStopCoordinator active_coordinator(true, active.actions);
  require(active_coordinator.clearEndpointMotion("cancelled"), "rolling preempt must succeed");
  orderIs(active, {"rolling_active", "preempt:cancelled"});
}

void testTaskPausePublishesStateOnlyAfterConfirmedStop() {
  Fixture confirmed;
  MotionStopCoordinator coordinator(true, confirmed.actions);
  const auto result =
      coordinator.pauseTask([&] { confirmed.order.emplace_back("state:paused"); });
  require(result.accepted && result.reason == "pause_requested",
          "confirmed task pause result mismatch");
  orderIs(confirmed,
          {"stop_control", "suspend_outputs:task_paused", "last_sequence",
           "confirm_zero:17", "state:paused"});
  require(!actionIndex(confirmed, "clear_outputs:task_paused").has_value(),
          "task pause must preserve the global path through the suspend output path");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timeout_result =
      timed_out_coordinator.pauseTask([&] { timed_out.order.emplace_back("state:paused"); });
  require(!timeout_result.accepted &&
              timeout_result.reason == "stop_confirmation_timeout_pause_remains_stopped",
          "task pause timeout mismatch");
  require(!actionIndex(timed_out, "state:paused").has_value(),
          "task must not publish PAUSED before stop confirmation");
  require(timed_out.stop_evidence_failure_latched,
          "task pause timeout must fail closed");
  orderIs(timed_out, {"stop_control", "suspend_outputs:task_paused", "last_sequence",
                      "confirm_zero:17",
                      "stop_failure:stop_confirmation_timeout_pause_remains_stopped"});
}

void testCancelStopAndConfirmationFailures() {
  Fixture cancelled;
  MotionStopCoordinator cancel_coordinator(true, cancelled.actions);
  const auto cancel_result = cancel_coordinator.cancel();
  require(cancel_result.accepted && cancel_result.reason == "cancelled", "cancel result mismatch");
  orderIs(cancelled,
          {"cancel_control", "clear_resume_required", "cancel_inspection:navigation_cancelled",
           "defer_abort:cancelled", "sync", "rolling_active", "clear_outputs:cancelled",
           "last_sequence", "confirm_zero:17", "abort:cancelled"});

  Fixture stopped;
  MotionStopCoordinator stopped_coordinator(true, stopped.actions);
  const auto stopped_result = stopped_coordinator.stop();
  require(stopped_result.accepted && stopped_result.reason == "stopped", "stop result mismatch");
  orderIs(stopped, {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
                    "defer_abort:stopped", "sync", "rolling_active", "clear_outputs:stopped",
                    "last_sequence", "confirm_zero:17", "abort:stopped"});

  Fixture zero_failure;
  zero_failure.clear_outputs_ok = false;
  MotionStopCoordinator zero_failure_coordinator(true, zero_failure.actions);
  const auto zero_result = zero_failure_coordinator.stop();
  require(!zero_result.accepted && zero_result.reason == "zero_publish_failed",
          "zero publication failure mismatch");
  orderIs(zero_failure,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "defer_abort:stopped", "sync", "rolling_active", "clear_outputs:stopped",
           "stop_failure:zero_publish_failed"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.cancel();
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_cancel_remains_stopped",
          "driver rejection reason mismatch");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timeout_result = timed_out_coordinator.stop();
  require(!timeout_result.accepted &&
              timeout_result.reason == "stop_confirmation_timeout_stop_remains_latched",
          "stop timeout reason mismatch");
  require(timed_out.stop_evidence_failure_latched, "stop-confirmation timeout did not fail closed");
  require(!actionIndex(timed_out, "abort:stopped").has_value(),
          "stopped terminal must not publish when stop confirmation times out");

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.cancel();
  require(!unavailable_result.accepted &&
              unavailable_result.reason == "zero_publish_unavailable_cancel_remains_stopped",
          "disabled writer reason mismatch");
}

void testCancelTerminalFollowsConfirmedStopEvidence() {
  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.cancel();
  require(confirmed_result.accepted && confirmed_result.reason == "cancelled",
          "confirmed cancel result mismatch");
  require(!confirmed.stop_evidence_failure_latched,
          "confirmed cancel unexpectedly latched a stop-evidence failure");
  const auto clear_index = actionIndex(confirmed, "clear_outputs:cancelled");
  const auto confirmation_index = actionIndex(confirmed, "confirm_zero:17");
  const auto terminal_index = actionIndex(confirmed, "abort:cancelled");
  require(clear_index.has_value() && confirmation_index.has_value() && terminal_index.has_value(),
          "confirmed cancel must clear motion, confirm zero, and publish terminal state");
  require(*clear_index < *confirmation_index && *confirmation_index < *terminal_index,
          "cancelled terminal must follow confirmed stop evidence");

  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.cancel();
  require(!zero_failed_result.accepted && zero_failed_result.reason == "zero_publish_failed",
          "cancel zero-publication failure mismatch");
  require(!actionIndex(zero_failed, "abort:cancelled").has_value(),
          "cancelled terminal must not publish when zero publication fails");
  require(zero_failed.stop_evidence_failure_latched,
          "cancel zero-publication failure did not fail closed");

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.cancel();
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_cancel_remains_stopped",
          "cancel driver rejection mismatch");
  require(!actionIndex(rejected, "abort:cancelled").has_value(),
          "cancelled terminal must not publish when the driver rejects zero");
  require(rejected.stop_evidence_failure_latched, "cancel driver rejection did not fail closed");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.cancel();
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_cancel_remains_stopped",
          "cancel stop-confirmation timeout mismatch");
  require(!actionIndex(timed_out, "abort:cancelled").has_value(),
          "cancelled terminal must not publish when stop confirmation times out");
  require(timed_out.stop_evidence_failure_latched,
          "cancel stop-confirmation timeout did not fail closed");

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.cancel();
  require(!unavailable_result.accepted &&
              unavailable_result.reason == "zero_publish_unavailable_cancel_remains_stopped",
          "cancel unavailable stop evidence mismatch");
  require(!actionIndex(unavailable, "abort:cancelled").has_value(),
          "cancelled terminal must not publish without a zero-output writer");
  require(unavailable.stop_evidence_failure_latched,
          "cancel without a zero-output writer did not fail closed");
}

void testStopPreservingGoalTerminalCommitsAfterConfirmedZero() {
  Fixture fixture;
  MotionStopCoordinator coordinator(true, fixture.actions);
  const auto result = coordinator.stopPreservingGoalTerminal(
      [&] { fixture.order.emplace_back("terminal:preserved"); });

  require(result.accepted && result.reason == "stopped" && result.terminal_committed,
          "preserving stop success result mismatch");
  orderIs(fixture, {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
                    "rolling_active", "clear_outputs:stopped", "last_sequence", "confirm_zero:17",
                    "terminal:preserved"});
  require(!actionIndex(fixture, "defer_abort:stopped").has_value(),
          "preserving stop must not defer a goal abort");
  require(!actionIndex(fixture, "sync").has_value(),
          "preserving stop must not sync goal diagnostics");
}

void testStopWithoutTerminalCommitKeepsPhysicalStopSideEffectsOnly() {
  Fixture fixture;
  MotionStopCoordinator coordinator(true, fixture.actions);
  const auto result = coordinator.stopWithoutTerminalCommit();

  require(result.accepted && result.reason == "stopped", "physical-only stop result mismatch");
  orderIs(fixture, {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
                    "rolling_active", "clear_outputs:stopped", "last_sequence",
                    "confirm_zero:17"});
  require(!actionIndex(fixture, "defer_abort:stopped").has_value(),
          "physical-only stop must not defer a goal abort");
  require(!actionIndex(fixture, "abort:stopped").has_value(),
          "physical-only stop must not publish a goal terminal");
  require(!actionIndex(fixture, "sync").has_value(),
          "physical-only stop must not sync goal diagnostics");
}

void testStopWithoutTerminalCommitFailsClosedWithoutConfirmedZero() {
  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.stopWithoutTerminalCommit();
  require(!zero_failed_result.accepted && zero_failed_result.reason == "zero_publish_failed",
          "physical-only stop zero-publication failure mismatch");
  require(zero_failed.stop_evidence_failure_latched,
          "physical-only stop zero-publication failure must fail closed");
  orderIs(zero_failed,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "stop_failure:zero_publish_failed"});

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.stopWithoutTerminalCommit();
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_stop_remains_latched",
          "physical-only stop timeout mismatch");
  require(timed_out.stop_evidence_failure_latched, "physical-only stop timeout must fail closed");
  orderIs(timed_out,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "last_sequence", "confirm_zero:17",
           "stop_failure:stop_confirmation_timeout_stop_remains_latched"});
}

void testStopPreservingGoalTerminalDoesNotCommitWithoutConfirmedZero() {
  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.stopPreservingGoalTerminal(
      [&] { zero_failed.order.emplace_back("terminal:preserved"); });
  require(!zero_failed_result.accepted && zero_failed_result.reason == "zero_publish_failed" &&
              !zero_failed_result.terminal_committed,
          "preserving stop zero-publication failure mismatch");
  require(zero_failed.stop_evidence_failure_latched,
          "preserving stop zero-publication failure must fail closed");
  orderIs(zero_failed,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "stop_failure:zero_publish_failed"});

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.stopPreservingGoalTerminal(
      [&] { unavailable.order.emplace_back("terminal:preserved"); });
  require(!unavailable_result.accepted &&
              unavailable_result.reason == "zero_publish_unavailable_stop_remains_latched" &&
              !unavailable_result.terminal_committed,
          "preserving stop unavailable confirmation mismatch");
  require(unavailable.stop_evidence_failure_latched,
          "preserving stop without a writer must fail closed");
  orderIs(unavailable,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped",
           "stop_failure:zero_publish_unavailable_stop_remains_latched"});

  Fixture no_sequence;
  no_sequence.last_output_sequence = 0U;
  MotionStopCoordinator no_sequence_coordinator(true, no_sequence.actions);
  const auto no_sequence_result = no_sequence_coordinator.stopPreservingGoalTerminal(
      [&] { no_sequence.order.emplace_back("terminal:preserved"); });
  require(!no_sequence_result.accepted &&
              no_sequence_result.reason == "zero_publish_unavailable_stop_remains_latched" &&
              !no_sequence_result.terminal_committed,
          "preserving stop missing sequence mismatch");
  require(no_sequence.stop_evidence_failure_latched,
          "preserving stop without an output sequence must fail closed");
  orderIs(no_sequence,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "last_sequence",
           "stop_failure:zero_publish_unavailable_stop_remains_latched"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.stopPreservingGoalTerminal(
      [&] { rejected.order.emplace_back("terminal:preserved"); });
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_stop_remains_latched" &&
              !rejected_result.terminal_committed,
          "preserving stop driver rejection mismatch");
  require(rejected.stop_evidence_failure_latched,
          "preserving stop driver rejection must fail closed");
  orderIs(rejected,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "last_sequence", "confirm_zero:17",
           "stop_failure:driver_rejected_zero_stop_remains_latched"});

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.stopPreservingGoalTerminal(
      [&] { timed_out.order.emplace_back("terminal:preserved"); });
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_stop_remains_latched" &&
              !timed_out_result.terminal_committed,
          "preserving stop timeout mismatch");
  require(timed_out.stop_evidence_failure_latched, "preserving stop timeout must fail closed");
  orderIs(timed_out,
          {"stop_control", "clear_resume_required", "cancel_inspection:navigation_stopped",
           "rolling_active", "clear_outputs:stopped", "last_sequence", "confirm_zero:17",
           "stop_failure:stop_confirmation_timeout_stop_remains_latched"});
}

void testEstopPreservingGoalTerminalCommitsAfterConfirmedZero() {
  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { confirmed.order.emplace_back("terminal:preserved"); });
  require(confirmed_result.accepted && confirmed_result.reason == "estop_latched" &&
              confirmed_result.terminal_committed,
          "preserving estop success result mismatch");
  require(confirmed.estop_latched, "preserving estop must latch control");
  orderIs(confirmed, {"latch_estop:operator_request", "persist_estop:operator_request",
                      "cancel_control", "clear_resume_required",
                      "cancel_inspection:estop_latched", "rolling_active",
                      "clear_outputs:estop_latched", "last_sequence", "confirm_zero:17",
                      "terminal:preserved"});
  require(!actionIndex(confirmed, "defer_abort:estop_latched").has_value(),
          "preserving estop must not defer a goal abort");
  require(!actionIndex(confirmed, "sync").has_value(),
          "preserving estop must not sync goal diagnostics");

}

void testEstopWithoutTerminalCommitKeepsPhysicalEstopSideEffectsOnly() {
  Fixture fixture;
  MotionStopCoordinator coordinator(true, fixture.actions);

  const auto result = coordinator.estopWithoutTerminalCommit("operator_request");

  require(result.accepted && result.reason == "estop_latched",
          "physical-only estop did not acknowledge the latched estop fact");
  require(fixture.estop_latched, "physical-only estop must latch control");
  orderIs(fixture, {"latch_estop:operator_request", "persist_estop:operator_request",
                    "cancel_control", "clear_resume_required",
                    "cancel_inspection:estop_latched", "rolling_active",
                    "clear_outputs:estop_latched", "last_sequence", "confirm_zero:17"});
  require(!actionIndex(fixture, "defer_abort:estop_latched").has_value(),
          "physical-only estop must not defer a goal terminal");
  require(!actionIndex(fixture, "abort:estop_latched").has_value(),
          "physical-only estop must not commit a goal terminal");
  require(!actionIndex(fixture, "sync").has_value(),
          "physical-only estop must not sync goal diagnostics");
}

void testEstopPersistFailureStillZerosButDoesNotCommitTerminal() {
  Fixture persist_failed;
  persist_failed.persist_estop_ok = false;
  MotionStopCoordinator persist_failed_coordinator(true, persist_failed.actions);
  const auto persist_failed_result =
      persist_failed_coordinator.estopPreservingGoalTerminal(
          "operator_request",
          [&] { persist_failed.order.emplace_back("terminal:preserved"); });

  require(!persist_failed_result.accepted &&
              persist_failed_result.reason ==
                  "estop_latch_persist_failed_estop_remains_latched" &&
              !persist_failed_result.terminal_committed,
          "persist-failed estop must remain retryable without committing terminal");
  require(persist_failed.estop_latched, "persist-failed estop must leave control latched");
  orderIs(persist_failed, {"latch_estop:operator_request",
                           "persist_estop:operator_request", "cancel_control",
                           "clear_resume_required", "cancel_inspection:estop_latched",
                           "rolling_active",
                           "clear_outputs:estop_latched", "last_sequence",
                           "confirm_zero:17"});

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.estopWithoutTerminalCommit(
      "operator_request");
  require(!unavailable_result.accepted &&
              unavailable_result.reason ==
                  "zero_publish_unavailable_estop_remains_latched",
          "physical-only estop without a zero writer must fail closed");
  require(unavailable.estop_latched,
          "physical-only estop without a zero writer must preserve the estop latch");
  orderIs(unavailable, {"latch_estop:operator_request",
                        "persist_estop:operator_request", "cancel_control",
                        "clear_resume_required", "cancel_inspection:estop_latched",
                        "rolling_active",
                        "clear_outputs:estop_latched"});
}

void testEstopPreservingGoalTerminalDoesNotCommitWithoutConfirmedZero() {
  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { zero_failed.order.emplace_back("terminal:preserved"); });
  require(!zero_failed_result.accepted &&
              zero_failed_result.reason == "zero_publish_failed_estop_remains_latched" &&
              !zero_failed_result.terminal_committed,
          "preserving estop zero-publication failure mismatch");
  orderIs(zero_failed, {"latch_estop:operator_request", "persist_estop:operator_request",
                        "cancel_control", "clear_resume_required",
                        "cancel_inspection:estop_latched", "rolling_active",
                        "clear_outputs:estop_latched"});

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { unavailable.order.emplace_back("terminal:preserved"); });
  require(!unavailable_result.accepted &&
              unavailable_result.reason == "zero_publish_unavailable_estop_remains_latched" &&
              !unavailable_result.terminal_committed,
          "preserving estop unavailable confirmation mismatch");
  require(unavailable.estop_latched,
          "preserving estop unavailable confirmation must preserve the estop latch");
  orderIs(unavailable, {"latch_estop:operator_request", "persist_estop:operator_request",
                        "cancel_control", "clear_resume_required",
                        "cancel_inspection:estop_latched", "rolling_active",
                        "clear_outputs:estop_latched"});

  Fixture no_sequence;
  no_sequence.last_output_sequence = 0U;
  MotionStopCoordinator no_sequence_coordinator(true, no_sequence.actions);
  const auto no_sequence_result = no_sequence_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { no_sequence.order.emplace_back("terminal:preserved"); });
  require(!no_sequence_result.accepted &&
              no_sequence_result.reason == "zero_publish_unavailable_estop_remains_latched" &&
              !no_sequence_result.terminal_committed,
          "preserving estop missing sequence mismatch");
  orderIs(no_sequence, {"latch_estop:operator_request", "persist_estop:operator_request",
                        "cancel_control", "clear_resume_required",
                        "cancel_inspection:estop_latched", "rolling_active",
                        "clear_outputs:estop_latched", "last_sequence"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { rejected.order.emplace_back("terminal:preserved"); });
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_estop_remains_latched" &&
              !rejected_result.terminal_committed,
          "preserving estop driver rejection mismatch");
  orderIs(rejected,
          {"latch_estop:operator_request", "persist_estop:operator_request", "cancel_control",
           "clear_resume_required", "cancel_inspection:estop_latched", "rolling_active",
           "clear_outputs:estop_latched", "last_sequence", "confirm_zero:17"});

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.estopPreservingGoalTerminal(
      "operator_request", [&] { timed_out.order.emplace_back("terminal:preserved"); });
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_estop_remains_latched" &&
              !timed_out_result.terminal_committed,
          "preserving estop timeout mismatch");
  require(timed_out.estop_latched, "preserving estop timeout must preserve the estop latch");
  orderIs(timed_out,
          {"latch_estop:operator_request", "persist_estop:operator_request", "cancel_control",
           "clear_resume_required", "cancel_inspection:estop_latched", "rolling_active",
           "clear_outputs:estop_latched", "last_sequence", "confirm_zero:17"});
}

void testPreservingStopReusesCopySafeTerminalCommitExactlyOnce() {
  Fixture fixture;
  MotionStopCoordinator coordinator(true, fixture.actions);
  const auto terminal_published = std::make_shared<bool>(false);
  const auto publish_count = std::make_shared<std::size_t>(0U);
  MotionStopTerminalCommit commit_terminal = [terminal_published, publish_count] {
    if (*terminal_published) {
      return;
    }
    *terminal_published = true;
    ++*publish_count;
  };
  MotionStopTerminalCommit copied_commit = commit_terminal;

  const auto first_result = coordinator.stopPreservingGoalTerminal(copied_commit);
  const auto second_result = coordinator.stopPreservingGoalTerminal(commit_terminal);

  require(first_result.accepted && first_result.terminal_committed && second_result.accepted &&
              second_result.terminal_committed,
          "copy-safe terminal commits must cross each confirmed stop barrier");
  require(*publish_count == 1U,
          "copies of an exact terminal commit must publish terminal state only once");
  require(!actionIndex(fixture, "defer_abort:stopped").has_value(),
          "repeated preserving stop must not defer a goal abort");
  require(!actionIndex(fixture, "sync").has_value(),
          "repeated preserving stop must not sync goal diagnostics");
}

void testGoalTerminalCommitRequiresConfirmedStopEvidence() {
  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.commitGoalTerminalAfterStop(
      "goal_reached", [&] { confirmed.order.emplace_back("terminal:reached"); });
  require(confirmed_result.accepted && confirmed_result.reason == "goal_reached",
          "confirmed goal-terminal result mismatch");
  require(!confirmed.stop_evidence_failure_latched,
          "confirmed goal terminal unexpectedly latched a stop-evidence failure");
  orderIs(confirmed, {"stop_control", "rolling_active", "clear_outputs:goal_reached",
                      "last_sequence", "confirm_zero:17", "terminal:reached"});

  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.commitGoalTerminalAfterStop(
      "goal_failed", [&] { zero_failed.order.emplace_back("terminal:failed"); });
  require(!zero_failed_result.accepted &&
              zero_failed_result.reason == "zero_publish_failed_goal_terminal_pending",
          "goal-terminal zero-publication failure mismatch");
  require(!actionIndex(zero_failed, "terminal:failed").has_value(),
          "goal terminal must not publish when zero publication fails");
  require(zero_failed.stop_evidence_failure_latched,
          "goal-terminal zero-publication failure did not fail closed");

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.commitGoalTerminalAfterStop(
      "goal_failed", [&] { rejected.order.emplace_back("terminal:failed"); });
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_goal_terminal_pending",
          "goal-terminal driver rejection mismatch");
  require(!actionIndex(rejected, "terminal:failed").has_value(),
          "goal terminal must not publish when the driver rejects zero");
  require(rejected.stop_evidence_failure_latched,
          "goal-terminal driver rejection did not fail closed");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.commitGoalTerminalAfterStop(
      "goal_failed", [&] { timed_out.order.emplace_back("terminal:failed"); });
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_goal_terminal_pending",
          "goal-terminal stop-confirmation timeout mismatch");
  require(!actionIndex(timed_out, "terminal:failed").has_value(),
          "goal terminal must not publish when stop confirmation times out");
  require(timed_out.stop_evidence_failure_latched,
          "goal-terminal stop-confirmation timeout did not fail closed");

  Fixture unavailable;
  MotionStopCoordinator unavailable_coordinator(false, unavailable.actions);
  const auto unavailable_result = unavailable_coordinator.commitGoalTerminalAfterStop(
      "goal_failed", [&] { unavailable.order.emplace_back("terminal:failed"); });
  require(!unavailable_result.accepted &&
              unavailable_result.reason == "zero_publish_unavailable_goal_terminal_pending",
          "goal-terminal unavailable stop evidence mismatch");
  require(!actionIndex(unavailable, "terminal:failed").has_value(),
          "goal terminal must not publish without a zero-output writer");
  require(unavailable.stop_evidence_failure_latched,
          "goal terminal without a zero-output writer did not fail closed");
}

void testGoalReplanStopConfirmsWithoutTerminalSideEffects() {
  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.confirmGoalReplanStop("goal_replan");
  require(confirmed_result.accepted && confirmed_result.reason == "replan_stop_confirmed",
          "confirmed goal-replan stop result mismatch");
  require(!confirmed.stop_evidence_failure_latched,
          "confirmed goal-replan stop unexpectedly latched a stop-evidence failure");
  orderIs(confirmed, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                      "last_sequence", "confirm_zero:17"});
  require(!actionIndex(confirmed, "defer_abort:goal_replan").has_value(),
          "goal replan stop must not defer a goal abort");
  require(!actionIndex(confirmed, "sync").has_value(),
          "goal replan stop must not sync terminal diagnostics");

  Fixture zero_failed;
  zero_failed.clear_outputs_ok = false;
  MotionStopCoordinator zero_failed_coordinator(true, zero_failed.actions);
  const auto zero_failed_result = zero_failed_coordinator.confirmGoalReplanStop("goal_replan");
  require(!zero_failed_result.accepted &&
              zero_failed_result.reason == "zero_publish_failed_goal_replan_pending",
          "goal-replan zero-publication failure mismatch");
  require(zero_failed.stop_evidence_failure_latched,
          "goal-replan zero-publication failure did not fail closed");
  orderIs(zero_failed, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                        "stop_failure:zero_publish_failed_goal_replan_pending"});

  Fixture disabled;
  MotionStopCoordinator disabled_coordinator(false, disabled.actions);
  const auto disabled_result = disabled_coordinator.confirmGoalReplanStop("goal_replan");
  require(!disabled_result.accepted &&
              disabled_result.reason == "zero_publish_unavailable_goal_replan_pending",
          "goal-replan disabled writer reason mismatch");
  require(disabled.stop_evidence_failure_latched,
          "goal replan without a zero-output writer did not fail closed");
  orderIs(disabled, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                     "stop_failure:zero_publish_unavailable_goal_replan_pending"});

  Fixture no_sequence;
  no_sequence.last_output_sequence = 0U;
  MotionStopCoordinator no_sequence_coordinator(true, no_sequence.actions);
  const auto no_sequence_result = no_sequence_coordinator.confirmGoalReplanStop("goal_replan");
  require(!no_sequence_result.accepted &&
              no_sequence_result.reason == "zero_publish_unavailable_goal_replan_pending",
          "goal-replan missing sequence reason mismatch");
  require(no_sequence.stop_evidence_failure_latched,
          "goal replan without an output sequence did not fail closed");
  orderIs(no_sequence, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                        "last_sequence",
                        "stop_failure:zero_publish_unavailable_goal_replan_pending"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.confirmGoalReplanStop("goal_replan");
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_goal_replan_pending",
          "goal-replan driver rejection mismatch");
  require(rejected.stop_evidence_failure_latched,
          "goal-replan driver rejection did not fail closed");
  orderIs(rejected, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                     "last_sequence", "confirm_zero:17",
                     "stop_failure:driver_rejected_zero_goal_replan_pending"});

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.confirmGoalReplanStop("goal_replan");
  require(!timed_out_result.accepted &&
              timed_out_result.reason == "stop_confirmation_timeout_goal_replan_pending",
          "goal-replan stop-confirmation timeout mismatch");
  require(timed_out.stop_evidence_failure_latched,
          "goal-replan stop-confirmation timeout did not fail closed");
  orderIs(timed_out, {"stop_control", "rolling_active", "clear_outputs:goal_replan",
                      "last_sequence", "confirm_zero:17",
                      "stop_failure:stop_confirmation_timeout_goal_replan_pending"});
}

void testEstopPersistFailureRemainsLatched() {
  Fixture fixture;
  fixture.persist_estop_ok = false;
  MotionStopCoordinator coordinator(true, fixture.actions);
  const auto result = coordinator.estop("operator_request");
  require(!result.accepted && result.reason == "estop_latch_persist_failed_estop_remains_latched",
          "estop persistence failure mismatch");
  require(fixture.estop_latched, "failed persistence must leave control latched");
  require(!actionIndex(fixture, "abort:estop_latched").has_value(),
          "persist-failed estop must not commit a terminal");
  orderIs(fixture,
          {"latch_estop:operator_request", "persist_estop:operator_request", "cancel_control",
           "clear_resume_required", "cancel_inspection:estop_latched", "rolling_active",
           "clear_outputs:estop_latched", "last_sequence", "confirm_zero:17"});
}

void testClearEstopRequiresConfirmedZero() {
  Fixture invalid;
  MotionStopCoordinator invalid_coordinator(true, invalid.actions);
  const auto invalid_result = invalid_coordinator.clearEstop("clear_estop_source_stamp_stale");
  require(!invalid_result.accepted && invalid_result.reason == "clear_estop_source_stamp_stale",
          "clear-estop precondition mismatch");
  orderIs(invalid, {});

  Fixture timed_out;
  timed_out.estop_latched = true;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timeout_result = timed_out_coordinator.clearEstop({});
  require(!timeout_result.accepted &&
              timeout_result.reason == "stop_confirmation_timeout_estop_remains_latched",
          "clear-estop timeout mismatch");
  require(timed_out.estop_latched, "unconfirmed zero must preserve control latch");
  orderIs(timed_out, {"defer_abort:estop_cleared", "sync", "rolling_active",
                      "clear_outputs:estop_cleared", "last_sequence", "confirm_zero:17"});

  Fixture persist_failure;
  persist_failure.estop_latched = true;
  persist_failure.clear_persisted_estop_ok = false;
  MotionStopCoordinator persist_failure_coordinator(true, persist_failure.actions);
  const auto persist_result = persist_failure_coordinator.clearEstop({});
  require(!persist_result.accepted &&
              persist_result.reason == "estop_latch_clear_failed_estop_remains_latched",
          "persistent clear failure mismatch");
  require(persist_failure.estop_latched, "persistent clear failure must preserve control latch");

  Fixture confirmed;
  confirmed.estop_latched = true;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.clearEstop({});
  require(confirmed_result.accepted && confirmed_result.reason == "estop_cleared",
          "clear-estop success mismatch");
  require(!confirmed.estop_latched, "confirmed zero must allow unlatching");
  orderIs(confirmed,
          {"defer_abort:estop_cleared", "sync", "rolling_active", "clear_outputs:estop_cleared",
           "last_sequence", "confirm_zero:17", "abort:estop_cleared", "clear_persisted_estop",
           "clear_control_estop", "clear_resume_required"});
}

void testResumeOnlyUnlatchesAfterConfirmation() {
  const ResumeAutonomyRequest request{{}, true, 42.5};

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.resumeAutonomy(request);
  require(!rejected_result.accepted &&
              rejected_result.reason == "driver_rejected_zero_takeover_remains_latched",
          "resume rejection mismatch");
  require(rejected.takeover_latched, "unconfirmed resume must preserve takeover latch");
  orderIs(rejected, {"defer_abort:autonomy_resume_ready", "sync", "rolling_active",
                     "clear_outputs:autonomy_resume_ready", "last_sequence", "confirm_zero:17"});

  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.resumeAutonomy(request);
  require(confirmed_result.accepted &&
              confirmed_result.reason == "autonomy_resume_ready_reissue_goal",
          "resume success mismatch");
  require(!confirmed.takeover_latched, "confirmed resume must unlatch takeover");
  orderIs(confirmed, {"defer_abort:autonomy_resume_ready", "sync", "rolling_active",
                      "clear_outputs:autonomy_resume_ready", "last_sequence", "confirm_zero:17",
                      "abort:autonomy_resume_ready", "resume_autonomy",
                      "set_resume_stamp:42.500000", "clear_resume_required"});

  Fixture already_ready;
  MotionStopCoordinator already_ready_coordinator(true, already_ready.actions);
  const auto ready_result = already_ready_coordinator.resumeAutonomy({{}, false, 1.0});
  require(ready_result.accepted && ready_result.reason == "autonomy_already_ready",
          "already-ready result mismatch");
  orderIs(already_ready, {});
}

void testDriverLossAndKeepZeroOrder() {
  Fixture fixture;
  MotionStopCoordinator coordinator(true, fixture.actions);
  require(coordinator.driverAuthorityLost("lease_not_owned"), "driver-loss clear must succeed");
  orderIs(fixture, {"cancel_control", "clear_resume_required",
                    "cancel_inspection:driver_control_lost:lease_not_owned", "rolling_active",
                    "clear_outputs:lease_not_owned"});
  fixture.order.clear();
  require(coordinator.keepZeroFresh(), "fresh zero must report publication result");
  orderIs(fixture, {"publish_zero"});
}

void testShutdownPhysicalOnlyAndTerminalPreservingPipelines() {
  Fixture physical_only;
  physical_only.estop_latched = true;
  MotionStopCoordinator physical_only_coordinator(true, physical_only.actions);
  const auto physical_only_result =
      physical_only_coordinator.finalShutdownWithoutTerminalCommit();
  require(physical_only_result.success &&
              physical_only_result.confirmation_state == StopConfirmationState::Confirmed,
          "physical-only shutdown did not confirm the final zero");
  require(physical_only.estop_latched,
          "physical-only shutdown cleared an already-latched estop");
  orderIs(physical_only, {"stop_control", "cancel_inspection:navd_shutdown", "rolling_active",
                          "clear_outputs:navd_shutdown", "publish_sequenced_zero",
                          "confirm_zero:23"});

  Fixture preserving;
  int terminal_commits = 0;
  MotionStopCoordinator preserving_coordinator(true, preserving.actions);
  const auto preserving_result = preserving_coordinator.finalShutdownPreservingGoalTerminal([&] {
    ++terminal_commits;
    preserving.order.emplace_back("terminal:navd_shutdown");
  });
  require(preserving_result.success && terminal_commits == 1,
          "confirmed shutdown did not commit the supplied terminal exactly once");
  orderIs(preserving, {"stop_control", "cancel_inspection:navd_shutdown", "rolling_active",
                       "clear_outputs:navd_shutdown", "publish_sequenced_zero",
                       "confirm_zero:23", "terminal:navd_shutdown"});
}

void testShutdownZeroFailuresNeverCommitTerminal() {
  Fixture publish_failed;
  publish_failed.sequenced_zero.reset();
  int publish_failed_commits = 0;
  MotionStopCoordinator publish_failed_coordinator(true, publish_failed.actions);
  const auto publish_result = publish_failed_coordinator.finalShutdownPreservingGoalTerminal(
      [&] { ++publish_failed_commits; });
  require(!publish_result.success && !publish_result.confirmation_state.has_value() &&
              publish_failed_commits == 0,
          "shutdown publish failure committed the terminal or reported completion");
  orderIs(publish_failed, {"stop_control", "cancel_inspection:navd_shutdown", "rolling_active",
                           "clear_outputs:navd_shutdown", "publish_sequenced_zero"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  int rejected_commits = 0;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.finalShutdownPreservingGoalTerminal(
      [&] { ++rejected_commits; });
  require(!rejected_result.success &&
              rejected_result.confirmation_state == StopConfirmationState::DriverRejected &&
              rejected_commits == 0,
          "driver-rejected shutdown zero committed the terminal");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  int timed_out_commits = 0;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  const auto timed_out_result = timed_out_coordinator.finalShutdownPreservingGoalTerminal(
      [&] { ++timed_out_commits; });
  require(!timed_out_result.success &&
              timed_out_result.confirmation_state == StopConfirmationState::TimedOut &&
              timed_out_commits == 0,
          "timed-out shutdown zero committed the terminal");
}

}  // namespace

int main() {
  try {
    testClearEndpointMotionBranches();
    testTaskPausePublishesStateOnlyAfterConfirmedStop();
    testCancelStopAndConfirmationFailures();
    testCancelTerminalFollowsConfirmedStopEvidence();
    testStopPreservingGoalTerminalCommitsAfterConfirmedZero();
    testStopWithoutTerminalCommitKeepsPhysicalStopSideEffectsOnly();
    testStopWithoutTerminalCommitFailsClosedWithoutConfirmedZero();
    testStopPreservingGoalTerminalDoesNotCommitWithoutConfirmedZero();
    testEstopWithoutTerminalCommitKeepsPhysicalEstopSideEffectsOnly();
    testEstopPersistFailureStillZerosButDoesNotCommitTerminal();
    testEstopPreservingGoalTerminalCommitsAfterConfirmedZero();
    testEstopPreservingGoalTerminalDoesNotCommitWithoutConfirmedZero();
    testPreservingStopReusesCopySafeTerminalCommitExactlyOnce();
    testGoalTerminalCommitRequiresConfirmedStopEvidence();
    testGoalReplanStopConfirmsWithoutTerminalSideEffects();
    testEstopPersistFailureRemainsLatched();
    testClearEstopRequiresConfirmedZero();
    testResumeOnlyUnlatchesAfterConfirmation();
    testDriverLossAndKeepZeroOrder();
    testShutdownPhysicalOnlyAndTerminalPreservingPipelines();
    testShutdownZeroFailuresNeverCommitTerminal();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_motion_stop_coordinator: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
