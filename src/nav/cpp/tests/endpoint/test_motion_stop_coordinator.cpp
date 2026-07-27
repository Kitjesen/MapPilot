#include <cstdint>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion/motion_stop_coordinator.hpp"

namespace {
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
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

void testEstopPersistFailureRemainsLatched() {
  Fixture fixture;
  fixture.persist_estop_ok = false;
  MotionStopCoordinator coordinator(true, fixture.actions);
  const auto result = coordinator.estop("operator_request");
  require(!result.accepted && result.reason == "estop_latch_persist_failed_estop_remains_latched",
          "estop persistence failure mismatch");
  require(fixture.estop_latched, "failed persistence must leave control latched");
  orderIs(fixture,
          {"latch_estop:operator_request", "clear_resume_required",
           "cancel_inspection:estop_latched", "persist_estop:operator_request",
           "defer_abort:estop_latched", "sync", "rolling_active", "clear_outputs:estop_latched",
           "last_sequence", "confirm_zero:17", "abort:estop_latched"});
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

void testShutdownFailsClosed() {
  Fixture publish_failed;
  publish_failed.sequenced_zero.reset();
  MotionStopCoordinator publish_failed_coordinator(true, publish_failed.actions);
  const auto publish_result = publish_failed_coordinator.finalShutdown();
  require(!publish_result.success && !publish_result.confirmation_state.has_value(),
          "shutdown publish failure must fail closed");
  orderIs(publish_failed, {"stop_control", "defer_abort:navd_shutdown", "sync", "clear_global_path",
                           "publish_sequenced_zero"});

  Fixture rejected;
  rejected.confirmation = StopConfirmationState::DriverRejected;
  MotionStopCoordinator rejected_coordinator(true, rejected.actions);
  const auto rejected_result = rejected_coordinator.finalShutdown();
  require(!rejected_result.success &&
              rejected_result.confirmation_state == StopConfirmationState::DriverRejected,
          "rejected shutdown zero must fail closed");
  require(!actionIndex(rejected, "abort:navd_shutdown").has_value(),
          "shutdown must not publish Cancelled when the driver rejects zero");

  Fixture timed_out;
  timed_out.confirmation = StopConfirmationState::TimedOut;
  MotionStopCoordinator timed_out_coordinator(true, timed_out.actions);
  require(!timed_out_coordinator.finalShutdown().success,
          "timed-out shutdown zero must fail closed");

  Fixture confirmed;
  MotionStopCoordinator confirmed_coordinator(true, confirmed.actions);
  const auto confirmed_result = confirmed_coordinator.finalShutdown();
  require(confirmed_result.success &&
              confirmed_result.confirmation_state == StopConfirmationState::Confirmed,
          "confirmed shutdown mismatch");
  orderIs(confirmed, {"stop_control", "defer_abort:navd_shutdown", "sync", "clear_global_path",
                      "publish_sequenced_zero", "confirm_zero:23", "abort:navd_shutdown"});

  Fixture disabled;
  MotionStopCoordinator disabled_coordinator(false, disabled.actions);
  const auto disabled_result = disabled_coordinator.finalShutdown();
  require(disabled_result.success && !disabled_result.confirmation_state.has_value(),
          "disabled writer shutdown mismatch");
  orderIs(disabled, {"stop_control", "defer_abort:navd_shutdown", "sync", "clear_global_path",
                     "abort:navd_shutdown"});
}

}  // namespace

int main() {
  try {
    testClearEndpointMotionBranches();
    testCancelStopAndConfirmationFailures();
    testCancelTerminalFollowsConfirmedStopEvidence();
    testGoalTerminalCommitRequiresConfirmedStopEvidence();
    testEstopPersistFailureRemainsLatched();
    testClearEstopRequiresConfirmedZero();
    testResumeOnlyUnlatchesAfterConfirmation();
    testDriverLossAndKeepZeroOrder();
    testShutdownFailsClosed();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_motion_stop_coordinator: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
