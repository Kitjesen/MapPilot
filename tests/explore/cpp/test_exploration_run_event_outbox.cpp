#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "endpoint/run_event_outbox.hpp"
#include "endpoint/run_lifecycle.hpp"
#include "endpoint/status_identity.hpp"

namespace {

using lingtu::message::ExplorationRunEventKind;
using lingtu::message::ExplorationRunState;
using lingtu::nav::endpoint::ExplorationRunBinding;
using lingtu::nav::endpoint::ExplorationRunEventEnvelope;
using lingtu::nav::endpoint::ExplorationRunEventOutbox;
using lingtu::nav::endpoint::ExplorationRunEventOutboxRecordResult;
using lingtu::nav::endpoint::ExplorationRunEventRecord;
using lingtu::nav::endpoint::ExplorationRunLifecycle;

constexpr const char *kRun = "01ARZ3NDEKTSV4RRFFQ69G5FAV";

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ExplorationRunBinding binding() {
  return {
      kRun,
      "start-request",
      "product-session",
      "map",
      "map-a",
      7,
  };
}

ExplorationRunEventRecord runningEvent() {
  return {
      10.0,
      "map",
      ExplorationRunEventKind::kStateChanged,
      kRun,
      "start-request",
      "start-request",
      "product-session",
      ExplorationRunState::kRunning,
      "map",
      "map-a",
      7,
      "exploration_running",
      false,
      {},
  };
}

void testOutboxRetriesOldestWithoutReordering() {
  std::vector<ExplorationRunEventEnvelope> delivered;
  bool writer_available = false;
  ExplorationRunEventOutbox outbox(
      "host-boot:42:100", [&](const ExplorationRunEventEnvelope &event) {
        if (!writer_available) {
          return false;
        }
        delivered.push_back(event);
        return true;
      });

  require(outbox.bootId() == "host-boot:42:100",
          "status and event publication must share the outbox boot identity");
  std::ostringstream status_identity;
  lingtu::nav::endpoint::writeExploreStatusIdentity(status_identity, outbox.bootId());
  require(status_identity.str().find("\"boot_id\": \"host-boot:42:100\"") !=
              std::string::npos,
          "status JSON must expose the exact ExplorationRunEvent boot_id");

  auto running = runningEvent();
  require(outbox.record(running) == ExplorationRunEventOutboxRecordResult::kAccepted,
          "valid event must enter outbox");
  auto pausing = running;
  pausing.kind = ExplorationRunEventKind::kStateChanged;
  pausing.command_request_id = "pause-request";
  pausing.state = ExplorationRunState::kPausing;
  pausing.reason = "operator_pause";
  require(outbox.record(pausing) == ExplorationRunEventOutboxRecordResult::kAccepted,
          "second valid event must enter outbox");

  require(outbox.flush() == 0U, "failed oldest write must stop the flush");
  require(outbox.diagnostics().pending == 2U, "failed write must retain both facts");
  writer_available = true;
  require(outbox.flush() == 2U, "retry must deliver all pending facts");
  require(delivered.size() == 2U && delivered[0].event_sequence == 1U &&
              delivered[1].event_sequence == 2U,
          "outbox must stamp one boot-local monotonic sequence");
  require(delivered[0].boot_id == "host-boot:42:100" &&
              delivered[1].event.command_request_id == "pause-request",
          "outbox must preserve identity and FIFO order");
}

void testOutboxRejectsFalseTerminalAndBackpressure() {
  ExplorationRunEventOutbox outbox("boot", [](const auto &) { return true; }, 1U);
  auto invalid = runningEvent();
  invalid.state = ExplorationRunState::kCompleted;
  invalid.reason = "coverage_complete";
  require(outbox.record(invalid) == ExplorationRunEventOutboxRecordResult::kInvalid,
          "terminal event without parking proof must be rejected");

  auto valid = runningEvent();
  require(outbox.record(valid) == ExplorationRunEventOutboxRecordResult::kAccepted,
          "capacity setup event");
  require(!outbox.canRecord(1U), "full outbox must advertise backpressure");
  require(outbox.record(valid) == ExplorationRunEventOutboxRecordResult::kBackpressure,
          "full outbox must never drop an accepted fact");
}

void testRunFactsRequireCanonicalRouteMapIdentity() {
  ExplorationRunEventOutbox outbox("boot", [](const auto &) { return true; });

  auto live_event = runningEvent();
  live_event.route = "live";
  live_event.map_id.clear();
  live_event.map_content_epoch = 0;
  require(outbox.record(live_event) == ExplorationRunEventOutboxRecordResult::kAccepted,
          "canonical live event must be accepted");

  live_event.map_id = "map-a";
  require(outbox.record(live_event) == ExplorationRunEventOutboxRecordResult::kInvalid,
          "live event must not bind a map_id");
  live_event.map_id.clear();
  live_event.map_content_epoch = 1;
  require(outbox.record(live_event) == ExplorationRunEventOutboxRecordResult::kInvalid,
          "live event must use epoch zero");

  auto live_binding = binding();
  live_binding.route = "live";
  live_binding.map_id.clear();
  live_binding.map_content_epoch = 0;
  require(live_binding.valid(), "canonical live lifecycle binding must be valid");
  live_binding.map_id = "map-a";
  require(!live_binding.valid(), "live lifecycle binding must not bind a map_id");
  live_binding.map_id.clear();
  live_binding.map_content_epoch = 1;
  require(!live_binding.valid(), "live lifecycle binding must use epoch zero");
}

void testLifecycleWaitsForPostStopTerminal() {
  std::vector<ExplorationRunEventEnvelope> delivered;
  ExplorationRunEventOutbox outbox(
      "boot", [&](const ExplorationRunEventEnvelope &event) {
        delivered.push_back(event);
        return true;
      });
  ExplorationRunLifecycle lifecycle(outbox);

  require(lifecycle.start(binding(), 20.0, "exploration_start_admitted"),
          "start lifecycle transition");
  require(lifecycle.pause("pause-request", 21.0, "operator_pause", true),
          "pause admission with active motion");
  require(lifecycle.state() == ExplorationRunState::kPausing &&
              lifecycle.stopConfirmationPending(),
          "active motion pause must remain nonterminal");
  require(outbox.flush() == 3U, "admitted/running/pausing facts must publish");
  require(delivered.back().event.state == ExplorationRunState::kPausing &&
              !delivered.back().event.motion_stop_confirmed,
          "pausing fact must not claim parking");

  require(lifecycle.recordStopConfirmationFailure(21.5, "cancel_rejected:authority_busy"),
          "a rejected stop attempt must be recorded without closing the run");
  require(lifecycle.state() == ExplorationRunState::kPausing &&
              lifecycle.stopConfirmationPending(),
          "stop failure must preserve the pending parking gate");
  require(outbox.flush() == 1U, "stop-confirmation failure fact must publish");
  require(delivered.back().event.kind == ExplorationRunEventKind::kStopConfirmationFailed &&
              !delivered.back().event.motion_stop_confirmed,
          "stop failure must remain nonterminal and must not claim parking");

  require(lifecycle.confirmMotionStop(22.0, "navigation_goal_cancelled_after_stop"),
          "post-stop terminal must close pause transition");
  require(lifecycle.state() == ExplorationRunState::kPaused &&
              !lifecycle.stopConfirmationPending(),
          "parking proof must produce paused state");
  require(outbox.flush() == 1U, "paused fact must publish");
  require(delivered.back().event.state == ExplorationRunState::kPaused &&
              delivered.back().event.motion_stop_confirmed &&
              delivered.back().event.motion_stop_reason ==
                  "navigation_goal_cancelled_after_stop",
          "paused fact must carry parking evidence");
}

void testLifecycleCancellationAndCompletionTruth() {
  std::vector<ExplorationRunEventEnvelope> delivered;
  ExplorationRunEventOutbox outbox(
      "boot", [&](const ExplorationRunEventEnvelope &event) {
        delivered.push_back(event);
        return true;
      });
  ExplorationRunLifecycle lifecycle(outbox);
  require(lifecycle.start(binding(), 30.0, "exploration_start_admitted"), "start setup");
  require(lifecycle.cancel("stop-request", 31.0, "operator_stop", true),
          "stop with motion must enter cancelling");
  require(lifecycle.state() == ExplorationRunState::kCancelling && lifecycle.active(),
          "cancelling run remains open until parking proof");
  require(lifecycle.confirmMotionStop(32.0, "segment_cancelled_after_clear_motion"),
          "segment post-stop terminal closes cancellation");
  require(!lifecycle.active() && lifecycle.state() == ExplorationRunState::kCancelled,
          "cancelled is terminal only after parking proof");

  ExplorationRunBinding second = binding();
  second.exploration_run_id = "01ARZ3NDEKTSV4RRFFQ69G5FAW";
  second.start_request_id = "start-request-2";
  require(lifecycle.start(second, 40.0, "exploration_start_admitted"), "second run start");
  require(lifecycle.complete(41.0, "coverage_complete"),
          "completion with no pending motion must be terminal");
  require(!lifecycle.active() && lifecycle.state() == ExplorationRunState::kCompleted,
          "completed run must close");
  require(outbox.flush() == 7U, "all ordered lifecycle facts must remain deliverable");
  require(delivered.back().event.motion_stop_confirmed &&
              delivered.back().event.motion_stop_reason == "no_pending_motion",
          "motion-free completion must carry explicit parking evidence");
}

void testStopLifecycleUsesReservedOutboxCapacity() {
  bool writer_available = false;
  std::vector<ExplorationRunEventEnvelope> delivered;
  ExplorationRunEventOutbox outbox(
      "boot",
      [&](const auto &event) {
        if (!writer_available) {
          return false;
        }
        delivered.push_back(event);
        return true;
      },
      2U);
  ExplorationRunLifecycle lifecycle(outbox);

  require(lifecycle.start(binding(), 45.0, "exploration_start_admitted"),
          "start must fill the normal two-event outbox capacity");
  require(!outbox.canRecord(), "normal lifecycle facts must observe backpressure");
  require(outbox.canRecordMotionStop(ExplorationRunEventOutbox::kMotionStopReserve),
          "a full normal outbox must preserve the complete STOP fact budget");
  require(lifecycle.cancel("stop-request", 46.0, "operator_stop", true),
          "STOP must still record cancelling truth through reserved capacity");
  require(lifecycle.recordStopConfirmationFailure(46.5, "cancel_rejected"),
          "one stop failure fact must use reserved capacity");
  require(lifecycle.recordStopConfirmationFailure(46.6, "cancel_rejected"),
          "a repeated rejection for the same stop request must be idempotent");
  require(lifecycle.confirmMotionStop(47.0, "navigation_terminal_after_stop"),
          "STOP terminal must retain confirmed parking truth in the reserve");
  require(outbox.diagnostics().pending == 5U,
          "normal and safety-critical facts must remain ordered and retained");

  writer_available = true;
  require(outbox.flush() == 5U, "all retained lifecycle facts must flush after recovery");
  require(delivered[2].event.state == ExplorationRunState::kCancelling &&
              delivered[3].event.kind == ExplorationRunEventKind::kStopConfirmationFailed &&
              delivered[4].event.state == ExplorationRunState::kCancelled,
          "STOP facts must preserve cancelling, first failure, terminal FIFO order");
}

void testStopPromotesPendingPauseWithoutAnotherTransitionFact() {
  bool writer_available = false;
  std::vector<ExplorationRunEventEnvelope> delivered;
  ExplorationRunEventOutbox outbox(
      "boot",
      [&](const auto &event) {
        if (!writer_available) {
          return false;
        }
        delivered.push_back(event);
        return true;
      },
      2U);
  ExplorationRunLifecycle lifecycle(outbox);

  require(lifecycle.start(binding(), 48.0, "exploration_start_admitted"),
          "pause-to-stop setup start");
  require(lifecycle.pause("pause-request", 49.0, "operator_pause", true),
          "pause must begin the physical stop transition");
  require(lifecycle.recordStopConfirmationFailure(49.5, "cancel_rejected"),
          "pause stop failure must be retained once");
  require(lifecycle.cancel("stop-request", 50.0, "operator_stop", true),
          "STOP must promote the pending pause instead of adding another transition fact");
  require(lifecycle.confirmMotionStop(51.0, "navigation_terminal_after_stop"),
          "promoted STOP must close as cancelled");
  require(outbox.diagnostics().pending == 5U,
          "pause promotion must stay within the three motion-stop reserve slots");

  writer_available = true;
  require(outbox.flush() == 5U, "promoted STOP facts must flush after recovery");
  require(delivered[2].event.state == ExplorationRunState::kPausing &&
              delivered[3].event.kind == ExplorationRunEventKind::kStopConfirmationFailed &&
              delivered[4].event.state == ExplorationRunState::kCancelled &&
              delivered[4].event.command_request_id == "stop-request",
          "pending pause must terminate as the later STOP request without a duplicate transition");
}

void testLifecycleShutdownFailureWaitsForMotionStop() {
  std::vector<ExplorationRunEventEnvelope> delivered;
  ExplorationRunEventOutbox outbox("boot", [&](const ExplorationRunEventEnvelope &event) {
    delivered.push_back(event);
    return true;
  });
  ExplorationRunLifecycle lifecycle(outbox);

  require(lifecycle.start(binding(), 50.0, "exploration_start_admitted"), "shutdown failure setup");
  require(lifecycle.fail(51.0, "exploration_endpoint_stopped", true),
          "shutdown with possible motion must enter pending failure");
  require(lifecycle.active() && lifecycle.stopConfirmationPending() &&
              lifecycle.state() == ExplorationRunState::kCancelling,
          "shutdown failure claimed completion before motion terminal");
  require(
      lifecycle.recordStopConfirmationFailure(52.0, "exploration_shutdown_motion_stop_unconfirmed"),
      "shutdown deadline must remain durable while stop is unconfirmed");
  require(lifecycle.confirmMotionStop(53.0, "navigation_goal_terminal_during_shutdown"),
          "matching shutdown terminal must close the failed run");
  require(!lifecycle.active() && lifecycle.state() == ExplorationRunState::kFailed,
          "shutdown terminal did not produce a failed run");
}

}  // namespace

int main() {
  testOutboxRetriesOldestWithoutReordering();
  testOutboxRejectsFalseTerminalAndBackpressure();
  testRunFactsRequireCanonicalRouteMapIdentity();
  testLifecycleWaitsForPostStopTerminal();
  testLifecycleCancellationAndCompletionTruth();
  testStopLifecycleUsesReservedOutboxCapacity();
  testStopPromotesPendingPauseWithoutAnotherTransitionFact();
  testLifecycleShutdownFailureWaitsForMotionStop();
  std::cout << "test_exploration_run_event_outbox passed\n";
  return 0;
}
