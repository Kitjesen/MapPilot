#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "explore/exploration_run_event_outbox.hpp"
#include "explore/exploration_run_lifecycle.hpp"
#include "explore/explore_status_contract.hpp"

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
      std::string(64U, 'a'),
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
      std::string(64U, 'a'),
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

}  // namespace

int main() {
  testOutboxRetriesOldestWithoutReordering();
  testOutboxRejectsFalseTerminalAndBackpressure();
  testLifecycleWaitsForPostStopTerminal();
  testLifecycleCancellationAndCompletionTruth();
  std::cout << "test_exploration_run_event_outbox passed\n";
  return 0;
}
