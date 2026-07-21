#include "explore_control.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using lingtu::message::ExplorationCommandKind;
using lingtu::nav::endpoint::ExplorationControlRequest;
using lingtu::nav::endpoint::ExploreControl;

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ExplorationControlRequest request(
    const std::string& request_id,
    ExplorationCommandKind kind) {
  ExplorationControlRequest value;
  value.request_id = request_id;
  value.kind = static_cast<std::int32_t>(kind);
  value.frame_id = "map";
  value.stamp_s = 100.0;
  value.now_s = 100.1;
  value.max_age_s = 2.0;
  value.future_tolerance_s = 0.1;
  value.inputs_ready = true;
  return value;
}

void testStrictValidation() {
  ExploreControl control;
  auto invalid = request("", ExplorationCommandKind::kStart);
  require(
      control.Apply(invalid).reason == "exploration_request_id_empty",
      "empty request id must be rejected");

  invalid = request("frame", ExplorationCommandKind::kStart);
  invalid.frame_id = "odom";
  require(
      control.Apply(invalid).reason == "exploration_command_frame_must_be_map",
      "non-map control frame must be rejected");

  invalid = request("stale", ExplorationCommandKind::kStart);
  invalid.stamp_s = 90.0;
  require(
      control.Apply(invalid).reason == "exploration_command_stamp_stale",
      "stale control request must be rejected");

  invalid = request("unknown", ExplorationCommandKind::kStart);
  invalid.kind = 99;
  require(
      control.Apply(invalid).reason == "unknown_exploration_command",
      "unknown command kind must be rejected");
  require(!control.active(), "invalid controls must not activate exploration");
}

void testStartAndIdempotency() {
  ExploreControl control;
  auto start = request("start-1", ExplorationCommandKind::kStart);
  start.session_id = "session-a";
  const auto accepted = control.Apply(start);
  require(accepted.accepted, "start must be accepted with fresh inputs");
  require(accepted.reason == "exploration_started", "start reason");
  require(accepted.reset_planner, "start must reset planner state");
  require(accepted.clear_history, "start must clear prior history");
  require(control.running(), "start must enter running state");
  require(control.session_id() == "session-a", "requested session must be retained");

  const auto duplicate = control.Apply(start);
  require(duplicate.accepted && duplicate.duplicate, "duplicate start must replay ACK");
  require(!duplicate.reset_planner, "duplicate start must not repeat actions");

  auto mismatch = start;
  mismatch.kind = static_cast<std::int32_t>(ExplorationCommandKind::kStop);
  const auto mismatch_result = control.Apply(mismatch);
  require(!mismatch_result.accepted, "request id kind mismatch must be rejected");
  require(
      mismatch_result.reason == "duplicate_request_id_kind_mismatch",
      "mismatch reason");

  auto conflicting = request("start-2", ExplorationCommandKind::kStart);
  conflicting.session_id = "session-b";
  require(
      control.Apply(conflicting).reason == "exploration_session_conflict",
      "active session replacement must be rejected");
}

void testInputAndStopInProgressGates() {
  ExploreControl control;
  auto start = request("not-ready", ExplorationCommandKind::kStart);
  start.inputs_ready = false;
  require(
      control.Apply(start).reason == "exploration_inputs_not_ready",
      "start requires fresh exploration inputs");

  start = request("stopping", ExplorationCommandKind::kStart);
  start.cancellation_pending = true;
  require(
      control.Apply(start).reason == "exploration_stop_in_progress",
      "start must not race an outstanding cancellation");
}

void testPauseResumeAndStop() {
  ExploreControl control;
  auto start = request("start", ExplorationCommandKind::kStart);
  require(control.Apply(start).accepted, "start setup");

  auto pause = request("pause", ExplorationCommandKind::kPause);
  pause.goal_pending = true;
  const auto paused = control.Apply(pause);
  require(paused.accepted && control.paused(), "pause must latch paused state");
  require(paused.clear_queue, "pause must clear queued goals");
  require(paused.request_cancel, "pause must cancel an active goal");

  auto resume = request("resume-stale", ExplorationCommandKind::kResume);
  resume.inputs_ready = false;
  require(
      control.Apply(resume).reason == "exploration_inputs_not_ready",
      "resume requires fresh inputs");
  require(control.paused(), "failed resume must preserve paused state");

  resume = request("resume", ExplorationCommandKind::kResume);
  require(control.Apply(resume).accepted, "resume must succeed with fresh inputs");
  require(control.running(), "resume must restore running state");

  auto stop = request("stop", ExplorationCommandKind::kStop);
  stop.goal_pending = true;
  stop.reason = "operator_stop";
  const auto stopped = control.Apply(stop);
  require(stopped.accepted, "stop must be accepted");
  require(stopped.reason == "exploration_stop_in_progress", "stop progress reason");
  require(stopped.request_cancel, "stop must cancel active goal");
  require(stopped.cancel_reason == "operator_stop", "stop reason must reach cancel");
  require(stopped.reset_planner && stopped.clear_history, "stop must clear session state");
  require(!control.active(), "stop must disable exploration before cancellation completes");

  auto repeated = request("stop-again", ExplorationCommandKind::kStop);
  const auto repeated_result = control.Apply(repeated);
  require(repeated_result.accepted, "stop must be idempotent");
  require(
      repeated_result.reason == "exploration_already_stopped",
      "idempotent stop reason");
}

}  // namespace

int main() {
  testStrictValidation();
  testStartAndIdempotency();
  testInputAndStopInProgressGates();
  testPauseResumeAndStop();
  std::cout << "test_explore_control passed\n";
  return 0;
}
