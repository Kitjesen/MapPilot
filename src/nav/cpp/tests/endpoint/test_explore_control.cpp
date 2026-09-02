#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "endpoint/control.hpp"

namespace {

using lingtu::message::ExplorationCommandKind;
using lingtu::nav::endpoint::ExplorationControlRequest;
using lingtu::nav::endpoint::ExploreControl;

constexpr const char *kRunA = "01ARZ3NDEKTSV4RRFFQ69G5FAV";
constexpr const char *kRunB = "01ARZ3NDEKTSV4RRFFQ69G5FAW";

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testProductSessionStatusFileFallbackContract() {
  const auto source_path =
      std::filesystem::path(__FILE__).parent_path() /
      "../../../../explore/cpp/endpoint/main.cpp";
  std::ifstream input(source_path);
  require(input.good(), "explore endpoint source must be readable");
  const std::string source((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());

  const auto explicit_status =
      source.find("config.status_file = envString(\"LINGTU_EXPLORE_STATUS_FILE\");");
  const auto empty_guard = source.find("if (config.status_file.empty())", explicit_status);
  const auto session_root = source.find("envString(\"LINGTU_SESSION_ROOT\")", empty_guard);
  const auto fallback_name = source.find("\"explore.status.json\"", session_root);
  const auto cli_override = source.find("arg == \"--status-file\"", fallback_name);

  require(explicit_status != std::string::npos,
          "explicit exploration status environment must be read first");
  require(empty_guard != std::string::npos && session_root != std::string::npos &&
              fallback_name != std::string::npos,
          "missing exploration status path must fall back inside LINGTU_SESSION_ROOT");
  require(explicit_status < empty_guard && empty_guard < session_root &&
              session_root < fallback_name && fallback_name < cli_override,
          "explicit environment and CLI status paths must win over the session fallback");
}

void testProductIdentityStatusContract() {
  const auto source_path =
      std::filesystem::path(__FILE__).parent_path() /
      "../../../../explore/cpp/endpoint/main.cpp";
  std::ifstream input(source_path);
  require(input.good(), "explore endpoint source must be readable");
  const std::string source((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());

  const auto product_env = source.find("envString(\"LINGTU_PRODUCT\")");
  const auto product_session_env = source.find("envString(\"LINGTU_PRODUCT_SESSION_ID\")");
  const auto sim_guard = source.find("runtime_env == \"sim\"");
  const auto product_status = source.find("\\\"product\\\": \\\"", sim_guard);
  const auto product_session_status =
      source.find("\\\"product_session_id\\\": \\\"", product_status);
  const auto map_status = source.find("\\\"map\\\": {\\\"frame_id\\\": \\\"", product_status);
  const auto map_id_status = source.find("\\\"map_id\\\": \\\"", map_status);
  const auto duplicate_map_session = source.find("jsonEscape(map.session_id)", map_status);
  const auto duplicate_segment_session =
      source.find("jsonEscape(pending_segment->binding.session_id)", map_status);

  require(product_env != std::string::npos && product_session_env != std::string::npos,
          "explore Product identity must come from the canonical process environment");
  require(sim_guard != std::string::npos,
          "simulation Product launch must fail closed without exact Product identity");
  require(product_status != std::string::npos && product_session_status != std::string::npos,
          "explore status must expose Product and Product session identity at top level");
  require(map_status != std::string::npos && map_id_status != std::string::npos &&
              duplicate_map_session == std::string::npos &&
              duplicate_segment_session == std::string::npos,
          "explore status must not duplicate the top-level Product session identity");
}

ExplorationControlRequest request(const std::string &request_id, ExplorationCommandKind kind) {
  ExplorationControlRequest value;
  value.request_id = request_id;
  value.kind = static_cast<std::int32_t>(kind);
  value.frame_id = "map";
  value.stamp_s = 100.0;
  value.now_s = 100.1;
  value.max_age_s = 2.0;
  value.future_tolerance_s = 0.1;
  value.inputs_ready = true;
  value.snapshot_ready = true;
  value.product_session_id = "session-a";
  value.expected_product_session_id = "session-a";
  value.exploration_run_id = kRunA;
  return value;
}

void testStrictValidation() {
  ExploreControl control;
  auto invalid = request("", ExplorationCommandKind::kStart);
  require(control.Apply(invalid).reason == "exploration_request_id_empty",
          "empty request id must be rejected");

  invalid = request("frame", ExplorationCommandKind::kStart);
  invalid.frame_id = "odom";
  require(control.Apply(invalid).reason == "exploration_command_frame_must_be_map",
          "non-map control frame must be rejected");

  invalid = request("stale", ExplorationCommandKind::kStart);
  invalid.stamp_s = 90.0;
  require(control.Apply(invalid).reason == "exploration_command_stamp_stale",
          "stale control request must be rejected");

  invalid = request("unknown", ExplorationCommandKind::kStart);
  invalid.kind = 99;
  require(control.Apply(invalid).reason == "unknown_exploration_command",
          "unknown command kind must be rejected");
  require(!control.active(), "invalid controls must not activate exploration");

  invalid = request("missing-run", ExplorationCommandKind::kStart);
  invalid.exploration_run_id.clear();
  require(control.Apply(invalid).reason == "exploration_run_id_invalid",
          "start must require a caller-supplied ULID");

  invalid = request("lowercase-run", ExplorationCommandKind::kStart);
  invalid.exploration_run_id = "01arz3ndektsv4rrffq69g5fav";
  require(control.Apply(invalid).reason == "exploration_run_id_invalid",
          "run identity must be a canonical uppercase ULID");

  invalid = request(kRunA, ExplorationCommandKind::kStart);
  require(control.Apply(invalid).reason == "exploration_run_id_matches_request_id",
          "run identity must be distinct from the request identity");
}

void testStartRequiresVerifiedProductSession() {
  ExploreControl empty_control;
  auto empty = request("empty-session", ExplorationCommandKind::kStart);
  empty.product_session_id.clear();
  empty.expected_product_session_id = "product-session";
  const auto empty_result = empty_control.Apply(empty);
  require(!empty_result.accepted, "start without a session must be rejected");
  require(empty_result.reason == "exploration_product_session_id_empty", "empty session reason");
  require(!empty_control.active(), "empty session must not activate exploration");

  ExploreControl unverified_control;
  auto unverified = request("unverified-session", ExplorationCommandKind::kStart);
  unverified.product_session_id = "product-session";
  unverified.expected_product_session_id.clear();
  const auto unverified_result = unverified_control.Apply(unverified);
  require(!unverified_result.accepted, "start without a verified session must be rejected");
  require(unverified_result.reason == "exploration_product_session_unverified",
          "unverified session reason");
  require(!unverified_control.active(), "unverified session must not activate exploration");

  ExploreControl mismatch_control;
  auto mismatch = request("mismatched-session", ExplorationCommandKind::kStart);
  mismatch.product_session_id = "request-session";
  mismatch.expected_product_session_id = "product-session";
  const auto mismatch_result = mismatch_control.Apply(mismatch);
  require(!mismatch_result.accepted, "start for another Product session must be rejected");
  require(mismatch_result.reason == "exploration_product_session_mismatch",
          "Product session mismatch reason");
  require(!mismatch_control.active(), "mismatched session must not activate exploration");
}

void testStartAndIdempotency() {
  ExploreControl control;
  auto start = request("start-1", ExplorationCommandKind::kStart);
  start.product_session_id = "session-a";
  const auto accepted = control.Apply(start);
  require(accepted.accepted, "start must be accepted with fresh inputs");
  require(accepted.reason == "exploration_start_admitted", "start admission reason");
  require(accepted.exploration_run_id == kRunA, "start ACK must echo run identity");
  require(accepted.reset_planner, "start must reset planner state");
  require(accepted.clear_history, "start must clear prior history");
  require(control.running(), "start must enter running state");
  require(control.product_session_id() == "session-a", "requested session must be retained");
  require(control.exploration_run_id() == kRunA, "active run identity must be retained");

  const auto duplicate = control.Apply(start);
  require(duplicate.accepted && duplicate.duplicate, "duplicate start must replay ACK");
  require(!duplicate.reset_planner, "duplicate start must not repeat actions");
  require(duplicate.exploration_run_id == kRunA,
          "duplicate start must replay the same run identity");

  auto rebound_run_retry = start;
  rebound_run_retry.exploration_run_id = kRunB;
  const auto rebound_run_result = control.Apply(rebound_run_retry);
  require(!rebound_run_result.accepted && rebound_run_result.duplicate,
          "request id must not be rebound to another exploration run");
  require(rebound_run_result.reason == "duplicate_request_id_run_mismatch",
          "duplicate run mismatch reason");

  auto empty_session_retry = start;
  empty_session_retry.product_session_id.clear();
  const auto empty_session_result = control.Apply(empty_session_retry);
  require(!empty_session_result.accepted && empty_session_result.duplicate,
          "duplicate start with an empty session must be rejected");
  require(empty_session_result.reason == "exploration_product_session_id_empty",
          "duplicate empty session reason");

  auto rebound_retry = start;
  rebound_retry.product_session_id = "session-b";
  rebound_retry.expected_product_session_id = "session-b";
  const auto rebound_result = control.Apply(rebound_retry);
  require(!rebound_result.accepted && rebound_result.duplicate,
          "request id must not be rebound to another Product session");
  require(rebound_result.reason == "duplicate_request_id_product_session_mismatch",
          "duplicate session mismatch reason");

  auto mismatch = start;
  mismatch.kind = static_cast<std::int32_t>(ExplorationCommandKind::kStop);
  const auto mismatch_result = control.Apply(mismatch);
  require(!mismatch_result.accepted, "request id kind mismatch must be rejected");
  require(mismatch_result.reason == "duplicate_request_id_kind_mismatch", "mismatch reason");

  auto conflicting = request("start-2", ExplorationCommandKind::kStart);
  conflicting.product_session_id = "session-b";
  conflicting.expected_product_session_id = "session-b";
  conflicting.exploration_run_id = kRunB;
  require(control.Apply(conflicting).reason == "exploration_product_session_conflict",
          "active session replacement must be rejected");

  auto same_session_new_run = request("start-3", ExplorationCommandKind::kStart);
  same_session_new_run.exploration_run_id = kRunB;
  const auto run_conflict = control.Apply(same_session_new_run);
  require(!run_conflict.accepted, "a new START must not alias the active execution");
  require(run_conflict.reason == "exploration_start_conflict",
          "same-session START conflict reason");
}

void testCompletionClosesExecutionWithoutAutoResume() {
  ExploreControl control;
  const auto start = request("complete-me", ExplorationCommandKind::kStart);
  require(control.Apply(start).accepted, "completion setup start");

  require(control.Complete(), "active execution must transition to completed");
  require(!control.active() && !control.running(),
          "completion must close the active execution and stop scheduling motion");

  const auto replay = control.Apply(start);
  require(replay.accepted && replay.duplicate,
          "completed START replay must return the original acknowledgement");
  require(!control.active(), "completed START replay must never reactivate motion");

  auto next_request = request("next-start", ExplorationCommandKind::kStart);
  next_request.exploration_run_id = kRunB;
  const auto next = control.Apply(next_request);
  require(next.accepted && !next.duplicate, "a new START request must open a later run");
  require(control.running(), "new run must run only after its explicit START");

  require(control.Complete(), "second run completion setup");
  auto reused_run = request("third-start", ExplorationCommandKind::kStart);
  reused_run.exploration_run_id = kRunB;
  const auto reuse = control.Apply(reused_run);
  require(!reuse.accepted && reuse.reason == "exploration_run_id_reuse",
          "a new START must carry a never-before-admitted run identity");
}

void testLastStartReplaySurvivesAckCacheChurn() {
  ExploreControl control(1U);
  const auto start = request("stable-start", ExplorationCommandKind::kStart);
  require(control.Apply(start).accepted, "cache replay setup start");
  require(control.Apply(request("pause-cache", ExplorationCommandKind::kPause)).accepted,
          "cache replay setup pause");
  require(control.Apply(request("resume-cache", ExplorationCommandKind::kResume)).accepted,
          "cache replay setup resume");
  require(control.Complete(), "cache replay setup completion");

  const auto replay = control.Apply(start);
  require(replay.accepted && replay.duplicate,
          "last START must remain idempotent after generic ACK cache churn");
  require(!control.active(), "cache-churn replay must not reopen a completed run");
}

void testInputAndStopInProgressGates() {
  ExploreControl control;
  auto start = request("not-ready", ExplorationCommandKind::kStart);
  start.inputs_ready = false;
  require(control.Apply(start).reason == "exploration_inputs_not_ready",
          "start requires fresh exploration inputs");
  const auto duplicate_not_ready = control.Apply(start);
  require(!duplicate_not_ready.accepted && duplicate_not_ready.duplicate,
          "rejected start retry must replay its ACK");
  require(duplicate_not_ready.reason == "exploration_inputs_not_ready",
          "rejected start retry reason");

  start = request("stopping", ExplorationCommandKind::kStart);
  start.cancellation_pending = true;
  require(control.Apply(start).reason == "exploration_stop_in_progress",
          "start must not race an outstanding cancellation");
}

void testEventBackpressureCanBeRetried() {
  ExploreControl control;
  require(control.Apply(request("start-backpressure", ExplorationCommandKind::kStart)).accepted,
          "backpressure retry setup start");

  auto stop = request("stop-backpressure", ExplorationCommandKind::kStop);
  stop.goal_pending = true;
  stop.event_capacity_ready = false;
  const auto blocked = control.Apply(stop);
  require(!blocked.accepted && !blocked.duplicate,
          "full lifecycle outbox must reject without mutating control state");
  require(blocked.reason == "exploration_event_outbox_backpressure",
          "backpressure rejection reason");
  require(control.running(), "backpressure must leave the active run untouched");

  stop.event_capacity_ready = true;
  const auto retried = control.Apply(stop);
  require(retried.accepted && !retried.duplicate,
          "the same STOP request must be admitted after outbox recovery");
  require(retried.request_cancel, "recovered STOP must request physical cancellation");
  require(!control.active(), "admitted STOP must disable further planning");
}

void testPauseResumeAndStop() {
  ExploreControl control;
  auto start = request("start", ExplorationCommandKind::kStart);
  require(control.Apply(start).accepted, "start setup");

  auto foreign_pause = request("pause-foreign", ExplorationCommandKind::kPause);
  foreign_pause.product_session_id = "session-b";
  const auto foreign_pause_result = control.Apply(foreign_pause);
  require(!foreign_pause_result.accepted, "foreign session must not pause the active task");
  require(foreign_pause_result.reason == "exploration_product_session_mismatch",
          "foreign pause session mismatch reason");
  require(control.running(), "foreign pause must not change the active task");

  auto foreign_run_pause = request("pause-foreign-run", ExplorationCommandKind::kPause);
  foreign_run_pause.exploration_run_id = kRunB;
  const auto foreign_run_pause_result = control.Apply(foreign_run_pause);
  require(!foreign_run_pause_result.accepted, "foreign run must not pause the active run");
  require(foreign_run_pause_result.reason == "exploration_run_mismatch",
          "foreign pause run mismatch reason");

  auto pause = request("pause", ExplorationCommandKind::kPause);
  pause.goal_pending = true;
  const auto paused = control.Apply(pause);
  require(paused.accepted && control.paused(), "pause must latch paused state");
  require(paused.reason == "exploration_pause_admitted", "pause ACK is admission only");
  require(paused.clear_queue, "pause must clear queued goals");
  require(paused.request_cancel, "pause must cancel an active goal");

  auto resume = request("resume-stale", ExplorationCommandKind::kResume);
  resume.inputs_ready = false;
  require(control.Apply(resume).reason == "exploration_inputs_not_ready",
          "resume requires fresh inputs");
  require(control.paused(), "failed resume must preserve paused state");

  auto early_resume = request("resume-early", ExplorationCommandKind::kResume);
  early_resume.cancellation_pending = true;
  require(control.Apply(early_resume).reason == "exploration_pause_in_progress",
          "resume must not race the post-stop pause terminal");
  require(control.paused(), "early resume must preserve pausing state");

  auto foreign_resume = request("resume-foreign", ExplorationCommandKind::kResume);
  foreign_resume.product_session_id = "session-b";
  const auto foreign_resume_result = control.Apply(foreign_resume);
  require(!foreign_resume_result.accepted, "foreign session must not resume the active task");
  require(foreign_resume_result.reason == "exploration_product_session_mismatch",
          "foreign resume session mismatch reason");
  require(control.paused(), "foreign resume must preserve paused state");

  resume = request("resume", ExplorationCommandKind::kResume);
  const auto resumed = control.Apply(resume);
  require(resumed.accepted, "resume must succeed with fresh inputs");
  require(resumed.reason == "exploration_resume_admitted", "resume ACK is admission only");
  require(control.running(), "resume must restore running state");

  auto foreign_stop = request("stop-foreign", ExplorationCommandKind::kStop);
  foreign_stop.product_session_id = "session-b";
  foreign_stop.goal_pending = true;
  const auto foreign_stop_result = control.Apply(foreign_stop);
  require(!foreign_stop_result.accepted, "foreign session must not stop the active task");
  require(foreign_stop_result.reason == "exploration_product_session_mismatch",
          "foreign stop session mismatch reason");
  require(!foreign_stop_result.request_cancel,
          "foreign stop must not cancel the active session's goal");
  require(control.running(), "foreign stop must preserve the active task");

  auto foreign_run_stop = request("stop-foreign-run", ExplorationCommandKind::kStop);
  foreign_run_stop.exploration_run_id = kRunB;
  foreign_run_stop.goal_pending = true;
  const auto foreign_run_stop_result = control.Apply(foreign_run_stop);
  require(!foreign_run_stop_result.accepted, "foreign run must not stop the active run");
  require(foreign_run_stop_result.reason == "exploration_run_mismatch",
          "foreign stop run mismatch reason");
  require(!foreign_run_stop_result.request_cancel,
          "foreign run stop must not cancel active motion");

  auto stop = request("stop", ExplorationCommandKind::kStop);
  stop.goal_pending = true;
  stop.reason = "operator_stop";
  const auto stopped = control.Apply(stop);
  require(stopped.accepted, "stop must be accepted");
  require(stopped.reason == "exploration_stop_admitted", "stop ACK is admission only");
  require(stopped.request_cancel, "stop must cancel active goal");
  require(stopped.cancel_reason == "operator_stop", "stop reason must reach cancel");
  require(stopped.reset_planner && stopped.clear_history, "stop must clear session state");
  require(!control.active(), "stop must disable exploration before cancellation completes");

  auto repeated = request("stop-again", ExplorationCommandKind::kStop);
  const auto repeated_result = control.Apply(repeated);
  require(repeated_result.accepted, "stop must be idempotent");
  require(repeated_result.reason == "exploration_already_stopped", "idempotent stop reason");
}

void testDirectedTargetLifecycle() {
  ExploreControl control;
  auto start = request("start-directed", ExplorationCommandKind::kStart);
  start.product_session_id = "session-a";
  require(control.Apply(start).accepted, "directed start setup");

  auto set = request("directed-set", ExplorationCommandKind::kSetDirectedTarget);
  set.product_session_id = "session-a";
  set.has_directed_target = true;
  set.directed_target_x = 42.0;
  set.directed_target_y = -3.0;
  set.directed_target_ttl_s = 120.0;
  set.goal_pending = true;
  const auto accepted = control.Apply(set);
  require(accepted.accepted, "directed target must be accepted");
  require(accepted.set_directed_target, "set must request target update");
  require(accepted.clear_queue, "set must clear queued goal");
  require(accepted.request_cancel, "set must cancel prior goal");
  require(accepted.cancel_reason == "directed_exploration_retarget",
          "set must identify retarget cancellation");

  control.RecordIntentRevision(set.request_id, 7U);

  const auto duplicate = control.Apply(set);
  require(duplicate.accepted && duplicate.duplicate, "set retry must replay ACK");
  require(!duplicate.set_directed_target, "set retry must not repeat action");
  require(duplicate.intent_revision == 7U, "set retry must retain intent revision");

  const auto require_payload_mismatch = [&control, &set](auto mutate,
                                                          const std::string &message) {
    auto changed = set;
    mutate(changed);
    const auto result = control.Apply(changed);
    require(!result.accepted && result.duplicate, message);
    require(result.reason == "duplicate_request_id_directed_target_mismatch",
            "changed directed target retry must report a payload mismatch");
  };
  require_payload_mismatch(
      [](ExplorationControlRequest &changed) { changed.has_directed_target = false; },
      "request id must not be rebound to another directed target presence");
  require_payload_mismatch(
      [](ExplorationControlRequest &changed) { changed.directed_target_x += 1.0; },
      "request id must not be rebound to another directed target x coordinate");
  require_payload_mismatch(
      [](ExplorationControlRequest &changed) { changed.directed_target_y -= 1.0; },
      "request id must not be rebound to another directed target y coordinate");
  require_payload_mismatch(
      [](ExplorationControlRequest &changed) { changed.directed_target_ttl_s += 1.0; },
      "request id must not be rebound to another directed target ttl");

  control.RecordIntentOutcome(set.request_id, false, "directed_target_clock_invalid", 8U);
  const auto rejected_retry = control.Apply(set);
  require(!rejected_retry.accepted && rejected_retry.duplicate,
          "store rejection must replace the replayed ACK");
  require(rejected_retry.reason == "directed_target_clock_invalid",
          "store rejection reason must be replayed");
  require(rejected_retry.intent_revision == 8U, "store rejection revision must be replayed");

  auto wrong_session =
      request("directed-wrong-session", ExplorationCommandKind::kSetDirectedTarget);
  wrong_session.product_session_id = "session-b";
  wrong_session.has_directed_target = true;
  wrong_session.directed_target_ttl_s = 30.0;
  require(control.Apply(wrong_session).reason == "directed_target_product_session_mismatch",
          "directed target must stay within its exploration session");

  auto wrong_run = request("directed-wrong-run", ExplorationCommandKind::kSetDirectedTarget);
  wrong_run.exploration_run_id = kRunB;
  wrong_run.has_directed_target = true;
  wrong_run.directed_target_ttl_s = 30.0;
  require(control.Apply(wrong_run).reason == "directed_target_run_mismatch",
          "directed target must stay within its exploration run");

  auto stale_snapshot = request("directed-no-snapshot", ExplorationCommandKind::kSetDirectedTarget);
  stale_snapshot.product_session_id = "session-a";
  stale_snapshot.has_directed_target = true;
  stale_snapshot.directed_target_ttl_s = 30.0;
  stale_snapshot.snapshot_ready = false;
  require(control.Apply(stale_snapshot).reason == "exploration_snapshot_not_ready",
          "directed target requires a fresh snapshot");

  auto clear = request("directed-clear", ExplorationCommandKind::kClearDirectedTarget);
  clear.product_session_id = "session-a";
  const auto cleared = control.Apply(clear);
  require(cleared.accepted && cleared.clear_directed_target,
          "clear must remove the directed target");

  auto clear_retry = clear;
  clear_retry.has_directed_target = true;
  clear_retry.directed_target_x = 123.0;
  clear_retry.directed_target_y = 456.0;
  clear_retry.directed_target_ttl_s = 789.0;
  const auto duplicate_clear = control.Apply(clear_retry);
  require(duplicate_clear.accepted && duplicate_clear.duplicate,
          "clear retry must ignore unrelated directed target fields");
}

}  // namespace

int main() {
  testProductSessionStatusFileFallbackContract();
  testProductIdentityStatusContract();
  testStrictValidation();
  testStartRequiresVerifiedProductSession();
  testStartAndIdempotency();
  testCompletionClosesExecutionWithoutAutoResume();
  testLastStartReplaySurvivesAckCacheChurn();
  testInputAndStopInProgressGates();
  testEventBackpressureCanBeRetried();
  testPauseResumeAndStop();
  testDirectedTargetLifecycle();
  std::cout << "test_explore_control passed\n";
  return 0;
}
