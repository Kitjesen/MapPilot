#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "inspection/inspection_command_coordinator.hpp"

namespace {

using lingtu::nav::endpoint::InspectionActiveMap;
using lingtu::nav::endpoint::InspectionCommandAck;
using lingtu::nav::endpoint::InspectionCommandActions;
using lingtu::nav::endpoint::InspectionCommandCommit;
using lingtu::nav::endpoint::InspectionCommandCoordinator;
using lingtu::nav::endpoint::InspectionCommandRequest;
using lingtu::nav::endpoint::InspectionStopBarrierResult;
using lingtu::nav::inspection::CommandKind;
using lingtu::nav::inspection::Executor;
using lingtu::nav::inspection::Point;
using lingtu::nav::inspection::Route;
using lingtu::nav::inspection::RunState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::int32_t raw(CommandKind kind) {
  return static_cast<std::int32_t>(kind);
}

Route makeRoute(std::string route_id = "route-a") {
  Route value;
  value.id = std::move(route_id);
  value.name = "Production route";
  value.map_id = "map-a";
  value.map_version = 7;
  value.revision = 3U;
  Point point;
  point.id = "point-a";
  value.points.push_back(std::move(point));
  return value;
}

InspectionCommandRequest makeRequest(
    std::string request_id,
    CommandKind kind,
    std::string task_id = "existing-run") {
  InspectionCommandRequest value;
  value.task_id = std::move(task_id);
  value.request_id = std::move(request_id);
  value.raw_kind = raw(kind);
  return value;
}

struct Harness {
  Executor executor;
  bool route_source_ready{true};
  std::optional<InspectionActiveMap> map{InspectionActiveMap{"map-a", 7}};
  bool route_found{true};
  bool takeover{false};
  bool stop_confirmed{true};
  bool ack_ok{true};
  double clock{42.0};
  int load_calls{0};
  int status_calls{0};
  std::vector<std::string> log;
  std::vector<InspectionCommandAck> acks;

  InspectionCommandActions actions() {
    InspectionCommandActions result;
    result.route_source_available = [&]() {
      log.emplace_back("route_source");
      return route_source_ready;
    };
    result.active_map = [&]() {
      log.emplace_back("active_map");
      return map;
    };
    result.load_route = [&](const std::string &map_id, const std::string &route_id) {
      ++load_calls;
      log.emplace_back("load:" + map_id + ":" + route_id);
      return route_found ? std::optional<Route>{makeRoute(route_id)} : std::nullopt;
    };
    result.operator_takeover_latched = [&]() {
      log.emplace_back("takeover");
      return takeover;
    };
    result.stop_and_commit = [&](const std::string &reason, InspectionCommandCommit commit) {
      log.emplace_back("stop:" + reason);
      if (!stop_confirmed) {
        return InspectionStopBarrierResult{
            false,
            "stop_confirmation_timeout_" + reason,
        };
      }
      if (!commit()) {
        return InspectionStopBarrierResult{
            false,
            "inspection_state_commit_failed",
        };
      }
      log.emplace_back("stop-confirmed");
      return InspectionStopBarrierResult{true, "zero_confirmed"};
    };
    result.publish_ack = [&](const InspectionCommandAck &ack) {
      log.emplace_back("ack:" + ack.reason);
      acks.push_back(ack);
      return ack_ok;
    };
    result.request_status = [&]() {
      log.emplace_back("status");
      ++status_calls;
    };
    result.now_s = [&]() {
      log.emplace_back("now");
      return clock;
    };
    return result;
  }

  void startDirect(const std::string &run_id = "existing-run") {
    std::string error;
    require(executor.Start(
                makeRoute(), run_id, "fixture-start", "map-a", 7, clock, &error),
            "fixture start must succeed");
  }
};

void requireAck(const InspectionCommandAck &ack, const std::string &request_id, CommandKind kind,
                bool accepted, const std::string &reason, const std::string &run_id,
                std::optional<std::string> task_id = std::nullopt) {
  require(ack.request_id == request_id, "ACK request id mismatch");
  require(ack.kind == kind, "ACK kind mismatch");
  require(ack.accepted == accepted, "ACK accepted mismatch");
  require(ack.reason == reason, "ACK reason mismatch");
  const std::string expected_task_id = task_id.value_or(run_id);
  if (ack.task_id != expected_task_id) {
    throw std::runtime_error(
        "ACK task id mismatch for " + request_id + ": expected=" + expected_task_id +
        " actual=" + ack.task_id);
  }
  if (ack.run_id != run_id) {
    throw std::runtime_error(
        "ACK run id mismatch for " + request_id + ": expected=" + run_id +
        " actual=" + ack.run_id);
  }
}

void testInvalidAndUnknown() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  InspectionCommandRequest invalid;
  invalid.raw_kind = 99;
  auto result = coordinator.handle(invalid);
  requireAck(result.ack, "", static_cast<CommandKind>(99), false, "inspection_request_id_empty",
             "");
  require(result.ack_published && !result.replayed && !result.status_requested,
          "empty-id result flags mismatch");
  require(h.log == std::vector<std::string>{"ack:inspection_request_id_empty"},
          "empty id must win before unknown-kind validation");

  h.log.clear();
  InspectionCommandRequest unknown;
  unknown.task_id = "unknown-task";
  unknown.request_id = "unknown-1";
  unknown.raw_kind = 99;
  result = coordinator.handle(unknown);
  requireAck(result.ack, "unknown-1", static_cast<CommandKind>(99), false,
             "unknown_inspection_command", "", "unknown-task");
  require(!result.replayed && !result.status_requested, "unknown command flags mismatch");
  h.log.clear();
  result = coordinator.handle(unknown);
  require(!result.replayed, "unknown command must not be cached");
  require(h.log == std::vector<std::string>{"ack:unknown_inspection_command"},
          "unknown retry must only republish rejection");
}

void testStartReplayMismatchAndOrdering() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());
  auto start = makeRequest("start-1", CommandKind::kStart);
  start.route_id = "route-a";
  start.route_revision = 3U;

  auto result = coordinator.handle(start);
  requireAck(result.ack, "start-1", CommandKind::kStart, true, "inspection_route_accepted",
             "existing-run");
  require(!result.replayed && result.status_requested && h.executor.active(),
          "new start flags/state mismatch");
  require(h.log == std::vector<std::string>({"active_map", "route_source", "load:map-a:route-a",
                                             "now", "ack:inspection_route_accepted", "status"}),
          "start action ordering mismatch");

  h.log.clear();
  result = coordinator.handle(start);
  requireAck(result.ack, "start-1", CommandKind::kStart, true, "inspection_route_accepted",
             "existing-run");
  require(result.replayed && !result.status_requested, "matching duplicate flags mismatch");
  require(h.log == std::vector<std::string>{"ack:inspection_route_accepted"},
          "matching duplicate must not redispatch");

  h.log.clear();
  result = coordinator.handle(makeRequest("start-1", CommandKind::kPause));
  requireAck(result.ack, "start-1", CommandKind::kPause, false,
             "duplicate_request_id_kind_mismatch", "");
  require(result.replayed && !result.status_requested, "kind-mismatch duplicate flags mismatch");
  require(h.log == std::vector<std::string>{"ack:duplicate_request_id_kind_mismatch"},
          "kind mismatch must not redispatch or expose run id");
}

void testStartDecisionOrderAndRouteChecks() {
  Harness unavailable;
  unavailable.startDirect();
  unavailable.route_source_ready = false;
  InspectionCommandCoordinator c1(unavailable.executor, unavailable.actions());
  auto start = makeRequest("unavailable", CommandKind::kStart);
  start.route_id = "route-a";
  auto result = c1.handle(start);
  requireAck(result.ack, "unavailable", CommandKind::kStart, false, "active_map_unavailable",
             "existing-run");
  require(unavailable.load_calls == 0, "unavailable source must precede load");
  require(unavailable.log == std::vector<std::string>({"active_map", "route_source",
                                                       "ack:active_map_unavailable", "status"}),
          "unavailable source must precede active-run check");

  Harness active;
  active.startDirect();
  InspectionCommandCoordinator c2(active.executor, active.actions());
  start.request_id = "active";
  result = c2.handle(start);
  requireAck(result.ack, "active", CommandKind::kStart, false, "inspection_run_active",
             "existing-run");
  require(active.load_calls == 0, "active run must precede route loading");

  Harness missing;
  missing.route_found = false;
  InspectionCommandCoordinator c3(missing.executor, missing.actions());
  start.request_id = "missing";
  result = c3.handle(start);
  requireAck(result.ack, "missing", CommandKind::kStart, false, "inspection_route_not_found", "",
             "existing-run");

  Harness revision;
  InspectionCommandCoordinator c4(revision.executor, revision.actions());
  start.request_id = "revision";
  start.route_revision = 4U;
  result = c4.handle(start);
  requireAck(result.ack, "revision", CommandKind::kStart, false,
             "inspection_route_revision_mismatch", "", "existing-run");
  require(!revision.executor.active(), "revision mismatch must not start run");
}

void testFifoCache() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions(), 2U);
  auto first = makeRequest("cancel-a", CommandKind::kCancel);
  auto second = makeRequest("cancel-b", CommandKind::kCancel);
  auto third = makeRequest("cancel-c", CommandKind::kCancel);
  (void)coordinator.handle(first);
  (void)coordinator.handle(second);
  require(coordinator.handle(first).replayed, "first must initially replay");
  (void)coordinator.handle(third);

  h.log.clear();
  const auto evicted = coordinator.handle(first);
  require(!evicted.replayed && evicted.status_requested,
          "FIFO overflow must evict oldest without replay refresh");
  require(h.log == std::vector<std::string>(
                       {"ack:inspection_task_id_mismatch", "status"}),
          "evicted request must execute as a new legal command");
}

void testPauseAndCancelAwaitStopConfirmation() {
  {
    Harness h;
    h.startDirect();
    h.stop_confirmed = false;
    InspectionCommandCoordinator coordinator(h.executor, h.actions());

    const auto result = coordinator.handle(makeRequest("pause-1", CommandKind::kPause));
    requireAck(result.ack, "pause-1", CommandKind::kPause, true, "pause_requested",
               "existing-run");
    require(h.executor.status().state == RunState::kPausing,
            "unconfirmed pause must remain visibly pending");
    require(h.executor.status().reason ==
                "stop_confirmation_timeout_inspection_pause_requested",
            "pending pause must retain the stop-evidence failure");
    require(h.log == std::vector<std::string>({
                         "now",
                         "stop:inspection_pause_requested",
                         "now",
                         "ack:pause_requested",
                         "status",
                     }),
            "pause request must be acknowledged without claiming completion");
  }

  {
    Harness h;
    h.startDirect();
    InspectionCommandCoordinator coordinator(h.executor, h.actions());

    const auto result = coordinator.handle(makeRequest("pause-2", CommandKind::kPause));
    requireAck(result.ack, "pause-2", CommandKind::kPause, true, "pause_requested",
               "existing-run");
    require(h.executor.status().state == RunState::kPaused,
            "confirmed pause may become PAUSED");
    require(h.log == std::vector<std::string>({
                         "now",
                         "stop:inspection_pause_requested",
                         "now",
                         "stop-confirmed",
                         "ack:pause_requested",
                         "status",
                     }),
            "pause terminal state must be committed after stop confirmation");
  }

  {
    Harness h;
    h.startDirect();
    h.stop_confirmed = false;
    InspectionCommandCoordinator coordinator(h.executor, h.actions());

    const auto result = coordinator.handle(makeRequest("cancel-1", CommandKind::kCancel));
    requireAck(result.ack, "cancel-1", CommandKind::kCancel, true, "cancel_requested",
               "existing-run");
    require(h.executor.status().state == RunState::kCancelling,
            "unconfirmed cancel must never be exposed as CANCELLED");
    require(h.executor.status().reason ==
                "stop_confirmation_timeout_inspection_cancel_requested",
            "pending cancel must retain the stop-evidence failure");
    require(h.log == std::vector<std::string>({
                         "now",
                         "stop:inspection_cancel_requested",
                         "now",
                         "ack:cancel_requested",
                         "status",
                     }),
            "cancel request must not claim a terminal result before proof");
  }

  {
    Harness h;
    h.startDirect();
    InspectionCommandCoordinator coordinator(h.executor, h.actions());

    const auto result = coordinator.handle(makeRequest("cancel-2", CommandKind::kCancel));
    requireAck(result.ack, "cancel-2", CommandKind::kCancel, true, "cancel_requested",
               "existing-run");
    require(h.executor.status().state == RunState::kCancelled,
            "confirmed cancel may become CANCELLED");
    require(h.log == std::vector<std::string>({
                         "now",
                         "stop:inspection_cancel_requested",
                         "now",
                         "stop-confirmed",
                         "ack:cancel_requested",
                         "status",
                     }),
            "cancel terminal state must be committed after stop confirmation");
  }
}

void testTaskIdentityIsDistinctAndLifecycleCommandsTargetIt() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());
  auto start = makeRequest(
      "request-start-42",
      CommandKind::kStart,
      "inspection-task-42");
  start.route_id = "route-a";
  start.route_revision = 3U;

  const auto accepted = coordinator.handle(start);
  requireAck(accepted.ack, "request-start-42", CommandKind::kStart, true,
             "inspection_route_accepted", "inspection-task-42");
  require(h.executor.status().task_id == "inspection-task-42",
          "native executor must retain the canonical task identity");
  require(h.executor.status().request_id == "request-start-42",
          "native executor must retain the accepted start request identity");
  require(h.executor.status().task_id != h.executor.status().request_id,
          "task and request identities must never collapse");

  auto duplicate_with_other_task = start;
  duplicate_with_other_task.task_id = "inspection-task-other";
  const auto collision = coordinator.handle(duplicate_with_other_task);
  requireAck(collision.ack, "request-start-42", CommandKind::kStart, false,
             "duplicate_request_id_payload_mismatch", "");
  require(collision.replayed && !collision.status_requested,
          "request-id payload collision must not redispatch");

  const auto wrong_task = coordinator.handle(makeRequest(
      "request-pause-wrong",
      CommandKind::kPause,
      "inspection-task-other"));
  requireAck(wrong_task.ack, "request-pause-wrong", CommandKind::kPause, false,
             "inspection_task_id_mismatch", "inspection-task-42", "inspection-task-other");
  require(h.executor.status().state == RunState::kPlanning,
          "a task-id mismatch must not mutate the active inspection task");

  const auto pause = coordinator.handle(makeRequest(
      "request-pause-42",
      CommandKind::kPause,
      "inspection-task-42"));
  requireAck(pause.ack, "request-pause-42", CommandKind::kPause, true,
             "pause_requested", "inspection-task-42");
  require(h.executor.status().state == RunState::kPaused,
          "the matching task may transition after stop confirmation");
  require(h.executor.status().request_id == "request-pause-42",
          "status must expose the lifecycle command that produced it");
}

void testTaskIdentityIsRequiredAndReplayed() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  auto start = makeRequest("task-wire-start", CommandKind::kStart);
  start.task_id.clear();
  start.route_id = "route-a";
  start.route_revision = 3U;

  const auto missing_identity = coordinator.handle(start);
  require(!missing_identity.ack.accepted,
          "the task wire must reject a start without caller-owned task id");
  require(missing_identity.ack.reason == "inspection_task_id_empty",
          "the task wire must explain the missing task identity");
  require(h.load_calls == 0 && h.status_calls == 0,
          "a missing task id must not begin route admission");

  start.task_id = "inspection-task-product-42";
  const auto accepted = coordinator.handle(start);
  require(accepted.ack.accepted, "a caller-owned task id must be admitted");
  require(accepted.ack.task_id == "inspection-task-product-42",
          "the task ACK must retain the caller-owned task identity");

  const auto replay = coordinator.handle(start);
  require(replay.replayed && replay.ack.accepted,
          "an identical task-wire request must replay its original admission");
  require(replay.ack.task_id == "inspection-task-product-42",
          "a replayed task ACK must not replace task_id with run_id");
}

void testResumeTakeover() {
  Harness h;
  h.startDirect();
  require(h.executor.Pause("fixture"), "fixture pause must succeed");
  h.takeover = true;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  const auto result = coordinator.handle(makeRequest("resume-1", CommandKind::kResume));
  requireAck(result.ack, "resume-1", CommandKind::kResume, false,
             "inspection_resume_requires_autonomy", "existing-run");
  require(h.executor.status().state == RunState::kPaused,
          "takeover rejection must leave run paused");
  require(h.log == std::vector<std::string>({"active_map", "takeover",
                                             "ack:inspection_resume_requires_autonomy", "status"}),
          "takeover must precede time sampling and resume");
}

void testAckFailureDoesNotRollbackAndRetryReplays() {
  Harness h;
  h.ack_ok = false;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());
  auto start = makeRequest("publish-failure", CommandKind::kStart);
  start.route_id = "route-a";
  start.route_revision = 3U;

  auto result = coordinator.handle(start);
  require(result.ack.accepted && !result.ack_published && result.status_requested &&
              h.executor.active(),
          "ACK failure must not roll back admission or suppress status");
  require(h.status_calls == 1, "new command must request status once");

  h.ack_ok = true;
  h.log.clear();
  result = coordinator.handle(start);
  require(result.replayed && result.ack_published && result.ack.accepted &&
              !result.status_requested,
          "retry must replay cached accepted ACK");
  require(h.status_calls == 1, "replay must not request status again");
  require(h.log == std::vector<std::string>{"ack:inspection_route_accepted"},
          "retry must only republish cached ACK");
}

void testRejectsFreshCommandsWithoutSideEffects() {
  Harness h;
  h.startDirect();
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  const std::vector<CommandKind> kinds{
      CommandKind::kStart,
      CommandKind::kPause,
      CommandKind::kResume,
      CommandKind::kCancel,
  };
  for (std::size_t index = 0; index < kinds.size(); ++index) {
    const auto request_id = "barrier-" + std::to_string(index);
    const auto result =
        coordinator.reject(makeRequest(request_id, kinds[index]), "goal_terminal_pending");
    requireAck(result.ack, request_id, kinds[index], false, "goal_terminal_pending",
               "existing-run");
    require(result.ack_published && !result.replayed && !result.status_requested,
            "fresh barrier rejection flags mismatch");
  }

  require(h.executor.status().state == RunState::kPlanning,
          "barrier rejection must not execute inspection commands");
  require(h.load_calls == 0 && h.status_calls == 0,
          "barrier rejection must not load routes or request status");
  require(h.log == std::vector<std::string>({
                       "ack:goal_terminal_pending",
                       "ack:goal_terminal_pending",
                       "ack:goal_terminal_pending",
                       "ack:goal_terminal_pending",
                   }),
          "barrier rejection must only publish ACKs");
}

void testRejectCachesBeforePublishAndOnlyReplays() {
  Harness h;
  h.startDirect();
  h.ack_ok = false;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());
  const auto request = makeRequest("barrier-retry", CommandKind::kPause);

  auto result = coordinator.reject(request, "goal_terminal_pending");
  requireAck(result.ack, "barrier-retry", CommandKind::kPause, false, "goal_terminal_pending",
             "existing-run");
  require(!result.ack_published && !result.replayed && !result.status_requested,
          "initial failed barrier ACK flags mismatch");

  h.ack_ok = true;
  h.log.clear();
  result = coordinator.reject(request, "replacement_reason_must_not_win");
  requireAck(result.ack, "barrier-retry", CommandKind::kPause, false, "goal_terminal_pending",
             "existing-run");
  require(result.ack_published && result.replayed && !result.status_requested,
          "barrier retry must replay cached ACK");
  require(h.log == std::vector<std::string>{"ack:goal_terminal_pending"},
          "barrier retry must only republish the original ACK");

  h.log.clear();
  result = coordinator.reject(makeRequest("barrier-retry", CommandKind::kCancel),
                              "goal_terminal_pending");
  requireAck(result.ack, "barrier-retry", CommandKind::kCancel, false,
             "duplicate_request_id_kind_mismatch", "");
  require(result.ack_published && result.replayed && !result.status_requested,
          "barrier kind mismatch flags mismatch");
  require(h.log == std::vector<std::string>{"ack:duplicate_request_id_kind_mismatch"},
          "barrier kind mismatch must only publish its ACK");
  require(h.executor.status().state == RunState::kPlanning,
          "barrier retries must never execute the inspection command");
}

void testRejectValidationAndFallbackReason() {
  Harness h;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  InspectionCommandRequest invalid;
  invalid.raw_kind = 99;
  auto result = coordinator.reject(invalid, "goal_terminal_pending");
  requireAck(result.ack, "", static_cast<CommandKind>(99), false, "inspection_request_id_empty",
             "");
  require(result.ack_published && !result.replayed && !result.status_requested,
          "empty-id barrier rejection flags mismatch");
  h.log.clear();
  result = coordinator.reject(invalid, "goal_terminal_pending");
  require(!result.replayed, "empty-id barrier rejection must not be cached");
  require(h.log == std::vector<std::string>{"ack:inspection_request_id_empty"},
          "empty id must win before unknown kind at the barrier");

  h.log.clear();
  InspectionCommandRequest unknown;
  unknown.task_id = "unknown-task";
  unknown.request_id = "barrier-unknown";
  unknown.raw_kind = 99;
  result = coordinator.reject(unknown, "goal_terminal_pending");
  requireAck(result.ack, "barrier-unknown", static_cast<CommandKind>(99), false,
             "unknown_inspection_command", "", "unknown-task");
  require(!result.replayed && !result.status_requested, "unknown barrier rejection flags mismatch");
  h.log.clear();
  result = coordinator.reject(unknown, "goal_terminal_pending");
  require(!result.replayed, "unknown barrier command must not be cached");
  require(h.log == std::vector<std::string>{"ack:unknown_inspection_command"},
          "unknown barrier retry must only republish validation rejection");

  h.log.clear();
  const auto valid = makeRequest("barrier-empty-reason", CommandKind::kResume);
  result = coordinator.reject(valid, "");
  requireAck(result.ack, "barrier-empty-reason", CommandKind::kResume, false,
             "inspection_command_rejected", "", "existing-run");
  require(!result.replayed && !result.status_requested,
          "empty barrier reason must fail closed with a fresh rejection");

  h.log.clear();
  result = coordinator.handle(valid);
  requireAck(result.ack, "barrier-empty-reason", CommandKind::kResume, false,
             "inspection_command_rejected", "", "existing-run");
  require(result.replayed && !result.status_requested,
          "normal handling must replay the barrier rejection");
  require(h.log == std::vector<std::string>{"ack:inspection_command_rejected"},
          "barrier-cached command must not execute after the barrier opens");
}

void expectInvalid(Executor &executor, InspectionCommandActions actions,
                   std::size_t cache_limit = 128U) {
  bool threw = false;
  try {
    InspectionCommandCoordinator coordinator(executor, std::move(actions), cache_limit);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  require(threw, "constructor must reject incomplete configuration");
}

void testConstructorValidation() {
  Harness h;
  const auto complete = h.actions();
  auto invalid = complete;
  invalid.route_source_available = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.active_map = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.load_route = {};
  expectInvalid(h.executor, invalid);
  invalid.operator_takeover_latched = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.stop_and_commit = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.publish_ack = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.request_status = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.now_s = {};
  expectInvalid(h.executor, invalid);
  expectInvalid(h.executor, complete, 0U);

  InspectionCommandCoordinator valid(h.executor, complete);
  (void)valid;
}

}  // namespace

int main() {
  try {
    testInvalidAndUnknown();
    testStartReplayMismatchAndOrdering();
    testStartDecisionOrderAndRouteChecks();
    testFifoCache();
    testPauseAndCancelAwaitStopConfirmation();
    testTaskIdentityIsDistinctAndLifecycleCommandsTargetIt();
    testTaskIdentityIsRequiredAndReplayed();
    testResumeTakeover();
    testAckFailureDoesNotRollbackAndRetryReplays();
    testRejectsFreshCommandsWithoutSideEffects();
    testRejectCachesBeforePublishAndOnlyReplays();
    testRejectValidationAndFallbackReason();
    testConstructorValidation();
  } catch (const std::exception &ex) {
    std::cerr << "FAIL: " << ex.what() << '\n';
    return 1;
  }
  std::cout << "inspection command coordinator tests passed\n";
  return 0;
}
