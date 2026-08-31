#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "command/ingress.hpp"

namespace {
using lingtu::message::NavigationCommandKind;
using lingtu::nav::endpoint::CommandAck;
using lingtu::nav::endpoint::CommandIngressController;
using lingtu::nav::endpoint::CommandIngressRequest;
using lingtu::nav::endpoint::CommandPayload;
using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

CommandIngressRequest request(std::string client, std::string id, NavigationCommandKind kind,
                              std::string payload = {}) {
  CommandPayload value;
  value.reason = std::move(payload);
  std::string task_id;
  if (kind == NavigationCommandKind::Goal || kind == NavigationCommandKind::TaskCancel ||
      kind == NavigationCommandKind::TaskPause || kind == NavigationCommandKind::TaskResume) {
    task_id = "task-" + id;
  }
  return {
      std::move(client),
      std::move(id),
      std::move(task_id),
      static_cast<std::int32_t>(kind),
      std::move(value),
  };
}

void testTaskPauseResumeCommandsAreDistinctUserActions() {
  CommandIngressController controller;
  const auto now = CommandIngressController::Clock::time_point{120s};
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind, const CommandPayload &) {
    ++dispatches;
    return CommandAck{true, "accepted"};
  };

  auto first_pause = request(
      "operator", "pause-1", NavigationCommandKind::TaskPause, "operator_pause");
  first_pause.task_id = "navigation-task-1";
  const auto paused = controller.handle(first_pause, dispatch, now);
  require(paused.dispatched && !paused.replayed, "first task pause must dispatch");

  auto resume = request(
      "operator", "resume-1", NavigationCommandKind::TaskResume, "operator_resume");
  resume.task_id = first_pause.task_id;
  require(controller.handle(resume, dispatch, now + 1s).dispatched,
          "task resume must dispatch");

  auto second_pause = first_pause;
  second_pause.request_id = "pause-2";
  require(controller.handle(second_pause, dispatch, now + 2s).dispatched,
          "a later pause of the same task must dispatch");
  auto aliased_task = second_pause;
  aliased_task.task_id = "navigation-task-2";
  const auto alias_result = controller.handle(aliased_task, dispatch, now + 3s);
  require(!alias_result.dispatched && alias_result.ack.reason == "idempotency_conflict",
          "one pause request id must not alias another task");
  require(controller.handle(second_pause, dispatch, now + 3s).replayed,
          "an exact pause request retry must replay its ACK");
  require(dispatches == 3, "pause-resume-pause dispatched the wrong number of user actions");
}

void testValidationOrderAndDiagnostics() {
  CommandIngressController controller;
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind, const CommandPayload &) {
    ++dispatches;
    return CommandAck{true, "accepted"};
  };
  auto invalid = request("", "", NavigationCommandKind::Goal);
  invalid.raw_kind = 99;
  require(controller.handle(invalid, dispatch).ack.reason == "command_request_id_empty",
          "request id validation must run first");
  invalid.request_id = "request";
  require(controller.handle(invalid, dispatch).ack.reason == "command_client_id_empty",
          "client id validation must precede kind validation");
  invalid.client_id = "client";
  require(controller.handle(invalid, dispatch).ack.reason == "unknown_command_kind",
          "known-kind validation must run last");
  require(dispatches == 0, "invalid commands must not dispatch");
  controller.recordAckPublication(true);
  controller.recordAckPublication(false);
  controller.recordAckPublication(true);
  const auto &diagnostics = controller.diagnostics();
  require(diagnostics.received == 3, "received mismatch");
  require(diagnostics.ack_sent == 2, "published ACK mismatch");
  require(diagnostics.ack_publish_failed == 1, "failed ACK publish mismatch");
  require(diagnostics.rejected == 3, "rejected mismatch");
  require(diagnostics.last_kind == "unknown", "unknown kind diagnostic mismatch");
}

void testReplayConflictAndPartitions() {
  CommandIngressController controller;
  const auto now = CommandIngressController::Clock::time_point{10s};
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind, const CommandPayload &payload) {
    ++dispatches;
    return CommandAck{payload.reason != "reject", payload.reason};
  };
  const auto accepted = request("client-a", "shared", NavigationCommandKind::Goal, "accepted");
  const auto first = controller.handle(accepted, dispatch, now);
  const auto replay = controller.handle(accepted, dispatch, now + 1s);
  require(first.dispatched && !first.replayed, "first command must dispatch");
  require(first.request_id == "shared", "result request id mismatch");
  require(first.kind == NavigationCommandKind::Goal, "result kind mismatch");
  require(replay.replayed && !replay.dispatched, "accepted ACK must replay");
  require(replay.ack.accepted, "accepted replay changed disposition");

  const auto rejected = request("client-a", "rejected", NavigationCommandKind::Stop, "reject");
  (void)controller.handle(rejected, dispatch, now + 2s);
  const auto rejected_replay = controller.handle(rejected, dispatch, now + 3s);
  require(rejected_replay.dispatched && !rejected_replay.replayed, "rejected ACK must dispatch again");
  require(!rejected_replay.ack.accepted, "rejected retry changed disposition");

  const auto conflict = request("client-a", "shared", NavigationCommandKind::Goal, "different");
  const auto conflict_result = controller.handle(conflict, dispatch, now + 4s);
  require(!conflict_result.dispatched && conflict_result.ack.reason == "idempotency_conflict",
          "conflicting payload must reject without dispatch");

  const auto other_client = request("client-b", "shared", NavigationCommandKind::Goal, "different");
  require(controller.handle(other_client, dispatch, now + 5s).dispatched,
          "client must partition journal identity");
  const auto other_kind = request("client-a", "shared", NavigationCommandKind::Stop, "different");
  require(controller.handle(other_kind, dispatch, now + 6s).dispatched,
          "kind must partition journal identity");
  require(dispatches == 5, "dispatch count mismatch");
  const auto &diagnostics = controller.diagnostics();
  require(diagnostics.replayed == 1, "replay diagnostic mismatch");
  require(diagnostics.rejected == 3, "rejected diagnostic mismatch");
}

void testNegativeCommandAcksAreNotCached() {
  struct Case {
    NavigationCommandKind kind;
    const char *request_id;
    const char *reason;
  };
  const Case cases[] = {
      {NavigationCommandKind::TaskCancel, "cancel-terminal-pending", "goal_terminal_pending"},
      {NavigationCommandKind::TaskCancel, "cancel-stop-timeout",
       "stop_confirmation_timeout_cancel_remains_stopped"},
      {NavigationCommandKind::Stop, "stop-terminal-pending", "goal_terminal_pending"},
      {NavigationCommandKind::Estop, "estop-persist-failed",
       "estop_latch_persist_failed_estop_remains_latched"},
      {NavigationCommandKind::Estop, "estop-zero-unavailable",
       "zero_publish_unavailable_estop_remains_latched"},
      {NavigationCommandKind::Estop, "estop-zero-timeout",
       "stop_confirmation_timeout_estop_remains_latched"},
      {NavigationCommandKind::ClearEstop, "clear-estop-stale",
       "clear_estop_source_stamp_stale"},
      {NavigationCommandKind::Goal, "goal-planner-busy", "planner_busy"},
  };

  for (const auto &entry : cases) {
    CommandIngressController controller;
    const auto now = CommandIngressController::Clock::time_point{40s};
    int dispatches = 0;
    const auto dispatch = [&](NavigationCommandKind, const CommandPayload &) {
      ++dispatches;
      if (dispatches == 1) {
        return CommandAck{false, entry.reason};
      }
      return CommandAck{true, "accepted_after_retry"};
    };

    const auto command = request("client-a", entry.request_id, entry.kind, entry.reason);
    const auto first = controller.handle(command, dispatch, now);
    const auto second = controller.handle(command, dispatch, now + 1s);
    require(first.dispatched && !first.replayed, "negative first attempt must dispatch");
    require(!first.ack.accepted && first.ack.reason == entry.reason,
            "negative first attempt returned the wrong ACK");
    require(second.dispatched && !second.replayed, "negative ACK retry must dispatch again");
    require(second.ack.accepted && second.ack.reason == "accepted_after_retry",
            "negative ACK retry must not replay the old failure");
    require(dispatches == 2, "negative ACK retry dispatch count mismatch");
  }
}

void testTypedEstopPositiveAckIsAcceptedEstopLatchedAndCached() {
  CommandIngressController controller;
  const auto now = CommandIngressController::Clock::time_point{50s};
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind kind, const CommandPayload &payload) {
    ++dispatches;
    require(kind == NavigationCommandKind::Estop, "typed estop dispatched the wrong kind");
    require(payload.reason == "operator_request", "typed estop lost its audit reason payload");
    return CommandAck{true, "estop_latched"};
  };

  const auto command =
      request("operator", "estop-1", NavigationCommandKind::Estop, "operator_request");
  const auto first = controller.handle(command, dispatch, now);
  const auto replay = controller.handle(command, dispatch, now + 1s);

  require(first.dispatched && !first.replayed, "first typed estop must dispatch");
  require(first.ack.accepted && first.ack.reason == "estop_latched",
          "typed estop success ACK must be accepted/estop_latched");
  require(replay.replayed && !replay.dispatched,
          "typed estop positive ACK must replay for exact retries");
  require(replay.ack.accepted && replay.ack.reason == "estop_latched",
          "typed estop replay changed the cached ACK");
  require(dispatches == 1, "typed estop positive ACK dispatched more than once");
}

void testRemovedCommandVelocityKindFailsClosed() {
  CommandIngressController controller;
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind, const CommandPayload &) {
    ++dispatches;
    return CommandAck{true, "accepted"};
  };
  CommandIngressRequest removed{
      "legacy-client",
      "removed-command-velocity",
      "",
      3,
      {},
  };
  const auto result = controller.handle(removed, dispatch);
  require(!result.dispatched && !result.ack.accepted,
          "removed command velocity kind must never dispatch");
  require(result.ack.reason == "unknown_command_kind",
          "removed command velocity kind must fail as unknown");
  require(dispatches == 0, "removed command velocity kind reached the dispatcher");
}

void testTaskIdStableAcrossRequestRetries() {
  CommandIngressController controller;
  const auto now = CommandIngressController::Clock::time_point{90s};
  int dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind, const CommandPayload &) {
    ++dispatches;
    return CommandAck{true, "planning_started"};
  };

  auto first_request = request("client-a", "goal-attempt-1", NavigationCommandKind::Goal);
  first_request.task_id = "navigation-task-1";
  const auto first = controller.handle(first_request, dispatch, now);
  require(first.dispatched && !first.replayed, "first task attempt must dispatch");
  require(first.task_id == "navigation-task-1", "first ACK task id mismatch");
  require(first.request_id == "goal-attempt-1", "first ACK request id mismatch");

  auto retry_request = first_request;
  retry_request.request_id = "goal-attempt-2";
  const auto retry = controller.handle(retry_request, dispatch, now + 1s);
  require(retry.replayed && !retry.dispatched, "same task retry must replay admission");
  require(retry.task_id == "navigation-task-1", "retry ACK changed task id");
  require(retry.request_id == "goal-attempt-2", "retry ACK must echo current request id");
  require(dispatches == 1, "same task retry dispatched the planner twice");

  auto changed_goal = retry_request;
  changed_goal.request_id = "goal-attempt-3";
  changed_goal.payload.goal[0] = 3.0;
  const auto conflict = controller.handle(changed_goal, dispatch, now + 2s);
  require(!conflict.dispatched && conflict.ack.reason == "idempotency_conflict",
          "changed goal under one task id must conflict");

  auto forked_task = first_request;
  forked_task.task_id = "navigation-task-2";
  const auto fork_conflict = controller.handle(forked_task, dispatch, now + 3s);
  require(!fork_conflict.dispatched && fork_conflict.ack.reason == "idempotency_conflict",
          "one request id must not alias two task ids");
  require(dispatches == 1, "conflicting task identity dispatched work");
}
}  // namespace

int main() {
  try {
    testValidationOrderAndDiagnostics();
    testReplayConflictAndPartitions();
    testNegativeCommandAcksAreNotCached();
    testTypedEstopPositiveAckIsAcceptedEstopLatchedAndCached();
    testRemovedCommandVelocityKindFailsClosed();
    testTaskIdStableAcrossRequestRetries();
    testTaskPauseResumeCommandsAreDistinctUserActions();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_command_ingress_controller: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
