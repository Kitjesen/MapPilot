#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "motion/command_ingress_controller.hpp"

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
  return {
      std::move(client),
      std::move(id),
      static_cast<std::int32_t>(kind),
      std::move(value),
  };
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
  require(rejected_replay.replayed && !rejected_replay.ack.accepted, "rejected ACK must replay");

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
  require(dispatches == 4, "dispatch count mismatch");
  const auto &diagnostics = controller.diagnostics();
  require(diagnostics.replayed == 2, "replay diagnostic mismatch");
  require(diagnostics.rejected == 3, "rejected diagnostic mismatch");
}

void testTeleopTtlAndBusinessIsolation() {
  CommandIngressController controller;
  const auto now = CommandIngressController::Clock::time_point{30s};
  int business_dispatches = 0;
  int teleop_dispatches = 0;
  const auto dispatch = [&](NavigationCommandKind kind, const CommandPayload &) {
    kind == NavigationCommandKind::Teleop ? ++teleop_dispatches : ++business_dispatches;
    return CommandAck{true, "accepted"};
  };
  const auto business = request("client", "business", NavigationCommandKind::Goal, "business");
  const auto teleop = request("client", "teleop", NavigationCommandKind::Teleop, "teleop");
  (void)controller.handle(business, dispatch, now);
  (void)controller.handle(teleop, dispatch, now);
  require(controller.handle(teleop, dispatch, now + 5s).replayed,
          "teleop ACK must live through TTL boundary");
  require(controller.handle(teleop, dispatch, now + 6s).dispatched,
          "teleop ACK must expire after five seconds");
  for (int index = 0; index < 80; ++index) {
    (void)controller.handle(request("client", "teleop-" + std::to_string(index),
                                    NavigationCommandKind::Teleop, std::to_string(index)),
                            dispatch, now + 7s);
  }
  require(controller.handle(business, dispatch, now + 8s).replayed,
          "teleop churn must not evict business ACK");
  require(business_dispatches == 1, "business command dispatched twice");
  require(teleop_dispatches == 82, "teleop dispatch count mismatch");
}
}  // namespace

int main() {
  try {
    testValidationOrderAndDiagnostics();
    testReplayConflictAndPartitions();
    testTeleopTtlAndBusinessIsolation();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_command_ingress_controller: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
