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
using lingtu::nav::endpoint::InspectionCommandCoordinator;
using lingtu::nav::endpoint::InspectionCommandRequest;
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

InspectionCommandRequest makeRequest(std::string request_id, CommandKind kind) {
  InspectionCommandRequest value;
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
  bool clear_ok{true};
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
    result.clear_motion = [&](const std::string &reason) {
      log.emplace_back("clear:" + reason);
      return clear_ok;
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
    require(executor.Start(makeRoute(), run_id, "map-a", 7, clock, &error),
            "fixture start must succeed");
  }
};

void requireAck(const InspectionCommandAck &ack, const std::string &request_id, CommandKind kind,
                bool accepted, const std::string &reason, const std::string &run_id) {
  require(ack.request_id == request_id, "ACK request id mismatch");
  require(ack.kind == kind, "ACK kind mismatch");
  require(ack.accepted == accepted, "ACK accepted mismatch");
  require(ack.reason == reason, "ACK reason mismatch");
  require(ack.run_id == run_id, "ACK run id mismatch");
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
  unknown.request_id = "unknown-1";
  unknown.raw_kind = 99;
  result = coordinator.handle(unknown);
  requireAck(result.ack, "unknown-1", static_cast<CommandKind>(99), false,
             "unknown_inspection_command", "");
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
             "start-1");
  require(!result.replayed && result.status_requested && h.executor.active(),
          "new start flags/state mismatch");
  require(h.log == std::vector<std::string>({"active_map", "route_source", "load:map-a:route-a",
                                             "now", "ack:inspection_route_accepted", "status"}),
          "start must retain legacy action ordering");

  h.log.clear();
  result = coordinator.handle(start);
  requireAck(result.ack, "start-1", CommandKind::kStart, true, "inspection_route_accepted",
             "start-1");
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
  requireAck(result.ack, "missing", CommandKind::kStart, false, "inspection_route_not_found", "");

  Harness revision;
  InspectionCommandCoordinator c4(revision.executor, revision.actions());
  start.request_id = "revision";
  start.route_revision = 4U;
  result = c4.handle(start);
  requireAck(result.ack, "revision", CommandKind::kStart, false,
             "inspection_route_revision_mismatch", "");
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
                       {"active_map", "ack:inspection_cancel_not_allowed", "status"}),
          "evicted request must execute as a new legal command");
}

void testPauseAndCancelZeroDowngrade() {
  Harness h;
  h.startDirect();
  h.clear_ok = false;
  InspectionCommandCoordinator coordinator(h.executor, h.actions());

  auto result = coordinator.handle(makeRequest("pause-1", CommandKind::kPause));
  requireAck(result.ack, "pause-1", CommandKind::kPause, false,
             "inspection_pause_zero_publish_failed", "existing-run");
  require(h.executor.status().state == RunState::kPaused,
          "zero failure must not roll back pause state");
  require(h.log == std::vector<std::string>({"active_map", "clear:inspection_paused",
                                             "ack:inspection_pause_zero_publish_failed", "status"}),
          "pause clear/ACK/status order mismatch");

  h.log.clear();
  result = coordinator.handle(makeRequest("cancel-1", CommandKind::kCancel));
  requireAck(result.ack, "cancel-1", CommandKind::kCancel, false,
             "inspection_cancel_zero_publish_failed", "existing-run");
  require(h.executor.status().state == RunState::kCancelled,
          "zero failure must not roll back cancel state");
  require(h.log ==
              std::vector<std::string>({"active_map", "clear:inspection_cancelled",
                                        "ack:inspection_cancel_zero_publish_failed", "status"}),
          "cancel clear/ACK/status order mismatch");
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
  invalid = complete;
  invalid.operator_takeover_latched = {};
  expectInvalid(h.executor, invalid);
  invalid = complete;
  invalid.clear_motion = {};
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
    testPauseAndCancelZeroDowngrade();
    testResumeTakeover();
    testAckFailureDoesNotRollbackAndRetryReplays();
    testConstructorValidation();
  } catch (const std::exception &ex) {
    std::cerr << "FAIL: " << ex.what() << '\n';
    return 1;
  }
  std::cout << "inspection command coordinator tests passed\n";
  return 0;
}
