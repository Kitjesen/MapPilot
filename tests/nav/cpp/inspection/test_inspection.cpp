#include "inspection.hpp"
#include "store.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace inspection = lingtu::nav::inspection;

#ifdef _WIN32
namespace lingtu::nav::inspection::detail {

bool AtomicPublishFile(
    const std::filesystem::path& temporary,
    const std::filesystem::path& target,
    std::string* error);

}  // namespace lingtu::nav::inspection::detail
#endif

inspection::Route MakeRoute() {
  inspection::Route route;
  route.id = "north_loop";
  route.name = "North loop";
  route.map_id = "factory";
  route.map_content_epoch = 7;
  route.revision = 3;
  route.loop_count = 2;
  route.failure_policy = inspection::FailurePolicy::kRetry;
  route.max_retries = 1;
  inspection::Point first;
  first.id = "camera_a";
  first.x_m = 1.0;
  first.y_m = 2.0;
  first.dwell_s = 1.5;
  inspection::Point second;
  second.id = "camera_b";
  second.x_m = 4.0;
  second.y_m = 5.0;
  route.points = {first, second};
  return route;
}

inspection::TaskEvent MakeTaskEvent(
    std::uint64_t sequence,
    inspection::RunState state = inspection::RunState::kPlanning) {
  inspection::TaskEvent event;
  event.sequence = sequence;
  event.timestamp_s = 100.0 + static_cast<double>(sequence);
  event.kind = inspection::TaskEventKind::kStateChanged;
  event.request_id = "request-start-42";
  event.status.state = state;
  event.status.task_id = "inspection-task-42";
  event.status.run_id = event.status.task_id;
  event.status.request_id = "request-start-42";
  event.status.map_id = "factory";
  event.status.map_content_epoch = 7;
  event.status.route_id = "north_loop";
  event.status.route_revision = 3U;
  event.status.point_index = 1U;
  event.status.point_count = 2U;
  event.status.loop_index = 1U;
  event.status.retry_count = 1U;
  event.status.point_id = "camera_b";
  event.status.action = "capture:overview";
  event.status.action_request_id = "inspection-task-42-action-1";
  event.status.evidence_id = "evidence-42";
  event.status.phase_started_at_s = 90.25;
  event.status.stable_since_s = 91.5;
  event.status.deadline_s = 120.75;
  event.status.reason = "line one\tline two\nquoted=\"yes\" path=\\safe";
  return event;
}

void AssertSameTaskEvent(
    const inspection::TaskEvent& actual,
    const inspection::TaskEvent& expected) {
  assert(actual.sequence == expected.sequence);
  assert(actual.timestamp_s == expected.timestamp_s);
  assert(actual.kind == expected.kind);
  assert(actual.request_id == expected.request_id);
  assert(actual.status.state == expected.status.state);
  assert(actual.status.task_id == expected.status.task_id);
  assert(actual.status.run_id == expected.status.run_id);
  assert(actual.status.request_id == expected.status.request_id);
  assert(actual.status.map_id == expected.status.map_id);
  assert(actual.status.map_content_epoch == expected.status.map_content_epoch);
  assert(actual.status.route_id == expected.status.route_id);
  assert(actual.status.route_revision == expected.status.route_revision);
  assert(actual.status.point_index == expected.status.point_index);
  assert(actual.status.point_count == expected.status.point_count);
  assert(actual.status.loop_index == expected.status.loop_index);
  assert(actual.status.retry_count == expected.status.retry_count);
  assert(actual.status.point_id == expected.status.point_id);
  assert(actual.status.action == expected.status.action);
  assert(actual.status.action_request_id == expected.status.action_request_id);
  assert(actual.status.evidence_id == expected.status.evidence_id);
  assert(actual.status.phase_started_at_s == expected.status.phase_started_at_s);
  assert(actual.status.stable_since_s == expected.status.stable_since_s);
  assert(actual.status.deadline_s == expected.status.deadline_s);
  assert(actual.status.reason == expected.status.reason);
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  assert(in);
  return std::string{
      std::istreambuf_iterator<char>{in},
      std::istreambuf_iterator<char>{}};
}

void PlanAndReach(inspection::Executor& executor, double now_s) {
  assert(executor.PendingGoal().has_value());
  assert(executor.OnPlanningStarted(now_s - 0.2));
  assert(executor.OnPlanReady(now_s - 0.1));
  assert(executor.status().state == inspection::RunState::kNavigating);
  executor.OnGoalReached(now_s);
  assert(executor.status().state == inspection::RunState::kSettling);
}

void FeedSettled(inspection::Executor& executor, double start_s) {
  assert(executor.OnArrivalSample({start_s, 0.01, 0.02}, start_s));
  assert(executor.OnArrivalSample({start_s + 0.25, 0.01, 0.02}, start_s + 0.25));
  assert(executor.status().state == inspection::RunState::kSettling);
  assert(executor.OnArrivalSample({start_s + 0.50, 0.01, 0.02}, start_s + 0.50));
  assert(executor.status().state == inspection::RunState::kDwelling);
}

void TestStore() {
  const auto root = std::filesystem::temp_directory_path() / "lingtu_inspection_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  std::filesystem::create_directories(root / "factory", ec);
  inspection::Store store(root);
  auto route = MakeRoute();
  route.revision = 1U;
  assert(store.Put(route).ok);
  assert(store.RoutePath(route.map_id, route.id) ==
         root / "routes" / route.map_id / "north_loop.ltroute");
  assert(!store.Put(route).ok);
  auto updated = route;
  updated.revision = route.revision + 1U;
  assert(store.Put(updated).ok);
  const auto loaded = store.Get("factory", "north_loop");
  assert(loaded.has_value());
  assert(loaded->revision == 2U);
  assert(loaded->points.size() == 2U);
  assert(loaded->points[0].dwell_s == 1.5);
  assert(store.List("factory").size() == 1U);
  inspection::RunStatus status;
  status.state = inspection::RunState::kNavigating;
  status.run_id = "run-1";
  status.route_id = route.id;
  status.point_count = 2U;
  status.point_id = "camera_a";
  status.action = "capture:overview";
  status.evidence_id = "evidence-1";
  status.phase_started_at_s = 12.5;
  status.stable_since_s = 12.75;
  status.deadline_s = 13.5;
  {
    std::ofstream out(root / "run_status.json", std::ios::binary | std::ios::trunc);
    out << inspection::RunStatusToJson(status) << "\n";
  }
  const auto status_json = store.StatusJson();
  assert(status_json.find("\"state\":\"navigating\"") != std::string::npos);
  assert(status_json.find("\"action\":\"capture:overview\"") != std::string::npos);
  assert(status_json.find("\"evidence_id\":\"evidence-1\"") != std::string::npos);
  assert(status_json.find("\"phase_started_at\":12.5") != std::string::npos);
  assert(status_json.find("\"stable_since\":12.75") != std::string::npos);
  assert(status_json.find("\"deadline\":13.5") != std::string::npos);
  assert(store.Delete("factory", "north_loop").ok);
  assert(!store.Get("factory", "north_loop").has_value());
  std::filesystem::remove_all(root, ec);
}

void TestConcurrentRouteRevisionHasSingleWinner() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_concurrent_revision_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  inspection::Store first_store(root);
  inspection::Store second_store(root);

  auto initial = MakeRoute();
  initial.revision = 1U;
  assert(first_store.Put(initial).ok);

  auto first_update = initial;
  first_update.revision = 2U;
  first_update.name = "first-writer";
  auto second_update = initial;
  second_update.revision = 2U;
  second_update.name = "second-writer";

  inspection::StoreResult first_result;
  inspection::StoreResult second_result;
  std::thread first([&] { first_result = first_store.Put(first_update); });
  std::thread second([&] { second_result = second_store.Put(second_update); });
  first.join();
  second.join();

  assert(first_result.ok != second_result.ok);
  const auto loaded = first_store.Get(initial.map_id, initial.id);
  assert(loaded.has_value());
  assert(loaded->revision == 2U);
  assert(loaded->name == "first-writer" || loaded->name == "second-writer");
  std::filesystem::remove_all(root, ec);
}

void TestTaskEventCheckpointPersistenceAndSequenceRules() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_checkpoint_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  inspection::Store store(root);

  assert(store.LoadTaskEventCheckpoint().state ==
         inspection::TaskEventCheckpointLoadState::kNotFound);
  const auto first = MakeTaskEvent(1U);
  assert(store.PutTaskEventCheckpoint(first).ok);
  const auto loaded_first = store.LoadTaskEventCheckpoint();
  assert(loaded_first.loaded());
  AssertSameTaskEvent(*loaded_first.event, first);
  assert(store.PutTaskEventCheckpoint(first).reason == "checkpoint_unchanged");
  const auto checkpoint_path = store.TaskEventCheckpointPath();
  const std::string first_bytes = ReadFile(checkpoint_path);

  auto nonfinite = first;
  nonfinite.status.deadline_s = std::numeric_limits<double>::infinity();
  assert(store.PutTaskEventCheckpoint(nonfinite).reason ==
         "checkpoint_invalid:event_status_time_invalid");
  auto embedded_null = first;
  embedded_null.status.reason.push_back('\0');
  assert(store.PutTaskEventCheckpoint(embedded_null).reason ==
         "checkpoint_invalid:event_reason_invalid");
  assert(ReadFile(checkpoint_path) == first_bytes);

  auto conflict = first;
  conflict.status.reason = "same sequence, different content";
  assert(store.PutTaskEventCheckpoint(conflict).reason ==
         "checkpoint_sequence_conflict");
  assert(ReadFile(checkpoint_path) == first_bytes);
  auto gap = MakeTaskEvent(3U, inspection::RunState::kNavigating);
  assert(store.PutTaskEventCheckpoint(gap).reason == "checkpoint_sequence_gap");
  assert(ReadFile(checkpoint_path) == first_bytes);

  auto second = MakeTaskEvent(2U, inspection::RunState::kNavigating);
  second.status.reason = "next event";
  assert(store.PutTaskEventCheckpoint(second).ok);
  assert(ReadFile(checkpoint_path) != first_bytes);
  assert(store.PutTaskEventCheckpoint(first).reason ==
         "checkpoint_sequence_regression");
  const auto loaded_second = store.LoadTaskEventCheckpoint();
  assert(loaded_second.loaded());
  AssertSameTaskEvent(*loaded_second.event, second);

  assert(std::filesystem::is_regular_file(checkpoint_path));
  for (const auto& entry : std::filesystem::directory_iterator(checkpoint_path.parent_path())) {
    assert(entry.path().filename().string().find("task_event_checkpoint.v1.tmp.") != 0U);
  }
  std::filesystem::remove_all(root, ec);
}

void TestTaskEventCheckpointCorruptionFailsClosed() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_checkpoint_corrupt_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  inspection::Store store(root);
  const auto first = MakeTaskEvent(1U);
  assert(store.PutTaskEventCheckpoint(first).ok);

  {
    std::fstream checkpoint(
        store.TaskEventCheckpointPath(),
        std::ios::binary | std::ios::in | std::ios::out);
    assert(checkpoint);
    checkpoint.seekp(8);
    checkpoint.put('X');
  }
  const auto corrupt = store.LoadTaskEventCheckpoint();
  assert(corrupt.state == inspection::TaskEventCheckpointLoadState::kCorrupt);
  assert(!store.PutTaskEventCheckpoint(MakeTaskEvent(2U)).ok);
  std::filesystem::remove_all(root, ec);
}

void TestCheckpointWriteFailureRetainsExecutorHead() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_checkpoint_write_failure";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  {
    std::ofstream file(root, std::ios::binary | std::ios::trunc);
    file << "not-a-directory";
  }
  inspection::Store store(root);
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(MakeRoute(), "task-write-failure", "request-write-failure",
                        "factory", 7, 10.0, &error));
  std::vector<std::uint64_t> outbox;
  inspection::StoreResult persist_result;
  assert(executor.FlushTaskEvents([&](const inspection::TaskEvent& event) {
           persist_result = store.PutTaskEventCheckpoint(event);
           if (!persist_result.ok) return false;
           outbox.push_back(event.sequence);
           return true;
         }) == 0U);
  assert(!persist_result.ok);
  assert(executor.pending_task_event_count() == 2U);
  assert(outbox.empty());
  std::filesystem::remove(root, ec);
}

void TestRestartReconciliationAndExecutorSequence() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_restart_checkpoint_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  inspection::Store store(root);
  inspection::Executor previous;
  std::string error;
  assert(previous.Start(MakeRoute(), "inspection-task-restart", "request-restart",
                        "factory", 7, 10.0, &error));
  assert(previous.FlushTaskEvents([&](const inspection::TaskEvent& event) {
           return store.PutTaskEventCheckpoint(event).ok;
         }) == 2U);

  const auto checkpoint = store.LoadTaskEventCheckpoint();
  assert(checkpoint.loaded());
  assert(checkpoint.event->status.state == inspection::RunState::kPlanning);
  const auto recovery =
      inspection::ReconcileTaskEventAfterRestart(*checkpoint.event, 200.0);
  assert(recovery.ok && recovery.synthesized_failure && !recovery.replayed_terminal);
  assert(recovery.event.sequence == checkpoint.event->sequence + 1U);
  assert(recovery.event.kind == inspection::TaskEventKind::kStateChanged);
  assert(recovery.event.status.state == inspection::RunState::kFailed);
  assert(recovery.event.status.reason == "native_endpoint_restarted");
  assert(recovery.event.status.action_request_id.empty());
  assert(store.PutTaskEventCheckpoint(recovery.event).ok);

  inspection::Executor fresh;
  assert(fresh.SetNextTaskEventSequence(recovery.event.sequence + 1U));
  assert(!fresh.active());
  assert(fresh.route() == nullptr);
  assert(!fresh.SetNextTaskEventSequence(0U));
  assert(!fresh.SetNextTaskEventSequence(recovery.event.sequence));

  const auto terminal = store.LoadTaskEventCheckpoint();
  assert(terminal.loaded());
  const auto replay = inspection::ReconcileTaskEventAfterRestart(*terminal.event, 300.0);
  assert(replay.ok && replay.replayed_terminal && !replay.synthesized_failure);
  AssertSameTaskEvent(replay.event, *terminal.event);
  std::filesystem::remove_all(root, ec);
}

void TestTaskEventOutboxRecoveryCursorAndRetry() {
  bool writable = false;
  std::vector<inspection::TaskEventEnvelope> writes;
  inspection::InspectionTaskEventOutbox outbox(
      "boot-recovery-2",
      [&](const inspection::TaskEventEnvelope& event) {
        writes.push_back(event);
        return writable;
      });
  assert(outbox.InitializeNextSequence(7U));
  assert(!outbox.InitializeNextSequence(6U));
  assert(outbox.Record(MakeTaskEvent(6U)) ==
         inspection::TaskEventOutboxRecordResult::kOutOfOrder);
  const auto recovered = MakeTaskEvent(7U, inspection::RunState::kFailed);
  assert(outbox.Record(recovered) == inspection::TaskEventOutboxRecordResult::kAccepted);
  assert(outbox.Flush() == 0U);
  assert(outbox.diagnostics().pending == 1U);
  writable = true;
  assert(outbox.Flush() == 1U);
  assert(writes.size() == 2U);
  assert(writes[0].sequence == 7U && writes[1].sequence == 7U);
  assert(writes[1].boot_id == "boot-recovery-2");
  assert(outbox.Record(MakeTaskEvent(8U)) ==
         inspection::TaskEventOutboxRecordResult::kAccepted);
  assert(outbox.Record(MakeTaskEvent(10U)) ==
         inspection::TaskEventOutboxRecordResult::kOutOfOrder);
}

#ifdef _WIN32
HANDLE LockFileForMove(const std::filesystem::path& path) {
  return CreateFileW(
      path.c_str(),
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,
      nullptr);
}

void TestFailedReplacementPreservesPublishedFiles() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_atomic_publish_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  inspection::Store store(root);

  auto route = MakeRoute();
  route.revision = 1U;
  route.name = "published-route";
  assert(store.Put(route).ok);

  const auto route_target = store.RoutePath(route.map_id, route.id);
  const auto route_temporary_path = route_target.string() + ".tmp.test";
  {
    std::ofstream out(route_temporary_path, std::ios::binary | std::ios::trunc);
    out << "replacement-route";
  }
  const HANDLE route_temporary = LockFileForMove(route_temporary_path);
  assert(route_temporary != INVALID_HANDLE_VALUE);
  std::string route_error;
  const bool route_published = inspection::detail::AtomicPublishFile(
      route_temporary_path, route_target, &route_error);
  CloseHandle(route_temporary);
  assert(!route_published);
  assert(route_error.find("route_publish_failed:") == 0U);
  const auto preserved_route = store.Get(route.map_id, route.id);
  assert(preserved_route.has_value());
  assert(preserved_route->revision == 1U);
  assert(preserved_route->name == "published-route");

  std::filesystem::remove_all(root, ec);
}
#endif

void TestExecutor() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(MakeRoute(), "run-1", "factory", 7, 10.0, &error));
  assert(executor.status().state == inspection::RunState::kPlanning);
  assert(executor.PendingGoal()->id == "camera_a");
  PlanAndReach(executor, 20.0);
  assert(executor.status().deadline_s == 20.0 + inspection::Executor::kSettlingTimeoutS);
  FeedSettled(executor, 20.1);
  executor.Tick(21.5);
  assert(executor.status().state == inspection::RunState::kDwelling);
  executor.Tick(22.1);
  assert(executor.PendingGoal()->id == "camera_b");
  assert(executor.OnPlanningStarted(22.2));
  executor.OnLegFailed("blocked", 22.3);
  assert(executor.status().retry_count == 1U);
  assert(executor.PendingGoal()->id == "camera_b");
  PlanAndReach(executor, 30.0);
  FeedSettled(executor, 30.1);
  executor.Tick(30.6);
  assert(executor.PendingGoal()->id == "camera_a");
  assert(executor.status().loop_index == 1U);
  PlanAndReach(executor, 40.0);
  FeedSettled(executor, 40.1);
  executor.Tick(42.1);
  PlanAndReach(executor, 50.0);
  FeedSettled(executor, 50.1);
  executor.Tick(50.6);
  assert(executor.status().state == inspection::RunState::kSucceeded);
}

void TestRunStateCompatibility() {
  assert(static_cast<std::int32_t>(inspection::RunState::kCancelled) == 9);
  assert(static_cast<std::int32_t>(inspection::RunState::kSettling) == 10);
  assert(static_cast<std::int32_t>(inspection::RunState::kActionPending) == 11);
  assert(std::string(inspection::RunStateName(inspection::RunState::kSettling)) ==
         "settling");
  assert(std::string(inspection::RunStateName(inspection::RunState::kActionPending)) ==
         "action_pending");
}

void TestTaskIdentityAndOrderedLifecycleEvents() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(
      MakeRoute(),
      "inspection-task-42",
      "request-start-42",
      "factory",
      7,
      10.0,
      &error));

  const auto started = executor.TakeTaskEvents();
  assert(started.size() == 2U);
  assert(started[0].kind == inspection::TaskEventKind::kTaskAccepted);
  assert(started[0].sequence == 1U);
  assert(started[0].timestamp_s == 10.0);
  assert(started[0].request_id == "request-start-42");
  assert(started[0].status.task_id == "inspection-task-42");
  assert(started[0].status.run_id == "inspection-task-42");
  assert(started[0].status.request_id == "request-start-42");
  assert(started[0].status.map_id == "factory");
  assert(started[0].status.map_content_epoch == 7);
  assert(started[1].kind == inspection::TaskEventKind::kStateChanged);
  assert(started[1].sequence == 2U);
  assert(started[1].status.state == inspection::RunState::kPlanning);

  assert(executor.RequestPause("operator_hold", "request-pause-42", 11.0));
  executor.MarkStopConfirmationFailed("driver_stop_confirmation_timeout", 11.2);
  assert(executor.CommitPause(11.4));

  const auto paused = executor.TakeTaskEvents();
  assert(paused.size() == 3U);
  assert(paused[0].kind == inspection::TaskEventKind::kStateChanged);
  assert(paused[0].sequence == 3U);
  assert(paused[0].request_id == "request-pause-42");
  assert(paused[0].status.state == inspection::RunState::kPausing);
  assert(paused[1].kind == inspection::TaskEventKind::kStopConfirmationFailed);
  assert(paused[1].sequence == 4U);
  assert(paused[1].status.state == inspection::RunState::kPausing);
  assert(paused[1].status.reason == "driver_stop_confirmation_timeout");
  assert(paused[2].kind == inspection::TaskEventKind::kStateChanged);
  assert(paused[2].sequence == 5U);
  assert(paused[2].status.state == inspection::RunState::kPaused);
  assert(paused[2].status.reason == "operator_hold");

  assert(executor.Resume("factory", 7, 12.0, "request-resume-42"));
  const auto resumed = executor.TakeTaskEvents();
  assert(resumed.size() == 1U);
  assert(resumed[0].kind == inspection::TaskEventKind::kStateChanged);
  assert(resumed[0].sequence == 6U);
  assert(resumed[0].request_id == "request-resume-42");
  assert(resumed[0].status.state == inspection::RunState::kPlanning);

  assert(executor.RequestCancel("operator_cancel", "request-cancel-42", 13.0));
  assert(executor.CommitCancel(13.1));
  const auto cancelled = executor.TakeTaskEvents();
  assert(cancelled.size() == 2U);
  assert(cancelled[0].sequence == 7U);
  assert(cancelled[0].status.state == inspection::RunState::kCancelling);
  assert(cancelled[1].sequence == 8U);
  assert(cancelled[1].status.state == inspection::RunState::kCancelled);
  assert(cancelled[1].status.task_id == "inspection-task-42");
  assert(cancelled[1].status.request_id == "request-cancel-42");

  const auto json = inspection::TaskEventToJson(cancelled.back());
  assert(json.find("\"schema_version\":\"nav.inspection.task-event.v1\"") !=
         std::string::npos);
  assert(json.find("\"task_id\":\"inspection-task-42\"") != std::string::npos);
  assert(json.find("\"request_id\":\"request-cancel-42\"") != std::string::npos);
  assert(json.find("\"state\":\"cancelled\"") != std::string::npos);
}

void TestTaskEventDeliveryRetainsRejectedHead() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(
      MakeRoute(),
      "inspection-task-delivery",
      "request-start-delivery",
      "factory",
      7,
      10.0,
      &error));

  std::vector<std::uint64_t> attempted;
  assert(executor.FlushTaskEvents([&](const inspection::TaskEvent& event) {
           attempted.push_back(event.sequence);
           return false;
         }) == 0U);
  assert((attempted == std::vector<std::uint64_t>{1U}));

  std::vector<std::uint64_t> delivered;
  assert(executor.FlushTaskEvents([&](const inspection::TaskEvent& event) {
           delivered.push_back(event.sequence);
           return true;
         }) == 2U);
  assert((delivered == std::vector<std::uint64_t>{1U, 2U}));
  assert(executor.TakeTaskEvents().empty());
}

void TestActionValidation() {
  auto route = MakeRoute();
  route.points[0].action = "capture:bin_full";
  assert(inspection::ValidateRoute(route).ok);
  route.points[0].action = std::string(129U, 'a');
  assert(inspection::ValidateRoute(route).reason == "invalid_point_action");
  route.points[0].action = "capture\nunsafe";
  assert(inspection::ValidateRoute(route).reason == "invalid_point_action");
  route.points[0].action = "../../capture";
  assert(inspection::ValidateRoute(route).reason == "invalid_point_action");
}

void TestArrivalRequiresFreshMonotonicStableSamples() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(MakeRoute(), "run-arrival", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 10.0);

  assert(!executor.OnArrivalSample({100.0, 0.0, 0.0}, 10.0));
  assert(!executor.OnArrivalSample({9.0, 0.0, 0.0}, 10.0));
  assert(executor.status().stable_since_s == 0.0);
  assert(executor.OnArrivalSample({10.1, 0.01, 0.02}, 10.15));
  assert(!executor.OnArrivalSample({10.1, 0.01, 0.02}, 10.16));
  assert(executor.status().stable_since_s == 0.0);

  assert(executor.OnArrivalSample({10.2, 0.01, 0.02}, 10.25));
  assert(executor.OnArrivalSample({10.55, 0.01, 0.02}, 10.60));
  assert(executor.status().stable_since_s == 10.55);
  assert(executor.OnArrivalSample({10.65, 0.051, 0.02}, 10.70));
  assert(executor.status().stable_since_s == 0.0);
  assert(executor.OnArrivalSample({10.75, 0.01, 0.02}, 10.80));
  assert(executor.OnArrivalSample({11.00, 0.01, 0.02}, 11.05));
  assert(executor.status().state == inspection::RunState::kSettling);
  assert(executor.OnArrivalSample({11.25, 0.01, 0.10}, 11.30));
  assert(executor.status().state == inspection::RunState::kDwelling);
  assert(executor.status().stable_since_s == 10.75);
}

void TestActionHandshakeAndEvidence() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.points.resize(1);
  route.points[0].dwell_s = 0.25;
  route.points[0].action = "capture:bin_full";

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-action", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 1.0);
  FeedSettled(executor, 1.1);
  executor.Tick(1.84);
  assert(executor.status().state == inspection::RunState::kDwelling);
  executor.Tick(1.85);
  assert(executor.status().state == inspection::RunState::kActionPending);
  const auto request = executor.PendingAction();
  assert(request.has_value());
  assert(request->action == "capture:bin_full");
  assert(request->point_id == "camera_a");
  assert(!executor.OnActionStarted("wrong-request", 1.9));
  assert(!executor.OnActionResult(
      request->request_id, true, "evidence-1", "", 1.9));
  assert(executor.OnActionStarted(request->request_id, 1.9));
  assert(!executor.PendingAction().has_value());
  assert(
      executor.status().deadline_s ==
      1.9 + inspection::Executor::kActionTimeoutS);
  assert(!executor.OnActionResult("wrong-request", true, "evidence-1", "", 2.0));
  assert(executor.OnActionResult(
      request->request_id, true, "evidence-1", "", 2.0));
  assert(executor.status().state == inspection::RunState::kSucceeded);
  assert(executor.status().evidence_id == "evidence-1");
}

void TestInvalidEvidenceIdFailsActionImmediately() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.points.resize(1);
  route.points[0].dwell_s = 0.0;
  route.points[0].action = "capture:overview";
  route.failure_policy = inspection::FailurePolicy::kStop;

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-invalid-evidence", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 1.0);
  FeedSettled(executor, 1.1);
  executor.Tick(1.6);
  const auto request = executor.PendingAction();
  assert(request.has_value());
  assert(executor.OnActionStarted(request->request_id, 1.7));
  assert(executor.OnActionResult(request->request_id, true, "", "", 1.8));
  assert(executor.status().state == inspection::RunState::kFailed);
  assert(executor.status().reason == "invalid_evidence_id");
}

void TestPointFailurePoliciesAndTimeouts() {
  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.failure_policy = inspection::FailurePolicy::kRetry;
    route.max_retries = 1;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-settle-retry", "factory", 7, 0.0, &error));
    PlanAndReach(executor, 5.0);
    executor.Tick(5.0 + inspection::Executor::kSettlingTimeoutS);
    assert(executor.status().state == inspection::RunState::kPlanning);
    assert(executor.status().retry_count == 1U);
    assert(executor.PendingGoal()->id == "camera_a");
    PlanAndReach(executor, 20.0);
    executor.Tick(20.0 + inspection::Executor::kSettlingTimeoutS);
    assert(executor.status().state == inspection::RunState::kFailed);
    assert(executor.status().reason == "settling_timeout");
  }

  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.failure_policy = inspection::FailurePolicy::kSkip;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-settle-skip", "factory", 7, 0.0, &error));
    PlanAndReach(executor, 5.0);
    executor.Tick(5.0 + inspection::Executor::kSettlingTimeoutS);
    assert(executor.PendingGoal()->id == "camera_b");
  }

  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.points.resize(1);
    route.points[0].dwell_s = 0.0;
    route.points[0].action = "capture:overview";
    route.failure_policy = inspection::FailurePolicy::kStop;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-action-timeout", "factory", 7, 0.0, &error));
    PlanAndReach(executor, 5.0);
    FeedSettled(executor, 5.1);
    executor.Tick(5.6);
    assert(executor.status().state == inspection::RunState::kActionPending);
    executor.Tick(5.6 + inspection::Executor::kActionTimeoutS);
    assert(executor.status().state == inspection::RunState::kFailed);
    assert(executor.status().reason == "action_timeout");
  }
}

void TestPlanningTimeoutUsesRouteFailurePolicy() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.failure_policy = inspection::FailurePolicy::kRetry;
  route.max_retries = 1;

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-planning-timeout", "factory", 7, 10.0, &error));
  assert(executor.status().phase_started_at_s == 10.0);
  assert(
      executor.status().deadline_s ==
      10.0 + inspection::Executor::kPlanningTimeoutS);

  executor.Tick(10.0 + inspection::Executor::kPlanningTimeoutS);
  assert(executor.status().state == inspection::RunState::kPlanning);
  assert(executor.status().retry_count == 1U);
  assert(executor.status().reason == "planning_timeout");
  assert(
      executor.status().deadline_s ==
      10.0 + 2.0 * inspection::Executor::kPlanningTimeoutS);

  executor.Tick(10.0 + 2.0 * inspection::Executor::kPlanningTimeoutS);
  assert(executor.status().state == inspection::RunState::kFailed);
  assert(executor.status().reason == "planning_timeout");
}

void TestNavigationTimeoutStopsTheRoute() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.failure_policy = inspection::FailurePolicy::kStop;

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-navigation-timeout", "factory", 7, 0.0, &error));
  assert(executor.OnPlanningStarted(0.1));
  assert(executor.OnPlanReady(0.2));
  assert(executor.status().phase_started_at_s == 0.2);
  assert(
      executor.status().deadline_s ==
      0.2 + inspection::Executor::kNavigationTimeoutS);

  executor.Tick(0.2 + inspection::Executor::kNavigationTimeoutS);
  assert(executor.status().state == inspection::RunState::kFailed);
  assert(executor.status().reason == "navigation_stalled");
  assert(executor.status().deadline_s == 0.0);
}

void TestActionFailureUsesRoutePolicy() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.points[0].dwell_s = 0.0;
  route.points[0].action = "capture:overview";
  route.failure_policy = inspection::FailurePolicy::kSkip;

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-action-skip", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 1.0);
  FeedSettled(executor, 1.1);
  executor.Tick(1.6);
  const auto request = executor.PendingAction();
  assert(request.has_value());
  assert(executor.OnActionStarted(request->request_id, 1.7));
  assert(executor.OnActionResult(
      request->request_id, false, "", "camera_unavailable", 1.8));
  assert(executor.status().state == inspection::RunState::kPlanning);
  assert(executor.PendingGoal()->id == "camera_b");
}

void TestActionRequestIdsDoNotRepeatAcrossRuns() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.points.resize(1);
  route.points[0].dwell_s = 0.0;
  route.points[0].action = "capture:overview";

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-old", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 1.0);
  FeedSettled(executor, 1.1);
  executor.Tick(1.6);
  const auto old_request = executor.PendingAction();
  assert(old_request.has_value());
  assert(executor.Cancel("replace_run"));

  inspection::Executor restarted_executor;
  assert(restarted_executor.Start(route, "run-new", "factory", 7, 2.0, &error));
  PlanAndReach(restarted_executor, 3.0);
  FeedSettled(restarted_executor, 3.1);
  restarted_executor.Tick(3.6);
  const auto new_request = restarted_executor.PendingAction();
  assert(new_request.has_value());
  assert(new_request->request_id != old_request->request_id);
  assert(new_request->request_id.find("run-new") != std::string::npos);
  assert(!restarted_executor.OnActionStarted(old_request->request_id, 3.7));
}

void TestMapVersionGate() {
  inspection::Executor executor;
  std::string error;
  assert(!executor.Start(MakeRoute(), "run-2", "factory", 8, 0.0, &error));
  assert(error == "route_map_content_epoch_mismatch");
}

void TestPausedPlanCancellationDoesNotFailRun() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(MakeRoute(), "run-3", "factory", 7, 0.0, &error));
  assert(executor.OnPlanningStarted(0.1));
  assert(executor.Pause("operator_takeover"));
  executor.OnLegFailed("planner_cancelled", 0.2);
  assert(executor.status().state == inspection::RunState::kPaused);
  assert(executor.Resume("factory", 7, 0.3));
  assert(executor.PendingGoal()->id == "camera_a");
}

void TestPauseClearsPointPhaseAndReplansCurrentPoint() {
  auto route = MakeRoute();
  route.points[0].action = "capture:overview";
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-pause", "factory", 7, 0.0, &error));
  PlanAndReach(executor, 2.0);
  assert(executor.OnArrivalSample({2.1, 0.0, 0.0}, 2.15));
  assert(executor.status().stable_since_s == 2.1);
  assert(executor.Pause("operator_takeover"));
  assert(executor.status().state == inspection::RunState::kPaused);
  assert(executor.status().phase_started_at_s == 0.0);
  assert(executor.status().stable_since_s == 0.0);
  assert(executor.status().deadline_s == 0.0);
  assert(!executor.PendingAction().has_value());
  assert(executor.Resume("factory", 7, 3.0));
  assert(executor.status().state == inspection::RunState::kPlanning);
  assert(executor.PendingGoal()->id == "camera_a");
}


void TestNavigationFailureUsesRoutePolicy() {
  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.failure_policy = inspection::FailurePolicy::kRetry;
    route.max_retries = 1;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-nav-retry", "factory", 7, 0.0, &error));
    assert(executor.OnPlanningStarted(0.1));
    assert(executor.OnPlanReady(0.2));
    assert(executor.OnNavigationFailed("local_recovery_exhausted", 5.0));
    assert(executor.status().state == inspection::RunState::kPlanning);
    assert(executor.status().retry_count == 1U);
    assert(executor.PendingGoal()->id == "camera_a");
  }

  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.failure_policy = inspection::FailurePolicy::kSkip;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-nav-skip", "factory", 7, 0.0, &error));
    assert(executor.OnPlanningStarted(0.1));
    assert(executor.OnPlanReady(0.2));
    assert(executor.OnNavigationFailed("local_recovery_exhausted", 5.0));
    assert(executor.status().state == inspection::RunState::kPlanning);
    assert(executor.PendingGoal()->id == "camera_b");
  }

  {
    auto route = MakeRoute();
    route.loop_count = 1;
    route.failure_policy = inspection::FailurePolicy::kStop;
    inspection::Executor executor;
    std::string error;
    assert(executor.Start(route, "run-nav-stop", "factory", 7, 0.0, &error));
    assert(executor.OnPlanningStarted(0.1));
    assert(executor.OnPlanReady(0.2));
    assert(executor.OnNavigationFailed("local_recovery_exhausted", 5.0));
    assert(executor.status().state == inspection::RunState::kFailed);
    assert(executor.status().reason == "local_recovery_exhausted");
  }
}


void TestNavigationProgressExtendsDeadlineWithoutKillingLongLegs() {
  auto route = MakeRoute();
  route.loop_count = 1;
  route.failure_policy = inspection::FailurePolicy::kStop;

  inspection::Executor executor;
  std::string error;
  assert(executor.Start(route, "run-long-leg", "factory", 7, 0.0, &error));
  assert(executor.OnPlanningStarted(0.1));
  assert(executor.OnPlanReady(0.2));

  double now_s = 1.0;
  double remaining_m = 100.0;
  assert(executor.OnNavigationProgress(remaining_m, now_s));
  for (int step = 0; step < 20; ++step) {
    now_s += 20.0;
    remaining_m -= 0.2;
    assert(executor.OnNavigationProgress(remaining_m, now_s));
    executor.Tick(now_s);
    assert(executor.status().state == inspection::RunState::kNavigating);
  }
  assert(now_s > 300.0);
  assert(
      executor.status().deadline_s ==
      now_s + inspection::Executor::kNavigationTimeoutS);

  executor.Tick(executor.status().deadline_s);
  assert(executor.status().state == inspection::RunState::kFailed);
  assert(executor.status().reason == "navigation_stalled");

  inspection::Executor expired;
  assert(expired.Start(route, "run-expired-progress", "factory", 7, 0.0, &error));
  assert(expired.OnPlanningStarted(0.1));
  assert(expired.OnPlanReady(0.2));
  const double expired_deadline = expired.status().deadline_s;
  assert(!expired.OnNavigationProgress(10.0, expired_deadline));
  assert(expired.status().state == inspection::RunState::kFailed);
  assert(expired.status().reason == "navigation_stalled");
}

int main() {
  TestStore();
  TestConcurrentRouteRevisionHasSingleWinner();
  TestTaskEventCheckpointPersistenceAndSequenceRules();
  TestTaskEventCheckpointCorruptionFailsClosed();
  TestCheckpointWriteFailureRetainsExecutorHead();
  TestRestartReconciliationAndExecutorSequence();
  TestTaskEventOutboxRecoveryCursorAndRetry();
#ifdef _WIN32
  TestFailedReplacementPreservesPublishedFiles();
#endif
  TestExecutor();
  TestRunStateCompatibility();
  TestActionValidation();
  TestArrivalRequiresFreshMonotonicStableSamples();
  TestActionHandshakeAndEvidence();
  TestInvalidEvidenceIdFailsActionImmediately();
  TestPointFailurePoliciesAndTimeouts();
  TestTaskIdentityAndOrderedLifecycleEvents();
  TestTaskEventDeliveryRetainsRejectedHead();
  TestPlanningTimeoutUsesRouteFailurePolicy();
  TestNavigationTimeoutStopsTheRoute();
  TestNavigationProgressExtendsDeadlineWithoutKillingLongLegs();
  TestNavigationFailureUsesRoutePolicy();
  TestActionFailureUsesRoutePolicy();
  TestActionRequestIdsDoNotRepeatAcrossRuns();
  TestMapVersionGate();
  TestPausedPlanCancellationDoesNotFailRun();
  TestPauseClearsPointPhaseAndReplansCurrentPoint();
  std::cout << "inspection tests passed\n";
  return 0;
}
