#include "inspection.hpp"
#include "store.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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
  route.map_version = 7;
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
  assert(store.PutStatus(status).ok);
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

  inspection::RunStatus status;
  status.state = inspection::RunState::kNavigating;
  status.run_id = "published-status";
  assert(store.PutStatus(status).ok);

  const auto status_target = root / ".inspection" / "run_status.json";
  const auto status_temporary_path = status_target.string() + ".tmp.test";
  {
    std::ofstream out(status_temporary_path, std::ios::binary | std::ios::trunc);
    out << "replacement-status";
  }
  const HANDLE status_temporary = LockFileForMove(status_temporary_path);
  assert(status_temporary != INVALID_HANDLE_VALUE);
  std::string status_error;
  const bool status_published = inspection::detail::AtomicPublishFile(
      status_temporary_path, status_target, &status_error);
  CloseHandle(status_temporary);
  assert(!status_published);
  assert(status_error.find("route_publish_failed:") == 0U);
  assert(store.StatusJson().find("\"run_id\":\"published-status\"") !=
         std::string::npos);

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
  assert(error == "route_map_version_mismatch");
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
