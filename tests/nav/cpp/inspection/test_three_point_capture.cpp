#include "inspection.hpp"

#include <cassert>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace inspection = lingtu::nav::inspection;

inspection::Route ThreeObservationPoints() {
  inspection::Route route;
  route.id = "three-observations-contract";
  route.map_id = "synthetic-contract-map";
  route.map_content_epoch = 7;
  route.revision = 1;
  route.failure_policy = inspection::FailurePolicy::kStop;
  // Synthetic test coordinates, never the licensed factory's real route.
  for (int index = 0; index < 3; ++index) {
    inspection::Point point;
    point.id = "observation-" + std::to_string(index);
    point.x_m = 2.0 * index;
    point.action = "capture:overview";
    route.points.push_back(point);
  }
  return route;
}

inspection::ActionRequest ReachAndSettle(inspection::Executor& executor, double now) {
  assert(executor.PendingGoal());
  assert(executor.OnPlanningStarted(now));
  assert(executor.OnPlanReady(now + 0.1));
  executor.OnGoalReached(now + 0.2);
  assert(!executor.PendingAction());
  // Goal arrival alone is insufficient; capture requires fresh stable samples.
  for (int sample = 0; sample < 3; ++sample) {
    const double stamp = now + 0.3 + sample * 0.25;
    assert(executor.OnArrivalSample({stamp, 0.01, 0.02}, stamp));
  }
  executor.Tick(now + 0.81);
  assert(executor.status().state == inspection::RunState::kActionPending);
  const auto request = executor.PendingAction();
  assert(request && request->action == "capture:overview");
  assert(executor.OnActionStarted(request->request_id, now + 0.9));
  assert(executor.status().state == inspection::RunState::kActionPending);
  return *request;
}

void TestThreeCapturesAndThreeCleanRestarts() {
  inspection::Executor executor;
  std::set<std::string> operation_ids;
  std::string previous_operation;
  for (int run = 0; run < 3; ++run) {
    const auto task = "contract-task-" + std::to_string(run);
    const double base = 100.0 * run;
    std::string error;
    assert(executor.Start(ThreeObservationPoints(), task, "start-" + task,
                          "synthetic-contract-map", 7, base, &error));
    for (std::size_t point = 0; point < 3; ++point) {
      const double now = base + 2.0 * point + 1.0;
      const auto request = ReachAndSettle(executor, now);
      assert(request.task_id == task && request.point_index == point);
      assert(operation_ids.insert(request.request_id).second);
      if (!previous_operation.empty()) {
        assert(!executor.OnActionResult(previous_operation, true,
                                        previous_operation, "", now + 0.95));
        assert(executor.status().point_index == point);
      }
      assert(executor.OnActionResult(request.request_id, true,
                                     request.request_id, "persisted", now + 1.0));
      previous_operation = request.request_id;
      if (point < 2) {
        assert(executor.status().state == inspection::RunState::kPlanning);
        assert(executor.status().point_index == point + 1);
      }
    }
    assert(executor.status().state == inspection::RunState::kSucceeded);
    assert(!executor.PendingGoal() && !executor.PendingAction());
    std::vector<std::size_t> completed;
    for (const auto& event : executor.TakeTaskEvents()) {
      assert(event.status.task_id == task);
      if (event.status.reason == "point_completed") {
        completed.push_back(event.status.point_index);
      }
    }
    assert((completed == std::vector<std::size_t>{0, 1, 2}));
  }
  assert(operation_ids.size() == 9);
}

void TestMiddleCaptureFailureDoesNotVisitThirdPoint() {
  inspection::Executor executor;
  std::string error;
  assert(executor.Start(ThreeObservationPoints(), "failed-capture-task", "request-start",
                        "synthetic-contract-map", 7, 0.0, &error));
  const auto first = ReachAndSettle(executor, 1.0);
  assert(executor.OnActionResult(first.request_id, true, first.request_id, "persisted", 2.0));
  const auto second = ReachAndSettle(executor, 3.0);
  assert(executor.OnActionResult(second.request_id, false, "", "sensor_stale", 4.0));
  assert(executor.status().state == inspection::RunState::kFailed);
  assert(executor.status().point_index == 1);
  assert(!executor.PendingGoal() && !executor.PendingAction());
  int completed = 0;
  for (const auto& event : executor.TakeTaskEvents()) {
    assert(event.status.point_index <= 1);
    if (event.status.reason == "point_completed") ++completed;
  }
  assert(completed == 1);
}

int main() {
  TestThreeCapturesAndThreeCleanRestarts();
  TestMiddleCaptureFailureDoesNotVisitThirdPoint();
  std::cout << "three-point inspection capture contracts passed\n";
}
