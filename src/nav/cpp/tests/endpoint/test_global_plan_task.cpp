#include <chrono>
#include <stdexcept>
#include <thread>

#include "plan/global_plan_task.hpp"

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::nav::plan::GlobalPlanResult
waitAndPlan(const lingtu::nav::plan::GlobalPlanRequest &request,
            const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check) {
  for (int i = 0; i < 50; ++i) {
    if (cancel_check && cancel_check()) {
      lingtu::nav::plan::GlobalPlanResult result;
      result.cancelled = true;
      return result;
    }
    std::this_thread::sleep_for(1ms);
  }
  lingtu::nav::plan::GlobalPlanResult result;
  result.ok = true;
  result.reached_goal = true;
  result.map_identity = {"field", 7, "sha256-a", "map"};
  result.path = {request.start, request.goal};
  return result;
}

std::optional<lingtu::nav::endpoint::GlobalPlanCompletion>
waitForResult(lingtu::nav::endpoint::GlobalPlanTask &task) {
  for (int i = 0; i < 100; ++i) {
    if (auto result = task.poll()) {
      return result;
    }
    std::this_thread::sleep_for(2ms);
  }
  return std::nullopt;
}

}  // namespace

int main() {
  using lingtu::nav::endpoint::GlobalPlanContext;
  using lingtu::nav::endpoint::GlobalPlanTask;

  bool rejected_missing_planner = false;
  try {
    GlobalPlanTask invalid({});
  } catch (const std::invalid_argument &) {
    rejected_missing_planner = true;
  }
  require(rejected_missing_planner, "missing planner implementation was accepted");

  GlobalPlanTask task(waitAndPlan);
  GlobalPlanContext first;
  first.request_id = "goal-1";
  first.goal_epoch = 11;
  first.frame_epoch = 4;
  first.request.start = {1.0, 2.0, 0.5};
  first.request.goal = {4.0, 5.0, 2.0};
  require(task.start(first), "first plan did not start");
  require(task.busy(), "task did not report busy after start");
  require(!task.start(first), "task accepted overlapping plan");
  const auto first_result = waitForResult(task);
  require(first_result.has_value(), "first plan did not complete");
  require(first_result->context.request_id == "goal-1", "plan context was not preserved");
  require(first_result->result.ok, "first plan failed");
  require(first_result->result.path.size() == 2, "first plan returned wrong path");
  require(!task.busy(), "task remained busy after polling completion");
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 4,
                                                       first_result->result.map_identity)
              .empty(),
          "current plan was classified as stale");
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 12, 4,
                                                       first_result->result.map_identity) ==
              "goal_superseded_during_planning",
          "superseded goal was not rejected");
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 5,
                                                       first_result->result.map_identity) ==
              "planning_frame_changed_during_planning",
          "changed planning frame was not rejected");
  auto changed_map = first_result->result.map_identity;
  changed_map.version += 1;
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 4, changed_map) ==
              "active_map_changed_during_planning",
          "changed active map was not rejected");
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 4, std::nullopt) ==
              "active_map_unavailable_after_planning",
          "missing active map was not rejected");

  GlobalPlanContext cancelled;
  cancelled.request_id = "goal-2";
  cancelled.request = first.request;
  require(task.start(cancelled), "cancelled plan did not start");
  task.cancel();
  for (int i = 0; i < 100 && task.busy(); ++i) {
    (void)task.poll();
    std::this_thread::sleep_for(1ms);
  }
  require(!task.busy(), "cancelled task did not stop");
  return 0;
}
