#include <chrono>
#include <stdexcept>
#include <thread>

#include "runtime/goal/task.hpp"

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
  result.map_identity = {"field", 7, "map"};
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
  changed_map.content_epoch += 1;
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 4, changed_map) ==
              "active_map_changed_during_planning",
          "changed active map was not rejected");
  require(lingtu::nav::endpoint::globalPlanStaleReason(*first_result, 11, 4, std::nullopt) ==
              "active_map_unavailable_after_planning",
          "missing active map was not rejected");

  auto single_point = *first_result;
  single_point.context.start = {1.0, 2.0, 0.5};
  single_point.context.goal = {4.0, 5.0, 2.0};
  single_point.result.path = {
      {single_point.context.goal.x, single_point.context.goal.y, single_point.context.goal.z}};
  const auto path_with_start = lingtu::nav::endpoint::globalPlanPath(single_point);
  require(path_with_start.size() == 2U && path_with_start.front().x == 1.0,
          "single-point plan did not include its start");
  single_point.result.path = {
      {single_point.context.start.x, single_point.context.start.y, single_point.context.start.z}};
  const auto path_with_goal = lingtu::nav::endpoint::globalPlanPath(single_point);
  require(path_with_goal.size() == 2U && path_with_goal.back().x == 4.0,
          "start-only plan did not include its goal");

  auto overlay_completion = *first_result;
  auto &requested_overlay = overlay_completion.context.request.temporary_overlay;
  requested_overlay.revision = 17U;
  requested_overlay.frame_epoch = 4U;
  requested_overlay.obstacle_generation = 23U;
  requested_overlay.traversability_generation = 29U;
  requested_overlay.blocked_regions.push_back({{2.0, 2.0, 0.5}, 0.6, -0.2, 1.8});
  overlay_completion.result.overlay_revision = requested_overlay.revision;
  overlay_completion.result.overlay_frame_epoch = requested_overlay.frame_epoch;
  overlay_completion.result.overlay_obstacle_generation = requested_overlay.obstacle_generation;
  overlay_completion.result.overlay_traversability_generation =
      requested_overlay.traversability_generation;
  require(lingtu::nav::endpoint::globalPlanStaleReason(overlay_completion, 11, 4,
                                                       overlay_completion.result.map_identity)
              .empty(),
          "matching temporary-overlay identity was rejected");
  ++overlay_completion.result.overlay_obstacle_generation;
  require(lingtu::nav::endpoint::globalPlanStaleReason(overlay_completion, 11, 4,
                                                       overlay_completion.result.map_identity) ==
              "temporary_overlay_identity_mismatch",
          "mismatched temporary-overlay identity was accepted");

  auto unexpected_overlay = *first_result;
  unexpected_overlay.result.overlay_revision = 1U;
  require(lingtu::nav::endpoint::globalPlanStaleReason(unexpected_overlay, 11, 4,
                                                       unexpected_overlay.result.map_identity) ==
              "unexpected_temporary_overlay_identity",
          "planner result leaked an unrequested temporary-overlay identity");

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
