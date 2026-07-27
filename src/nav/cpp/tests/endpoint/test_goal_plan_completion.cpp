#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "plan/goal_plan_controller.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::GoalPlanActions;
using lingtu::nav::endpoint::GoalPlanAdmissionContext;
using lingtu::nav::endpoint::GoalPlanAdvanceContext;
using lingtu::nav::endpoint::GoalPlanAdvanceResult;
using lingtu::nav::endpoint::GoalPlanController;
using lingtu::nav::endpoint::GoalPlanInspectionDecision;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalPlanOrigin;
using lingtu::nav::endpoint::GoalPlanPathActivation;
using lingtu::nav::endpoint::GoalPlanRequest;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::GoalPlanTarget;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_goal_plan_completion: FAIL: %s\n", message);
    throw std::runtime_error(message);
  }
}

lingtu::nav::plan::GlobalPlanResult
successfulPlan(const lingtu::nav::plan::GlobalPlanRequest &request,
               const lingtu::nav::plan::GlobalPlanCancelCheck &) {
  lingtu::nav::plan::GlobalPlanResult result;
  result.ok = true;
  result.reached_goal = true;
  result.map_identity = {"field", 7, "sha256-a", "map"};
  result.path = {request.start, request.goal};
  return result;
}

struct Recorder {
  std::vector<GoalPlanStatus> statuses;
  std::vector<GoalPlanPathActivation> activations;
  GoalPlanMapIdentityResult current_map{
      lingtu::nav::plan::MapIdentity{"field", 7, "sha256-a", "map"}, {}};
  bool inspection_is_active{false};
  GoalPlanInspectionDecision inspection_decision;
  std::vector<std::string> inspection_failures;
  std::vector<std::string> inspection_pauses;

  GoalPlanActions actions() {
    GoalPlanActions result;
    result.preempt_rolling = [](const std::string &) { return true; };
    result.clear_external_inspection = [] {};
    result.current_map_identity = [this] { return current_map; };
    result.publish_status = [this](const GoalPlanStatus &status) { statuses.push_back(status); };
    result.inspection_active = [this] { return inspection_is_active; };
    result.inspection_leg_failed = [this](const std::string &reason, double) {
      inspection_failures.push_back(reason);
    };
    result.inspection_pause = [this](const std::string &reason) {
      inspection_pauses.push_back(reason);
    };
    result.inspection_plan_ready = [this](double) { return inspection_decision; };
    result.activate_path = [this](const GoalPlanPathActivation &activation) {
      activations.push_back(activation);
    };
    return result;
  }
};

GoalPlanAdmissionContext admissionContext() {
  GoalPlanAdmissionContext context;
  context.motion_allowed = true;
  context.autonomy_mode = true;
  context.planner_map_configured = true;
  context.map_position = nav_kernel::Vec3{1.0, 2.0, 0.5};
  context.odometry_ready = true;
  context.frame_epoch = 4U;
  return context;
}

GoalPlanRequest request() {
  GoalPlanRequest value;
  value.task_id = "navigation-task-stale-frame";
  value.request_id = "goal-stale-frame";
  value.origin = GoalPlanOrigin::kExternal;
  value.source_stamp_s = 10.0;
  value.target = GoalPlanTarget{
      nav_kernel::Vec3{4.0, 5.0, 0.5},
      0.25,
  };
  return value;
}

GoalPlanAdvanceResult waitForCompletion(GoalPlanController &controller,
                                        GoalPlanAdvanceContext context) {
  GoalPlanAdvanceResult result;
  for (int attempt = 0; attempt < 100 && !result.completion_consumed; ++attempt) {
    result = controller.advance(context);
    if (!result.completion_consumed) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return result;
}

}  // namespace

int main() {
  Recorder recorder;
  GoalPlanController controller(successfulPlan, recorder.actions());
  const auto admission = admissionContext();
  require(controller.submit(request(), admission).accepted,
          "stale-frame test could not start planning");

  const auto result = waitForCompletion(controller, GoalPlanAdvanceContext{
                                                        admission.frame_epoch + 1U,
                                                        false,
                                                        20.0,
                                                    });

  require(result.completion_consumed, "stale planner completion was not consumed");
  require(!result.path_activated, "stale planner completion activated a path");
  require(result.counted_failure, "stale planner completion did not count failure");
  require(result.record_frame_error, "stale planner completion lost frame error");
  require(recorder.activations.empty(), "stale planner completion installed a path");
  require(!result.terminal_after_stop.has_value(),
          "planning-only stale completion requested an unnecessary stop barrier");
  require(recorder.statuses.size() == 2U, "stale goal status count changed");
  require(recorder.statuses.back().state == NavigationGoalState::Cancelled,
          "frame-stale plan did not publish Cancelled");
  require(recorder.statuses.back().reason == "planning_frame_changed_during_planning",
          "frame-stale status reason changed");

  const auto snapshot = controller.snapshot();
  require(snapshot.goal_epoch == 2U, "stale completion did not invalidate goal epoch");
  require(snapshot.planning_request_id.empty(), "stale planning identity remained live");
  require(snapshot.active_request_id.empty(), "stale completion left an active goal");
  require(snapshot.diagnostics.reason == "planning_frame_changed_during_planning",
          "stale completion diagnostics reason changed");

  Recorder exception_recorder;
  exception_recorder.inspection_is_active = true;
  GoalPlanController exception_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) -> lingtu::nav::plan::GlobalPlanResult {
        throw std::runtime_error("planner exploded");
      },
      exception_recorder.actions());
  auto inspection_request = request();
  inspection_request.origin = GoalPlanOrigin::kInspection;
  require(exception_controller.submit(inspection_request, admission).accepted,
          "planner-exception test could not start planning");
  const auto exception_result = waitForCompletion(exception_controller, GoalPlanAdvanceContext{
                                                                            admission.frame_epoch,
                                                                            false,
                                                                            21.0,
                                                                        });
  require(exception_result.completion_consumed, "planner exception was not consumed");
  require(exception_result.counted_failure, "planner exception did not count failure");
  require(!exception_result.record_frame_error, "planner exception became frame error");
  require(!exception_result.path_activated, "planner exception activated a path");
  require(exception_recorder.statuses.size() == 2U &&
              exception_recorder.statuses.back().state == NavigationGoalState::Failed,
          "planner exception did not publish Failed");
  require(exception_recorder.statuses.back().reason == "global_planner_exception",
          "planner exception status reason changed");
  require(exception_recorder.inspection_failures.size() == 1U &&
              exception_recorder.inspection_failures.front() == "global_planner_exception",
          "planner exception did not fail the inspection leg");
  require(exception_controller.snapshot().planning_request_id.empty(),
          "planner exception left planning identity live");
  require(exception_controller.snapshot().diagnostics.reason == "global_planner_exception",
          "planner exception diagnostics reason changed");

  Recorder failure_recorder;
  GoalPlanController failure_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = false;
        result.reached_goal = false;
        result.map_identity = {"field", 7, "sha256-a", "map"};
        result.failure_reason = "octomap_no_path";
        result.path = {request.start};
        result.elapsed_ms = 12.5;
        return result;
      },
      failure_recorder.actions());
  require(failure_controller.submit(request(), admission).accepted,
          "planner-failure test could not start planning");
  const auto failure_result = waitForCompletion(failure_controller, GoalPlanAdvanceContext{
                                                                        admission.frame_epoch,
                                                                        false,
                                                                        22.0,
                                                                    });
  require(failure_result.completion_consumed, "planner failure was not consumed");
  require(failure_result.counted_failure, "planner failure did not count failure");
  require(!failure_result.path_activated, "planner failure activated a path");
  require(failure_recorder.statuses.size() == 2U &&
              failure_recorder.statuses.back().state == NavigationGoalState::Failed,
          "planner failure did not publish Failed");
  require(failure_recorder.statuses.back().reason == "octomap_no_path",
          "planner failure reason was not preserved");
  const auto failure_snapshot = failure_controller.snapshot();
  require(failure_snapshot.planning_request_id.empty(),
          "planner failure left planning identity live");
  require(failure_snapshot.diagnostics.reason == "octomap_no_path",
          "planner failure diagnostics reason changed");
  require(failure_snapshot.diagnostics.waypoints == 1U,
          "planner failure diagnostics lost returned path size");

  Recorder takeover_recorder;
  takeover_recorder.inspection_is_active = true;
  GoalPlanController takeover_controller(successfulPlan, takeover_recorder.actions());
  require(takeover_controller.submit(inspection_request, admission).accepted,
          "operator-takeover test could not start planning");
  const auto takeover_result = waitForCompletion(takeover_controller, GoalPlanAdvanceContext{
                                                                          admission.frame_epoch,
                                                                          true,
                                                                          23.0,
                                                                      });
  require(takeover_result.completion_consumed, "takeover plan was not consumed");
  require(!takeover_result.counted_failure, "takeover cancellation counted failure");
  require(!takeover_result.path_activated, "takeover plan activated a path");
  require(takeover_recorder.statuses.size() == 2U &&
              takeover_recorder.statuses.back().state == NavigationGoalState::Cancelled,
          "takeover plan did not publish Cancelled");
  require(takeover_recorder.statuses.back().reason == "operator_takeover_discarded_plan",
          "takeover cancellation reason changed");
  require(takeover_recorder.inspection_pauses.size() == 1U &&
              takeover_recorder.inspection_pauses.front() == "operator_takeover_discarded_plan",
          "takeover did not pause inspection");
  require(takeover_recorder.activations.empty(), "takeover installed a planner path");
  const auto takeover_snapshot = takeover_controller.snapshot();
  require(takeover_snapshot.planning_request_id.empty(), "takeover left planning identity live");
  require(!takeover_snapshot.diagnostics.accepted &&
              takeover_snapshot.diagnostics.reason == "operator_takeover_discarded_plan",
          "takeover diagnostics changed");

  Recorder empty_recorder;
  GoalPlanController empty_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "sha256-a", "map"};
        return result;
      },
      empty_recorder.actions());
  require(empty_controller.submit(request(), admission).accepted,
          "empty-path test could not start planning");
  const auto empty_result = waitForCompletion(empty_controller, GoalPlanAdvanceContext{
                                                                    admission.frame_epoch,
                                                                    false,
                                                                    24.0,
                                                                });
  require(empty_result.completion_consumed, "empty path was not consumed");
  require(empty_result.counted_failure, "empty path did not count failure");
  require(!empty_result.path_activated, "empty path activated navigation");
  require(empty_recorder.statuses.size() == 2U &&
              empty_recorder.statuses.back().state == NavigationGoalState::Failed,
          "empty path did not publish Failed");
  require(empty_recorder.statuses.back().reason == "empty_path",
          "empty path status reason changed");
  require(empty_controller.snapshot().planning_request_id.empty(),
          "empty path left planning identity live");
  require(empty_controller.snapshot().diagnostics.reason == "empty_path",
          "empty path diagnostics reason changed");

  Recorder inspection_recorder;
  inspection_recorder.inspection_is_active = true;
  inspection_recorder.inspection_decision.accepted = false;
  inspection_recorder.inspection_decision.reason = "inspection_plan_rejected";
  GoalPlanController inspection_controller(successfulPlan, inspection_recorder.actions());
  require(inspection_controller.submit(inspection_request, admission).accepted,
          "inspection-plan test could not start planning");
  const auto inspection_result = waitForCompletion(inspection_controller, GoalPlanAdvanceContext{
                                                                              admission.frame_epoch,
                                                                              false,
                                                                              25.0,
                                                                          });
  require(inspection_result.completion_consumed, "inspection rejection was not consumed");
  require(!inspection_result.counted_failure,
          "inspection policy rejection counted planner failure");
  require(inspection_result.inspection_status_changed,
          "inspection policy rejection did not request status refresh");
  require(!inspection_result.path_activated && inspection_recorder.activations.empty(),
          "inspection policy rejection activated a path");
  require(!inspection_result.terminal_after_stop.has_value(),
          "planning-only inspection rejection requested an unnecessary stop barrier");
  require(inspection_recorder.statuses.size() == 2U &&
              inspection_recorder.statuses.back().state == NavigationGoalState::Cancelled,
          "inspection policy rejection did not cancel planning lifecycle");
  require(inspection_recorder.statuses.back().reason == "inspection_plan_rejected",
          "inspection policy status reason changed");
  const auto inspection_snapshot = inspection_controller.snapshot();
  require(inspection_snapshot.goal_epoch == 2U,
          "inspection policy rejection did not invalidate goal epoch");
  require(inspection_snapshot.planning_request_id.empty(),
          "inspection policy rejection left planning identity live");
  require(!inspection_snapshot.diagnostics.accepted &&
              inspection_snapshot.diagnostics.reason == "inspection_plan_rejected",
          "inspection policy diagnostics changed");

  Recorder hold_recorder;
  GoalPlanController hold_controller(successfulPlan, hold_recorder.actions());
  require(hold_controller.submit(request(), admission).accepted,
          "hold-invalidation test could not start planning");
  hold_controller.invalidateForHold("input_epoch_changed");
  require(hold_recorder.statuses.size() == 1U &&
              hold_recorder.statuses.back().state == NavigationGoalState::Planning,
          "hold invalidation published a terminal state without stop evidence");
  const auto hold_snapshot = hold_controller.snapshot();
  require(hold_snapshot.goal_epoch == 2U, "hold invalidation did not invalidate the goal epoch");
  require(hold_snapshot.planning_request_id == request().request_id,
          "hold invalidation detached the externally visible planning identity");
  require(!hold_snapshot.diagnostics.accepted &&
              hold_snapshot.diagnostics.reason == "input_epoch_changed",
          "hold invalidation diagnostics changed");

  Recorder deferred_abort_recorder;
  GoalPlanController deferred_abort_controller(successfulPlan, deferred_abort_recorder.actions());
  require(deferred_abort_controller.submit(request(), admission).accepted,
          "deferred-abort test could not start planning");
  require(waitForCompletion(deferred_abort_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 25.0})
              .path_activated,
          "deferred-abort test could not activate the goal");
  require(deferred_abort_recorder.statuses.size() == 2U &&
              deferred_abort_recorder.statuses.back().state == NavigationGoalState::PathActive,
          "deferred-abort test did not reach active state");

  auto commit_deferred_abort =
      deferred_abort_controller.deferAbort("cancelled_after_stop_confirmation");
  require(deferred_abort_recorder.statuses.size() == 2U,
          "deferred abort published Cancelled before its stop-evidence commit");
  const auto deferred_abort_snapshot = deferred_abort_controller.snapshot();
  require(deferred_abort_snapshot.goal_epoch == 2U,
          "deferred abort did not invalidate the goal epoch");
  require(deferred_abort_snapshot.planning_request_id.empty() &&
              deferred_abort_snapshot.active_request_id == request().request_id,
          "deferred abort detached the active identity before stop evidence");
  require(!deferred_abort_snapshot.diagnostics.accepted &&
              deferred_abort_snapshot.diagnostics.reason == "cancelled_after_stop_confirmation",
          "deferred abort diagnostics changed");

  commit_deferred_abort();
  require(deferred_abort_recorder.statuses.size() == 3U &&
              deferred_abort_recorder.statuses.back().request_id == request().request_id &&
              deferred_abort_recorder.statuses.back().state == NavigationGoalState::Cancelled &&
              deferred_abort_recorder.statuses.back().reason == "cancelled_after_stop_confirmation",
          "deferred abort commit did not publish the cancelled terminal exactly once");
  require(deferred_abort_controller.snapshot().active_request_id.empty(),
          "deferred abort commit left the active identity live");
  commit_deferred_abort();
  require(deferred_abort_recorder.statuses.size() == 3U,
          "deferred abort commit was not idempotent");

  Recorder stale_failure_recorder;
  GoalPlanController stale_failure_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = false;
        result.reached_goal = false;
        result.failure_reason = "backend_failure_must_not_hide_stale_frame";
        result.path = {request.start};
        return result;
      },
      stale_failure_recorder.actions());
  require(stale_failure_controller.submit(request(), admission).accepted,
          "stale-failure precedence test could not start planning");
  const auto stale_failure_result =
      waitForCompletion(stale_failure_controller, GoalPlanAdvanceContext{
                                                      admission.frame_epoch + 1U,
                                                      false,
                                                      26.0,
                                                  });
  require(stale_failure_result.completion_consumed,
          "stale failed planner completion was not consumed");
  require(stale_failure_result.counted_failure,
          "stale failed planner completion did not count failure");
  require(stale_failure_recorder.statuses.size() == 2U &&
              stale_failure_recorder.statuses.back().state == NavigationGoalState::Cancelled,
          "stale frame did not take precedence over backend failure");
  require(stale_failure_controller.snapshot().diagnostics.reason ==
              "planning_frame_changed_during_planning",
          "stale failure precedence diagnostics changed");
  require(!stale_failure_result.terminal_after_stop.has_value(),
          "planning-only stale failure requested an unnecessary stop barrier");

  Recorder planner_identity_missing_recorder;
  GoalPlanController planner_identity_missing_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.path = {plan_request.start, plan_request.goal};
        if (plan_request.goal.x < 8.0) {
          result.map_identity = {"field", 7, "sha256-a", "map"};
        }
        return result;
      },
      planner_identity_missing_recorder.actions());
  auto active_a_request = request();
  active_a_request.request_id = "goal-active-a";
  require(planner_identity_missing_controller.submit(active_a_request, admission).accepted,
          "planner-identity-missing test could not start active A");
  require(waitForCompletion(planner_identity_missing_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 27.0})
              .path_activated,
          "planner-identity-missing test could not activate A");
  auto stale_b_request = request();
  stale_b_request.request_id = "goal-stale-b";
  stale_b_request.target = GoalPlanTarget{nav_kernel::Vec3{8.0, 5.0, 0.5}, 0.5};
  require(planner_identity_missing_controller.submit(stale_b_request, admission).accepted,
          "planner-identity-missing test could not start B");
  auto planner_identity_missing_result =
      waitForCompletion(planner_identity_missing_controller,
                        GoalPlanAdvanceContext{admission.frame_epoch, false, 28.0});
  require(planner_identity_missing_result.completion_consumed,
          "planner identity missing completion was not consumed");
  require(planner_identity_missing_result.counted_failure,
          "planner identity missing did not count failure");
  require(!planner_identity_missing_result.path_activated,
          "planner identity missing activated B path");
  require(planner_identity_missing_result.terminal_after_stop.has_value() &&
              planner_identity_missing_result.terminal_after_stop->reason ==
                  "planner_map_identity_missing",
          "planner identity missing did not request a terminal stop barrier");
  require(planner_identity_missing_recorder.statuses.size() == 4U &&
              planner_identity_missing_recorder.statuses[3].request_id == "goal-stale-b" &&
              planner_identity_missing_recorder.statuses[3].state == NavigationGoalState::Failed,
          "planner identity missing published active cancellation before stop evidence");
  planner_identity_missing_result.terminal_after_stop->commit();
  require(planner_identity_missing_recorder.statuses.size() == 5U &&
              planner_identity_missing_recorder.statuses[4].request_id == "goal-active-a" &&
              planner_identity_missing_recorder.statuses[4].state == NavigationGoalState::Cancelled,
          "planner identity missing did not commit active cancellation after stop evidence");
  require(planner_identity_missing_controller.snapshot().active_request_id.empty(),
          "planner identity missing left active A live");

  Recorder active_map_unavailable_recorder;
  GoalPlanController active_map_unavailable_controller(successfulPlan,
                                                       active_map_unavailable_recorder.actions());
  require(active_map_unavailable_controller.submit(active_a_request, admission).accepted,
          "active-map-unavailable test could not start active A");
  require(waitForCompletion(active_map_unavailable_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 29.0})
              .path_activated,
          "active-map-unavailable test could not activate A");
  require(active_map_unavailable_controller.submit(stale_b_request, admission).accepted,
          "active-map-unavailable test could not start B");
  active_map_unavailable_recorder.current_map.identity.reset();
  active_map_unavailable_recorder.current_map.reason = "active_map_lookup_failed";
  auto active_map_unavailable_result =
      waitForCompletion(active_map_unavailable_controller,
                        GoalPlanAdvanceContext{admission.frame_epoch, false, 30.0});
  require(active_map_unavailable_result.completion_consumed,
          "active map unavailable completion was not consumed");
  require(active_map_unavailable_result.map_identity_error == "active_map_lookup_failed",
          "active map unavailable did not surface map lookup reason");
  require(active_map_unavailable_result.terminal_after_stop.has_value() &&
              active_map_unavailable_result.terminal_after_stop->reason ==
                  "active_map_unavailable_after_planning",
          "active map unavailable did not request a terminal stop barrier");
  require(active_map_unavailable_recorder.statuses.size() == 4U &&
              active_map_unavailable_recorder.statuses[3].state == NavigationGoalState::Failed,
          "active map unavailable published cancellation before stop evidence");
  active_map_unavailable_result.terminal_after_stop->commit();
  require(active_map_unavailable_recorder.statuses.size() == 5U &&
              active_map_unavailable_recorder.statuses[4].state == NavigationGoalState::Cancelled,
          "active map unavailable did not commit cancellation after stop evidence");

  Recorder active_map_changed_recorder;
  GoalPlanController active_map_changed_controller(successfulPlan,
                                                   active_map_changed_recorder.actions());
  require(active_map_changed_controller.submit(active_a_request, admission).accepted,
          "active-map-changed test could not start active A");
  require(waitForCompletion(active_map_changed_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 31.0})
              .path_activated,
          "active-map-changed test could not activate A");
  require(active_map_changed_controller.submit(stale_b_request, admission).accepted,
          "active-map-changed test could not start B");
  active_map_changed_recorder.current_map.identity =
      lingtu::nav::plan::MapIdentity{"field", 8, "sha256-b", "map"};
  auto active_map_changed_result = waitForCompletion(
      active_map_changed_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 32.0});
  require(active_map_changed_result.completion_consumed,
          "active map changed completion was not consumed");
  require(active_map_changed_result.counted_failure, "active map changed did not count failure");
  require(active_map_changed_result.terminal_after_stop.has_value() &&
              active_map_changed_result.terminal_after_stop->reason ==
                  "active_map_changed_during_planning",
          "active map changed did not request a terminal stop barrier");
  require(active_map_changed_recorder.statuses.size() == 4U &&
              active_map_changed_recorder.statuses[3].request_id == "goal-stale-b" &&
              active_map_changed_recorder.statuses[3].state == NavigationGoalState::Failed,
          "active map changed published cancellation before stop evidence");
  active_map_changed_result.terminal_after_stop->commit();
  require(active_map_changed_recorder.statuses.size() == 5U &&
              active_map_changed_recorder.statuses[4].request_id == "goal-active-a" &&
              active_map_changed_recorder.statuses[4].state == NavigationGoalState::Cancelled,
          "active map changed did not commit cancellation after stop evidence");

  Recorder active_inspection_rejection_recorder;
  GoalPlanController active_inspection_rejection_controller(
      successfulPlan, active_inspection_rejection_recorder.actions());
  require(active_inspection_rejection_controller.submit(active_a_request, admission).accepted,
          "active-inspection-rejection test could not start active A");
  require(waitForCompletion(active_inspection_rejection_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 33.0})
              .path_activated,
          "active-inspection-rejection test could not activate A");
  active_inspection_rejection_recorder.inspection_is_active = true;
  active_inspection_rejection_recorder.inspection_decision.accepted = false;
  active_inspection_rejection_recorder.inspection_decision.reason = "inspection_plan_rejected";
  auto inspection_b_request = stale_b_request;
  inspection_b_request.origin = GoalPlanOrigin::kInspection;
  require(active_inspection_rejection_controller.submit(inspection_b_request, admission).accepted,
          "active-inspection-rejection test could not start B");
  auto active_inspection_rejection_result =
      waitForCompletion(active_inspection_rejection_controller,
                        GoalPlanAdvanceContext{admission.frame_epoch, false, 34.0});
  require(active_inspection_rejection_result.completion_consumed,
          "active inspection rejection completion was not consumed");
  require(active_inspection_rejection_result.terminal_after_stop.has_value() &&
              active_inspection_rejection_result.terminal_after_stop->reason ==
                  "inspection_plan_rejected",
          "active inspection rejection did not request a terminal stop barrier");
  require(active_inspection_rejection_recorder.statuses.size() == 4U &&
              active_inspection_rejection_recorder.statuses[3].request_id == "goal-stale-b" &&
              active_inspection_rejection_recorder.statuses[3].state ==
                  NavigationGoalState::Cancelled,
          "active inspection rejection published active cancellation before stop evidence");
  require(active_inspection_rejection_controller.snapshot().active_request_id == "goal-active-a",
          "active inspection rejection detached active A before stop evidence");
  active_inspection_rejection_result.terminal_after_stop->commit();
  require(active_inspection_rejection_recorder.statuses.size() == 5U &&
              active_inspection_rejection_recorder.statuses[4].request_id == "goal-active-a" &&
              active_inspection_rejection_recorder.statuses[4].state ==
                  NavigationGoalState::Cancelled,
          "active inspection rejection did not commit cancellation after stop evidence");

  Recorder far_single_point_recorder;
  GoalPlanController far_single_point_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "sha256-a", "map"};
        result.path = {plan_request.goal};
        return result;
      },
      far_single_point_recorder.actions());
  require(far_single_point_controller.submit(request(), admission).accepted,
          "far single-point test could not start planning");
  require(waitForCompletion(far_single_point_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 33.0})
              .path_activated,
          "far single-point path was not activated");
  require(far_single_point_recorder.activations.size() == 1U &&
              far_single_point_recorder.activations.front().path.size() == 2U,
          "far single-point path was not repaired to two points");
  require(far_single_point_recorder.activations.front().path.front().x ==
                  admission.map_position->x &&
              far_single_point_recorder.activations.front().path.back().x == 4.0,
          "far single-point path did not prepend current start");

  Recorder near_single_point_recorder;
  GoalPlanController near_single_point_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "sha256-a", "map"};
        result.path = {plan_request.start};
        return result;
      },
      near_single_point_recorder.actions());
  require(near_single_point_controller.submit(request(), admission).accepted,
          "near single-point test could not start planning");
  require(waitForCompletion(near_single_point_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 34.0})
              .path_activated,
          "near single-point path was not activated");
  require(near_single_point_recorder.activations.size() == 1U &&
              near_single_point_recorder.activations.front().path.size() == 2U,
          "near single-point path was not repaired to two points");
  require(near_single_point_recorder.activations.front().path.front().x ==
                  admission.map_position->x &&
              near_single_point_recorder.activations.front().path.back().x == 4.0,
          "near single-point path did not append requested goal");

  Recorder sequential_recorder;
  GoalPlanController sequential_controller(successfulPlan, sequential_recorder.actions());
  require(sequential_controller.submit(active_a_request, admission).accepted,
          "sequential-goal test could not start A");
  require(waitForCompletion(sequential_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 35.0})
              .path_activated,
          "sequential-goal test could not activate A");
  auto active_b_request = stale_b_request;
  active_b_request.request_id = "goal-active-b";
  require(sequential_controller.submit(active_b_request, admission).accepted,
          "sequential-goal test could not start B");
  require(waitForCompletion(sequential_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 36.0})
              .path_activated,
          "sequential-goal test could not activate B");
  require(sequential_recorder.statuses.size() == 5U &&
              sequential_recorder.statuses[0].request_id == "goal-active-a" &&
              sequential_recorder.statuses[0].state == NavigationGoalState::Planning &&
              sequential_recorder.statuses[1].request_id == "goal-active-a" &&
              sequential_recorder.statuses[1].state == NavigationGoalState::PathActive &&
              sequential_recorder.statuses[2].request_id == "goal-active-b" &&
              sequential_recorder.statuses[2].state == NavigationGoalState::Planning &&
              sequential_recorder.statuses[3].request_id == "goal-active-a" &&
              sequential_recorder.statuses[3].state == NavigationGoalState::Cancelled &&
              sequential_recorder.statuses[3].reason == "superseded_by_new_goal" &&
              sequential_recorder.statuses[4].request_id == "goal-active-b" &&
              sequential_recorder.statuses[4].state == NavigationGoalState::PathActive,
          "sequential goals did not publish A active, B planning, A superseded, B active");
  require(sequential_controller.snapshot().active_request_id == "goal-active-b",
          "sequential-goal test did not leave B active");

  Recorder active_hold_recorder;
  GoalPlanController active_hold_controller(successfulPlan, active_hold_recorder.actions());
  require(active_hold_controller.submit(active_a_request, admission).accepted,
          "active-hold test could not start A");
  require(waitForCompletion(active_hold_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 37.0})
              .path_activated,
          "active-hold test could not activate A");
  active_hold_controller.invalidateForHold("localization_epoch_changed");
  require(active_hold_recorder.statuses.size() == 2U &&
              active_hold_recorder.statuses.back().state == NavigationGoalState::PathActive,
          "active hold published a terminal state without stop evidence");
  require(active_hold_controller.snapshot().active_request_id == "goal-active-a",
          "active hold detached the externally visible active identity");

  Recorder deferred_terminal_recorder;
  GoalPlanController deferred_terminal_controller(successfulPlan,
                                                  deferred_terminal_recorder.actions());
  require(deferred_terminal_controller.submit(active_a_request, admission).accepted,
          "deferred-terminal test could not start A");
  require(waitForCompletion(deferred_terminal_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 37.5})
              .path_activated,
          "deferred-terminal test could not activate A");
  const auto deferred_terminal_before = deferred_terminal_controller.snapshot().diagnostics;
  auto commit_reached = deferred_terminal_controller.deferActiveTerminal(
      NavigationGoalState::Reached, "goal_reached_after_stop_confirmation");
  require(deferred_terminal_recorder.statuses.size() == 2U,
          "deferred active terminal published before its stop-evidence commit");
  const auto deferred_terminal_pending = deferred_terminal_controller.snapshot();
  require(deferred_terminal_pending.active_request_id == "goal-active-a",
          "deferred active terminal detached identity before stop evidence");
  require(deferred_terminal_pending.diagnostics.accepted == deferred_terminal_before.accepted &&
              deferred_terminal_pending.diagnostics.reached_goal ==
                  deferred_terminal_before.reached_goal &&
              deferred_terminal_pending.diagnostics.reason == deferred_terminal_before.reason,
          "deferred active terminal changed diagnostics before stop evidence");
  commit_reached();
  require(deferred_terminal_recorder.statuses.size() == 3U &&
              deferred_terminal_recorder.statuses.back().request_id == "goal-active-a" &&
              deferred_terminal_recorder.statuses.back().state == NavigationGoalState::Reached &&
              deferred_terminal_recorder.statuses.back().reason ==
                  "goal_reached_after_stop_confirmation",
          "deferred active terminal commit did not publish Reached");
  const auto deferred_terminal_committed = deferred_terminal_controller.snapshot();
  require(deferred_terminal_committed.diagnostics.accepted &&
              deferred_terminal_committed.diagnostics.reached_goal &&
              deferred_terminal_committed.diagnostics.reason ==
                  "goal_reached_after_stop_confirmation",
          "deferred active terminal did not commit reached diagnostics");
  require(deferred_terminal_committed.active_request_id.empty(),
          "deferred active terminal commit left active identity live");
  commit_reached();
  require(deferred_terminal_recorder.statuses.size() == 3U,
          "deferred active terminal commit was not idempotent");

  Recorder deferred_failed_recorder;
  GoalPlanController deferred_failed_controller(successfulPlan, deferred_failed_recorder.actions());
  require(deferred_failed_controller.submit(active_a_request, admission).accepted,
          "deferred-failed test could not start A");
  require(waitForCompletion(deferred_failed_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 37.75})
              .path_activated,
          "deferred-failed test could not activate A");
  require(deferred_failed_controller.submit(active_b_request, admission).accepted,
          "deferred-failed test could not start planning B");
  auto commit_failed = deferred_failed_controller.deferFailure("local_recovery_exhausted");
  require(deferred_failed_recorder.statuses.size() == 3U,
          "deferred Failed published before its stop-evidence commit");
  commit_failed();
  require(deferred_failed_recorder.statuses.size() == 5U &&
              deferred_failed_recorder.statuses[3].request_id == "goal-active-a" &&
              deferred_failed_recorder.statuses[3].state == NavigationGoalState::Failed &&
              deferred_failed_recorder.statuses[3].reason == "local_recovery_exhausted" &&
              deferred_failed_recorder.statuses[4].request_id == "goal-active-b" &&
              deferred_failed_recorder.statuses[4].state == NavigationGoalState::Cancelled &&
              deferred_failed_recorder.statuses[4].reason == "local_recovery_exhausted",
          "deferred failure did not publish one Failed active terminal and one planning cancel");
  const auto active_a_cancelled_count =
      std::count_if(deferred_failed_recorder.statuses.begin(),
                    deferred_failed_recorder.statuses.end(), [](const GoalPlanStatus &status) {
                      return status.request_id == "goal-active-a" &&
                             status.state == NavigationGoalState::Cancelled;
                    });
  require(active_a_cancelled_count == 0,
          "deferred failure published a second Cancelled terminal for active A");
  const auto deferred_failed_snapshot = deferred_failed_controller.snapshot();
  require(!deferred_failed_snapshot.diagnostics.accepted &&
              !deferred_failed_snapshot.diagnostics.reached_goal &&
              deferred_failed_snapshot.diagnostics.reason == "local_recovery_exhausted",
          "deferred Failed commit did not update failure diagnostics");

  return 0;
}
