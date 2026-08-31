#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "runtime/goal/plan.hpp"

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
  result.map_identity = {"field", 7, "map"};
  result.path = {request.start, request.goal};
  return result;
}

struct Recorder {
  std::vector<GoalPlanStatus> statuses;
  std::vector<GoalPlanPathActivation> activations;
  GoalPlanMapIdentityResult current_map{
      lingtu::nav::plan::MapIdentity{"field", 7, "map"}, {}};
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

void waitUntilNotBusy(GoalPlanController &controller, GoalPlanAdvanceContext context) {
  for (int attempt = 0; attempt < 100 && controller.snapshot().busy; ++attempt) {
    (void)controller.advance(context);
    if (controller.snapshot().busy) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

std::size_t countStatus(const std::vector<GoalPlanStatus> &statuses, const std::string &request_id,
                        NavigationGoalState state, const std::string &reason = {}) {
  return static_cast<std::size_t>(
      std::count_if(statuses.begin(), statuses.end(), [&](const GoalPlanStatus &status) {
        return status.request_id == request_id && status.state == state &&
               (reason.empty() || status.reason == reason);
      }));
}

const GoalPlanStatus &lastStatus(const std::vector<GoalPlanStatus> &statuses) {
  require(!statuses.empty(), "expected at least one lifecycle status");
  return statuses.back();
}

bool sameStatus(const GoalPlanStatus &lhs, const GoalPlanStatus &rhs) {
  return lhs.task_id == rhs.task_id && lhs.request_id == rhs.request_id &&
         lhs.goal_epoch == rhs.goal_epoch && lhs.state == rhs.state && lhs.reason == rhs.reason &&
         lhs.project_to_navigation_state == rhs.project_to_navigation_state;
}

bool sameStatuses(const std::vector<GoalPlanStatus> &lhs, const std::vector<GoalPlanStatus> &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (!sameStatus(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
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
        result.map_identity = {"field", 7, "map"};
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
        result.map_identity = {"field", 7, "map"};
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
          result.map_identity = {"field", 7, "map"};
        }
        return result;
      },
      planner_identity_missing_recorder.actions());
  auto active_a_request = request();
  active_a_request.task_id = "navigation-task-a";
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
  require(!planner_identity_missing_result.terminal_after_stop.has_value(),
          "planner identity missing replacement requested an active stop barrier");
  require(planner_identity_missing_recorder.statuses.size() == 4U &&
              planner_identity_missing_recorder.statuses[3].request_id == "goal-stale-b" &&
              planner_identity_missing_recorder.statuses[3].state == NavigationGoalState::Failed,
          "planner identity missing did not fail B exactly once");
  require(planner_identity_missing_controller.snapshot().active_request_id == "goal-active-a" &&
              planner_identity_missing_recorder.activations.size() == 1U,
          "planner identity missing replacement changed active A");

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
  require(!active_map_unavailable_result.terminal_after_stop.has_value(),
          "active map unavailable replacement requested an active stop barrier");
  require(active_map_unavailable_recorder.statuses.size() == 4U &&
              active_map_unavailable_recorder.statuses[3].state == NavigationGoalState::Failed,
          "active map unavailable did not fail B exactly once");
  require(active_map_unavailable_controller.snapshot().active_request_id == "goal-active-a" &&
              active_map_unavailable_recorder.activations.size() == 1U,
          "active map unavailable replacement changed active A");

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
      lingtu::nav::plan::MapIdentity{"field", 8, "map"};
  auto active_map_changed_result = waitForCompletion(
      active_map_changed_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 32.0});
  require(active_map_changed_result.completion_consumed,
          "active map changed completion was not consumed");
  require(active_map_changed_result.counted_failure, "active map changed did not count failure");
  require(!active_map_changed_result.terminal_after_stop.has_value(),
          "active map changed replacement requested an active stop barrier");
  require(active_map_changed_recorder.statuses.size() == 4U &&
              active_map_changed_recorder.statuses[3].request_id == "goal-stale-b" &&
              active_map_changed_recorder.statuses[3].state == NavigationGoalState::Failed,
          "active map changed did not fail B exactly once");
  require(active_map_changed_controller.snapshot().active_request_id == "goal-active-a" &&
              active_map_changed_recorder.activations.size() == 1U,
          "active map changed replacement changed active A");

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
  require(!active_inspection_rejection_result.terminal_after_stop.has_value(),
          "active inspection replacement requested an active stop barrier");
  require(active_inspection_rejection_recorder.statuses.size() == 4U &&
              active_inspection_rejection_recorder.statuses[3].request_id == "goal-stale-b" &&
              active_inspection_rejection_recorder.statuses[3].state ==
                  NavigationGoalState::Cancelled,
          "active inspection rejection did not cancel B exactly once");
  require(active_inspection_rejection_controller.snapshot().active_request_id == "goal-active-a",
          "active inspection rejection detached active A");

  Recorder far_single_point_recorder;
  GoalPlanController far_single_point_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "map"};
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
        result.map_identity = {"field", 7, "map"};
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
  active_b_request.task_id = "navigation-task-b";
  active_b_request.request_id = "goal-active-b";
  auto active_c_request = stale_b_request;
  active_c_request.task_id = "navigation-task-c";
  active_c_request.request_id = "goal-active-c";
  active_c_request.target = GoalPlanTarget{nav_kernel::Vec3{6.0, 7.0, 0.5}, 0.5};
  require(sequential_controller.submit(active_b_request, admission).accepted,
          "sequential-goal test could not start B");
  const auto replacement_ready = waitForCompletion(
      sequential_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 36.0});
  require(replacement_ready.completion_consumed && !replacement_ready.path_activated,
          "sequential-goal test did not retain B behind A's terminal barrier");
  require(replacement_ready.terminal_after_stop.has_value() &&
              replacement_ready.terminal_after_stop->reason == "superseded_by_new_goal" &&
              replacement_ready.terminal_after_stop->delivery_ticket.statuses.size() == 1U &&
              replacement_ready.terminal_after_stop->delivery_ticket.statuses.front().task_id ==
                  active_a_request.task_id &&
              replacement_ready.terminal_after_stop->delivery_ticket.statuses.front().request_id ==
                  active_a_request.request_id &&
              replacement_ready.terminal_after_stop->delivery_ticket.statuses.front().state ==
                  NavigationGoalState::Cancelled,
          "sequential-goal test did not retain A's exact superseded terminal ticket");
  require(sequential_recorder.activations.size() == 1U &&
              sequential_controller.snapshot().active_request_id == active_a_request.request_id,
          "sequential-goal test activated B before A's exact terminal ACK");
  const auto before_exact_ack = sequential_controller.activateDeferredReplacement(36.1, admission);
  require(!before_exact_ack.path_activated && sequential_recorder.activations.size() == 1U,
          "sequential-goal test crossed the replacement barrier before terminal commit/ACK");

  replacement_ready.terminal_after_stop->commit();
  require(sequential_recorder.statuses.size() == 4U &&
              sequential_recorder.statuses.back().task_id == active_a_request.task_id &&
              sequential_recorder.statuses.back().request_id == active_a_request.request_id &&
              sequential_recorder.statuses.back().state == NavigationGoalState::Cancelled &&
              sequential_recorder.statuses.back().reason == "superseded_by_new_goal" &&
              sequential_controller.snapshot().active_request_id.empty(),
          "sequential-goal test did not commit A's exact superseded terminal");

  // The runtime coordinator invokes this seam only after exact terminal delivery is ACKed.
  const auto replacement_activated =
      sequential_controller.activateDeferredReplacement(36.2, admission);
  require(replacement_activated.path_activated,
          "sequential-goal test could not activate B after A's exact terminal ACK");
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
              sequential_recorder.statuses[4].state == NavigationGoalState::PathActive &&
              sequential_recorder.statuses[4].reason == "replacement_plan_completed",
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
  auto terminal_reached = deferred_terminal_controller.deferActiveTerminalWithTicket(
      NavigationGoalState::Reached, "goal_reached_after_stop_confirmation");
  require(terminal_reached.ticket.statuses.size() == 1U &&
              terminal_reached.ticket.statuses.front().request_id == "goal-active-a" &&
              terminal_reached.ticket.statuses.front().state == NavigationGoalState::Reached &&
              terminal_reached.ticket.statuses.front().reason ==
                  "goal_reached_after_stop_confirmation",
          "deferred active terminal ticket lost status identity");
  auto commit_reached = terminal_reached.commit;
  auto commit_reached_copy = commit_reached;
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
  const std::size_t reached_status_count_before = deferred_terminal_recorder.statuses.size();
  commit_reached();
  require(deferred_terminal_recorder.statuses.size() == 3U &&
              deferred_terminal_recorder.statuses.back().request_id == "goal-active-a" &&
              deferred_terminal_recorder.statuses.back().state == NavigationGoalState::Reached &&
              deferred_terminal_recorder.statuses.back().reason ==
                  "goal_reached_after_stop_confirmation",
          "deferred active terminal commit did not publish Reached");
  const std::vector<GoalPlanStatus> reached_published(
      deferred_terminal_recorder.statuses.begin() +
          static_cast<std::ptrdiff_t>(reached_status_count_before),
      deferred_terminal_recorder.statuses.end());
  require(sameStatuses(terminal_reached.ticket.statuses, reached_published),
          "deferred active terminal ticket differed from published status");
  const auto deferred_terminal_committed = deferred_terminal_controller.snapshot();
  require(deferred_terminal_committed.diagnostics.accepted &&
              deferred_terminal_committed.diagnostics.reached_goal &&
              deferred_terminal_committed.diagnostics.reason ==
                  "goal_reached_after_stop_confirmation",
          "deferred active terminal did not commit reached diagnostics");
  require(deferred_terminal_committed.active_request_id.empty(),
          "deferred active terminal commit left active identity live");
  commit_reached_copy();
  require(deferred_terminal_recorder.statuses.size() == 3U,
          "copied deferred active terminal commit was not exactly-once");

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
  auto terminal_failed =
      deferred_failed_controller.deferFailureWithTicket("local_recovery_exhausted");
  require(terminal_failed.ticket.statuses.size() == 2U &&
              terminal_failed.ticket.statuses[0].request_id == "goal-active-a" &&
              terminal_failed.ticket.statuses[0].state == NavigationGoalState::Failed &&
              terminal_failed.ticket.statuses[0].reason == "local_recovery_exhausted" &&
              terminal_failed.ticket.statuses[1].request_id == "goal-active-b" &&
              terminal_failed.ticket.statuses[1].state == NavigationGoalState::Cancelled &&
              terminal_failed.ticket.statuses[1].reason == "local_recovery_exhausted",
          "deferred failure ticket did not retain active+planning status vector");
  auto commit_failed = terminal_failed.commit;
  require(deferred_failed_recorder.statuses.size() == 3U,
          "deferred Failed published before its stop-evidence commit");
  const std::size_t failed_status_count_before = deferred_failed_recorder.statuses.size();
  commit_failed();
  require(deferred_failed_recorder.statuses.size() == 5U &&
              deferred_failed_recorder.statuses[3].request_id == "goal-active-a" &&
              deferred_failed_recorder.statuses[3].state == NavigationGoalState::Failed &&
              deferred_failed_recorder.statuses[3].reason == "local_recovery_exhausted" &&
              deferred_failed_recorder.statuses[4].request_id == "goal-active-b" &&
              deferred_failed_recorder.statuses[4].state == NavigationGoalState::Cancelled &&
              deferred_failed_recorder.statuses[4].reason == "local_recovery_exhausted",
          "deferred failure did not publish one Failed active terminal and one planning cancel");
  const std::vector<GoalPlanStatus> failed_published(
      deferred_failed_recorder.statuses.begin() +
          static_cast<std::ptrdiff_t>(failed_status_count_before),
      deferred_failed_recorder.statuses.end());
  require(sameStatuses(terminal_failed.ticket.statuses, failed_published),
          "deferred failure ticket differed from published multi-status vector");
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

  Recorder replan_success_recorder;
  GoalPlanController replan_success_controller(successfulPlan, replan_success_recorder.actions());
  require(replan_success_controller.submit(active_a_request, admission).accepted,
          "replan-success test could not start active A");
  require(waitForCompletion(replan_success_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 38.0})
              .path_activated,
          "replan-success test could not activate A");
  auto replan_context = admission;
  replan_context.map_position = nav_kernel::Vec3{2.0, 2.0, 0.5};
  const auto replan_success_submit = replan_success_controller.replanActive(replan_context);
  require(replan_success_submit.accepted, "valid active external replan was rejected");
  require(replan_success_submit.reason == "replan_started", "replan start reason changed");
  require(replan_success_controller.snapshot().replan_in_progress,
          "replan snapshot did not expose in-progress state");
  require(replan_success_controller.snapshot().replan_attempt == 1U,
          "replan snapshot did not expose attempt count");
  const auto replan_success_result = waitForCompletion(
      replan_success_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 39.0});
  require(replan_success_result.completion_consumed, "replan-success completion was not consumed");
  require(replan_success_result.path_activated, "replan-success did not activate replacement path");
  require(!replan_success_result.terminal_after_stop.has_value(),
          "replan-success requested an unnecessary stop barrier");
  require(replan_success_recorder.statuses.size() == 4U &&
              replan_success_recorder.statuses[2].request_id == "goal-active-a" &&
              replan_success_recorder.statuses[2].state == NavigationGoalState::Planning &&
              replan_success_recorder.statuses[2].reason == "replanning_after_local_recovery" &&
              replan_success_recorder.statuses[3].request_id == "goal-active-a" &&
              replan_success_recorder.statuses[3].state == NavigationGoalState::PathActive &&
              replan_success_recorder.statuses[3].reason == "path_replanned_after_local_recovery",
          "replan-success lifecycle statuses changed");
  const auto replan_cancelled_count =
      std::count_if(replan_success_recorder.statuses.begin(),
                    replan_success_recorder.statuses.end(), [](const GoalPlanStatus &status) {
                      return status.request_id == "goal-active-a" &&
                             status.state == NavigationGoalState::Cancelled;
                    });
  require(replan_cancelled_count == 0,
          "replan-success published an intermediate Cancelled for the active goal");
  require(replan_success_controller.snapshot().active_request_id == "goal-active-a",
          "replan-success did not keep the same active goal identity");
  require(!replan_success_controller.snapshot().replan_in_progress,
          "replan-success left replan marked in progress");

  Recorder replan_failure_recorder;
  int replan_failure_calls = 0;
  std::mutex replan_failure_mutex;
  GoalPlanController replan_failure_controller(
      [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
          const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        int call_index = 0;
        {
          std::lock_guard<std::mutex> lock(replan_failure_mutex);
          call_index = ++replan_failure_calls;
        }
        lingtu::nav::plan::GlobalPlanResult result;
        result.map_identity = {"field", 7, "map"};
        result.path = {plan_request.start, plan_request.goal};
        result.reached_goal = call_index == 1;
        result.ok = call_index == 1;
        if (call_index > 1) {
          result.failure_reason = "octomap_no_replan_path";
        }
        return result;
      },
      replan_failure_recorder.actions());
  require(replan_failure_controller.submit(active_a_request, admission).accepted,
          "replan-failure test could not start active A");
  require(waitForCompletion(replan_failure_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 40.0})
              .path_activated,
          "replan-failure test could not activate A");
  require(replan_failure_controller.replanActive(replan_context).accepted,
          "replan-failure test could not start replan");
  const auto replan_failure_result = waitForCompletion(
      replan_failure_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 41.0});
  require(replan_failure_result.completion_consumed, "replan-failure completion was not consumed");
  require(replan_failure_result.counted_failure, "replan-failure did not count failure");
  require(replan_failure_result.terminal_after_stop.has_value() &&
              replan_failure_result.terminal_after_stop->reason == "octomap_no_replan_path",
          "replan-failure did not request a stop-barrier terminal");
  require(replan_failure_recorder.statuses.size() == 3U &&
              replan_failure_recorder.statuses.back().state == NavigationGoalState::Planning,
          "replan-failure published terminal status before stop evidence");
  replan_failure_result.terminal_after_stop->commit();
  require(replan_failure_recorder.statuses.size() == 4U &&
              replan_failure_recorder.statuses.back().request_id == "goal-active-a" &&
              replan_failure_recorder.statuses.back().state == NavigationGoalState::Failed,
          "replan-failure stop-barrier commit did not fail active goal");

  Recorder replan_map_drift_recorder;
  GoalPlanController replan_map_drift_controller(successfulPlan,
                                                 replan_map_drift_recorder.actions());
  require(replan_map_drift_controller.submit(active_a_request, admission).accepted,
          "replan-map-drift test could not start active A");
  require(waitForCompletion(replan_map_drift_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 42.0})
              .path_activated,
          "replan-map-drift test could not activate A");
  require(replan_map_drift_controller.replanActive(replan_context).accepted,
          "replan-map-drift test could not start replan");
  replan_map_drift_recorder.current_map.identity =
      lingtu::nav::plan::MapIdentity{"field", 8, "map"};
  const auto replan_map_drift_result = waitForCompletion(
      replan_map_drift_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 43.0});
  require(replan_map_drift_result.completion_consumed,
          "replan-map-drift completion was not consumed");
  require(replan_map_drift_result.terminal_after_stop.has_value() &&
              replan_map_drift_result.terminal_after_stop->reason ==
                  "active_map_changed_during_planning",
          "replan-map-drift did not request stop-barrier terminal");
  require(replan_map_drift_recorder.statuses.size() == 3U,
          "replan-map-drift published terminal status before stop evidence");
  replan_map_drift_result.terminal_after_stop->commit();
  require(replan_map_drift_recorder.statuses.size() == 4U &&
              replan_map_drift_recorder.statuses.back().state == NavigationGoalState::Failed,
          "replan-map-drift commit did not fail active goal");

  Recorder replan_inspection_recorder;
  GoalPlanController replan_inspection_controller(successfulPlan,
                                                  replan_inspection_recorder.actions());
  auto inspection_active_request = active_a_request;
  inspection_active_request.origin = GoalPlanOrigin::kInspection;
  require(replan_inspection_controller.submit(inspection_active_request, admission).accepted,
          "replan-inspection test could not start inspection active goal");
  require(waitForCompletion(replan_inspection_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 44.0})
              .path_activated,
          "replan-inspection test could not activate inspection goal");
  const auto replan_inspection_result = replan_inspection_controller.replanActive(replan_context);
  require(!replan_inspection_result.accepted,
          "inspection-origin active goal accepted local-recovery replan");
  require(replan_inspection_result.reason == "inspection_replan_not_supported",
          "inspection replan rejection reason changed");

  Recorder replan_pause_recorder;
  GoalPlanController replan_pause_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
        if (plan_request.start.x == 2.0) {
          for (int i = 0; i < 200 && !cancelled(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "map"};
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      replan_pause_recorder.actions());
  require(replan_pause_controller.submit(active_a_request, admission).accepted,
          "replan-pause test could not start active A");
  require(waitForCompletion(replan_pause_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 44.25})
              .path_activated,
          "replan-pause test could not activate active A");
  require(replan_pause_controller.replanActive(replan_context).accepted,
          "replan-pause test could not start in-flight replan");
  const auto active_before_pause = replan_pause_controller.snapshot();
  const auto activation_count_before_pause = replan_pause_recorder.activations.size();
  const auto status_count_before_pause = replan_pause_recorder.statuses.size();

  auto pause_during_replan = replan_pause_controller.deferPause(
      active_a_request.task_id, "pause-during-replan", "operator_pause");
  require(pause_during_replan.accepted && pause_during_replan.reason == "pause_ready",
          "active task could not pause while its recovery replan was in flight");
  require(replan_pause_recorder.statuses.size() == status_count_before_pause,
          "pause during replan published PAUSED before stop confirmation");
  const auto pause_deferred_snapshot = replan_pause_controller.snapshot();
  require(!pause_deferred_snapshot.replan_in_progress &&
              pause_deferred_snapshot.active_task_id == active_before_pause.active_task_id &&
              pause_deferred_snapshot.active_request_id == active_before_pause.active_request_id &&
              pause_deferred_snapshot.active_map_identity.has_value(),
          "deferring pause did not isolate replan while retaining active task state");

  pause_during_replan.commit();
  require(replan_pause_recorder.statuses.size() == status_count_before_pause + 1U &&
              lastStatus(replan_pause_recorder.statuses).task_id == active_a_request.task_id &&
              lastStatus(replan_pause_recorder.statuses).request_id == "pause-during-replan" &&
              lastStatus(replan_pause_recorder.statuses).state == NavigationGoalState::Paused,
          "stop-confirmed pause did not publish PAUSED for the active task");
  waitUntilNotBusy(replan_pause_controller,
                   GoalPlanAdvanceContext{admission.frame_epoch, false, 44.5});
  const auto after_late_replan = replan_pause_controller.snapshot();
  require(after_late_replan.active_paused &&
              after_late_replan.active_task_id == active_a_request.task_id &&
              after_late_replan.active_map_identity.has_value() &&
              replan_pause_recorder.activations.size() == activation_count_before_pause &&
              lastStatus(replan_pause_recorder.statuses).state == NavigationGoalState::Paused,
          "late cancelled replan completion overwrote paused task state or path");

  Recorder replan_supersede_recorder;
  GoalPlanController replan_supersede_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
        if (plan_request.start.x == 2.0) {
          for (int i = 0; i < 200 && !cancelled(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "map"};
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      replan_supersede_recorder.actions());
  require(replan_supersede_controller.submit(active_a_request, admission).accepted,
          "replan-supersede test could not start active A");
  require(waitForCompletion(replan_supersede_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 45.0})
              .path_activated,
          "replan-supersede test could not activate A");
  require(replan_supersede_controller.replanActive(replan_context).accepted,
          "replan-supersede test could not start replan");
  const auto superseding_submit = replan_supersede_controller.submit(active_b_request, admission);
  require(superseding_submit.accepted, "external goal did not supersede busy replan");
  require(superseding_submit.reason == "planning_queued",
          "external supersede did not report queued planning");
  const auto queued_b_snapshot = replan_supersede_controller.snapshot();
  require(queued_b_snapshot.pending_plan_queued, "external supersede did not leave a pending plan");
  require(queued_b_snapshot.pending_task_id == active_b_request.task_id &&
              queued_b_snapshot.pending_request_id == "goal-active-b" &&
              queued_b_snapshot.pending_goal_epoch != 0U,
          "queued superseding goal did not expose pending identity");
  require(replan_supersede_recorder.statuses.size() == 4U &&
              lastStatus(replan_supersede_recorder.statuses).task_id == active_b_request.task_id &&
              lastStatus(replan_supersede_recorder.statuses).state ==
                  NavigationGoalState::Planning &&
              lastStatus(replan_supersede_recorder.statuses).reason == "planning_queued" &&
              !lastStatus(replan_supersede_recorder.statuses).project_to_navigation_state,
          "queued superseding goal was not immediately queryable without replacing active state");
  const auto resume_too_early = replan_supersede_controller.resumePending(admission);
  require(!resume_too_early.accepted && resume_too_early.reason == "pending_planner_not_drained",
          "pending resume did not wait for planner drain");
  waitUntilNotBusy(replan_supersede_controller,
                   GoalPlanAdvanceContext{admission.frame_epoch, false, 46.0});
  require(replan_supersede_controller.snapshot().pending_plan_queued,
          "planner drain lost the queued superseding goal");
  auto fresh_supersede_context = admission;
  fresh_supersede_context.frame_epoch = admission.frame_epoch + 1U;
  const auto resumed_supersede = replan_supersede_controller.resumePending(fresh_supersede_context);
  require(resumed_supersede.accepted, "fresh pending resume rejected superseding goal");
  require(resumed_supersede.reason == "planning_started",
          "fresh pending resume did not start normal planning");
  require(!replan_supersede_controller.snapshot().pending_plan_queued,
          "fresh pending resume left request queued");
  const auto active_while_replacement_plans = replan_supersede_controller.snapshot();
  require(active_while_replacement_plans.active_task_id == active_a_request.task_id &&
              active_while_replacement_plans.active_request_id == "goal-active-a" &&
              replan_supersede_recorder.activations.size() == 1U,
          "replacement planning changed active task or path before activation");
  const auto duplicate_pending_resume =
      replan_supersede_controller.resumePending(fresh_supersede_context);
  require(!duplicate_pending_resume.accepted &&
              duplicate_pending_resume.reason == "no_pending_plan",
          "pending task was started more than once");

  const auto replan_supersede_result =
      waitForCompletion(replan_supersede_controller,
                        GoalPlanAdvanceContext{fresh_supersede_context.frame_epoch, false, 47.0});
  require(replan_supersede_result.completion_consumed && !replan_supersede_result.path_activated &&
              replan_supersede_result.terminal_after_stop.has_value(),
          "superseding goal did not wait behind A's terminal barrier after fresh resume");
  require(replan_supersede_controller.snapshot().active_request_id == "goal-active-a" &&
              replan_supersede_recorder.activations.size() == 1U,
          "superseding goal activated before A's exact terminal ACK");
  replan_supersede_result.terminal_after_stop->commit();
  const auto activated_supersede =
      replan_supersede_controller.activateDeferredReplacement(47.1, fresh_supersede_context);
  require(activated_supersede.path_activated,
          "superseding goal did not activate after A's exact terminal ACK");
  require(replan_supersede_controller.snapshot().active_request_id == "goal-active-b",
          "superseding goal did not become active");
  require(replan_supersede_controller.snapshot().replan_attempt == 0U,
          "new non-replan active goal inherited prior replan attempt count");
  const auto supersede_b_planning_count =
      std::count_if(replan_supersede_recorder.statuses.begin(),
                    replan_supersede_recorder.statuses.end(), [](const GoalPlanStatus &status) {
                      return status.request_id == "goal-active-b" &&
                             status.state == NavigationGoalState::Planning;
                    });
  const auto supersede_b_cancelled_count =
      std::count_if(replan_supersede_recorder.statuses.begin(),
                    replan_supersede_recorder.statuses.end(), [](const GoalPlanStatus &status) {
                      return status.request_id == "goal-active-b" &&
                             status.state == NavigationGoalState::Cancelled;
                    });
  require(supersede_b_planning_count == 2,
          "superseding goal did not publish queued and running Planning statuses");
  require(supersede_b_cancelled_count == 0, "superseding goal published a stale Cancelled status");
  require(replan_supersede_recorder.statuses.size() == 7U &&
              replan_supersede_recorder.statuses[3].request_id == "goal-active-b" &&
              replan_supersede_recorder.statuses[3].state == NavigationGoalState::Planning &&
              replan_supersede_recorder.statuses[3].reason == "planning_queued" &&
              !replan_supersede_recorder.statuses[3].project_to_navigation_state &&
              replan_supersede_recorder.statuses[4].request_id == "goal-active-b" &&
              replan_supersede_recorder.statuses[4].state == NavigationGoalState::Planning &&
              !replan_supersede_recorder.statuses[4].project_to_navigation_state &&
              replan_supersede_recorder.statuses[5].request_id == "goal-active-a" &&
              replan_supersede_recorder.statuses[5].state == NavigationGoalState::Cancelled &&
              replan_supersede_recorder.statuses[6].request_id == "goal-active-b" &&
              replan_supersede_recorder.statuses[6].state == NavigationGoalState::PathActive &&
              replan_supersede_recorder.statuses[6].project_to_navigation_state,
          "superseding goal lifecycle did not preserve queued, running, and activation order");
  auto replan_b_context = fresh_supersede_context;
  replan_b_context.map_position = nav_kernel::Vec3{3.0, 2.0, 0.5};
  require(replan_supersede_controller.replanActive(replan_b_context).accepted,
          "B active goal did not get its own replan budget");
  require(replan_supersede_controller.snapshot().replan_attempt == 1U,
          "B active goal did not start replan attempts from one");
  require(
      waitForCompletion(replan_supersede_controller,
                        GoalPlanAdvanceContext{fresh_supersede_context.frame_epoch, false, 48.0})
          .path_activated,
      "B active goal replan did not complete");

  Recorder replace_pending_recorder;
  GoalPlanController replace_pending_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
        if (plan_request.start.x == 2.0) {
          for (int i = 0; i < 200 && !cancelled(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "map"};
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      replace_pending_recorder.actions());
  require(replace_pending_controller.submit(active_a_request, admission).accepted,
          "replace-pending test could not start active A");
  require(waitForCompletion(replace_pending_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 48.5})
              .path_activated,
          "replace-pending test could not activate A");
  require(replace_pending_controller.replanActive(replan_context).accepted,
          "replace-pending test could not start replan");
  require(replace_pending_controller.submit(active_b_request, admission).accepted,
          "replace-pending test could not queue B");
  const auto replace_c = replace_pending_controller.submit(active_c_request, admission);
  require(replace_c.accepted && replace_c.reason == "planning_queued",
          "new external goal C did not replace pending B");
  const auto replace_snapshot = replace_pending_controller.snapshot();
  require(replace_snapshot.pending_plan_queued &&
              replace_snapshot.pending_request_id == "goal-active-c" &&
              replace_snapshot.pending_task_id == active_c_request.task_id &&
              replace_snapshot.pending_goal_epoch != 0U,
          "pending replacement did not expose C identity");
  require(countStatus(replace_pending_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled, "superseded_by_new_goal") == 1U,
          "pending B was not cancelled exactly once when C replaced it");
  require(countStatus(replace_pending_recorder.statuses, "goal-active-c",
                      NavigationGoalState::Planning, "planning_queued") == 1U &&
              !lastStatus(replace_pending_recorder.statuses).project_to_navigation_state,
          "replacement pending C was not immediately queryable as queued");
  auto slow_supersede_planner = [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
                                   const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
    if (plan_request.start.x == 2.0) {
      for (int i = 0; i < 200 && !cancelled(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    lingtu::nav::plan::GlobalPlanResult result;
    result.ok = true;
    result.reached_goal = true;
    result.map_identity = {"field", 7, "map"};
    result.path = {plan_request.start, plan_request.goal};
    return result;
  };
  auto queue_superseding_goal = [&](GoalPlanController &controller, double base_time) {
    require(controller.submit(active_a_request, admission).accepted,
            "pending-gate test could not start active A");
    require(waitForCompletion(controller,
                              GoalPlanAdvanceContext{admission.frame_epoch, false, base_time})
                .path_activated,
            "pending-gate test could not activate A");
    require(controller.replanActive(replan_context).accepted,
            "pending-gate test could not start replan");
    require(controller.submit(active_b_request, admission).accepted,
            "pending-gate test could not queue B");
    waitUntilNotBusy(controller,
                     GoalPlanAdvanceContext{admission.frame_epoch, false, base_time + 1.0});
    require(controller.snapshot().pending_plan_queued,
            "pending-gate test lost queued B after drain");
  };
  auto require_active_a_unchanged = [&](const GoalPlanController &controller,
                                        const Recorder &recorder, const char *message) {
    const auto snapshot = controller.snapshot();
    require(snapshot.active_task_id == active_a_request.task_id &&
                snapshot.active_request_id == "goal-active-a" && !snapshot.active_paused &&
                snapshot.active_map_identity.has_value() && recorder.activations.size() == 1U,
            message);
  };
  auto require_no_active_terminal_request = [](const auto &result, const char *message) {
    require(!result.active_terminal_after_stop.has_value(), message);
  };

  auto direct_replacement_planner = [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
                                       const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
    if (plan_request.goal.x == 8.0) {
      for (int i = 0; i < 200 && !cancelled(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    lingtu::nav::plan::GlobalPlanResult result;
    result.ok = true;
    result.reached_goal = true;
    result.map_identity = {"field", 7, "map"};
    result.path = {plan_request.start, plan_request.goal};
    return result;
  };
  Recorder direct_replacement_cancel_recorder;
  GoalPlanController direct_replacement_cancel_controller(
      direct_replacement_planner, direct_replacement_cancel_recorder.actions());
  require(direct_replacement_cancel_controller.submit(active_a_request, admission).accepted,
          "direct replacement cancel test could not start active A");
  require(waitForCompletion(direct_replacement_cancel_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 48.55})
              .path_activated,
          "direct replacement cancel test could not activate A");
  const auto direct_replacement_submit =
      direct_replacement_cancel_controller.submit(active_b_request, admission);
  require(direct_replacement_submit.accepted &&
              direct_replacement_submit.reason == "planning_started",
          "direct replacement B was not admitted");
  const auto direct_replacement_snapshot = direct_replacement_cancel_controller.snapshot();
  require(direct_replacement_snapshot.replacement_plan_in_progress &&
              direct_replacement_snapshot.planning_task_id == active_b_request.task_id &&
              lastStatus(direct_replacement_cancel_recorder.statuses).task_id ==
                  active_b_request.task_id &&
              lastStatus(direct_replacement_cancel_recorder.statuses).state ==
                  NavigationGoalState::Planning &&
              !lastStatus(direct_replacement_cancel_recorder.statuses).project_to_navigation_state,
          "direct replacement B planning projected over active A");
  const auto direct_replacement_status_count = direct_replacement_cancel_recorder.statuses.size();
  auto cancel_direct_replacement =
      direct_replacement_cancel_controller.deferCancelReplacementPlanning(
          active_b_request.task_id, "cancel-direct-replacement-b", "operator_cancel");
  require(cancel_direct_replacement.accepted && cancel_direct_replacement.reason == "cancel_ready",
          "direct replacement B cancel was rejected");
  cancel_direct_replacement.commit();
  cancel_direct_replacement.commit();
  waitUntilNotBusy(direct_replacement_cancel_controller,
                   GoalPlanAdvanceContext{admission.frame_epoch, false, 48.6});
  require(direct_replacement_cancel_recorder.statuses.size() ==
                  direct_replacement_status_count + 1U &&
              lastStatus(direct_replacement_cancel_recorder.statuses).task_id ==
                  active_b_request.task_id &&
              lastStatus(direct_replacement_cancel_recorder.statuses).request_id ==
                  "cancel-direct-replacement-b" &&
              lastStatus(direct_replacement_cancel_recorder.statuses).state ==
                  NavigationGoalState::Cancelled &&
              !lastStatus(direct_replacement_cancel_recorder.statuses).project_to_navigation_state,
          "direct replacement cancel did not publish exactly one non-projecting B terminal");
  require_active_a_unchanged(direct_replacement_cancel_controller,
                             direct_replacement_cancel_recorder,
                             "direct replacement cancel or late completion changed active A");

  Recorder direct_replacement_failure_recorder;
  GoalPlanController direct_replacement_failure_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        lingtu::nav::plan::GlobalPlanResult result;
        result.map_identity = {"field", 7, "map"};
        if (plan_request.goal.x == 8.0) {
          result.failure_reason = "direct_replacement_no_path";
          return result;
        }
        result.ok = true;
        result.reached_goal = true;
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      direct_replacement_failure_recorder.actions());
  require(direct_replacement_failure_controller.submit(active_a_request, admission).accepted,
          "direct replacement failure test could not start active A");
  require(waitForCompletion(direct_replacement_failure_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 48.61})
              .path_activated,
          "direct replacement failure test could not activate A");
  require(direct_replacement_failure_controller.submit(active_b_request, admission).accepted,
          "direct replacement failure test could not start B");
  require(!lastStatus(direct_replacement_failure_recorder.statuses).project_to_navigation_state,
          "direct replacement B planning was projected before activation");
  const auto direct_replacement_failure =
      waitForCompletion(direct_replacement_failure_controller,
                        GoalPlanAdvanceContext{admission.frame_epoch, false, 48.62});
  require(direct_replacement_failure.completion_consumed &&
              !direct_replacement_failure.path_activated &&
              !direct_replacement_failure.terminal_after_stop.has_value() &&
              lastStatus(direct_replacement_failure_recorder.statuses).state ==
                  NavigationGoalState::Failed &&
              lastStatus(direct_replacement_failure_recorder.statuses).reason ==
                  "direct_replacement_no_path" &&
              !lastStatus(direct_replacement_failure_recorder.statuses).project_to_navigation_state,
          "direct replacement failure did not remain isolated from active A");
  require_active_a_unchanged(direct_replacement_failure_controller,
                             direct_replacement_failure_recorder,
                             "direct replacement failure changed active A");

  Recorder direct_replacement_stale_recorder;
  GoalPlanController direct_replacement_stale_controller(
      successfulPlan, direct_replacement_stale_recorder.actions());
  require(direct_replacement_stale_controller.submit(active_a_request, admission).accepted,
          "direct replacement stale test could not start active A");
  require(waitForCompletion(direct_replacement_stale_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 48.63})
              .path_activated,
          "direct replacement stale test could not activate A");
  require(direct_replacement_stale_controller.submit(active_b_request, admission).accepted,
          "direct replacement stale test could not start B");
  const auto direct_replacement_stale =
      waitForCompletion(direct_replacement_stale_controller,
                        GoalPlanAdvanceContext{admission.frame_epoch + 1U, false, 48.64});
  require(direct_replacement_stale.completion_consumed,
          "stale direct replacement completion was not consumed");
  require(!direct_replacement_stale.path_activated, "stale direct replacement activated B path");
  require(!direct_replacement_stale.terminal_after_stop.has_value(),
          "stale direct replacement requested an active stop barrier");
  require(lastStatus(direct_replacement_stale_recorder.statuses).task_id ==
              active_b_request.task_id,
          "stale direct replacement published the wrong task");
  require(lastStatus(direct_replacement_stale_recorder.statuses).state ==
              NavigationGoalState::Cancelled,
          "stale direct replacement did not cancel B");
  require(lastStatus(direct_replacement_stale_recorder.statuses).reason ==
              "planning_frame_changed_during_planning",
          "stale direct replacement changed the failure reason");
  require(!lastStatus(direct_replacement_stale_recorder.statuses).project_to_navigation_state,
          "stale direct replacement projected over active A");
  require_active_a_unchanged(direct_replacement_stale_controller, direct_replacement_stale_recorder,
                             "stale direct replacement changed active A");

  Recorder replacement_cancel_recorder;
  GoalPlanController replacement_cancel_controller(slow_supersede_planner,
                                                   replacement_cancel_recorder.actions());
  queue_superseding_goal(replacement_cancel_controller, 48.625);
  require(replacement_cancel_controller.resumePending(admission).accepted,
          "replacement-cancel test could not start queued B planning");
  require_active_a_unchanged(replacement_cancel_controller, replacement_cancel_recorder,
                             "replacement B planning changed active A before activation");
  const auto replacement_cancel_status_count = replacement_cancel_recorder.statuses.size();
  auto cancel_replacement = replacement_cancel_controller.deferCancelReplacementPlanning(
      active_b_request.task_id, "cancel-replacement-b", "operator_cancel");
  auto cancel_replacement_copy = cancel_replacement;
  require(cancel_replacement.accepted && cancel_replacement.reason == "cancel_ready",
          "task-specific cancel did not accept replacement B planning");
  require(replacement_cancel_recorder.statuses.size() == replacement_cancel_status_count,
          "replacement planning cancel published terminal before commit");
  require(replacement_cancel_controller.snapshot().planning_task_id.empty(),
          "replacement planning cancel left B planning identity live");
  require_active_a_unchanged(replacement_cancel_controller, replacement_cancel_recorder,
                             "deferring replacement cancel changed active A");
  cancel_replacement.commit();
  cancel_replacement_copy.commit();
  waitUntilNotBusy(replacement_cancel_controller,
                   GoalPlanAdvanceContext{admission.frame_epoch, false, 48.75});
  require(
      replacement_cancel_recorder.statuses.size() == replacement_cancel_status_count + 1U &&
          lastStatus(replacement_cancel_recorder.statuses).task_id == active_b_request.task_id &&
          lastStatus(replacement_cancel_recorder.statuses).request_id == "cancel-replacement-b" &&
          lastStatus(replacement_cancel_recorder.statuses).state ==
              NavigationGoalState::Cancelled &&
          !lastStatus(replacement_cancel_recorder.statuses).project_to_navigation_state,
      "replacement planning cancel did not publish one non-projecting B terminal");
  require_active_a_unchanged(replacement_cancel_controller, replacement_cancel_recorder,
                             "late replacement planner completion changed active A");

  Recorder replacement_failure_recorder;
  GoalPlanController replacement_failure_controller(
      [](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
         const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
        if (plan_request.start.x == 2.0) {
          for (int i = 0; i < 200 && !cancelled(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        lingtu::nav::plan::GlobalPlanResult result;
        result.map_identity = {"field", 7, "map"};
        if (plan_request.goal.x == 8.0) {
          result.failure_reason = "replacement_no_path";
          return result;
        }
        result.ok = true;
        result.reached_goal = true;
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      replacement_failure_recorder.actions());
  queue_superseding_goal(replacement_failure_controller, 48.8);
  require(replacement_failure_controller.resumePending(admission).accepted,
          "replacement-failure test could not start queued B planning");
  const auto replacement_failure_result = waitForCompletion(
      replacement_failure_controller, GoalPlanAdvanceContext{admission.frame_epoch, false, 48.9});
  require(replacement_failure_result.completion_consumed &&
              !replacement_failure_result.path_activated &&
              !replacement_failure_result.terminal_after_stop.has_value(),
          "replacement planning failure changed active motion lifecycle");
  require(countStatus(replacement_failure_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "replacement_no_path") == 1U &&
              !lastStatus(replacement_failure_recorder.statuses).project_to_navigation_state,
          "replacement planning failure projected over active A");
  require_active_a_unchanged(replacement_failure_controller, replacement_failure_recorder,
                             "replacement planning failure changed active A");

  Recorder pending_cancel_recorder;
  GoalPlanController pending_cancel_controller(slow_supersede_planner,
                                               pending_cancel_recorder.actions());
  queue_superseding_goal(pending_cancel_controller, 48.75);
  const auto pending_cancel_before = pending_cancel_controller.snapshot();
  const auto pending_cancel_status_count = pending_cancel_recorder.statuses.size();
  require(countStatus(pending_cancel_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Planning, "planning_queued") == 1U &&
              lastStatus(pending_cancel_recorder.statuses).task_id == active_b_request.task_id &&
              !lastStatus(pending_cancel_recorder.statuses).project_to_navigation_state &&
              pending_cancel_before.active_task_id == active_a_request.task_id,
          "accepted pending task was not immediately queryable without replacing active A");
  auto cancel_pending = pending_cancel_controller.deferCancelPending(
      active_b_request.task_id, "cancel-pending-b", "operator_cancel");
  require(cancel_pending.accepted && cancel_pending.reason == "cancel_ready",
          "task-specific cancel did not accept the queued task");
  const auto pending_cancel_deferred = pending_cancel_controller.snapshot();
  require(!pending_cancel_deferred.pending_plan_queued &&
              pending_cancel_deferred.active_task_id == pending_cancel_before.active_task_id &&
              pending_cancel_deferred.active_request_id ==
                  pending_cancel_before.active_request_id &&
              pending_cancel_deferred.active_map_identity.has_value(),
          "pending-only cancel changed the active task snapshot");
  require(pending_cancel_recorder.statuses.size() == pending_cancel_status_count,
          "pending-only cancel published terminal before commit");

  cancel_pending.commit();
  cancel_pending.commit();
  const auto pending_cancel_after = pending_cancel_controller.snapshot();
  require(pending_cancel_recorder.statuses.size() == pending_cancel_status_count + 1U &&
              lastStatus(pending_cancel_recorder.statuses).task_id == active_b_request.task_id &&
              lastStatus(pending_cancel_recorder.statuses).request_id == "cancel-pending-b" &&
              lastStatus(pending_cancel_recorder.statuses).state ==
                  NavigationGoalState::Cancelled &&
              lastStatus(pending_cancel_recorder.statuses).reason == "operator_cancel" &&
              !lastStatus(pending_cancel_recorder.statuses).project_to_navigation_state,
          "pending-only cancel did not publish one task-specific terminal");
  require(pending_cancel_after.active_task_id == pending_cancel_before.active_task_id &&
              pending_cancel_after.active_request_id == pending_cancel_before.active_request_id &&
              pending_cancel_recorder.activations.size() == 1U,
          "pending-only cancel terminated or replaced active A");
  const auto cancelled_pending_resume = pending_cancel_controller.resumePending(admission);
  require(!cancelled_pending_resume.accepted &&
              cancelled_pending_resume.reason == "no_pending_plan" &&
              pending_cancel_recorder.activations.size() == 1U,
          "cancelled pending task started after its terminal status");

  Recorder pending_takeover_recorder;
  GoalPlanController pending_takeover_controller(slow_supersede_planner,
                                                 pending_takeover_recorder.actions());
  queue_superseding_goal(pending_takeover_controller, 49.0);
  auto takeover_resume_context = admission;
  takeover_resume_context.operator_takeover_latched = true;
  const auto takeover_resume = pending_takeover_controller.resumePending(takeover_resume_context);
  require(!takeover_resume.accepted &&
              takeover_resume.reason == "operator_takeover_resume_required",
          "pending resume did not reject fresh operator takeover");
  require(!pending_takeover_controller.snapshot().pending_plan_queued,
          "operator-takeover pending rejection left stale request queued");
  require(pending_takeover_controller.snapshot().active_request_id == "goal-active-a",
          "operator-takeover pending rejection detached active A before stop commit");
  require_no_active_terminal_request(takeover_resume,
                                     "operator-takeover rejection tried to terminate active A");
  require_active_a_unchanged(pending_takeover_controller, pending_takeover_recorder,
                             "operator-takeover rejection changed active A");
  require(countStatus(pending_takeover_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled, "operator_takeover_resume_required") == 1U &&
              !lastStatus(pending_takeover_recorder.statuses).project_to_navigation_state,
          "operator-takeover rejection did not publish one non-projecting terminal for B");
  require(countStatus(pending_takeover_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Planning, "planning_queued") == 1U,
          "operator-takeover flow lost B's admitted queued status");

  Recorder pending_driver_recorder;
  GoalPlanController pending_driver_controller(slow_supersede_planner,
                                               pending_driver_recorder.actions());
  queue_superseding_goal(pending_driver_controller, 50.0);
  auto driver_resume_context = admission;
  driver_resume_context.driver_control_blocker = "driver_control_map_identity_stale";
  const auto driver_resume = pending_driver_controller.resumePending(driver_resume_context);
  require(!driver_resume.accepted && driver_resume.reason == "driver_control_map_identity_stale",
          "pending resume did not reject fresh driver-control blocker");
  require(!pending_driver_controller.snapshot().pending_plan_queued,
          "driver-control pending rejection left stale request queued");
  require(pending_driver_controller.snapshot().active_request_id == "goal-active-a",
          "driver-control pending rejection detached active A before stop commit");
  require_no_active_terminal_request(driver_resume,
                                     "driver-control rejection tried to terminate active A");
  require_active_a_unchanged(pending_driver_controller, pending_driver_recorder,
                             "driver-control rejection changed active A");
  require(countStatus(pending_driver_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled, "driver_control_map_identity_stale") == 1U,
          "driver-control pending rejection did not cancel B exactly once");
  require(countStatus(pending_driver_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Planning, "planning_queued") == 1U,
          "driver-control flow lost B's admitted queued status");
  Recorder pending_input_recorder;
  GoalPlanController pending_input_controller(slow_supersede_planner,
                                              pending_input_recorder.actions());
  queue_superseding_goal(pending_input_controller, 50.25);
  auto input_resume_context = admission;
  input_resume_context.input_ready = false;
  input_resume_context.input_gate_reason = "localization_stale";
  const auto input_resume = pending_input_controller.resumePending(input_resume_context);
  require(!input_resume.accepted && input_resume.reason == "input_gate_localization_stale",
          "pending resume did not reject fresh input gate failure");
  require(!pending_input_controller.snapshot().pending_plan_queued &&
              pending_input_controller.snapshot().active_task_id == active_a_request.task_id,
          "input-gate pending rejection changed active A before stop commit");
  require_no_active_terminal_request(input_resume,
                                     "input-gate rejection tried to terminate active A");
  require_active_a_unchanged(pending_input_controller, pending_input_recorder,
                             "input-gate rejection changed active A");
  require(countStatus(pending_input_recorder.statuses, "goal-active-b", NavigationGoalState::Failed,
                      "input_gate_localization_stale") == 1U &&
              !lastStatus(pending_input_recorder.statuses).project_to_navigation_state,
          "input-gate rejection did not publish one non-projecting terminal for queued B");

  Recorder pending_nonfinite_position_recorder;
  GoalPlanController pending_nonfinite_position_controller(
      slow_supersede_planner, pending_nonfinite_position_recorder.actions());
  queue_superseding_goal(pending_nonfinite_position_controller, 50.375);
  auto nonfinite_position_context = admission;
  nonfinite_position_context.map_position =
      nav_kernel::Vec3{std::numeric_limits<double>::quiet_NaN(), 2.0, 0.5};
  const auto nonfinite_position_resume =
      pending_nonfinite_position_controller.resumePending(nonfinite_position_context);
  require(!nonfinite_position_resume.accepted &&
              nonfinite_position_resume.reason == "invalid_replan_admission",
          "pending resume accepted a non-finite map position");
  require_no_active_terminal_request(nonfinite_position_resume,
                                     "non-finite position rejection tried to terminate active A");
  require(countStatus(pending_nonfinite_position_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "invalid_replan_admission") == 1U,
          "non-finite position rejection did not fail B exactly once");
  require_active_a_unchanged(pending_nonfinite_position_controller,
                             pending_nonfinite_position_recorder,
                             "non-finite position rejection changed active A");

  Recorder pending_nonfinite_barrier_recorder;
  GoalPlanController pending_nonfinite_barrier_controller(
      slow_supersede_planner, pending_nonfinite_barrier_recorder.actions());
  queue_superseding_goal(pending_nonfinite_barrier_controller, 50.4375);
  auto nonfinite_barrier_context = admission;
  nonfinite_barrier_context.autonomy_request_not_before_s =
      std::numeric_limits<double>::quiet_NaN();
  const auto nonfinite_barrier_resume =
      pending_nonfinite_barrier_controller.resumePending(nonfinite_barrier_context);
  require(!nonfinite_barrier_resume.accepted &&
              nonfinite_barrier_resume.reason == "invalid_replan_admission",
          "pending resume accepted a non-finite autonomy barrier");
  require_no_active_terminal_request(
      nonfinite_barrier_resume,
      "non-finite autonomy barrier rejection tried to terminate active A");
  require(countStatus(pending_nonfinite_barrier_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "invalid_replan_admission") == 1U,
          "non-finite autonomy barrier rejection did not fail B exactly once");
  require_active_a_unchanged(pending_nonfinite_barrier_controller,
                             pending_nonfinite_barrier_recorder,
                             "non-finite autonomy barrier rejection changed active A");

  Recorder pending_mapping_mode_recorder;
  GoalPlanController pending_mapping_mode_controller(slow_supersede_planner,
                                                     pending_mapping_mode_recorder.actions());
  queue_superseding_goal(pending_mapping_mode_controller, 50.5);
  auto mapping_mode_resume_context = admission;
  mapping_mode_resume_context.autonomy_mode = false;
  mapping_mode_resume_context.control_mode_name = "planner_map_debug";
  const auto mapping_mode_resume =
      pending_mapping_mode_controller.resumePending(mapping_mode_resume_context);
  require(!mapping_mode_resume.accepted &&
              mapping_mode_resume.reason == "goal_not_allowed_in_planner_map_debug",
          "pending resume did not reject planner-map-debug control mode");
  require_no_active_terminal_request(mapping_mode_resume,
                                     "planner-map-debug rejection tried to terminate active A");
  require_active_a_unchanged(pending_mapping_mode_controller, pending_mapping_mode_recorder,
                             "planner-map-debug rejection changed active A");
  require(countStatus(pending_mapping_mode_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled,
                      "goal_not_allowed_in_planner_map_debug") == 1U,
          "planner-map-debug control-mode pending rejection did not cancel B exactly once");
  require(countStatus(pending_mapping_mode_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "goal_not_allowed_in_planner_map_debug") == 0U,
          "planner-map-debug control-mode pending rejection failed B instead of cancelling it");
  Recorder pending_map_config_recorder;
  GoalPlanController pending_map_config_controller(slow_supersede_planner,
                                                   pending_map_config_recorder.actions());
  queue_superseding_goal(pending_map_config_controller, 51.0);
  auto map_config_resume_context = admission;
  map_config_resume_context.planner_map_configured = false;
  map_config_resume_context.planner_map_missing_reason = "active_octomap_not_configured";
  const auto map_config_resume =
      pending_map_config_controller.resumePending(map_config_resume_context);
  require(!map_config_resume.accepted &&
              map_config_resume.reason == "active_octomap_not_configured",
          "pending resume did not reject fresh map gate failure");
  require(!pending_map_config_controller.snapshot().pending_plan_queued,
          "map-config pending rejection left stale request queued");
  require(pending_map_config_controller.snapshot().active_request_id == "goal-active-a",
          "map-config pending rejection detached active A before stop commit");
  require_no_active_terminal_request(map_config_resume,
                                     "map-config rejection tried to terminate active A");
  require_active_a_unchanged(pending_map_config_controller, pending_map_config_recorder,
                             "map-config rejection changed active A");
  require(countStatus(pending_map_config_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "active_octomap_not_configured") == 1U &&
              !lastStatus(pending_map_config_recorder.statuses).project_to_navigation_state,
          "map-config rejection did not publish one non-projecting terminal for B");
  require(countStatus(pending_map_config_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Planning, "planning_queued") == 1U,
          "map-config flow lost B's admitted queued status");

  Recorder pending_identity_recorder;
  GoalPlanController pending_identity_controller(slow_supersede_planner,
                                                 pending_identity_recorder.actions());
  queue_superseding_goal(pending_identity_controller, 53.0);
  pending_identity_recorder.current_map.identity =
      lingtu::nav::plan::MapIdentity{"field", 8, "map"};
  const auto identity_resume = pending_identity_controller.resumePending(admission);
  require(!identity_resume.accepted &&
              identity_resume.reason == "active_map_changed_before_pending_plan",
          "pending resume did not reject active map identity drift");
  require(!pending_identity_controller.snapshot().pending_plan_queued,
          "identity-drift pending rejection left stale request queued");
  require(pending_identity_controller.snapshot().active_request_id == "goal-active-a",
          "identity-drift pending rejection detached active A before stop commit");
  require_no_active_terminal_request(identity_resume,
                                     "identity-drift rejection tried to terminate active A");
  require_active_a_unchanged(pending_identity_controller, pending_identity_recorder,
                             "identity-drift rejection changed active A");
  require(countStatus(pending_identity_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Failed, "active_map_changed_before_pending_plan") == 1U,
          "identity-drift pending rejection did not fail B exactly once");
  require(countStatus(pending_identity_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Planning, "planning_queued") == 1U,
          "identity-drift flow lost B's admitted queued status");

  Recorder pending_abort_recorder;
  GoalPlanController pending_abort_controller(slow_supersede_planner,
                                              pending_abort_recorder.actions());
  queue_superseding_goal(pending_abort_controller, 54.0);
  const auto pending_abort_status_count_before = pending_abort_recorder.statuses.size();
  auto pending_abort_commit = pending_abort_controller.deferAbort("shutdown_requested");
  require(!pending_abort_controller.snapshot().pending_plan_queued,
          "pending abort did not clear queued B");
  require(pending_abort_recorder.statuses.size() == pending_abort_status_count_before,
          "pending abort published terminal before deferred commit");
  pending_abort_commit();
  pending_abort_commit();
  require(countStatus(pending_abort_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled, "shutdown_requested") == 1U,
          "pending abort did not cancel B exactly once");
  require(countStatus(pending_abort_recorder.statuses, "goal-active-a",
                      NavigationGoalState::Cancelled, "shutdown_requested") == 1U,
          "pending abort did not cancel active A exactly once");
  require(pending_abort_controller.snapshot().active_request_id.empty(),
          "pending abort commit left active A live");
  Recorder pending_invalidate_recorder;
  GoalPlanController pending_invalidate_controller(slow_supersede_planner,
                                                   pending_invalidate_recorder.actions());
  queue_superseding_goal(pending_invalidate_controller, 54.5);
  pending_invalidate_controller.invalidateForHold("localization_epoch_changed");
  require(!pending_invalidate_controller.snapshot().pending_plan_queued,
          "pending invalidate did not clear queued B");
  require(pending_invalidate_controller.snapshot().active_request_id == "goal-active-a",
          "pending invalidate detached active A before stop evidence");
  require(countStatus(pending_invalidate_recorder.statuses, "goal-active-b",
                      NavigationGoalState::Cancelled, "localization_epoch_changed") == 1U,
          "pending invalidate did not cancel B exactly once");
  Recorder replan_options_recorder;
  std::vector<int> observed_replan_options;
  GoalPlanController replan_options_controller(
      [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
          const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        observed_replan_options.push_back(plan_request.options.max_iterations);
        lingtu::nav::plan::GlobalPlanResult result;
        result.ok = true;
        result.reached_goal = true;
        result.map_identity = {"field", 7, "map"};
        result.path = {plan_request.start, plan_request.goal};
        return result;
      },
      replan_options_recorder.actions());
  auto original_options_context = admission;
  original_options_context.planner_options.max_iterations = 123;
  require(replan_options_controller.submit(active_a_request, original_options_context).accepted,
          "replan-options test could not start active A");
  require(waitForCompletion(replan_options_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 55.0})
              .path_activated,
          "replan-options test could not activate A");
  auto changed_options_context = original_options_context;
  changed_options_context.map_position = nav_kernel::Vec3{2.5, 2.0, 0.5};
  changed_options_context.planner_options.max_iterations = 999;
  require(replan_options_controller.replanActive(changed_options_context).accepted,
          "replan-options test could not start replan");
  require(waitForCompletion(replan_options_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 56.0})
              .path_activated,
          "replan-options test could not complete replan");
  require(observed_replan_options.size() == 2U && observed_replan_options[0] == 123 &&
              observed_replan_options[1] == 123,
          "replan did not reuse active path planner options");

  Recorder replan_hold_recorder;
  GoalPlanController replan_hold_controller(successfulPlan, replan_hold_recorder.actions());
  require(replan_hold_controller.submit(active_a_request, admission).accepted,
          "replan-hold test could not start active A");
  require(waitForCompletion(replan_hold_controller,
                            GoalPlanAdvanceContext{admission.frame_epoch, false, 47.0})
              .path_activated,
          "replan-hold test could not activate A");
  require(replan_hold_controller.replanActive(replan_context).accepted,
          "replan-hold test could not start replan");
  replan_hold_controller.invalidateForHold("localization_epoch_changed");
  require(!replan_hold_controller.snapshot().replan_in_progress,
          "hold invalidation left replan marked in progress");
  require(replan_hold_controller.snapshot().active_request_id == "goal-active-a",
          "hold invalidation detached active goal before stop evidence");
  auto replan_hold_commit = replan_hold_controller.deferAbort("cancelled_after_stop_confirmation");
  replan_hold_commit();
  require(replan_hold_controller.snapshot().active_request_id.empty(),
          "hold abort commit left active identity live");
  return 0;
}
