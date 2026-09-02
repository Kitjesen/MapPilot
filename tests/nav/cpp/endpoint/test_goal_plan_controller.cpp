#include <chrono>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "runtime/goal/plan.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::nav::plan::GlobalPlanResult
planImmediately(const lingtu::nav::plan::GlobalPlanRequest &request,
                const lingtu::nav::plan::GlobalPlanCancelCheck &) {
  lingtu::nav::plan::GlobalPlanResult result;
  result.ok = true;
  result.reached_goal = true;
  result.map_identity = {"field", 7, "map"};
  result.overlay_revision = request.temporary_overlay.revision;
  result.overlay_frame_epoch = request.temporary_overlay.frame_epoch;
  result.overlay_obstacle_generation = request.temporary_overlay.obstacle_generation;
  result.overlay_traversability_generation = request.temporary_overlay.traversability_generation;
  result.path = {request.start, request.goal};
  return result;
}

}  // namespace

int main() {
  using lingtu::message::NavigationGoalState;
  using lingtu::nav::endpoint::GoalPlanActions;
  using lingtu::nav::endpoint::GoalPlanAdmissionContext;
  using lingtu::nav::endpoint::GoalPlanController;
  using lingtu::nav::endpoint::GoalPlanInspectionDecision;
  using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
  using lingtu::nav::endpoint::GoalPlanOrigin;
  using lingtu::nav::endpoint::GoalPlanPathActivation;
  using lingtu::nav::endpoint::GoalPlanRequest;
  using lingtu::nav::endpoint::GoalPlanStatus;
  using lingtu::nav::endpoint::GoalPlanTarget;

  std::vector<GoalPlanStatus> statuses;
  GoalPlanActions actions;
  actions.preempt_rolling = [](const std::string &) { return true; };
  actions.clear_external_inspection = [] {};
  actions.current_map_identity = [] {
    return GoalPlanMapIdentityResult{lingtu::nav::plan::MapIdentity{"field", 7, "map"},
                                     {}};
  };
  actions.publish_status = [&](const GoalPlanStatus &status) { statuses.push_back(status); };
  actions.inspection_active = [] { return false; };
  actions.inspection_leg_failed = [](const std::string &, double) {};
  actions.inspection_pause = [](const std::string &) {};
  actions.inspection_plan_ready = [](double) { return GoalPlanInspectionDecision{}; };
  actions.activate_path = [](const GoalPlanPathActivation &) {};

  GoalPlanController controller(planImmediately, std::move(actions));
  GoalPlanAdmissionContext context;
  context.motion_allowed = true;
  context.autonomy_mode = true;
  context.planner_map_configured = true;
  context.map_position = nav_kernel::Vec3{1.0, 2.0, 0.5};
  context.odometry_ready = true;
  context.frame_epoch = 4;

  GoalPlanRequest request;
  request.task_id = "navigation-task-1";
  request.request_id = "goal-1";
  request.origin = GoalPlanOrigin::kExternal;
  request.source_stamp_s = 10.0;
  request.target = GoalPlanTarget{
      nav_kernel::Vec3{4.0, 5.0, 0.5},
      0.25,
  };

  const auto admission = controller.submit(request, context);
  require(admission.accepted, "valid goal was rejected by controller");
  require(admission.reason == "planning_started", "accepted goal returned wrong reason");
  require(!admission.counted_failure, "accepted goal counted a planning failure");

  const auto snapshot = controller.snapshot();
  require(snapshot.goal_epoch == 1U, "first goal did not allocate epoch one");
  require(snapshot.planning_task_id == "navigation-task-1",
          "planning task identity was not retained");
  require(snapshot.planning_request_id == "goal-1", "planning identity was not retained");
  require(snapshot.active_request_id.empty(), "new goal became active before a path existed");
  require(!snapshot.active_origin.has_value(), "planning-only goal exposed an active origin");
  require(snapshot.busy, "accepted goal did not start the planner task");
  require(snapshot.diagnostics.seen, "goal diagnostics were not initialized");
  require(snapshot.diagnostics.reason == "planning", "planning diagnostics reason changed");
  require(snapshot.diagnostics.start.x == 1.0, "planning start was not captured");
  require(snapshot.diagnostics.goal.x == 4.0, "planning goal was not captured");

  require(statuses.size() == 1U, "planning emitted an unexpected status count");
  require(statuses.front().task_id == "navigation-task-1", "planning status lost task identity");
  require(statuses.front().request_id == "goal-1", "planning status lost request identity");
  require(statuses.front().goal_epoch == 1U, "planning status lost goal epoch");
  require(statuses.front().state == NavigationGoalState::Planning,
          "planning status used the wrong lifecycle state");
  require(statuses.front().reason == "planning", "planning status reason changed");

  bool blocked_external_inspection_cleared = false;
  std::vector<GoalPlanStatus> blocked_statuses;
  GoalPlanActions blocked_actions;
  blocked_actions.preempt_rolling = [](const std::string &) { return true; };
  blocked_actions.clear_external_inspection = [&] { blocked_external_inspection_cleared = true; };
  blocked_actions.current_map_identity = [] {
    return GoalPlanMapIdentityResult{lingtu::nav::plan::MapIdentity{"field", 7, "map"},
                                     {}};
  };
  blocked_actions.publish_status = [&](const GoalPlanStatus &status) {
    blocked_statuses.push_back(status);
  };
  blocked_actions.inspection_active = [] { return true; };
  blocked_actions.inspection_leg_failed = [](const std::string &, double) {};
  blocked_actions.inspection_pause = [](const std::string &) {};
  blocked_actions.inspection_plan_ready = [](double) { return GoalPlanInspectionDecision{}; };
  blocked_actions.activate_path = [](const GoalPlanPathActivation &) {};

  GoalPlanController blocked_controller(planImmediately, std::move(blocked_actions));
  const auto blocked = blocked_controller.submit(request, context);
  require(!blocked.accepted, "external goal bypassed active inspection");
  require(blocked.reason == "inspection_run_active", "inspection rejection reason changed");
  require(blocked.counted_failure, "inspection rejection did not count a failure");
  require(!blocked_external_inspection_cleared,
          "rejected external goal cleared the active inspection point");
  require(blocked_statuses.empty(), "rejected external goal published lifecycle status");
  require(blocked_controller.snapshot().goal_epoch == 0U,
          "rejected external goal consumed an epoch");

  auto make_admission_actions = [](std::vector<GoalPlanStatus> *observed_statuses,
                                   bool *rolling_preempted, bool rolling_preempt_ok) {
    GoalPlanActions value;
    value.preempt_rolling = [rolling_preempted, rolling_preempt_ok](const std::string &) {
      *rolling_preempted = true;
      return rolling_preempt_ok;
    };
    value.clear_external_inspection = [] {};
    value.current_map_identity = [] {
      return GoalPlanMapIdentityResult{
          lingtu::nav::plan::MapIdentity{"field", 7, "map"}, {}};
    };
    value.publish_status = [observed_statuses](const GoalPlanStatus &status) {
      observed_statuses->push_back(status);
    };
    value.inspection_active = [] { return false; };
    value.inspection_leg_failed = [](const std::string &, double) {};
    value.inspection_pause = [](const std::string &) {};
    value.inspection_plan_ready = [](double) { return GoalPlanInspectionDecision{}; };
    value.activate_path = [](const GoalPlanPathActivation &) {};
    return value;
  };

  auto expect_admission_rejection =
      [&](GoalPlanRequest candidate, GoalPlanAdmissionContext candidate_context,
          const std::string &expected_reason, bool expected_frame_rejection,
          bool expected_frame_error, bool rolling_preempt_ok = true) {
        std::vector<GoalPlanStatus> observed_statuses;
        bool rolling_preempted = false;
        GoalPlanController candidate_controller(
            planImmediately,
            make_admission_actions(&observed_statuses, &rolling_preempted, rolling_preempt_ok));
        const auto result = candidate_controller.submit(candidate, candidate_context);
        require(!result.accepted, "admission gate accepted a blocked goal");
        require(result.reason == expected_reason, "admission rejection reason changed");
        require(result.counted_failure, "admission rejection did not count a failure");
        require(result.count_frame_rejection == expected_frame_rejection,
                "admission frame-rejection accounting changed");
        require(result.record_frame_error == expected_frame_error,
                "admission frame-error accounting changed");
        require(observed_statuses.empty(), "rejected goal published lifecycle status");
        const auto rejected_snapshot = candidate_controller.snapshot();
        require(rejected_snapshot.goal_epoch == 0U, "rejected goal consumed an epoch");
        require(!rejected_snapshot.busy, "rejected goal started the planner");
        return rolling_preempted;
      };

  auto blocked_context = context;
  blocked_context.motion_allowed = false;
  expect_admission_rejection(request, blocked_context, "estop_latched", true, true);

  blocked_context = context;
  blocked_context.operator_takeover_latched = true;
  expect_admission_rejection(request, blocked_context, "operator_takeover_resume_required", true,
                             true);

  blocked_context = context;
  blocked_context.autonomy_mode = false;
  blocked_context.control_mode_name = "teleop";
  expect_admission_rejection(request, blocked_context, "goal_not_allowed_in_teleop", true, true);

  blocked_context = context;
  blocked_context.driver_control_blocker = "driver_control_stale";
  expect_admission_rejection(request, blocked_context, "driver_control_stale", true, true);

  blocked_context = context;
  blocked_context.autonomy_request_not_before_s = request.source_stamp_s;
  expect_admission_rejection(request, blocked_context, "goal_predates_autonomy_resume", true, true);

  auto malformed = request;
  malformed.target.reset();
  malformed.decode_error = "goal_frame_invalid";
  expect_admission_rejection(malformed, context, "goal_frame_invalid", true, true);

  blocked_context = context;
  blocked_context.map_position.reset();
  blocked_context.odometry_ready = true;
  expect_admission_rejection(request, blocked_context, "map_odom_tf_not_ready", false, false);

  blocked_context.odometry_ready = false;
  expect_admission_rejection(request, blocked_context, "odometry_not_ready", false, false);

  blocked_context = context;
  blocked_context.planner_map_configured = false;
  blocked_context.planner_map_missing_reason = "active_octomap_not_configured";
  expect_admission_rejection(request, blocked_context, "active_octomap_not_configured", false,
                             false);

  blocked_context = context;
  blocked_context.rolling_segment_active = true;
  const bool attempted_preempt = expect_admission_rejection(
      request, blocked_context, "segment_preempt_zero_publish_failed", false, true, false);
  require(attempted_preempt, "rolling handoff failure was not attempted");

  std::vector<std::string> rolling_handoff_events;
  std::mutex rolling_handoff_mutex;
  std::vector<GoalPlanStatus> rolling_success_statuses;
  bool rolling_success_preempted = false;
  GoalPlanActions rolling_success_actions =
      make_admission_actions(&rolling_success_statuses, &rolling_success_preempted, true);
  rolling_success_actions.preempt_rolling = [&](const std::string &reason) {
    std::lock_guard<std::mutex> lock(rolling_handoff_mutex);
    rolling_handoff_events.push_back(reason);
    return true;
  };
  GoalPlanController rolling_success_controller(
      [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
          const lingtu::nav::plan::GlobalPlanCancelCheck &) {
        {
          std::lock_guard<std::mutex> lock(rolling_handoff_mutex);
          rolling_handoff_events.push_back("planner_started");
        }
        return planImmediately(plan_request, {});
      },
      std::move(rolling_success_actions));
  auto rolling_success_context = context;
  rolling_success_context.rolling_segment_active = true;
  const auto rolling_success_result =
      rolling_success_controller.submit(request, rolling_success_context);
  require(rolling_success_result.accepted,
          "rolling handoff success did not start generic planning");
  for (int attempt = 0; attempt < 100; ++attempt) {
    {
      std::lock_guard<std::mutex> lock(rolling_handoff_mutex);
      if (rolling_handoff_events.size() >= 2U) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  {
    std::lock_guard<std::mutex> lock(rolling_handoff_mutex);
    require(rolling_handoff_events.size() >= 2U, "rolling handoff did not reach planner start");
    require(rolling_handoff_events[0] == "superseded_by_generic_goal",
            "generic planning started before rolling preempt");
    require(rolling_handoff_events[1] == "planner_started",
            "rolling handoff did not start the planner after preempt");
  }
  require(rolling_success_statuses.size() == 1U &&
              rolling_success_statuses.front().state == NavigationGoalState::Planning,
          "rolling handoff success did not publish planning status");

  std::vector<GoalPlanStatus> busy_statuses;
  bool busy_preempted = false;
  GoalPlanController busy_controller(planImmediately,
                                     make_admission_actions(&busy_statuses, &busy_preempted, true));
  require(busy_controller.submit(request, context).accepted,
          "busy test could not start first goal");
  auto overlapping_request = request;
  overlapping_request.request_id = "goal-overlap";
  const auto busy_result = busy_controller.submit(overlapping_request, context);
  require(!busy_result.accepted, "overlapping goal bypassed planner busy gate");
  require(busy_result.reason == "global_planner_busy", "planner busy rejection reason changed");
  require(busy_controller.snapshot().goal_epoch == 1U, "busy rejection consumed a second epoch");
  require(busy_statuses.size() == 1U, "busy rejection published a second planning status");
  require(!busy_preempted, "busy rejection touched rolling execution");

  std::vector<GoalPlanStatus> success_statuses;
  std::vector<GoalPlanPathActivation> success_activations;
  bool success_preempted = false;
  std::optional<lingtu::nav::plan::MapIdentity> success_current_map =
      lingtu::nav::plan::MapIdentity{"field", 7, "map"};
  auto success_actions = make_admission_actions(&success_statuses, &success_preempted, true);
  success_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
    success_activations.push_back(activation);
  };
  success_actions.current_map_identity = [&] {
    return GoalPlanMapIdentityResult{success_current_map,
                                     success_current_map ? "" : "active_map_unavailable"};
  };
  GoalPlanController success_controller(planImmediately, std::move(success_actions));
  require(success_controller.submit(request, context).accepted,
          "successful completion test could not start planning");

  lingtu::nav::endpoint::GoalPlanAdvanceResult completion;
  for (int attempt = 0; attempt < 100 && !completion.completion_consumed; ++attempt) {
    completion = success_controller.advance(lingtu::nav::endpoint::GoalPlanAdvanceContext{
        context.frame_epoch,
        false,
        20.0,
    });
    if (!completion.completion_consumed) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  require(completion.completion_consumed, "completed planner task was not consumed");
  require(completion.path_activated, "valid planner result did not activate a path");
  require(!completion.counted_failure, "valid planner result counted a failure");
  require(success_activations.size() == 1U, "path activation count changed");
  require(success_activations.front().path.size() == 2U, "activated path lost planner waypoints");
  require(success_activations.front().map_identity.has_value() &&
              lingtu::nav::plan::sameMapIdentity(
                  *success_activations.front().map_identity,
                  lingtu::nav::plan::MapIdentity{"field", 7, "map"}),
          "path activation lost the planner map identity");
  require(success_activations.front().goal_yaw.has_value() &&
              *success_activations.front().goal_yaw == 0.25,
          "activated path lost goal yaw");
  require(success_statuses.size() == 2U, "goal lifecycle status count changed");
  require(success_statuses.back().state == NavigationGoalState::PathActive,
          "successful plan did not publish PathActive");
  require(success_statuses.back().request_id == request.request_id,
          "PathActive status lost request identity");
  const auto active_snapshot = success_controller.snapshot();
  require(active_snapshot.planning_request_id.empty(),
          "planning identity remained after path activation");
  require(active_snapshot.active_request_id == request.request_id,
          "active goal identity was not transferred");
  require(active_snapshot.active_origin == GoalPlanOrigin::kExternal,
          "active external origin was not projected into snapshot");
  require(active_snapshot.diagnostics.accepted, "accepted plan diagnostics were false");
  require(active_snapshot.diagnostics.reason == "accepted",
          "accepted plan diagnostics reason changed");

  const auto premature_resume =
      success_controller.deferResume(request.task_id, "resume-too-early", context);
  require(!premature_resume.accepted && premature_resume.reason == "task_not_paused",
          "an executing task resumed without a confirmed pause");

  const auto wrong_pause =
      success_controller.deferPause("another-task", "pause-wrong", "operator_pause");
  require(!wrong_pause.accepted && wrong_pause.reason == "task_not_active",
          "task pause accepted the wrong active task");

  auto deferred_pause = success_controller.deferPause(request.task_id, "pause-1", "operator_pause");
  require(deferred_pause.accepted && deferred_pause.reason == "pause_ready",
          "active task pause was not prepared");
  require(success_statuses.size() == 2U, "pause published state before stop evidence");
  deferred_pause.commit();
  require(success_statuses.size() == 3U, "confirmed pause did not publish state");
  require(success_statuses.back().state == NavigationGoalState::Paused,
          "confirmed pause used the wrong lifecycle state");
  require(success_statuses.back().task_id == request.task_id &&
              success_statuses.back().request_id == "pause-1",
          "pause status lost task or causal request identity");
  require(success_controller.snapshot().active_paused,
          "confirmed pause did not retain a resumable task");

  const auto wrong_resume = success_controller.deferResume("another-task", "resume-wrong", context);
  require(!wrong_resume.accepted && wrong_resume.reason == "task_not_active",
          "task resume accepted the wrong paused task");

  const auto expect_resume_rejection = [&](GoalPlanAdmissionContext candidate_context,
                                           const std::string &expected_reason) {
    const auto result =
        success_controller.deferResume(request.task_id, "resume-blocked", candidate_context);
    require(!result.accepted && result.reason == expected_reason,
            "task resume admission returned the wrong rejection");
    require(success_controller.snapshot().active_paused,
            "a rejected resume changed the paused task state");
  };

  auto blocked_resume_context = context;
  blocked_resume_context.motion_allowed = false;
  expect_resume_rejection(blocked_resume_context, "estop_latched");

  blocked_resume_context = context;
  blocked_resume_context.operator_takeover_latched = true;
  expect_resume_rejection(blocked_resume_context, "operator_takeover_resume_required");

  blocked_resume_context = context;
  blocked_resume_context.autonomy_mode = false;
  blocked_resume_context.control_mode_name = "teleop";
  expect_resume_rejection(blocked_resume_context, "resume_not_allowed_in_teleop");

  blocked_resume_context = context;
  blocked_resume_context.driver_control_blocker = "driver_control_stale";
  expect_resume_rejection(blocked_resume_context, "driver_control_stale");

  blocked_resume_context = context;
  blocked_resume_context.input_ready = false;
  blocked_resume_context.input_gate_reason = "localization_stale";
  expect_resume_rejection(blocked_resume_context, "input_gate_localization_stale");

  blocked_resume_context = context;
  blocked_resume_context.retained_path_ready = false;
  blocked_resume_context.retained_path_reason = "retained_global_path_missing";
  expect_resume_rejection(blocked_resume_context, "retained_global_path_missing");

  blocked_resume_context = context;
  blocked_resume_context.map_position.reset();
  expect_resume_rejection(blocked_resume_context, "map_odom_tf_not_ready");

  blocked_resume_context.odometry_ready = false;
  expect_resume_rejection(blocked_resume_context, "odometry_not_ready");

  blocked_resume_context = context;
  blocked_resume_context.planner_map_configured = false;
  blocked_resume_context.planner_map_missing_reason = "active_octomap_not_configured";
  expect_resume_rejection(blocked_resume_context, "active_octomap_not_configured");

  success_current_map.reset();
  expect_resume_rejection(context, "active_map_unavailable_before_resume");
  success_current_map = lingtu::nav::plan::MapIdentity{"field", 8, "map"};
  expect_resume_rejection(context, "active_map_changed_before_resume");
  success_current_map = lingtu::nav::plan::MapIdentity{"field", 7, "map"};

  auto deferred_resume = success_controller.deferResume(request.task_id, "resume-1", context);
  require(deferred_resume.accepted && deferred_resume.reason == "resume_ready",
          "paused task was not resumable");
  deferred_resume.commit();
  require(success_statuses.size() == 4U &&
              success_statuses.back().state == NavigationGoalState::PathActive,
          "confirmed resume did not republish PathActive");
  require(success_statuses.back().request_id == "resume-1",
          "resumed task did not carry the new request identity");
  require(!success_controller.snapshot().active_paused, "resumed task remained paused");

  {
    std::vector<lingtu::nav::plan::GlobalPlanRequest> observed_requests;
    std::vector<GoalPlanPathActivation> overlay_activations;
    GoalPlanActions overlay_actions;
    overlay_actions.preempt_rolling = [](const std::string &) { return true; };
    overlay_actions.clear_external_inspection = [] {};
    overlay_actions.current_map_identity = [] {
      return GoalPlanMapIdentityResult{
          lingtu::nav::plan::MapIdentity{"field", 7, "map"}, {}};
    };
    overlay_actions.publish_status = [](const GoalPlanStatus &) {};
    overlay_actions.inspection_active = [] { return false; };
    overlay_actions.inspection_leg_failed = [](const std::string &, double) {};
    overlay_actions.inspection_pause = [](const std::string &) {};
    overlay_actions.inspection_plan_ready = [](double) { return GoalPlanInspectionDecision{}; };
    overlay_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      overlay_activations.push_back(activation);
    };

    GoalPlanController overlay_controller(
        [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
            const lingtu::nav::plan::GlobalPlanCancelCheck &) {
          observed_requests.push_back(plan_request);
          auto result = planImmediately(plan_request, {});
          return result;
        },
        std::move(overlay_actions));
    GoalPlanAdmissionContext overlay_context = context;
    overlay_context.temporary_overlay.revision = 9U;
    overlay_context.temporary_overlay.frame_epoch = context.frame_epoch;
    overlay_context.temporary_overlay.obstacle_generation = 21U;
    overlay_context.temporary_overlay.traversability_generation = 13U;
    overlay_context.temporary_overlay.blocked_regions.push_back({{2.0, 3.0, 0.5}, 0.7, -0.2, 1.8});
    require(overlay_controller.submit(request, overlay_context).accepted,
            "overlay fixture initial goal was rejected");
    lingtu::nav::endpoint::GoalPlanAdvanceResult overlay_completion;
    for (int i = 0; i < 1000 && !overlay_completion.path_activated; ++i) {
      overlay_completion = overlay_controller.advance(
          lingtu::nav::endpoint::GoalPlanAdvanceContext{context.frame_epoch, false, 30.0});
      if (!overlay_completion.completion_consumed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    require(overlay_completion.path_activated && observed_requests.size() == 1U &&
                observed_requests.front().temporary_overlay.empty(),
            "ordinary planning unexpectedly carried a temporary overlay");

    require(overlay_controller.replanActive(overlay_context).accepted,
            "overlay replan was rejected");
    overlay_completion = {};
    for (int i = 0; i < 1000 && !overlay_completion.path_activated; ++i) {
      overlay_completion = overlay_controller.advance(
          lingtu::nav::endpoint::GoalPlanAdvanceContext{context.frame_epoch, false, 31.0});
      if (!overlay_completion.completion_consumed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    require(overlay_completion.path_activated && observed_requests.size() == 2U,
            "overlay replan did not atomically activate one replacement path");
    const auto &observed_overlay = observed_requests.back().temporary_overlay;
    require(observed_overlay.revision == 9U &&
                observed_overlay.frame_epoch == context.frame_epoch &&
                observed_overlay.obstacle_generation == 21U &&
                observed_overlay.traversability_generation == 13U &&
                observed_overlay.blocked_regions.size() == 1U &&
                observed_overlay.blocked_regions.front().radius_xy_m == 0.7,
            "GoalPlan did not preserve the request-scoped overlay across replan admission");
    require(overlay_activations.size() == 2U, "overlay replan activated a path more than once");
  }

  return 0;
}
