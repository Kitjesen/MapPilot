#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "app/goal/plan.hpp"

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
  result.path = {request.start, request.goal};
  return result;
}

lingtu::nav::endpoint::GoalPlanAdvanceResult
waitForCompletion(lingtu::nav::endpoint::GoalPlanController &controller,
                  std::uint64_t frame_epoch, double now_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = controller.advance({frame_epoch, {}, now_s});
    if (result.completion_consumed) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("planner fixture did not complete");
}

}  // namespace

int runTests() {
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

  {
    std::vector<GoalPlanStatus> goal_statuses;
    std::vector<GoalPlanPathActivation> activations;
    bool goal_preempted = false;
    auto goal_actions = make_admission_actions(&goal_statuses, &goal_preempted, true);
    goal_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      activations.push_back(activation);
    };
    GoalPlanController shifted_controller(
        [](const lingtu::nav::plan::GlobalPlanRequest &input,
           const lingtu::nav::plan::GlobalPlanCancelCheck &cancel) {
          auto result = planImmediately(input, cancel);
          result.path.back().x -= 0.14;
          return result;
        }, std::move(goal_actions));
    require(shifted_controller.submit(request, context).accepted,
            "shifted goal fixture could not start planning");
    require(waitForCompletion(shifted_controller, context.frame_epoch, 20.0).path_activated &&
                activations.size() == 1U,
            "shifted goal fixture did not activate its path");
    require(activations.front().requested_goal &&
                activations.front().requested_goal->x == request.target->position.x &&
                activations.front().path.back().x == request.target->position.x - 0.14,
            "goal activation replaced the requested arrival position with a shifted endpoint");
  }

  for (int support_case = 0; support_case < 4; ++support_case) {
    std::vector<GoalPlanStatus> support_statuses;
    std::vector<GoalPlanPathActivation> support_activations;
    bool support_preempted = false;
    auto support_actions = make_admission_actions(&support_statuses, &support_preempted, true);
    support_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      support_activations.push_back(activation);
    };
    GoalPlanController support_controller(planImmediately, std::move(support_actions));
    auto support_context = context;
    support_context.planner_options.ground_support = support_case != 1;
    support_context.planner_options.support_height_m = support_case == 3 ? 0.0 : 0.52;
    support_context.planner_options.support_height_tolerance_m = support_case == 2 ? 0.0 : 0.03;
    require(support_controller.submit(request, support_context).accepted,
            "support activation test could not start planning");
    // Later admission/config changes cannot alter the completed request's datum.
    support_context.planner_options.support_height_m = 1.4;
    support_context.planner_options.support_height_tolerance_m = 0.2;
    lingtu::nav::endpoint::GoalPlanAdvanceResult support_completion;
    for (int attempt = 0; attempt < 100 && !support_completion.completion_consumed; ++attempt) {
      support_completion = support_controller.advance({context.frame_epoch, {}, 20.0});
      if (!support_completion.completion_consumed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    require(support_completion.path_activated && support_activations.size() == 1U,
            "support activation test lost its completed path");
    const auto &activation = support_activations.front();
    require(activation.requested_goal &&
                activation.requested_goal->x == request.target->position.x &&
                activation.requested_goal->y == request.target->position.y &&
                activation.requested_goal->z == request.target->position.z,
            "activation lost the original task position");
    if (support_case == 1 || support_case == 3) {
      require(!activation.support_height_m && !activation.support_height_tolerance_m,
              "missing or disabled support fabricated a turn support datum");
    } else {
      require(activation.support_height_m == std::optional<double>{0.52} &&
                  activation.support_height_tolerance_m ==
                      std::optional<double>{support_case == 2 ? 0.0 : 0.03},
              "activation did not retain the completed request's exact support datum");
    }
  }

  for (const char *retry_failure : {"empty_path", "start_snap_exhausted", "start_occupied",
                                   "start_unsupported", "start_temporary_obstacle"}) {
    std::vector<GoalPlanStatus> retry_statuses;
    bool retry_preempted = false;
    std::vector<lingtu::nav::plan::GlobalPlanRequest> retry_requests;
    auto retry_actions = make_admission_actions(&retry_statuses, &retry_preempted, true);
    std::vector<GoalPlanPathActivation> retry_activations;
    retry_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      retry_activations.push_back(activation);
    };
    GoalPlanController retry_controller(
        [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
            const lingtu::nav::plan::GlobalPlanCancelCheck &cancel) {
          retry_requests.push_back(plan_request);
          if (retry_requests.size() > 1U) return planImmediately(plan_request, cancel);
          lingtu::nav::plan::GlobalPlanResult result;
          result.map_identity = {"field", 7, "map"};
          result.failure_reason = retry_failure;
          return result;
        },
        std::move(retry_actions));
    require(retry_controller.submit(request, context).accepted,
            "initial no-path retry fixture was not admitted");
    lingtu::nav::endpoint::GoalPlanAdvanceResult retry_completion;
    for (int tick = 0; tick < 1000 && !retry_completion.completion_consumed; ++tick) {
      retry_completion = retry_controller.advance({context.frame_epoch, {}, 20.0});
      if (!retry_completion.completion_consumed)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(retry_completion.completion_consumed && !retry_completion.path_activated,
            "initial no-path search did not complete without activating motion");
    const auto retained = retry_controller.snapshot();
    require(retained.pending_plan_queued && !retained.busy &&
                retained.active_request_id.empty(),
            "initial ordinary no-path search dropped its accepted pending goal");
    require(retry_statuses.back().state == NavigationGoalState::Planning &&
                retry_statuses.back().reason == "planning_waiting",
            "initial ordinary no-path search published a terminal task state");
    require(retained.pending_task_id == request.task_id &&
                retained.pending_request_id == request.request_id &&
                retained.pending_goal_epoch == 1U,
            "initial no-path retry changed the task/request/goal identity");
    require(retry_activations.empty(), "initial no-path search activated motion");
    auto fresh_context = context;
    fresh_context.map_position = nav_kernel::Vec3{2.0, 3.0, .5};
    const auto early = retry_controller.resumePending(fresh_context, 20.49);
    require(!early.accepted && early.reason == "planning_waiting" &&
                retry_requests.size() == 1U && retry_controller.snapshot().pending_plan_queued,
            "initial no-path retry ignored its backoff");
    require(retry_controller.resumePending(fresh_context, 20.5).accepted,
            "initial no-path retry did not start at its deadline");
    retry_completion = {};
    for (int tick = 0; tick < 1000 && !retry_completion.completion_consumed; ++tick) {
      retry_completion = retry_controller.advance({context.frame_epoch, {}, 20.6});
      if (!retry_completion.completion_consumed)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(retry_completion.path_activated && retry_activations.size() == 1U &&
                retry_requests.size() == 2U && retry_requests.back().start.x == 2.0 &&
                retry_requests.back().start.y == 3.0 &&
                retry_requests.back().goal.x == request.target->position.x,
            "initial no-path retry did not use the fresh pose and retained target");
    const auto active = retry_controller.snapshot();
    require(active.active_task_id == request.task_id &&
                active.active_request_id == request.request_id && active.active_goal_epoch == 1U,
            "successful initial retry replaced the accepted task identity");
  }

  for (const std::string interruption : {"cancel", "new_goal", "map_changed"}) {
    std::vector<GoalPlanStatus> observed;
    bool preempted = false;
    int planner_calls = 0;
    std::optional<lingtu::nav::plan::MapIdentity> current_map{
        lingtu::nav::plan::MapIdentity{"field", 7, "map"}};
    auto retry_actions = make_admission_actions(&observed, &preempted, true);
    retry_actions.current_map_identity = [&] {
      return GoalPlanMapIdentityResult{current_map, {}};
    };
    std::vector<GoalPlanPathActivation> activations;
    retry_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      activations.push_back(activation);
    };
    GoalPlanController waiting(
        [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
            const lingtu::nav::plan::GlobalPlanCancelCheck &cancel) {
          if (++planner_calls > 1) return planImmediately(plan_request, cancel);
          lingtu::nav::plan::GlobalPlanResult result;
          result.map_identity = {"field", 7, "map"};
          result.failure_reason = "support_disconnected";
          return result;
        },
        std::move(retry_actions));
    require(waiting.submit(request, context).accepted, "retry interruption fixture was rejected");
    (void)waitForCompletion(waiting, context.frame_epoch, 30.0);
    require(waiting.snapshot().pending_plan_queued, "retry interruption fixture lost pending goal");

    if (interruption == "cancel") {
      auto cancelled = waiting.deferCancelPending(request.task_id, "cancel-retry", "operator_cancel");
      require(cancelled.accepted && cancelled.commit, "waiting goal could not be cancelled");
      cancelled.commit();
      require(!waiting.resumePending(context, 31.0).accepted && planner_calls == 1 &&
                  !waiting.snapshot().pending_plan_queued && activations.empty() &&
                  observed.back().state == NavigationGoalState::Cancelled,
              "cancelled waiting goal was revived by its retry deadline");
    } else if (interruption == "new_goal") {
      auto replacement = request;
      replacement.task_id = "replacement-task";
      replacement.request_id = "replacement-goal";
      replacement.target->position.x = 8.0;
      require(waiting.submit(replacement, context).accepted &&
                  waiting.resumePending(context, 30.1).accepted,
              "new goal did not supersede the waiting search");
      require(waitForCompletion(waiting, context.frame_epoch, 30.2).path_activated &&
                  planner_calls == 2 && activations.size() == 1U &&
                  activations.front().path.back().x == 8.0 &&
                  waiting.snapshot().active_task_id == replacement.task_id,
              "retry activated the abandoned target after replacement");
    } else {
      current_map->content_epoch = 8;
      const auto changed = waiting.resumePending(context, 30.5);
      require(!changed.accepted && changed.reason == "active_map_changed_before_pending_plan" &&
                  !waiting.snapshot().pending_plan_queued && planner_calls == 1 &&
                  activations.empty() && observed.back().state == NavigationGoalState::Failed,
              "waiting search retried the old goal against a different map");
    }
  }

  for (const std::string failure : {"body_pose_input_invalid", "temporary_overlay_region_invalid",
                                    "cancelled", "exception"}) {
    std::vector<GoalPlanStatus> observed;
    bool preempted = false;
    auto failed_actions = make_admission_actions(&observed, &preempted, true);
    int activations = 0;
    failed_actions.activate_path = [&](const GoalPlanPathActivation &) { ++activations; };
    GoalPlanController invalid(
        [&](const lingtu::nav::plan::GlobalPlanRequest &,
            const lingtu::nav::plan::GlobalPlanCancelCheck &)
            -> lingtu::nav::plan::GlobalPlanResult {
          if (failure == "exception") throw std::runtime_error("fixture algorithm failure");
          lingtu::nav::plan::GlobalPlanResult result;
          result.map_identity = {"field", 7, "map"};
          result.cancelled = failure == "cancelled";
          result.failure_reason = failure;
          return result;
        },
        std::move(failed_actions));
    require(invalid.submit(request, context).accepted, "invalid-result fixture was not admitted");
    (void)waitForCompletion(invalid, context.frame_epoch, 40.0);
    require(!invalid.snapshot().pending_plan_queued && activations == 0 &&
                observed.back().state == NavigationGoalState::Failed,
            "invalid request/cancel/algorithm failure was silently retried");
  }

  {
    std::vector<GoalPlanStatus> observed;
    bool preempted = false;
    int calls = 0;
    auto timeout_actions = make_admission_actions(&observed, &preempted, true);
    GoalPlanController timeout(
        [&](const lingtu::nav::plan::GlobalPlanRequest &plan_request,
            const lingtu::nav::plan::GlobalPlanCancelCheck &cancel) {
          if (++calls > 1) return planImmediately(plan_request, cancel);
          while (!cancel()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          lingtu::nav::plan::GlobalPlanResult result;
          result.map_identity = {"field", 7, "map"};
          result.cancelled = true;
          return result;
        },
        std::move(timeout_actions));
    require(timeout.submit(request, context).accepted, "timeout fixture was not admitted");
    (void)waitForCompletion(timeout, context.frame_epoch, 50.0);
    require(timeout.snapshot().pending_plan_queued &&
                timeout.snapshot().diagnostics.reason == "global_planning_timeout" &&
                observed.back().state == NavigationGoalState::Planning,
            "real planner timeout ended the accepted goal instead of waiting to retry");
    require(timeout.resumePending(context, 50.5).accepted &&
                waitForCompletion(timeout, context.frame_epoch, 50.6).path_activated && calls == 2,
            "timed-out initial search did not recover on a later successful attempt");
  }

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
  blocked_context.hold_reason = "operator_takeover";
  expect_admission_rejection(request, blocked_context, "operator_takeover", true,
                             true);

  blocked_context.hold_reason = "stop_confirmation_timeout";
  expect_admission_rejection(request, blocked_context, "stop_confirmation_timeout", true, true);

  blocked_context = context;
  blocked_context.autonomy_mode = false;
  blocked_context.control_mode_name = "teleop";
  expect_admission_rejection(request, blocked_context, "goal_not_allowed_in_teleop", true, true);

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
  auto inspection_request = request;
  inspection_request.origin = GoalPlanOrigin::kInspection;
  expect_admission_rejection(inspection_request, blocked_context, "map_odom_tf_not_ready", false,
                             false);

  blocked_context.odometry_ready = false;
  expect_admission_rejection(inspection_request, blocked_context, "odometry_not_ready", false, false);
  {
    std::vector<GoalPlanStatus> queued_statuses;
    bool preempted = false;
    GoalPlanController queued(
        planImmediately, make_admission_actions(&queued_statuses, &preempted, true));
    const auto result = queued.submit(request, blocked_context);
    const auto queued_snapshot = queued.snapshot();
    require(result.accepted && result.reason == "planning_queued" && !queued_snapshot.busy,
            "external goal must wait for pose without starting search");
    require(queued_snapshot.pending_plan_queued && queued_snapshot.goal_epoch == 1U &&
                queued_snapshot.pending_task_id == request.task_id &&
                queued_snapshot.pending_request_id == request.request_id,
            "waiting for pose must preserve the pending goal identity");
    require(queued_statuses.size() == 1U &&
                queued_statuses.front().state == NavigationGoalState::Planning &&
                queued_statuses.front().reason == "planning_queued",
            "queued goal must publish its waiting state");
  }

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
  overlapping_request.origin = GoalPlanOrigin::kInspection;
  overlapping_request.request_id = "goal-overlap";
  const auto busy_result = busy_controller.submit(overlapping_request, context);
  require(!busy_result.accepted, "inspection goal bypassed planner busy gate");
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
        context.frame_epoch, {},
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

  const auto expect_resume_wait = [&](GoalPlanAdmissionContext candidate_context,
                                      const std::string &expected_reason) {
    const auto previous = success_controller.snapshot();
    const auto status_count = success_statuses.size();
    const auto result =
        success_controller.deferResume(request.task_id, "resume-wait", candidate_context);
    require(result.accepted && result.reason == expected_reason && result.commit,
            "recoverable resume gap must retain a deferred transition");
    const auto waiting = success_controller.snapshot();
    require(waiting.active_paused && waiting.active_request_id == previous.active_request_id &&
                success_statuses.size() == status_count,
            "preparing a waiting resume must not commit execution state");
  };

  auto blocked_resume_context = context;
  blocked_resume_context.motion_allowed = false;
  expect_resume_rejection(blocked_resume_context, "estop_latched");

  blocked_resume_context = context;
  blocked_resume_context.hold_reason = "operator_takeover";
  expect_resume_rejection(blocked_resume_context, "operator_takeover");

  blocked_resume_context = context;
  blocked_resume_context.autonomy_mode = false;
  blocked_resume_context.control_mode_name = "teleop";
  expect_resume_rejection(blocked_resume_context, "resume_not_allowed_in_teleop");

  blocked_resume_context = context;
  blocked_resume_context.input_ready = false;
  blocked_resume_context.input_gate_reason = "profile_transition";
  expect_resume_rejection(blocked_resume_context, "input_gate_profile_transition");
  blocked_resume_context.motion_allowed = false;
  expect_resume_rejection(blocked_resume_context, "estop_latched");

  blocked_resume_context = context;
  blocked_resume_context.input_ready = false;
  blocked_resume_context.input_gate_reason = "localization_health_stale";
  expect_resume_wait(blocked_resume_context, "resume_waiting_for_inputs");

  blocked_resume_context = context;
  blocked_resume_context.retained_path_ready = false;
  blocked_resume_context.retained_path_reason = "retained_global_path_missing";
  expect_resume_rejection(blocked_resume_context, "retained_global_path_missing");

  blocked_resume_context = context;
  blocked_resume_context.map_position.reset();
  expect_resume_wait(blocked_resume_context, "resume_waiting_for_pose");

  blocked_resume_context.odometry_ready = false;
  expect_resume_wait(blocked_resume_context, "resume_waiting_for_odometry");

  blocked_resume_context = context;
  blocked_resume_context.planner_map_configured = false;
  blocked_resume_context.planner_map_missing_reason = "active_octomap_not_configured";
  expect_resume_rejection(blocked_resume_context, "active_octomap_not_configured");

  success_current_map.reset();
  expect_resume_wait(context, "resume_waiting_for_map");
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
    overlay_context.temporary_overlay.blocked_regions.push_back({{2.0, 3.0, 0.5}, 0.7, -0.2, 1.8});
    require(overlay_controller.submit(request, overlay_context).accepted,
            "overlay fixture initial goal was rejected");
    lingtu::nav::endpoint::GoalPlanAdvanceResult overlay_completion;
    for (int i = 0; i < 1000 && !overlay_completion.path_activated; ++i) {
      overlay_completion = overlay_controller.advance(
          lingtu::nav::endpoint::GoalPlanAdvanceContext{context.frame_epoch, {}, 30.0});
      if (!overlay_completion.completion_consumed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    require(overlay_completion.path_activated && observed_requests.size() == 1U &&
                observed_requests.front().temporary_overlay.empty(),
            "ordinary planning unexpectedly carried a temporary overlay");

    auto invalid_replan = overlay_context;
    invalid_replan.map_position->x = std::numeric_limits<double>::quiet_NaN();
    require(overlay_controller.replanActive(invalid_replan).reason == "invalid_replan_admission" &&
                observed_requests.size() == 1U && !overlay_controller.snapshot().busy,
            "replan admission submitted an invalid start to the planner");

    require(overlay_controller.replanActive(overlay_context).accepted,
            "overlay replan was rejected");
    overlay_completion = {};
    for (int i = 0; i < 1000 && !overlay_completion.path_activated; ++i) {
      overlay_completion = overlay_controller.advance(
          lingtu::nav::endpoint::GoalPlanAdvanceContext{context.frame_epoch, {}, 31.0});
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
                observed_overlay.blocked_regions.size() == 1U &&
                observed_overlay.blocked_regions.front().radius_xy_m == 0.7,
            "GoalPlan did not preserve the request-scoped overlay across replan admission");
    require(overlay_activations.size() == 2U, "overlay replan activated a path more than once");
  }

  {
    std::vector<GoalPlanStatus> body_statuses;
    bool body_preempted = false;
    auto body_actions = make_admission_actions(&body_statuses, &body_preempted, true);
    std::vector<GoalPlanPathActivation> body_activations;
    std::vector<lingtu::nav::plan::GlobalPlanRequest> body_requests;
    body_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      body_activations.push_back(activation);
    };
    GoalPlanController body_controller(
        [&](const lingtu::nav::plan::GlobalPlanRequest &planned,
            const lingtu::nav::plan::GlobalPlanCancelCheck &) {
          body_requests.push_back(planned);
          auto result = planImmediately(planned, {});
          result.path = {{planned.start.x, planned.start.y, planned.start.z},
                         {planned.goal.x, planned.goal.y, planned.goal.z}};
          return result;
        }, std::move(body_actions));
    auto wait_body_completion = [&] {
      lingtu::nav::endpoint::GoalPlanAdvanceResult result;
      for (int i = 0; i < 1000 && !result.completion_consumed; ++i) {
        result = body_controller.advance({context.frame_epoch, {}, 50.0});
        if (!result.completion_consumed)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      require(result.completion_consumed, "body fixture planning did not complete");
      return result;
    };
    auto body = std::make_shared<lingtu::nav::collision::Body>();
    lingtu::nav::collision::Part box;
    box.link = "navigation_body";
    box.half_size = {0.38, 0.155, 0.17};
    body->parts.push_back(box);
    auto body_context = context;
    body_context.body = body;

    require(body_controller.submit(request, body_context).accepted,
            "body goal was rejected");
    require(wait_body_completion().path_activated && body_requests.size() == 1U,
            "body goal did not activate");
    require(!body_requests.front().start.yaw && !body_requests.front().goal.yaw,
            "global request must carry XYZ only");

    require(body_activations.size() == 1U && body_activations.front().body == body &&
                body_activations.front().path.size() == 2U &&
                body_activations.front().pose_points.empty() &&
                body_activations.front().goal_yaw == 0.25,
            "XYZ activation must retain the local body and terminal yaw without pose stages");

    auto fresh_body = std::make_shared<lingtu::nav::collision::Body>(*body);
    fresh_body->margin_m = 0.03;
    body_context.body = fresh_body;
    body_context.map_position = nav_kernel::Vec3{2.0, 3.0, 0.5};

    require(body_controller.replanActive(body_context).accepted,
            "body replan was rejected");
    require(wait_body_completion().path_activated && body_requests.size() == 2U,
            "body replan did not activate");
    require(body_requests.back().start.x == 2.0 &&
                !body_requests.back().start.yaw && !body_requests.back().goal.yaw &&
                body_activations.back().body == fresh_body &&
                body_activations.back().pose_points.empty(),
            "XYZ replan reused an old model or added mandatory heading stages");


    auto replacement = request;
    replacement.task_id = "replacement-body-task";
    replacement.request_id = "replacement-body-goal";
    replacement.target->yaw.reset();
    require(body_controller.submit(replacement, body_context).accepted,
            "replacement body goal was rejected");
    auto replacement_completion = wait_body_completion();
    require(replacement_completion.terminal_after_stop.has_value() &&
                !replacement_completion.path_activated && body_activations.size() == 2U,
            "replacement body route bypassed the active task stop barrier");
    require(!body_requests.back().goal.yaw && !body_requests.back().start.yaw,
            "replacement global request must carry XYZ only");
    replacement_completion.terminal_after_stop->commit();
    require(body_controller.activateDeferredReplacement(51.0, body_context).path_activated,
            "deferred replacement body route did not activate after stop");
    require(body_activations.size() == 3U && body_activations.back().body == fresh_body &&
                !body_activations.back().goal_yaw && body_activations.back().pose_points.empty(),
            "deferred XYZ replacement lost the local body or optional terminal yaw");

    std::vector<GoalPlanStatus> invalid_statuses;
    bool invalid_preempted = false;
    auto invalid_actions = make_admission_actions(&invalid_statuses, &invalid_preempted, true);
    bool invalid_activated = false;
    invalid_actions.activate_path = [&](const GoalPlanPathActivation &) {
      invalid_activated = true;
    };
    GoalPlanController invalid_body_controller(planImmediately, std::move(invalid_actions));
    require(invalid_body_controller.submit(replacement, body_context).accepted,
            "invalid body result fixture failed admission");
    lingtu::nav::endpoint::GoalPlanAdvanceResult invalid_completion;
    for (int i = 0; i < 1000 && !invalid_completion.completion_consumed; ++i) {
      invalid_completion = invalid_body_controller.advance({context.frame_epoch, {}, 52.0});
      if (!invalid_completion.completion_consumed)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(invalid_completion.completion_consumed && !invalid_completion.counted_failure &&
                invalid_activated,
            "XYZ-only result was incorrectly rejected because body headings were absent");
  }

  {
    std::vector<GoalPlanStatus> speed_statuses;
    std::vector<GoalPlanPathActivation> speed_activations;
    bool speed_preempted = false;
    auto speed_actions = make_admission_actions(&speed_statuses, &speed_preempted, true);
    speed_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      speed_activations.push_back(activation);
    };
    GoalPlanController speed_controller(planImmediately, std::move(speed_actions));
    auto speed_request = request;
    speed_request.target->max_speed_mps = 1.0;
    require(!speed_controller.setTaskSpeedLimit(request.task_id, 0.4, 1.0).first,
            "speed update accepted without an active task");
    require(speed_controller.submit(speed_request, context).accepted &&
                waitForCompletion(speed_controller, context.frame_epoch, 60.0).path_activated,
            "speed fixture did not activate");
    const auto before = speed_controller.snapshot();
    const auto status_count = speed_statuses.size();
    for (double invalid : {0.0, -1.0, 1.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
      require(!speed_controller.setTaskSpeedLimit(request.task_id, invalid, 1.0).first,
              "invalid speed limit was accepted");
    }
    require(!speed_controller.setTaskSpeedLimit("other-task", 0.4, 1.0).first,
            "speed update changed another task");
    require(speed_controller.setTaskSpeedLimit(request.task_id, 0.6, 1.0).first,
            "active task speed update was rejected");
    const auto after = speed_controller.snapshot();
    require(after.active_task_id == before.active_task_id &&
                after.active_request_id == before.active_request_id &&
                after.active_goal_epoch == before.active_goal_epoch && !after.busy &&
                speed_activations.size() == 1 && speed_statuses.size() == status_count,
            "speed update reset the route, lifecycle or task identity");
    require(speed_controller.replanActive(context).accepted,
            "speed fixture replan was rejected");
    require(speed_controller.setTaskSpeedLimit(request.task_id, 0.3, 1.0).first,
            "in-flight replan blocked the task speed update");
    require(waitForCompletion(speed_controller, context.frame_epoch, 61.0).path_activated &&
                speed_activations.back().max_speed_mps == 0.3,
            "in-flight replan restored its captured higher speed limit");
    require(speed_controller.replanActive(context).accepted &&
                waitForCompletion(speed_controller, context.frame_epoch, 62.0).path_activated &&
                speed_activations.back().max_speed_mps == 0.3,
            "subsequent replan lost the task speed limit");
    auto terminal = speed_controller.deferActiveTerminalWithTicket(
        NavigationGoalState::Reached, "goal_reached");
    terminal.commit();
    require(!speed_controller.setTaskSpeedLimit(request.task_id, 0.4, 1.0).first,
            "completed task accepted a speed update");
  }

  {
    GoalPlanRequest leg = request;
    leg.task_id = "inspection-task";
    leg.request_id = "inspection-start";
    leg.origin = GoalPlanOrigin::kInspection;
    std::uint64_t previous_epoch = 0;
    blocked_statuses.clear();
    for (int point = 0; point < 2; ++point) {
      require(blocked_controller.submit(leg, context).accepted, "inspection leg rejected");
      bool activated = false;
      for (int i = 0; i < 1000 && !activated; ++i) {
        activated = blocked_controller.advance({context.frame_epoch, {}, 40.0}).path_activated;
        if (!activated) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      require(activated, "inspection leg failed to activate");
      const auto epoch = blocked_controller.snapshot().active_goal_epoch;
      require(epoch > previous_epoch, "next waypoint reused the old planning epoch");
      previous_epoch = epoch;
      auto terminal = blocked_controller.deferActiveTerminalWithTicket(
          NavigationGoalState::Reached, "goal_reached");
      require(terminal.ticket.statuses.size() == 1 &&
                  terminal.ticket.statuses.front().origin == GoalPlanOrigin::kInspection,
              "terminal barrier lost the inspection origin");
      terminal.commit();
    }
    for (const auto &status : blocked_statuses) {
      require(status.task_id == leg.task_id && status.request_id == leg.request_id &&
                  status.origin == GoalPlanOrigin::kInspection,
              "inspection leg published a separate task identity or lost its owner");
    }
    require(blocked_controller.submit(leg, context).accepted, "third inspection leg rejected");
    auto cancelled = blocked_controller.deferAbortWithTicket("operator_cancel");
    require(!cancelled.ticket.statuses.empty(), "cancel lost planning status");
    for (const auto &status : cancelled.ticket.statuses) {
      require(status.origin == GoalPlanOrigin::kInspection, "cancel lost inspection ownership");
    }
    cancelled.commit();
  }
  return 0;
}

int main() {
  try {
    return runTests();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_goal_plan_controller: FAIL: %s\n", exc.what());
    return 1;
  }
}
