#include "app/goal/plan.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "input/gate.hpp"
#include "app/goal/retry.hpp"

namespace lingtu::nav::endpoint {
namespace {

bool retryableGlobalSearchFailure(const std::string &reason) {
  return reason == "empty_path" || reason == "no_path" || reason == "octomap_no_path" ||
         reason == "far_no_path" || reason == "global_planner_failed" ||
         reason == "goal_not_reached" || reason == "search_iteration_limit" ||
         reason == "support_disconnected" || reason == "start_pose_blocked" ||
         reason == "start_body_unknown" || reason == "start_body_occupied" ||
         reason == "start_temporary_overlay" || reason == "start_snap_exhausted" ||
         reason == "start_occupied" || reason == "start_unsupported" ||
         reason == "start_temporary_obstacle" ||
         reason == "goal_snap_exhausted" || reason == "start_outside_static_map" ||
         reason == "goal_outside_static_map";
}

}  // namespace

bool goalPlanInputGapIsRecoverable(const std::string &reason) noexcept {
  return reason == "recovering" || reason == "odom_missing" || reason == "odom_stale" ||
         reason == "odom_future" || reason == "odom_velocity_nonfinite" ||
         reason == "odom_velocity_out_of_bounds" || reason == "tf_missing" ||
         reason == "tf_future" || reason == "tf_stale" || reason == "cloud_missing" ||
         reason == "cloud_future" || reason == "cloud_stale" || localCollisionInputHold(reason) ||
         reason == "traversability_missing" ||
         reason == "traversability_future" || reason == "traversability_stale" ||
         reason == "localization_health_missing" || reason == "localization_health_future" ||
         reason == "localization_health_stale" || reason == "localization_not_tracking" ||
         reason == "localization_unhealthy" || reason == "localization_catastrophic" ||
         reason == "simulation_clock_missing" || reason == "simulation_clock_stale";
}

GoalPlanController::GoalPlanController(GlobalPlanTask::Planner planner, GoalPlanActions actions)
    : task_(std::move(planner)), actions_(std::move(actions)) {
  if (!actions_.preempt_rolling || !actions_.clear_external_inspection ||
      !actions_.current_map_identity || !actions_.publish_status || !actions_.inspection_active ||
      !actions_.inspection_leg_failed || !actions_.inspection_pause ||
      !actions_.inspection_plan_ready || !actions_.activate_path) {
    throw std::invalid_argument("GoalPlanController requires complete actions");
  }
}

GoalPlanTerminalAfterStop
GoalPlanController::terminalAfterStop(std::string reason,
                                      GoalPlanTerminalCommitWithTicket terminal) {
  return {std::move(reason), std::move(terminal.ticket), std::move(terminal.commit)};
}

std::pair<bool, std::string> GoalPlanController::setTaskSpeedLimit(
    const std::string &task_id, double max_speed_mps, double configured_max_speed_mps) {
  if (!std::isfinite(max_speed_mps) || max_speed_mps <= 0.0 ||
      max_speed_mps > configured_max_speed_mps) {
    return {false, "task_speed_limit_invalid"};
  }
  if (task_id.empty() || task_id != active_task_id_ || !active_target_) {
    return {false, "task_not_active"};
  }
  active_target_->max_speed_mps = max_speed_mps;
  if (planning_task_id_ == task_id && planning_target_) {
    planning_target_->max_speed_mps = max_speed_mps;
  }
  if (pending_plan_start_ && pending_plan_start_->request.task_id == task_id &&
      pending_plan_start_->request.target) {
    pending_plan_start_->request.target->max_speed_mps = max_speed_mps;
  }
  return {true, "task_speed_limit_updated"};
}

GoalPlanSubmitResult GoalPlanController::submit(const GoalPlanRequest &request,
                                                const GoalPlanAdmissionContext &context) {
  diagnostics_ = {};
  diagnostics_.seen = true;

  if (request.task_id.empty()) {
    return reject("goal_task_id_empty");
  }
  if (request.request_id.empty()) {
    return reject("goal_request_id_empty");
  }
  if (request.origin == GoalPlanOrigin::kExternal && actions_.inspection_active()) {
    return reject("inspection_run_active");
  }
  if (request.origin == GoalPlanOrigin::kExternal) {
    actions_.clear_external_inspection();
  }
  if (!context.motion_allowed) {
    return reject("estop_latched", true, true);
  }
  if (!context.hold_reason.empty()) {
    return reject(context.hold_reason, true, true);
  }
  if (!context.autonomy_mode) {
    return reject(std::string{"goal_not_allowed_in_"} + context.control_mode_name, true, true);
  }
  if (context.autonomy_request_not_before_s > 0.0 &&
      (!std::isfinite(request.source_stamp_s) || request.source_stamp_s <= 0.0 ||
       request.source_stamp_s <= context.autonomy_request_not_before_s)) {
    return reject("goal_predates_autonomy_resume", true, true);
  }
  if (!request.target) {
    return reject(request.decode_error.empty() ? "invalid_goal" : request.decode_error, true, true);
  }
  diagnostics_.goal = request.target->position;
  if (!context.planner_map_configured) {
    return reject(context.planner_map_missing_reason);
  }
  const bool projects_to_navigation_state =
      active_task_id_.empty() || active_task_id_ == request.task_id;
  const bool pose_gap = !context.map_position;
  if (request.origin == GoalPlanOrigin::kExternal &&
      (pending_plan_start_ || task_.busy() || pose_gap)) {
    pause_pending_ = false;
    active_paused_ = false;
    paused_task_id_.clear();
    if (task_.busy()) {
      task_.cancel();
    }
    if (pending_plan_start_) {
      publishPendingTerminal(lingtu::message::NavigationGoalState::Cancelled,
                             "superseded_by_new_goal");
    }
    // A replan belongs to the still-active path; only its search is superseded.
    if (!planning_is_replan_ && !planning_request_id_.empty()) {
      finishPlanning(lingtu::message::NavigationGoalState::Cancelled, "superseded_by_new_goal");
    }
    clearPlanningIdentity();
    const auto pending_goal_epoch = ++goal_epoch_;
    pending_plan_start_ =
        DeferredPlanStart{request, pending_goal_epoch, projects_to_navigation_state};
    publishStatus(request.task_id, request.request_id, pending_goal_epoch,
                  lingtu::message::NavigationGoalState::Planning, "planning_queued",
                  projects_to_navigation_state, request.origin);
    diagnostics_.reason = "planning_queued";
    return {true, "planning_queued", false, false, false, std::nullopt};
  }
  if (!context.map_position) {
    return reject(context.odometry_ready ? "map_odom_tf_not_ready" : "odometry_not_ready");
  }
  diagnostics_.start = *context.map_position;
  if (task_.busy()) {
    return reject("global_planner_busy");
  }

  if (context.rolling_segment_active && !actions_.preempt_rolling("superseded_by_generic_goal")) {
    return reject("segment_preempt_zero_publish_failed", false, true);
  }

  pause_pending_ = false;
  active_paused_ = false;
  paused_task_id_.clear();
  return startPlanning(request.task_id, request.request_id, *request.target, request.origin,
                       context, "planning", false, std::nullopt, projects_to_navigation_state,
                       request.source_stamp_s);
}

GoalPlanSubmitResult GoalPlanController::replanActive(const GoalPlanAdmissionContext &context) {
  diagnostics_ = {};
  diagnostics_.seen = true;

  if (!std::isfinite(context.autonomy_request_not_before_s) ||
      (context.map_position &&
       (!std::isfinite(context.map_position->x) || !std::isfinite(context.map_position->y) ||
        !std::isfinite(context.map_position->z)))) {
    return reject("invalid_replan_admission");
  }
  if (active_task_id_.empty() || active_request_id_.empty() || !active_target_) {
    return reject("no_active_external_goal");
  }
  if (active_origin_ != GoalPlanOrigin::kInspection && actions_.inspection_active()) {
    return reject("inspection_replan_not_supported");
  }
  if (task_.busy()) {
    return reject("global_planner_busy");
  }
  if (!context.motion_allowed) {
    return reject("estop_latched", true, true);
  }
  if (!context.hold_reason.empty()) {
    return reject(context.hold_reason, true, true);
  }
  if (!context.autonomy_mode) {
    return reject(std::string{"goal_not_allowed_in_"} + context.control_mode_name, true, true);
  }
  if (!context.map_position) {
    return reject(context.odometry_ready ? "map_odom_tf_not_ready" : "odometry_not_ready");
  }
  diagnostics_.start = *context.map_position;
  diagnostics_.goal = active_target_->position;
  if (!context.planner_map_configured) {
    return reject(context.planner_map_missing_reason);
  }
  if (!active_map_identity_ || !active_map_identity_->valid()) {
    return reject("active_goal_map_identity_missing", false, true);
  }

  GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
  if (current_map.identity && current_map.identity->valid() &&
      !lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
    return reject("active_map_changed_before_replan", false, true);
  }

  return startPlanning(active_task_id_, active_request_id_, *active_target_, active_origin_,
                       context, "replanning_after_local_recovery", true, std::nullopt, true,
                       active_source_stamp_s_);
}

GoalPlanSubmitResult GoalPlanController::startPlanning(
    const std::string &task_id, const std::string &request_id, const GoalPlanTarget &target,
    GoalPlanOrigin origin, const GoalPlanAdmissionContext &context,
    const std::string &planning_reason, bool is_replan,
    std::optional<std::uint64_t> reserved_goal_epoch, bool project_planning_to_navigation_state,
    double source_stamp_s) {
  if (reserved_goal_epoch) {
    goal_epoch_ = std::max(goal_epoch_, *reserved_goal_epoch);
  } else {
    ++goal_epoch_;
  }
  GlobalPlanContext plan_context;
  plan_context.request_id = request_id;
  plan_context.start = *context.map_position;
  plan_context.goal = target.position;
  plan_context.goal_yaw = target.yaw;
  plan_context.max_speed_mps = target.max_speed_mps;
  plan_context.acceptance_radius_m = target.acceptance_radius_m;
  plan_context.goal_epoch = goal_epoch_;
  plan_context.frame_epoch = context.frame_epoch;
  plan_context.request.start = {
      plan_context.start.x,
      plan_context.start.y,
      plan_context.start.z,
  };
  plan_context.request.goal = {
      plan_context.goal.x,
      plan_context.goal.y,
      plan_context.goal.z,
  };
  plan_context.local_body = context.body;
  plan_context.request.options = is_replan ? active_planner_options_ : context.planner_options;
  if (target.acceptance_radius_m) {
    plan_context.request.options.terminal_goal_xy_tolerance_m = std::min(
        plan_context.request.options.terminal_goal_xy_tolerance_m, *target.acceptance_radius_m);
  }
  if (is_replan) {
    plan_context.request.temporary_overlay = context.temporary_overlay;
  }

  if (!task_.start(std::move(plan_context))) {
    return reject("global_planner_busy");
  }

  planning_task_id_ = task_id;
  planning_request_id_ = request_id;
  planning_goal_epoch_ = goal_epoch_;
  planning_is_replan_ = is_replan;
  planning_replan_attempt_ = is_replan ? ++replan_attempt_ : 0U;
  planning_projects_to_navigation_state_ = project_planning_to_navigation_state;
  planning_origin_ = origin;
  planning_target_ = target;
  planning_source_stamp_s_ = source_stamp_s;
  publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_,
                lingtu::message::NavigationGoalState::Planning, planning_reason,
                project_planning_to_navigation_state, origin);
  diagnostics_.reason = planning_reason;
  return {true,        is_replan ? "replan_started" : "planning_started", false, false, false,
          std::nullopt};
}

GoalPlanSubmitResult
GoalPlanController::resumePending(const GoalPlanAdmissionContext &fresh_context, double now_s) {
  diagnostics_ = {};
  diagnostics_.seen = true;

  if (!pending_plan_start_) {
    return reject("no_pending_plan");
  }
  if (pause_pending_) {
    return reject("pause_pending");
  }
  if (active_paused_ && paused_task_id_ == pending_plan_start_->request.task_id) {
    return reject("task_paused");
  }
  if (task_.busy()) {
    return reject("pending_planner_not_drained");
  }

  auto reject_pending = [this](lingtu::message::NavigationGoalState terminal_state,
                               std::string reason, bool count_frame_rejection,
                               bool record_frame_error) {
    publishPendingTerminal(terminal_state, reason);
    pending_plan_start_.reset();
    return reject(std::move(reason), count_frame_rejection, record_frame_error);
  };

  const auto &pending_map = pending_plan_start_->map_identity
                                ? pending_plan_start_->map_identity
                                : active_map_identity_;
  if (pending_map && pending_map->valid()) {
    GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
    if (!current_map.identity || !current_map.identity->valid()) {
      diagnostics_.accepted = false;
      diagnostics_.reason = "active_map_unavailable_before_pending_plan";
      return {false, diagnostics_.reason, false, false, true, std::nullopt};
    }
    if (!lingtu::nav::plan::sameMapIdentity(*pending_map, *current_map.identity)) {
      return reject_pending(lingtu::message::NavigationGoalState::Failed,
                            "active_map_changed_before_pending_plan", false, true);
    }
  }

  const DeferredPlanStart &pending = *pending_plan_start_;
  if (!fresh_context.motion_allowed) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled, "estop_latched", true,
                          true);
  }
  if (!fresh_context.hold_reason.empty()) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          fresh_context.hold_reason, true, true);
  }
  if (!fresh_context.autonomy_mode) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          std::string{"goal_not_allowed_in_"} + fresh_context.control_mode_name,
                          true, true);
  }
  if (!fresh_context.input_ready && !localCollisionInputHold(fresh_context.input_gate_reason)) {
    if (goalPlanInputGapIsRecoverable(fresh_context.input_gate_reason)) {
      // Keep the queued plan and wait for the next valid input frame. A
      // transient sensor gap must not turn a stopped replacement into a
      // terminal task result.
      diagnostics_.accepted = false;
      diagnostics_.reason = inputGateStopReason(fresh_context.input_gate_reason);
      return {false, diagnostics_.reason, false, false, true, std::nullopt};
    }
    return reject_pending(lingtu::message::NavigationGoalState::Failed,
                          inputGateStopReason(fresh_context.input_gate_reason),
                          false, true);
  }
  if (!std::isfinite(fresh_context.autonomy_request_not_before_s)) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed, "invalid_replan_admission",
                          false, true);
  }
  if (fresh_context.autonomy_request_not_before_s > 0.0 &&
      (!std::isfinite(pending.request.source_stamp_s) || pending.request.source_stamp_s <= 0.0 ||
       (pending.request.source_stamp_s <= fresh_context.autonomy_request_not_before_s &&
        !(pending.resumed_after_pause &&
          pending.request.source_stamp_s == fresh_context.autonomy_request_not_before_s)))) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          "goal_predates_autonomy_resume", true, true);
  }
  if (!pending.request.target) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed,
                          pending.request.decode_error.empty() ? "invalid_goal"
                                                               : pending.request.decode_error,
                          true, true);
  }
  diagnostics_.goal = pending.request.target->position;
  if (!fresh_context.map_position) {
    diagnostics_.accepted = false;
    diagnostics_.reason = fresh_context.odometry_ready ? "map_odom_tf_not_ready"
                                                        : "odometry_not_ready";
    return {false, diagnostics_.reason, false, false, false, std::nullopt};
  }
  if (!std::isfinite(fresh_context.map_position->x) ||
      !std::isfinite(fresh_context.map_position->y) ||
      !std::isfinite(fresh_context.map_position->z)) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed, "invalid_replan_admission",
                          false, true);
  }
  diagnostics_.start = *fresh_context.map_position;
  if (!fresh_context.planner_map_configured) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed,
                          fresh_context.planner_map_missing_reason, false, false);
  }
  if (now_s < pending.retry_not_before_s) {
    diagnostics_.reason = "planning_waiting";
    return {false, "planning_waiting", false, false, false, std::nullopt};
  }
  if (fresh_context.rolling_segment_active &&
      !actions_.preempt_rolling("superseded_by_generic_goal")) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          "segment_preempt_zero_publish_failed", false, true);
  }

  DeferredPlanStart ready = std::move(*pending_plan_start_);
  pending_plan_start_.reset();
  return startPlanning(ready.request.task_id, ready.request.request_id, *ready.request.target,
                       ready.request.origin, fresh_context, "planning", false, ready.goal_epoch,
                       active_task_id_.empty() || active_task_id_ == ready.request.task_id,
                       ready.request.source_stamp_s);
}

void GoalPlanController::clearPlanningIdentity() {
  planning_task_id_.clear();
  planning_request_id_.clear();
  planning_goal_epoch_ = 0U;
  planning_is_replan_ = false;
  planning_projects_to_navigation_state_ = true;
  planning_replan_attempt_ = 0U;
  planning_origin_ = GoalPlanOrigin::kExternal;
  planning_target_.reset();
  planning_source_stamp_s_ = 0.0;
}

GoalPlanAdvanceResult GoalPlanController::advance(const GoalPlanAdvanceContext &context) {
  GoalPlanAdvanceResult advance_result;
  if (pause_pending_ || active_paused_) {
    // A paused search was cancelled with discard_result set. Poll only to
    // release its worker; never interpret or activate its late completion.
    (void)task_.poll();
    return advance_result;
  }
  auto completion = task_.poll();
  if (!completion) {
    return advance_result;
  }

  advance_result.completion_consumed = true;
  const auto &plan_result = completion->result;
  diagnostics_ = {};
  diagnostics_.seen = true;
  diagnostics_.start = completion->context.start;
  diagnostics_.goal = completion->context.goal;
  diagnostics_.reached_goal = plan_result.reached_goal;
  diagnostics_.goal_error_m = plan_result.goal_error_m;
  diagnostics_.elapsed_ms = plan_result.elapsed_ms;
  advance_result.elapsed_ms = plan_result.elapsed_ms;
  const bool completing_replan = planning_is_replan_;

  GoalPlanMapIdentityResult current_map;
  if (completion->context.goal_epoch == goal_epoch_ &&
      completion->context.frame_epoch == context.frame_epoch && completion->error.empty() &&
      !plan_result.cancelled) {
    current_map = actions_.current_map_identity();
    advance_result.map_identity_error = current_map.reason;
  }
  std::string stale_reason =
      globalPlanStaleReason(*completion, goal_epoch_, context.frame_epoch, current_map.identity);
  if (stale_reason.empty() && completing_replan && active_map_identity_ &&
      plan_result.map_identity.valid() &&
      !lingtu::nav::plan::sameMapIdentity(*active_map_identity_, plan_result.map_identity)) {
    stale_reason = "active_map_changed_during_replan";
  }
  if (!stale_reason.empty()) {
    diagnostics_.accepted = false;
    diagnostics_.reason = stale_reason;
    advance_result.counted_failure = true;
    advance_result.record_frame_error = true;
    const bool map_invalidated_plan = stale_reason == "planner_map_identity_missing" ||
                                      stale_reason == "active_map_changed_during_planning" ||
                                      stale_reason == "active_map_changed_during_replan";
    if (completing_replan) {
      clearPlanningIdentity();
      advance_result.terminal_after_stop = terminalAfterStop(
          stale_reason, deferActiveTerminalWithTicket(
                            map_invalidated_plan ? lingtu::message::NavigationGoalState::Failed
                                                 : lingtu::message::NavigationGoalState::Cancelled,
                            stale_reason));
      ++goal_epoch_;
      task_.cancel();
      return advance_result;
    }
    const bool planning_projects_to_navigation_state = planning_projects_to_navigation_state_;
    finishPlanning(map_invalidated_plan ? lingtu::message::NavigationGoalState::Failed
                                        : lingtu::message::NavigationGoalState::Cancelled,
                   stale_reason);
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(stale_reason, context.now_s);
    }
    if (planning_projects_to_navigation_state && !active_request_id_.empty()) {
      advance_result.terminal_after_stop = terminalAfterStop(
          stale_reason, deferActiveTerminalWithTicket(
                            lingtu::message::NavigationGoalState::Cancelled, stale_reason));
    }
    ++goal_epoch_;
    task_.cancel();
    return advance_result;
  }

  if (!completion->error.empty()) {
    std::cerr << "nav_native: global planner exception: " << completion->error << '\n';
    diagnostics_.reason = "global_planner_exception";
    advance_result.counted_failure = true;
    if (completing_replan) {
      clearPlanningIdentity();
      advance_result.terminal_after_stop =
          terminalAfterStop(diagnostics_.reason,
                            deferActiveTerminalWithTicket(
                                lingtu::message::NavigationGoalState::Failed, diagnostics_.reason));
      return advance_result;
    }
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(diagnostics_.reason, context.now_s);
    }
    return advance_result;
  }

  if (!plan_result.ok || !plan_result.reached_goal) {
    diagnostics_.reason = plan_result.ok
                              ? "goal_not_reached"
                              : (plan_result.failure_reason.empty() ? "global_planner_failed"
                                                                    : plan_result.failure_reason);
    diagnostics_.waypoints = plan_result.path.size();
    advance_result.counted_failure = true;
    if (completing_replan) {
      const auto planning_goal_epoch = planning_goal_epoch_;
      clearPlanningIdentity();
      if (plan_result.cancelled || !context.hold_reason.empty()) {
        advance_result.terminal_after_stop =
            terminalAfterStop(diagnostics_.reason,
                              deferActiveTerminalWithTicket(
                                  lingtu::message::NavigationGoalState::Failed, diagnostics_.reason));
      } else {
        publishStatus(active_task_id_, active_request_id_, planning_goal_epoch,
                      lingtu::message::NavigationGoalState::PathActive, "replan_waiting", true,
                      active_origin_);
      }
      return advance_result;
    }
    if (planning_target_ && context.hold_reason.empty() &&
        (completion->timed_out ||
         (!plan_result.cancelled && retryableGlobalSearchFailure(diagnostics_.reason)))) {
      GoalPlanRequest retry_request;
      retry_request.task_id = planning_task_id_;
      retry_request.request_id = planning_request_id_;
      retry_request.origin = planning_origin_;
      retry_request.source_stamp_s = planning_source_stamp_s_;
      retry_request.target = planning_target_;
      auto retry_map = plan_result.map_identity.valid()
                           ? std::optional{plan_result.map_identity}
                           : actions_.current_map_identity().identity;
      pending_plan_start_ = DeferredPlanStart{
          std::move(retry_request), planning_goal_epoch_, planning_projects_to_navigation_state_,
          false, context.now_s + BoundedGoalReplanConfig{}.backoff_s, std::move(retry_map)};
      publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_,
                    lingtu::message::NavigationGoalState::Planning, "planning_waiting",
                    planning_projects_to_navigation_state_, planning_origin_);
      clearPlanningIdentity();
      return advance_result;
    }
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(diagnostics_.reason, context.now_s);
    }
    return advance_result;
  }

  if (!context.hold_reason.empty()) {
    diagnostics_.accepted = false;
    diagnostics_.reason = context.hold_reason;
    if (completing_replan) {
      clearPlanningIdentity();
      advance_result.terminal_after_stop = terminalAfterStop(
          diagnostics_.reason,
          deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState::Cancelled,
                                        diagnostics_.reason));
      return advance_result;
    }
    finishPlanning(lingtu::message::NavigationGoalState::Cancelled, diagnostics_.reason);
    if (actions_.inspection_active()) {
      actions_.inspection_pause(diagnostics_.reason);
    }
    return advance_result;
  }

  std::vector<nav_kernel::Vec3> global_path = globalPlanPath(*completion);
  if (global_path.empty()) {
    diagnostics_.reason = "empty_path";
    advance_result.counted_failure = true;
    if (completing_replan) {
      clearPlanningIdentity();
      advance_result.terminal_after_stop =
          terminalAfterStop(diagnostics_.reason,
                            deferActiveTerminalWithTicket(
                                lingtu::message::NavigationGoalState::Failed, diagnostics_.reason));
      return advance_result;
    }
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(diagnostics_.reason, context.now_s);
    }
    return advance_result;
  }
  GoalPlanInspectionDecision inspection_decision;
  if (actions_.inspection_active()) {
    inspection_decision = actions_.inspection_plan_ready(context.now_s);
    if (!inspection_decision.accepted) {
      diagnostics_.accepted = false;
      diagnostics_.reason = inspection_decision.reason.empty() ? "inspection_plan_rejected"
                                                               : inspection_decision.reason;
      if (completing_replan) {
        clearPlanningIdentity();
        advance_result.terminal_after_stop = terminalAfterStop(
            diagnostics_.reason,
            deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState::Cancelled,
                                          diagnostics_.reason));
        ++goal_epoch_;
        task_.cancel();
        advance_result.inspection_status_changed = true;
        return advance_result;
      }
      const bool planning_projects_to_navigation_state = planning_projects_to_navigation_state_;
      finishPlanning(lingtu::message::NavigationGoalState::Cancelled, diagnostics_.reason);
      if (planning_projects_to_navigation_state && !active_request_id_.empty()) {
        advance_result.terminal_after_stop = terminalAfterStop(
            diagnostics_.reason,
            deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState::Cancelled,
                                          diagnostics_.reason));
      }
      ++goal_epoch_;
      task_.cancel();
      advance_result.inspection_status_changed = true;
      return advance_result;
    }
  }

  diagnostics_.waypoints = global_path.size();
  GoalPlanPathActivation activation{
      global_path,
      completion->context.goal_yaw,
      inspection_decision.tolerance,
      plan_result.map_identity,
      context.now_s,
      completion->context.max_speed_mps,
      completion->context.acceptance_radius_m,
      {},
      completion->context.local_body,
  };
  // A running replan captured the previous limit when its worker started.
  if (completing_replan && active_target_) {
    activation.max_speed_mps = active_target_->max_speed_mps;
  }
  const auto &support_options = completion->context.request.options;
  activation.requested_goal = completion->context.goal;
  if (support_options.ground_support && std::isfinite(support_options.support_height_m) &&
      support_options.support_height_m > 0.0 &&
      std::isfinite(support_options.support_height_tolerance_m) &&
      support_options.support_height_tolerance_m >= 0.0) {
    activation.support_height_m = support_options.support_height_m;
    activation.support_height_tolerance_m = support_options.support_height_tolerance_m;
  }
  const bool completing_replacement =
      !completing_replan && !planning_projects_to_navigation_state_ && !active_task_id_.empty() &&
      planning_task_id_ != active_task_id_;
  if (completing_replacement) {
    deferred_replacement_activation_ = DeferredReplacementActivation{
        planning_task_id_,
        planning_request_id_,
        planning_goal_epoch_,
        std::move(activation),
        GoalPlanTarget{completion->context.goal, completion->context.goal_yaw,
                       completion->context.max_speed_mps, completion->context.acceptance_radius_m},
        planning_origin_,
        planning_source_stamp_s_,
        completion->context.request.options,
    };
    advance_result.terminal_after_stop = terminalAfterStop(
        "superseded_by_new_goal",
        deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState::Cancelled,
                                      "superseded_by_new_goal"));
    clearPlanningIdentity();
    diagnostics_.accepted = true;
    diagnostics_.reason = "replacement_plan_ready";
    return advance_result;
  }

  actions_.activate_path(activation);
  if (!completing_replan) {
    finishActive(lingtu::message::NavigationGoalState::Cancelled, "superseded_by_new_goal");
    replan_attempt_ = 0U;
  }
  active_task_id_ = planning_task_id_;
  active_request_id_ = planning_request_id_;
  active_goal_epoch_ = planning_goal_epoch_;
  active_paused_ = false;
  pause_pending_ = false;
  paused_task_id_.clear();
  active_map_identity_ = plan_result.map_identity;
  active_target_ = GoalPlanTarget{completion->context.goal, completion->context.goal_yaw,
                       activation.max_speed_mps, completion->context.acceptance_radius_m};
  active_source_stamp_s_ = planning_source_stamp_s_;
  active_origin_ = planning_origin_;
  active_planner_options_ = completion->context.request.options;
  planning_projects_to_navigation_state_ = true;

  finishPlanning(lingtu::message::NavigationGoalState::PathActive,
                 completing_replan ? "path_replanned_after_local_recovery" : "path_active");
  diagnostics_.accepted = true;
  diagnostics_.reason = completing_replan ? "replan_accepted" : "accepted";
  advance_result.path_activated = true;
  return advance_result;
}

GoalPlanAdvanceResult
GoalPlanController::activateDeferredReplacement(double now_s,
                                                const GoalPlanAdmissionContext &fresh_context) {
  GoalPlanAdvanceResult result;
  if (!deferred_replacement_activation_ || !active_task_id_.empty()) {
    return result;
  }

  if (!fresh_context.motion_allowed) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         "estop_latched");
  }
  if (!fresh_context.hold_reason.empty()) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         fresh_context.hold_reason);
  }
  if (!fresh_context.autonomy_mode) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         std::string{"goal_not_allowed_in_"} +
                                             fresh_context.control_mode_name);
  }
  if (!fresh_context.input_ready) {
    const std::string &reason = fresh_context.input_gate_reason;
    if (goalPlanInputGapIsRecoverable(reason)) {
      deferred_replacement_activation_->replan_after_input_gap = true;
      diagnostics_.accepted = false;
      diagnostics_.reason = inputGateStopReason(reason);
      return result;
    }
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         inputGateStopReason(fresh_context.input_gate_reason));
  }
  if (!fresh_context.map_position) {
    diagnostics_.accepted = false;
    diagnostics_.reason = fresh_context.odometry_ready ? "map_odom_tf_not_ready"
                                                        : "odometry_not_ready";
    return result;
  }
  if (!std::isfinite(fresh_context.map_position->x) ||
      !std::isfinite(fresh_context.map_position->y) ||
      !std::isfinite(fresh_context.map_position->z)) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "invalid_replan_admission");
  }
  if (!fresh_context.planner_map_configured) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         fresh_context.planner_map_missing_reason);
  }
  if (deferred_replacement_activation_->activation.path.empty()) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "replacement_path_missing");
  }

  DeferredReplacementActivation ready = std::move(*deferred_replacement_activation_);
  if (!ready.activation.map_identity || !ready.activation.map_identity->valid()) {
    deferred_replacement_activation_ = std::move(ready);
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "replacement_map_identity_missing");
  }
  GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
  if (!current_map.identity || !current_map.identity->valid()) {
    deferred_replacement_activation_ = std::move(ready);
    diagnostics_.accepted = false;
    diagnostics_.reason = "active_map_unavailable_before_replacement_activation";
    return result;
  }
  if (!lingtu::nav::plan::sameMapIdentity(*ready.activation.map_identity, *current_map.identity)) {
    deferred_replacement_activation_ = std::move(ready);
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "active_map_changed_before_replacement_activation");
  }

  if (ready.replan_after_input_gap) {
    // The robot may have moved while braking. Retain the new goal identity,
    // but rebuild its route from the fresh stopped pose before activating it.
    GoalPlanAdmissionContext recovered = fresh_context;
    recovered.planner_options = ready.planner_options;
    const auto restarted = startPlanning(
        ready.task_id, ready.request_id, ready.target, ready.origin, recovered,
        "replacement_replanning_after_input_recovery", false, ready.goal_epoch, true,
        ready.source_stamp_s);
    if (!restarted.accepted) {
      deferred_replacement_activation_ = std::move(ready);
      return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                           restarted.reason);
    }
    deferred_replacement_activation_.reset();
    return result;
  }

  deferred_replacement_activation_.reset();
  ready.activation.stamp_s = now_s;
  actions_.activate_path(ready.activation);

  active_task_id_ = ready.task_id;
  active_request_id_ = ready.request_id;
  active_goal_epoch_ = ready.goal_epoch;
  active_paused_ = false;
  pause_pending_ = false;
  paused_task_id_.clear();
  active_map_identity_ = ready.activation.map_identity;
  active_target_ = ready.target;
  active_source_stamp_s_ = ready.source_stamp_s;
  active_origin_ = ready.origin;
  active_planner_options_ = ready.planner_options;
  replan_attempt_ = 0U;
  diagnostics_.seen = true;
  diagnostics_.accepted = true;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = "replacement_plan_completed";
  diagnostics_.waypoints = ready.activation.path.size();
  diagnostics_.goal = ready.target.position;
  publishStatus(active_task_id_, active_request_id_, active_goal_epoch_,
                lingtu::message::NavigationGoalState::PathActive, "replacement_plan_completed",
                true, active_origin_);
  result.path_activated = true;
  return result;
}

GoalPlanAdvanceResult
GoalPlanController::failDeferredReplacement(lingtu::message::NavigationGoalState state,
                                            const std::string &reason) {
  return failDeferredReplacementLocked(state, reason);
}

GoalPlanAdvanceResult
GoalPlanController::failDeferredReplacementLocked(lingtu::message::NavigationGoalState state,
                                                  const std::string &reason) {
  GoalPlanAdvanceResult result;
  if (!deferred_replacement_activation_) {
    return result;
  }
  DeferredReplacementActivation failed = std::move(*deferred_replacement_activation_);
  deferred_replacement_activation_.reset();
  diagnostics_.seen = true;
  diagnostics_.accepted = false;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = reason;
  diagnostics_.goal = failed.target.position;
  diagnostics_.waypoints = failed.activation.path.size();
  result.counted_failure = state == lingtu::message::NavigationGoalState::Failed;
  result.terminal_after_stop = terminalAfterStop(
      reason, GoalPlanTerminalCommitWithTicket{
                  GoalPlanTerminalDeliveryTicket{{GoalPlanStatus{
                      failed.task_id, failed.request_id, failed.goal_epoch, state, reason, false, failed.origin}}},
                  terminalCommit({GoalPlanStatus{failed.task_id, failed.request_id,
                                                 failed.goal_epoch, state, reason, false, failed.origin}})});
  return result;
}
GoalPlanTaskTransition GoalPlanController::deferPause(const std::string &task_id,
                                                      const std::string &request_id,
                                                      const std::string &reason) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (pause_pending_) {
    return {false, "task_pause_pending", {}};
  }
  const bool pending_match =
      pending_plan_start_ && pending_plan_start_->request.task_id == task_id;
  const bool planning_match = task_.busy() && planning_task_id_ == task_id;
  const bool active_match = !active_task_id_.empty() && active_task_id_ == task_id;
  if (!pending_match && !planning_match && !active_match) {
    return {false, "task_not_active", {}};
  }
  if (active_paused_ && paused_task_id_ == task_id) {
    return {false, "task_already_paused", {}};
  }
  if (task_.busy() && active_match && !planning_match) {
    return {false, "task_not_executing", {}};
  }

  if (planning_match) {
    if (active_match) {
      // A retry belongs to the retained active task. Cancel only the search;
      // the active route remains the resumable task identity.
      task_.cancel();
      clearPlanningIdentity();
    } else {
      if (!planning_target_) {
        return {false, "planning_target_missing", {}};
      }
      GoalPlanRequest paused_request;
      paused_request.task_id = planning_task_id_;
      paused_request.request_id = planning_request_id_;
      paused_request.origin = planning_origin_;
      paused_request.source_stamp_s = planning_source_stamp_s_;
      paused_request.target = *planning_target_;
      pending_plan_start_ = DeferredPlanStart{std::move(paused_request), planning_goal_epoch_,
                                              planning_projects_to_navigation_state_};
      task_.cancel();
      clearPlanningIdentity();
    }
  }
  if (task_.busy() && !planning_match) {
    if (!pending_match) {
      return {false, "task_not_executing", {}};
    }
  }

  const std::string pause_reason = reason.empty() ? "operator_pause" : reason;
  pause_pending_ = true;
  paused_task_id_ = task_id;
  return {
      true,
      "pause_ready",
      [this, task_id, request_id, pause_reason] {
        if (!pause_pending_ || paused_task_id_ != task_id || active_paused_) {
          return;
        }
        std::uint64_t goal_epoch = 0U;
        bool project_to_navigation_state = true;
        GoalPlanOrigin origin = GoalPlanOrigin::kExternal;
        if (pending_plan_start_ && pending_plan_start_->request.task_id == task_id) {
          pending_plan_start_->request.request_id = request_id;
          goal_epoch = pending_plan_start_->goal_epoch;
          project_to_navigation_state = pending_plan_start_->project_to_navigation_state;
          origin = pending_plan_start_->request.origin;
        } else if (active_task_id_ == task_id) {
          active_request_id_ = request_id;
          goal_epoch = active_goal_epoch_;
          origin = active_origin_;
        } else {
          pause_pending_ = false;
          paused_task_id_.clear();
          return;
        }
        pause_pending_ = false;
        active_paused_ = true;
        diagnostics_.seen = true;
        diagnostics_.accepted = true;
        diagnostics_.reached_goal = false;
        diagnostics_.reason = pause_reason;
        publishStatus(task_id, request_id, goal_epoch,
                      lingtu::message::NavigationGoalState::Paused, pause_reason,
                      project_to_navigation_state, origin);
      },
  };
}

GoalPlanTaskTransition GoalPlanController::deferResume(const std::string &task_id,
                                                       const std::string &request_id,
                                                       const GoalPlanAdmissionContext &context) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (pause_pending_) {
    return {false, "task_pause_pending", {}};
  }
  const bool pending_match =
      pending_plan_start_ && pending_plan_start_->request.task_id == task_id;
  const bool active_match = !active_task_id_.empty() && active_task_id_ == task_id;
  if (!pending_match && !active_match) {
    return {false, "task_not_active", {}};
  }
  if (!active_paused_ || paused_task_id_ != task_id) {
    return {false, "task_not_paused", {}};
  }
  if (task_.busy() && active_match) {
    return {false, "task_not_executing", {}};
  }
  const bool resume_waiting_for_replan =
      active_match && active_origin_ == GoalPlanOrigin::kExternal && active_target_.has_value() &&
      !context.retained_path_ready && context.replan_pending;
  const auto make_resume_transition =
      [this, task_id, request_id, pending_match, active_match,
       resume_waiting_for_replan,
       resume_source_stamp_s = context.autonomy_request_not_before_s](std::string reason) {
        return GoalPlanTaskTransition{
            true,
            std::move(reason),
            [this, task_id, request_id, pending_match, active_match,
             resume_waiting_for_replan, resume_source_stamp_s] {
              if (!active_paused_ || paused_task_id_ != task_id) {
                return;
              }
              std::uint64_t goal_epoch = 0U;
              GoalPlanOrigin origin = GoalPlanOrigin::kExternal;
              bool project_to_navigation_state = true;
              lingtu::message::NavigationGoalState state =
                  lingtu::message::NavigationGoalState::PathActive;
              if (pending_match && pending_plan_start_ &&
                  pending_plan_start_->request.task_id == task_id) {
                pending_plan_start_->request.request_id = request_id;
                pending_plan_start_->request.source_stamp_s = resume_source_stamp_s;
                pending_plan_start_->resumed_after_pause = true;
                goal_epoch = pending_plan_start_->goal_epoch;
                origin = pending_plan_start_->request.origin;
                project_to_navigation_state = pending_plan_start_->project_to_navigation_state;
                state = lingtu::message::NavigationGoalState::Planning;
              } else if (active_match && active_task_id_ == task_id) {
                active_request_id_ = request_id;
                goal_epoch = active_goal_epoch_;
                origin = active_origin_;
              } else {
                return;
              }
              active_paused_ = false;
              paused_task_id_.clear();
              diagnostics_.seen = true;
              diagnostics_.accepted = true;
              diagnostics_.reached_goal = false;
              diagnostics_.reason = pending_match
                                        ? "planning_queued"
                                        : (resume_waiting_for_replan ? "replan_waiting"
                                                                     : "path_resumed");
              publishStatus(task_id, request_id, goal_epoch, state, diagnostics_.reason,
                            project_to_navigation_state, origin);
            },
        };
      };
  if (!context.motion_allowed) {
    return {false, "estop_latched", {}};
  }
  if (!context.hold_reason.empty()) {
    return {false, context.hold_reason, {}};
  }
  if (!context.autonomy_mode) {
    return {false, std::string{"resume_not_allowed_in_"} + context.control_mode_name, {}};
  }
  if (!context.input_ready) {
    if (goalPlanInputGapIsRecoverable(context.input_gate_reason)) {
      return make_resume_transition("resume_waiting_for_inputs");
    }
    return {false,
            inputGateStopReason(context.input_gate_reason),
            {}};
  }
  if (active_match && !context.retained_path_ready && !resume_waiting_for_replan) {
    return {false,
            context.retained_path_reason.empty() ? "retained_global_path_missing"
                                                 : context.retained_path_reason,
            {}};
  }
  if (!context.map_position) {
    return make_resume_transition(context.odometry_ready ? "resume_waiting_for_pose"
                                                         : "resume_waiting_for_odometry");
  }
  if (!context.planner_map_configured) {
    return {false, context.planner_map_missing_reason, {}};
  }
  if (active_match && (!active_map_identity_ || !active_map_identity_->valid())) {
    return {false, "active_goal_map_identity_missing", {}};
  }
  if (active_match) {
    const GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
    if (!current_map.identity || !current_map.identity->valid()) {
      return make_resume_transition("resume_waiting_for_map");
    }
    if (!lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
      return {false, "active_map_changed_before_resume", {}};
    }
  }

  return make_resume_transition("resume_ready");
}
GoalPlanTaskTransition GoalPlanController::deferCancelPending(const std::string &task_id,
                                                              const std::string &request_id,
                                                              const std::string &reason) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (!pending_plan_start_ || pending_plan_start_->request.task_id != task_id) {
    return {false, "task_not_pending", {}};
  }

  const std::uint64_t pending_goal_epoch = pending_plan_start_->goal_epoch;
  const bool project_to_navigation_state = pending_plan_start_->project_to_navigation_state;
  const auto origin = pending_plan_start_->request.origin;
  const std::string cancel_reason = reason.empty() ? "operator_cancel" : reason;
  pending_plan_start_.reset();
  if (paused_task_id_ == task_id) {
    pause_pending_ = false;
    active_paused_ = false;
    paused_task_id_.clear();
  }
  auto publish_terminal = std::make_shared<bool>(true);
  return {
      true,
      "cancel_ready",
      [this, task_id, request_id, pending_goal_epoch, cancel_reason, project_to_navigation_state, origin,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*publish_terminal) {
          return;
        }
        *publish_terminal = false;
        publishStatus(task_id, request_id, pending_goal_epoch,
                      lingtu::message::NavigationGoalState::Cancelled, cancel_reason,
                      project_to_navigation_state, origin);
      },
  };
}

GoalPlanTaskTransition GoalPlanController::deferCancelReplacementPlanning(
    const std::string &task_id, const std::string &request_id, const std::string &reason) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (!task_.busy() || planning_task_id_ != task_id) {
    return {false, "task_not_planning", {}};
  }
  if (planning_is_replan_ || active_task_id_.empty() || active_task_id_ == task_id ||
      planning_projects_to_navigation_state_) {
    return {false, "task_not_replacement_planning", {}};
  }

  const std::uint64_t planning_goal_epoch = planning_goal_epoch_;
  const std::string cancel_reason = reason.empty() ? "operator_cancel" : reason;
  const auto origin = planning_origin_;
  task_.cancel();
  ++goal_epoch_;
  clearPlanningIdentity();
  if (paused_task_id_ == task_id) {
    pause_pending_ = false;
    active_paused_ = false;
    paused_task_id_.clear();
  }
  auto publish_terminal = std::make_shared<bool>(true);
  return {
      true,
      "cancel_ready",
      [this, task_id, request_id, planning_goal_epoch, cancel_reason, origin,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*publish_terminal) {
          return;
        }
        *publish_terminal = false;
        publishStatus(task_id, request_id, planning_goal_epoch,
                      lingtu::message::NavigationGoalState::Cancelled, cancel_reason, false, origin);
      },
  };
}

GoalPlanTerminalCommit GoalPlanController::deferAbort(const std::string &reason,
                                                      bool project_planning_to_navigation_state) {
  return deferAbortWithTicket(reason, project_planning_to_navigation_state).commit;
}

GoalPlanTerminalCommitWithTicket
GoalPlanController::deferAbortWithTicket(const std::string &reason,
                                         bool project_planning_to_navigation_state) {
  const std::string planning_task_id = planning_task_id_;
  const std::string planning_request_id = planning_request_id_;
  const std::uint64_t planning_goal_epoch = planning_goal_epoch_;
  const std::string active_task_id = active_task_id_;
  const std::string active_request_id = active_request_id_;
  const std::uint64_t active_goal_epoch = active_goal_epoch_;
  const std::optional<DeferredPlanStart> pending_plan_start = pending_plan_start_;
  std::vector<GoalPlanStatus> pending_statuses;
  if (!planning_request_id.empty()) {
    pending_statuses.push_back({
        planning_task_id,
        planning_request_id,
        planning_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        project_planning_to_navigation_state,
        planning_origin_,
    });
  }
  if (pending_plan_start) {
    pending_statuses.push_back({
        pending_plan_start->request.task_id,
        pending_plan_start->request.request_id,
        pending_plan_start->goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        pending_plan_start->project_to_navigation_state,
        pending_plan_start->request.origin,
    });
  }
  if (!active_request_id.empty()) {
    pending_statuses.push_back({
        active_task_id,
        active_request_id,
        active_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        true,
        active_origin_,
    });
  }
  invalidateForHold(reason, false);

  GoalPlanTerminalDeliveryTicket ticket{pending_statuses};
  auto has_pending_status = std::make_shared<bool>(!pending_statuses.empty());
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return {
      std::move(ticket),
      [this, planning_task_id, planning_request_id, planning_goal_epoch, active_task_id,
       active_request_id, active_goal_epoch, has_pending_status,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*has_pending_status) {
          return;
        }
        *has_pending_status = false;
        if (planning_task_id_ == planning_task_id && planning_request_id_ == planning_request_id &&
            planning_goal_epoch_ == planning_goal_epoch) {
          clearPlanningIdentity();
        }
        if (active_task_id_ == active_task_id && active_request_id_ == active_request_id &&
            active_goal_epoch_ == active_goal_epoch) {
          active_task_id_.clear();
          active_request_id_.clear();
          active_goal_epoch_ = 0U;
          active_paused_ = false;
          pause_pending_ = false;
          paused_task_id_.clear();
          active_map_identity_.reset();
          active_target_.reset();
          active_source_stamp_s_ = 0.0;
          active_origin_ = GoalPlanOrigin::kExternal;
          active_planner_options_ = {};
          replan_attempt_ = 0U;
        }
        publish_terminal();
      },
  };
}

GoalPlanTerminalCommit GoalPlanController::deferFailure(const std::string &reason) {
  return deferFailureWithTicket(reason).commit;
}

GoalPlanTerminalCommitWithTicket
GoalPlanController::deferFailureWithTicket(const std::string &reason) {
  GoalPlanTerminalCommitWithTicket failed =
      deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState::Failed, reason);
  GoalPlanTerminalCommitWithTicket cancelled_planning = deferPlanningAbortWithTicket(reason);
  GoalPlanTerminalDeliveryTicket ticket;
  ticket.statuses.reserve(failed.ticket.statuses.size() +
                          cancelled_planning.ticket.statuses.size());
  ticket.statuses.insert(ticket.statuses.end(), failed.ticket.statuses.begin(),
                         failed.ticket.statuses.end());
  ticket.statuses.insert(ticket.statuses.end(), cancelled_planning.ticket.statuses.begin(),
                         cancelled_planning.ticket.statuses.end());
  return {
      std::move(ticket),
      [commit_failed = std::move(failed.commit),
       commit_cancelled_planning = std::move(cancelled_planning.commit)]() mutable {
        commit_failed();
        commit_cancelled_planning();
      },
  };
}

GoalPlanTerminalCommit
GoalPlanController::deferActiveTerminal(lingtu::message::NavigationGoalState state,
                                        const std::string &reason) {
  return deferActiveTerminalWithTicket(state, reason).commit;
}

GoalPlanTerminalCommitWithTicket
GoalPlanController::deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState state,
                                                  const std::string &reason) {
  if (!lingtu::message::isTerminalNavigationGoalState(state)) {
    throw std::invalid_argument("deferred active status must be terminal");
  }
  const std::string active_task_id = active_task_id_;
  const std::string active_request_id = active_request_id_;
  const std::uint64_t active_goal_epoch = active_goal_epoch_;
  std::vector<GoalPlanStatus> pending_statuses;
  if (!active_request_id.empty()) {
    pending_statuses.push_back({
        active_task_id,
        active_request_id,
        active_goal_epoch,
        state,
        reason,
        true,
        active_origin_,
    });
  }
  GoalPlanTerminalDeliveryTicket ticket{pending_statuses};
  auto has_pending_status = std::make_shared<bool>(!pending_statuses.empty());
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return {
      std::move(ticket),
      [this, state, reason, active_task_id, active_request_id, active_goal_epoch,
       has_pending_status, publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*has_pending_status) {
          return;
        }
        *has_pending_status = false;
        if (active_task_id_ == active_task_id && active_request_id_ == active_request_id &&
            active_goal_epoch_ == active_goal_epoch) {
          active_task_id_.clear();
          active_request_id_.clear();
          active_goal_epoch_ = 0U;
          active_paused_ = false;
          pause_pending_ = false;
          paused_task_id_.clear();
          active_map_identity_.reset();
          active_target_.reset();
          active_source_stamp_s_ = 0.0;
          active_origin_ = GoalPlanOrigin::kExternal;
          active_planner_options_ = {};
          replan_attempt_ = 0U;
        }
        diagnostics_.seen = true;
        diagnostics_.accepted = state == lingtu::message::NavigationGoalState::Reached;
        diagnostics_.reached_goal = state == lingtu::message::NavigationGoalState::Reached;
        diagnostics_.reason = reason;
        publish_terminal();
      },
  };
}

GoalPlanTerminalCommitWithTicket
GoalPlanController::deferPlanningAbortWithTicket(const std::string &reason,
                                                 bool project_planning_to_navigation_state) {
  const std::string planning_task_id = planning_task_id_;
  const std::string planning_request_id = planning_request_id_;
  const std::uint64_t planning_goal_epoch = planning_goal_epoch_;
  const std::optional<DeferredPlanStart> pending_plan_start = pending_plan_start_;
  std::vector<GoalPlanStatus> pending_statuses;
  if (!planning_request_id.empty()) {
    pending_statuses.push_back({
        planning_task_id,
        planning_request_id,
        planning_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        project_planning_to_navigation_state,
        planning_origin_,
    });
  }
  if (pending_plan_start) {
    pending_statuses.push_back({
        pending_plan_start->request.task_id,
        pending_plan_start->request.request_id,
        pending_plan_start->goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        pending_plan_start->project_to_navigation_state,
        pending_plan_start->request.origin,
    });
  }
  invalidateForHold(reason, false);

  GoalPlanTerminalDeliveryTicket ticket{pending_statuses};
  auto has_pending_status = std::make_shared<bool>(!pending_statuses.empty());
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return {
      std::move(ticket),
      [this, planning_task_id, planning_request_id, planning_goal_epoch, has_pending_status,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*has_pending_status) {
          return;
        }
        *has_pending_status = false;
        if (planning_task_id_ == planning_task_id && planning_request_id_ == planning_request_id &&
            planning_goal_epoch_ == planning_goal_epoch) {
          clearPlanningIdentity();
        }
        publish_terminal();
      },
  };
}
void GoalPlanController::invalidateForHold(const std::string &reason) {
  invalidateForHold(reason, true);
}

void GoalPlanController::invalidateForHold(const std::string &reason, bool close_pending_now) {
  ++goal_epoch_;
  task_.cancel();
  if (close_pending_now) {
    publishPendingTerminal(lingtu::message::NavigationGoalState::Cancelled, reason);
  }
  pending_plan_start_.reset();
  deferred_replacement_activation_.reset();
  pause_pending_ = false;
  active_paused_ = false;
  paused_task_id_.clear();
  planning_is_replan_ = false;
  planning_replan_attempt_ = 0U;
  diagnostics_.seen = true;
  diagnostics_.accepted = false;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = reason;
}

void GoalPlanController::finishPlanning(lingtu::message::NavigationGoalState state,
                                        const std::string &reason) {
  publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_, state, reason,
                planning_projects_to_navigation_state_, planning_origin_);
  clearPlanningIdentity();
}

void GoalPlanController::finishActive(lingtu::message::NavigationGoalState state,
                                      const std::string &reason) {
  GoalPlanTerminalCommit commit = deferActiveTerminal(state, reason);
  commit();
}

void GoalPlanController::publishPendingTerminal(lingtu::message::NavigationGoalState state,
                                                const std::string &reason) {
  if (!pending_plan_start_) {
    return;
  }
  publishStatus(pending_plan_start_->request.task_id, pending_plan_start_->request.request_id,
                pending_plan_start_->goal_epoch, state, reason,
                pending_plan_start_->project_to_navigation_state, pending_plan_start_->request.origin);
}

GoalPlanSnapshot GoalPlanController::snapshot() const {
  GoalPlanSnapshot result;
  result.goal_epoch = goal_epoch_;
  result.planning_task_id = planning_task_id_;
  result.planning_request_id = planning_request_id_;
  result.planning_goal_epoch = planning_goal_epoch_;
  if (!planning_task_id_.empty()) result.planning_origin = planning_origin_;
  result.replan_in_progress = planning_is_replan_ && task_.busy();
  result.replacement_plan_in_progress = task_.busy() && !planning_is_replan_ &&
                                        !planning_task_id_.empty() && !active_task_id_.empty() &&
                                        planning_task_id_ != active_task_id_;
  result.replan_attempt = replan_attempt_;
  result.pending_plan_queued = pending_plan_start_.has_value();
  if (pending_plan_start_) {
    result.pending_task_id = pending_plan_start_->request.task_id;
    result.pending_request_id = pending_plan_start_->request.request_id;
    result.pending_goal_epoch = pending_plan_start_->goal_epoch;
    result.pending_origin = pending_plan_start_->request.origin;
  }
  if (deferred_replacement_activation_) {
    result.deferred_replacement_task_id = deferred_replacement_activation_->task_id;
    result.deferred_replacement_request_id = deferred_replacement_activation_->request_id;
    result.deferred_replacement_goal_epoch = deferred_replacement_activation_->goal_epoch;
  }
  result.active_task_id = active_task_id_;
  result.active_request_id = active_request_id_;
  result.active_goal_epoch = active_goal_epoch_;
  if (!active_task_id_.empty() && !active_request_id_.empty()) {
    result.active_origin = active_origin_;
  }
  result.active_paused = active_paused_;
  result.pause_pending = pause_pending_;
  result.paused_task_id = paused_task_id_;
  result.active_map_identity = active_map_identity_;
  result.busy = task_.busy();
  result.diagnostics = diagnostics_;
  return result;
}

GoalPlanSubmitResult GoalPlanController::reject(std::string reason, bool count_frame_rejection,
                                                bool record_frame_error) {
  diagnostics_.accepted = false;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = reason;
  return {
      false, std::move(reason), true, count_frame_rejection, record_frame_error, std::nullopt,
  };
}

void GoalPlanController::publishStatus(const std::string &task_id, const std::string &request_id,
                                       std::uint64_t goal_epoch,
                                       lingtu::message::NavigationGoalState state,
                                       const std::string &reason,
                                       bool project_to_navigation_state, GoalPlanOrigin origin) {
  if (!task_id.empty() && !request_id.empty()) {
    actions_.publish_status(
        {task_id, request_id, goal_epoch, state, reason, project_to_navigation_state, origin});
  }
}

GoalPlanTerminalCommit
GoalPlanController::terminalCommit(std::vector<GoalPlanStatus> pending_statuses) const {
  auto publish_status = actions_.publish_status;
  auto shared_statuses = std::make_shared<std::vector<GoalPlanStatus>>(std::move(pending_statuses));
  return [publish_status = std::move(publish_status),
          shared_statuses = std::move(shared_statuses)]() mutable {
    if (shared_statuses->empty()) {
      return;
    }
    std::vector<GoalPlanStatus> statuses = std::move(*shared_statuses);
    shared_statuses->clear();
    for (const GoalPlanStatus &status : statuses) {
      publish_status(status);
    }
  };
}

}  // namespace lingtu::nav::endpoint
