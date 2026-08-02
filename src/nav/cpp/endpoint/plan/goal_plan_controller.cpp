#include "plan/goal_plan_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

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
  if (context.operator_takeover_latched) {
    return reject("operator_takeover_resume_required", true, true);
  }
  if (!context.autonomy_mode) {
    return reject(std::string{"goal_not_allowed_in_"} + context.control_mode_name, true, true);
  }
  if (!context.driver_control_blocker.empty()) {
    return reject(context.driver_control_blocker, true, true);
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
  if (!context.map_position) {
    return reject(context.odometry_ready ? "map_odom_tf_not_ready" : "odometry_not_ready");
  }
  diagnostics_.start = *context.map_position;

  if (!context.planner_map_configured) {
    return reject(context.planner_map_missing_reason);
  }
  if (pending_plan_start_ && request.origin == GoalPlanOrigin::kExternal) {
    publishPendingTerminal(lingtu::message::NavigationGoalState::Cancelled,
                           "superseded_by_new_goal");
    const auto pending_goal_epoch = ++goal_epoch_;
    pending_plan_start_ = DeferredPlanStart{request, pending_goal_epoch};
    publishStatus(request.task_id, request.request_id, pending_goal_epoch,
                  lingtu::message::NavigationGoalState::Planning, "planning_queued", false);
    diagnostics_.reason = "planning_queued";
    return {true, "planning_queued", false, false, false, std::nullopt};
  }
  if (task_.busy()) {
    if (request.origin == GoalPlanOrigin::kExternal && planning_is_replan_) {
      task_.cancel();
      if (pending_plan_start_) {
        publishPendingTerminal(lingtu::message::NavigationGoalState::Cancelled,
                               "superseded_by_new_goal");
      }
      const auto pending_goal_epoch = ++goal_epoch_;
      pending_plan_start_ = DeferredPlanStart{request, pending_goal_epoch};
      clearPlanningIdentity();
      publishStatus(request.task_id, request.request_id, pending_goal_epoch,
                    lingtu::message::NavigationGoalState::Planning, "planning_queued", false);
      diagnostics_.reason = "planning_queued";
      return {true, "planning_queued", false, false, false, std::nullopt};
    }
    return reject("global_planner_busy");
  }

  if (context.rolling_segment_active && !actions_.preempt_rolling("superseded_by_generic_goal")) {
    return reject("segment_preempt_zero_publish_failed", false, true);
  }

  const bool projects_to_navigation_state =
      active_task_id_.empty() || active_task_id_ == request.task_id;
  return startPlanning(request.task_id, request.request_id, *request.target, request.origin,
                       context, "planning", false, std::nullopt, projects_to_navigation_state);
}

GoalPlanSubmitResult GoalPlanController::replanActive(const GoalPlanAdmissionContext &context) {
  diagnostics_ = {};
  diagnostics_.seen = true;

  if (active_task_id_.empty() || active_request_id_.empty() || !active_target_) {
    return reject("no_active_external_goal");
  }
  if (active_origin_ != GoalPlanOrigin::kExternal || actions_.inspection_active()) {
    return reject("inspection_replan_not_supported");
  }
  if (task_.busy()) {
    return reject("global_planner_busy");
  }
  if (!context.motion_allowed) {
    return reject("estop_latched", true, true);
  }
  if (context.operator_takeover_latched) {
    return reject("operator_takeover_resume_required", true, true);
  }
  if (!context.autonomy_mode) {
    return reject(std::string{"goal_not_allowed_in_"} + context.control_mode_name, true, true);
  }
  if (!context.driver_control_blocker.empty()) {
    return reject(context.driver_control_blocker, true, true);
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
  if (!current_map.identity || !current_map.identity->valid()) {
    return reject("active_map_unavailable_before_replan", false, true);
  }
  if (!lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
    return reject("active_map_changed_before_replan", false, true);
  }

  return startPlanning(active_task_id_, active_request_id_, *active_target_, active_origin_,
                       context, "replanning_after_local_recovery", true);
}

GoalPlanSubmitResult GoalPlanController::startPlanning(
    const std::string &task_id, const std::string &request_id, const GoalPlanTarget &target,
    GoalPlanOrigin origin, const GoalPlanAdmissionContext &context,
    const std::string &planning_reason, bool is_replan,
    std::optional<std::uint64_t> reserved_goal_epoch, bool project_planning_to_navigation_state) {
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
  plan_context.request.options = is_replan ? active_planner_options_ : context.planner_options;
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
  publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_,
                lingtu::message::NavigationGoalState::Planning, planning_reason,
                project_planning_to_navigation_state);
  diagnostics_.reason = planning_reason;
  return {true,        is_replan ? "replan_started" : "planning_started", false, false, false,
          std::nullopt};
}

GoalPlanSubmitResult
GoalPlanController::resumePending(const GoalPlanAdmissionContext &fresh_context) {
  diagnostics_ = {};
  diagnostics_.seen = true;

  if (!pending_plan_start_) {
    return reject("no_pending_plan");
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

  if (active_map_identity_ && active_map_identity_->valid()) {
    GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
    if (!current_map.identity || !current_map.identity->valid()) {
      return reject_pending(lingtu::message::NavigationGoalState::Failed,
                            "active_map_unavailable_before_pending_plan", false, true);
    }
    if (!lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
      return reject_pending(lingtu::message::NavigationGoalState::Failed,
                            "active_map_changed_before_pending_plan", false, true);
    }
  }

  const DeferredPlanStart &pending = *pending_plan_start_;
  if (!fresh_context.motion_allowed) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled, "estop_latched", true,
                          true);
  }
  if (fresh_context.operator_takeover_latched) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          "operator_takeover_resume_required", true, true);
  }
  if (!fresh_context.autonomy_mode) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          std::string{"goal_not_allowed_in_"} + fresh_context.control_mode_name,
                          true, true);
  }
  if (!fresh_context.driver_control_blocker.empty()) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          fresh_context.driver_control_blocker, true, true);
  }
  if (!fresh_context.input_ready) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed,
                          std::string{"input_gate_"} + (fresh_context.input_gate_reason.empty()
                                                            ? "blocked"
                                                            : fresh_context.input_gate_reason),
                          false, true);
  }
  if (!std::isfinite(fresh_context.autonomy_request_not_before_s)) {
    return reject_pending(lingtu::message::NavigationGoalState::Failed, "invalid_replan_admission",
                          false, true);
  }
  if (fresh_context.autonomy_request_not_before_s > 0.0 &&
      (!std::isfinite(pending.request.source_stamp_s) || pending.request.source_stamp_s <= 0.0 ||
       pending.request.source_stamp_s <= fresh_context.autonomy_request_not_before_s)) {
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
    return reject_pending(lingtu::message::NavigationGoalState::Failed,
                          fresh_context.odometry_ready ? "map_odom_tf_not_ready"
                                                       : "odometry_not_ready",
                          false, false);
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
  if (fresh_context.rolling_segment_active &&
      !actions_.preempt_rolling("superseded_by_generic_goal")) {
    return reject_pending(lingtu::message::NavigationGoalState::Cancelled,
                          "segment_preempt_zero_publish_failed", false, true);
  }

  DeferredPlanStart ready = std::move(*pending_plan_start_);
  pending_plan_start_.reset();
  return startPlanning(ready.request.task_id, ready.request.request_id, *ready.request.target,
                       ready.request.origin, fresh_context, "planning", false, ready.goal_epoch,
                       false);
}

void GoalPlanController::clearPlanningIdentity() {
  planning_task_id_.clear();
  planning_request_id_.clear();
  planning_goal_epoch_ = 0U;
  planning_is_replan_ = false;
  planning_projects_to_navigation_state_ = true;
  planning_replan_attempt_ = 0U;
  planning_origin_ = GoalPlanOrigin::kExternal;
}

GoalPlanAdvanceResult GoalPlanController::advance(const GoalPlanAdvanceContext &context) {
  GoalPlanAdvanceResult advance_result;
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
  if (stale_reason.empty() && completing_replan && active_map_identity_ && current_map.identity &&
      !lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
    stale_reason = "active_map_changed_during_replan";
  }
  if (!stale_reason.empty()) {
    diagnostics_.accepted = false;
    diagnostics_.reason = stale_reason;
    advance_result.counted_failure = true;
    advance_result.record_frame_error = true;
    const bool map_invalidated_plan = stale_reason == "planner_map_identity_missing" ||
                                      stale_reason == "active_map_unavailable_after_planning" ||
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

  if (context.operator_takeover_latched) {
    diagnostics_.accepted = false;
    diagnostics_.reason = "operator_takeover_discarded_plan";
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

  std::vector<nav_kernel::Vec3> global_path;
  global_path.reserve(plan_result.path.size());
  for (const auto &point : plan_result.path) {
    global_path.push_back({point.x, point.y, point.z});
  }
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
  if (global_path.size() == 1U) {
    const auto &start = completion->context.start;
    const auto &point = global_path.front();
    const double dx = start.x - point.x;
    const double dy = start.y - point.y;
    const double dz = start.z - point.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.02) {
      global_path.insert(global_path.begin(), start);
    } else {
      global_path.push_back(completion->context.goal);
    }
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
  };
  const bool completing_replacement =
      !completing_replan && !planning_projects_to_navigation_state_ && !active_task_id_.empty() &&
      planning_task_id_ != active_task_id_;
  if (completing_replacement) {
    deferred_replacement_activation_ = DeferredReplacementActivation{
        planning_task_id_,
        planning_request_id_,
        planning_goal_epoch_,
        std::move(activation),
        GoalPlanTarget{completion->context.goal, completion->context.goal_yaw},
        planning_origin_,
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
  active_map_identity_ = plan_result.map_identity;
  active_target_ = GoalPlanTarget{completion->context.goal, completion->context.goal_yaw};
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
  if (fresh_context.operator_takeover_latched) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         "operator_takeover_resume_required");
  }
  if (!fresh_context.autonomy_mode) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         std::string{"goal_not_allowed_in_"} +
                                             fresh_context.control_mode_name);
  }
  if (!fresh_context.driver_control_blocker.empty()) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Cancelled,
                                         fresh_context.driver_control_blocker);
  }
  if (!fresh_context.input_ready) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         std::string{"input_gate_"} +
                                             (fresh_context.input_gate_reason.empty()
                                                  ? "blocked"
                                                  : fresh_context.input_gate_reason));
  }
  if (!fresh_context.map_position) {
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         fresh_context.odometry_ready ? "map_odom_tf_not_ready"
                                                                      : "odometry_not_ready");
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
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "active_map_unavailable_before_replacement_activation");
  }
  if (!lingtu::nav::plan::sameMapIdentity(*ready.activation.map_identity, *current_map.identity)) {
    deferred_replacement_activation_ = std::move(ready);
    return failDeferredReplacementLocked(lingtu::message::NavigationGoalState::Failed,
                                         "active_map_changed_before_replacement_activation");
  }

  deferred_replacement_activation_.reset();
  ready.activation.stamp_s = now_s;
  actions_.activate_path(ready.activation);

  active_task_id_ = ready.task_id;
  active_request_id_ = ready.request_id;
  active_goal_epoch_ = ready.goal_epoch;
  active_paused_ = false;
  active_map_identity_ = ready.activation.map_identity;
  active_target_ = ready.target;
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
                true);
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
                      failed.task_id, failed.request_id, failed.goal_epoch, state, reason, false}}},
                  terminalCommit({GoalPlanStatus{failed.task_id, failed.request_id,
                                                 failed.goal_epoch, state, reason, false}})});
  return result;
}
GoalPlanTaskTransition GoalPlanController::deferPause(const std::string &task_id,
                                                      const std::string &request_id,
                                                      const std::string &reason) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (active_task_id_.empty() || active_task_id_ != task_id) {
    return {false, "task_not_active", {}};
  }
  if (task_.busy()) {
    if (!planning_is_replan_ || planning_task_id_ != task_id) {
      return {false, "task_not_executing", {}};
    }
    task_.cancel();
    clearPlanningIdentity();
  }
  if (active_paused_) {
    return {false, "task_already_paused", {}};
  }

  const std::uint64_t active_goal_epoch = active_goal_epoch_;
  const std::string pause_reason = reason.empty() ? "operator_pause" : reason;
  return {
      true,
      "pause_ready",
      [this, task_id, request_id, active_goal_epoch, pause_reason] {
        if (active_task_id_ != task_id || active_goal_epoch_ != active_goal_epoch ||
            active_paused_) {
          return;
        }
        active_request_id_ = request_id;
        active_paused_ = true;
        diagnostics_.seen = true;
        diagnostics_.accepted = true;
        diagnostics_.reached_goal = false;
        diagnostics_.reason = pause_reason;
        publishStatus(task_id, request_id, active_goal_epoch,
                      lingtu::message::NavigationGoalState::Paused, pause_reason);
      },
  };
}

GoalPlanTaskTransition GoalPlanController::deferResume(const std::string &task_id,
                                                       const std::string &request_id,
                                                       const GoalPlanAdmissionContext &context) {
  if (task_id.empty() || request_id.empty()) {
    return {false, task_id.empty() ? "command_task_id_empty" : "command_request_id_empty", {}};
  }
  if (task_.busy()) {
    return {false, "task_not_executing", {}};
  }
  if (active_task_id_.empty() || active_task_id_ != task_id) {
    return {false, "task_not_active", {}};
  }
  if (!active_paused_) {
    return {false, "task_not_paused", {}};
  }
  if (!context.motion_allowed) {
    return {false, "estop_latched", {}};
  }
  if (context.operator_takeover_latched) {
    return {false, "operator_takeover_resume_required", {}};
  }
  if (!context.autonomy_mode) {
    return {false, std::string{"resume_not_allowed_in_"} + context.control_mode_name, {}};
  }
  if (!context.driver_control_blocker.empty()) {
    return {false, context.driver_control_blocker, {}};
  }
  if (!context.input_ready) {
    return {false,
            std::string{"input_gate_"} +
                (context.input_gate_reason.empty() ? "blocked" : context.input_gate_reason),
            {}};
  }
  if (!context.retained_path_ready) {
    return {false,
            context.retained_path_reason.empty() ? "retained_global_path_missing"
                                                 : context.retained_path_reason,
            {}};
  }
  if (!context.map_position) {
    return {false, context.odometry_ready ? "map_odom_tf_not_ready" : "odometry_not_ready", {}};
  }
  if (!context.planner_map_configured) {
    return {false, context.planner_map_missing_reason, {}};
  }
  if (!active_map_identity_ || !active_map_identity_->valid()) {
    return {false, "active_goal_map_identity_missing", {}};
  }
  const GoalPlanMapIdentityResult current_map = actions_.current_map_identity();
  if (!current_map.identity || !current_map.identity->valid()) {
    return {false, "active_map_unavailable_before_resume", {}};
  }
  if (!lingtu::nav::plan::sameMapIdentity(*active_map_identity_, *current_map.identity)) {
    return {false, "active_map_changed_before_resume", {}};
  }

  const std::uint64_t active_goal_epoch = active_goal_epoch_;
  return {
      true,
      "resume_ready",
      [this, task_id, request_id, active_goal_epoch] {
        if (active_task_id_ != task_id || active_goal_epoch_ != active_goal_epoch ||
            !active_paused_) {
          return;
        }
        active_request_id_ = request_id;
        active_paused_ = false;
        diagnostics_.seen = true;
        diagnostics_.accepted = true;
        diagnostics_.reached_goal = false;
        diagnostics_.reason = "path_resumed";
        publishStatus(task_id, request_id, active_goal_epoch,
                      lingtu::message::NavigationGoalState::PathActive, "path_resumed");
      },
  };
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
  const std::string cancel_reason = reason.empty() ? "operator_cancel" : reason;
  pending_plan_start_.reset();
  auto publish_terminal = std::make_shared<bool>(true);
  return {
      true,
      "cancel_ready",
      [this, task_id, request_id, pending_goal_epoch, cancel_reason,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*publish_terminal) {
          return;
        }
        *publish_terminal = false;
        publishStatus(task_id, request_id, pending_goal_epoch,
                      lingtu::message::NavigationGoalState::Cancelled, cancel_reason, false);
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
  task_.cancel();
  ++goal_epoch_;
  clearPlanningIdentity();
  auto publish_terminal = std::make_shared<bool>(true);
  return {
      true,
      "cancel_ready",
      [this, task_id, request_id, planning_goal_epoch, cancel_reason,
       publish_terminal = std::move(publish_terminal)]() mutable {
        if (!*publish_terminal) {
          return;
        }
        *publish_terminal = false;
        publishStatus(task_id, request_id, planning_goal_epoch,
                      lingtu::message::NavigationGoalState::Cancelled, cancel_reason, false);
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
    });
  }
  if (pending_plan_start) {
    pending_statuses.push_back({
        pending_plan_start->request.task_id,
        pending_plan_start->request.request_id,
        pending_plan_start->goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        false,
    });
  }
  if (!active_request_id.empty()) {
    pending_statuses.push_back({
        active_task_id,
        active_request_id,
        active_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
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
          active_map_identity_.reset();
          active_target_.reset();
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
          active_map_identity_.reset();
          active_target_.reset();
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
    });
  }
  if (pending_plan_start) {
    pending_statuses.push_back({
        pending_plan_start->request.task_id,
        pending_plan_start->request.request_id,
        pending_plan_start->goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
        false,
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
                planning_projects_to_navigation_state_);
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
                pending_plan_start_->goal_epoch, state, reason, false);
}

GoalPlanSnapshot GoalPlanController::snapshot() const {
  GoalPlanSnapshot result;
  result.goal_epoch = goal_epoch_;
  result.planning_task_id = planning_task_id_;
  result.planning_request_id = planning_request_id_;
  result.planning_goal_epoch = planning_goal_epoch_;
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
                                       bool project_to_navigation_state) {
  if (!task_id.empty() && !request_id.empty()) {
    actions_.publish_status(
        {task_id, request_id, goal_epoch, state, reason, project_to_navigation_state});
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
