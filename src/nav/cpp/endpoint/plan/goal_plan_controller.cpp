#include "plan/goal_plan_controller.hpp"

#include <cmath>
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
  if (task_.busy()) {
    return reject("global_planner_busy");
  }

  if (context.rolling_segment_active && !actions_.preempt_rolling("superseded_by_generic_goal")) {
    return reject("segment_preempt_zero_publish_failed", false, true);
  }

  ++goal_epoch_;
  GlobalPlanContext plan_context;
  plan_context.request_id = request.request_id;
  plan_context.start = diagnostics_.start;
  plan_context.goal = diagnostics_.goal;
  plan_context.goal_yaw = request.target->yaw;
  plan_context.goal_epoch = goal_epoch_;
  plan_context.frame_epoch = context.frame_epoch;
  plan_context.request.start = {
      diagnostics_.start.x,
      diagnostics_.start.y,
      diagnostics_.start.z,
  };
  plan_context.request.goal = {
      diagnostics_.goal.x,
      diagnostics_.goal.y,
      diagnostics_.goal.z,
  };
  plan_context.request.options = context.planner_options;

  if (!task_.start(std::move(plan_context))) {
    return reject("global_planner_busy");
  }

  planning_task_id_ = request.task_id;
  planning_request_id_ = request.request_id;
  planning_goal_epoch_ = goal_epoch_;
  publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_,
                lingtu::message::NavigationGoalState::Planning, "planning");
  diagnostics_.reason = "planning";
  return {true, "planning_started", false, false, false};
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

  GoalPlanMapIdentityResult current_map;
  if (completion->context.goal_epoch == goal_epoch_ &&
      completion->context.frame_epoch == context.frame_epoch && completion->error.empty() &&
      !plan_result.cancelled) {
    current_map = actions_.current_map_identity();
    advance_result.map_identity_error = current_map.reason;
  }
  const std::string stale_reason =
      globalPlanStaleReason(*completion, goal_epoch_, context.frame_epoch, current_map.identity);
  if (!stale_reason.empty()) {
    diagnostics_.accepted = false;
    diagnostics_.reason = stale_reason;
    advance_result.counted_failure = true;
    advance_result.record_frame_error = true;
    const bool map_invalidated_plan = stale_reason == "planner_map_identity_missing" ||
                                      stale_reason == "active_map_unavailable_after_planning" ||
                                      stale_reason == "active_map_changed_during_planning";
    finishPlanning(map_invalidated_plan ? lingtu::message::NavigationGoalState::Failed
                                        : lingtu::message::NavigationGoalState::Cancelled,
                   stale_reason);
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(stale_reason, context.now_s);
    }
    if (!active_request_id_.empty()) {
      advance_result.terminal_after_stop = GoalPlanTerminalAfterStop{
          stale_reason,
          deferActiveTerminal(lingtu::message::NavigationGoalState::Cancelled, stale_reason),
      };
    }
    ++goal_epoch_;
    task_.cancel();
    return advance_result;
  }

  if (!completion->error.empty()) {
    diagnostics_.reason = "global_planner_exception";
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    advance_result.counted_failure = true;
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
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    advance_result.counted_failure = true;
    if (actions_.inspection_active()) {
      actions_.inspection_leg_failed(diagnostics_.reason, context.now_s);
    }
    return advance_result;
  }

  if (context.operator_takeover_latched) {
    diagnostics_.accepted = false;
    diagnostics_.reason = "operator_takeover_discarded_plan";
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
    finishPlanning(lingtu::message::NavigationGoalState::Failed, diagnostics_.reason);
    advance_result.counted_failure = true;
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
      finishPlanning(lingtu::message::NavigationGoalState::Cancelled, diagnostics_.reason);
      if (!active_request_id_.empty()) {
        advance_result.terminal_after_stop = GoalPlanTerminalAfterStop{
            diagnostics_.reason,
            deferActiveTerminal(lingtu::message::NavigationGoalState::Cancelled,
                                diagnostics_.reason),
        };
      }
      ++goal_epoch_;
      task_.cancel();
      advance_result.inspection_status_changed = true;
      return advance_result;
    }
  }

  diagnostics_.waypoints = global_path.size();
  actions_.activate_path({
      global_path,
      completion->context.goal_yaw,
      inspection_decision.tolerance,
      context.now_s,
  });
  finishActive(lingtu::message::NavigationGoalState::Cancelled, "superseded_by_new_goal");
  active_task_id_ = planning_task_id_;
  active_request_id_ = planning_request_id_;
  active_goal_epoch_ = planning_goal_epoch_;
  finishPlanning(lingtu::message::NavigationGoalState::PathActive, "path_active");
  diagnostics_.accepted = true;
  diagnostics_.reason = "accepted";
  advance_result.path_activated = true;
  return advance_result;
}

GoalPlanTerminalCommit GoalPlanController::deferAbort(const std::string &reason) {
  const std::string planning_task_id = planning_task_id_;
  const std::string planning_request_id = planning_request_id_;
  const std::uint64_t planning_goal_epoch = planning_goal_epoch_;
  const std::string active_task_id = active_task_id_;
  const std::string active_request_id = active_request_id_;
  const std::uint64_t active_goal_epoch = active_goal_epoch_;
  std::vector<GoalPlanStatus> pending_statuses;
  if (!planning_request_id.empty()) {
    pending_statuses.push_back({
        planning_task_id,
        planning_request_id,
        planning_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
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
  invalidateForHold(reason);

  bool has_pending_status = !pending_statuses.empty();
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return [this, planning_task_id, planning_request_id, planning_goal_epoch, active_task_id,
          active_request_id, active_goal_epoch, has_pending_status,
          publish_terminal = std::move(publish_terminal)]() mutable {
    if (!has_pending_status) {
      return;
    }
    has_pending_status = false;
    if (planning_task_id_ == planning_task_id && planning_request_id_ == planning_request_id &&
        planning_goal_epoch_ == planning_goal_epoch) {
      planning_task_id_.clear();
      planning_request_id_.clear();
      planning_goal_epoch_ = 0U;
    }
    if (active_task_id_ == active_task_id && active_request_id_ == active_request_id &&
        active_goal_epoch_ == active_goal_epoch) {
      active_task_id_.clear();
      active_request_id_.clear();
      active_goal_epoch_ = 0U;
    }
    publish_terminal();
  };
}

GoalPlanTerminalCommit GoalPlanController::deferFailure(const std::string &reason) {
  GoalPlanTerminalCommit commit_failed =
      deferActiveTerminal(lingtu::message::NavigationGoalState::Failed, reason);
  GoalPlanTerminalCommit commit_cancelled_planning = deferPlanningAbort(reason);
  return [commit_failed = std::move(commit_failed),
          commit_cancelled_planning = std::move(commit_cancelled_planning)]() mutable {
    commit_failed();
    commit_cancelled_planning();
  };
}

void GoalPlanController::invalidateForHold(const std::string &reason) {
  ++goal_epoch_;
  task_.cancel();
  diagnostics_.seen = true;
  diagnostics_.accepted = false;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = reason;
}

GoalPlanCancelAdmission GoalPlanController::admitCancel(const std::string &task_id) const {
  if (task_id.empty()) {
    if (!planning_task_id_.empty() || !active_task_id_.empty()) {
      return {true, "cancel_current"};
    }
    return {false, "task_not_active"};
  }
  if (!planning_task_id_.empty() && !active_task_id_.empty() &&
      planning_task_id_ != active_task_id_) {
    return {false, "task_cancel_ambiguous"};
  }
  if (task_id == planning_task_id_ || task_id == active_task_id_) {
    return {true, "cancel_target_active"};
  }
  return {false, "task_not_active"};
}

GoalPlanTerminalCommit
GoalPlanController::deferActiveTerminal(lingtu::message::NavigationGoalState state,
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
  bool has_pending_status = !pending_statuses.empty();
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return [this, state, reason, active_task_id, active_request_id, active_goal_epoch,
          has_pending_status, publish_terminal = std::move(publish_terminal)]() mutable {
    if (!has_pending_status) {
      return;
    }
    has_pending_status = false;
    if (active_task_id_ == active_task_id && active_request_id_ == active_request_id &&
        active_goal_epoch_ == active_goal_epoch) {
      active_task_id_.clear();
      active_request_id_.clear();
      active_goal_epoch_ = 0U;
    }
    diagnostics_.seen = true;
    diagnostics_.accepted = state == lingtu::message::NavigationGoalState::Reached;
    diagnostics_.reached_goal = state == lingtu::message::NavigationGoalState::Reached;
    diagnostics_.reason = reason;
    publish_terminal();
  };
}

GoalPlanTerminalCommit GoalPlanController::deferPlanningAbort(const std::string &reason) {
  const std::string planning_task_id = planning_task_id_;
  const std::string planning_request_id = planning_request_id_;
  const std::uint64_t planning_goal_epoch = planning_goal_epoch_;
  std::vector<GoalPlanStatus> pending_statuses;
  if (!planning_request_id.empty()) {
    pending_statuses.push_back({
        planning_task_id,
        planning_request_id,
        planning_goal_epoch,
        lingtu::message::NavigationGoalState::Cancelled,
        reason,
    });
  }
  invalidateForHold(reason);

  bool has_pending_status = !pending_statuses.empty();
  GoalPlanTerminalCommit publish_terminal = terminalCommit(std::move(pending_statuses));
  return [this, planning_task_id, planning_request_id, planning_goal_epoch, has_pending_status,
          publish_terminal = std::move(publish_terminal)]() mutable {
    if (!has_pending_status) {
      return;
    }
    has_pending_status = false;
    if (planning_task_id_ == planning_task_id && planning_request_id_ == planning_request_id &&
        planning_goal_epoch_ == planning_goal_epoch) {
      planning_task_id_.clear();
      planning_request_id_.clear();
      planning_goal_epoch_ = 0U;
    }
    publish_terminal();
  };
}

void GoalPlanController::finishPlanning(lingtu::message::NavigationGoalState state,
                                        const std::string &reason) {
  publishStatus(planning_task_id_, planning_request_id_, planning_goal_epoch_, state, reason);
  planning_task_id_.clear();
  planning_request_id_.clear();
  planning_goal_epoch_ = 0U;
}

void GoalPlanController::finishActive(lingtu::message::NavigationGoalState state,
                                      const std::string &reason) {
  GoalPlanTerminalCommit commit = deferActiveTerminal(state, reason);
  commit();
}

GoalPlanSnapshot GoalPlanController::snapshot() const {
  return {
      goal_epoch_,          planning_task_id_, planning_request_id_,
      planning_goal_epoch_, active_task_id_,   active_request_id_,
      active_goal_epoch_,   task_.busy(),      diagnostics_,
  };
}

GoalPlanSubmitResult GoalPlanController::reject(std::string reason, bool count_frame_rejection,
                                                bool record_frame_error) {
  diagnostics_.accepted = false;
  diagnostics_.reached_goal = false;
  diagnostics_.reason = reason;
  return {
      false, std::move(reason), true, count_frame_rejection, record_frame_error,
  };
}

void GoalPlanController::publishStatus(const std::string &task_id, const std::string &request_id,
                                       std::uint64_t goal_epoch,
                                       lingtu::message::NavigationGoalState state,
                                       const std::string &reason) {
  if (!task_id.empty() && !request_id.empty()) {
    actions_.publish_status({task_id, request_id, goal_epoch, state, reason});
  }
}

GoalPlanTerminalCommit
GoalPlanController::terminalCommit(std::vector<GoalPlanStatus> pending_statuses) const {
  auto publish_status = actions_.publish_status;
  return [publish_status = std::move(publish_status),
          pending_statuses = std::move(pending_statuses)]() mutable {
    for (const GoalPlanStatus &status : pending_statuses) {
      publish_status(status);
    }
    pending_statuses.clear();
  };
}

}  // namespace lingtu::nav::endpoint
