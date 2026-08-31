#include "command/cancel.hpp"

#include <utility>

#include "runtime/goal/plan.hpp"
#include "runtime/goal/runtime.hpp"

namespace lingtu::nav::endpoint {

GoalTaskCancelRouter::GoalTaskCancelRouter(GoalPlanController &goal_plan,
                                           GoalReplanRuntimeCoordinator &runtime,
                                           TerminalService terminal_service,
                                           StatusRequest request_status)
    : goal_plan_(goal_plan),
      runtime_(runtime),
      terminal_service_(std::move(terminal_service)),
      request_status_(std::move(request_status)) {}

CommandAck GoalTaskCancelRouter::handle(const GoalTaskCancelRequest &request) {
  if (request.task_id.empty()) {
    return {false, "command_task_id_empty"};
  }
  if (request.cancel_request_id.empty()) {
    return {false, "command_request_id_empty"};
  }
  if (runtime_.terminalPending()) {
    return {false, "goal_terminal_pending"};
  }

  const GoalPlanSnapshot snapshot = goal_plan_.snapshot();
  if (request.task_id == snapshot.pending_task_id) {
    auto transition =
        goal_plan_.deferCancelPending(request.task_id, request.cancel_request_id, request.reason);
    if (!transition.accepted) {
      return {false, transition.reason};
    }
    transition.commit();
    if (request_status_) {
      request_status_();
    }
    return {true, "cancel_requested"};
  }

  if (request.task_id == snapshot.planning_task_id && snapshot.replacement_plan_in_progress) {
    auto transition = goal_plan_.deferCancelReplacementPlanning(
        request.task_id, request.cancel_request_id, request.reason);
    if (!transition.accepted) {
      return {false, transition.reason};
    }
    transition.commit();
    if (request_status_) {
      request_status_();
    }
    return {true, "cancel_requested"};
  }

  if (request.task_id == snapshot.deferred_replacement_task_id ||
      request.task_id == snapshot.active_task_id || request.task_id == snapshot.planning_task_id) {
    if (!terminal_service_) {
      return {false, "goal_terminal_service_missing"};
    }
    const GoalReplanRuntimeResult runtime_result =
        runtime_.interrupt(GoalReplanRuntimeInterruption::kCancel, request.steady_now_s);
    const GoalTaskCancelTerminalServiceResult serviced = terminal_service_(runtime_result);
    if (request_status_) {
      request_status_();
    }
    return {serviced.action_committed,
            serviced.action_committed ? "cancel_requested" : serviced.reason};
  }

  return {false, "task_not_active"};
}

}  // namespace lingtu::nav::endpoint
