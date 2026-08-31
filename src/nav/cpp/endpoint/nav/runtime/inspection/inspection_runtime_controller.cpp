#include "runtime/inspection/inspection_runtime_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace lingtu::nav::endpoint {
namespace {

using InspectionRunState = lingtu::nav::inspection::RunState;

bool isMotionStoppingTimeout(const std::string &reason) {
  return reason == "planning_timeout" || reason == "navigation_stalled" ||
         reason == "settling_timeout" || reason == "action_timeout";
}

}  // namespace

InspectionRuntimeController::InspectionRuntimeController(
    lingtu::nav::inspection::Executor &executor, InspectionRuntimeConfig config)
    : executor_(executor), config_(config) {
  if (!std::isfinite(config_.map_check_interval_s) || config_.map_check_interval_s <= 0.0) {
    throw std::invalid_argument("inspection map check interval must be positive");
  }
  if (!std::isfinite(config_.status_interval_s) || config_.status_interval_s <= 0.0) {
    throw std::invalid_argument("inspection status interval must be positive");
  }
}

InspectionRuntimeTickResult
InspectionRuntimeController::tick(const InspectionRuntimeTickInput &input) {
  InspectionRuntimeTickResult result;
  const InspectionRunState state_before_sample = executor_.status().state;

  for (const auto &evidence : input.evidence_results) {
    if (applyEvidenceResult(evidence, input.now_s)) {
      requestStatus();
    }
  }
  if (state_before_sample == InspectionRunState::kActionPending &&
      executor_.status().state != state_before_sample) {
    appendClearMotion(result, executor_.status().reason);
    evidence_dispatch_outstanding_request_id_.clear();
  }

  if (state_before_sample == InspectionRunState::kSettling) {
    if (input.odom_generation != arrival_odom_generation_) {
      arrival_odom_generation_ = input.odom_generation;
      if (input.arrival_sample) {
        (void)executor_.OnArrivalSample(*input.arrival_sample, input.now_s);
      }
    }
  } else {
    // Only odometry received after OnGoalReached may count toward settling.
    arrival_odom_generation_ = input.odom_generation;
  }

  const std::string reason_before_tick = executor_.status().reason;
  if (executor_.status().state == InspectionRunState::kNavigating && active_point_ &&
      input.robot_position && input.path_active) {
    (void)executor_.OnNavigationProgress(std::hypot(active_point_->x_m - input.robot_position->x_m,
                                                    active_point_->y_m - input.robot_position->y_m),
                                         input.now_s);
  }
  executor_.Tick(input.now_s);
  const std::string reason_after_tick = executor_.status().reason;
  if (reason_after_tick != reason_before_tick && isMotionStoppingTimeout(reason_after_tick)) {
    appendClearMotion(result, reason_after_tick);
  }
  if (executor_.status().state != state_before_sample) {
    requestStatus();
  }

  const InspectionRunState state_before_map_check = executor_.status().state;
  if (input.now_s >= next_map_check_s_) {
    next_map_check_s_ = input.now_s + config_.map_check_interval_s;
    if (input.active_map) {
      executor_.OnMapChanged(input.active_map->map_id, input.active_map->version);
    } else if (executor_.active()) {
      executor_.OnMapChanged("", -1);
    }
    if (state_before_map_check != executor_.status().state &&
        executor_.status().state == InspectionRunState::kFailed &&
        executor_.status().reason == "active_map_changed") {
      result.ordered_intents.push_back(
          {InspectionRuntimeIntentKind::kStopControlAuthority, "inspection_active_map_changed"});
      appendClearMotion(result, "inspection_active_map_changed");
      requestStatus();
    }
  }

  // An accepted evidence publication is completed synchronously, but the
  // worker result is asynchronous. If the executor times out, retries, skips,
  // is cancelled, or starts another run before that result arrives, the old
  // request must not suppress the next action dispatch.
  if (!evidence_dispatch_outstanding_request_id_.empty() &&
      (executor_.status().state != InspectionRunState::kActionPending ||
       executor_.status().action_request_id != evidence_dispatch_outstanding_request_id_)) {
    evidence_dispatch_outstanding_request_id_.clear();
  }

  if (executor_.status().state != InspectionRunState::kPlanning) {
    goal_dispatch_outstanding_ = false;
  }
  if (!goal_dispatch_outstanding_) {
    if (const auto point = executor_.PendingGoal()) {
      if (!input.goal_plan_busy) {
        active_point_ = *point;
        goal_dispatch_outstanding_ = true;
        result.goal_dispatch = InspectionGoalDispatchIntent{*point};
      }
    }
  }

  if (const auto action = executor_.PendingAction()) {
    const auto *route = executor_.route();
    if (route != nullptr && input.evidence_worker_matched &&
        evidence_dispatch_outstanding_request_id_.empty() &&
        executor_.OnActionStarted(action->request_id, input.now_s)) {
      evidence_dispatch_outstanding_request_id_ = action->request_id;
      result.evidence_dispatch = InspectionEvidenceDispatchIntent{
          *action,
          route->map_id,
          route->map_content_epoch,
          executor_.status().deadline_s,
      };
      requestStatus();
    }
  }

  return result;
}

InspectionRuntimeDispatchCompletion
InspectionRuntimeController::completeGoalDispatch(bool accepted, const std::string &reason,
                                                  double now_s) {
  InspectionRuntimeDispatchCompletion result;
  if (!goal_dispatch_outstanding_) {
    return result;
  }
  result.consumed = true;
  goal_dispatch_outstanding_ = false;
  if (accepted) {
    if (!executor_.OnPlanningStarted(now_s)) {
      result.clear_motion_reason = executor_.status().reason;
    }
  } else {
    executor_.OnLegFailed(reason, now_s);
  }
  requestStatus();
  return result;
}

InspectionRuntimeDispatchCompletion
InspectionRuntimeController::completeEvidenceDispatch(const std::string &request_id, bool published,
                                                      double now_s) {
  InspectionRuntimeDispatchCompletion result;
  if (request_id.empty() || request_id != evidence_dispatch_outstanding_request_id_) {
    return result;
  }
  result.consumed = true;
  evidence_dispatch_outstanding_request_id_.clear();
  if (!published) {
    (void)executor_.OnActionResult(request_id, false, "", "evidence_request_publish_failed", now_s);
    result.clear_motion_reason = "evidence_request_publish_failed";
  }
  requestStatus();
  return result;
}

void InspectionRuntimeController::requestStatus() noexcept {
  status_requested_ = true;
}

bool InspectionRuntimeController::takeStatusDue(double now_s) noexcept {
  if (!std::isfinite(now_s)) {
    return false;
  }
  if (!status_requested_ && now_s < next_status_s_) {
    return false;
  }
  status_requested_ = false;
  next_status_s_ = now_s + config_.status_interval_s;
  return true;
}

void InspectionRuntimeController::onGoalReached(double now_s) {
  executor_.OnGoalReached(now_s);
  requestStatus();
}

void InspectionRuntimeController::resetArrivalOdomGeneration(std::uint64_t generation) noexcept {
  arrival_odom_generation_ = generation;
}

void InspectionRuntimeController::clearActivePoint() noexcept {
  active_point_.reset();
  goal_dispatch_outstanding_ = false;
}

const std::optional<lingtu::nav::inspection::Point> &
InspectionRuntimeController::activePoint() const noexcept {
  return active_point_;
}

lingtu::nav::inspection::Executor &InspectionRuntimeController::executor() noexcept {
  return executor_;
}

const lingtu::nav::inspection::Executor &InspectionRuntimeController::executor() const noexcept {
  return executor_;
}

bool InspectionRuntimeController::applyEvidenceResult(const InspectionRuntimeEvidenceResult &result,
                                                      double now_s) {
  const auto &status = executor_.status();
  if (status.state != InspectionRunState::kActionPending || status.action_request_id.empty() ||
      result.request_id != status.action_request_id) {
    return false;
  }
  std::string reason = result.reason;
  if (!result.persisted && reason.empty()) {
    reason = "evidence_not_persisted";
  }
  return executor_.OnActionResult(result.request_id, result.persisted, result.evidence_id, reason,
                                  now_s);
}

void InspectionRuntimeController::appendClearMotion(InspectionRuntimeTickResult &result,
                                                    const std::string &reason) const {
  result.ordered_intents.push_back({InspectionRuntimeIntentKind::kClearMotion, reason});
}

}  // namespace lingtu::nav::endpoint
