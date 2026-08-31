#include "runtime/goal/retry.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

bool finite(double value) {
  return std::isfinite(value);
}

bool isActiveState(BoundedGoalReplanState state) {
  return state == BoundedGoalReplanState::kBackoffPending ||
      state == BoundedGoalReplanState::kReplanInFlight;
}

}  // namespace

BoundedGoalReplanController::BoundedGoalReplanController(BoundedGoalReplanConfig config)
    : config_(config) {
  if (!finite(config_.backoff_s) || config_.backoff_s < 0.0) {
    throw std::invalid_argument("bounded goal replan backoff must be finite and non-negative");
  }
}

BoundedGoalReplanDecision BoundedGoalReplanController::observeGoal(
    const BoundedGoalReplanGoal &goal, double now_s) {
  if (!validTime(now_s)) {
    return reject(reason_);
  }
  if (!validGoal(goal)) {
    consumeAttempt("invalid_goal_identity");
    return {BoundedGoalReplanAction::kReject, reason_};
  }
  if (!has_goal_) {
    bindGoal(goal);
    state_ = BoundedGoalReplanState::kIdle;
    budget_consumed_ = false;
    start_emitted_ = false;
    deadline_s_ = 0.0;
    return transition(BoundedGoalReplanAction::kNone, "new_goal_observed");
  }
  if (!sameKey(goal)) {
    const bool superseded_active = isActiveState(state_);
    bindGoal(goal);
    state_ = BoundedGoalReplanState::kIdle;
    budget_consumed_ = false;
    start_emitted_ = false;
    deadline_s_ = 0.0;
    return transition(superseded_active ? BoundedGoalReplanAction::kCancel
                                        : BoundedGoalReplanAction::kNone,
                      superseded_active ? "superseded_by_new_goal" : "new_goal_observed");
  }
  const bool same_map = map_identity_ &&
      lingtu::nav::plan::sameMapIdentity(*map_identity_, goal.map_identity);
  if (!same_map) {
    if (isActiveState(state_)) {
      consumeAttempt("map_identity_changed");
      map_identity_ = goal.map_identity;
      active_goal_epoch_ = goal.active_goal_epoch;
      return {BoundedGoalReplanAction::kCancel, reason_};
    }
    map_identity_ = goal.map_identity;
    active_goal_epoch_ = goal.active_goal_epoch;
    return transition(BoundedGoalReplanAction::kNone,
                      "map_identity_advanced_budget_retained");
  }
  if (goal.active_goal_epoch != active_goal_epoch_) {
    if (isActiveState(state_)) {
      consumeAttempt("active_goal_epoch_changed");
      active_goal_epoch_ = goal.active_goal_epoch;
      return {BoundedGoalReplanAction::kCancel, reason_};
    }
    active_goal_epoch_ = goal.active_goal_epoch;
    return transition(BoundedGoalReplanAction::kNone,
                      "active_goal_epoch_advanced_budget_retained");
  }
  return transition(BoundedGoalReplanAction::kNone, "goal_unchanged");
}

BoundedGoalReplanDecision BoundedGoalReplanController::armAfterConfirmedStop(
    const BoundedGoalReplanGoal &goal, double now_s, bool stop_confirmed) {
  if (!validTime(now_s)) {
    return reject(reason_);
  }
  if (!validGoal(goal)) {
    consumeAttempt("invalid_goal_identity");
    return {BoundedGoalReplanAction::kReject, reason_};
  }
  if (!has_goal_ || !sameKey(goal)) {
    bindGoal(goal);
    budget_consumed_ = false;
    start_emitted_ = false;
  }
  if (!sameToken(goal)) {
    consumeAttempt(goal.map_identity.valid() && map_identity_ &&
                           lingtu::nav::plan::sameMapIdentity(*map_identity_, goal.map_identity)
                       ? "stale_active_goal_epoch"
                       : "stale_map_identity");
    return {BoundedGoalReplanAction::kCancel, reason_};
  }
  if (budget_consumed_) {
    if (state_ == BoundedGoalReplanState::kBackoffPending) {
      return transition(BoundedGoalReplanAction::kHold, "backoff_already_pending");
    }
    if (state_ == BoundedGoalReplanState::kReplanInFlight) {
      return transition(BoundedGoalReplanAction::kHold, "replan_already_in_flight");
    }
    return transition(BoundedGoalReplanAction::kReject, "replan_budget_consumed");
  }
  budget_consumed_ = true;
  if (!stop_confirmed) {
    state_ = BoundedGoalReplanState::kAttemptConsumed;
    return transition(BoundedGoalReplanAction::kConsume, "stop_not_confirmed_budget_consumed");
  }
  const double max_value = std::numeric_limits<double>::max();
  if ((config_.backoff_s > 0.0 && now_s == max_value) ||
      now_s > max_value - config_.backoff_s) {
    state_ = BoundedGoalReplanState::kAttemptConsumed;
    return transition(BoundedGoalReplanAction::kReject, "deadline_overflow_budget_consumed");
  }
  deadline_s_ = now_s + config_.backoff_s;
  state_ = BoundedGoalReplanState::kBackoffPending;
  start_emitted_ = false;
  return transition(BoundedGoalReplanAction::kHold, "backoff_pending");
}

BoundedGoalReplanDecision BoundedGoalReplanController::tick(const BoundedGoalReplanGoal &goal,
                                                            double now_s) {
  if (!validTime(now_s)) {
    return reject(reason_);
  }
  if (!has_goal_ || !sameKey(goal)) {
    return transition(BoundedGoalReplanAction::kIgnore, "tick_for_unknown_goal");
  }
  if (!sameToken(goal)) {
    consumeAttempt(goal.map_identity.valid() && map_identity_ &&
                           lingtu::nav::plan::sameMapIdentity(*map_identity_, goal.map_identity)
                       ? "stale_active_goal_epoch"
                       : "stale_map_identity");
    return {BoundedGoalReplanAction::kCancel, reason_};
  }
  if (state_ != BoundedGoalReplanState::kBackoffPending) {
    return transition(BoundedGoalReplanAction::kNone, "no_pending_replan");
  }
  if (now_s < deadline_s_) {
    return transition(BoundedGoalReplanAction::kHold, "backoff_pending");
  }
  if (start_emitted_) {
    return transition(BoundedGoalReplanAction::kNone, "start_already_emitted");
  }
  start_emitted_ = true;
  state_ = BoundedGoalReplanState::kReplanInFlight;
  return transition(BoundedGoalReplanAction::kStart, "start_replan");
}

BoundedGoalReplanDecision BoundedGoalReplanController::completeReplan(
    const BoundedGoalReplanGoal &goal, double now_s, bool success) {
  if (!validTime(now_s)) {
    return reject(reason_);
  }
  if (!has_goal_ || !sameKey(goal) || !sameToken(goal) ||
      state_ != BoundedGoalReplanState::kReplanInFlight) {
    return transition(BoundedGoalReplanAction::kIgnore, "stale_replan_completion");
  }
  state_ = BoundedGoalReplanState::kAttemptConsumed;
  return transition(success ? BoundedGoalReplanAction::kNone : BoundedGoalReplanAction::kReject,
                    success ? "replan_completed" : "replan_failed_budget_consumed");
}

BoundedGoalReplanDecision BoundedGoalReplanController::cancel(
    const BoundedGoalReplanGoal &goal, double now_s, const std::string &reason) {
  if (!validTime(now_s)) {
    return reject(reason_);
  }
  if (!has_goal_ || !sameKey(goal)) {
    return transition(BoundedGoalReplanAction::kIgnore, "cancel_for_unknown_goal");
  }
  consumeAttempt(reason.empty() ? "cancelled_budget_consumed" : reason);
  return {BoundedGoalReplanAction::kCancel, reason_};
}

void BoundedGoalReplanController::reset() {
  clearGoal();
  state_ = BoundedGoalReplanState::kIdle;
  budget_consumed_ = false;
  start_emitted_ = false;
  deadline_s_ = 0.0;
  last_time_s_.reset();
  reason_ = "reset";
}

BoundedGoalReplanSnapshot BoundedGoalReplanController::snapshot() const {
  BoundedGoalReplanSnapshot result;
  result.state = state_;
  result.task_id = key_.task_id;
  result.request_id = key_.request_id;
  result.active_goal_epoch = active_goal_epoch_;
  result.map_identity = map_identity_;
  result.budget_consumed = budget_consumed_;
  result.start_emitted = start_emitted_;
  result.deadline_s = deadline_s_;
  result.reason = reason_;
  return result;
}

BoundedGoalReplanDecision BoundedGoalReplanController::reject(const std::string &reason) {
  return {BoundedGoalReplanAction::kReject, reason};
}

BoundedGoalReplanDecision BoundedGoalReplanController::transition(
    BoundedGoalReplanAction action, const std::string &reason) {
  reason_ = reason;
  return {action, reason_};
}

bool BoundedGoalReplanController::validGoal(const BoundedGoalReplanGoal &goal) const {
  return !goal.task_id.empty() && !goal.request_id.empty() && goal.active_goal_epoch > 0U &&
      goal.map_identity.valid();
}

bool BoundedGoalReplanController::sameKey(const BoundedGoalReplanGoal &goal) const {
  return has_goal_ && key_.task_id == goal.task_id && key_.request_id == goal.request_id;
}

bool BoundedGoalReplanController::sameToken(const BoundedGoalReplanGoal &goal) const {
  return sameKey(goal) && active_goal_epoch_ == goal.active_goal_epoch && map_identity_ &&
      lingtu::nav::plan::sameMapIdentity(*map_identity_, goal.map_identity);
}

bool BoundedGoalReplanController::validTime(double now_s) {
  if (!finite(now_s)) {
    consumeAttempt("invalid_time_budget_consumed");
    return false;
  }
  if (last_time_s_ && now_s < *last_time_s_) {
    consumeAttempt("clock_regression_budget_consumed");
    return false;
  }
  last_time_s_ = now_s;
  return true;
}

void BoundedGoalReplanController::bindGoal(const BoundedGoalReplanGoal &goal) {
  key_.task_id = goal.task_id;
  key_.request_id = goal.request_id;
  active_goal_epoch_ = goal.active_goal_epoch;
  map_identity_ = goal.map_identity;
  has_goal_ = true;
}

void BoundedGoalReplanController::consumeAttempt(const std::string &reason) {
  budget_consumed_ = true;
  start_emitted_ = false;
  if (has_goal_) {
    state_ = BoundedGoalReplanState::kAttemptConsumed;
  } else {
    state_ = BoundedGoalReplanState::kIdle;
  }
  reason_ = reason;
}

void BoundedGoalReplanController::clearGoal() {
  key_ = {};
  active_goal_epoch_ = 0U;
  map_identity_.reset();
  has_goal_ = false;
}

}  // namespace lingtu::nav::endpoint
