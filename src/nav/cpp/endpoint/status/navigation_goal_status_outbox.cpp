#include "status/navigation_goal_status_outbox.hpp"

#include <utility>

namespace lingtu::nav::endpoint {

NavigationGoalStatusOutbox::NavigationGoalStatusOutbox(ObserveCallback observe, WriteCallback write)
    : observe_(std::move(observe)), write_(std::move(write)) {}

bool NavigationGoalStatusOutbox::record(const GoalPlanStatus &status) {
  if (!valid(status) || containsIdentity(status)) {
    return false;
  }
  records_.push_back(Record{status, false});
  if (observe_) {
    observe_(records_.back().status);
  }
  return true;
}

std::size_t NavigationGoalStatusOutbox::flush() {
  std::size_t delivered_count = 0U;
  for (auto &record : records_) {
    if (record.delivered) {
      continue;
    }
    if (!write_ || !write_(record.status)) {
      break;
    }
    record.delivered = true;
    ++delivered_count;
  }
  return delivered_count;
}

bool NavigationGoalStatusOutbox::delivered(const GoalPlanStatus &status) const {
  if (!valid(status)) {
    return false;
  }
  for (const auto &record : records_) {
    if (record.delivered && sameIdentity(record.status, status)) {
      return true;
    }
  }
  return false;
}

bool NavigationGoalStatusOutbox::valid(const GoalPlanStatus &status) {
  return !status.task_id.empty() && !status.request_id.empty() && status.goal_epoch > 0U &&
         validState(status.state);
}

bool NavigationGoalStatusOutbox::validState(lingtu::message::NavigationGoalState state) {
  switch (state) {
    case lingtu::message::NavigationGoalState::Planning:
    case lingtu::message::NavigationGoalState::PathActive:
    case lingtu::message::NavigationGoalState::Paused:
    case lingtu::message::NavigationGoalState::Failed:
    case lingtu::message::NavigationGoalState::Reached:
    case lingtu::message::NavigationGoalState::Cancelled:
      return true;
  }
  return false;
}

bool NavigationGoalStatusOutbox::sameIdentity(const GoalPlanStatus &left,
                                              const GoalPlanStatus &right) {
  return left.task_id == right.task_id && left.request_id == right.request_id &&
         left.goal_epoch == right.goal_epoch && left.state == right.state &&
         left.reason == right.reason &&
         left.project_to_navigation_state == right.project_to_navigation_state;
}

bool NavigationGoalStatusOutbox::containsIdentity(const GoalPlanStatus &status) const {
  for (const auto &record : records_) {
    if (sameIdentity(record.status, status)) {
      return true;
    }
  }
  return false;
}

}  // namespace lingtu::nav::endpoint
