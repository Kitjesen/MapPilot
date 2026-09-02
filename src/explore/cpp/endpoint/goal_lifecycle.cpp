#include "endpoint/goal_lifecycle.hpp"

namespace lingtu::nav::endpoint {

ExploreGoalLifecycleReaction
reactToNavigationGoalLifecycle(const PendingExploreGoalLifecycle *pending,
                               const NavigationGoalLifecycleEvent &event) {
  ExploreGoalLifecycleReaction reaction;
  if (pending == nullptr || pending->request_id.empty() || event.request_id.empty()) {
    return reaction;
  }
  const bool matches_goal = pending->request_id == event.request_id;
  const bool matches_cancel = !pending->cancel_request_id.empty() &&
                              pending->cancel_request_id == event.request_id;
  if (!matches_goal && !matches_cancel) {
    return reaction;
  }

  reaction.matched = true;
  reaction.reason = event.reason;
  switch (event.state) {
    case lingtu::message::NavigationGoalState::Planning:
    case lingtu::message::NavigationGoalState::PathActive:
    case lingtu::message::NavigationGoalState::Paused:
      return reaction;
    case lingtu::message::NavigationGoalState::Failed:
      reaction.clear_pending = true;
      reaction.exclude_target = true;
      reaction.request_replan = true;
      return reaction;
    case lingtu::message::NavigationGoalState::Reached:
      reaction.clear_pending = true;
      reaction.exclude_target = true;
      return reaction;
    case lingtu::message::NavigationGoalState::Cancelled:
      reaction.clear_pending = true;
      return reaction;
  }
  return reaction;
}

bool shouldMarkExploreGoalVisited(const ExploreGoalLifecycleReaction &reaction,
                                  bool pending_map_is_current) {
  return reaction.exclude_target && pending_map_is_current;
}

RejectedExploreStartReaction reactToRejectedExploreStart(bool cancellation_pending,
                                                         bool exploration_running) noexcept {
  return {
      !cancellation_pending && exploration_running,
      cancellation_pending,
  };
}

}  // namespace lingtu::nav::endpoint
