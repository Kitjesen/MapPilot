#pragma once

#include <string>

#include "message/cpp/navigation_command.hpp"

namespace lingtu::nav::endpoint {

// Pure lifecycle seam between the native navigation owner and the native TARE
// policy. It is intentionally DDS-free so the matching and terminal-state
// rules can be exercised without a middleware runtime.
struct PendingExploreGoalLifecycle {
  std::string request_id;
};

struct NavigationGoalLifecycleEvent {
  std::string request_id;
  lingtu::message::NavigationGoalState state{lingtu::message::NavigationGoalState::Planning};
  std::string reason;
};

struct ExploreGoalLifecycleReaction {
  bool matched{false};
  bool clear_pending{false};
  bool exclude_target{false};
  bool request_replan{false};
  std::string reason;
};

ExploreGoalLifecycleReaction
reactToNavigationGoalLifecycle(const PendingExploreGoalLifecycle *pending,
                               const NavigationGoalLifecycleEvent &event);

bool shouldMarkExploreGoalVisited(const ExploreGoalLifecycleReaction &reaction,
                                  bool pending_map_is_current);

}  // namespace lingtu::nav::endpoint
