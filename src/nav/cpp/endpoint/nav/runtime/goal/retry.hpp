#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "planning/global/contract.hpp"

namespace lingtu::nav::endpoint {

struct BoundedGoalReplanConfig {
  double backoff_s{0.5};
};

struct BoundedGoalReplanGoal {
  std::string task_id;
  std::string request_id;
  std::uint64_t active_goal_epoch{0U};
  lingtu::nav::plan::MapIdentity map_identity;
};

enum class BoundedGoalReplanState {
  kIdle,
  kBackoffPending,
  kReplanInFlight,
  kAttemptConsumed,
};

enum class BoundedGoalReplanAction {
  kNone,
  kHold,
  kStart,
  kReject,
  kCancel,
  kConsume,
  kIgnore,
};

struct BoundedGoalReplanDecision {
  BoundedGoalReplanAction action{BoundedGoalReplanAction::kNone};
  std::string reason{"idle"};
};

struct BoundedGoalReplanSnapshot {
  BoundedGoalReplanState state{BoundedGoalReplanState::kIdle};
  std::string task_id;
  std::string request_id;
  std::uint64_t active_goal_epoch{0U};
  std::optional<lingtu::nav::plan::MapIdentity> map_identity;
  bool budget_consumed{false};
  bool start_emitted{false};
  double deadline_s{0.0};
  std::string reason{"idle"};
};

class BoundedGoalReplanController {
 public:
  explicit BoundedGoalReplanController(BoundedGoalReplanConfig config = {});

  [[nodiscard]] BoundedGoalReplanDecision observeGoal(const BoundedGoalReplanGoal &goal,
                                                      double now_s);
  [[nodiscard]] BoundedGoalReplanDecision armAfterConfirmedStop(
      const BoundedGoalReplanGoal &goal, double now_s, bool stop_confirmed);
  [[nodiscard]] BoundedGoalReplanDecision tick(const BoundedGoalReplanGoal &goal, double now_s);
  [[nodiscard]] BoundedGoalReplanDecision completeReplan(const BoundedGoalReplanGoal &goal,
                                                         double now_s, bool success);
  [[nodiscard]] BoundedGoalReplanDecision cancel(const BoundedGoalReplanGoal &goal,
                                                 double now_s, const std::string &reason);
  void reset();

  [[nodiscard]] BoundedGoalReplanSnapshot snapshot() const;

 private:
  struct GoalKey {
    std::string task_id;
    std::string request_id;
  };

  [[nodiscard]] BoundedGoalReplanDecision reject(const std::string &reason);
  [[nodiscard]] BoundedGoalReplanDecision transition(BoundedGoalReplanAction action,
                                                     const std::string &reason);
  [[nodiscard]] bool validGoal(const BoundedGoalReplanGoal &goal) const;
  [[nodiscard]] bool sameKey(const BoundedGoalReplanGoal &goal) const;
  [[nodiscard]] bool sameToken(const BoundedGoalReplanGoal &goal) const;
  [[nodiscard]] bool validTime(double now_s);
  void bindGoal(const BoundedGoalReplanGoal &goal);
  void consumeAttempt(const std::string &reason);
  void clearGoal();

  BoundedGoalReplanConfig config_{};
  BoundedGoalReplanState state_{BoundedGoalReplanState::kIdle};
  GoalKey key_{};
  std::uint64_t active_goal_epoch_{0U};
  std::optional<lingtu::nav::plan::MapIdentity> map_identity_;
  bool has_goal_{false};
  bool budget_consumed_{false};
  bool start_emitted_{false};
  double deadline_s_{0.0};
  std::optional<double> last_time_s_;
  std::string reason_{"idle"};
};

}  // namespace lingtu::nav::endpoint
