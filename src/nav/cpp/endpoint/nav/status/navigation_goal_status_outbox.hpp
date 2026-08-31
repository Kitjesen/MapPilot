#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "runtime/goal/plan.hpp"

namespace lingtu::nav::endpoint {

class NavigationGoalStatusOutbox {
 public:
  using ObserveCallback = std::function<void(const GoalPlanStatus &)>;
  using WriteCallback = std::function<bool(const GoalPlanStatus &)>;

  NavigationGoalStatusOutbox(ObserveCallback observe, WriteCallback write);

  [[nodiscard]] bool record(const GoalPlanStatus &status);
  [[nodiscard]] std::size_t flush();
  [[nodiscard]] bool delivered(const GoalPlanStatus &status) const;

 private:
  struct Record {
    GoalPlanStatus status;
    bool delivered{false};
  };

  [[nodiscard]] static bool valid(const GoalPlanStatus &status);
  [[nodiscard]] static bool validState(lingtu::message::NavigationGoalState state);
  [[nodiscard]] static bool sameIdentity(const GoalPlanStatus &left, const GoalPlanStatus &right);
  [[nodiscard]] bool containsIdentity(const GoalPlanStatus &status) const;

  ObserveCallback observe_;
  WriteCallback write_;
  std::vector<Record> records_;
};

}  // namespace lingtu::nav::endpoint
