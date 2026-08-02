#pragma once

#include <optional>
#include <string>

#include "nav/cpp/planning/global/global_planner_contract.hpp"

namespace lingtu::nav::endpoint {

struct GoalPlanMapIdentityResult {
  std::optional<lingtu::nav::plan::MapIdentity> identity;
  std::string reason;
};

}  // namespace lingtu::nav::endpoint
