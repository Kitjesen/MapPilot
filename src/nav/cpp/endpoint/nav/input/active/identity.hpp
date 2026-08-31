#pragma once

#include <optional>
#include <string>

#include "planning/global/contract.hpp"

namespace lingtu::nav::endpoint {

struct GoalPlanMapIdentityResult {
  std::optional<lingtu::nav::plan::MapIdentity> identity;
  std::string reason;
};

}  // namespace lingtu::nav::endpoint
