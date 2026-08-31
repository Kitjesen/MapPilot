#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "planning/global/far/planner.hpp"

namespace lingtu::nav::endpoint {

class ValidatedFarMap {
 public:
  const plan::far_planner::FarGridMap &map() const { return map_; }
  const plan::MapIdentity &identity() const { return map_.identity; }
  std::uint64_t generation() const { return map_.generation; }

 private:
  friend class ActiveOccupancyGate;
  explicit ValidatedFarMap(plan::far_planner::FarGridMap map) : map_(std::move(map)) {}

  plan::far_planner::FarGridMap map_;
};

struct ActiveOccupancyGateResult {
  std::shared_ptr<const ValidatedFarMap> artifact;
  std::string reason;

  bool ok() const { return artifact != nullptr; }
};

struct ActiveOccupancyIdentityResult {
  std::optional<plan::MapIdentity> identity;
  std::string reason;

  bool ok() const { return identity.has_value(); }
};

class ActiveOccupancyGate {
 public:
  explicit ActiveOccupancyGate(plan::MapIdentity identity);

  ActiveOccupancyGate(const ActiveOccupancyGate &) = delete;
  ActiveOccupancyGate &operator=(const ActiveOccupancyGate &) = delete;

  ActiveOccupancyGateResult prepare(const std::filesystem::path &configured_occupancy_path) const;
  ActiveOccupancyIdentityResult
  currentIdentity(const std::filesystem::path &configured_occupancy_path) const;

 private:
  plan::MapIdentity identity_;
  mutable std::mutex mutex_;
  mutable std::shared_ptr<const ValidatedFarMap> cached_artifact_;
  mutable std::uint64_t next_generation_{1U};
};

plan::GlobalPlanResult
runWithActiveOccupancy(const ActiveOccupancyGate &gate,
                       const std::filesystem::path &configured_occupancy_path,
                       plan::far_planner::FarPlanner &planner, const plan::GlobalPlanRequest &request,
                       const plan::GlobalPlanCancelCheck &cancel_check = {});

}  // namespace lingtu::nav::endpoint
