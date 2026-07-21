#pragma once

#include "far/far_planner.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace lingtu::maps {
class MapStore;
}

namespace lingtu::nav::endpoint {

class ValidatedFarMap {
 public:
  const plan::far::FarGridMap& map() const { return map_; }
  const plan::MapIdentity& identity() const { return map_.identity; }
  std::uint64_t generation() const { return map_.generation; }

 private:
  friend class ActiveOccupancyGate;
  explicit ValidatedFarMap(plan::far::FarGridMap map)
      : map_(std::move(map)) {}

  plan::far::FarGridMap map_;
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
  explicit ActiveOccupancyGate(
      std::filesystem::path map_root = {},
      std::string expected_frame_id = "map");
  ~ActiveOccupancyGate();

  ActiveOccupancyGate(const ActiveOccupancyGate&) = delete;
  ActiveOccupancyGate& operator=(const ActiveOccupancyGate&) = delete;

  ActiveOccupancyGateResult prepare(
      const std::filesystem::path& configured_occupancy_path) const;
  ActiveOccupancyIdentityResult currentIdentity(
      const std::filesystem::path& configured_occupancy_path) const;

 private:
  lingtu::maps::MapStore* resolveStore(
      const std::filesystem::path& configured_occupancy_path,
      std::string* error) const;

  std::filesystem::path map_root_;
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  mutable std::filesystem::path resolved_map_root_;
  mutable std::unique_ptr<lingtu::maps::MapStore> map_store_;
  mutable std::shared_ptr<const ValidatedFarMap> cached_artifact_;
  mutable std::uint64_t next_generation_{1U};
};

plan::GlobalPlanResult runWithActiveOccupancy(
    const ActiveOccupancyGate& gate,
    const std::filesystem::path& configured_occupancy_path,
    plan::far::FarPlanner& planner,
    const plan::GlobalPlanRequest& request,
    const plan::GlobalPlanCancelCheck& cancel_check = {});

}  // namespace lingtu::nav::endpoint
