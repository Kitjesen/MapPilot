#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "planning/global/contract.hpp"
#include "lingtu/maps/build/grids.hpp"

namespace lingtu::nav::endpoint {

class ValidatedOccupancyMap {
 public:
  const lingtu::maps::OccupancyArtifactData &data() const { return data_; }
  const plan::MapIdentity &identity() const { return identity_; }
  std::uint64_t generation() const { return generation_; }

 private:
  friend class ActiveOccupancyGate;
  explicit ValidatedOccupancyMap(lingtu::maps::OccupancyArtifactData data,
                                plan::MapIdentity identity,
                                std::uint64_t generation)
      : data_(std::move(data)), identity_(std::move(identity)), generation_(generation) {}

  lingtu::maps::OccupancyArtifactData data_;
  plan::MapIdentity identity_;
  std::uint64_t generation_{0U};
};

struct ActiveOccupancyGateResult {
  std::shared_ptr<const ValidatedOccupancyMap> artifact;
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
  mutable std::shared_ptr<const ValidatedOccupancyMap> cached_artifact_;
  mutable std::uint64_t next_generation_{1U};
};

}  // namespace lingtu::nav::endpoint
