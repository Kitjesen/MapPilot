#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav/cpp/planning/global/global_planner_contract.hpp"

namespace lingtu::maps {
class MapStore;
}

namespace lingtu::nav::endpoint {

class ValidatedOctomap {
 public:
  ~ValidatedOctomap();

  ValidatedOctomap(const ValidatedOctomap &) = delete;
  ValidatedOctomap &operator=(const ValidatedOctomap &) = delete;

  const std::filesystem::path &loadPath() const { return load_path_; }
  const lingtu::nav::plan::MapIdentity &identity() const { return identity_; }
  const std::string &mapId() const { return identity_.map_id; }
  const std::string &sha256() const { return identity_.artifact_sha256; }

 private:
  friend class ActiveOctomapGate;

  ValidatedOctomap(std::filesystem::path load_path, lingtu::nav::plan::MapIdentity identity);

  std::filesystem::path load_path_;
  lingtu::nav::plan::MapIdentity identity_;
};

struct ActiveOctomapGateResult {
  std::shared_ptr<const ValidatedOctomap> artifact;
  std::string reason;

  bool ok() const { return artifact != nullptr; }
};

struct ActiveOctomapIdentityResult {
  std::optional<lingtu::nav::plan::MapIdentity> identity;
  std::string reason;

  bool ok() const { return identity.has_value(); }
};

class ActiveOctomapGate {
 public:
  explicit ActiveOctomapGate(std::filesystem::path map_root = {},
                             std::string expected_frame_id = "map");
  ~ActiveOctomapGate();

  ActiveOctomapGate(const ActiveOctomapGate &) = delete;
  ActiveOctomapGate &operator=(const ActiveOctomapGate &) = delete;

  ActiveOctomapGateResult prepare(const std::filesystem::path &configured_octomap_path) const;
  ActiveOctomapIdentityResult
  currentIdentity(const std::filesystem::path &configured_octomap_path) const;
  ActiveOctomapIdentityResult
  currentDeclaredIdentity(const std::filesystem::path &configured_octomap_path) const;

 private:
  lingtu::maps::MapStore *resolveStore(const std::filesystem::path &configured_octomap_path,
                                       std::string *error) const;

  std::filesystem::path map_root_;
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  mutable std::filesystem::path resolved_map_root_;
  mutable std::unique_ptr<lingtu::maps::MapStore> map_store_;
  mutable std::shared_ptr<const ValidatedOctomap> cached_artifact_;
};

using OctomapPlanner = std::function<lingtu::nav::plan::GlobalPlanResult(
    const std::filesystem::path &, const lingtu::nav::plan::MapIdentity &,
    const lingtu::nav::plan::GlobalPlanRequest &,
    const lingtu::nav::plan::GlobalPlanCancelCheck &)>;

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate &gate, const std::filesystem::path &configured_octomap_path,
    const lingtu::nav::plan::GlobalPlanRequest &request,
    const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check, const OctomapPlanner &planner);

lingtu::nav::plan::GlobalPlanResult
runWithActiveOctomap(const ActiveOctomapGate &gate,
                     const std::filesystem::path &configured_octomap_path,
                     const lingtu::nav::plan::GlobalPlanRequest &request,
                     const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check = {});

}  // namespace lingtu::nav::endpoint
