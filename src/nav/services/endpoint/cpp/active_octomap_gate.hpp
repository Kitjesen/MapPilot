#pragma once

#include "global_planner_contract.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace lingtu::maps {
class MapStore;
}

namespace lingtu::nav::endpoint {

class ValidatedOctomap {
 public:
  ~ValidatedOctomap();

  ValidatedOctomap(const ValidatedOctomap&) = delete;
  ValidatedOctomap& operator=(const ValidatedOctomap&) = delete;

  const std::filesystem::path& loadPath() const { return load_path_; }
  const std::string& mapId() const { return map_id_; }
  const std::string& sha256() const { return sha256_; }

 private:
  friend class ActiveOctomapGate;

  ValidatedOctomap(
      std::filesystem::path load_path,
      std::string map_id,
      std::string sha256);

  std::filesystem::path load_path_;
  std::string map_id_;
  std::string sha256_;
};

struct ActiveOctomapGateResult {
  std::unique_ptr<ValidatedOctomap> artifact;
  std::string reason;

  bool ok() const { return artifact != nullptr; }
};

class ActiveOctomapGate {
 public:
  explicit ActiveOctomapGate(
      std::filesystem::path map_root = {},
      std::string expected_frame_id = "map");
  ~ActiveOctomapGate();

  ActiveOctomapGate(const ActiveOctomapGate&) = delete;
  ActiveOctomapGate& operator=(const ActiveOctomapGate&) = delete;

  ActiveOctomapGateResult prepare(
      const std::filesystem::path& configured_octomap_path) const;

 private:
  std::filesystem::path map_root_;
  std::string expected_frame_id_;
  mutable std::filesystem::path resolved_map_root_;
  mutable std::unique_ptr<lingtu::maps::MapStore> map_store_;
};

using OctomapPlanner = lingtu::nav::plan::GlobalPlannerFunction;

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate& gate,
    const lingtu::nav::plan::GlobalPlanRequest& request,
    const lingtu::nav::plan::GlobalPlanCancelCheck& cancel_check,
    const OctomapPlanner& planner);

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate& gate,
    const lingtu::nav::plan::GlobalPlanRequest& request,
    const lingtu::nav::plan::GlobalPlanCancelCheck& cancel_check = {});

}  // namespace lingtu::nav::endpoint
