#include "input/active/octomap.hpp"
#include "input/active/occupancy.hpp"

#include <system_error>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

template <typename Result>
Result boundIdentity(const std::filesystem::path &artifact_path,
                     const lingtu::nav::plan::MapIdentity &identity,
                     const char *artifact_name) {
  if (!identity.valid()) {
    return {std::nullopt, "runtime map identity is incomplete"};
  }
  if (artifact_path.empty()) {
    return {std::nullopt, std::string("configured ") + artifact_name + " path is empty"};
  }
  std::error_code error;
  const auto resolved = std::filesystem::canonical(artifact_path, error);
  if (error || !std::filesystem::is_regular_file(resolved, error) || error) {
    return {std::nullopt, std::string("configured ") + artifact_name + " is not a file"};
  }
  if (resolved.parent_path().filename() != identity.map_id) {
    return {std::nullopt,
            std::string("configured ") + artifact_name +
                " does not belong to runtime map " + identity.map_id};
  }
  return {identity, {}};
}

}  // namespace

ActiveOctomapGate::ActiveOctomapGate(lingtu::nav::plan::MapIdentity identity)
    : identity_(std::move(identity)) {}

ActiveOctomapIdentityResult ActiveOctomapGate::currentIdentity(
    const std::filesystem::path &configured_octomap_path) const {
  return boundIdentity<ActiveOctomapIdentityResult>(configured_octomap_path, identity_,
                                                     "octomap");
}

ActiveOccupancyGate::ActiveOccupancyGate(plan::MapIdentity identity)
    : identity_(std::move(identity)) {}

ActiveOccupancyIdentityResult ActiveOccupancyGate::currentIdentity(
    const std::filesystem::path &configured_occupancy_path) const {
  return boundIdentity<ActiveOccupancyIdentityResult>(configured_occupancy_path, identity_,
                                                       "occupancy");
}

}  // namespace lingtu::nav::endpoint
