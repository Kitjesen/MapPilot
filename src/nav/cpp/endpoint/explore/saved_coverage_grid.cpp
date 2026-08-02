#include "explore/saved_coverage_grid.hpp"

#include <exception>
#include <utility>

namespace lingtu::nav::endpoint {

SavedCoverageGridResult
buildSavedCoverageGrid(const plan::far::FarGridMap &artifact, const std::string &source_map_sha256,
                       const lingtu::explore::ExploreMapIdentity &snapshot_identity) {
  if (snapshot_identity.live) {
    return {std::nullopt, "saved coverage grid requires the map route"};
  }
  try {
    artifact.Validate();
  } catch (const std::exception &error) {
    return {std::nullopt, "validated occupancy grid is invalid: " + std::string(error.what())};
  }
  if (artifact.identity.map_id != snapshot_identity.map_id) {
    return {std::nullopt, "saved coverage map_id does not match the exploration snapshot"};
  }
  if (artifact.identity.version != snapshot_identity.map_version) {
    return {std::nullopt, "saved coverage map version does not match the exploration snapshot"};
  }
  if (artifact.identity.frame_id != snapshot_identity.frame_id ||
      artifact.frame_id != snapshot_identity.frame_id) {
    return {std::nullopt, "saved coverage frame does not match the exploration snapshot"};
  }
  if (source_map_sha256.empty() || source_map_sha256 != snapshot_identity.artifact_hash) {
    return {
        std::nullopt,
        "saved coverage source map hash does not match the exploration snapshot",
    };
  }

  lingtu::explore::Grid2D grid;
  grid.width = artifact.width;
  grid.height = artifact.height;
  grid.resolution = artifact.resolution_m;
  grid.origin_x = artifact.origin_x_m;
  grid.origin_y = artifact.origin_y_m;
  grid.cells = artifact.cells;
  if (!grid.valid()) {
    return {std::nullopt, "saved coverage grid conversion produced invalid geometry"};
  }
  return {std::move(grid), {}};
}

}  // namespace lingtu::nav::endpoint
