#include "endpoint/saved_coverage_grid.hpp"

#include <cmath>
#include <exception>
#include <utility>

namespace lingtu::nav::endpoint {

SavedCoverageGridResult
buildSavedCoverageGrid(const lingtu::maps::OccupancyArtifactData &artifact,
                       const plan::MapIdentity &artifact_identity,
                       const lingtu::explore::ExploreMapIdentity &snapshot_identity) {
  if (snapshot_identity.live) {
    return {std::nullopt, "saved coverage grid requires the map route"};
  }
  try {
    if (artifact.rows <= 0 || artifact.cols <= 0 ||
        artifact.grid.size() != static_cast<std::size_t>(artifact.rows) *
                                     static_cast<std::size_t>(artifact.cols) ||
        !std::isfinite(artifact.resolution) || artifact.resolution <= 0.0 ||
        !std::isfinite(artifact.origin_x) || !std::isfinite(artifact.origin_y)) {
      return {std::nullopt, "validated occupancy grid is invalid"};
    }
  } catch (const std::exception &error) {
    return {std::nullopt, "validated occupancy grid is invalid: " + std::string(error.what())};
  }
  if (artifact_identity.map_id != snapshot_identity.map_id) {
    return {std::nullopt, "saved coverage map_id does not match the exploration snapshot"};
  }
  if (artifact_identity.content_epoch != snapshot_identity.map_content_epoch) {
    return {std::nullopt, "saved coverage content epoch does not match the exploration snapshot"};
  }
  if (artifact_identity.frame_id != snapshot_identity.frame_id) {
    return {std::nullopt, "saved coverage frame does not match the exploration snapshot"};
  }
  lingtu::explore::Grid2D grid;
  grid.width = artifact.cols;
  grid.height = artifact.rows;
  grid.resolution = artifact.resolution;
  grid.origin_x = artifact.origin_x;
  grid.origin_y = artifact.origin_y;
  grid.cells = artifact.grid;
  if (!grid.valid()) {
    return {std::nullopt, "saved coverage grid conversion produced invalid geometry"};
  }
  return {std::move(grid), {}};
}

}  // namespace lingtu::nav::endpoint
