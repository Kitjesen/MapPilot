#pragma once

#include <optional>
#include <string>

#include "explore_contract.hpp"
#include "far/far_planner.hpp"

namespace lingtu::nav::endpoint {

struct SavedCoverageGridResult {
  std::optional<lingtu::explore::Grid2D> grid;
  std::string reason;

  [[nodiscard]] bool ok() const noexcept { return grid.has_value(); }
};

// Convert only an ActiveOccupancyGate-validated artifact whose saved-map
// identity and source point cloud exactly match the current Explore snapshot.
[[nodiscard]] SavedCoverageGridResult
buildSavedCoverageGrid(const plan::far::FarGridMap &artifact, const std::string &source_map_sha256,
                       const lingtu::explore::ExploreMapIdentity &snapshot_identity);

}  // namespace lingtu::nav::endpoint
