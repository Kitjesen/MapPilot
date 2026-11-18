#pragma once

#include <optional>
#include <string>

#include "lingtu/maps/build/grids.hpp"
#include "planning/global/contract.hpp"

#include "explore_contract.hpp"

namespace lingtu::nav::endpoint {

struct SavedCoverageGridResult {
  std::optional<lingtu::explore::Grid2D> grid;
  std::string reason;

  [[nodiscard]] bool ok() const noexcept { return grid.has_value(); }
};

// Convert only an ActiveOccupancyGate-validated artifact whose saved-map
// identity matches the current Explore snapshot.
[[nodiscard]] SavedCoverageGridResult
buildSavedCoverageGrid(const lingtu::maps::OccupancyArtifactData &artifact,
                       const plan::MapIdentity &artifact_identity,
                       const lingtu::explore::ExploreMapIdentity &snapshot_identity);

}  // namespace lingtu::nav::endpoint
