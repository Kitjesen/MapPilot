#pragma once

#include <optional>
#include <string>

#include "explore_contract.hpp"

namespace lingtu::nav::plan::far_planner {
struct FarGridMap;
}

namespace lingtu::nav::endpoint {

struct SavedCoverageGridResult {
  std::optional<lingtu::explore::Grid2D> grid;
  std::string reason;

  [[nodiscard]] bool ok() const noexcept { return grid.has_value(); }
};

// Convert only an ActiveOccupancyGate-validated artifact whose saved-map
// identity matches the current Explore snapshot.
[[nodiscard]] SavedCoverageGridResult
buildSavedCoverageGrid(const plan::far_planner::FarGridMap &artifact,
                       const lingtu::explore::ExploreMapIdentity &snapshot_identity);

}  // namespace lingtu::nav::endpoint
