#pragma once

#include <string>
#include <vector>

#include "planning/local/scan/grid.hpp"

namespace nav_kernel::local::scan {

struct SearchResult {
  std::vector<Vec3> path;
  int expandedNodes{0};
  bool boundaryFallback{false};
  Vec3 fallbackTarget{};
  std::string reason{"no_path"};

  [[nodiscard]] bool found() const noexcept { return path.size() >= 2; }
};

SearchResult search(const Grid &grid, const Vec3 &start, double startYaw,
                    const LocalPlannerParams &params, const LocalPlanCancel &cancel = {});

// Projected A* between the free entry and exit controls of one collision
// segment. This is the search used by SCAN's rebound-anchor construction.
SearchResult searchSegment(const Grid &grid, const Vec3 &start, const Vec3 &goal,
                           double startYaw, const LocalPlannerParams &params,
                           const LocalPlanCancel &cancel = {});

}  // namespace nav_kernel::local::scan
