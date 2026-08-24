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
                    const LocalPlannerParams &params);

}  // namespace nav_kernel::local::scan
