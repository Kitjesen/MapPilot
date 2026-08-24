#pragma once

#include <vector>

#include "planning/local/scan/grid.hpp"

namespace nav_kernel::local::scan {

struct ReboundOptimizationResult {
  std::vector<Vec3> controls;
  bool attempted{false};
  bool improved{false};
  int status{0};
  int evaluations{0};
  double initialCost{0.0};
  double finalCost{0.0};
};

ReboundOptimizationResult optimizeRebound(const Grid &grid, const std::vector<Vec3> &seed_controls,
                                          const std::vector<Vec3> &projected_guide, double interval,
                                          const LocalPlannerParams &params);

}  // namespace nav_kernel::local::scan
