#pragma once

#include <string>
#include <vector>

#include "planning/local/scan/anchors.hpp"
#include "planning/local/scan/grid.hpp"

namespace nav_kernel::local::scan {

struct OptimizationResult {
  std::vector<Vec3> controls;
  bool attempted{false};
  bool improved{false};
  bool anchorsComplete{true};
  bool reboundRequested{false};
  int status{0};
  int evaluations{0};
  int reboundPasses{0};
  int collisionSegments{0};
  int anchorSearches{0};
  double initialCost{0.0};
  double finalCost{0.0};
  std::string failureReason;
};

using ReboundOptimizationResult = OptimizationResult;
using RefineOptimizationResult = OptimizationResult;

// One B-spline solve keeps collision anchors across the official post-check
// restarts. The state is local to buildSpline; it never crosses planning calls.
struct ReboundState {
  AnchorSet anchors;
};

ReboundOptimizationResult optimizeRebound(const Grid &grid,
                                          const std::vector<Vec3> &seed_controls,
                                          double interval,
                                          const LocalPlannerParams &params,
                                          ReboundState *state = nullptr,
                                          const LocalPlanCancel &cancel = {});

RefineOptimizationResult optimizeRefine(const std::vector<Vec3> &seed_controls,
                                        const std::vector<Vec3> &reference_points,
                                        double interval,
                                        const LocalPlannerParams &params,
                                        const LocalPlanCancel &cancel = {});

}  // namespace nav_kernel::local::scan
