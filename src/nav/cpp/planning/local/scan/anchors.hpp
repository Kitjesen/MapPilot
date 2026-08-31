// Adapted from SCAN-Planner BsplineOptimizer::initControlPoints at upstream
// commit 348e8a5. Modified for LingTu's ROS/Eigen-free grid and Vec3 types.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "planning/local/scan/grid.hpp"

namespace nav_kernel::local::scan {

struct Anchor {
  Vec3 base{};
  Vec3 direction{};
  double clearance{0.0};
};

struct AnchorSet {
  std::vector<std::vector<Anchor>> controls;
  std::vector<std::vector<Vec3>> paths;
  int collisionSegments{0};
  int searches{0};
  bool complete{true};
  std::string failureReason;
};

AnchorSet buildAnchors(const Grid &grid, const std::vector<Vec3> &controls,
                       const LocalPlannerParams &params,
                       const LocalPlanCancel &cancel = {});

}  // namespace nav_kernel::local::scan
