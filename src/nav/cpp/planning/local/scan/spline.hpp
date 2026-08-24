#pragma once

#include <vector>

#include "planning/local/scan/grid.hpp"

namespace nav_kernel::local::scan {

struct TrajectorySample {
  Vec3 position{};
  Vec3 velocity{};
  Vec3 acceleration{};
  double yaw{0.0};
  double yawRate{0.0};
  double timeFromStartS{0.0};
};

struct SplineResult {
  std::vector<Vec3> path;
  std::vector<TrajectorySample> trajectory;
  bool fallback{false};
  bool optimizerUsed{false};
  bool zGradientSuppressed{false};
  int optimizerStatus{0};
  int optimizerEvaluations{0};
  double optimizerInitialCost{0.0};
  double optimizerFinalCost{0.0};

  [[nodiscard]] bool valid() const noexcept { return path.size() >= 2 && trajectory.size() >= 2; }
};

SplineResult buildSpline(const Grid &grid, const std::vector<Vec3> &path,
                         const LocalPlanInput &input, const LocalPlannerParams &params);

}  // namespace nav_kernel::local::scan
