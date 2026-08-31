#pragma once

#include <string>
#include <vector>

#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/seed.hpp"

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
  std::string reason{"spline_not_built"};
  std::vector<Vec3> path;
  std::vector<TrajectorySample> trajectory;
  bool fallback{false};
  SeedMode seedMode{SeedMode::Polynomial};
  std::vector<Vec3> controls;
  double interval{0.0};
  bool optimizerUsed{false};
  bool zGradientSuppressed{false};
  bool timeReallocated{false};
  bool refineUsed{false};
  bool dynamicFeasible{false};
  int reboundRestarts{0};
  int collisionSegments{0};
  int anchorSearches{0};
  int optimizerStatus{0};
  int optimizerEvaluations{0};
  double optimizerInitialCost{0.0};
  double optimizerFinalCost{0.0};
  int refineStatus{0};
  int refineEvaluations{0};
  double refineInitialCost{0.0};
  double refineFinalCost{0.0};

  [[nodiscard]] bool valid() const noexcept { return path.size() >= 2 && trajectory.size() >= 2; }
};

SplineResult buildSpline(const Grid &grid, const std::vector<Vec3> &path,
                         const LocalPlanRequest &input, const LocalPlannerParams &params,
                         const LocalPlanCancel &cancel = {},
                         const SeedHistory &history = {}, int failures = 0);

}  // namespace nav_kernel::local::scan
