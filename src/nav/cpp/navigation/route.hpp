#pragma once

#include <optional>
#include <vector>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::navigation {

// Global-planner output activated for local execution. The geometric route and
// terminal tolerances travel together so Executor receives one coherent value.
struct Route {
  std::vector<nav_kernel::Vec3> points;
  std::optional<double> finalYaw;
  std::optional<double> goalToleranceM;
  std::optional<double> yawToleranceRad;

  [[nodiscard]] bool empty() const noexcept { return points.empty(); }
};

}  // namespace lingtu::nav::navigation
