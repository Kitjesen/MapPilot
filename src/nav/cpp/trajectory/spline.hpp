#pragma once

#include "nav_kernel/types.hpp"
#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"

namespace nav_kernel {

// Read-only view of the exact SCAN B-spline message carried by SplineTarget.
class SplineView {
 public:
  explicit SplineView(const SplineTarget &target);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double duration() const noexcept;
  [[nodiscard]] double interval() const noexcept;
  [[nodiscard]] Vec3 position(double timeFromStartS) const;
  [[nodiscard]] Vec3 velocity(double timeFromStartS) const;

 private:
  local::scan::upstream::UniformBspline position_{};
  local::scan::upstream::UniformBspline velocity_{};
  double duration_{0.0};
  double interval_{0.0};
  bool valid_{false};
};

}  // namespace nav_kernel
