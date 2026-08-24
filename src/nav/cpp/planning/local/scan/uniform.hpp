// Adapted from SCAN-Planner UniformBspline at upstream commit 348e8a5.
// Modified for LingTu: ROS/Eigen-free Vec3 storage and C++17 interfaces.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <vector>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

class UniformSpline {
 public:
  UniformSpline() = default;
  UniformSpline(std::vector<Vec3> controls, int degree, double interval);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] int degree() const noexcept;
  [[nodiscard]] double interval() const noexcept;
  [[nodiscard]] double duration() const noexcept;
  [[nodiscard]] const std::vector<Vec3> &controls() const noexcept;

  [[nodiscard]] Vec3 evaluate(double time_from_start_s) const;
  [[nodiscard]] UniformSpline derivative() const;

  // Returns the uniform time scale needed to satisfy component-wise limits.
  [[nodiscard]] double feasibilityRatio(double max_speed, double max_acceleration,
                                        double tolerance = 0.0) const;

  // Cubic B-spline interpolation with start/end velocity and acceleration.
  // Boundary order: start velocity, end velocity, start acceleration, end acceleration.
  static std::vector<Vec3> parameterize(double interval, const std::vector<Vec3> &samples,
                                        const std::array<Vec3, 4> &boundary_derivatives);

 private:
  void rebuildKnots();
  [[nodiscard]] Vec3 evaluateKnot(double knot) const;

  std::vector<Vec3> controls_;
  std::vector<double> knots_;
  int degree_{0};
  double interval_{0.0};
};

}  // namespace nav_kernel::local::scan
