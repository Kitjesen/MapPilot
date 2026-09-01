#pragma once

// Ported from SCAN-Planner bspline_opt/uniform_bspline.h at commit 348e8a5.
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Eigen>
#include <vector>

namespace nav_kernel::local::scan::upstream {

class UniformBspline {
 public:
  UniformBspline() = default;
  UniformBspline(const Eigen::MatrixXd &points, int order, double interval);

  void setUniformBspline(const Eigen::MatrixXd &points, int order,
                         double interval);
  void setKnot(const Eigen::VectorXd &knot);
  [[nodiscard]] Eigen::VectorXd getKnot() const;
  [[nodiscard]] Eigen::MatrixXd getControlPoint() const;
  [[nodiscard]] double getInterval() const;
  [[nodiscard]] bool getTimeSpan(double &start, double &end) const;
  [[nodiscard]] Eigen::VectorXd evaluateDeBoor(double u) const;
  [[nodiscard]] Eigen::VectorXd evaluateDeBoorT(double time) const;
  [[nodiscard]] UniformBspline getDerivative() const;

  static void parameterizeToBspline(
      double interval, const std::vector<Eigen::Vector3d> &points,
      const std::vector<Eigen::Vector3d> &startEndDerivative,
      Eigen::MatrixXd &controls);

  void setPhysicalLimits(double velocity, double acceleration,
                         double tolerance);
  [[nodiscard]] bool checkFeasibility(double &ratio, bool show = false) const;
  void lengthenTime(double ratio);
  [[nodiscard]] double getTimeSum() const;
  [[nodiscard]] double getLength(double resolution = 0.01) const;
  [[nodiscard]] double getJerk() const;
  void getMeanAndMaxVel(double &meanVelocity, double &maxVelocity) const;
  void getMeanAndMaxAcc(double &meanAcceleration,
                        double &maxAcceleration) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  [[nodiscard]] Eigen::MatrixXd getDerivativeControlPoints() const;

  Eigen::MatrixXd controlPoints_{};
  int degree_{0};
  int lastControl_{0};
  int lastKnot_{0};
  Eigen::VectorXd knots_{};
  double interval_{0.0};
  double velocityLimit_{0.0};
  double accelerationLimit_{0.0};
  double feasibilityTolerance_{0.0};
};

}  // namespace nav_kernel::local::scan::upstream
