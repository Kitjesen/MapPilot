#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace nav_kernel::local::scan::upstream {

UniformBspline::UniformBspline(const Eigen::MatrixXd &points, int order,
                               double interval) {
  setUniformBspline(points, order, interval);
}

void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points,
                                       int order, double interval) {
  controlPoints_ = points;
  degree_ = order;
  interval_ = interval;
  lastControl_ = static_cast<int>(points.cols()) - 1;
  lastKnot_ = lastControl_ + degree_ + 1;
  knots_ = Eigen::VectorXd::Zero(lastKnot_ + 1);
  for (int index = 0; index <= lastKnot_; ++index) {
    if (index <= degree_) {
      knots_(index) = static_cast<double>(-degree_ + index) * interval_;
    } else {
      knots_(index) = knots_(index - 1) + interval_;
    }
  }
}

void UniformBspline::setKnot(const Eigen::VectorXd &knot) {
  knots_ = knot;
}

Eigen::VectorXd UniformBspline::getKnot() const {
  return knots_;
}

bool UniformBspline::getTimeSpan(double &start, double &end) const {
  if (degree_ >= knots_.rows() || lastKnot_ - degree_ >= knots_.rows()) {
    return false;
  }
  start = knots_(degree_);
  end = knots_(lastKnot_ - degree_);
  return true;
}

Eigen::MatrixXd UniformBspline::getControlPoint() const {
  return controlPoints_;
}

Eigen::VectorXd UniformBspline::evaluateDeBoor(double u) const {
  const double bounded =
      std::min(std::max(knots_(degree_), u), knots_(lastKnot_ - degree_));
  int knot = degree_;
  while (knots_(knot + 1) < bounded) ++knot;

  std::vector<Eigen::VectorXd> values;
  values.reserve(static_cast<std::size_t>(degree_ + 1));
  for (int index = 0; index <= degree_; ++index) {
    values.push_back(controlPoints_.col(knot - degree_ + index));
  }
  for (int level = 1; level <= degree_; ++level) {
    for (int index = degree_; index >= level; --index) {
      const double alpha =
          (bounded - knots_(index + knot - degree_)) /
          (knots_(index + 1 + knot - level) -
           knots_(index + knot - degree_));
      values[index] =
          (1.0 - alpha) * values[index - 1] + alpha * values[index];
    }
  }
  return values[degree_];
}

Eigen::VectorXd UniformBspline::evaluateDeBoorT(double time) const {
  return evaluateDeBoor(time + knots_(degree_));
}

Eigen::MatrixXd UniformBspline::getDerivativeControlPoints() const {
  Eigen::MatrixXd controls(controlPoints_.rows(), controlPoints_.cols() - 1);
  for (int index = 0; index < controls.cols(); ++index) {
    controls.col(index) =
        degree_ * (controlPoints_.col(index + 1) - controlPoints_.col(index)) /
        (knots_(index + degree_ + 1) - knots_(index + 1));
  }
  return controls;
}

UniformBspline UniformBspline::getDerivative() const {
  UniformBspline derivative(getDerivativeControlPoints(), degree_ - 1,
                            interval_);
  derivative.setKnot(knots_.segment(1, knots_.rows() - 2));
  return derivative;
}

double UniformBspline::getInterval() const {
  return interval_;
}

void UniformBspline::setPhysicalLimits(double velocity, double acceleration,
                                       double tolerance) {
  velocityLimit_ = velocity;
  accelerationLimit_ = acceleration;
  feasibilityTolerance_ = tolerance;
}

bool UniformBspline::checkFeasibility(double &ratio, bool show) const {
  bool feasible = true;
  const Eigen::MatrixXd points = controlPoints_;
  const int dimension = static_cast<int>(controlPoints_.rows());
  double maxVelocity = -1.0;
  const double velocityLimit =
      velocityLimit_ * (1.0 + feasibilityTolerance_) + 1e-4;
  for (int index = 0; index < points.cols() - 1; ++index) {
    const Eigen::VectorXd velocity =
        degree_ * (points.col(index + 1) - points.col(index)) /
        (knots_(index + degree_ + 1) - knots_(index + 1));
    if (std::abs(velocity(0)) > velocityLimit ||
        std::abs(velocity(1)) > velocityLimit ||
        std::abs(velocity(2)) > velocityLimit) {
      if (show) std::cout << "[Check]: Infeasible vel " << index << " :" << velocity.transpose() << '\n';
      feasible = false;
      for (int axis = 0; axis < dimension; ++axis) {
        maxVelocity = std::max(maxVelocity, std::abs(velocity(axis)));
      }
    }
  }

  double maxAcceleration = -1.0;
  const double accelerationLimit =
      accelerationLimit_ * (1.0 + feasibilityTolerance_) + 1e-4;
  for (int index = 0; index < points.cols() - 2; ++index) {
    const Eigen::VectorXd acceleration =
        degree_ * (degree_ - 1) *
        ((points.col(index + 2) - points.col(index + 1)) /
             (knots_(index + degree_ + 2) - knots_(index + 2)) -
         (points.col(index + 1) - points.col(index)) /
             (knots_(index + degree_ + 1) - knots_(index + 1))) /
        (knots_(index + degree_ + 1) - knots_(index + 2));
    if (std::abs(acceleration(0)) > accelerationLimit ||
        std::abs(acceleration(1)) > accelerationLimit ||
        std::abs(acceleration(2)) > accelerationLimit) {
      if (show) std::cout << "[Check]: Infeasible acc " << index << " :" << acceleration.transpose() << '\n';
      feasible = false;
      for (int axis = 0; axis < dimension; ++axis) {
        maxAcceleration =
            std::max(maxAcceleration, std::abs(acceleration(axis)));
      }
    }
  }

  ratio = std::max(maxVelocity / velocityLimit_,
                   std::sqrt(std::abs(maxAcceleration) / accelerationLimit_));
  return feasible;
}

void UniformBspline::lengthenTime(double ratio) {
  const int first = 5;
  const int last = static_cast<int>(getKnot().rows()) - 1 - 5;
  const double delta = (ratio - 1.0) * (knots_(last) - knots_(first));
  const double increment = delta / static_cast<double>(last - first);
  for (int index = first + 1; index <= last; ++index) {
    knots_(index) += static_cast<double>(index - first) * increment;
  }
  for (int index = last + 1; index < knots_.rows(); ++index) {
    knots_(index) += delta;
  }
}

void UniformBspline::parameterizeToBspline(
    double interval, const std::vector<Eigen::Vector3d> &points,
    const std::vector<Eigen::Vector3d> &startEndDerivative,
    Eigen::MatrixXd &controls) {
  if (interval <= 0.0 || points.size() <= 3U ||
      startEndDerivative.size() != 4U) {
    controls.resize(0, 0);
    return;
  }

  const int count = static_cast<int>(points.size());
  Eigen::Vector3d positionRow(1.0, 4.0, 1.0);
  Eigen::Vector3d velocityRow(-1.0, 0.0, 1.0);
  Eigen::Vector3d accelerationRow(1.0, -2.0, 1.0);
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(count + 4, count + 2);
  for (int index = 0; index < count; ++index) {
    matrix.block(index, index, 1, 3) =
        (1.0 / 6.0) * positionRow.transpose();
  }
  matrix.block(count, 0, 1, 3) =
      (1.0 / 2.0 / interval) * velocityRow.transpose();
  matrix.block(count + 1, count - 1, 1, 3) =
      (1.0 / 2.0 / interval) * velocityRow.transpose();
  matrix.block(count + 2, 0, 1, 3) =
      (1.0 / interval / interval) * accelerationRow.transpose();
  matrix.block(count + 3, count - 1, 1, 3) =
      (1.0 / interval / interval) * accelerationRow.transpose();

  Eigen::VectorXd x(count + 4), y(count + 4), z(count + 4);
  for (int index = 0; index < count; ++index) {
    x(index) = points[index].x();
    y(index) = points[index].y();
    z(index) = points[index].z();
  }
  for (int index = 0; index < 4; ++index) {
    x(count + index) = startEndDerivative[index].x();
    y(count + index) = startEndDerivative[index].y();
    z(count + index) = startEndDerivative[index].z();
  }

  controls.resize(3, count + 2);
  controls.row(0) = matrix.colPivHouseholderQr().solve(x).transpose();
  controls.row(1) = matrix.colPivHouseholderQr().solve(y).transpose();
  controls.row(2) = matrix.colPivHouseholderQr().solve(z).transpose();
}

double UniformBspline::getTimeSum() const {
  double start = 0.0;
  double end = 0.0;
  return getTimeSpan(start, end) ? end - start : -1.0;
}

double UniformBspline::getLength(double resolution) const {
  double length = 0.0;
  const double duration = getTimeSum();
  Eigen::VectorXd previous = evaluateDeBoorT(0.0);
  for (double time = resolution; time <= duration + 1e-4;
       time += resolution) {
    const Eigen::VectorXd next = evaluateDeBoorT(time);
    length += (next - previous).norm();
    previous = next;
  }
  return length;
}

double UniformBspline::getJerk() const {
  UniformBspline jerk = getDerivative().getDerivative().getDerivative();
  const Eigen::VectorXd times = jerk.getKnot();
  const Eigen::MatrixXd controls = jerk.getControlPoint();
  double result = 0.0;
  for (int column = 0; column < controls.cols(); ++column) {
    for (int row = 0; row < controls.rows(); ++row) {
      result += (times(column + 1) - times(column)) * controls(row, column) *
                controls(row, column);
    }
  }
  return result;
}

void UniformBspline::getMeanAndMaxVel(double &meanVelocity,
                                      double &maxVelocity) const {
  const UniformBspline velocity = getDerivative();
  double start = 0.0;
  double end = 0.0;
  (void)velocity.getTimeSpan(start, end);
  maxVelocity = -1.0;
  meanVelocity = 0.0;
  int count = 0;
  for (double time = start; time <= end; time += 0.01) {
    const double norm = velocity.evaluateDeBoor(time).norm();
    meanVelocity += norm;
    ++count;
    maxVelocity = std::max(maxVelocity, norm);
  }
  meanVelocity /= static_cast<double>(count);
}

void UniformBspline::getMeanAndMaxAcc(double &meanAcceleration,
                                      double &maxAcceleration) const {
  const UniformBspline acceleration = getDerivative().getDerivative();
  double start = 0.0;
  double end = 0.0;
  (void)acceleration.getTimeSpan(start, end);
  maxAcceleration = -1.0;
  meanAcceleration = 0.0;
  int count = 0;
  for (double time = start; time <= end; time += 0.01) {
    const double norm = acceleration.evaluateDeBoor(time).norm();
    meanAcceleration += norm;
    ++count;
    maxAcceleration = std::max(maxAcceleration, norm);
  }
  meanAcceleration /= static_cast<double>(count);
}

}  // namespace nav_kernel::local::scan::upstream
