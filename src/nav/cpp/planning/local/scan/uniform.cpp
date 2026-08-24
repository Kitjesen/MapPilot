// Adapted from SCAN-Planner UniformBspline at upstream commit 348e8a5.
// Modified for LingTu: ROS/Eigen-free Vec3 storage and C++17 interfaces.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/uniform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace nav_kernel::local::scan {
namespace {

Vec3 add(const Vec3 &left, const Vec3 &right) {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 subtract(const Vec3 &left, const Vec3 &right) {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 scale(const Vec3 &value, double factor) {
  return {factor * value.x, factor * value.y, factor * value.z};
}

double component(const Vec3 &value, int axis) {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

bool finite(const Vec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool solveLeastSquares(std::vector<double> matrix, std::vector<Vec3> right_hand_side, int rows,
                       int columns, std::vector<Vec3> *solution) {
  if (solution == nullptr || rows < columns || columns <= 0 ||
      matrix.size() != static_cast<std::size_t>(rows * columns) ||
      right_hand_side.size() != static_cast<std::size_t>(rows)) {
    return false;
  }

  for (int column = 0; column < columns; ++column) {
    double column_norm = 0.0;
    for (int row = column; row < rows; ++row) {
      const double value = matrix[static_cast<std::size_t>(row * columns + column)];
      column_norm = std::hypot(column_norm, value);
    }
    if (column_norm <= 1e-12)
      return false;

    const double diagonal = matrix[static_cast<std::size_t>(column * columns + column)];
    const double reflected_diagonal = diagonal >= 0.0 ? -column_norm : column_norm;
    std::vector<double> reflector(static_cast<std::size_t>(rows - column), 0.0);
    for (int row = column; row < rows; ++row) {
      reflector[static_cast<std::size_t>(row - column)] =
          matrix[static_cast<std::size_t>(row * columns + column)];
    }
    reflector.front() -= reflected_diagonal;
    double reflector_norm_squared = 0.0;
    for (double value : reflector)
      reflector_norm_squared += value * value;
    if (reflector_norm_squared <= 1e-20)
      return false;
    const double beta = 2.0 / reflector_norm_squared;

    for (int target_column = column; target_column < columns; ++target_column) {
      double projection = 0.0;
      for (int row = column; row < rows; ++row) {
        projection += reflector[static_cast<std::size_t>(row - column)] *
                      matrix[static_cast<std::size_t>(row * columns + target_column)];
      }
      projection *= beta;
      for (int row = column; row < rows; ++row) {
        matrix[static_cast<std::size_t>(row * columns + target_column)] -=
            projection * reflector[static_cast<std::size_t>(row - column)];
      }
    }

    Vec3 rhs_projection{};
    for (int row = column; row < rows; ++row) {
      rhs_projection =
          add(rhs_projection, scale(right_hand_side[static_cast<std::size_t>(row)],
                                    reflector[static_cast<std::size_t>(row - column)]));
    }
    rhs_projection = scale(rhs_projection, beta);
    for (int row = column; row < rows; ++row) {
      right_hand_side[static_cast<std::size_t>(row)] =
          subtract(right_hand_side[static_cast<std::size_t>(row)],
                   scale(rhs_projection, reflector[static_cast<std::size_t>(row - column)]));
    }
  }

  solution->assign(static_cast<std::size_t>(columns), Vec3{});
  for (int row = columns - 1; row >= 0; --row) {
    Vec3 value = right_hand_side[static_cast<std::size_t>(row)];
    for (int column = row + 1; column < columns; ++column) {
      value = subtract(value, scale((*solution)[static_cast<std::size_t>(column)],
                                    matrix[static_cast<std::size_t>(row * columns + column)]));
    }
    const double diagonal = matrix[static_cast<std::size_t>(row * columns + row)];
    if (std::abs(diagonal) <= 1e-12)
      return false;
    (*solution)[static_cast<std::size_t>(row)] = scale(value, 1.0 / diagonal);
  }
  return std::all_of(solution->begin(), solution->end(), finite);
}

}  // namespace

UniformSpline::UniformSpline(std::vector<Vec3> controls, int degree, double interval)
    : controls_(std::move(controls)), degree_(degree), interval_(interval) {
  rebuildKnots();
}

bool UniformSpline::valid() const noexcept {
  return degree_ >= 0 && interval_ > 0.0 && std::isfinite(interval_) &&
         controls_.size() >= static_cast<std::size_t>(degree_ + 1) &&
         knots_.size() == controls_.size() + static_cast<std::size_t>(degree_) + 1U &&
         std::all_of(controls_.begin(), controls_.end(), finite);
}

int UniformSpline::degree() const noexcept {
  return degree_;
}

double UniformSpline::interval() const noexcept {
  return interval_;
}

double UniformSpline::duration() const noexcept {
  if (!valid())
    return 0.0;
  const int last_knot = static_cast<int>(knots_.size()) - 1;
  return knots_[static_cast<std::size_t>(last_knot - degree_)] -
         knots_[static_cast<std::size_t>(degree_)];
}

const std::vector<Vec3> &UniformSpline::controls() const noexcept {
  return controls_;
}

Vec3 UniformSpline::evaluate(double time_from_start_s) const {
  if (!valid() || !std::isfinite(time_from_start_s))
    return {};
  const double time = std::clamp(time_from_start_s, 0.0, duration());
  return evaluateKnot(time + knots_[static_cast<std::size_t>(degree_)]);
}

UniformSpline UniformSpline::derivative() const {
  if (!valid() || degree_ == 0 || controls_.size() < 2U)
    return {};
  std::vector<Vec3> derivative_controls;
  derivative_controls.reserve(controls_.size() - 1U);
  for (std::size_t index = 0; index + 1U < controls_.size(); ++index) {
    const double denominator =
        knots_[index + static_cast<std::size_t>(degree_) + 1U] - knots_[index + 1U];
    derivative_controls.push_back(scale(subtract(controls_[index + 1U], controls_[index]),
                                        static_cast<double>(degree_) / denominator));
  }
  return UniformSpline(std::move(derivative_controls), degree_ - 1, interval_);
}

double UniformSpline::feasibilityRatio(double max_speed, double max_acceleration,
                                       double tolerance) const {
  if (!valid() || max_speed <= 0.0 || max_acceleration <= 0.0 || !std::isfinite(max_speed) ||
      !std::isfinite(max_acceleration)) {
    return std::numeric_limits<double>::infinity();
  }
  const double allowed_speed = max_speed * (1.0 + std::max(0.0, tolerance)) + 1e-4;
  const double allowed_acceleration = max_acceleration * (1.0 + std::max(0.0, tolerance)) + 1e-4;
  double observed_speed = 0.0;
  double observed_acceleration = 0.0;
  const UniformSpline velocity = derivative();
  const UniformSpline acceleration = velocity.derivative();
  for (const Vec3 &value : velocity.controls()) {
    for (int axis = 0; axis < 3; ++axis) {
      observed_speed = std::max(observed_speed, std::abs(component(value, axis)));
    }
  }
  for (const Vec3 &value : acceleration.controls()) {
    for (int axis = 0; axis < 3; ++axis) {
      observed_acceleration = std::max(observed_acceleration, std::abs(component(value, axis)));
    }
  }
  const bool feasible =
      observed_speed <= allowed_speed && observed_acceleration <= allowed_acceleration;
  if (feasible)
    return 1.0;
  return std::max(observed_speed / max_speed, std::sqrt(observed_acceleration / max_acceleration));
}

std::vector<Vec3> UniformSpline::parameterize(double interval, const std::vector<Vec3> &samples,
                                              const std::array<Vec3, 4> &boundary_derivatives) {
  if (!(interval > 0.0) || !std::isfinite(interval) || samples.size() <= 3U ||
      !std::all_of(samples.begin(), samples.end(), finite) ||
      !std::all_of(boundary_derivatives.begin(), boundary_derivatives.end(), finite)) {
    return {};
  }

  const int sample_count = static_cast<int>(samples.size());
  const int rows = sample_count + 4;
  const int columns = sample_count + 2;
  std::vector<double> matrix(static_cast<std::size_t>(rows * columns), 0.0);
  const auto set = [&](int row, int column, double value) {
    matrix[static_cast<std::size_t>(row * columns + column)] = value;
  };
  for (int row = 0; row < sample_count; ++row) {
    set(row, row, 1.0 / 6.0);
    set(row, row + 1, 4.0 / 6.0);
    set(row, row + 2, 1.0 / 6.0);
  }
  const double velocity_factor = 1.0 / (2.0 * interval);
  set(sample_count, 0, -velocity_factor);
  set(sample_count, 2, velocity_factor);
  set(sample_count + 1, sample_count - 1, -velocity_factor);
  set(sample_count + 1, sample_count + 1, velocity_factor);
  const double acceleration_factor = 1.0 / (interval * interval);
  set(sample_count + 2, 0, acceleration_factor);
  set(sample_count + 2, 1, -2.0 * acceleration_factor);
  set(sample_count + 2, 2, acceleration_factor);
  set(sample_count + 3, sample_count - 1, acceleration_factor);
  set(sample_count + 3, sample_count, -2.0 * acceleration_factor);
  set(sample_count + 3, sample_count + 1, acceleration_factor);

  std::vector<Vec3> right_hand_side(static_cast<std::size_t>(rows));
  std::copy(samples.begin(), samples.end(), right_hand_side.begin());
  for (int index = 0; index < 4; ++index) {
    right_hand_side[static_cast<std::size_t>(sample_count + index)] =
        boundary_derivatives[static_cast<std::size_t>(index)];
  }
  std::vector<Vec3> controls;
  if (!solveLeastSquares(std::move(matrix), std::move(right_hand_side), rows, columns, &controls)) {
    return {};
  }
  return controls;
}

void UniformSpline::rebuildKnots() {
  knots_.clear();
  if (degree_ < 0 || !(interval_ > 0.0) || !std::isfinite(interval_) ||
      controls_.size() < static_cast<std::size_t>(degree_ + 1)) {
    return;
  }
  knots_.resize(controls_.size() + static_cast<std::size_t>(degree_) + 1U);
  for (std::size_t index = 0; index < knots_.size(); ++index) {
    knots_[index] = (static_cast<double>(index) - static_cast<double>(degree_)) * interval_;
  }
}

Vec3 UniformSpline::evaluateKnot(double knot) const {
  const int last_knot = static_cast<int>(knots_.size()) - 1;
  const double bounded = std::clamp(knot, knots_[static_cast<std::size_t>(degree_)],
                                    knots_[static_cast<std::size_t>(last_knot - degree_)]);
  int span = degree_;
  while (span + 1 < static_cast<int>(knots_.size()) &&
         knots_[static_cast<std::size_t>(span + 1)] < bounded) {
    ++span;
  }
  span = std::min(span, static_cast<int>(controls_.size()) - 1);

  std::vector<Vec3> work(static_cast<std::size_t>(degree_ + 1));
  for (int index = 0; index <= degree_; ++index) {
    work[static_cast<std::size_t>(index)] =
        controls_[static_cast<std::size_t>(span - degree_ + index)];
  }
  for (int level = 1; level <= degree_; ++level) {
    for (int index = degree_; index >= level; --index) {
      const int knot_index = index + span - degree_;
      const double denominator = knots_[static_cast<std::size_t>(index + 1 + span - level)] -
                                 knots_[static_cast<std::size_t>(knot_index)];
      const double alpha =
          denominator > 0.0 ? (bounded - knots_[static_cast<std::size_t>(knot_index)]) / denominator
                            : 0.0;
      work[static_cast<std::size_t>(index)] =
          add(scale(work[static_cast<std::size_t>(index - 1)], 1.0 - alpha),
              scale(work[static_cast<std::size_t>(index)], alpha));
    }
  }
  return work.back();
}

}  // namespace nav_kernel::local::scan
