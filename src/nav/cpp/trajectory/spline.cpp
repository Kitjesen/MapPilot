#include "trajectory/spline.hpp"

#include <algorithm>
#include <cmath>

namespace nav_kernel {
namespace {

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

Vec3 point(const Eigen::VectorXd &value) {
  return value.size() >= 3 ? Vec3{value(0), value(1), value(2)} : Vec3{};
}

}  // namespace

SplineView::SplineView(const SplineTarget &target) {
  if (target.order < 1 || target.order > 3 ||
      target.controls.size() < static_cast<std::size_t>(target.order + 1) ||
      target.knots.size() !=
          target.controls.size() + static_cast<std::size_t>(target.order) + 1U ||
      !std::all_of(target.controls.begin(), target.controls.end(), finitePoint) ||
      !std::all_of(target.knots.begin(), target.knots.end(),
                   [](double value) { return std::isfinite(value); })) {
    return;
  }

  interval_ = target.knots[1] - target.knots[0];
  if (!(interval_ > 0.0))
    return;

  Eigen::MatrixXd controls(3,
                           static_cast<Eigen::Index>(target.controls.size()));
  for (std::size_t index = 0; index < target.controls.size(); ++index) {
    controls(0, static_cast<Eigen::Index>(index)) = target.controls[index].x;
    controls(1, static_cast<Eigen::Index>(index)) = target.controls[index].y;
    controls(2, static_cast<Eigen::Index>(index)) = target.controls[index].z;
  }
  Eigen::VectorXd knots(static_cast<Eigen::Index>(target.knots.size()));
  for (std::size_t index = 0; index < target.knots.size(); ++index)
    knots(static_cast<Eigen::Index>(index)) = target.knots[index];

  position_.setUniformBspline(controls, target.order, interval_);
  position_.setKnot(knots);
  duration_ = position_.getTimeSum();
  if (!(duration_ >= 0.0) || !std::isfinite(duration_))
    return;
  velocity_ = position_.getDerivative();
  valid_ = true;
}

bool SplineView::valid() const noexcept {
  return valid_;
}

double SplineView::duration() const noexcept {
  return duration_;
}

double SplineView::interval() const noexcept {
  return interval_;
}

Vec3 SplineView::position(double timeFromStartS) const {
  return valid_ ? point(position_.evaluateDeBoorT(timeFromStartS)) : Vec3{};
}

Vec3 SplineView::velocity(double timeFromStartS) const {
  return valid_ ? point(velocity_.evaluateDeBoorT(timeFromStartS)) : Vec3{};
}

}  // namespace nav_kernel
