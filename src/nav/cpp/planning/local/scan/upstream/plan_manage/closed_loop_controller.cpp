#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"

#include <algorithm>
#include <cmath>

namespace nav_kernel::local::scan::upstream {
namespace {

constexpr double kMaxYawRateLimit = 1.0;

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double maximumNorm) {
  const double norm = value.norm();
  if (norm <= maximumNorm || norm < 1e-6) return value;
  return value / norm * maximumNorm;
}

}  // namespace

bool ClosedLoopController::setTrajectory(const BsplineTrajectory &trajectory,
                                         double nowS) {
  if (trajectory.order < 1 || trajectory.positionPoints.rows() != 3 ||
      trajectory.positionPoints.cols() <= trajectory.order ||
      trajectory.knots.size() !=
          trajectory.positionPoints.cols() + trajectory.order + 1 ||
      !trajectory.positionPoints.allFinite() || !trajectory.knots.allFinite() ||
      !std::isfinite(nowS) || !std::isfinite(trajectory.startTimeS)) {
    reset();
    return false;
  }

  UniformBspline position(trajectory.positionPoints, trajectory.order, 0.1);
  position.setKnot(trajectory.knots);
  const double duration = position.getTimeSum();
  if (!std::isfinite(duration) || duration <= 0.0) {
    reset();
    return false;
  }

  positionTrajectory_ = std::move(position);
  velocityTrajectory_ = positionTrajectory_.getDerivative();
  hasAcceleration_ = trajectory.order >= 2;
  if (hasAcceleration_)
    accelerationTrajectory_ = velocityTrajectory_.getDerivative();
  durationS_ = duration;
  trajectoryId_ = trajectory.trajectoryId;
  executionTimeS_ = clamp(nowS - trajectory.startTimeS, 0.0, durationS_);
  lastUpdateTimeS_ = nowS;
  receiveTrajectory_ = true;
  return true;
}

ClosedLoopControllerOutput ClosedLoopController::step(
    const ClosedLoopControllerState &state,
    const ClosedLoopControllerParams &params) {
  ClosedLoopControllerOutput output;
  if (!receiveTrajectory_ || !state.position.allFinite() ||
      !std::isfinite(state.yaw) || !std::isfinite(state.nowS) ||
      (state.desiredHeading && !std::isfinite(*state.desiredHeading)) ||
      (state.desiredYawRate && !std::isfinite(*state.desiredYawRate))) {
    return output;
  }

  const double dt = std::max(0.0, state.nowS - lastUpdateTimeS_);

  const double evaluationTime = std::min(executionTimeS_, durationS_);
  Eigen::Vector3d desiredPosition =
      positionTrajectory_.evaluateDeBoorT(evaluationTime);
  Eigen::Vector3d desiredVelocity =
      velocityTrajectory_.evaluateDeBoorT(evaluationTime);

  const double desiredHeading = state.desiredHeading
      ? *state.desiredHeading
      : desiredYaw(evaluationTime, state.yaw,
                   std::max(0.0, params.timeForward));
  const double yawError = state.desiredYawRate ? 0.0 : normalizeAngle(desiredHeading - state.yaw);
  const double yawLimit =
      std::min(kMaxYawRateLimit, std::max(0.0, params.maxYawRate));
  // Keep a small numerical band around the official threshold to avoid
  // freezing a trajectory on a boundary fluctuation.
  const bool aligning = !state.desiredYawRate &&
      std::abs(yawError) > std::max(0.0, params.headingErrorThreshold) + 0.002;
  double yawFeedforward=0;
  if(hasAcceleration_ && params.timeForward<=0 && !state.desiredHeading && !state.desiredYawRate &&
     !aligning && desiredVelocity.head<2>().norm()>0.03) {
    const Eigen::Vector3d acceleration=accelerationTrajectory_.evaluateDeBoorT(evaluationTime);
    yawFeedforward=(desiredVelocity.x()*acceleration.y()-desiredVelocity.y()*acceleration.x())/
        desiredVelocity.head<2>().squaredNorm();
  }
  const double yawCommand = clamp(
      state.desiredYawRate.value_or(std::max(0.0, params.yawGain) * yawError+yawFeedforward),
      -yawLimit, yawLimit);

  output.yawError = yawError;
  output.durationS = durationS_;
  output.positionErrorM =
      (desiredPosition.head<2>() - state.position.head<2>()).norm();
  output.executionTimeS = executionTimeS_;
  const Eigen::Vector3d endpoint =
      positionTrajectory_.evaluateDeBoorT(durationS_);
  output.endDistance = (endpoint.head<2>() - state.position.head<2>()).norm();

  if (aligning) {
    output.command.wz = yawCommand;
    output.executionFrozen = true;
    lastUpdateTimeS_ = state.nowS;
    return output;
  }

  executionTimeS_ = std::min(durationS_, executionTimeS_ + dt);
  lastUpdateTimeS_ = state.nowS;

  desiredPosition = positionTrajectory_.evaluateDeBoorT(executionTimeS_);
  desiredVelocity = velocityTrajectory_.evaluateDeBoorT(executionTimeS_);

  const Eigen::Vector2d positionError =
      desiredPosition.head<2>() - state.position.head<2>();
  output.positionErrorM = positionError.norm();
  const Eigen::Vector2d feedForward = desiredVelocity.head<2>();
  const Eigen::Vector2d worldVelocity = clampNorm(
      feedForward + std::max(0.0, params.positionGain) * positionError,
      std::max(0.0, params.maxLinearSpeed));

  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double bodyVx = c * worldVelocity.x() + s * worldVelocity.y();
  const double bodyVy = -s * worldVelocity.x() + c * worldVelocity.y();
  const double maxVx = std::max(0.0, params.maxVx);
  const double maxVy = std::max(0.0, params.maxVy);
  double axisScale = 1.0;
  if (std::abs(bodyVx) > maxVx) axisScale = std::min(axisScale, maxVx / std::abs(bodyVx));
  if (std::abs(bodyVy) > maxVy) axisScale = std::min(axisScale, maxVy / std::abs(bodyVy));
  // Keep the feedback-corrected travel direction when either body-axis limit
  // is active. Independent clipping bends the command away from its spline.
  output.command.vx = bodyVx * axisScale;
  output.command.vy = bodyVy * axisScale;
  output.command.wz = yawCommand;
  output.executionTimeS = executionTimeS_;

  if (executionTimeS_ >= durationS_ &&
      positionError.norm() < std::max(0.0, params.finishDistance)) {
    output.command = {};
    output.finished = true;
  }
  return output;
}

void ClosedLoopController::reset() noexcept {
  receiveTrajectory_ = false;
  trajectoryId_ = 0;
  positionTrajectory_ = {};
  velocityTrajectory_ = {};
  accelerationTrajectory_ = {};
  hasAcceleration_ = false;
  durationS_ = 0.0;
  executionTimeS_ = 0.0;
  lastUpdateTimeS_ = 0.0;
}

bool ClosedLoopController::hasTrajectory() const noexcept {
  return receiveTrajectory_;
}

std::int64_t ClosedLoopController::trajectoryId() const noexcept {
  return trajectoryId_;
}

double ClosedLoopController::desiredYaw(
    double time, double currentYaw,
    double timeForward) const {
  const Eigen::Vector3d direction =
      positionTrajectory_.headingDirection(time, velocityTrajectory_, timeForward);
  if (direction.head<2>().squaredNorm() == 0.0) return currentYaw;
  return std::atan2(direction.y(), direction.x());
}

}  // namespace nav_kernel::local::scan::upstream
