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
      !std::isfinite(nowS)) {
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
  accelerationTrajectory_ = velocityTrajectory_.getDerivative();
  durationS_ = duration;
  trajectoryId_ = trajectory.trajectoryId;
  executionTimeS_ = 0.0;
  lastUpdateTimeS_ = nowS;
  receiveTrajectory_ = true;
  return true;
}

ClosedLoopControllerOutput ClosedLoopController::step(
    const ClosedLoopControllerState &state,
    const ClosedLoopControllerParams &params) {
  ClosedLoopControllerOutput output;
  if (!receiveTrajectory_ || !state.position.allFinite() ||
      !std::isfinite(state.yaw) || !std::isfinite(state.nowS)) {
    return output;
  }

  double dt = state.nowS - lastUpdateTimeS_;
  if (dt < 0.0 || dt > 0.2) dt = 0.0;

  const double evaluationTime = std::min(executionTimeS_, durationS_);
  Eigen::Vector3d desiredPosition =
      positionTrajectory_.evaluateDeBoorT(evaluationTime);
  Eigen::Vector3d desiredVelocity =
      velocityTrajectory_.evaluateDeBoorT(evaluationTime);

  const double desiredHeading = desiredYaw(
      evaluationTime, desiredPosition, state.yaw,
      std::max(0.0, params.timeForward));
  const double yawError = normalizeAngle(desiredHeading - state.yaw);
  const double yawLimit =
      std::min(kMaxYawRateLimit, std::max(0.0, params.maxYawRate));
  const double yawCommand = clamp(
      std::max(0.0, params.yawGain) * yawError, -yawLimit, yawLimit);

  output.yawError = yawError;
  output.executionTimeS = executionTimeS_;
  const Eigen::Vector3d endpoint =
      positionTrajectory_.evaluateDeBoorT(durationS_);
  output.endDistance = (endpoint.head<2>() - state.position.head<2>()).norm();

  if (std::abs(yawError) >
      std::max(0.0, params.headingErrorThreshold)) {
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
  const Eigen::Vector2d feedForward = desiredVelocity.head<2>();
  const Eigen::Vector2d worldVelocity = clampNorm(
      feedForward + std::max(0.0, params.positionGain) * positionError,
      std::max(std::max(0.0, params.maxVx),
               std::max(0.0, params.maxVy)));

  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  output.command.vx =
      clamp(c * worldVelocity.x() + s * worldVelocity.y(),
            -std::max(0.0, params.maxVx), std::max(0.0, params.maxVx));
  output.command.vy =
      clamp(-s * worldVelocity.x() + c * worldVelocity.y(),
            -std::max(0.0, params.maxVy), std::max(0.0, params.maxVy));
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
    double time, const Eigen::Vector3d &desiredPosition, double currentYaw,
    double timeForward) const {
  const double lookaheadTime = std::min(durationS_, time + timeForward);
  Eigen::Vector3d direction =
      positionTrajectory_.evaluateDeBoorT(lookaheadTime) - desiredPosition;
  if (direction.head<2>().squaredNorm() < 1e-4) {
    direction = velocityTrajectory_.evaluateDeBoorT(time);
  }
  if (direction.head<2>().squaredNorm() < 1e-4) return currentYaw;
  return std::atan2(direction.y(), direction.x());
}

}  // namespace nav_kernel::local::scan::upstream
