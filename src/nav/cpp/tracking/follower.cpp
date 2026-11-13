#include "tracking/follower.hpp"

#include "planning/local/planner.hpp"
#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"
#include "trajectory/spline.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <variant>

namespace nav_kernel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedEpsilon = 1e-4;
constexpr double kCornerCos = 0.7071067811865476;  // 45 degrees
constexpr double kCornerReachM = 0.10;

double wrapPi(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0)
    angle += 2.0 * kPi;
  return angle - kPi;
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

double adaptiveLookAhead(double currentSpeed, const FollowerParams &params) {
  const double lookahead =
      params.baseLookAheadDis + params.lookAheadRatio * std::fabs(currentSpeed);
  return clamp(lookahead, params.minLookAheadDis, params.maxLookAheadDis);
}

int cornerLimit(const std::vector<Vec3> &path, int begin, const Vec3 &vehicle) {
  constexpr double kSegmentEpsilon = 1e-4;
  const int path_size = static_cast<int>(path.size());
  for (int index = std::max(1, begin); index + 1 < path_size; ++index) {
    const double before_x = path[index].x - path[index - 1].x;
    const double before_y = path[index].y - path[index - 1].y;
    const double after_x = path[index + 1].x - path[index].x;
    const double after_y = path[index + 1].y - path[index].y;
    const double before_length = std::hypot(before_x, before_y);
    const double after_length = std::hypot(after_x, after_y);
    if (before_length <= kSegmentEpsilon || after_length <= kSegmentEpsilon)
      continue;
    const double turn_cos =
        (before_x * after_x + before_y * after_y) / (before_length * after_length);
    if (turn_cos >= kCornerCos)
      continue;
    if (std::hypot(path[index].x - vehicle.x, path[index].y - vehicle.y) > kCornerReachM)
      return index;
  }
  return path_size - 1;
}

bool updateHeadingAlignment(bool active, double heading_error,
                            const FollowerParams &params, bool allow_omni_translation) {
  if (allow_omni_translation) {
    return false;
  }
  const double enter = std::max(0.0, params.headingAlignEnterRad);
  if (enter <= 0.0) {
    return false;
  }
  const double exit = clamp(params.headingAlignExitRad, 0.0, enter);
  const double magnitude = std::abs(heading_error);
  if (std::abs(enter - exit) <= 1e-12) {
    return magnitude >= enter;
  }
  if (active) {
    return magnitude > exit;
  }
  return magnitude >= enter;
}

class PathAlgorithm {
 public:
  FollowerOutput follow(const std::vector<Vec3> &path, const FollowerState &input) {
    FollowerOutput output;
    const FollowerParams &params = input.params;

    if (!std::isfinite(input.vehicleRelative.x) || !std::isfinite(input.vehicleRelative.y) ||
        !std::isfinite(input.vehicleYawRelative)) {
      return output;
    }

    const int path_size = static_cast<int>(path.size());
    if (path_size == 0)
      return output;

    const double end_x = path[path_size - 1].x - input.vehicleRelative.x;
    const double end_y = path[path_size - 1].y - input.vehicleRelative.y;
    const double local_end_distance = std::hypot(end_x, end_y);
    const double end_distance = std::isfinite(input.goalDistance) && input.goalDistance >= 0.0
                                    ? input.goalDistance
                                    : local_end_distance;
    output.endDistance = end_distance;

    const double lookahead_distance = adaptiveLookAhead(state_.vehicleSpeed, params);
    if (path_size != state_.lastPathSize) {
      state_.lastPathPoint = 0;
      state_.lastPathSize = path_size;
    }
    int path_point = state_.lastPathPoint > 2 ? state_.lastPathPoint - 2 : 0;

    double dx = 0.0;
    double dy = 0.0;
    const double lookahead_squared = lookahead_distance * lookahead_distance;
    const int path_point_limit =
        params.cornerGate ? cornerLimit(path, path_point, input.vehicleRelative)
                          : path_size - 1;
    while (path_point < path_point_limit) {
      dx = path[path_point].x - input.vehicleRelative.x;
      dy = path[path_point].y - input.vehicleRelative.y;
      if (dx * dx + dy * dy >= lookahead_squared)
        break;
      ++path_point;
    }
    dx = path[path_point].x - input.vehicleRelative.x;
    dy = path[path_point].y - input.vehicleRelative.y;
    state_.lastPathPoint = path_point;

    const double distance = std::hypot(dx, dy);
    const double path_direction = std::atan2(dy, dx);
    double direction_error = wrapPi(input.vehicleYawRelative - path_direction);

    const double nominal_dt = clamp(params.nominalDt, 1e-4, 1.0);
    const double max_dt = std::max(nominal_dt, params.maxDt);
    double dt = nominal_dt;
    if (std::isfinite(input.currentTime) && state_.lastControlTime >= 0.0 &&
        input.currentTime > state_.lastControlTime) {
      dt = clamp(input.currentTime - state_.lastControlTime, 1e-4, max_dt);
    }
    if (std::isfinite(input.currentTime)) {
      state_.lastControlTime = input.currentTime;
    }
    const double acceleration_step = std::max(0.0, params.maxAccel) * dt;
    const double nominal_acceleration_step = std::max(0.0, params.maxAccel) * nominal_dt;

    if (state_.directionTransition) {
      const double current_speed =
          std::hypot(state_.linearCommand.vx, state_.linearCommand.vy);
      const double next_speed = std::max(0.0, current_speed - acceleration_step);
      if (current_speed > kSpeedEpsilon && next_speed > params.linearStopThreshold) {
        const double scale = next_speed / current_speed;
        state_.linearCommand.vx *= scale;
        state_.linearCommand.vy *= scale;
      } else {
        state_.linearCommand = {};
        state_.directionTransition = false;
      }
      state_.vehicleSpeed = 0.0;
      output.cmd = state_.linearCommand;
      output.directionTransition = true;
      output.executionFrozen = true;
      return output;
    }

    if (params.twoWayDrive) {
      constexpr double kHysteresis = 0.1;
      if (std::fabs(direction_error) > kPi / 2.0 + kHysteresis && state_.forward &&
          input.currentTime - state_.switchTime > params.switchTimeThre) {
        state_.forward = false;
        state_.switchTime = input.currentTime;
      } else if (std::fabs(direction_error) < kPi / 2.0 - kHysteresis && !state_.forward &&
                 input.currentTime - state_.switchTime > params.switchTimeThre) {
        state_.forward = true;
        state_.switchTime = input.currentTime;
      }
    }

    // The requested speed is a normalized contract shared with the spline
    // follower. Clamp it here as well so path execution cannot exceed the
    // configured cap or turn a malformed negative request into reverse motion.
    const double requested_speed = clamp(input.requestedSpeed, 0.0, 1.0);
    double target_speed = params.maxSpeed * requested_speed;
    if (!state_.forward) {
      direction_error = wrapPi(direction_error + kPi);
      target_speed *= -1.0;
    }
    const double min_speed = clamp(params.minSpeed, 0.0, std::max(0.0, params.maxSpeed));
    if (std::fabs(target_speed) > kSpeedEpsilon && std::fabs(target_speed) < min_speed) {
      target_speed = std::copysign(min_speed, target_speed);
    }
    output.directionError = direction_error;

    double yaw_rate =
        -(std::fabs(state_.vehicleSpeed) < 2.0 * nominal_acceleration_step ? params.stopYawRateGain
                                                                           : params.yawRateGain) *
        direction_error;
    const double max_yaw_rate = std::max(0.0, params.maxYawRateRadS);
    yaw_rate = clamp(yaw_rate, -max_yaw_rate, max_yaw_rate);

    const bool force_yaw_stop =
        path_size <= 1 || (distance < params.stopDisThre && params.noRotAtGoal);
    if (force_yaw_stop) {
      yaw_rate = 0.0;
    }

    double turn_speed_scale = 1.0;
    if (params.turnSpeedYawRateStart > 0.0 && params.turnSpeedMinScale < 1.0 &&
        max_yaw_rate > params.turnSpeedYawRateStart) {
      const double min_scale = clamp(params.turnSpeedMinScale, 0.0, 1.0);
      const double ratio = clamp((std::fabs(yaw_rate) - params.turnSpeedYawRateStart) /
                                     (max_yaw_rate - params.turnSpeedYawRateStart),
                                 0.0, 1.0);
      turn_speed_scale = 1.0 - (1.0 - min_scale) * ratio;
    }
    output.turnSpeedScale = turn_speed_scale;

    if (path_size <= 1) {
      target_speed = 0.0;
    } else if (params.slowDwnDisThre > 0.0 &&
               end_distance / params.slowDwnDisThre < requested_speed) {
      target_speed *= end_distance / params.slowDwnDisThre;
    }

    const double commanded_speed =
        target_speed * clamp(input.slowFactor, 0.0, 1.0) * turn_speed_scale;
    const auto step_toward_by = [](double current, double target, double max_step) {
      if (current < target) {
        return std::min(current + max_step, target);
      }
      if (current > target) {
        return std::max(current - max_step, target);
      }
      return current;
    };

    const bool allow_omni_translation =
        distance < params.omniDirGoalThre &&
        std::fabs(direction_error) < params.omniDirDiffThre;
    state_.headingAlignmentActive =
        updateHeadingAlignment(state_.headingAlignmentActive, direction_error, params,
                               allow_omni_translation);
    const bool can_accelerate = !state_.headingAlignmentActive &&
                                distance > params.stopDisThre;
    output.canAccelerate = can_accelerate;
    output.executionFrozen = state_.headingAlignmentActive;
    state_.vehicleSpeed =
        can_accelerate ? step_toward_by(state_.vehicleSpeed, commanded_speed, acceleration_step)
                       : step_toward_by(state_.vehicleSpeed, 0.0, acceleration_step);

    if (input.safetyStop >= 1)
      state_.vehicleSpeed = 0.0;
    if (input.safetyStop >= 2 || force_yaw_stop) {
      yaw_rate = 0.0;
      state_.yawCommand = 0.0;
    } else if (params.maxYawAccelRadS2 > 0.0) {
      const double max_yaw_step = params.maxYawAccelRadS2 * dt;
      state_.yawCommand = step_toward_by(state_.yawCommand, yaw_rate, max_yaw_step);
      yaw_rate = state_.yawCommand;
    } else {
      state_.yawCommand = yaw_rate;
    }

    output.cmd.wz = yaw_rate;
    if (std::fabs(state_.vehicleSpeed) > std::max(0.0, params.linearStopThreshold)) {
      if (params.omniDirGoalThre > 0.0) {
        output.cmd.vx = std::cos(direction_error) * state_.vehicleSpeed;
        output.cmd.vy = -std::sin(direction_error) * state_.vehicleSpeed;
      } else {
        output.cmd.vx = state_.vehicleSpeed;
      }
    }
    state_.linearCommand.vx = output.cmd.vx;
    state_.linearCommand.vy = output.cmd.vy;
    return output;
  }

  void stopLinear() {
    state_.vehicleSpeed = 0.0;
    state_.linearCommand = {};
    state_.directionTransition = false;
  }

  void resetTarget() {
    state_.lastPathPoint = 0;
    state_.lastPathSize = 0;
    state_.headingAlignmentActive = false;
  }

  void resetIntent() {
    resetTarget();
    state_.vehicleSpeed = 0.0;
    state_.yawCommand = 0.0;
    state_.forward = true;
    state_.switchTime = 0.0;
    state_.directionTransition =
        std::hypot(state_.linearCommand.vx, state_.linearCommand.vy) > kSpeedEpsilon;
  }

  void reset() { state_ = {}; }

  FollowerDiagnostics diagnostics() const {
    return {
        true,
        FollowerAlgorithm::Path,
        state_.vehicleSpeed,
        state_.forward,
    };
  }

 private:
  struct State {
    double vehicleSpeed{0.0};
    double yawCommand{0.0};
    int lastPathPoint{0};
    int lastPathSize{0};
    bool forward{true};
    bool headingAlignmentActive{false};
    bool directionTransition{false};
    double switchTime{0.0};
    double lastControlTime{-1.0};
    Twist linearCommand{};
  } state_;
};

class SplineAlgorithm {
 public:
  FollowerOutput follow(const SplineTarget &spline, const FollowerState &input) {
    FollowerOutput output;
    if (!std::isfinite(input.currentTime) || !std::isfinite(input.vehicleYawRelative) ||
        !std::isfinite(input.vehicleRelative.x) || !std::isfinite(input.vehicleRelative.y) ||
        !std::isfinite(input.vehicleRelative.z) ||
        (input.desiredHeading && !std::isfinite(*input.desiredHeading)) ||
        (input.desiredYawRate && !std::isfinite(*input.desiredYawRate))) {
      stopLinear();
      return output;
    }
    const SplineFollowerParams &scan = input.params.spline;
    const double requestedLimit = std::min(
        std::hypot(scan.maxVx, scan.maxVy),
        std::max(0.0, input.params.maxSpeed) * clamp(input.requestedSpeed, 0.0, 1.0) *
            clamp(input.slowFactor, 0.0, 1.0));
    const bool newTrajectory = !controller_.hasTrajectory() ||
                              controller_.trajectoryId() != spline.trajectoryId;
    if (newTrajectory) {
      splineView_.emplace(spline);
      splineSpeedBound_ = splineView_->speedBound();
    }
    const bool speedReplan = spline.speedTransitionEndS <= 0.0 &&
                            spline.maxLinearSpeedMps > 0.0 &&
                            spline.maxLinearSpeedMps > requestedLimit + 1e-9 &&
                            splineSpeedBound_ > requestedLimit + 1e-9;
    if (invalidatedTrajectoryId_ && *invalidatedTrajectoryId_ != spline.trajectoryId)
      invalidatedTrajectoryId_.reset();
    if (speedReplan || (spline.speedTransitionEndS > 0.0 &&
                       spline.maxLinearSpeedMps > requestedLimit + 1e-9))
      invalidatedTrajectoryId_ = spline.trajectoryId;
    const bool waitingForReduction = invalidatedTrajectoryId_.has_value();
    const bool externalHold = waitingForReduction ||
                              requestedLimit <= kSpeedEpsilon || input.safetyStop >= 1;
    double speedLimit = spline.maxLinearSpeedMps > 0.0
                                 ? std::min(requestedLimit, spline.maxLinearSpeedMps)
                                 : requestedLimit;

    // Command time stays independent of the reference clock reset by a new ID.
    const double elapsed = lastInputTime_ ? input.currentTime - *lastInputTime_ : 0.0;
    double commandDt = clamp(input.params.nominalDt, 0.0, std::max(0.0, input.params.maxDt));
    // A declared pause resumes with one command step, not the whole hold.
    if (lastInputTime_ && (!paused_ || elapsed <= 0.0))
      commandDt = clamp(elapsed, 0.0, std::max(0.0, input.params.maxDt));
    // Endpoint holds stop execution without restarting the upstream spline.
    const bool referenceHeld = paused_ || externalHold;
    const double referenceDt = referenceHeld ? 0.0 : std::max(0.0, elapsed);
    if (referenceHeld && lastInputTime_ && input.currentTime > *lastInputTime_)
      pausedTime_ += input.currentTime - *lastInputTime_;
    lastInputTime_ = input.currentTime;
    paused_ = false;
    const double controlTime = input.currentTime - pausedTime_;

    if (newTrajectory) {
      local::scan::upstream::BsplineTrajectory trajectory;
      trajectory.positionPoints.resize(
          3, static_cast<Eigen::Index>(spline.controls.size()));
      for (std::size_t index = 0; index < spline.controls.size(); ++index) {
        trajectory.positionPoints(0, static_cast<Eigen::Index>(index)) =
            spline.controls[index].x;
        trajectory.positionPoints(1, static_cast<Eigen::Index>(index)) =
            spline.controls[index].y;
        trajectory.positionPoints(2, static_cast<Eigen::Index>(index)) =
            spline.controls[index].z;
      }
      trajectory.knots.resize(static_cast<Eigen::Index>(spline.knots.size()));
      for (std::size_t index = 0; index < spline.knots.size(); ++index) {
        trajectory.knots(static_cast<Eigen::Index>(index)) =
            spline.knots[index];
      }
      trajectory.order = spline.order;
      trajectory.trajectoryId = spline.trajectoryId;
      // Only an executing trajectory supplies a physical replacement epoch.
      // Initial planning wait is not motion; start the first reference at zero.
      // A replacement must not replay its already elapsed prefix and pull a
      // moving robot back toward the planning snapshot.
      const double elapsed = controller_.hasTrajectory() && !referenceHeld && spline.startTimeS >= 0.0
                                 ? std::max(0.0, input.currentTime - spline.startTimeS)
                                 : 0.0;
      trajectory.startTimeS = controlTime - elapsed;
      if (!controller_.setTrajectory(trajectory, controlTime)) {
        stopLinear();
        return output;
      }
    }

    // This prefix has its own physical time and collision checks; an old
    // overspeed spline never gets this allowance merely from a new request.
    if (spline.speedTransitionEndS > 0.0 && !waitingForReduction) {
      const double phase = newTrajectory
          ? (spline.startTimeS >= 0.0 && !referenceHeld
                 ? std::max(0.0, input.currentTime - spline.startTimeS) : 0.0)
          : tracking_.executionTimeS + referenceDt;
      if (phase < spline.speedTransitionEndS) {
        const Vec3 velocity = splineView_->velocity(phase);
        speedLimit = std::max(speedLimit, std::hypot(velocity.x, velocity.y));
      }
    }

    local::scan::upstream::ClosedLoopControllerState state;
    state.position = {input.vehicleRelative.x, input.vehicleRelative.y,
                      input.vehicleRelative.z};
    state.yaw = input.vehicleYawRelative;
    state.nowS = controlTime;
    state.desiredHeading = input.desiredHeading;
    state.desiredYawRate = input.desiredYawRate;

    local::scan::upstream::ClosedLoopControllerParams params;
    params.timeForward = scan.timeForward;
    params.headingErrorThreshold = scan.headingErrorThreshold;
    params.positionGain = scan.positionGain;
    params.yawGain = scan.yawGain;
    params.maxLinearSpeed = speedLimit;
    params.maxVx = std::min(scan.maxVx, speedLimit);
    params.maxVy = std::min(scan.maxVy, speedLimit);
    params.maxYawRate = scan.maxYawRateRadS;
    params.finishDistance = scan.finishDistance;

    const local::scan::upstream::ClosedLoopControllerOutput controlled =
        controller_.step(state, params);
    output.cmd = controlled.command;
    output.directionError = controlled.yawError;
    output.endDistance = controlled.endDistance;
    output.executionFrozen = controlled.executionFrozen;
    output.finished = controlled.finished && !waitingForReduction;
    output.awaitingTrajectory = invalidatedTrajectoryId_.has_value();
    const double remaining = std::max(0.0, output.endDistance - params.finishDistance);
    const double minVy = std::min({scan.minVy, params.maxVy,
        std::sqrt(2.0 * input.params.maxAccel * remaining)});
    if (!externalHold && std::abs(output.cmd.vy) > kSpeedEpsilon &&
        std::abs(output.cmd.vy) < minVy) {
      // Shape the target before the acceleration ramp and final collision check.
      output.cmd.vy = std::copysign(minVy, output.cmd.vy);
      const double maxVx = std::sqrt(std::max(0.0, speedLimit * speedLimit - minVy * minVy));
      output.cmd.vx = clamp(output.cmd.vx, -maxVx, maxVx);
    }
    if (!hasMotionCommand_) {
      const auto &measured = input.measuredBodyTwist;
      if (input.measuredBodyTwistValid && std::isfinite(measured.vx) &&
          std::isfinite(measured.vy) && std::isfinite(measured.wz)) {
        lastCommand_ = measured;
      }
      lastCommandYaw_ = input.vehicleYawRelative;
      hasMotionCommand_ = true;
    }
    if (!output.finished) {
      if (!externalHold) {
        // Bound the physical XY velocity change in one world frame. Rotating
        // the body frame alone must not look like extra acceleration.
        const double c = std::cos(input.vehicleYawRelative);
        const double s = std::sin(input.vehicleYawRelative);
        const double pc = std::cos(lastCommandYaw_);
        const double ps = std::sin(lastCommandYaw_);
        const double previousX = pc * lastCommand_.vx - ps * lastCommand_.vy;
        const double previousY = ps * lastCommand_.vx + pc * lastCommand_.vy;
        const double dx = c * output.cmd.vx - s * output.cmd.vy - previousX;
        const double dy = s * output.cmd.vx + c * output.cmd.vy - previousY;
        const double delta = std::hypot(dx, dy);
        const double step = std::max(0.0, input.params.maxAccel) * commandDt;
        const double scale = delta > step && delta > 1e-12 ? step / delta : 1.0;
        const double worldX = previousX + scale * dx;
        const double worldY = previousY + scale * dy;
        output.cmd.vx = clamp(c * worldX + s * worldY, -params.maxVx, params.maxVx);
        output.cmd.vy = clamp(-s * worldX + c * worldY, -params.maxVy, params.maxVy);
        const double speed = std::hypot(output.cmd.vx, output.cmd.vy);
        if (speed > speedLimit && speed > 1e-12) {
          output.cmd.vx *= speedLimit / speed;
          output.cmd.vy *= speedLimit / speed;
        }
      }
      if (input.params.maxYawAccelRadS2 > 0.0) {
        const double step = input.params.maxYawAccelRadS2 * commandDt;
        output.cmd.wz = clamp(output.cmd.wz, lastCommand_.wz - step, lastCommand_.wz + step);
        const double yawLimit = std::min(1.0, std::max(0.0, params.maxYawRate));
        output.cmd.wz = clamp(output.cmd.wz, -yawLimit, yawLimit);
      }
    }
    if (externalHold) {
      output.cmd.vx = 0.0;
      output.cmd.vy = 0.0;
      if (waitingForReduction && requestedLimit > kSpeedEpsilon && input.safetyStop == 0) {
        // Do not advance an invalid reference while its replacement is being
        // prepared. Brake from the applied command; the final arbiter still
        // checks this motion against the current map and measured velocity.
        const double previousSpeed = std::hypot(lastCommand_.vx, lastCommand_.vy);
        const double speed = std::max(0.0, previousSpeed -
            std::max(0.0, input.params.maxAccel) * commandDt);
        const double angle = lastCommandYaw_ - input.vehicleYawRelative;
        const double scale = previousSpeed > 0.0 ? speed / previousSpeed : 0.0;
        output.cmd.vx = scale * (std::cos(angle) * lastCommand_.vx - std::sin(angle) * lastCommand_.vy);
        output.cmd.vy = scale * (std::sin(angle) * lastCommand_.vx + std::cos(angle) * lastCommand_.vy);
      }
      if (input.safetyStop >= 2) output.cmd.wz = 0.0;
      output.executionFrozen = true;
    }
    output.tracking = {controller_.hasTrajectory(), spline.trajectoryId,
                       controlled.executionTimeS, controlled.durationS,
                       controlled.positionErrorM, controlled.yawError,
                       controlled.endDistance, output.executionFrozen,
                       output.finished, speedLimit, input.currentTime};
    lastCommand_ = output.cmd;
    lastCommandYaw_ = input.vehicleYawRelative;
    tracking_ = output.tracking;
    output.canAccelerate =
        std::hypot(output.cmd.vx, output.cmd.vy) > kSpeedEpsilon;
    return output;
  }

  void commitApplied(const Twist &command) {
    if (controller_.hasTrajectory() && hasMotionCommand_)
      lastCommand_ = command;
  }

  void stopLinear() {
    paused_ = true;
    lastCommand_ = {};
    hasMotionCommand_ = true;
    tracking_.executionFrozen = true;
  }

  void resetTarget() { reset(); }

  void resetIntent() { reset(); }

  void reset() {
    controller_.reset();
    lastInputTime_.reset();
    pausedTime_ = 0.0;
    paused_ = false;
    lastCommand_ = {};
    lastCommandYaw_ = 0.0;
    hasMotionCommand_ = false;
    tracking_ = {};
    invalidatedTrajectoryId_.reset();
    splineView_.reset();
  }

  FollowerDiagnostics diagnostics() const {
    return {
        controller_.hasTrajectory(),
        FollowerAlgorithm::Spline,
        std::hypot(lastCommand_.vx, lastCommand_.vy),
        lastCommand_.vx >= 0.0,
        tracking_,
    };
  }


 private:
  local::scan::upstream::ClosedLoopController controller_{};
  std::optional<double> lastInputTime_;
  double pausedTime_{0.0};
  bool paused_{false};
  Twist lastCommand_{};
  double lastCommandYaw_{0.0};
  bool hasMotionCommand_{false};
  FollowerTracking tracking_{};
  std::optional<std::int64_t> invalidatedTrajectoryId_;
  double splineSpeedBound_{0.0};
  std::optional<SplineView> splineView_;
};

}  // namespace

FollowerParams cmuFollowerParams(FollowerParams params) {
  const auto &profile = params.cmu;
  params.baseLookAheadDis = profile.lookahead_m;
  params.lookAheadRatio = 0.0;
  params.minLookAheadDis = profile.lookahead_m;
  params.maxLookAheadDis = profile.lookahead_m;
  params.yawRateGain = profile.yaw_gain;
  params.stopYawRateGain = profile.yaw_gain;
  params.minSpeed = 0.0;
  params.linearStopThreshold = params.maxAccel / 100.0;
  params.nominalDt = profile.nominal_dt_s;
  params.switchTimeThre = profile.switch_time_s;
  params.headingAlignEnterRad = profile.heading_align_rad;
  params.headingAlignExitRad = profile.heading_align_rad;
  params.omniDirGoalThre = profile.omni_goal_m;
  params.omniDirDiffThre = profile.omni_heading_rad;
  params.stopDisThre = profile.stop_distance_m;
  params.slowDwnDisThre = profile.slowdown_distance_m;
  params.turnSpeedYawRateStart = 0.0;
  params.turnSpeedMinScale = 1.0;
  params.cornerGate = false;
  params.noRotAtGoal = true;
  return params;
}

FollowerParams recoveryFollowerParams(FollowerParams params, double speed, double localRange) {
  params.maxSpeed = speed;
  params.minSpeed = 0.0;
  // Recovery completes within 0.125 m; its follower must not stop outside that radius.
  params.stopDisThre = 0.08;
  params.slowDwnDisThre = 0.0;
  params.yawRateGain = 0.0;
  params.stopYawRateGain = 0.0;
  params.maxYawRateRadS = 0.0;
  params.twoWayDrive = false;
  params.headingAlignEnterRad = kPi + 0.1;
  params.headingAlignExitRad = kPi;
  params.omniDirDiffThre = kPi + 0.1;
  params.omniDirGoalThre = std::max(2.0, localRange);
  params.noRotAtGoal = true;
  return params;
}

const char *followerAlgorithmName(FollowerAlgorithm algorithm) noexcept {
  switch (algorithm) {
    case FollowerAlgorithm::Path:
      return "path";
    case FollowerAlgorithm::Spline:
      return "spline";
  }
  return "path";
}

class Follower::Impl {
 public:
  FollowerOutput follow(const LocalPlan &plan, const FollowerState &state) {
    auto input = state;
    const auto *path = std::get_if<PathTarget>(&plan.target());
    if (plan.ready()) {
      const auto requested = path ? FollowerAlgorithm::Path : FollowerAlgorithm::Spline;
      if (active_ != requested) {
        path_.reset();
        spline_.reset();
        active_ = requested;
      }
    }
    if (path && input.standardPathProfile) input.params = cmuFollowerParams(input.params);

    if (!plan.ready() && !path) {
      FollowerOutput result;
      result.awaitingTrajectory = true;
      return result;
    }
    // A reference hold must not overwrite the last command confirmed by final control.
    if (!path && hasApplied_) spline_.commitApplied(applied_);
    auto result = path ? path_.follow(path->points, input)
                      : spline_.follow(std::get<SplineTarget>(plan.target()), input);
    return result;
  }

  void commitApplied(const Twist &command) {
    applied_ = command;
    hasApplied_ = true;
    if (active_ == FollowerAlgorithm::Spline) spline_.commitApplied(command);
  }

  void stopLinear() {
    path_.stopLinear();
    spline_.stopLinear();
  }

  void resetTarget() {
    if (active_ == FollowerAlgorithm::Path) path_.resetTarget();
    else if (active_ == FollowerAlgorithm::Spline) spline_.resetTarget();
  }

  void resetIntent() {
    if (active_ == FollowerAlgorithm::Path) path_.resetIntent();
    else if (active_ == FollowerAlgorithm::Spline) spline_.resetIntent();
  }

  void reset() {
    hasApplied_ = false;
    path_.reset();
    spline_.reset();
    active_.reset();
  }

  FollowerDiagnostics diagnostics() const {
    auto result = active_ == FollowerAlgorithm::Path ? path_.diagnostics()
        : active_ == FollowerAlgorithm::Spline ? spline_.diagnostics() : FollowerDiagnostics{};
    return result;
  }

 private:
  PathAlgorithm path_;
  SplineAlgorithm spline_;
  Twist applied_{};
  bool hasApplied_{false};
  std::optional<FollowerAlgorithm> active_;
};

Follower::Follower() : impl_(std::make_unique<Impl>()) {}
Follower::~Follower() = default;
Follower::Follower(Follower &&) noexcept = default;
Follower &Follower::operator=(Follower &&) noexcept = default;

FollowerOutput Follower::follow(const LocalPlan &plan, const FollowerState &state) {
  return impl_->follow(plan, state);
}

double Follower::turnRate(double yawError, double gain, double maxRate) {
  return std::clamp(gain * yawError, -maxRate, maxRate);
}

Twist Follower::rotationCommand(int direction, double rate) {
  return {0.0, 0.0, static_cast<double>(direction) * rate};
}

void Follower::stopLinear() {
  impl_->stopLinear();
}

void Follower::commitApplied(const Twist &command) {
  impl_->commitApplied(command);
}

void Follower::resetTarget() {
  impl_->resetTarget();
}

void Follower::resetIntent() {
  impl_->resetIntent();
}

void Follower::reset() {
  impl_->reset();
}

FollowerDiagnostics Follower::diagnostics() const {
  return impl_->diagnostics();
}

}  // namespace nav_kernel
