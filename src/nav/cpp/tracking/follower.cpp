#include "tracking/follower.hpp"

#include "planning/local/planner.hpp"
#include "trajectory/spline.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <type_traits>
#include <utility>

namespace nav_kernel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedEpsilon = 1e-4;
constexpr double kCornerCos = 0.7071067811865476;  // 45 degrees
constexpr double kCornerReachM = 0.10;

using FollowerTarget = std::variant<std::reference_wrapper<const std::vector<Vec3>>,
                                    std::reference_wrapper<const SplineTarget>>;

struct FollowerInput : FollowerState {
  explicit FollowerInput(const std::vector<Vec3> &path) : target(std::cref(path)) {}
  explicit FollowerInput(const SplineTarget &spline) : target(std::cref(spline)) {}

  [[nodiscard]] FollowerAlgorithm algorithm() const noexcept {
    return std::holds_alternative<std::reference_wrapper<const std::vector<Vec3>>>(target)
               ? FollowerAlgorithm::Path
               : FollowerAlgorithm::Spline;
  }

  FollowerTarget target;
};

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

class Algorithm {
 public:
  virtual ~Algorithm() = default;
  virtual FollowerOutput follow(const FollowerInput &input) = 0;
  virtual void stopLinear() = 0;
  virtual void resetTarget() = 0;
  virtual void resetIntent() = 0;
  virtual void reset() = 0;
  [[nodiscard]] virtual FollowerDiagnostics diagnostics() const = 0;
};

class PathAlgorithm final : public Algorithm {
 public:
  FollowerOutput follow(const FollowerInput &input) override {
    FollowerOutput output;
    const auto *target =
        std::get_if<std::reference_wrapper<const std::vector<Vec3>>>(&input.target);
    if (target == nullptr)
      return output;
    const std::vector<Vec3> &path = target->get();
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
    state_.pathPoint = state_.lastPathPoint > 2 ? state_.lastPathPoint - 2 : 0;

    double dx = 0.0;
    double dy = 0.0;
    const double lookahead_squared = lookahead_distance * lookahead_distance;
    const int path_point_limit =
        params.cornerGate ? cornerLimit(path, state_.pathPoint, input.vehicleRelative)
                          : path_size - 1;
    while (state_.pathPoint < path_point_limit) {
      dx = path[state_.pathPoint].x - input.vehicleRelative.x;
      dy = path[state_.pathPoint].y - input.vehicleRelative.y;
      if (dx * dx + dy * dy >= lookahead_squared)
        break;
      ++state_.pathPoint;
    }
    dx = path[state_.pathPoint].x - input.vehicleRelative.x;
    dy = path[state_.pathPoint].y - input.vehicleRelative.y;
    state_.lastPathPoint = state_.pathPoint;

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

    double target_speed = params.maxSpeed * input.requestedSpeed;
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

    if (path_size <= 1 || (distance < params.stopDisThre && params.noRotAtGoal)) {
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
               end_distance / params.slowDwnDisThre < input.requestedSpeed) {
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
    const bool force_yaw_stop =
        path_size <= 1 || (distance < params.stopDisThre && params.noRotAtGoal);
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

  void stopLinear() override {
    state_.vehicleSpeed = 0.0;
    state_.linearCommand = {};
    state_.directionTransition = false;
  }

  void resetTarget() override {
    state_.pathPoint = 0;
    state_.lastPathPoint = 0;
    state_.lastPathSize = 0;
    state_.headingAlignmentActive = false;
  }

  void resetIntent() override {
    resetTarget();
    state_.vehicleSpeed = 0.0;
    state_.yawCommand = 0.0;
    state_.forward = true;
    state_.switchTime = 0.0;
    state_.directionTransition =
        std::hypot(state_.linearCommand.vx, state_.linearCommand.vy) > kSpeedEpsilon;
  }

  void reset() override { state_ = {}; }

  FollowerDiagnostics diagnostics() const override {
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
    int pathPoint{0};
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

class SplineAlgorithm final : public Algorithm {
 public:
  FollowerOutput follow(const FollowerInput &input) override {
    FollowerOutput output;
    const auto *target =
        std::get_if<std::reference_wrapper<const SplineTarget>>(&input.target);
    if (target == nullptr)
      return output;
    const SplineTarget &spline = target->get();
    const FollowerParams &params = input.params;
    const SplineView spline_view(spline);
    if (!spline_view.valid())
      return output;

    const SplineFollowerParams &scan = params.spline;
    const double duration = spline_view.duration();
    // SCAN owns the trajectory clock. Do not jump that clock to an arbitrary
    // nearest point: doing so can skip across bends or revisit an older branch
    // when the local spline passes near the robot more than once.
    const double time = clamp(spline.timeS, 0.0, duration);
    const Vec3 current_position = spline_view.position(time);
    const Vec3 current_velocity = spline_view.velocity(time);
    const double lookahead = std::max(0.0, scan.timeForward);
    const double lookahead_time = std::min(duration, time + lookahead);
    const Vec3 lookahead_position = spline_view.position(lookahead_time);
    const Vec3 endpoint = spline_view.position(duration);
    if (!std::isfinite(current_position.x) || !std::isfinite(current_position.y) ||
        !std::isfinite(current_velocity.x) || !std::isfinite(current_velocity.y) ||
        !std::isfinite(lookahead_position.x) || !std::isfinite(lookahead_position.y) ||
        !std::isfinite(endpoint.x) || !std::isfinite(endpoint.y)) {
      return output;
    }
    const double end_distance = std::hypot(endpoint.x, endpoint.y);
    double terminal_scale = 1.0;
    if (std::isfinite(input.goalDistance) && input.goalDistance >= 0.0 &&
        params.slowDwnDisThre > params.stopDisThre) {
      terminal_scale = clamp((input.goalDistance - params.stopDisThre) /
                                 (params.slowDwnDisThre - params.stopDisThre),
                             0.0, 1.0);
    }

    double heading_x = lookahead_position.x - current_position.x;
    double heading_y = lookahead_position.y - current_position.y;
    constexpr double kLookaheadDirectionSquaredThreshold = 1e-4;
    if (heading_x * heading_x + heading_y * heading_y <
        kLookaheadDirectionSquaredThreshold) {
      heading_x = current_velocity.x;
      heading_y = current_velocity.y;
    }
    const double desired_yaw =
        input.holdBodyHeading
            ? 0.0
            : (heading_x * heading_x + heading_y * heading_y >=
                       kLookaheadDirectionSquaredThreshold
                   ? std::atan2(heading_y, heading_x)
                   : 0.0);
    const double yaw_error = wrapPi(desired_yaw);
    const double yaw_limit =
        std::min(std::max(0.0, params.maxYawRateRadS), std::max(0.0, scan.maxYawRateRadS));
    // Match SCAN's closed-loop controller: use look-ahead heading error for
    // yaw control. The sampled spline yaw rate is a finite-difference
    // diagnostic and becomes noisy near low-speed control points.
    double yaw_command = input.holdBodyHeading
                             ? 0.0
                             : clamp(std::max(0.0, scan.yawGain) * yaw_error,
                                     -yaw_limit, yaw_limit);
    if (params.noRotAtGoal)
      yaw_command *= terminal_scale;
    if (!input.holdBodyHeading &&
        std::abs(yaw_error) > std::max(0.0, scan.headingErrorThreshold)) {
      state_.command.vx = 0.0;
      state_.command.vy = 0.0;
      state_.command.wz = input.safetyStop >= 2 ? 0.0 : limitYaw(input, yaw_command);
      rememberControlTime(input);
      output.cmd = state_.command;
      output.endDistance = end_distance;
      output.directionError = yaw_error;
      output.executionFrozen = true;
      return output;
    }

    const double gain = std::max(0.0, scan.positionGain);
    Twist desired;
    // Match SCAN's closed-loop controller: the look-ahead point selects yaw,
    // while translation tracks the state at the current trajectory time.
    // Driving toward the future state here cuts corners and overreacts at every
    // replan boundary.
    desired.vx = current_velocity.x + gain * current_position.x;
    desired.vy = current_velocity.y + gain * current_position.y;
    desired.wz = yaw_command;

    const double policy_limit = std::max(0.0, params.maxSpeed) *
                                clamp(input.requestedSpeed, 0.0, 1.0) *
                                clamp(input.slowFactor, 0.0, 1.0);
    const double max_vx =
        std::min(policy_limit, std::max(0.0, scan.maxVx)) * terminal_scale;
    const double max_vy =
        std::min(policy_limit, std::max(0.0, scan.maxVy)) * terminal_scale;
    const double norm_limit = std::max(max_vx, max_vy);
    const double speed = std::hypot(desired.vx, desired.vy);
    if (speed > norm_limit && speed > 1e-9) {
      const double scale = norm_limit / speed;
      desired.vx *= scale;
      desired.vy *= scale;
    }
    desired.vx = clamp(desired.vx, -max_vx, max_vx);
    desired.vy = clamp(desired.vy, -max_vy, max_vy);
    desired.wz = clamp(desired.wz, -yaw_limit, yaw_limit);

    const double dt = controlDt(input);
    const Twist base = state_.lastControlTime >= 0.0 ? state_.command : input.measuredBodyTwist;
    const double delta_x = desired.vx - base.vx;
    const double delta_y = desired.vy - base.vy;
    const double delta_norm = std::hypot(delta_x, delta_y);
    const double max_linear_delta = std::max(0.0, params.maxAccel) * dt;
    if (delta_norm > max_linear_delta && delta_norm > 1e-9) {
      const double scale = max_linear_delta / delta_norm;
      state_.command.vx = base.vx + scale * delta_x;
      state_.command.vy = base.vy + scale * delta_y;
    } else {
      state_.command.vx = desired.vx;
      state_.command.vy = desired.vy;
    }
    state_.command.wz = limitYaw(input, desired.wz);
    rememberControlTime(input);

    if (spline.timeS >= duration &&
        end_distance < std::max(0.0, scan.finishDistance)) {
      state_.command = {};
      output.finished = true;
    }

    if (input.safetyStop >= 1) {
      state_.command.vx = 0.0;
      state_.command.vy = 0.0;
    }
    if (input.safetyStop >= 2)
      state_.command.wz = 0.0;

    output.cmd = state_.command;
    output.endDistance = end_distance;
    output.directionError = yaw_error;
    output.canAccelerate = norm_limit > 0.0;
    return output;
  }

  void stopLinear() override {
    state_.command.vx = 0.0;
    state_.command.vy = 0.0;
  }

  void resetTarget() override {}

  void resetIntent() override { reset(); }

  void reset() override { state_ = {}; }

  FollowerDiagnostics diagnostics() const override {
    return {
        true,
        FollowerAlgorithm::Spline,
        std::hypot(state_.command.vx, state_.command.vy),
        true,
    };
  }

 private:
  double controlDt(const FollowerInput &input) const {
    const double nominal = clamp(input.params.nominalDt, 1e-4, 1.0);
    if (!std::isfinite(input.currentTime) || state_.lastControlTime < 0.0 ||
        input.currentTime <= state_.lastControlTime) {
      return nominal;
    }
    return clamp(input.currentTime - state_.lastControlTime, 1e-4,
                 std::max(nominal, input.params.maxDt));
  }

  double limitYaw(const FollowerInput &input, double desired) const {
    if (!(input.params.maxYawAccelRadS2 > 0.0))
      return desired;
    const double base = state_.lastControlTime >= 0.0 ? state_.command.wz
                                                      : input.measuredBodyTwist.wz;
    const double step = input.params.maxYawAccelRadS2 * controlDt(input);
    return base + clamp(desired - base, -step, step);
  }

  void rememberControlTime(const FollowerInput &input) {
    if (std::isfinite(input.currentTime))
      state_.lastControlTime = input.currentTime;
  }

  struct State {
    Twist command{};
    double lastControlTime{-1.0};
  } state_;
};

using AlgorithmFactory = std::unique_ptr<Algorithm> (*)();

class AlgorithmRegistry {
 public:
  void registerAlgorithm(FollowerAlgorithm algorithm, AlgorithmFactory factory) {
    factories_.insert_or_assign(algorithm, factory);
  }

  [[nodiscard]] std::unique_ptr<Algorithm> create(FollowerAlgorithm algorithm) const {
    const auto found = factories_.find(algorithm);
    return found == factories_.end() ? nullptr : found->second();
  }

 private:
  std::map<FollowerAlgorithm, AlgorithmFactory> factories_;
};

std::unique_ptr<Algorithm> makePathAlgorithm() {
  return std::make_unique<PathAlgorithm>();
}

std::unique_ptr<Algorithm> makeSplineAlgorithm() {
  return std::make_unique<SplineAlgorithm>();
}

const AlgorithmRegistry &builtinAlgorithms() {
  static const AlgorithmRegistry registry = [] {
    AlgorithmRegistry result;
    result.registerAlgorithm(FollowerAlgorithm::Path, &makePathAlgorithm);
    result.registerAlgorithm(FollowerAlgorithm::Spline, &makeSplineAlgorithm);
    return result;
  }();
  return registry;
}

}  // namespace

FollowerParams cmuFollowerParams(FollowerParams params) {
  constexpr double kLookAheadM = 0.5;
  params.baseLookAheadDis = kLookAheadM;
  params.lookAheadRatio = 0.0;
  params.minLookAheadDis = kLookAheadM;
  params.maxLookAheadDis = kLookAheadM;
  params.yawRateGain = 1.5;
  params.stopYawRateGain = 1.5;
  params.minSpeed = 0.0;
  params.maxAccel = 2.0;
  params.linearStopThreshold = params.maxAccel / 100.0;
  params.nominalDt = 0.01;
  params.switchTimeThre = 1.0;
  params.omniDirGoalThre = 0.4;
  params.omniDirDiffThre = 1.5;
  params.stopDisThre = 0.3;
  params.slowDwnDisThre = 0.75;
  params.turnSpeedYawRateStart = 0.0;
  params.turnSpeedMinScale = 1.0;
  params.cornerGate = false;
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
  Impl() {
    registerAlgorithm(FollowerAlgorithm::Path);
    registerAlgorithm(FollowerAlgorithm::Spline);
  }

  FollowerOutput follow(const FollowerInput &input) {
    const FollowerAlgorithm requested = input.algorithm();
    const auto found = algorithms_.find(requested);
    if (found == algorithms_.end())
      return {};

    if (!active_.has_value() || *active_ != requested) {
      if (active_.has_value())
        algorithms_.at(*active_)->reset();
      found->second->reset();
      active_ = requested;
    }
    return found->second->follow(input);
  }

  FollowerOutput follow(const LocalPlan &plan, const FollowerState &state) {
    return std::visit(
        [this, &state](const auto &target) {
          using Target = std::decay_t<decltype(target)>;
          FollowerState effective = state;
          if constexpr (std::is_same_v<Target, PathTarget>) {
            if (effective.standardPathProfile)
              effective.params = cmuFollowerParams(effective.params);
            if (effective.holdBodyHeading) {
              effective.params.yawRateGain = 0.0;
              effective.params.stopYawRateGain = 0.0;
              effective.params.maxYawRateRadS = 0.0;
              effective.params.twoWayDrive = false;
              effective.params.headingAlignEnterRad = M_PI + 0.1;
              effective.params.headingAlignExitRad = M_PI;
              effective.params.omniDirDiffThre = M_PI + 0.1;
              effective.params.omniDirGoalThre =
                  std::max(2.0, effective.params.omniDirGoalThre);
            }
          }
          FollowerInput input = [&target] {
            if constexpr (std::is_same_v<Target, PathTarget>) {
              return FollowerInput(target.points);
            } else {
              return FollowerInput(target);
            }
          }();
          static_cast<FollowerState &>(input) = effective;
          return follow(input);
        },
        plan.target());
  }

  void stopLinear() {
    for (auto &entry : algorithms_)
      entry.second->stopLinear();
  }

  void resetTarget() {
    if (active_.has_value())
      algorithms_.at(*active_)->resetTarget();
  }

  void resetIntent() {
    if (active_.has_value())
      algorithms_.at(*active_)->resetIntent();
  }

  void reset() {
    for (auto &entry : algorithms_)
      entry.second->reset();
    active_.reset();
  }

  FollowerDiagnostics diagnostics() const {
    if (!active_.has_value())
      return {};
    return algorithms_.at(*active_)->diagnostics();
  }

 private:
  void registerAlgorithm(FollowerAlgorithm algorithm) {
    std::unique_ptr<Algorithm> instance = builtinAlgorithms().create(algorithm);
    if (instance != nullptr) {
      algorithms_.insert_or_assign(algorithm, std::move(instance));
    }
  }

  std::map<FollowerAlgorithm, std::unique_ptr<Algorithm>> algorithms_;
  std::optional<FollowerAlgorithm> active_;
};

Follower::Follower() : impl_(std::make_unique<Impl>()) {}
Follower::~Follower() = default;
Follower::Follower(Follower &&) noexcept = default;
Follower &Follower::operator=(Follower &&) noexcept = default;

FollowerOutput Follower::follow(const LocalPlan &plan, const FollowerState &state) {
  return impl_->follow(plan, state);
}

void Follower::stopLinear() {
  impl_->stopLinear();
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
