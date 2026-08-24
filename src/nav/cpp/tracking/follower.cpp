#include "tracking/follower.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <utility>

namespace nav_kernel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedEpsilon = 1e-4;

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

class Algorithm {
 public:
  virtual ~Algorithm() = default;
  virtual FollowerOutput follow(const FollowerInput &input) = 0;
  virtual void stopLinear() = 0;
  virtual void resetTarget() = 0;
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
    while (state_.pathPoint < path_size - 1) {
      dx = path[state_.pathPoint].x - input.vehicleRelative.x;
      dy = path[state_.pathPoint].y - input.vehicleRelative.y;
      if (dx * dx + dy * dy >= lookahead_squared)
        break;
      ++state_.pathPoint;
    }
    if (state_.pathPoint >= path_size - 1) {
      dx = path[path_size - 1].x - input.vehicleRelative.x;
      dy = path[path_size - 1].y - input.vehicleRelative.y;
    }
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
    const double max_yaw_rate = params.maxYawRate * kPi / 180.0;
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
    const auto step_toward = [acceleration_step](double current, double target) {
      if (current < target) {
        return std::min(current + acceleration_step, target);
      }
      if (current > target) {
        return std::max(current - acceleration_step, target);
      }
      return current;
    };

    const bool can_accelerate = (std::fabs(direction_error) < params.dirDiffThre ||
                                 (distance < params.omniDirGoalThre &&
                                  std::fabs(direction_error) < params.omniDirDiffThre)) &&
                                distance > params.stopDisThre;
    output.canAccelerate = can_accelerate;
    state_.vehicleSpeed = can_accelerate ? step_toward(state_.vehicleSpeed, commanded_speed)
                                         : step_toward(state_.vehicleSpeed, 0.0);

    if (input.safetyStop >= 1)
      state_.vehicleSpeed = 0.0;
    if (input.safetyStop >= 2)
      yaw_rate = 0.0;

    output.cmd.wz = yaw_rate;
    if (std::fabs(state_.vehicleSpeed) > kSpeedEpsilon) {
      if (params.omniDirGoalThre > 0.0) {
        output.cmd.vx = std::cos(direction_error) * state_.vehicleSpeed;
        output.cmd.vy = -std::sin(direction_error) * state_.vehicleSpeed;
      } else {
        output.cmd.vx = state_.vehicleSpeed;
      }
    }
    return output;
  }

  void stopLinear() override { state_.vehicleSpeed = 0.0; }

  void resetTarget() override {
    state_.pathPoint = 0;
    state_.lastPathPoint = 0;
    state_.lastPathSize = 0;
    state_.forward = true;
    state_.switchTime = 0.0;
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
    int pathPoint{0};
    int lastPathPoint{0};
    int lastPathSize{0};
    bool forward{true};
    double switchTime{0.0};
    double lastControlTime{-1.0};
  } state_;
};

class TrajectoryAlgorithm final : public Algorithm {
 public:
  FollowerOutput follow(const FollowerInput &input) override {
    FollowerOutput output;
    const auto *target =
        std::get_if<std::reference_wrapper<const std::vector<TrajectoryPoint>>>(&input.target);
    if (target == nullptr)
      return output;
    const std::vector<TrajectoryPoint> &trajectory = target->get();
    const FollowerParams &params = input.params;
    if (trajectory.size() < 2)
      return output;

    const double lookahead = std::max(0.0, params.trajectoryLookAheadS);
    auto sample = std::lower_bound(
        trajectory.begin(), trajectory.end(), lookahead,
        [](const TrajectoryPoint &point, double time) { return point.timeFromStartS < time; });
    if (sample == trajectory.end())
      sample = std::prev(trajectory.end());
    const TrajectoryPoint &current = trajectory.front();
    if (!std::isfinite(current.position.x) || !std::isfinite(current.position.y) ||
        !std::isfinite(current.velocity.x) || !std::isfinite(current.velocity.y) ||
        !std::isfinite(current.yaw) || !std::isfinite(current.yawRate) ||
        !std::isfinite(sample->position.x) || !std::isfinite(sample->position.y)) {
      return output;
    }

    double heading_x = sample->position.x - current.position.x;
    double heading_y = sample->position.y - current.position.y;
    if (std::hypot(heading_x, heading_y) <= 1e-4) {
      heading_x = current.velocity.x;
      heading_y = current.velocity.y;
    }
    const double desired_yaw =
        std::hypot(heading_x, heading_y) > 1e-4 ? std::atan2(heading_y, heading_x) : current.yaw;
    const double yaw_error = wrapPi(desired_yaw);
    const double yaw_limit = std::max(0.0, params.maxYawRate) * kPi / 180.0;
    const double yaw_command =
        clamp(current.yawRate + std::max(0.0, params.trajectoryYawGain) * yaw_error, -yaw_limit,
              yaw_limit);
    if (std::abs(yaw_error) > std::max(0.0, params.trajectoryHeadingErrorThreshold)) {
      state_.command.vx = 0.0;
      state_.command.vy = 0.0;
      state_.command.wz = input.safetyStop >= 2 ? 0.0 : yaw_command;
      if (std::isfinite(input.currentTime))
        state_.lastControlTime = input.currentTime;
      output.cmd = state_.command;
      output.endDistance = std::hypot(trajectory.back().position.x, trajectory.back().position.y);
      output.directionError = yaw_error;
      output.executionFrozen = true;
      return output;
    }

    const double gain = std::max(0.0, params.trajectoryPositionGain);
    Twist desired;
    desired.vx = current.velocity.x + gain * current.position.x;
    desired.vy = current.velocity.y + gain * current.position.y;
    desired.wz = yaw_command;

    const double speed_limit = std::max(0.0, params.maxSpeed) *
                               clamp(input.requestedSpeed, 0.0, 1.0) *
                               clamp(input.slowFactor, 0.0, 1.0);
    const double speed = std::hypot(desired.vx, desired.vy);
    if (speed > speed_limit && speed > 1e-9) {
      const double scale = speed_limit / speed;
      desired.vx *= scale;
      desired.vy *= scale;
    }
    desired.wz = clamp(desired.wz, -yaw_limit, yaw_limit);

    const double nominal_dt = clamp(params.nominalDt, 1e-4, 1.0);
    const double max_dt = std::max(nominal_dt, params.maxDt);
    double dt = nominal_dt;
    if (std::isfinite(input.currentTime) && state_.lastControlTime >= 0.0 &&
        input.currentTime > state_.lastControlTime) {
      dt = clamp(input.currentTime - state_.lastControlTime, 1e-4, max_dt);
    }
    const Twist base = state_.lastControlTime >= 0.0 ? state_.command : input.measuredBodyTwist;
    const double step = std::max(0.0, params.maxAccel) * dt;
    const auto step_toward = [step](double current, double requested) {
      return requested > current ? std::min(current + step, requested)
                                 : std::max(current - step, requested);
    };
    state_.command.vx = step_toward(base.vx, desired.vx);
    state_.command.vy = step_toward(base.vy, desired.vy);
    state_.command.wz = desired.wz;
    if (std::isfinite(input.currentTime)) {
      state_.lastControlTime = input.currentTime;
    }

    if (input.safetyStop >= 1) {
      state_.command.vx = 0.0;
      state_.command.vy = 0.0;
    }
    if (input.safetyStop >= 2)
      state_.command.wz = 0.0;

    output.cmd = state_.command;
    output.endDistance = std::hypot(trajectory.back().position.x, trajectory.back().position.y);
    output.directionError = yaw_error;
    output.canAccelerate = speed_limit > 0.0;
    return output;
  }

  void stopLinear() override {
    state_.command.vx = 0.0;
    state_.command.vy = 0.0;
  }

  void resetTarget() override {}

  void reset() override { state_ = {}; }

  FollowerDiagnostics diagnostics() const override {
    return {
        true,
        FollowerAlgorithm::Trajectory,
        std::hypot(state_.command.vx, state_.command.vy),
        true,
    };
  }

 private:
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

std::unique_ptr<Algorithm> makeTrajectoryAlgorithm() {
  return std::make_unique<TrajectoryAlgorithm>();
}

const AlgorithmRegistry &builtinAlgorithms() {
  static const AlgorithmRegistry registry = [] {
    AlgorithmRegistry result;
    result.registerAlgorithm(FollowerAlgorithm::Path, &makePathAlgorithm);
    result.registerAlgorithm(FollowerAlgorithm::Trajectory, &makeTrajectoryAlgorithm);
    return result;
  }();
  return registry;
}

}  // namespace

FollowerInput::FollowerInput(const std::vector<Vec3> &path) : target(std::cref(path)) {}

FollowerInput::FollowerInput(const std::vector<TrajectoryPoint> &trajectory)
    : target(std::cref(trajectory)) {}

FollowerAlgorithm FollowerInput::algorithm() const noexcept {
  return std::holds_alternative<std::reference_wrapper<const std::vector<Vec3>>>(target)
             ? FollowerAlgorithm::Path
             : FollowerAlgorithm::Trajectory;
}

const char *followerAlgorithmName(FollowerAlgorithm algorithm) noexcept {
  switch (algorithm) {
    case FollowerAlgorithm::Path:
      return "path";
    case FollowerAlgorithm::Trajectory:
      return "trajectory";
  }
  return "path";
}

class Follower::Impl {
 public:
  Impl() {
    registerAlgorithm(FollowerAlgorithm::Path);
    registerAlgorithm(FollowerAlgorithm::Trajectory);
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

  void stopLinear() {
    for (auto &entry : algorithms_)
      entry.second->stopLinear();
  }

  void resetTarget() {
    if (active_.has_value())
      algorithms_.at(*active_)->resetTarget();
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

FollowerOutput Follower::follow(const FollowerInput &input) {
  return impl_->follow(input);
}

void Follower::stopLinear() {
  impl_->stopLinear();
}

void Follower::resetTarget() {
  impl_->resetTarget();
}

void Follower::reset() {
  impl_->reset();
}

FollowerDiagnostics Follower::diagnostics() const {
  return impl_->diagnostics();
}

}  // namespace nav_kernel
