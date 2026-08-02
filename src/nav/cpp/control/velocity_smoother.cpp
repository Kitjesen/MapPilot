#include "nav_kernel/velocity_smoother.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace nav_kernel {
namespace {

constexpr double kEpsilon = 1e-9;

void SetError(std::string *error, const std::string &value) {
  if (error != nullptr) {
    *error = value;
  }
}

bool Finite(double value) {
  return std::isfinite(value);
}

bool Finite(const Twist &value) {
  return Finite(value.vx) && Finite(value.vy) && Finite(value.wz);
}

bool ValidateAxis(const VelocityAxisLimits &limits, const char *name, std::string *error) {
  if (!Finite(limits.minimum) || !Finite(limits.maximum) || !Finite(limits.acceleration) ||
      !Finite(limits.deceleration) || !Finite(limits.deadband)) {
    SetError(error, std::string(name) + "_limits_nonfinite");
    return false;
  }
  if (limits.minimum > 0.0 || limits.maximum < 0.0 || limits.minimum > limits.maximum) {
    SetError(error, std::string(name) + "_limits_must_contain_zero");
    return false;
  }
  if (limits.acceleration <= 0.0 || limits.deceleration <= 0.0) {
    SetError(error, std::string(name) + "_rate_limit_nonpositive");
    return false;
  }
  const double magnitude = std::max(std::abs(limits.minimum), std::abs(limits.maximum));
  if (limits.deadband < 0.0 || limits.deadband > magnitude) {
    SetError(error, std::string(name) + "_deadband_invalid");
    return false;
  }
  return true;
}

double Clamp(double value, const VelocityAxisLimits &limits) {
  return std::clamp(value, limits.minimum, limits.maximum);
}

double ApplyDeadband(double value, const VelocityAxisLimits &limits) {
  return std::abs(value) < limits.deadband ? 0.0 : value;
}

double LimitAxis(double current, double target, const VelocityAxisLimits &limits, double dt) {
  const bool same_direction = std::abs(current) <= kEpsilon || std::abs(target) <= kEpsilon ||
                              std::signbit(current) == std::signbit(target);
  const bool speeding_up = same_direction && std::abs(target) > std::abs(current);
  const double rate = speeding_up ? limits.acceleration : limits.deceleration;
  const double max_delta = rate * dt;
  const double delta = std::clamp(target - current, -max_delta, max_delta);
  double next = current + delta;

  // A reversal first decelerates to zero. It cannot jump through zero in one
  // control step even when the configured deceleration is large.
  if (!same_direction && std::abs(current) > kEpsilon &&
      std::signbit(next) != std::signbit(current)) {
    next = 0.0;
  }
  return Clamp(next, limits);
}

bool Different(double a, double b) {
  return std::abs(a - b) > kEpsilon;
}

}  // namespace

VelocitySmoother::VelocitySmoother() {
  std::string error;
  configured_ = Configure(VelocitySmootherConfig{}, &error);
}

VelocitySmoother::VelocitySmoother(const VelocitySmootherConfig &config) {
  std::string error;
  if (!Configure(config, &error)) {
    throw std::invalid_argument(error);
  }
}

bool VelocitySmoother::Configure(const VelocitySmootherConfig &config, std::string *error) {
  if (!ValidateAxis(config.x, "x", error) || !ValidateAxis(config.y, "y", error) ||
      !ValidateAxis(config.yaw, "yaw", error)) {
    return false;
  }
  if (!Finite(config.target_timeout_s) || config.target_timeout_s <= 0.0) {
    SetError(error, "target_timeout_invalid");
    return false;
  }
  if (!Finite(config.feedback_timeout_s) || config.feedback_timeout_s <= 0.0) {
    SetError(error, "feedback_timeout_invalid");
    return false;
  }
  if (!Finite(config.max_step_s) || config.max_step_s <= 0.0) {
    SetError(error, "max_step_invalid");
    return false;
  }
  if (!Finite(config.future_tolerance_s) || config.future_tolerance_s < 0.0) {
    SetError(error, "future_tolerance_invalid");
    return false;
  }

  config_ = config;
  configured_ = true;
  Reset();
  SetError(error, std::string{});
  return true;
}

bool VelocitySmoother::SetTarget(const Twist &target, double timestamp_s, std::string *error) {
  if (!configured_) {
    SetError(error, "not_configured");
    return false;
  }
  if (!Finite(target) || !Finite(timestamp_s) || timestamp_s < 0.0) {
    SetError(error, "target_invalid");
    return false;
  }
  if (has_target_ && timestamp_s < target_stamp_s_) {
    SetError(error, "target_out_of_order");
    return false;
  }
  target_ = target;
  target_stamp_s_ = timestamp_s;
  has_target_ = true;
  SetError(error, std::string{});
  return true;
}

bool VelocitySmoother::SetFeedback(const Twist &velocity, double timestamp_s, std::string *error) {
  if (!configured_) {
    SetError(error, "not_configured");
    return false;
  }
  if (!Finite(velocity) || !Finite(timestamp_s) || timestamp_s < 0.0) {
    SetError(error, "feedback_invalid");
    return false;
  }
  if (has_feedback_ && timestamp_s < feedback_stamp_s_) {
    SetError(error, "feedback_out_of_order");
    return false;
  }
  feedback_ = velocity;
  feedback_stamp_s_ = timestamp_s;
  has_feedback_ = true;
  SetError(error, std::string{});
  return true;
}

VelocitySmootherOutput VelocitySmoother::Step(double now_s) {
  VelocitySmootherOutput output;
  output.command = {};

  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }
  if (!Finite(now_s) || now_s < 0.0) {
    Reset();
    output.reason = "time_invalid";
    return output;
  }
  if (!has_target_) {
    last_output_ = {};
    last_step_s_ = now_s;
    has_last_step_ = true;
    output.valid = true;
    output.timed_out = true;
    output.reason = "target_missing";
    return output;
  }

  const double target_age_s = now_s - target_stamp_s_;
  if (target_age_s < -config_.future_tolerance_s) {
    Reset();
    output.reason = "target_timestamp_future";
    return output;
  }
  if (target_age_s > config_.target_timeout_s) {
    last_output_ = {};
    last_step_s_ = now_s;
    has_last_step_ = true;
    output.valid = true;
    output.timed_out = true;
    output.reason = "target_timeout";
    return output;
  }

  Twist base = last_output_;
  if (config_.feedback_mode == VelocityFeedbackMode::kClosedLoop) {
    if (!has_feedback_) {
      last_output_ = {};
      last_step_s_ = now_s;
      has_last_step_ = true;
      output.reason = "feedback_missing";
      return output;
    }
    const double feedback_age_s = now_s - feedback_stamp_s_;
    if (feedback_age_s < -config_.future_tolerance_s) {
      last_output_ = {};
      last_step_s_ = now_s;
      has_last_step_ = true;
      output.reason = "feedback_timestamp_future";
      return output;
    }
    if (feedback_age_s > config_.feedback_timeout_s) {
      last_output_ = {};
      last_step_s_ = now_s;
      has_last_step_ = true;
      output.reason = "feedback_timeout";
      return output;
    }
    base = feedback_;
    output.feedback_used = true;
  }

  base.vx = Clamp(base.vx, config_.x);
  base.vy = Clamp(base.vy, config_.y);
  base.wz = Clamp(base.wz, config_.yaw);

  if (!has_last_step_) {
    last_output_ = base;
    last_step_s_ = now_s;
    has_last_step_ = true;
    output.command = base;
    output.valid = true;
    output.reason = "initialized";
    return output;
  }
  if (now_s < last_step_s_) {
    Reset();
    output.reason = "time_reversed";
    return output;
  }
  if (now_s == last_step_s_) {
    output.command = last_output_;
    output.valid = true;
    output.feedback_used = config_.feedback_mode == VelocityFeedbackMode::kClosedLoop;
    output.reason = "no_time_advance";
    return output;
  }

  const double dt = std::min(now_s - last_step_s_, config_.max_step_s);
  Twist bounded_target{
      Clamp(target_.vx, config_.x),
      Clamp(target_.vy, config_.y),
      Clamp(target_.wz, config_.yaw),
  };
  Twist candidate{
      LimitAxis(base.vx, bounded_target.vx, config_.x, dt),
      LimitAxis(base.vy, bounded_target.vy, config_.y, dt),
      LimitAxis(base.wz, bounded_target.wz, config_.yaw, dt),
  };

  if (config_.scale_velocities) {
    const std::array<double, 3> current{base.vx, base.vy, base.wz};
    const std::array<double, 3> desired{bounded_target.vx, bounded_target.vy, bounded_target.wz};
    const std::array<double, 3> limited{candidate.vx, candidate.vy, candidate.wz};
    double common_fraction = 1.0;
    bool has_motion = false;
    for (std::size_t i = 0; i < current.size(); ++i) {
      const double requested_delta = desired[i] - current[i];
      if (std::abs(requested_delta) <= kEpsilon) {
        continue;
      }
      has_motion = true;
      const double fraction = std::abs((limited[i] - current[i]) / requested_delta);
      common_fraction = std::min(common_fraction, std::clamp(fraction, 0.0, 1.0));
    }
    if (has_motion) {
      candidate.vx = base.vx + common_fraction * (bounded_target.vx - base.vx);
      candidate.vy = base.vy + common_fraction * (bounded_target.vy - base.vy);
      candidate.wz = base.wz + common_fraction * (bounded_target.wz - base.wz);
    }
  }

  candidate.vx = ApplyDeadband(Clamp(candidate.vx, config_.x), config_.x);
  candidate.vy = ApplyDeadband(Clamp(candidate.vy, config_.y), config_.y);
  candidate.wz = ApplyDeadband(Clamp(candidate.wz, config_.yaw), config_.yaw);

  output.command = candidate;
  output.valid = true;
  output.limited = Different(candidate.vx, target_.vx) || Different(candidate.vy, target_.vy) ||
                   Different(candidate.wz, target_.wz);
  output.reason = output.limited ? "limited" : "target_reached";

  last_output_ = candidate;
  last_step_s_ = now_s;
  return output;
}

VelocitySmootherOutput VelocitySmoother::Stop(double now_s, const std::string &reason) {
  Reset();
  if (Finite(now_s) && now_s >= 0.0) {
    last_step_s_ = now_s;
    has_last_step_ = true;
  }
  VelocitySmootherOutput output;
  output.command = {};
  output.valid = Finite(now_s) && now_s >= 0.0;
  output.reason = reason;
  return output;
}

void VelocitySmoother::Reset() {
  target_ = {};
  target_stamp_s_ = 0.0;
  has_target_ = false;
  feedback_ = {};
  feedback_stamp_s_ = 0.0;
  has_feedback_ = false;
  last_output_ = {};
  last_step_s_ = 0.0;
  has_last_step_ = false;
}

}  // namespace nav_kernel
