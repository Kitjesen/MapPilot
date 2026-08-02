#pragma once

#include <string>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

enum class VelocityFeedbackMode {
  kOpenLoop,
  kClosedLoop,
};

struct VelocityAxisLimits {
  double minimum{-1.0};
  double maximum{1.0};
  double acceleration{1.0};
  double deceleration{2.0};
  double deadband{0.0};
};

struct VelocitySmootherConfig {
  VelocityAxisLimits x{-1.0, 1.0, 1.0, 2.0, 0.0};
  VelocityAxisLimits y{-0.5, 0.5, 1.0, 2.0, 0.0};
  VelocityAxisLimits yaw{-1.5, 1.5, 2.0, 3.0, 0.0};
  VelocityFeedbackMode feedback_mode{VelocityFeedbackMode::kOpenLoop};
  double target_timeout_s{0.5};
  double feedback_timeout_s{0.25};
  double max_step_s{0.2};
  double future_tolerance_s{0.05};
  bool scale_velocities{false};
};

struct VelocitySmootherOutput {
  Twist command{};
  bool valid{false};
  bool limited{false};
  bool timed_out{false};
  bool feedback_used{false};
  std::string reason{"not_started"};
};

// Portable command shaping only. Collision checking must run after this core so
// a safety stop is never softened or delayed by acceleration limits.
class VelocitySmoother {
 public:
  VelocitySmoother();
  explicit VelocitySmoother(const VelocitySmootherConfig &config);

  bool Configure(const VelocitySmootherConfig &config, std::string *error = nullptr);
  bool SetTarget(const Twist &target, double timestamp_s, std::string *error = nullptr);
  bool SetFeedback(const Twist &velocity, double timestamp_s, std::string *error = nullptr);

  VelocitySmootherOutput Step(double now_s);

  // Emergency, cancellation, and authority-loss paths use Stop(), bypassing
  // gradual deceleration. The next accepted target starts from zero.
  VelocitySmootherOutput Stop(double now_s, const std::string &reason = "stopped");
  void Reset();

  const VelocitySmootherConfig &config() const { return config_; }
  const Twist &last_output() const { return last_output_; }

 private:
  VelocitySmootherConfig config_{};
  bool configured_{false};

  Twist target_{};
  double target_stamp_s_{0.0};
  bool has_target_{false};

  Twist feedback_{};
  double feedback_stamp_s_{0.0};
  bool has_feedback_{false};

  Twist last_output_{};
  double last_step_s_{0.0};
  bool has_last_step_{false};
};

}  // namespace nav_kernel
