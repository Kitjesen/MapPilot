#pragma once

#include <string>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

enum class VelocityFeedbackMode {
  kOpenLoop,
  kClosedLoop,
};

// Per-axis command limits. Velocities are m/s, yaw rate is rad/s, and rate
// limits are m/s^2 (rad/s^2 for yaw). Deadband uses the same unit as the
// axis velocity.
struct VelocityAxisLimits {
  double minimum{-1.0};   // Lower bound in m/s (rad/s for yaw).
  double maximum{1.0};    // Upper bound in m/s (rad/s for yaw).
  double acceleration{1.0};   // Ramp-up rate limit in m/s^2 (rad/s^2).
  double deceleration{2.0};   // Ramp-down/reversal rate limit, same unit.
  double deadband{0.0};       // Outputs below this magnitude become zero.
};

struct VelocitySmootherConfig {
  VelocityAxisLimits x{-1.0, 1.0, 1.0, 2.0, 0.0};
  VelocityAxisLimits y{-0.5, 0.5, 1.0, 2.0, 0.0};
  VelocityAxisLimits yaw{-1.5, 1.5, 2.0, 3.0, 0.0};
  VelocityFeedbackMode feedback_mode{VelocityFeedbackMode::kOpenLoop};
  double target_timeout_s{0.5};     // Seconds before a stale target fails closed.
  double feedback_timeout_s{0.25};  // Seconds before stale feedback fails closed.
  double max_step_s{0.2};           // Upper bound in seconds applied to one step dt.
  double future_tolerance_s{0.05};  // Seconds a timestamp may lead ahead of now.
  bool scale_velocities{false};
};

struct VelocitySmootherOutput {
  Twist command{};
  // True when this step produced a meaningful command; false on fail-closed
  // rejection paths where the command is forced to zero.
  bool valid{false};
  // True when the candidate differs from the raw target. Under
  // scale_velocities it reflects per-axis limit comparisons and does not
  // imply the whole command was proportionally scaled.
  bool limited{false};
  // Staleness loss of authority: the target timed out or feedback became
  // unusable (missing, future, or stale), so the output failed closed to
  // zero.
  bool timed_out{false};
  // True only when this step actually ramped from the closed-loop feedback
  // measurement.
  bool feedback_used{false};
  // True when the elapsed interval exceeded max_step_s and the ramp delta
  // was computed with dt clamped to max_step_s.
  bool dt_clamped{false};
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
  bool CommitApplied(const Twist &applied, double timestamp_s, std::string *error = nullptr);

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
