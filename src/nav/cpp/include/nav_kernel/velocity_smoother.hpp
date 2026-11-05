#pragma once

#include <lingtu/settings.hpp>

#include <string>
#include <string_view>

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
  double minimum{-::lingtu::settings::teleop::native_nav::teleop::max_speed_mps};
  double maximum{::lingtu::settings::teleop::native_nav::teleop::max_speed_mps};
  double acceleration{::lingtu::settings::limits::native_nav::follower::max_accel_mps2};
  double deceleration{::lingtu::settings::limits::native_nav::braking_deceleration_mps2};
  double deadband{::lingtu::settings::control::native_nav::smoother_x_deadband_mps};
};

struct VelocitySmootherConfig {
  VelocityAxisLimits x{};
  VelocityAxisLimits y{-::lingtu::settings::teleop::native_nav::teleop::max_speed_mps, ::lingtu::settings::teleop::native_nav::teleop::max_speed_mps,
      ::lingtu::settings::limits::native_nav::follower::max_accel_mps2, ::lingtu::settings::limits::native_nav::braking_deceleration_mps2, ::lingtu::settings::control::native_nav::smoother_y_deadband_mps};
  VelocityAxisLimits yaw{-::lingtu::settings::limits::native_nav::follower::max_yaw_rate_rad_s, ::lingtu::settings::limits::native_nav::follower::max_yaw_rate_rad_s,
      ::lingtu::settings::limits::native_nav::follower::max_yaw_accel_rad_s2, ::lingtu::settings::limits::native_nav::follower::max_yaw_accel_rad_s2, ::lingtu::settings::control::native_nav::smoother_yaw_deadband_rad_s};
  VelocityFeedbackMode feedback_mode{
      std::string_view(::lingtu::settings::control::native_nav::smoother_feedback_mode) == "closed"
          ? VelocityFeedbackMode::kClosedLoop
          : VelocityFeedbackMode::kOpenLoop};
  double target_timeout_s{::lingtu::settings::control::native_nav::smoother_target_timeout_s};     // Seconds before a stale target fails closed.
  double feedback_timeout_s{::lingtu::settings::control::native_nav::smoother_feedback_timeout_s};  // Seconds before stale feedback fails closed.
  double max_step_s{::lingtu::settings::control::native_nav::smoother_max_step_s};           // Upper bound in seconds applied to one step dt.
  double future_tolerance_s{::lingtu::settings::control::native_nav::smoother_future_tolerance_s};  // Seconds a timestamp may lead ahead of now.
  bool scale_velocities{::lingtu::settings::control::native_nav::smoother_scale_velocities};
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
