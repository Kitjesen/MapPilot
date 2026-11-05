/**
 * tracking/follower.hpp -- ROS-free local motion tracking interface.
 *
 * Callers submit either a geometric path or an exact B-spline through the
 * same Follower::follow entry point. Algorithm selection and state ownership
 * stay inside Follower.
 *
 * The geometric branch implements the CMU-style path tracking law while its
 * caller supplies the operating profile. Executor applies the upstream Go2
 * values for CMU plans; generic LingTu paths may use different parameters.
 */
#pragma once

#include <lingtu/settings.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

class LocalPlan;

struct SplineFollowerParams {
  double timeForward{::lingtu::settings::follower::parameters::scan_follower::time_forward_s};
  double headingErrorThreshold{::lingtu::settings::follower::parameters::scan_follower::heading_error_rad};
  double positionGain{::lingtu::settings::follower::parameters::scan_follower::position_gain};
  double yawGain{::lingtu::settings::follower::parameters::scan_follower::yaw_gain};
  double maxVx{::lingtu::settings::limits::parameters::scan_follower::max_vx_mps};
  double maxVy{::lingtu::settings::limits::parameters::scan_follower::max_vy_mps};
  double maxYawRateRadS{::lingtu::settings::limits::native_nav::follower::max_yaw_rate_rad_s};
  double finishDistance{::lingtu::settings::follower::parameters::scan_follower::finish_distance_m};
};

struct CmuFollowerProfile {
  double lookahead_m{::lingtu::settings::follower::native_nav::cmu::follower_lookahead_m};
  double yaw_gain{::lingtu::settings::follower::native_nav::cmu::follower_yaw_gain};
  double nominal_dt_s{::lingtu::settings::follower::native_nav::cmu::follower_nominal_dt_s};
  double switch_time_s{::lingtu::settings::follower::native_nav::cmu::follower_switch_time_s};
  double heading_align_rad{::lingtu::settings::follower::native_nav::cmu::follower_heading_align_rad};
  double omni_goal_m{::lingtu::settings::follower::native_nav::cmu::follower_omni_goal_m};
  double omni_heading_rad{::lingtu::settings::follower::native_nav::cmu::follower_omni_heading_rad};
  double stop_distance_m{::lingtu::settings::follower::native_nav::cmu::follower_stop_distance_m};
  double slowdown_distance_m{::lingtu::settings::follower::native_nav::cmu::follower_slowdown_distance_m};
};

struct FollowerParams {
  double baseLookAheadDis{::lingtu::settings::follower::native_nav::follower::lookahead_m};
  double lookAheadRatio = 0.5;
  double minLookAheadDis = 0.2;
  double maxLookAheadDis = 2.0;
  double yawRateGain = 7.5;
  double stopYawRateGain = 7.5;
  double maxYawRateRadS{::lingtu::settings::limits::native_nav::follower::max_yaw_rate_rad_s};
  double headingAlignEnterRad{::lingtu::settings::follower::native_nav::follower::align_enter_rad};
  double headingAlignExitRad{::lingtu::settings::follower::native_nav::follower::align_exit_rad};
  double maxSpeed{::lingtu::settings::limits::native_nav::follower::max_speed_mps};
  double minSpeed{::lingtu::settings::follower::native_nav::follower::min_speed_mps};
  double maxAccel{::lingtu::settings::limits::native_nav::follower::max_accel_mps2};  // m/s^2
  double maxYawAccelRadS2{::lingtu::settings::limits::native_nav::follower::max_yaw_accel_rad_s2};  // rad/s^2; <=0 disables rate limiting
  double linearStopThreshold = 1e-4;
  double nominalDt = 0.01;
  double maxDt = 0.10;
  double turnSpeedYawRateStart = 0.0;  // rad/s; <=0 disables coupling
  double turnSpeedMinScale = 1.0;      // [0,1], applied at maxYawRateRadS
  double switchTimeThre = 1.0;
  double omniDirGoalThre = 1.0;
  double omniDirDiffThre = 1.5;  // rad, about 86 deg
  double stopDisThre{::lingtu::settings::follower::native_nav::follower::goal_tolerance_m};
  double slowDwnDisThre = 1.0;
  bool cornerGate = false;
  bool twoWayDrive = true;
  bool noRotAtGoal = true;
  SplineFollowerParams spline{};
  CmuFollowerProfile cmu{};
};

// Apply the upstream CMU pathFollower profile while preserving the Product's
// selected maximum speed and external slow/stop inputs.
FollowerParams cmuFollowerParams(FollowerParams params);
FollowerParams recoveryFollowerParams(FollowerParams params, double speed, double localRange);

enum class FollowerAlgorithm {
  Path,
  Spline,
};

const char *followerAlgorithmName(FollowerAlgorithm algorithm) noexcept;

struct FollowerState {
  Vec3 vehicleRelative{};
  double vehicleYawRelative{0.0};
  Twist measuredBodyTwist{};
  double requestedSpeed{1.0};  // normalized [0, 1]
  double currentTime{0.0};
  double slowFactor{1.0};  // speed multiplier [0, 1]
  int safetyStop{0};       // 0=none, 1=stop linear, 2=stop all
  FollowerParams params{};
  double goalDistance{-1.0};
  bool standardPathProfile{true};
  // Callers can supply body heading independently of the spline tangent.
  std::optional<double> desiredHeading{};
  // An absent observation must not be mistaken for measured standstill.
  bool measuredBodyTwistValid{false};
  // Explicit angular-velocity intent is distinct from a fixed heading target.
  std::optional<double> desiredYawRate{};
};

struct FollowerTracking {
  bool active{false};
  std::int64_t trajectoryId{0};
  double executionTimeS{0.0};
  double durationS{0.0};
  double positionErrorM{0.0};
  double headingErrorRad{0.0};
  double endDistanceM{0.0};
  bool executionFrozen{false};
  bool finished{false};
  double speedLimitMps{0.0};
};

struct FollowerOutput {
  Twist cmd{};                 // body-frame velocity command
  double directionError{0.0};  // heading error after direction correction
  double endDistance{0.0};
  double turnSpeedScale{1.0};
  bool canAccelerate{false};
  bool executionFrozen{false};
  bool directionTransition{false};
  bool finished{false};
  bool awaitingTrajectory{false};
  FollowerTracking tracking{};
};

struct FollowerDiagnostics {
  bool active{false};
  FollowerAlgorithm algorithm{FollowerAlgorithm::Path};
  double linearSpeed{0.0};
  bool forward{true};
  FollowerTracking tracking{};
};

class Follower {
 public:
  Follower();
  ~Follower();

  Follower(Follower &&) noexcept;
  Follower &operator=(Follower &&) noexcept;
  Follower(const Follower &) = delete;
  Follower &operator=(const Follower &) = delete;

  FollowerOutput follow(const LocalPlan &plan, const FollowerState &state);
  [[nodiscard]] static double turnRate(double yawError, double gain,
                                       double minRate, double maxRate);
  [[nodiscard]] static Twist rotationCommand(int direction, double rate);
  // Feed back the command accepted by final control, without advancing time.
  void commitApplied(const Twist &command);
  void stopLinear();
  void resetTarget();
  void resetIntent();
  void reset();
  [[nodiscard]] FollowerDiagnostics diagnostics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel
