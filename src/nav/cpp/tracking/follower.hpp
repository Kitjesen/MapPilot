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

#include <functional>
#include <memory>
#include <variant>
#include <vector>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

class LocalPlan;

struct SplineFollowerParams {
  double timeForward = 0.55;
  double headingErrorThreshold = 0.8;
  double positionGain = 0.9;
  double yawGain = 1.2;
  double maxVx = 0.5;
  double maxVy = 0.25;
  double maxYawRateRadS = 1.0;
  double finishDistance = 0.15;
};

struct FollowerParams {
  double baseLookAheadDis = 0.3;
  double lookAheadRatio = 0.5;
  double minLookAheadDis = 0.2;
  double maxLookAheadDis = 2.0;
  double yawRateGain = 7.5;
  double stopYawRateGain = 7.5;
  double maxYawRateRadS = 0.7853981633974483;
  double headingAlignEnterRad = 0.7853981633974483;
  double headingAlignExitRad = 0.35;
  double maxSpeed = 1.0;
  double minSpeed = 0.0;
  double maxAccel = 1.0;  // m/s^2
  double maxYawAccelRadS2 = 0.0;  // rad/s^2; <=0 disables rate limiting
  double linearStopThreshold = 1e-4;
  double nominalDt = 0.01;
  double maxDt = 0.10;
  double turnSpeedYawRateStart = 0.0;  // rad/s; <=0 disables coupling
  double turnSpeedMinScale = 1.0;      // [0,1], applied at maxYawRateRadS
  double switchTimeThre = 1.0;
  double omniDirGoalThre = 1.0;
  double omniDirDiffThre = 1.5;  // rad, about 86 deg
  double stopDisThre = 0.2;
  double slowDwnDisThre = 1.0;
  bool cornerGate = false;
  bool twoWayDrive = true;
  bool noRotAtGoal = true;
  SplineFollowerParams spline{};
};

// Apply the upstream CMU pathFollower profile while preserving the Product's
// selected maximum speed and external slow/stop inputs.
FollowerParams cmuFollowerParams(FollowerParams params);

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
  bool holdBodyHeading{false};
  bool standardPathProfile{true};
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
};

struct FollowerDiagnostics {
  bool active{false};
  FollowerAlgorithm algorithm{FollowerAlgorithm::Path};
  double linearSpeed{0.0};
  bool forward{true};
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
