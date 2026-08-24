/**
 * tracking/follower.hpp -- ROS-free local motion tracking interface.
 *
 * Callers submit either a geometric path or a timed trajectory through the
 * same Follower::follow entry point. Algorithm selection and state ownership
 * stay inside Follower.
 */
#pragma once

#include <functional>
#include <memory>
#include <variant>
#include <vector>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

struct FollowerParams {
  double baseLookAheadDis = 0.3;
  double lookAheadRatio = 0.5;
  double minLookAheadDis = 0.2;
  double maxLookAheadDis = 2.0;
  double yawRateGain = 7.5;
  double stopYawRateGain = 7.5;
  double maxYawRate = 45.0;  // degrees
  double maxSpeed = 1.0;
  double minSpeed = 0.0;
  double maxAccel = 1.0;  // m/s^2
  double nominalDt = 0.01;
  double maxDt = 0.10;
  double turnSpeedYawRateStart = 0.0;  // rad/s; <=0 disables coupling
  double turnSpeedMinScale = 1.0;      // [0,1], applied at maxYawRate
  double switchTimeThre = 1.0;
  double dirDiffThre = 0.1;  // rad, about 5.7 deg
  double omniDirGoalThre = 1.0;
  double omniDirDiffThre = 1.5;  // rad, about 86 deg
  double stopDisThre = 0.2;
  double slowDwnDisThre = 1.0;
  bool twoWayDrive = true;
  bool noRotAtGoal = true;
  double trajectoryLookAheadS = 0.8;
  double trajectoryPositionGain = 0.8;
  double trajectoryYawGain = 1.5;
  double trajectoryHeadingErrorThreshold = 0.8;
};

enum class FollowerAlgorithm {
  Path,
  Trajectory,
};

const char *followerAlgorithmName(FollowerAlgorithm algorithm) noexcept;

using FollowerTarget = std::variant<std::reference_wrapper<const std::vector<Vec3>>,
                                    std::reference_wrapper<const std::vector<TrajectoryPoint>>>;

struct FollowerInput {
  explicit FollowerInput(const std::vector<Vec3> &path);
  explicit FollowerInput(const std::vector<TrajectoryPoint> &trajectory);

  [[nodiscard]] FollowerAlgorithm algorithm() const noexcept;

  FollowerTarget target;
  Vec3 vehicleRelative{};
  double vehicleYawRelative{0.0};
  Twist measuredBodyTwist{};
  double requestedSpeed{1.0};  // normalized [0, 1]
  double currentTime{0.0};
  double slowFactor{1.0};  // speed multiplier [0, 1]
  int safetyStop{0};       // 0=none, 1=stop linear, 2=stop all
  FollowerParams params{};
  double goalDistance{-1.0};
};

struct FollowerOutput {
  Twist cmd{};                 // body-frame velocity command
  double directionError{0.0};  // heading error after direction correction
  double endDistance{0.0};
  double turnSpeedScale{1.0};
  bool canAccelerate{false};
  bool executionFrozen{false};
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

  FollowerOutput follow(const FollowerInput &input);
  void stopLinear();
  void resetTarget();
  void reset();
  [[nodiscard]] FollowerDiagnostics diagnostics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel
