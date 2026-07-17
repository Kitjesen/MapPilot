/**
 * nav_kernel/path_follower_core.hpp -- ROS-free path follower declarations.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include <vector>

namespace nav_kernel {

struct PathFollowerParams {
  double sensorOffsetX    = 0;
  double sensorOffsetY    = 0;
  double baseLookAheadDis = 0.3;
  double lookAheadRatio   = 0.5;
  double minLookAheadDis  = 0.2;
  double maxLookAheadDis  = 2.0;
  double yawRateGain      = 7.5;
  double stopYawRateGain  = 7.5;
  double maxYawRate        = 45.0;  // degrees
  double maxSpeed          = 1.0;
  double maxAccel          = 1.0;   // m/s^2
  double nominalDt         = 0.01;
  double maxDt             = 0.10;
  double turnSpeedYawRateStart = 0.0;  // rad/s; <=0 disables turn-speed coupling
  double turnSpeedMinScale = 1.0;      // [0,1], applied at maxYawRate
  double switchTimeThre    = 1.0;
  double dirDiffThre       = 0.1;   // rad, about 5.7 deg
  double omniDirGoalThre   = 1.0;
  double omniDirDiffThre   = 1.5;   // rad, about 86 deg
  double stopDisThre       = 0.2;
  double slowDwnDisThre    = 1.0;
  bool   twoWayDrive       = true;
  bool   noRotAtGoal       = true;
};

struct PathFollowerState {
  double vehicleSpeed     = 0;
  int    pathPointID      = 0;
  int    lastPathPointID  = 0;
  int    lastPathSize     = 0;
  bool   navFwd           = true;
  double switchTime       = 0;
  double lastControlTime  = -1;
};

struct PathFollowerOutput {
  Twist  cmd;          // body-frame velocity command
  double dirDiff  = 0; // heading error after two-way correction (rad)
  double endDis   = 0; // distance to path end
  double turnSpeedScale = 1.0;
  bool   canAccel = false;
};

double adaptiveLookAhead(double currentSpeed, const PathFollowerParams& p);

PathFollowerOutput computeControl(
    const Vec3& vehicleRel,
    double vehicleYawDiff,
    const std::vector<Vec3>& pathPoints,
    double joySpeed,               // normalized [0, 1]
    double currentTime,
    double slowFactor,             // speed multiplier [0, 1], 1=normal
    int safetyStop,                // 0=none, 1=stop linear, 2=stop all
    const PathFollowerParams& p,
    PathFollowerState& state,
    double goalDistance = -1.0);

}  // namespace nav_kernel
