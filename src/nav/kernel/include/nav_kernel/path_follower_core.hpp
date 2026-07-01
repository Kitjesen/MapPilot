/**
 * nav_kernel/path_follower_core.hpp -- ROS-free path follower core.
 *
 * Owns pure pursuit style tracking math:
 *   - adaptiveLookAhead()
 *   - computeControl()
 *
 * Runtime modules own transport, frames, and message packing.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include "nav_kernel/validation.hpp"
#include <algorithm>
#include <cmath>
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
  double maxAccel          = 1.0;
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
};

struct PathFollowerOutput {
  Twist  cmd;          // body-frame velocity command
  double dirDiff  = 0; // heading error after two-way correction (rad)
  double endDis   = 0; // distance to path end
  double turnSpeedScale = 1.0;
  bool   canAccel = false;
};

inline double adaptiveLookAhead(double currentSpeed,
                                const PathFollowerParams& p) {
  double la = p.baseLookAheadDis + p.lookAheadRatio * std::fabs(currentSpeed);
  if (la < p.minLookAheadDis) la = p.minLookAheadDis;
  if (la > p.maxLookAheadDis) la = p.maxLookAheadDis;
  return la;
}

inline PathFollowerOutput computeControl(
    const Vec3& vehicleRel,
    double vehicleYawDiff,
    const std::vector<Vec3>& pathPoints,
    double joySpeed,               // normalized [0, 1]
    double currentTime,
    double slowFactor,             // speed multiplier [0, 1], 1=normal
    int safetyStop,                // 0=none, 1=stop linear, 2=stop all
    const PathFollowerParams& p,
    PathFollowerState& state)
{
  PathFollowerOutput out;

  if (!std::isfinite(vehicleRel.x) || !std::isfinite(vehicleRel.y) ||
      !std::isfinite(vehicleYawDiff)) {
    return out;
  }

  const int pathSize = static_cast<int>(pathPoints.size());
  if (pathSize == 0) return out;

  double endDisX = pathPoints[pathSize - 1].x - vehicleRel.x;
  double endDisY = pathPoints[pathSize - 1].y - vehicleRel.y;
  double endDis  = std::sqrt(endDisX * endDisX + endDisY * endDisY);
  out.endDis = endDis;

  double lookAheadDis = adaptiveLookAhead(state.vehicleSpeed, p);

  // Reuse the previous target index so tracking is stable and O(local path tail).
  if (pathSize != state.lastPathSize) {
    state.lastPathPointID = 0;
    state.lastPathSize = pathSize;
  }
  state.pathPointID = (state.lastPathPointID > 2)
                    ? (state.lastPathPointID - 2) : 0;

  double disX = 0, disY = 0;
  double lookAheadSq = lookAheadDis * lookAheadDis;
  while (state.pathPointID < pathSize - 1) {
    disX = pathPoints[state.pathPointID].x - vehicleRel.x;
    disY = pathPoints[state.pathPointID].y - vehicleRel.y;
    if (disX * disX + disY * disY >= lookAheadSq) break;
    state.pathPointID++;
  }
  if (state.pathPointID >= pathSize - 1) {
    disX = pathPoints[pathSize - 1].x - vehicleRel.x;
    disY = pathPoints[pathSize - 1].y - vehicleRel.y;
  }
  state.lastPathPointID = state.pathPointID;

  double dis = std::sqrt(disX * disX + disY * disY);
  double pathDir = std::atan2(disY, disX);

  double dirDiff = vehicleYawDiff - pathDir;
  dirDiff = std::fmod(dirDiff + M_PI, 2.0 * M_PI);
  if (dirDiff < 0) dirDiff += 2.0 * M_PI;
  dirDiff -= M_PI;

  // Two-way drive avoids a full turn when reversing is cheaper.
  if (p.twoWayDrive) {
    constexpr double kHysteresis = 0.1;
    if (std::fabs(dirDiff) > M_PI / 2 + kHysteresis
        && state.navFwd && currentTime - state.switchTime > p.switchTimeThre) {
      state.navFwd = false;
      state.switchTime = currentTime;
    } else if (std::fabs(dirDiff) < M_PI / 2 - kHysteresis
               && !state.navFwd && currentTime - state.switchTime > p.switchTimeThre) {
      state.navFwd = true;
      state.switchTime = currentTime;
    }
  }

  double joySpeed2 = p.maxSpeed * joySpeed;
  if (!state.navFwd) {
    dirDiff += M_PI;
    if (dirDiff > M_PI) dirDiff -= 2 * M_PI;
    joySpeed2 *= -1;
  }
  out.dirDiff = dirDiff;

  double vehicleYawRate;
  if (std::fabs(state.vehicleSpeed) < 2.0 * p.maxAccel / 100.0) {
    vehicleYawRate = -p.stopYawRateGain * dirDiff;
  } else {
    vehicleYawRate = -p.yawRateGain * dirDiff;
  }
  double maxYawRateRad = p.maxYawRate * M_PI / 180.0;
  if (vehicleYawRate >  maxYawRateRad) vehicleYawRate =  maxYawRateRad;
  if (vehicleYawRate < -maxYawRateRad) vehicleYawRate = -maxYawRateRad;

  if (pathSize <= 1 || (dis < p.stopDisThre && p.noRotAtGoal)) {
    vehicleYawRate = 0;
  }

  double turnSpeedScale = 1.0;
  if (p.turnSpeedYawRateStart > 0.0 &&
      p.turnSpeedMinScale < 1.0 &&
      maxYawRateRad > p.turnSpeedYawRateStart) {
    double minScale = p.turnSpeedMinScale;
    if (minScale < 0.0) minScale = 0.0;
    if (minScale > 1.0) minScale = 1.0;
    double ratio = (std::fabs(vehicleYawRate) - p.turnSpeedYawRateStart)
                 / (maxYawRateRad - p.turnSpeedYawRateStart);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    turnSpeedScale = 1.0 - (1.0 - minScale) * ratio;
  }
  out.turnSpeedScale = turnSpeedScale;

  if (pathSize <= 1) {
    joySpeed2 = 0;
  } else if (endDis / p.slowDwnDisThre < joySpeed) {
    joySpeed2 *= endDis / p.slowDwnDisThre;
  }

  double joySpeed3 = joySpeed2 * slowFactor * turnSpeedScale;

  auto stepToward = [&](double cur, double tgt) {
    double step = p.maxAccel / 100.0;
    return (cur < tgt) ? std::min(cur + step, tgt)
         : (cur > tgt) ? std::max(cur - step, tgt) : cur;
  };

  bool canAccel = (std::fabs(dirDiff) < p.dirDiffThre ||
                   (dis < p.omniDirGoalThre && std::fabs(dirDiff) < p.omniDirDiffThre))
                  && dis > p.stopDisThre;
  out.canAccel = canAccel;
  state.vehicleSpeed = canAccel ? stepToward(state.vehicleSpeed, joySpeed3)
                                : stepToward(state.vehicleSpeed, 0.0);

  if (safetyStop >= 1) state.vehicleSpeed = 0;
  if (safetyStop >= 2) vehicleYawRate = 0;

  out.cmd.wz = vehicleYawRate;
  if (std::fabs(state.vehicleSpeed) > p.maxAccel / 100.0) {
    if (p.omniDirGoalThre > 0) {
#if defined(__GLIBC__) || defined(__APPLE__)
      double sinD, cosD;
      sincos(dirDiff, &sinD, &cosD);
      out.cmd.vx =  cosD * state.vehicleSpeed;
      out.cmd.vy = -sinD * state.vehicleSpeed;
#else
      out.cmd.vx =  std::cos(dirDiff) * state.vehicleSpeed;
      out.cmd.vy = -std::sin(dirDiff) * state.vehicleSpeed;
#endif
    } else {
      out.cmd.vx = state.vehicleSpeed;
    }
  }

  return out;
}

}  // namespace nav_kernel
