#include "nav_kernel/path_follower_core.hpp"

#include <algorithm>
#include <cmath>

namespace nav_kernel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedEpsilon = 1e-4;

double wrapPi(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0) angle += 2.0 * kPi;
  return angle - kPi;
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

}  // namespace

double adaptiveLookAhead(double currentSpeed, const PathFollowerParams& p) {
  double la = p.baseLookAheadDis + p.lookAheadRatio * std::fabs(currentSpeed);
  return clamp(la, p.minLookAheadDis, p.maxLookAheadDis);
}

PathFollowerOutput computeControl(
    const Vec3& vehicleRel,
    double vehicleYawDiff,
    const std::vector<Vec3>& pathPoints,
    double joySpeed,
    double currentTime,
    double slowFactor,
    int safetyStop,
    const PathFollowerParams& p,
    PathFollowerState& state,
    double goalDistance) {
  PathFollowerOutput out;

  if (!std::isfinite(vehicleRel.x) || !std::isfinite(vehicleRel.y) ||
      !std::isfinite(vehicleYawDiff)) {
    return out;
  }

  const int pathSize = static_cast<int>(pathPoints.size());
  if (pathSize == 0) return out;

  const double endDisX = pathPoints[pathSize - 1].x - vehicleRel.x;
  const double endDisY = pathPoints[pathSize - 1].y - vehicleRel.y;
  const double localEndDis = std::sqrt(endDisX * endDisX + endDisY * endDisY);
  const double endDis = std::isfinite(goalDistance) && goalDistance >= 0.0
      ? goalDistance
      : localEndDis;
  out.endDis = endDis;

  const double lookAheadDis = adaptiveLookAhead(state.vehicleSpeed, p);
  if (pathSize != state.lastPathSize) {
    state.lastPathPointID = 0;
    state.lastPathSize = pathSize;
  }
  state.pathPointID = (state.lastPathPointID > 2)
                    ? (state.lastPathPointID - 2) : 0;

  double disX = 0;
  double disY = 0;
  const double lookAheadSq = lookAheadDis * lookAheadDis;
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

  const double dis = std::sqrt(disX * disX + disY * disY);
  const double pathDir = std::atan2(disY, disX);
  double dirDiff = wrapPi(vehicleYawDiff - pathDir);

  const double nominalDt = clamp(p.nominalDt, 1e-4, 1.0);
  const double maxDt = std::max(nominalDt, p.maxDt);
  double dt = nominalDt;
  if (std::isfinite(currentTime) && state.lastControlTime >= 0.0 &&
      currentTime > state.lastControlTime) {
    dt = clamp(currentTime - state.lastControlTime, 1e-4, maxDt);
  }
  if (std::isfinite(currentTime)) {
    state.lastControlTime = currentTime;
  }
  const double accelStep = std::max(0.0, p.maxAccel) * dt;
  const double nominalAccelStep = std::max(0.0, p.maxAccel) * nominalDt;

  if (p.twoWayDrive) {
    constexpr double kHysteresis = 0.1;
    if (std::fabs(dirDiff) > kPi / 2.0 + kHysteresis &&
        state.navFwd && currentTime - state.switchTime > p.switchTimeThre) {
      state.navFwd = false;
      state.switchTime = currentTime;
    } else if (std::fabs(dirDiff) < kPi / 2.0 - kHysteresis &&
               !state.navFwd && currentTime - state.switchTime > p.switchTimeThre) {
      state.navFwd = true;
      state.switchTime = currentTime;
    }
  }

  double targetSpeed = p.maxSpeed * joySpeed;
  if (!state.navFwd) {
    dirDiff = wrapPi(dirDiff + kPi);
    targetSpeed *= -1.0;
  }
  out.dirDiff = dirDiff;

  double vehicleYawRate = -(
      std::fabs(state.vehicleSpeed) < 2.0 * nominalAccelStep
      ? p.stopYawRateGain
      : p.yawRateGain) * dirDiff;
  const double maxYawRateRad = p.maxYawRate * kPi / 180.0;
  vehicleYawRate = clamp(vehicleYawRate, -maxYawRateRad, maxYawRateRad);

  if (pathSize <= 1 || (dis < p.stopDisThre && p.noRotAtGoal)) {
    vehicleYawRate = 0;
  }

  double turnSpeedScale = 1.0;
  if (p.turnSpeedYawRateStart > 0.0 &&
      p.turnSpeedMinScale < 1.0 &&
      maxYawRateRad > p.turnSpeedYawRateStart) {
    const double minScale = clamp(p.turnSpeedMinScale, 0.0, 1.0);
    const double ratio = clamp(
        (std::fabs(vehicleYawRate) - p.turnSpeedYawRateStart)
            / (maxYawRateRad - p.turnSpeedYawRateStart),
        0.0,
        1.0);
    turnSpeedScale = 1.0 - (1.0 - minScale) * ratio;
  }
  out.turnSpeedScale = turnSpeedScale;

  if (pathSize <= 1) {
    targetSpeed = 0;
  } else if (endDis / p.slowDwnDisThre < joySpeed) {
    targetSpeed *= endDis / p.slowDwnDisThre;
  }

  const double commandedSpeed = targetSpeed * slowFactor * turnSpeedScale;
  auto stepToward = [&](double cur, double tgt) {
    return (cur < tgt) ? std::min(cur + accelStep, tgt)
         : (cur > tgt) ? std::max(cur - accelStep, tgt) : cur;
  };

  const bool canAccel = (std::fabs(dirDiff) < p.dirDiffThre ||
                         (dis < p.omniDirGoalThre &&
                          std::fabs(dirDiff) < p.omniDirDiffThre)) &&
                        dis > p.stopDisThre;
  out.canAccel = canAccel;
  state.vehicleSpeed = canAccel ? stepToward(state.vehicleSpeed, commandedSpeed)
                                : stepToward(state.vehicleSpeed, 0.0);

  if (safetyStop >= 1) state.vehicleSpeed = 0;
  if (safetyStop >= 2) vehicleYawRate = 0;

  out.cmd.wz = vehicleYawRate;
  if (std::fabs(state.vehicleSpeed) > kSpeedEpsilon) {
    if (p.omniDirGoalThre > 0) {
      out.cmd.vx = std::cos(dirDiff) * state.vehicleSpeed;
      out.cmd.vy = -std::sin(dirDiff) * state.vehicleSpeed;
    } else {
      out.cmd.vx = state.vehicleSpeed;
    }
  }

  return out;
}

}  // namespace nav_kernel
