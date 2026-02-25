/**
 * nav_core/path_follower_core.hpp — pathFollower.cpp 纯算法提取
 *
 * 从 src/base_autonomy/local_planner/src/pathFollower.cpp 提取:
 *   - adaptiveLookAhead()
 *   - findLookAheadPoint()
 *   - computeControl() — dirDiff, canAccel, omni 速度分解
 *
 * 零 ROS2 依赖。ROS2 Shell 在边界处做类型转换。
 */
#pragma once

#include "nav_core/types.hpp"
#include <cmath>
#include <vector>

namespace nav_core {

// ── 参数 (对应 pathFollower.cpp 的 ROS2 参数) ──

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
  double switchTimeThre    = 1.0;
  double dirDiffThre       = 0.1;   // rad, ~5.7°
  double omniDirGoalThre   = 1.0;
  double omniDirDiffThre   = 1.5;   // rad, ~86°
  double stopDisThre       = 0.2;
  double slowDwnDisThre    = 1.0;
  bool   twoWayDrive       = true;
  bool   noRotAtGoal       = true;
};

// ── 持久状态 (控制循环之间保持) ──

struct PathFollowerState {
  double vehicleSpeed     = 0;
  int    pathPointID      = 0;
  int    lastPathPointID  = 0;
  int    lastPathSize     = 0;
  bool   navFwd           = true;
  double switchTime       = 0;
};

// ── 控制输出 ──

struct PathFollowerOutput {
  Twist  cmd;          // body-frame velocity command
  double dirDiff  = 0; // heading error after two-way correction (rad)
  double endDis   = 0; // distance to path end
  bool   canAccel = false;
};

// ── 自适应前视距离 (pathFollower.cpp:432-439) ──

inline double adaptiveLookAhead(double currentSpeed,
                                const PathFollowerParams& p) {
  double la = p.baseLookAheadDis + p.lookAheadRatio * std::fabs(currentSpeed);
  if (la < p.minLookAheadDis) la = p.minLookAheadDis;
  if (la > p.maxLookAheadDis) la = p.maxLookAheadDis;
  return la;
}

// ── 主控制计算 ──
// pathPoints: 局部路径点 (pathFollower 参考帧, 即 path_.poses[].position)
// vehicleRel: 机器人在路径参考帧中的相对位置 (vehicleXRel, vehicleYRel)
// vehicleYawDiff: vehicleYaw_ - vehicleYawRec_

inline PathFollowerOutput computeControl(
    const Vec3& vehicleRel,        // 机器人在路径参考帧中的位置
    double vehicleYawDiff,         // vehicleYaw_ - vehicleYawRec_
    const std::vector<Vec3>& pathPoints,
    double joySpeed,               // normalized [0, 1]
    double currentTime,
    double slowFactor,             // speed multiplier [0, 1], 1=normal
    int safetyStop,                // 0=none, 1=stop linear, 2=stop all
    const PathFollowerParams& p,
    PathFollowerState& state)
{
  PathFollowerOutput out;
  const int pathSize = static_cast<int>(pathPoints.size());
  if (pathSize == 0) return out;

  // ── endDis: 到路径末端的距离 ──
  double endDisX = pathPoints[pathSize - 1].x - vehicleRel.x;
  double endDisY = pathPoints[pathSize - 1].y - vehicleRel.y;
  double endDis  = std::sqrt(endDisX * endDisX + endDisY * endDisY);
  out.endDis = endDis;

  // ── 自适应前视距离 ──
  double lookAheadDis = adaptiveLookAhead(state.vehicleSpeed, p);

  // ── 前视点搜索 (pathFollower.cpp:442-464) ──
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

  // ── dirDiff 归一化 (pathFollower.cpp:471-475) ──
  double dirDiff = vehicleYawDiff - pathDir;
  dirDiff = std::fmod(dirDiff + M_PI, 2.0 * M_PI);
  if (dirDiff < 0) dirDiff += 2.0 * M_PI;
  dirDiff -= M_PI;

  // ── 双向行驶切换 (pathFollower.cpp:477-488) ──
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

  // ── yaw rate (pathFollower.cpp:497-507) ──
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

  // ── 速度限制 (pathFollower.cpp:509-518) ──
  if (pathSize <= 1) {
    joySpeed2 = 0;
  } else if (endDis / p.slowDwnDisThre < joySpeed) {
    joySpeed2 *= endDis / p.slowDwnDisThre;
  }

  double joySpeed3 = joySpeed2 * slowFactor;

  // ── canAccel + 速度积分 (pathFollower.cpp:526-530) ──
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

  // ── 安全停车 (pathFollower.cpp:537-538) ──
  if (safetyStop >= 1) state.vehicleSpeed = 0;
  if (safetyStop >= 2) vehicleYawRate = 0;

  // ── 输出速度 (pathFollower.cpp:549-555) ──
  out.cmd.wz = vehicleYawRate;
  if (std::fabs(state.vehicleSpeed) > p.maxAccel / 100.0) {
    if (p.omniDirGoalThre > 0) {
      out.cmd.vx =  std::cos(dirDiff) * state.vehicleSpeed;
      out.cmd.vy = -std::sin(dirDiff) * state.vehicleSpeed;
    } else {
      out.cmd.vx = state.vehicleSpeed;
    }
  }

  return out;
}

}  // namespace nav_core
