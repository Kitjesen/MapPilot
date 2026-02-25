/**
 * nav_core/pct_adapter_core.hpp — pct_path_adapter.cpp 纯算法提取
 *
 * 从 src/global_planning/pct_adapters/src/pct_path_adapter.cpp 提取:
 *   - downsamplePath()  — 3D 距离降采样
 *   - WaypointTracker   — 航点推进 + 到达检测 + stuck 判定
 *
 * 零 ROS2 依赖。
 */
#pragma once

#include "nav_core/types.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace nav_core {

// ── 3D 距离降采样 (pct_path_adapter.cpp:121-148) ──

inline Path downsamplePath(const Path& input, double minDist) {
  if (input.empty()) return {};

  Path result;
  result.reserve(input.size());
  result.push_back(input.front());

  Pose lastPose = input.front();
  for (size_t i = 1; i < input.size(); ++i) {
    double dist = distance3D(input[i].position, lastPose.position);
    if (dist >= minDist) {
      result.push_back(input[i]);
      lastPose = input[i];
    }
  }

  // 始终包含终点 (pct_path_adapter.cpp:146)
  result.push_back(input.back());
  return result;
}

// ── 航点跟踪器参数 ──

struct WaypointTrackerParams {
  double waypointDistance   = 0.5;
  double arrivalThreshold  = 0.5;
  double stuckTimeoutSec   = 10.0;
  int    maxReplanCount    = 2;
  double replanCooldownSec = 5.0;
  size_t searchWindow      = 5;    // 正常模式下的前方搜索窗口
};

// ── 航点跟踪器事件 ──

enum class WaypointEvent {
  kNone,
  kWaypointReached,
  kGoalReached,
  kPathReceived,
  kReplanning,       // 第1级: 几何层重规划
  kStuckFinal,       // 第2级: 几何层重规划耗尽
};

struct WaypointResult {
  WaypointEvent event = WaypointEvent::kNone;
  size_t currentIndex  = 0;
  size_t totalWaypoints = 0;
  Vec3   targetPoint;          // 当前航点 (世界坐标)
  bool   hasTarget = false;    // 是否有有效航点输出
};

// ── 航点跟踪器 (pct_path_adapter.cpp control_loop 逻辑) ──

class WaypointTracker {
public:
  explicit WaypointTracker(const WaypointTrackerParams& params = {})
    : p_(params) {}

  // 收到新路径时调用
  WaypointResult setPath(const Path& rawPath, double currentTime) {
    path_ = downsamplePath(rawPath, p_.waypointDistance);
    currentIdx_ = 0;
    goalReached_ = false;
    lastProgressTime_ = currentTime;
    replanCount_ = 0;

    WaypointResult r;
    r.event = WaypointEvent::kPathReceived;
    r.currentIndex = 0;
    r.totalWaypoints = path_.size();
    return r;
  }

  // 每帧调用一次 (pct_path_adapter.cpp:191-303)
  // robotPos: 机器人在里程计坐标系中的位置
  // waypointPositions: 航点在里程计坐标系中的位置 (已做 TF 变换)
  //   长度必须 == path_.size(), 或传空则用 path_ 原始坐标
  WaypointResult update(const Vec3& robotPos,
                        const std::vector<Vec3>& waypointsInOdom,
                        double currentTime) {
    WaypointResult r;
    r.totalWaypoints = path_.size();
    if (path_.empty() || goalReached_) return r;

    // ── Stuck detection + 两级重规划 (pct_path_adapter.cpp:198-232) ──
    if (lastProgressTime_ > 0) {
      double elapsed = currentTime - lastProgressTime_;
      if (elapsed > p_.stuckTimeoutSec) {
        if (replanCount_ < p_.maxReplanCount) {
          if (currentTime - lastReplanTime_ > p_.replanCooldownSec) {
            replanCount_++;
            lastReplanTime_ = currentTime;
            lastProgressTime_ = currentTime;
            r.event = WaypointEvent::kReplanning;
            r.currentIndex = currentIdx_;
            return r;
          }
        } else {
          r.event = WaypointEvent::kStuckFinal;
          r.currentIndex = currentIdx_;
          return r;
        }
      }
    }

    // ── 窗口搜索最近航点 (pct_path_adapter.cpp:239-261) ──
    {
      size_t searchEnd = std::min(currentIdx_ + p_.searchWindow, path_.size());
      size_t bestIdx = currentIdx_;
      double bestDist = std::numeric_limits<double>::max();
      for (size_t i = currentIdx_; i < searchEnd; ++i) {
        const Vec3& wp = waypointsInOdom.empty() ? path_[i].position
                                                  : waypointsInOdom[i];
        double d = distance2D(robotPos, wp);
        if (d < bestDist) {
          bestDist = d;
          bestIdx = i;
        }
      }
      if (bestIdx > currentIdx_) {
        currentIdx_ = bestIdx;
        lastProgressTime_ = currentTime;
      }
    }

    // ── 到达检测 (pct_path_adapter.cpp:270-291) ──
    const Vec3& target = waypointsInOdom.empty()
                       ? path_[currentIdx_].position
                       : waypointsInOdom[currentIdx_];
    double distToTarget = distance2D(robotPos, target);

    if (distToTarget < p_.arrivalThreshold) {
      if (currentIdx_ < path_.size() - 1) {
        r.event = WaypointEvent::kWaypointReached;
        r.currentIndex = currentIdx_;
        lastProgressTime_ = currentTime;
        currentIdx_++;
      } else {
        r.event = WaypointEvent::kGoalReached;
        r.currentIndex = currentIdx_;
        goalReached_ = true;
        return r;
      }
    }

    // 输出当前目标航点
    r.currentIndex = currentIdx_;
    r.hasTarget = true;
    r.targetPoint = path_[currentIdx_].position;  // map 坐标
    return r;
  }

  // ── 查询 ──
  const Path& path() const { return path_; }
  size_t currentIndex() const { return currentIdx_; }
  bool goalReached() const { return goalReached_; }
  int replanCount() const { return replanCount_; }

  // 获取终点 (用于重规划)
  Pose goalPose() const {
    if (path_.empty()) return {};
    return path_.back();
  }

private:
  WaypointTrackerParams p_;
  Path   path_;
  size_t currentIdx_      = 0;
  bool   goalReached_     = false;
  double lastProgressTime_ = 0;
  int    replanCount_      = 0;
  double lastReplanTime_   = -1.0;
};

}  // namespace nav_core
