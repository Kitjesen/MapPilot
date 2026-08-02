/**
 * nav_kernel/waypoint_helpers_core.hpp -- path downsampling and waypoint helpers.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include "nav_kernel/validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace nav_kernel {

inline Path downsamplePath(const Path& input, double minDist) {
  if (input.empty()) return {};
  if (input.size() == 1) return input;

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

  if (distance3D(result.back().position, input.back().position) > 1e-9) {
    result.push_back(input.back());
  }
  return result;
}

struct WaypointTrackerParams {
  double waypointDistance   = 0.5;
  double arrivalThreshold  = 0.5;
  double stuckTimeoutSec   = 10.0;
  int    maxReplanCount    = 2;
  double replanCooldownSec = 5.0;
  size_t searchWindow      = 5;
};

enum class WaypointEvent {
  kNone,
  kWaypointReached,
  kGoalReached,
  kPathReceived,
  kReplanning,
  kStuckFinal,
};

struct WaypointResult {
  WaypointEvent event = WaypointEvent::kNone;
  size_t currentIndex = 0;
  size_t totalWaypoints = 0;
  Vec3 targetPoint;
  bool hasTarget = false;
};

class WaypointTracker {
public:
  explicit WaypointTracker(const WaypointTrackerParams& params = {})
    : p_(params) {}

  WaypointResult setPath(const Path& rawPath, double currentTime) {
    Path cleanPath = filterInvalidPoses(rawPath);
    path_ = downsamplePath(cleanPath, p_.waypointDistance);
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

  WaypointResult update(const Vec3& robotPos,
                        const std::vector<Vec3>& waypointsInOdom,
                        double currentTime) {
    WaypointResult r;
    r.totalWaypoints = path_.size();
    if (path_.empty() || goalReached_) return r;
    if (!isValidPosition(robotPos)) return r;

    // Stuck detection requests replanning but does not block waypoint updates.
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
          }
        } else {
          r.event = WaypointEvent::kStuckFinal;
          r.currentIndex = currentIdx_;
        }
      }
    }

    const bool useOdom = !waypointsInOdom.empty();
    // Clamp all indexing to the shorter of the two arrays to prevent OOB
    const size_t effectiveEnd = useOdom
        ? std::min(path_.size(), waypointsInOdom.size())
        : path_.size();

    {
      size_t searchEnd = std::min(currentIdx_ + p_.searchWindow, effectiveEnd);
      size_t bestIdx = currentIdx_;
      double bestDist = std::numeric_limits<double>::max();
      for (size_t i = currentIdx_; i < searchEnd; ++i) {
        const Vec3& wp = useOdom ? waypointsInOdom[i]
                                 : path_[i].position;
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

    // Clamp currentIdx_ to effectiveEnd for safe indexing
    if (currentIdx_ >= effectiveEnd) {
      currentIdx_ = effectiveEnd - 1;
    }

    const Vec3& target = useOdom ? waypointsInOdom[currentIdx_]
                                 : path_[currentIdx_].position;
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

    r.currentIndex = currentIdx_;
    r.hasTarget = true;
    r.targetPoint = target;
    return r;
  }

  const Path& path() const { return path_; }
  size_t currentIndex() const { return currentIdx_; }
  bool goalReached() const { return goalReached_; }
  int replanCount() const { return replanCount_; }

  Pose goalPose() const {
    if (path_.empty()) return {};
    return path_.back();
  }

private:
  WaypointTrackerParams p_;
  Path path_;
  size_t currentIdx_ = 0;
  bool goalReached_ = false;
  double lastProgressTime_ = 0;
  int replanCount_ = 0;
  double lastReplanTime_ = -1.0;
};

}  // namespace nav_kernel
