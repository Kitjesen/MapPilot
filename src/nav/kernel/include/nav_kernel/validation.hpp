/**
 * nav_kernel/validation.hpp -- NaN/Inf input validation utilities
 *
 * Inline helpers to guard against invalid floating-point inputs
 * (NaN, Inf) that can propagate through the control pipeline and
 * cause undefined robot behavior.
 *
 * Zero ROS2 dependency.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace nav_kernel {

// -- Position / point validation --

inline bool isValidPosition(const Vec3& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

inline bool isValidPose(const Pose& p) {
  return isValidPosition(p.position) && std::isfinite(p.yaw);
}

// -- Path validation --

inline bool isValidPath(const std::vector<Vec3>& path) {
  return !path.empty() &&
         std::all_of(path.begin(), path.end(), isValidPosition);
}

inline bool isValidPath(const Path& path) {
  return !path.empty() &&
         std::all_of(path.begin(), path.end(), isValidPose);
}

// -- Filter invalid points from a path (returns new path without NaN/Inf) --

inline Path filterInvalidPoses(const Path& input) {
  Path result;
  result.reserve(input.size());
  for (const auto& pose : input) {
    if (isValidPose(pose)) {
      result.push_back(pose);
    }
  }
  return result;
}

}  // namespace nav_kernel
