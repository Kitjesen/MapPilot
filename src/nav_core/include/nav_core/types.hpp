/**
 * nav_core/types.hpp — 零 ROS2 依赖的导航核心类型
 *
 * 所有 nav_core 算法使用这些类型, ROS2 Shell 在边界处转换。
 * 可在任何 C++17 编译器上编译 (Windows MSVC, GCC, Clang)。
 */
#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav_core {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

struct Pose {
  Vec3 position;
  double yaw = 0;
};

struct Twist {
  double vx = 0;    // body-frame forward
  double vy = 0;    // body-frame left
  double wz = 0;    // yaw rate (rad/s)
};

using Path = std::vector<Pose>;

// ── 数学工具 ──

inline double normalizeAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

inline double distance2D(const Vec3& a, const Vec3& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double distance3D(const Vec3& a, const Vec3& b) {
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace nav_core
