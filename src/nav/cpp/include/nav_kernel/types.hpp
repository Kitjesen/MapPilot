/**
 * nav_kernel/types.hpp -- ROS-free navigation core types.
 *
 * All navigation kernels use these types. Runtime adapters convert at the
 * boundary, so this header stays portable C++17.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav_kernel {

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

// One body-relative sample of an executable local trajectory.
struct TrajectoryPoint {
  Vec3 position{};
  Vec3 velocity{};
  Vec3 acceleration{};
  double yaw{0.0};
  double yawRate{0.0};
  double timeFromStartS{0.0};
};

// Exact executable B-spline in the planning frame. The final two fields are
// the ROS-free equivalent of SCAN-Planner's Bspline message identity/knots.
struct SplineTarget {
  std::vector<Vec3> controls;
  std::vector<double> knots;
  int order{3};
  std::int64_t trajectoryId{0};
  double startTimeS{-1.0};
};

using Path = std::vector<Pose>;

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

}  // namespace nav_kernel
