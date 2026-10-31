#pragma once

#include <cmath>
#include <optional>
#include "slam.hpp"

namespace lingtu::slam {

// Parent<-child poses. Resolve one source snapshot before exporting to consumers.
inline Pose3d composePoses(const Pose3d& lhs, const Pose3d& rhs) {
  auto normalized = [](double& x, double& y, double& z, double& w) {
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!std::isfinite(norm) || norm <= 1e-12) {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      w = 1.0;
      return;
    }
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  };

  double lx = lhs.qx;
  double ly = lhs.qy;
  double lz = lhs.qz;
  double lw = lhs.qw;
  double rx = rhs.qx;
  double ry = rhs.qy;
  double rz = rhs.qz;
  double rw = rhs.qw;
  normalized(lx, ly, lz, lw);
  normalized(rx, ry, rz, rw);

  const double tx = 2.0 * (ly * rhs.z - lz * rhs.y);
  const double ty = 2.0 * (lz * rhs.x - lx * rhs.z);
  const double tz = 2.0 * (lx * rhs.y - ly * rhs.x);

  Pose3d out;
  out.x = lhs.x + rhs.x + lw * tx + (ly * tz - lz * ty);
  out.y = lhs.y + rhs.y + lw * ty + (lz * tx - lx * tz);
  out.z = lhs.z + rhs.z + lw * tz + (lx * ty - ly * tx);
  out.qx = lw * rx + lx * rw + ly * rz - lz * ry;
  out.qy = lw * ry - lx * rz + ly * rw + lz * rx;
  out.qz = lw * rz + lx * ry - ly * rx + lz * rw;
  out.qw = lw * rw - lx * rx - ly * ry - lz * rz;
  normalized(out.qx, out.qy, out.qz, out.qw);
  return out;
}

struct MapFrames {
  std::optional<Pose3d> body;
  std::optional<Pose3d> scan_body;
  std::optional<Pose3d> scan_lidar;
};

inline MapFrames resolveMapFrames(const SlamOutputs& out, const Pose3d& body_lidar) {
  MapFrames frames;
  if (!out.map_odom_tf) return frames;
  if (out.odometry_odom_body)
    frames.body = composePoses(out.map_odom_tf->pose, *out.odometry_odom_body);
  if (out.state_estimation_at_scan) {
    frames.scan_body = composePoses(out.map_odom_tf->pose, *out.state_estimation_at_scan);
    frames.scan_lidar = composePoses(*frames.scan_body, body_lidar);
  }
  return frames;
}

}  // namespace lingtu::slam
