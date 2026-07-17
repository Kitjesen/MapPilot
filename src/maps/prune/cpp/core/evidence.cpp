#include "core/evidence.hpp"

#include <cmath>

namespace lingtu::map_cleaning {

PointXYZI transformPoint(const PointXYZI &pt, const Pose &pose) {
  const double norm =
      std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz);
  const double qw = norm > 1e-12 ? pose.qw / norm : 1.0;
  const double qx = norm > 1e-12 ? pose.qx / norm : 0.0;
  const double qy = norm > 1e-12 ? pose.qy / norm : 0.0;
  const double qz = norm > 1e-12 ? pose.qz / norm : 0.0;

  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;

  PointXYZI out;
  out.x = static_cast<float>((1.0 - 2.0 * (yy + zz)) * pt.x + 2.0 * (xy - wz) * pt.y +
                             2.0 * (xz + wy) * pt.z + pose.tx);
  out.y = static_cast<float>(2.0 * (xy + wz) * pt.x + (1.0 - 2.0 * (xx + zz)) * pt.y +
                             2.0 * (yz - wx) * pt.z + pose.ty);
  out.z = static_cast<float>(2.0 * (xz - wy) * pt.x + 2.0 * (yz + wx) * pt.y +
                             (1.0 - 2.0 * (xx + yy)) * pt.z + pose.tz);
  out.intensity = pt.intensity;
  return out;
}

VoxelKey voxelKey(const PointXYZI &pt, float voxel_size_m) {
  return {
      static_cast<int>(std::floor(pt.x / voxel_size_m)),
      static_cast<int>(std::floor(pt.y / voxel_size_m)),
      static_cast<int>(std::floor(pt.z / voxel_size_m)),
  };
}

bool isProtected(const VoxelEvidence &evidence, const StaticCleanerOptions &options) {
  return evidence.ground_hits > 0 || evidence.frame_count >= options.min_frame_support ||
         evidence.hits >= options.min_hit_support;
}

}  // namespace lingtu::map_cleaning
