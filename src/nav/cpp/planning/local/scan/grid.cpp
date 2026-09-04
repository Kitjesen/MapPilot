// Mapd adapter for SCAN-Planner's GridMap query seam.
// Upstream algorithm commit: 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_kernel::local::scan {

Grid::Grid(const LocalPlannerParams &params, const LocalPlanRequest &input)
    : checkObstacle_(params.checkObstacle),
      cylinderOffset_(std::max(0.0, params.scan.cylinderOffset)),
      collision_(input.environment.collision) {
  if (!checkObstacle_) {
    reason_ = "ready";
    return;
  }
  if (!collision_.valid()) {
    return;
  }
  if (!collision_.complete) {
    reason_ = "collision_map_incomplete";
    return;
  }
  if (collision_.resetEpoch == 0U || collision_.observationSequence == 0U ||
      collision_.generation == 0U) {
    return;
  }
  if (std::abs(collision_.resolution - params.scan.voxelResolution) >
      std::max(1e-9, 1e-6 * params.scan.voxelResolution)) {
    reason_ = "collision_map_resolution_mismatch";
    return;
  }
  reason_ = "ready";
}

bool Grid::valid() const noexcept {
  return reason_ == "ready";
}

const std::string &Grid::reason() const noexcept {
  return reason_;
}

double Grid::resolution() const noexcept {
  return collision_.resolution;
}

int Grid::occupiedCellCount() const noexcept {
  return static_cast<int>(std::min<std::size_t>(
      collision_.occupiedCount(), static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

int Grid::collisionPointCount() const noexcept {
  return occupiedCellCount();
}

bool Grid::obstacleFree(const Vec3 &center, double yaw) const noexcept {
  if (!valid() || !checkObstacle_)
    return valid();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return !occupied({center.x + cylinderOffset_ * c,
                    center.y + cylinderOffset_ * s, center.z}) &&
         !occupied({center.x - cylinderOffset_ * c,
                    center.y - cylinderOffset_ * s, center.z});
}

bool Grid::occupied(const Vec3 &planningPoint) const noexcept {
  return collision_.occupied(planningPoint);
}

}  // namespace nav_kernel::local::scan
