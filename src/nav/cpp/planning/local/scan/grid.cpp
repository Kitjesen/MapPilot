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
  return inflatedOccupancy(center, yaw) == 0;
}

int Grid::inflatedOccupancy(const Vec3 &center, double yaw) const noexcept {
  if (!valid()) return -1;
  if (!checkObstacle_) return 0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const int front = occupiedState({center.x + cylinderOffset_ * c,
                                   center.y + cylinderOffset_ * s, center.z});
  if (front != 0) return front;
  return occupiedState({center.x - cylinderOffset_ * c,
                        center.y - cylinderOffset_ * s, center.z});
}

int Grid::occupiedState(const Vec3 &planningPoint) const noexcept {
  if (!valid() || !std::isfinite(planningPoint.x) || !std::isfinite(planningPoint.y) ||
      !std::isfinite(planningPoint.z)) {
    return -1;
  }

  const double c = std::cos(collision_.gridFromPlanningYaw);
  const double s = std::sin(collision_.gridFromPlanningYaw);
  const Vec3 point{
      collision_.gridFromPlanningTranslation.x + c * planningPoint.x - s * planningPoint.y,
      collision_.gridFromPlanningTranslation.y + s * planningPoint.x + c * planningPoint.y,
      collision_.gridFromPlanningTranslation.z + planningPoint.z,
  };
  constexpr double boundaryEpsilon = 1e-4;
  if (point.x < collision_.aabbMin.x + boundaryEpsilon ||
      point.y < collision_.aabbMin.y + boundaryEpsilon ||
      point.z < collision_.aabbMin.z + boundaryEpsilon ||
      point.x > collision_.aabbMax.x - boundaryEpsilon ||
      point.y > collision_.aabbMax.y - boundaryEpsilon ||
      point.z > collision_.aabbMax.z - boundaryEpsilon) {
    return -1;
  }

  const auto logicalIndex = [this](double value, double minimum) {
    const auto global = static_cast<std::int64_t>(
        std::floor(value / collision_.resolution));
    const auto minimumIndex = static_cast<std::int64_t>(
        std::llround(minimum / collision_.resolution));
    return global - minimumIndex;
  };
  const std::int64_t x = logicalIndex(point.x, collision_.aabbMin.x);
  const std::int64_t y = logicalIndex(point.y, collision_.aabbMin.y);
  const std::int64_t z = logicalIndex(point.z, collision_.aabbMin.z);
  if (x < 0 || x >= collision_.sizeX || y < 0 || y >= collision_.sizeY ||
      z < 0 || z >= collision_.sizeZ) {
    return -1;
  }
  const std::size_t linear =
      (static_cast<std::size_t>(z) * static_cast<std::size_t>(collision_.sizeY) +
       static_cast<std::size_t>(y)) *
          static_cast<std::size_t>(collision_.sizeX) +
      static_cast<std::size_t>(x);
  return collision_.occupiedLinear(linear) ? 1 : 0;
}

}  // namespace nav_kernel::local::scan
