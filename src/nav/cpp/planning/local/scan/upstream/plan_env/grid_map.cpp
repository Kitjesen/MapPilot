#include "planning/local/scan/upstream/plan_env/grid_map.h"

#include "planning/local/scan/grid.hpp"
#include <algorithm>
#include <cmath>

namespace nav_kernel::local::scan::upstream {

GridMap::GridMap(const Grid &grid) noexcept : grid_(&grid) {}

void GridMap::setGrid(const Grid *grid) noexcept {
  grid_ = grid;
}

double GridMap::getResolution() const noexcept {
  return grid_ == nullptr ? 0.0 : grid_->resolution();
}

int GridMap::getInflateOccupancy(const Eigen::Vector3d &position,
                                 double yaw) const noexcept {
  if (grid_ == nullptr) return -1;
  const Vec3 point{position.x(), position.y(), position.z()};
  return grid_->cylinderOccupancy(point, yaw);
}

int GridMap::getInflateOccupancySegment(const Eigen::Vector3d &start, double startYaw,
                                        const Eigen::Vector3d &end, double endYaw) const noexcept {
  if (grid_ == nullptr || !grid_->valid()) return -1;
  const int steps = std::max(1, static_cast<int>(std::ceil(
      (end - start).norm() / getResolution())));
  const double turn = std::atan2(std::sin(endYaw - startYaw), std::cos(endYaw - startYaw));
  for (int i = 0; i <= steps; ++i) {
    const double fraction = static_cast<double>(i) / steps;
    const int occupied = getInflateOccupancy(start + fraction * (end - start),
                                             startYaw + fraction * turn);
    if (occupied != 0) return occupied;
  }
  return 0;
}

int GridMap::getTrajectoryOccupancy(const Eigen::Vector3d &start,
    const Eigen::Vector3d &velocity, const Eigen::Vector3d &end,
    const Eigen::Vector3d &endVelocity, double startTime, double endTime,
    std::optional<double> *previousHeading) const noexcept {
  if (grid_ == nullptr) return -1;
  static_cast<void>(startTime);
  static_cast<void>(endTime);
  const auto heading = [](const Eigen::Vector3d &value, double fallback) {
    return std::hypot(value.x(), value.y()) > 0.03
        ? std::atan2(value.y(), value.x()) : fallback;
  };
  const double startYaw = heading(
      velocity, previousHeading ? previousHeading->value_or(grid_->robotYaw())
                                : grid_->robotYaw());
  const double endYaw = heading(endVelocity, startYaw);
  if (previousHeading) *previousHeading = endYaw;
  // The upstream optimizer queries the inflated voxel cylinder. Multipart body
  // checks belong to the execution/safety layer and must not change this path.
  return getInflateOccupancySegment(start, startYaw, end, endYaw);
}

}  // namespace nav_kernel::local::scan::upstream
