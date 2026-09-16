#include "planning/local/scan/upstream/plan_env/grid_map.h"

#include "planning/local/scan/grid.hpp"

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
  return grid_->inflatedOccupancy(point, yaw);
}

int GridMap::getInflateOccupancySegment(const Eigen::Vector3d &start, double startYaw,
                                        const Eigen::Vector3d &end, double endYaw) const noexcept {
  if (grid_ == nullptr) return -1;
  return grid_->segmentInflatedOccupancy({start.x(), start.y(), start.z()}, startYaw,
                                         {end.x(), end.y(), end.z()}, endYaw);
}

}  // namespace nav_kernel::local::scan::upstream
