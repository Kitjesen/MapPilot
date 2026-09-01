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
  return grid_->obstacleFree(point, yaw) ? 0 : 1;
}

}  // namespace nav_kernel::local::scan::upstream
