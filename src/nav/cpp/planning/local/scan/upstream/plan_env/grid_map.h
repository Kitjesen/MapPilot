#pragma once

#include <Eigen/Eigen>
#include <memory>

namespace nav_kernel::local::scan {
class Grid;
}

namespace nav_kernel::local::scan::upstream {

class GridMap {
 public:
  using Ptr = std::shared_ptr<GridMap>;

  GridMap() = default;
  explicit GridMap(const Grid &grid) noexcept;

  void setGrid(const Grid *grid) noexcept;

  [[nodiscard]] double getResolution() const noexcept;
  [[nodiscard]] int getInflateOccupancy(const Eigen::Vector3d &position,
                                        double yaw) const noexcept;
  [[nodiscard]] int getInflateOccupancySegment(const Eigen::Vector3d &start, double startYaw,
                                               const Eigen::Vector3d &end, double endYaw) const noexcept;

 private:
  const Grid *grid_{nullptr};
};

}  // namespace nav_kernel::local::scan::upstream
