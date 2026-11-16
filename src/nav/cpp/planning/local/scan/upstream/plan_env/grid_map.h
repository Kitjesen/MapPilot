#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <optional>

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
  void setPredictionMode(bool enabled) noexcept { usePredictions_ = enabled; }

  [[nodiscard]] double getResolution() const noexcept;
  [[nodiscard]] int getInflateOccupancy(const Eigen::Vector3d &position,
                                        double yaw) const noexcept;
  [[nodiscard]] int getInflateOccupancySegment(const Eigen::Vector3d &start, double startYaw,
                                               const Eigen::Vector3d &end, double endYaw) const noexcept;
  [[nodiscard]] int getTrajectoryOccupancy(const Eigen::Vector3d &start,
      const Eigen::Vector3d &velocity, const Eigen::Vector3d &end,
      const Eigen::Vector3d &endVelocity, double startTime, double endTime,
      std::optional<double> *previousHeading = nullptr) const noexcept;
  [[nodiscard]] int getTimedTrajectoryOccupancy(const Eigen::Vector3d &start,
      const Eigen::Vector3d &velocity, const Eigen::Vector3d &end,
      const Eigen::Vector3d &endVelocity, double startTime, double endTime,
      std::optional<double> *previousHeading = nullptr) const noexcept;

 private:
  const Grid *grid_{nullptr};
  bool usePredictions_{false};
};

}  // namespace nav_kernel::local::scan::upstream
