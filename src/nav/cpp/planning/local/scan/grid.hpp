#pragma once

#include <string>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

// ROS-free adapter from Mapd's inflated bitmap to SCAN GridMap queries.
class Grid {
 public:
  Grid(const LocalPlannerParams &params, const LocalPlanRequest &input);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const std::string &reason() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] int occupiedCellCount() const noexcept;
  [[nodiscard]] int collisionPointCount() const noexcept;
  [[nodiscard]] bool obstacleFree(const Vec3 &center, double yaw) const noexcept;

 private:
  [[nodiscard]] bool occupied(const Vec3 &planningPoint) const noexcept;

  bool checkObstacle_{false};
  double cylinderOffset_{0.0};
  LocalCollisionMapView collision_{};
  std::string reason_{"collision_map_invalid"};
};

}  // namespace nav_kernel::local::scan
