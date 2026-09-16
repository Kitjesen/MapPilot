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
  [[nodiscard]] int inflatedOccupancy(const Vec3 &center, double yaw) const noexcept;
  [[nodiscard]] int segmentInflatedOccupancy(const Vec3 &start, double startYaw,
                                              const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] bool obstacleFree(const Vec3 &center, double yaw) const noexcept;
  // A straight, fixed-heading exit may leave its initially occupied boundary
  // cell, but must end free and cannot enter any other occupied cell.
  [[nodiscard]] bool boundaryDepartureFree(const Pose &start, const Vec3 &end) const noexcept;
  [[nodiscard]] bool boundaryDepartureMotionFree(const Pose &body, const Twist &command,
                                                const Twist &measured, double maxSpeed,
                                                double reactionS, double deceleration) const noexcept;
  [[nodiscard]] int brakingOccupancy(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;
  [[nodiscard]] double brakingScale(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;

 private:
  [[nodiscard]] int occupiedState(const Vec3 &planningPoint) const noexcept;
  [[nodiscard]] int occupiedGridState(const Vec3 &point) const noexcept;
  [[nodiscard]] int occupiedGridSegment(const Vec3 &start, const Vec3 &end,
                                         bool leaveInitialCell = false) const noexcept;

  bool checkObstacle_{false};
  double configuredResolution_{0.0};
  double cylinderOffset_{0.0};
  std::optional<double> bodyHeading_{};
  LocalCollisionMapView collision_{};
  std::string reason_{"collision_map_invalid"};
};

}  // namespace nav_kernel::local::scan
