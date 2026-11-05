#pragma once

#include <string>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

// ROS-free Mapd adapter: inflated bits for twin cylinders, measured bits for Body.
class Grid {
 public:
  Grid(const LocalPlannerParams &params, const LocalPlanRequest &input);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const std::string &reason() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] int occupiedCellCount() const noexcept;
  [[nodiscard]] int collisionPointCount() const noexcept;
  // SCAN's upstream twin-cylinder query; inflation belongs to Mapd.
  [[nodiscard]] int cylinderOccupancy(const Vec3 &center, double yaw) const noexcept;
  // Snapshot geometry at the request time, including aged dynamic positions.
  // Native motion checks use this separately from SCAN's cylinder query.
  [[nodiscard]] int inflatedOccupancy(const Vec3 &center, double yaw) const noexcept;
  [[nodiscard]] int segmentInflatedOccupancy(const Vec3 &start, double startYaw,
                                              const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] bool obstacleFree(const Vec3 &center, double yaw) const noexcept;
  // Explicit conservative future sweep, used by the boundary-exit exception.
  [[nodiscard]] bool predictionIntersects(const Vec3 &start, double startYaw,
                                          const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] int trajectoryOccupancy(const Vec3 &start, const Vec3 &velocity,
                                         const Vec3 &end, const Vec3 &endVelocity,
                                         double startAheadS, double endAheadS) const noexcept;
  [[nodiscard]] std::size_t predictionCount() const noexcept { return predictions_.size(); }
  // A straight, fixed-heading exit may leave its initially occupied boundary
  // cell, but must end free and cannot enter any other occupied cell.
  [[nodiscard]] bool boundaryDepartureFree(const Pose &start, const Vec3 &end) const noexcept;
  [[nodiscard]] bool boundaryDepartureMotionFree(const Pose &body, const Twist &command,
                                                const Twist &measured, double maxSpeed,
                                                double reactionS, double deceleration,
                                                double yawDeceleration = 1.0) const noexcept;
  [[nodiscard]] int brakingOccupancy(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;
  [[nodiscard]] double brakingScale(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;

 private:
  [[nodiscard]] bool atRobotPosition(const Vec3 &point) const noexcept;
  [[nodiscard]] int occupiedState(const Vec3 &planningPoint) const noexcept;
  [[nodiscard]] int measuredSegmentOccupancy(const Vec3 &start, double startYaw,
                                              const Vec3 &end, double endYaw,
                                              bool useBodyHeading = true) const noexcept;
  [[nodiscard]] bool timedPredictionIntersects(const Vec3 &start, double startYaw,
                                               const Vec3 &end, double endYaw,
                                               double startAheadS, double endAheadS,
                                               bool useBodyHeading = true) const noexcept;
  [[nodiscard]] bool bodyPredictionIntersects(const Vec3 &start, double startYaw,
                                              const Vec3 &end, double endYaw,
                                              double startAheadS, double endAheadS,
                                              bool useBodyHeading,
                                              bool futureSweep = false) const noexcept;
  [[nodiscard]] int occupiedGridState(const Vec3 &point) const noexcept;
  [[nodiscard]] int bodyPoseOccupancy(const Vec3 &center, double yaw,
                                     double extraMargin = 0.0,
                                     std::vector<std::size_t> *occupiedCells = nullptr) const noexcept;
  [[nodiscard]] int bodySweepOccupancy(const Vec3 &start, double startYaw,
                                      const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] Grid sourceBodyGrid() const;
  [[nodiscard]] Grid excludingCells(const std::vector<std::size_t> &cells) const;
  [[nodiscard]] int occupiedGridSegment(const Vec3 &start, const Vec3 &end,
                                         bool leaveInitialCell = false) const noexcept;

  bool checkObstacle_{false};
  double configuredResolution_{0.0};
  double cylinderOffset_{0.0};
  std::vector<double> bodyHeadings_{};
  double headingSweepPadding_{0.0};
  double robotYaw_{0.0};
  Vec3 robotPosition_{};
  double predictionAgeS_{0.0}, predictionHorizonS_{1.0};
  LocalCollisionMapView collision_{};
  std::shared_ptr<const lingtu::nav::collision::Body> body_{};
  std::vector<PredictedObstacle> predictions_;
  std::string reason_{"collision_map_invalid"};
};

}  // namespace nav_kernel::local::scan
