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
                                                double reactionS, double deceleration) const noexcept;
  [[nodiscard]] int brakingOccupancy(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;
  [[nodiscard]] double brakingScale(const Pose &body, const Twist &command,
                                    double reactionS, double linearDeceleration,
                                    double yawDeceleration) const noexcept;

 private:
  [[nodiscard]] bool supported(const Vec3 &center, double yaw,
                               const char **failure = nullptr) const noexcept;
  [[nodiscard]] bool supportedSegment(const Vec3 &start, double startYaw,
                                      const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] bool supportPatch(double x, double y, double bodyZ,
                                  double &height, const char **failure = nullptr) const noexcept;
  [[nodiscard]] bool evidenceBit(const std::vector<std::uint8_t> &bits,
                                 int x, int y, int z) const noexcept;
  [[nodiscard]] int occupiedState(const Vec3 &planningPoint) const noexcept;
  [[nodiscard]] int measuredSegmentOccupancy(const Vec3 &start, double startYaw,
                                              const Vec3 &end, double endYaw) const noexcept;
  [[nodiscard]] bool timedPredictionIntersects(const Vec3 &start, double startYaw,
                                               const Vec3 &end, double endYaw,
                                               double startAheadS, double endAheadS) const noexcept;
  [[nodiscard]] int occupiedGridState(const Vec3 &point) const noexcept;
  [[nodiscard]] int occupiedGridSegment(const Vec3 &start, const Vec3 &end,
                                         bool leaveInitialCell = false) const noexcept;

  bool checkObstacle_{false};
  double configuredResolution_{0.0};
  double cylinderOffset_{0.0};
  ScanPlannerParams support_{};
  std::optional<double> bodyHeading_{};
  double robotYaw_{0.0};
  double predictionAgeS_{0.0}, predictionHorizonS_{1.0};
  LocalCollisionMapView collision_{};
  std::vector<PredictedObstacle> predictions_;
  std::string reason_{"collision_map_invalid"};
};

}  // namespace nav_kernel::local::scan
