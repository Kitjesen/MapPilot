#pragma once

#include "planning/local/planner.hpp"

#include <array>

namespace nav_kernel {

// Keeps test setup concise while production callers exercise the atomic
// LocalPlanRequest contract directly.
class PlannerFixture {
 public:
  explicit PlannerFixture(
      const LocalPlannerParams& params = LocalPlannerParams())
      : planner_(params) {}

  bool configure(const std::string& pathsDir) {
    return planner_.configure(pathsDir);
  }

  bool configured() const { return planner_.configured(); }

  LocalPlannerDebugSnapshot debugSnapshot() const {
    return planner_.debugSnapshot();
  }

  void setVehicle(double x, double y, double z, double yaw) {
    vehicle_ = {{x, y, z}, yaw};
  }

  void setGoal(double x, double y) { target_ = {x, y, 0.0}; }

  void setTraversabilityGrid(const float* values,
                             int rows,
                             int cols,
                             double resolution,
                             double originX,
                             double originY) {
    traversability_ = {
        values, rows, cols, resolution, originX, originY};
  }

  void clearTraversabilityGrid() { traversability_ = {}; }

  LocalPlan plan(
      const float* obstaclePoints, int pointCount, double timestampS) {
    const LocalPlanRequest request = makeRequest(
        obstaclePoints, pointCount, timestampS);
    return planner_.plan(request);
  }

  LocalPlan planObjective(const float* obstaclePoints,
                          int pointCount,
                          double timestampS,
                          double directionBodyDeg,
                          double speedNormalized,
                          double horizonM,
                          double maxDirectionDeviationDeg) {
    LocalPlanRequest request = makeRequest(
        obstaclePoints, pointCount, timestampS);
    const LocalRouteView guide = *request.route();
    request.objective = MotionIntentTarget{
        {directionBodyDeg,
         speedNormalized,
         horizonM,
         maxDirectionDeviationDeg},
        guide};
    return planner_.plan(request);
  }

  LocalPlan planFrame(double x,
                      double y,
                      double z,
                      double yaw,
                      double goalX,
                      double goalY,
                      const float* traversabilityGrid,
                      int traversabilityRows,
                      int traversabilityCols,
                      double traversabilityResolution,
                      double traversabilityOriginX,
                      double traversabilityOriginY,
                      const float* obstaclePoints,
                      int pointCount,
                      double timestampS) {
    setVehicle(x, y, z, yaw);
    setGoal(goalX, goalY);
    setTraversabilityGrid(
        traversabilityGrid,
        traversabilityRows,
        traversabilityCols,
        traversabilityResolution,
        traversabilityOriginX,
        traversabilityOriginY);
    return plan(obstaclePoints, pointCount, timestampS);
  }

  void reset() { planner_.reset(); }

  const LocalPlannerParams& params() const { return planner_.params(); }

 private:
  LocalPlanRequest makeRequest(
      const float* obstaclePoints,
      int pointCount,
      double timestampS) const {
    LocalPlanRequest request;
    request.robot.pose = vehicle_;
    route_ = {vehicle_.position, target_};
    request.objective = RouteTarget{{
        route_.data(), static_cast<int>(route_.size()), 0, true}};
    request.environment.obstacles = {obstaclePoints, pointCount};
    request.environment.traversability = traversability_;
    request.clock.timestampS = timestampS;
    return request;
  }

  local::Planner planner_;
  Pose vehicle_{};
  Vec3 target_{};
  mutable std::array<Vec3, 2> route_{};
  LocalTraversabilityView traversability_{};
};

}  // namespace nav_kernel
