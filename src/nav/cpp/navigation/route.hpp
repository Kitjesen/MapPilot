#pragma once

#include <optional>
#include <memory>
#include <vector>

#include "nav_kernel/types.hpp"
#include "navigation/stairs.hpp"
#include "navigation/pose_route.hpp"
#include "collision/body.hpp"

namespace lingtu::nav::navigation {

// Global-planner output activated for local execution. The geometric route and
// terminal tolerances travel together so Executor receives one coherent value.
struct Route {
  std::vector<nav_kernel::Vec3> points;
  std::optional<double> finalYaw;
  std::optional<double> goalToleranceM;
  std::optional<double> yawToleranceRad;
  std::optional<double> maxSpeedMps{};
  std::vector<StairSegment> stairs;
  // Explicit body-origin pose route. When present, this is the execution
  // source; points remains the legacy XYZ input and need not be duplicated.
  std::vector<PoseRoutePoint> posePoints;
  std::shared_ptr<const collision::Body> body{};
  // Authoritative base-to-support height from the global planning request.
  // Wide-turn blending needs this physical datum; torso extent is not a floor.
  std::optional<double> supportHeightM;
  std::optional<double> supportHeightToleranceM;
  // Task arrival is measured from the requested position, not an adjusted path endpoint.
  std::optional<nav_kernel::Vec3> requestedGoal;

  [[nodiscard]] bool empty() const noexcept { return points.empty() && posePoints.empty(); }
};

}  // namespace lingtu::nav::navigation
