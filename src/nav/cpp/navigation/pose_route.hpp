#pragma once

#include <vector>
#include <memory>
#include <optional>

#include "nav_kernel/types.hpp"
#include "collision/body.hpp"

namespace lingtu::nav::navigation {

struct PoseRoutePoint {
  nav_kernel::Vec3 position{};
  double yaw{0.0};
};

struct PoseRouteStage {
  std::vector<nav_kernel::Vec3> points;
  double yaw{0.0};
  bool rotation{false};
  bool tangentHeading{false};
};

// Coupled edges support forward tangent poses, not an independently scheduled
// body yaw (crabbing/reversing while turning). Keep explicit stationary turns.
std::vector<PoseRouteStage> compilePoseRoute(const std::vector<PoseRoutePoint>& points,
                                          double yawToleranceRad = 0.05);

struct PoseRouteBlend {
  PoseRouteStage stage;
  std::size_t lastStageIndex{0};
  double radiusM{0.0};
};

// Round compatible forward legs as an optimization reference for SCAN.
// SCAN validates the resulting optimized spline before it can be executed.
std::optional<PoseRouteBlend> poseRouteBlend(
    const std::vector<PoseRouteStage>& stages, std::size_t index, double yawToleranceRad,
    double gridResolutionM=0.0);

// Join compatible forward legs without changing their XYZ geometry. SCAN owns
// smoothing and checking the resulting spline; corners are not local goals.
std::optional<PoseRouteBlend> poseRouteContinuous(
    const std::vector<PoseRouteStage>& stages, std::size_t index, double yawToleranceRad,
    double gridResolutionM=0.0);

}  // namespace lingtu::nav::navigation
