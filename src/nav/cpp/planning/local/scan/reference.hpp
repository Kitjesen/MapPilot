#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan::detail {

inline std::vector<Eigen::Vector3d> adaptReferencePath(
    const LocalRouteView &route, double bodyHeightM) {
  std::vector<Eigen::Vector3d> reference;
  reference.reserve(static_cast<std::size_t>(route.count));
  for (int index = 0; index < route.count; ++index) {
    Eigen::Vector3d point{
        route.points[index].x,
        route.points[index].y,
        route.points[index].z - bodyHeightM,
    };
    if (reference.size() >= 2U) {
      const Eigen::Vector3d incoming =
          reference.back() - reference[reference.size() - 2U];
      const Eigen::Vector3d outgoing = point - reference.back();
      const double incomingLength = incoming.norm();
      const double outgoingLength = outgoing.norm();
      const bool shortCollinearCut =
          incomingLength > 1e-9 && outgoingLength > 1e-9 &&
          outgoingLength < ScanPlannerParams::kMinReferenceWaypointDistanceM &&
          incoming.dot(outgoing) > 0.0 &&
          incoming.cross(outgoing).norm() <=
              1e-6 * incomingLength * outgoingLength;
      if (shortCollinearCut) {
        // The rolling corridor may end just before the next point on the same
        // global edge. Keep the real bend/endpoint instead of a redundant cut.
        reference.back() = point;
        continue;
      }
    }
    reference.push_back(point);
  }
  return reference;
}

}  // namespace nav_kernel::local::scan::detail
