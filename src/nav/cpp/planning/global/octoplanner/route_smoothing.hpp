#pragma once

#include "global_planner.h"
#include <functional>
#include <optional>
#include <vector>

namespace global_planner {

using EdgeCheck = std::function<bool(const PointPose &, const PointPose &)>;
using EdgeCost = std::function<double(const PointPose &, const PointPose &)>;
// Returns body-reference Z on the corresponding layer, or no value outside the standing-height band.
using BodyHeight = std::function<std::optional<double>(const PointPose &)>;
using RouteCancelCheck = std::function<bool()>;

bool checkRoute(std::vector<PointPose> &route, const EdgeCheck &valid,
                const RouteCancelCheck &cancel = {});

std::vector<PointPose> smoothRoute(const std::vector<PointPose> &route,
                                  const EdgeCheck &valid,
                                  const BodyHeight &height,
                                  const RouteCancelCheck &cancel = {},
                                  const EdgeCost &cost = {});

}  // namespace global_planner
