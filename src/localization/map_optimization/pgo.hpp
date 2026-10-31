#pragma once

#include "localization/map_optimization/map.hpp"
#include "localization/map_optimization/graph.hpp"

#include <vector>

namespace lingtu::localization::map_optimization {

Result pgo(const Map& map, const std::vector<GeometricConstraint>& constraints);

}  // namespace lingtu::localization::map_optimization
