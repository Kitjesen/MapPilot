#pragma once

#include "localization/opt/map.hpp"
#include "localization/opt/graph.hpp"

#include <vector>

namespace lingtu::localization::opt {

Result pgo(const Map& map, const std::vector<GeometricConstraint>& constraints);

}  // namespace lingtu::localization::opt
