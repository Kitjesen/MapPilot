#pragma once

#include "localization/map_optimization/graph.hpp"

namespace lingtu::localization::map_optimization {
// Inputs have passed the shared graph, pose, and gravity-reference validation.
GraphSolution optimize_gravity_graph(const std::vector<Keyframe>& keyframes,
                                     const OptimizeOptions& options);
}
