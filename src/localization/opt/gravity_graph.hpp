#pragma once

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {
// Inputs have passed the shared graph, pose, and gravity-reference validation.
GraphSolution optimize_gravity_graph(const std::vector<Keyframe>& keyframes,
                                     const OptimizeOptions& options);
}
