#include "localization/map_optimization/pgo.hpp"

#include "localization/map_optimization/graph.hpp"

namespace lingtu::localization::map_optimization {

Result pgo(const Map& map, const std::vector<GeometricConstraint>& constraints) {
  OptimizeOptions options;
  options.strategy = "pgo";
  // Measured, poorly conditioned loop information can need more LM steps.
  // Save-time optimization uses the kernel's bounded iteration ceiling.
  options.max_iterations = 200;
  options.geometric_constraints = constraints;
  return optimize_map(map, options);
}

}  // namespace lingtu::localization::map_optimization
