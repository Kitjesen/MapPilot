#include "localization/opt/pgo.hpp"

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {

Result pgo(const Map& map, const std::vector<GeometricConstraint>& constraints) {
  OptimizeOptions options;
  options.strategy = "pgo";
  // Measured, poorly conditioned loop information can need more LM steps.
  // Save-time optimization uses the kernel's bounded iteration ceiling.
  options.max_iterations = 200;
  options.geometric_constraints = constraints;
  return optimize_map(map, options);
}

}  // namespace lingtu::localization::opt
