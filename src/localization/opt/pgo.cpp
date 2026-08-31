#include "localization/opt/pgo.hpp"

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {

Result pgo(const Map& map, const std::vector<GeometricConstraint>& constraints) {
  OptimizeOptions options;
  options.strategy = "pgo";
  options.max_iterations = 30;
  options.geometric_constraints = constraints;
  return optimize_map(map, options);
}

}  // namespace lingtu::localization::opt
