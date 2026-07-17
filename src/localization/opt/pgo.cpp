#include "localization/opt/pgo.hpp"

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {

Result pgo(const Map& map) {
  OptimizeOptions options;
  options.strategy = "pgo";
  options.max_iterations = 30;
  options.chain_rot_weight = 10000.0;
  options.chain_trans_weight = 10000.0;
  options.skip_stride = 0;
  return optimize_map(map, options);
}

}  // namespace lingtu::localization::opt
