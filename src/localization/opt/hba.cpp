#include "localization/opt/hba.hpp"

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {

Result hba(const Map& map) {
  OptimizeOptions options;
  options.strategy = "hba";
  options.max_iterations = 50;
  options.chain_rot_weight = 15000.0;
  options.chain_trans_weight = 15000.0;
  options.skip_rot_weight = 2500.0;
  options.skip_trans_weight = 2500.0;
  options.skip_stride = 5;
  return optimize_map(map, options);
}

}  // namespace lingtu::localization::opt
