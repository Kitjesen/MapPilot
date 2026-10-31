#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "cleaner.hpp"
#include "core/types.hpp"

namespace lingtu::map_cleaning {

struct MovingInstanceScore {
  int cell_x{0};
  int cell_y{0};
  std::uint64_t total_points{0};
  std::uint64_t candidate_points{0};
  std::uint64_t protected_points{0};
  double candidate_ratio{0.0};
  bool moving{false};
};

struct MovingScoreSummary {
  std::uint64_t scored_instances{0};
  std::uint64_t moving_instances{0};
  std::uint64_t candidate_points{0};
  double max_candidate_ratio{0.0};
};

MovingScoreSummary
scoreMovingInstances(const std::vector<PointXYZI> &source_map,
                     const std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> &evidence,
                     const StaticCleanerOptions &options);

}  // namespace lingtu::map_cleaning
