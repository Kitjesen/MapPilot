#include "core/score.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "core/evidence.hpp"

namespace lingtu::map_cleaning {
namespace {

struct CellKey {
  int x{0};
  int y{0};

  bool operator<(const CellKey &other) const {
    if (x != other.x) {
      return x < other.x;
    }
    return y < other.y;
  }
};

CellKey instanceCell(const PointXYZI &pt, float grid_m) {
  return {
      static_cast<int>(std::floor(pt.x / grid_m)),
      static_cast<int>(std::floor(pt.y / grid_m)),
  };
}

}  // namespace

MovingScoreSummary
scoreMovingInstances(const std::vector<PointXYZI> &source_map,
                     const std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> &evidence,
                     const StaticCleanerOptions &options) {
  std::map<CellKey, MovingInstanceScore> cells;
  const float grid_m = std::max(options.instance_grid_m, options.voxel_size_m);

  for (const PointXYZI &pt : source_map) {
    const CellKey cell = instanceCell(pt, grid_m);
    MovingInstanceScore &score = cells[cell];
    score.cell_x = cell.x;
    score.cell_y = cell.y;
    ++score.total_points;

    const VoxelKey key = voxelKey(pt, options.voxel_size_m);
    auto found = evidence.find(key);
    const bool protected_voxel = found == evidence.end() || isProtected(found->second, options);
    if (protected_voxel) {
      ++score.protected_points;
    } else {
      ++score.candidate_points;
    }
  }

  MovingScoreSummary summary;
  for (auto &entry : cells) {
    MovingInstanceScore &score = entry.second;
    if (score.total_points == 0) {
      continue;
    }
    score.candidate_ratio =
        static_cast<double>(score.candidate_points) / static_cast<double>(score.total_points);
    score.moving = score.total_points >= options.min_instance_points &&
                   score.candidate_ratio >= options.moving_score_threshold;

    ++summary.scored_instances;
    summary.candidate_points += score.candidate_points;
    summary.max_candidate_ratio = std::max(summary.max_candidate_ratio, score.candidate_ratio);
    if (score.moving) {
      ++summary.moving_instances;
    }
  }
  return summary;
}

}  // namespace lingtu::map_cleaning
