#pragma once

#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "planning/global/contract.hpp"
#include "global_planner.h"

#include <octomap/OcTree.h>
#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <unordered_map>

namespace octoplanner3d::runtime {

struct BodyQueryResult {
  bool free{true};
  std::string reason;
  std::string link;
};

// Body collision checks occupied geometry. Unknown remains distinct from free
// and cannot establish the observed ground required by support contacts.
// Live free may clear a saved body obstacle without changing support evidence.
class BodyQuery {
 public:
  BodyQuery(const octomap::OcTree &tree, const lingtu::nav::collision::Body &body,
            const lingtu::nav::plan::GlobalPlanTemporaryOverlay &overlay,
            std::function<bool(const lingtu::nav::collision::Aabb &)> observed_free = {},
            std::shared_ptr<const lingtu::nav::collision::Terrain> terrain = {});
  BodyQueryResult pose(const lingtu::nav::collision::Pose &pose, double extra_margin = 0) const;
  bool edge(const lingtu::nav::collision::Pose &from,
            const lingtu::nav::collision::Pose &to) const;
  std::size_t queries() const { return queries_; }
  std::size_t edgeQueries() const { return edge_queries_; }
  std::size_t cellLookups() const { return cell_lookups_; }
  std::size_t cellCacheHits() const { return cell_cache_hits_; }
  std::size_t freePoses() const { return free_poses_; }
  std::size_t overlayRejections() const { return overlay_rejections_; }
  std::size_t occupiedRejections() const { return occupied_rejections_; }
  std::size_t unknownRejections() const { return unknown_rejections_; }

 private:
  // The OcTree is immutable for the lifetime of one planner request. Cache
  // static cell evidence and the fixed Body's sweep radius. Terrain,
  // overlays and pose geometry are still evaluated for every pose.
  // Unknown remains a first-class value.
  // As in the planner, 16^3 blocks avoid hashing every neighboring voxel.
  enum class CachedEvidence : unsigned char { Unqueried, Unknown, Free, Occupied };
  using CellBlock = std::array<CachedEvidence, 16 * 16 * 16>;
  CachedEvidence cellEvidence(int x, int y, int z) const;

  const octomap::OcTree &tree_;
  const lingtu::nav::collision::Body &body_;
  const lingtu::nav::plan::GlobalPlanTemporaryOverlay &overlay_;
  const std::function<bool(const lingtu::nav::collision::Aabb &)> observed_free_;
  const std::shared_ptr<const lingtu::nav::collision::Terrain> terrain_;
  const double sweep_radius_;
  mutable std::size_t queries_{0};
  mutable std::size_t edge_queries_{0};
  mutable std::size_t cell_lookups_{0};
  mutable std::size_t cell_cache_hits_{0};
  mutable std::size_t free_poses_{0};
  mutable std::size_t overlay_rejections_{0};
  mutable std::size_t occupied_rejections_{0};
  mutable std::size_t unknown_rejections_{0};
  mutable std::unordered_map<global_planner::GridIndex, CellBlock,
                             global_planner::GridIndexHash> cell_blocks_;
  mutable global_planner::GridIndex last_block_index_{};
  mutable CellBlock *last_block_{nullptr};
  // Integer bounds include touching cells; this cache never stores live clearing decisions.
  mutable std::map<std::array<int,6>,bool> empty_regions_;
};

}  // namespace octoplanner3d::runtime
