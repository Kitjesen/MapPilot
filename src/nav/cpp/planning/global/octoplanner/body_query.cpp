#include "body_query.hpp"
#include "collision/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace octoplanner3d::runtime {
namespace c = lingtu::nav::collision;

BodyQuery::BodyQuery(const octomap::OcTree &tree, const c::Body &body,
                     const lingtu::nav::plan::GlobalPlanTemporaryOverlay &overlay,
                     std::function<bool(const c::Aabb &)> observed_free,
                     std::shared_ptr<const c::Terrain> terrain)
    : tree_(tree), body_(body), overlay_(overlay), observed_free_(std::move(observed_free)),
      terrain_(std::move(terrain)),
      sweep_radius_(c::sweepRadius(body)) {}

BodyQuery::CachedEvidence BodyQuery::cellEvidence(int x, int y, int z) const {
  ++cell_lookups_;
  const auto block_axis = [](int value) {
    return value >= 0 ? value / 16 : (value + 1) / 16 - 1;
  };
  const global_planner::GridIndex block{block_axis(x),block_axis(y),block_axis(z)};
  if (!last_block_ || !(last_block_index_ == block)) {
    last_block_ = &cell_blocks_.try_emplace(block).first->second;
    last_block_index_ = block;
  }
  const auto offset = static_cast<std::size_t>(
      ((x-block.x*16)*16+y-block.y*16)*16+z-block.z*16);
  auto &evidence = (*last_block_)[offset];
  if (evidence != CachedEvidence::Unqueried) {
    ++cell_cache_hits_;
    return evidence;
  }
  const double resolution = tree_.getResolution();
  const auto *node = tree_.search((x + .5) * resolution, (y + .5) * resolution,
                                  (z + .5) * resolution);
  evidence = !node ? CachedEvidence::Unknown
      : tree_.isNodeOccupied(node) ? CachedEvidence::Occupied
                                    : CachedEvidence::Free;
  return evidence;
}

BodyQueryResult BodyQuery::pose(const c::Pose &pose, double extra_margin) const {
  ++queries_;
  const double resolution = tree_.getResolution();
  const double margin = body_.margin_m + extra_margin;
  const double ground_height = c::nominalGroundHeight(body_, pose);
  for (const auto &part : body_.parts) {
    const auto geometry = c::preparePart(part, pose, margin, body_.horizontal_margin_m);
    std::optional<c::TerrainSurface> surface;
    bool surface_queried = false;
    const auto prepare_surface = [&] {
      if (!surface_queried) {
        surface = c::partTerrainSurface(terrain_.get(), part, pose, ground_height, resolution,
                                        margin, body_.horizontal_margin_m);
        surface_queried = true;
      }
    };
    for (const auto &region : overlay_.blocked_regions) {
      // A box conservatively contains the request-scoped hard cylinder.
      const c::Aabb box{{region.center.x, region.center.y, (region.min_z + region.max_z) * .5},
                         {region.radius_xy_m, region.radius_xy_m, (region.max_z - region.min_z) * .5}};
      if (c::intersects(geometry, box)) {
        ++overlay_rejections_;
        return {false, "temporary_overlay", part.link};
      }
    }
    const auto &box = geometry.bounds;
    // Include both touching faces, matching the SAT contact tolerance.
    constexpr double contact = 1e-12;
    const int x0 = static_cast<int>(std::floor((box.center.x - box.half_size.x - contact) / resolution));
    const int y0 = static_cast<int>(std::floor((box.center.y - box.half_size.y - contact) / resolution));
    const int z0 = static_cast<int>(std::floor((box.center.z - box.half_size.z - contact) / resolution));
    const int x1 = static_cast<int>(std::floor((box.center.x + box.half_size.x + contact) / resolution));
    const int y1 = static_cast<int>(std::floor((box.center.y + box.half_size.y + contact) / resolution));
    const int z1 = static_cast<int>(std::floor((box.center.z + box.half_size.z + contact) / resolution));
    const auto [region, inserted]=empty_regions_.try_emplace(std::array<int,6>{x0,y0,z0,x1,y1,z1},true);
    if (inserted) {
      const octomap::point3d minimum((x0+.5)*resolution,(y0+.5)*resolution,(z0+.5)*resolution);
      const octomap::point3d maximum((x1+.5)*resolution,(y1+.5)*resolution,(z1+.5)*resolution);
      for (auto it=tree_.begin_leafs_bbx(minimum,maximum); it!=tree_.end_leafs_bbx(); ++it) {
        if (tree_.isNodeOccupied(*it)) { region->second=false; break; }
      }
    }
    if (region->second) continue;
    for (int x = x0; x <= x1; ++x) for (int y = y0; y <= y1; ++y)
      for (int z = z0; z <= z1; ++z) {
        const c::Aabb cell{{(x+.5)*resolution,(y+.5)*resolution,(z+.5)*resolution},
                            {resolution*.5,resolution*.5,resolution*.5}};
        const auto cell_state = cellEvidence(x, y, z);
        if (cell_state != CachedEvidence::Occupied) continue;
        if (!c::intersects(geometry, cell)) continue;
        if (c::allowsGroundContact(geometry, cell)) continue;
        if (terrain_ && terrain_->hitBoundsClear(geometry, cell)) continue;
        prepare_surface();
        if (surface && c::terrainCellClear(*terrain_, part, pose, margin, cell,
                                           *surface, body_.horizontal_margin_m)) continue;
        // Live free clears body obstacles without erasing saved ground evidence.
        if (observed_free_ && observed_free_(cell)) continue;
        ++occupied_rejections_;
        return {false, "occupied_body", part.link};
      }
  }
  ++free_poses_;
  return {};
}

bool BodyQuery::edge(const c::Pose &from, const c::Pose &to) const {
  ++edge_queries_;
  if (!pose(from).free || !pose(to).free) return false;
  const double distance = c::sweepBound(sweep_radius_, from, to);
  const double spacing = std::min(.01, .25 * tree_.getResolution());
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  const double padding = distance / (2 * samples);
  const double yaw_delta = c::shortestYawDelta(from.yaw, to.yaw);
  // Center the covering padding on each segment, not on the exact endpoints.
  for (int i = 0; i < samples; ++i) {
    const double t = (static_cast<double>(i) + .5) / samples;
    const c::Pose sample{{from.position.x+t*(to.position.x-from.position.x),
                          from.position.y+t*(to.position.y-from.position.y),
                          from.position.z+t*(to.position.z-from.position.z)},
                         from.yaw+t*yaw_delta};
    if (!pose(sample, padding).free) return false;
  }
  return true;
}

}  // namespace octoplanner3d::runtime
