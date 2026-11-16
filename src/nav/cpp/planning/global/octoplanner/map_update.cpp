#include "global_planner.h"
#include "mls_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace global_planner {

bool OctoPlanner3D::updateOctomap(std::shared_ptr<octomap::OcTree> map) {
  if (!octree_ || !map || !artifact_ || !artifact_->rust_graph ||
      map->getResolution() != octree_->getResolution()) return false;

  const double resolution = map->getResolution();
  LtMlsRegion region{INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN};
  const auto changes = [&](const octomap::OcTree& source, const octomap::OcTree& other) {
    for (auto it = source.begin_leafs(); it != source.end_leafs(); ++it) {
      const int span = std::max(1, static_cast<int>(std::lround(it.getSize() / resolution)));
      const int x0 = static_cast<int>(std::lround((it.getX() - it.getSize() * .5) / resolution));
      const int y0 = static_cast<int>(std::lround((it.getY() - it.getSize() * .5) / resolution));
      const int z0 = static_cast<int>(std::lround((it.getZ() - it.getSize() * .5) / resolution));
      const bool occupied = source.isNodeOccupied(*it);
      for (int x = x0; x < x0 + span; ++x)
        for (int y = y0; y < y0 + span; ++y)
          for (int z = z0; z < z0 + span; ++z) {
            const auto* node = other.search((x + .5) * resolution,
                                            (y + .5) * resolution, (z + .5) * resolution);
            if (node && occupied == other.isNodeOccupied(node)) continue;
            region.min_x = std::min(region.min_x, x); region.max_x = std::max(region.max_x, x);
            region.min_y = std::min(region.min_y, y); region.max_y = std::max(region.max_y, y);
            region.min_z = std::min(region.min_z, z); region.max_z = std::max(region.max_z, z);
          }
    }
  };
  // Compare both ways so clearing occupied cells back to unknown also removes
  // their surfaces. Unchanged free/occupied evidence needs no graph update.
  changes(*map, *octree_);
  changes(*octree_, *map);
  if (region.min_x <= region.max_x) {
    std::vector<mls::SurfaceCell> occupied, known_free;
    for (auto it = map->begin_leafs(); it != map->end_leafs(); ++it) {
      const int span = std::max(1, static_cast<int>(std::lround(it.getSize() / resolution)));
      const int x0 = static_cast<int>(std::lround((it.getX() - it.getSize() * .5) / resolution));
      const int y0 = static_cast<int>(std::lround((it.getY() - it.getSize() * .5) / resolution));
      const int z0 = static_cast<int>(std::lround((it.getZ() - it.getSize() * .5) / resolution));
      auto& cells = map->isNodeOccupied(*it) ? occupied : known_free;
      for (int x = std::max(x0, region.min_x); x <= std::min(x0 + span - 1, region.max_x); ++x)
        for (int y = std::max(y0, region.min_y); y <= std::min(y0 + span - 1, region.max_y); ++y)
          for (int z = std::max(z0, region.min_z); z <= std::min(z0 + span - 1, region.max_z); ++z)
            cells.push_back({x, y, z});
    }
    const auto update = artifact_->rust_graph->update(region, occupied, known_free);
    std::fprintf(stderr, "OctoPlanner3D updated map region: surfaces=%llu preserved_edges=%llu rebuilt_edges=%llu\n",
        static_cast<unsigned long long>(update.surface_cells_rebuilt),
        static_cast<unsigned long long>(update.preserved_edges),
        static_cast<unsigned long long>(update.rebuilt_edges));
  }
  octree_ = std::move(map);
  artifact_->graph.reset();
  clearMapQueryCaches();
  rebuildExternalPreblockedCells();
  rebuildDerivedLayers();
  return true;
}

}  // namespace global_planner
