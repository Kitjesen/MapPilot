#pragma once
#include "lingtu/maps/build/saved_scans.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include <octomap/OcTree.h>
#include <stdexcept>
#include <cmath>

namespace lingtu::maps {

inline std::size_t OccupiedVoxelCount(const octomap::OcTree& tree) {
  std::size_t count=0;
  for (auto it=tree.begin_leafs(),end=tree.end_leafs();it!=end;++it) {
    if (!tree.isNodeOccupied(*it)) continue;
    const auto side=static_cast<std::size_t>(std::llround(it.getSize()/tree.getResolution()));
    count+=side*side*side;
  }
  return count;
}

inline std::size_t PopulateSavedRayOctomap(
    octomap::OcTree& tree, const std::filesystem::path& directory,
    const std::function<bool()>& cancelled = {}) {
  std::size_t count = 0;
  VisitSavedScans(directory, [&](const SavedScan& scan) {
    if (cancelled && cancelled()) throw std::runtime_error("saved ray build cancelled");
    octomap::Pointcloud cloud;
    for (std::size_t i = 0; i < scan.xyz.size(); i += 3)
      cloud.push_back(scan.xyz[i],scan.xyz[i+1],scan.xyz[i+2]);
    // Integrate real measured segments. Unknown space above a surface stays
    // unknown unless a ray traversed it; successive scans can clear old hits.
    tree.insertPointCloud(cloud,
        octomap::point3d(scan.origin[0],scan.origin[1],scan.origin[2]), -1, true, false);
    count += cloud.size();
  });
  // The save-time cleaned map owns retained geometry. Replaying the original
  // scans must not resurrect discarded dynamic points or removed surfaces.
  auto retained = LoadPcdXyz(directory / "map.pcd");
  if (!retained.ok || retained.points.empty())
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");
  // Lazy insertion above leaves full-depth cells. Rebuild from observed free
  // cells instead of deleteNode: deleting the last child in some OctoMap
  // versions leaves an allocated child array and triggers its destructor assert.
  std::vector<std::pair<octomap::OcTreeKey,float>> free_cells;
  for (auto it=tree.begin_leafs(), end=tree.end_leafs(); it!=end; ++it)
    if (!tree.isNodeOccupied(*it)) free_cells.emplace_back(it.getKey(),it->getLogOdds());
  tree.clear();
  for (const auto& cell : free_cells) tree.setNodeValue(cell.first,cell.second,true);
  for (const auto& point : retained.points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      tree.setNodeValue(point.x,point.y,point.z,tree.getClampingThresMaxLog(),true);
  tree.updateInnerOccupancy();
  return count;
}

}  // namespace lingtu::maps
