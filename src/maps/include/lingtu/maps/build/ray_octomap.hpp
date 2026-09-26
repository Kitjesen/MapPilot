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

// Replays the saved scans into `tree` from their sensor origins. The
// save-time cleaned map.pcd decides which surfaces are static:
//  * only endpoints inside a retained voxel are inserted, so returns the
//    dynamic filter discarded leave neither hits nor carved free space;
//  * a miss never lowers a retained voxel. A ray grazing a floor at a
//    shallow angle traverses the surface cell next to its endpoint; OctoMap
//    labels that cell free although the ray did not pass below the surface.
//    On 903room at 5 cm, plain replay turned 37% of the retained voxels free;
//    the Go2 planner then accepted 56% of the path the robot had walked while
//    mapping, against 75% with this rule.
// Every occupied voxel is a measured hit; nothing is written that no ray
// produced, and a retained voxel no endpoint reached stays unknown.
inline std::size_t PopulateSavedRayOctomap(
    octomap::OcTree& tree, const std::filesystem::path& directory,
    const std::function<bool()>& cancelled = {}) {
  const auto retained = LoadPcdXyz(directory / "map.pcd");
  if (!retained.ok || retained.points.empty())
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");
  octomap::KeySet retained_keys;
  for (const auto& point : retained.points) {
    octomap::OcTreeKey key;
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        tree.coordToKeyChecked(point.x, point.y, point.z, key))
      retained_keys.insert(key);
  }
  if (retained_keys.empty())
    throw std::runtime_error("saved ray build retained map.pcd has no valid OctoMap keys");

  std::size_t count = 0;
  VisitSavedScans(directory, [&](const SavedScan& scan) {
    if (cancelled && cancelled()) throw std::runtime_error("saved ray build cancelled");
    octomap::Pointcloud cloud;
    for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
      octomap::OcTreeKey key;
      if (std::isfinite(scan.xyz[i]) && std::isfinite(scan.xyz[i + 1]) &&
          std::isfinite(scan.xyz[i + 2]) &&
          tree.coordToKeyChecked(scan.xyz[i], scan.xyz[i + 1], scan.xyz[i + 2], key) &&
          retained_keys.count(key) != 0U)
        cloud.push_back(scan.xyz[i], scan.xyz[i + 1], scan.xyz[i + 2]);
    }
    if (cloud.size() == 0U) return;
    // insertPointCloud(lazy, no discretize) without the misses on retained voxels.
    octomap::KeySet free_cells, occupied_cells;
    tree.computeUpdate(cloud, octomap::point3d(scan.origin[0], scan.origin[1], scan.origin[2]),
                       free_cells, occupied_cells, -1);
    for (const auto& key : free_cells)
      if (retained_keys.count(key) == 0U) tree.updateNode(key, false, true);
    for (const auto& key : occupied_cells) tree.updateNode(key, true, true);
    count += cloud.size();
  });
  if (count == 0U)
    throw std::runtime_error("saved ray build retained map.pcd matched no saved scan endpoints");
  tree.updateInnerOccupancy();
  return count;
}

}  // namespace lingtu::maps
