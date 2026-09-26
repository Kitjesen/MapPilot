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

struct SavedRayBuildStats {
  std::size_t inserted_points{0};
  std::size_t retained_voxels{0};
  // Retained voxels the replayed rays left free or unknown. The cleaned map
  // keeps them as surfaces, so they are raised to one-hit occupied.
  std::size_t raised_voxels{0};
};

// Replays the saved scans into `tree` as OctoMap ray evidence and keeps the
// save-time cleaned map.pcd as the authority on which surfaces exist:
//  * only endpoints inside a retained map.pcd voxel are inserted, so returns
//    the dynamic filter discarded leave neither hits nor free carving behind;
//  * every voxel keeps the hit/miss log-odds its rays accumulated;
//  * a retained voxel the rays still left free or unknown (grazing rays over
//    floors erode surface voxels) is raised to one-hit occupied rather than
//    dropped. On 903room at 5 cm, 39% of retained voxels, and a quarter of the
//    walkable surface support, would otherwise disappear.
inline SavedRayBuildStats PopulateSavedRayOctomap(
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

  SavedRayBuildStats stats;
  stats.retained_voxels = retained_keys.size();
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
    tree.insertPointCloud(cloud,
        octomap::point3d(scan.origin[0],scan.origin[1],scan.origin[2]), -1, true, false);
    stats.inserted_points += cloud.size();
  });
  if (stats.inserted_points == 0U)
    throw std::runtime_error("saved ray build: no saved scan endpoint lies in map.pcd");

  const float one_hit = tree.getProbHitLog();
  for (const auto& key : retained_keys) {
    const auto* node = tree.search(key);
    if (node != nullptr && tree.isNodeOccupied(node)) continue;
    tree.setNodeValue(key, one_hit, true);
    ++stats.raised_voxels;
  }
  tree.updateInnerOccupancy();
  return stats;
}

}  // namespace lingtu::maps
