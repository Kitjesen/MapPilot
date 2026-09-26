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

// Near its endpoint a ray grazing a floor stays within one voxel of the
// surface: from a sensor 0.4 m up, a 5 cm cell for the last metre of a ray that
// lands 8 m away. OctoMap marks those surface cells free although the ray never
// passed below the surface.
inline constexpr double kGrazingGuardM = 1.0;

// Replays the saved scans into `tree` from their sensor origins. The
// save-time cleaned map.pcd decides which surfaces are static:
//  * only endpoints inside a retained voxel are inserted, so returns the
//    dynamic filter discarded leave neither hits nor carved free space;
//  * the last kGrazingGuardM of a ray does not lower a retained voxel, nor a
//    cell beside one in the same layer: a 5 cm hole in the sampled floor that
//    a grazing ray would otherwise mark free, which the planner reads as a
//    drop. The rest of the ray still clears them, so a person the filter
//    missed is carved away by the rays that later pass through where they
//    stood.
// Every occupied voxel is a measured hit; a guarded cell no ray end reached
// stays unknown. On 903room at 5 cm the Go2 planner accepts 91% of the path
// the robot walked while mapping; plain replay (which freed 37% of the
// retained voxels) 56%, and forcing every retained voxel occupied 61%.
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
  octomap::KeySet guarded;
  for (const auto& key : retained_keys)
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        guarded.insert(octomap::OcTreeKey(key[0] + dx, key[1] + dy, key[2]));

  const auto guard_cells = static_cast<std::size_t>(kGrazingGuardM / tree.getResolution());
  std::size_t count = 0;
  octomap::KeyRay ray;
  VisitSavedScans(directory, [&](const SavedScan& scan) {
    if (cancelled && cancelled()) throw std::runtime_error("saved ray build cancelled");
    const octomap::point3d origin(scan.origin[0], scan.origin[1], scan.origin[2]);
    // One scan updates each cell once, and its hits win over its misses, as in
    // OcTree::insertPointCloud.
    octomap::KeySet free_cells, occupied_cells;
    for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
      const octomap::point3d end(scan.xyz[i], scan.xyz[i + 1], scan.xyz[i + 2]);
      octomap::OcTreeKey key;
      if (!std::isfinite(end.x()) || !std::isfinite(end.y()) || !std::isfinite(end.z()) ||
          !tree.coordToKeyChecked(end, key) || retained_keys.count(key) == 0U)
        continue;
      occupied_cells.insert(key);
      ++count;
      if (!tree.computeRayKeys(origin, end, ray)) continue;
      std::size_t to_end = ray.size();
      for (const auto& cell : ray) {
        if (to_end-- > guard_cells || guarded.count(cell) == 0U) free_cells.insert(cell);
      }
    }
    for (const auto& cell : free_cells)
      if (occupied_cells.count(cell) == 0U) tree.updateNode(cell, false, true);
    for (const auto& cell : occupied_cells) tree.updateNode(cell, true, true);
  });
  if (count == 0U)
    throw std::runtime_error("saved ray build retained map.pcd matched no saved scan endpoints");
  tree.updateInnerOccupancy();
  return count;
}

}  // namespace lingtu::maps
