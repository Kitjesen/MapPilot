#pragma once
#include "lingtu/maps/build/saved_scans.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include <octomap/OcTree.h>
#include <stdexcept>
#include <cmath>
#include <unordered_set>

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
  // The retained PCD is the result of the save-time dynamic filter.  Filter
  // scan endpoints before ray insertion so discarded dynamic returns cannot
  // leave occupied leaves (or free carving) in the persistent tree.  The
  // previous implementation inserted every scan, then cleared the tree and
  // reconstructed it from free leaves plus retained endpoints.  That erased
  // OctoMap's hit/miss log-odds and made the saved map semantically different
  // from the measured rays.
  const auto retained = LoadPcdXyz(directory / "map.pcd");
  if (!retained.ok || retained.points.empty())
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");

  struct Key {
    unsigned int x = 0;
    unsigned int y = 0;
    unsigned int z = 0;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key& key) const {
      std::size_t seed = std::hash<unsigned int>{}(key.x);
      seed ^= std::hash<unsigned int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      seed ^= std::hash<unsigned int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      return seed;
    }
  };
  std::unordered_set<Key, KeyHash> retained_keys;
  retained_keys.reserve(retained.points.size());
  for (const auto& point : retained.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    octomap::OcTreeKey key;
    if (tree.coordToKeyChecked(point.x, point.y, point.z, key)) {
      retained_keys.insert(Key{key.k[0], key.k[1], key.k[2]});
    }
  }
  if (retained_keys.empty())
    throw std::runtime_error("saved ray build retained map.pcd has no valid OctoMap keys");

  std::size_t count = 0;
  VisitSavedScans(directory, [&](const SavedScan& scan) {
    if (cancelled && cancelled()) throw std::runtime_error("saved ray build cancelled");
    octomap::Pointcloud cloud;
    for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
      const float x = static_cast<float>(scan.xyz[i]);
      const float y = static_cast<float>(scan.xyz[i + 1]);
      const float z = static_cast<float>(scan.xyz[i + 2]);
      octomap::OcTreeKey key;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          !tree.coordToKeyChecked(x, y, z, key) ||
          retained_keys.count(Key{key.k[0], key.k[1], key.k[2]}) == 0U) {
        continue;
      }
      cloud.push_back(x, y, z);
    }
    // Integrate only retained measured segments.  OctoMap keeps the
    // accumulated hit/miss log-odds and unknown leaves exactly as observed;
    // no post-integration clear/rebuild is allowed here.
    if (cloud.size() == 0U) return;
    tree.insertPointCloud(cloud,
        octomap::point3d(scan.origin[0],scan.origin[1],scan.origin[2]), -1, true, false);
    count += cloud.size();
  });
  if (count == 0U)
    throw std::runtime_error("saved ray build retained map.pcd matched no saved scan endpoints");
  tree.updateInnerOccupancy();
  return count;
}

}  // namespace lingtu::maps
