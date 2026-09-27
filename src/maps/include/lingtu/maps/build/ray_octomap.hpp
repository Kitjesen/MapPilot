#pragma once
#include "lingtu/maps/build/saved_scans.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include <octomap/OcTree.h>
#include <array>
#include <stdexcept>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

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
// Match prune's metric endpoint confirmation across voxel boundaries.
inline constexpr double kRetainedEndpointMatchM = 0.05;

struct SavedRayOctomapStats {
  std::size_t valid_endpoints{0};
  std::size_t retained_endpoints{0};
  std::size_t dropped_endpoints{0};
  std::size_t free_updates{0};
  std::size_t hit_updates{0};
  std::size_t guarded_miss_suppressions{0};

  [[nodiscard]] std::size_t inserted_points() const noexcept {
    return retained_endpoints;
  }
};

// Replays saved scans into `tree` from their measured sensor origins. The
// cleaned map.pcd decides which endpoints remain static. A removed endpoint
// still contributes the measured free prefix before it, but contributes no
// hit and does not infer anything at or beyond the endpoint. Near every raw
// endpoint, misses against retained same-layer surface cells are suppressed
// using a metric distance; farther ray cells remain ordinary free evidence.
// Updates are de-duplicated per scan and measured hits win over misses.
inline SavedRayOctomapStats PopulateSavedRayOctomap(
    octomap::OcTree& tree, const std::filesystem::path& directory,
    const std::function<bool()>& cancelled = {}) {
  const auto retained = LoadPcdXyz(directory / "map.pcd");
  if (!retained.ok || retained.points.empty())
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");
  octomap::KeySet retained_keys;
  std::map<std::array<std::int64_t, 3>, std::vector<PointXyz>> retained_buckets;
  for (const auto& point : retained.points) {
    octomap::OcTreeKey key;
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        tree.coordToKeyChecked(point.x, point.y, point.z, key)) {
      retained_keys.insert(key);
      retained_buckets[{static_cast<std::int64_t>(std::floor(point.x / kRetainedEndpointMatchM)),
                        static_cast<std::int64_t>(std::floor(point.y / kRetainedEndpointMatchM)),
                        static_cast<std::int64_t>(std::floor(point.z / kRetainedEndpointMatchM))}]
          .push_back(point);
    }
  }
  if (retained_keys.empty())
    throw std::runtime_error("saved ray build retained map.pcd has no valid OctoMap keys");
  octomap::KeySet guarded;
  for (const auto& key : retained_keys)
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        guarded.insert(octomap::OcTreeKey(key[0] + dx, key[1] + dy, key[2]));

  SavedRayOctomapStats stats;
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
          !tree.coordToKeyChecked(end, key))
        continue;
      ++stats.valid_endpoints;
      bool retained_endpoint = retained_keys.count(key) != 0U;
      if (!retained_endpoint) {
        // A sampled surface point may be in the adjacent voxel. Confirm the
        // measured endpoint against its position, then keep the real hit key.
        const std::array<std::int64_t, 3> bucket{
            static_cast<std::int64_t>(std::floor(end.x() / kRetainedEndpointMatchM)),
            static_cast<std::int64_t>(std::floor(end.y() / kRetainedEndpointMatchM)),
            static_cast<std::int64_t>(std::floor(end.z() / kRetainedEndpointMatchM))};
        const double match_distance_sq =
            kRetainedEndpointMatchM * kRetainedEndpointMatchM;
        for (int dx = -1; dx <= 1 && !retained_endpoint; ++dx)
          for (int dy = -1; dy <= 1 && !retained_endpoint; ++dy)
            for (int dz = -1; dz <= 1 && !retained_endpoint; ++dz) {
              const auto nearby = retained_buckets.find(
                  {bucket[0] + dx, bucket[1] + dy, bucket[2] + dz});
              if (nearby == retained_buckets.end()) continue;
              for (const auto& point : nearby->second) {
                const double x = static_cast<double>(point.x) - end.x();
                const double y = static_cast<double>(point.y) - end.y();
                const double z = static_cast<double>(point.z) - end.z();
                if (x * x + y * y + z * z <= match_distance_sq) {
                  retained_endpoint = true;
                  break;
                }
              }
            }
      }
      if (!retained_endpoint) {
        ++stats.dropped_endpoints;
      } else {
        ++stats.retained_endpoints;
        occupied_cells.insert(key);
      }
      if (!tree.computeRayKeys(origin, end, ray)) continue;
      for (const auto& cell : ray) {
        if (guarded.count(cell) != 0U) {
          const auto delta = tree.keyToCoord(cell) - end;
          if (delta.norm_sq() <= kGrazingGuardM * kGrazingGuardM) {
            ++stats.guarded_miss_suppressions;
            continue;
          }
        }
        free_cells.insert(cell);
      }
    }
    for (const auto& cell : free_cells)
      if (occupied_cells.count(cell) == 0U) {
        tree.updateNode(cell, false, true);
        ++stats.free_updates;
      }
    for (const auto& cell : occupied_cells) {
      tree.updateNode(cell, true, true);
      ++stats.hit_updates;
    }
  });
  if (stats.retained_endpoints == 0U)
    throw std::runtime_error("saved ray build retained map.pcd matched no saved scan endpoints");
  tree.updateInnerOccupancy();
  return stats;
}

}  // namespace lingtu::maps
