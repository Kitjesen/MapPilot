#pragma once

#include <octomap/OcTree.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lingtu::maps::sampled_octomap {

struct VoxelKey {
  unsigned int x = 0;
  unsigned int y = 0;
  unsigned int z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<unsigned int>{}(key.x);
    seed ^= std::hash<unsigned int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<unsigned int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

inline void markFreeEnvelope(
  octomap::OcTree & tree,
  const VoxelKey & key,
  int layers,
  int dilation,
  const std::unordered_set<VoxelKey, VoxelKeyHash> & occupied_keys)
{
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());
  for (int dx = -dilation; dx <= dilation; ++dx) {
    for (int dy = -dilation; dy <= dilation; ++dy) {
      for (int dz = 1; dz <= layers; ++dz) {
        const long long x = static_cast<long long>(key.x) + dx;
        const long long y = static_cast<long long>(key.y) + dy;
        const long long z = static_cast<long long>(key.z) + dz;
        if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
          continue;
        }
        VoxelKey free_key{
          static_cast<unsigned int>(x),
          static_cast<unsigned int>(y),
          static_cast<unsigned int>(z)};
        if (occupied_keys.count(free_key)) {
          continue;
        }
        octomap::OcTreeKey octo_key;
        octo_key.k[0] = free_key.x;
        octo_key.k[1] = free_key.y;
        octo_key.k[2] = free_key.z;
        tree.updateNode(tree.keyToCoord(octo_key), false);
      }
    }
  }
}

inline std::unordered_set<VoxelKey, VoxelKeyHash> horizontalSupportKeys(
  const std::unordered_set<VoxelKey, VoxelKeyHash> & occupied)
{
  std::unordered_set<VoxelKey, VoxelKeyHash> supports;
  supports.reserve(occupied.size());
  constexpr int kVerticalTolerance = 1;
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());

  auto contains = [&](const VoxelKey & key, int dx, int dy, int dz) {
    const long long x = static_cast<long long>(key.x) + dx;
    const long long y = static_cast<long long>(key.y) + dy;
    const long long z = static_cast<long long>(key.z) + dz;
    if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
      return false;
    }
    return occupied.count(VoxelKey{
      static_cast<unsigned int>(x),
      static_cast<unsigned int>(y),
      static_cast<unsigned int>(z)}) > 0;
  };

  auto hasDirection = [&](const VoxelKey & key, int axis_x, int axis_y) {
    // Support padding needs a locally connected, exposed surface. Distant
    // probes can join opposite corridor walls and inflate their interiors.
    for (int dz = -kVerticalTolerance; dz <= kVerticalTolerance; ++dz) {
      if (contains(key, axis_x, axis_y, dz) &&
          !contains(key, axis_x, axis_y, dz + 1)) {
        return true;
      }
    }
    return false;
  };

  for (const auto & key : occupied) {
    if (contains(key, 0, 0, 1)) {
      continue;
    }
    int directions = 0;
    directions += hasDirection(key, 1, 0) ? 1 : 0;
    directions += hasDirection(key, -1, 0) ? 1 : 0;
    directions += hasDirection(key, 0, 1) ? 1 : 0;
    directions += hasDirection(key, 0, -1) ? 1 : 0;
    if (directions >= 3) {
      supports.insert(key);
    }
  }
  return supports;
}

template <typename Point>
std::unordered_set<VoxelKey, VoxelKeyHash> buildOccupiedKeys(
  const std::vector<Point> & points,
  octomap::OcTree & tree,
  int support_dilation_cells,
  std::unordered_set<VoxelKey, VoxelKeyHash> & support_keys)
{
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(points.size());
  // Saved SLAM maps are already voxel-sampled and pruned. Point multiplicity
  // no longer measures repeated observations; filtering it erases real surfaces.
  for (const Point & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    octomap::OcTreeKey raw_key;
    if (!tree.coordToKeyChecked(point.x, point.y, point.z, raw_key)) {
      continue;
    }
    occupied.insert(VoxelKey{raw_key.k[0], raw_key.k[1], raw_key.k[2]});
  }

  support_keys = horizontalSupportKeys(occupied);
  if (support_dilation_cells <= 0 || occupied.empty()) {
    return occupied;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> dilated = occupied;
  std::unordered_map<VoxelKey, unsigned int, VoxelKeyHash> column_lowest;
  column_lowest.reserve(occupied.size());
  for (const auto & key : occupied) {
    const VoxelKey column{key.x, key.y, 0};
    const auto inserted = column_lowest.emplace(column, key.z);
    if (!inserted.second) inserted.first->second = std::min(inserted.first->second, key.z);
  }
  const int dilation = std::max(0, support_dilation_cells);
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());
  const auto support_seeds = support_keys;
  for (const auto & key : support_seeds) {
    for (int dx = -dilation; dx <= dilation; ++dx) {
      for (int dy = -dilation; dy <= dilation; ++dy) {
        const long long x = static_cast<long long>(key.x) + dx;
        const long long y = static_cast<long long>(key.y) + dy;
        const long long z = static_cast<long long>(key.z);
        if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
          continue;
        }
        const VoxelKey dilated_key{
          static_cast<unsigned int>(x),
          static_cast<unsigned int>(y),
          static_cast<unsigned int>(z)};
        // Fill sampling gaps, not a second surface above observed lower ground.
        // Otherwise a table or step top grows sideways into a walkable aisle.
        // Original occupied voxels remain untouched, including upper floors.
        const auto lower = column_lowest.find({dilated_key.x, dilated_key.y, 0});
        if (!occupied.count(dilated_key) && lower != column_lowest.end() &&
            static_cast<long long>(lower->second) < z) {
          continue;
        }
        dilated.insert(dilated_key);
        support_keys.insert(dilated_key);
      }
    }
  }
  return dilated;
}

}  // namespace lingtu::maps::sampled_octomap
