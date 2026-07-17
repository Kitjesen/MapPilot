
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>

#include <pcl/io/pcd_io.h>

namespace pcd2octomap
{

bool Key::operator==(const Key & other) const
{
  return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
}

std::size_t KeyHash::operator()(const Key & key) const
{
  std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
  seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

Pcd2OctomapConverter::Pcd2OctomapConverter()
: cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  tree_(std::make_shared<octomap::OcTree>(resolution_))
{
}

void Pcd2OctomapConverter::setInputPcdFile(const std::string & path)
{
  input_pcd_ = path;
}

void Pcd2OctomapConverter::setOutputBtFile(const std::string & path)
{
  output_bt_ = path;
}

void Pcd2OctomapConverter::setResolution(double resolution)
{
  if (resolution > 0.0) {
    resolution_ = resolution;
  }
}

void Pcd2OctomapConverter::setFreeEnvelopeLayers(int layers)
{
  free_layers_above_ = std::max(0, layers);
}

void Pcd2OctomapConverter::setFreeEnvelopeDilationCells(int cells)
{
  free_xy_dilation_cells_ = std::max(0, cells);
}

void Pcd2OctomapConverter::setSupportDilationCells(int cells)
{
  support_xy_dilation_cells_ = std::max(0, cells);
}

bool Pcd2OctomapConverter::convert()
{
  if (!loadPointCloud()) {
    return false;
  }

  tree_ = std::make_shared<octomap::OcTree>(resolution_);

  buildVoxelCounts();
  filterByPointCount();
  filterByConnectedClusters();
  occupied_keys_ = dilatedOccupiedKeys();
  fillOcTree();
  carveFreeEnvelope();

  if (!saveOctomap()) {
    return false;
  }

  std::cout << "\nConversion finished. To visualize, run:\n";
  std::cout << "octovis " << output_bt_ << std::endl;

  return true;
}

bool Pcd2OctomapConverter::loadPointCloud()
{
  cloud_->clear();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_, *cloud_) == -1) {
    std::cerr << "Couldn't read file " << input_pcd_ << std::endl;
    return false;
  }

  std::cout << "Loaded " << cloud_->size() << " points." << std::endl;
  return true;
}

void Pcd2OctomapConverter::buildVoxelCounts()
{
  voxel_counts_.clear();

  for (const auto & p : cloud_->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }

    octomap::OcTreeKey raw_key;
    if (tree_->coordToKeyChecked(p.x, p.y, p.z, raw_key)) {
      Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts_[key];
    }
  }
}

void Pcd2OctomapConverter::filterByPointCount()
{
  occupied_keys_.clear();

  for (const auto & entry : voxel_counts_) {
    if (static_cast<int>(entry.second) >= min_points_per_voxel_) {
      occupied_keys_.insert(entry.first);
    }
  }

  std::cout << "Voxels after point-count filtering: "
            << occupied_keys_.size() << std::endl;
}

void Pcd2OctomapConverter::filterByConnectedClusters()
{
  if (min_cluster_voxels_ <= 1 || occupied_keys_.empty()) {
    std::cout << "Voxels after cluster filtering: "
              << occupied_keys_.size() << std::endl;
    return;
  }

  std::unordered_set<Key, KeyHash> filtered_keys;
  std::unordered_set<Key, KeyHash> visited;
  filtered_keys.reserve(occupied_keys_.size());

  for (const auto & seed : occupied_keys_) {
    if (visited.count(seed)) {
      continue;
    }

    std::deque<Key> queue;
    std::vector<Key> cluster;

    queue.push_back(seed);
    visited.insert(seed);

    while (!queue.empty()) {
      Key current = queue.front();
      queue.pop_front();
      cluster.push_back(current);

      // 检查 26 邻域，3x3x3 范围
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }

            Key neighbor{{
              static_cast<unsigned int>(current.k[0] + dx),
              static_cast<unsigned int>(current.k[1] + dy),
              static_cast<unsigned int>(current.k[2] + dz)
            }};

            if (occupied_keys_.count(neighbor) && !visited.count(neighbor)) {
              visited.insert(neighbor);
              queue.push_back(neighbor);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_cluster_voxels_) {
      for (const auto & key : cluster) {
        filtered_keys.insert(key);
      }
    }
  }

  occupied_keys_ = std::move(filtered_keys);

  std::cout << "Voxels after cluster filtering: "
            << occupied_keys_.size() << std::endl;
}

void Pcd2OctomapConverter::fillOcTree()
{
  for (const auto & key : occupied_keys_) {
    octomap::OcTreeKey octo_key;
    octo_key.k[0] = key.k[0];
    octo_key.k[1] = key.k[1];
    octo_key.k[2] = key.k[2];

    tree_->updateNode(tree_->keyToCoord(octo_key), true);
  }

  tree_->updateInnerOccupancy();
}

std::unordered_set<Key, KeyHash> Pcd2OctomapConverter::dilatedOccupiedKeys()
{
  support_keys_ = horizontalSupportKeys();
  if (support_xy_dilation_cells_ <= 0 || occupied_keys_.empty()) {
    return occupied_keys_;
  }

  std::unordered_set<Key, KeyHash> out = occupied_keys_;
  const auto support_seeds = support_keys_;
  const int dilation = std::max(0, support_xy_dilation_cells_);
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());

  for (const auto & key : support_seeds) {
    for (int dx = -dilation; dx <= dilation; ++dx) {
      for (int dy = -dilation; dy <= dilation; ++dy) {
        const long long x = static_cast<long long>(key.k[0]) + dx;
        const long long y = static_cast<long long>(key.k[1]) + dy;
        const long long z = static_cast<long long>(key.k[2]);
        if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
          continue;
        }
        const Key dilated_key{{
          static_cast<unsigned int>(x),
          static_cast<unsigned int>(y),
          static_cast<unsigned int>(z)}};
        out.insert(dilated_key);
        support_keys_.insert(dilated_key);
      }
    }
  }

  std::cout << "Horizontal support candidates: " << support_seeds.size() << std::endl;
  std::cout << "High-confidence support voxels: " << support_keys_.size() << std::endl;
  std::cout << "Support-dilated voxels: " << out.size()
            << " xy_dilation=" << support_xy_dilation_cells_ << std::endl;
  return out;
}

std::unordered_set<Key, KeyHash> Pcd2OctomapConverter::horizontalSupportKeys() const
{
  std::unordered_set<Key, KeyHash> supports;
  supports.reserve(occupied_keys_.size());
  constexpr int kMinimumProbeDistance = 2;
  constexpr int kProbeRadius = 4;
  constexpr int kLateralTolerance = 2;
  constexpr int kVerticalTolerance = 1;
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());

  auto contains = [&](const Key & key, int dx, int dy, int dz) {
    const long long x = static_cast<long long>(key.k[0]) + dx;
    const long long y = static_cast<long long>(key.k[1]) + dy;
    const long long z = static_cast<long long>(key.k[2]) + dz;
    if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
      return false;
    }
    return occupied_keys_.count(Key{{
      static_cast<unsigned int>(x),
      static_cast<unsigned int>(y),
      static_cast<unsigned int>(z)}}) > 0;
  };

  auto hasDirection = [&](const Key & key, int axis_x, int axis_y) {
    for (int step = kMinimumProbeDistance; step <= kProbeRadius; ++step) {
      for (int lateral = -kLateralTolerance; lateral <= kLateralTolerance; ++lateral) {
        for (int dz = -kVerticalTolerance; dz <= kVerticalTolerance; ++dz) {
          const int dx = axis_x * step + axis_y * lateral;
          const int dy = axis_y * step + axis_x * lateral;
          if (contains(key, dx, dy, dz)) {
            return true;
          }
        }
      }
    }
    return false;
  };

  for (const auto & key : occupied_keys_) {
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

void Pcd2OctomapConverter::carveFreeEnvelope()
{
  if (!tree_ || free_layers_above_ <= 0 || occupied_keys_.empty()) {
    return;
  }

  std::unordered_set<Key, KeyHash> free_keys;
  const int dilation = std::max(0, free_xy_dilation_cells_);
  const auto max_key = static_cast<long long>(std::numeric_limits<unsigned int>::max());

  for (const auto & key : occupied_keys_) {
    for (int dx = -dilation; dx <= dilation; ++dx) {
      for (int dy = -dilation; dy <= dilation; ++dy) {
        for (int dz = 1; dz <= free_layers_above_; ++dz) {
          const long long x = static_cast<long long>(key.k[0]) + dx;
          const long long y = static_cast<long long>(key.k[1]) + dy;
          const long long z = static_cast<long long>(key.k[2]) + dz;
          if (x < 0 || y < 0 || z < 0 || x > max_key || y > max_key || z > max_key) {
            continue;
          }
          Key free_key{{
            static_cast<unsigned int>(x),
            static_cast<unsigned int>(y),
            static_cast<unsigned int>(z)
          }};
          if (!occupied_keys_.count(free_key)) {
            free_keys.insert(free_key);
          }
        }
      }
    }
  }

  for (const auto & key : free_keys) {
    octomap::OcTreeKey octo_key;
    octo_key.k[0] = key.k[0];
    octo_key.k[1] = key.k[1];
    octo_key.k[2] = key.k[2];
    tree_->updateNode(tree_->keyToCoord(octo_key), false);
  }

  tree_->updateInnerOccupancy();
  std::cout << "Free envelope cells: " << free_keys.size()
            << " layers=" << free_layers_above_
            << " xy_dilation=" << free_xy_dilation_cells_ << std::endl;
}

bool Pcd2OctomapConverter::saveOctomap() const
{
  if (!tree_) {
    std::cerr << "OcTree is null, cannot save." << std::endl;
    return false;
  }

  if (tree_->write(output_bt_)) {
    std::cout << "Success! Saved to " << output_bt_ << std::endl;
    return true;
  }

  std::cerr << "Failed to save " << output_bt_ << std::endl;
  return false;
}

bool Pcd2OctomapConverter::isPointFree(const octomap::point3d & p) const
{
  if (!tree_) {
    return false;
  }

  octomap::OcTreeNode * node = tree_->search(p);

  // 未知区域按可通行处理
  if (node == nullptr) {
    return true;
  }

  return !tree_->isNodeOccupied(node);
}

bool Pcd2OctomapConverter::isSpaceFree(
  const octomap::point3d & min_pt,
  const octomap::point3d & max_pt) const
{
  if (!tree_) {
    return false;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min_pt, max_pt),
       end = tree_->end_leafs_bbx(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      return false;
    }
  }

  return true;
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getOctomap()
{
    return tree_;
}

void Pcd2OctomapConverter::printOccupiedNodes() const
{
  if (!tree_) {
    return;
  }

  int count = 0;

  for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
       end = tree_->end_leafs(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      octomap::point3d p = it.getCoordinate();
      double size = it.getSize();

      std::cout << "Node [" << count << "]: "
                << "x=" << p.x() << ", y=" << p.y() << ", z=" << p.z()
                << " (Size: " << size << ")" << std::endl;

      ++count;
    }
  }
}

void Pcd2OctomapConverter::printQueryExamples() const
{
  octomap::point3d current_pos(13.1, 4.1, 14);

  if (isPointFree(current_pos)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupy" << std::endl;
  }

  octomap::point3d min_pt(13.1, 4.1, 10);
  octomap::point3d max_pt(13.1, 4.1, 16);

  if (isSpaceFree(min_pt, max_pt)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupancy" << std::endl;
  }
}

void Pcd2OctomapConverter::visualizeWithOctovis() const
{
  std::system(("octovis " + output_bt_).c_str());
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getTree() const
{
  return tree_;
}

}  // namespace pcd2octomap

// int main()
// {
//   pcd2octomap::Pcd2OctomapConverter converter;

//   if (!converter.convert()) {
//     return -1;
//   }

//   converter.printOccupiedNodes();
//   converter.printQueryExamples();

//   // 如果你已经安装了 octovis，保留下面这行会自动弹窗显示。
//   converter.visualizeWithOctovis();

//   return 0;
// }
