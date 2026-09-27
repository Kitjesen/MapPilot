
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

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

bool Pcd2OctomapConverter::convert()
{
  if (!loadPointCloud()) {
    return false;
  }

  tree_ = std::make_shared<octomap::OcTree>(resolution_);

  buildOccupiedKeys();
  if (occupied_keys_.empty()) {
    std::cerr << "PCD contains no finite points within OctoMap bounds" << std::endl;
    return false;
  }
  fillOcTree();

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

void Pcd2OctomapConverter::buildOccupiedKeys()
{
  occupied_keys_.clear();

  for (const auto & p : cloud_->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }

    octomap::OcTreeKey raw_key;
    if (tree_->coordToKeyChecked(p.x, p.y, p.z, raw_key)) {
      Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      occupied_keys_.insert(key);
    }
  }
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

  // A missing leaf is unknown, not observed free space.  Treating it as free
  // lets the legacy PCD compatibility path authorize motion through holes in
  // the sparse point cloud.
  if (node == nullptr) {
    return false;
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
