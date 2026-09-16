#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <octomap/OcTree.h>

namespace lingtu::maps {

// Both the external .ot converter and the embedded binary writer are supported.
inline std::unique_ptr<octomap::OcTree> LoadOctomapTree(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  std::string header;
  if (!std::getline(input, header)) return nullptr;
  if (!header.empty() && header.back() == '\r') header.pop_back();
  if (header == "# Octomap OcTree binary file") {
    auto tree = std::make_unique<octomap::OcTree>(0.1);
    if (!tree->readBinary(path.string()) || tree->size() == 0U) return nullptr;
    return tree;
  }
  if (header != "# Octomap OcTree file") return nullptr;
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(path.string()));
  auto* tree = dynamic_cast<octomap::OcTree*>(raw.get());
  if (tree == nullptr || tree->size() == 0U) return nullptr;
  raw.release();
  return std::unique_ptr<octomap::OcTree>(tree);
}

}  // namespace lingtu::maps
