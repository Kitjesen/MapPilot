#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <octomap/OcTree.h>

namespace lingtu::maps {

// Reads both OctoMap encodings: the full `.ot` tree and the binary `.bt` tree.
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

// Saved maps use the full `.ot` encoding, which keeps every node's accumulated
// log-odds. The binary encoding keeps only occupied/free, so it is written
// only when the artifact is an explicit `.bt`. Both writers are const: the
// non-const writeBinary() would first collapse the in-memory tree to its
// maximum-likelihood state.
inline bool SaveOctomapTree(const octomap::OcTree& tree, const std::filesystem::path& path,
                            bool binary = false) {
  return binary ? tree.writeBinaryConst(path.string()) : tree.write(path.string());
}

}  // namespace lingtu::maps
