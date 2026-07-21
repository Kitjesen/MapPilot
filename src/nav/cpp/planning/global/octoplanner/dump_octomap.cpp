#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
  std::string input;
  std::string output;
  std::size_t max_points = 0;
};

Args parseArgs(int argc, char ** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--input" || arg == "-i") {
      args.input = next();
    } else if (arg == "--output" || arg == "-o") {
      args.output = next();
    } else if (arg == "--max-points") {
      args.max_points = static_cast<std::size_t>(std::strtoull(next().c_str(), nullptr, 10));
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
        "usage: octoplanner3d_dump_octomap --input octomap.ot --output occupied.xyz "
        "[--max-points 20000]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (args.input.empty()) {
    throw std::runtime_error("missing --input");
  }
  if (args.output.empty()) {
    throw std::runtime_error("missing --output");
  }
  return args;
}

std::unique_ptr<octomap::OcTree> loadTree(const std::string & path)
{
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(path));
  if (!raw) {
    auto tree = std::make_unique<octomap::OcTree>(0.2);
    if (tree->readBinary(path)) {
      return tree;
    }
    throw std::runtime_error("failed to read OctoMap: " + path);
  }
  auto * tree = dynamic_cast<octomap::OcTree *>(raw.get());
  if (tree == nullptr) {
    throw std::runtime_error("OctoMap is not an OcTree: " + path);
  }
  raw.release();
  return std::unique_ptr<octomap::OcTree>(tree);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Args args = parseArgs(argc, argv);
    const auto tree = loadTree(args.input);
    std::vector<octomap::point3d> points;
    points.reserve(tree->size());
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (!tree->isNodeOccupied(*it)) {
        continue;
      }
      points.push_back(it.getCoordinate());
    }

    const std::size_t original_count = points.size();
    if (args.max_points > 0 && points.size() > args.max_points) {
      const std::size_t step = (points.size() + args.max_points - 1) / args.max_points;
      std::vector<octomap::point3d> sampled;
      sampled.reserve(args.max_points);
      for (std::size_t i = 0; i < points.size() && sampled.size() < args.max_points; i += step) {
        sampled.push_back(points[i]);
      }
      points = std::move(sampled);
    }

    std::ofstream out(args.output);
    if (!out) {
      throw std::runtime_error("failed to open output: " + args.output);
    }
    out << "# x y z\n";
    for (const auto & point : points) {
      out << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
    }
    std::cout
      << "{\"success\":true"
      << ",\"input\":\"" << args.input << "\""
      << ",\"output\":\"" << args.output << "\""
      << ",\"occupied_leafs\":" << original_count
      << ",\"written_points\":" << points.size()
      << ",\"resolution\":" << tree->getResolution()
      << "}" << std::endl;
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << "{\"success\":false,\"error\":\"" << exc.what() << "\"}" << std::endl;
    return 1;
  }
}
