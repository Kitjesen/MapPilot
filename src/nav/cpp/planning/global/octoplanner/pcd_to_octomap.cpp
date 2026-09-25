#include <octomap/OcTree.h>
#include "lingtu/maps/build/sampled_octomap.hpp"
#include "lingtu/maps/build/ray_octomap.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(OCTOPLANNER3D_ENABLE_PCD)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#endif

namespace {

using namespace lingtu::maps::sampled_octomap;

struct CliConfig {
  std::string input;
  std::string output;
  double resolution = 0.2;
  int support_dilation_cells = 0;
  int free_layers_above = 0;
  int free_dilation_cells = 0;
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

CliConfig parseArgs(int argc, char ** argv)
{
  CliConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--input" || arg == "-i") {
      cfg.input = next();
    } else if (arg == "--output" || arg == "-o") {
      cfg.output = next();
    } else if (arg == "--resolution") {
      cfg.resolution = std::stod(next());
    } else if (arg == "--support-dilation-cells") {
      cfg.support_dilation_cells = std::stoi(next());
    } else if (arg == "--free-layers-above") {
      cfg.free_layers_above = std::stoi(next());
    } else if (arg == "--free-dilation-cells") {
      cfg.free_dilation_cells = std::stoi(next());
    } else if (arg == "--frame") {
      (void)next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
        "usage: octoplanner3d_pcd_to_octomap --input map.pcd --output octomap.ot "
        "[--resolution 0.2] [--support-dilation-cells 0] "
        "[--free-layers-above 0] [--free-dilation-cells 0] [--frame map]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (cfg.input.empty()) {
    throw std::runtime_error("missing --input");
  }
  if (cfg.output.empty()) {
    throw std::runtime_error("missing --output");
  }
  if (!(cfg.resolution > 0.0)) {
    throw std::runtime_error("--resolution must be positive");
  }
  if (cfg.free_layers_above < 0) {
    throw std::runtime_error("--free-layers-above must be non-negative");
  }
  if (cfg.support_dilation_cells < 0) {
    throw std::runtime_error("--support-dilation-cells must be non-negative");
  }
  if (cfg.free_dilation_cells < 0) {
    throw std::runtime_error("--free-dilation-cells must be non-negative");
  }
  return cfg;
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::vector<Point> readPcd(const std::string & path)
{
#if defined(OCTOPLANNER3D_ENABLE_PCD)
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) < 0) {
    throw std::runtime_error("failed to read input PCD: " + path);
  }
  std::vector<Point> points;
  points.reserve(cloud.size());
  for (const auto & point : cloud) {
    points.push_back({point.x, point.y, point.z});
  }
  return points;
#else
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open input PCD: " + path);
  }

  std::vector<std::string> fields;
  int x_index = -1;
  int y_index = -1;
  int z_index = -1;
  bool in_data = false;
  std::vector<Point> points;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    const std::string key_l = lower(key);
    if (!in_data) {
      if (key_l == "fields") {
        fields.clear();
        std::string field;
        while (iss >> field) {
          fields.push_back(lower(field));
        }
        for (std::size_t idx = 0; idx < fields.size(); ++idx) {
          if (fields[idx] == "x") {
            x_index = static_cast<int>(idx);
          } else if (fields[idx] == "y") {
            y_index = static_cast<int>(idx);
          } else if (fields[idx] == "z") {
            z_index = static_cast<int>(idx);
          }
        }
      } else if (key_l == "data") {
        std::string mode;
        iss >> mode;
        if (lower(mode) != "ascii") {
          throw std::runtime_error("only ASCII PCD is supported by this converter");
        }
        if (x_index < 0 || y_index < 0 || z_index < 0) {
          throw std::runtime_error("PCD must contain x y z fields");
        }
        in_data = true;
      }
      continue;
    }

    std::vector<double> values;
    std::istringstream row(line);
    double value = 0.0;
    while (row >> value) {
      values.push_back(value);
    }
    const int max_index = std::max({x_index, y_index, z_index});
    if (static_cast<int>(values.size()) <= max_index) {
      continue;
    }
    points.push_back({values[x_index], values[y_index], values[z_index]});
  }

  if (!in_data) {
    throw std::runtime_error("PCD DATA ascii section missing");
  }
  if (points.empty()) {
    throw std::runtime_error("PCD contains no valid xyz points");
  }
  return points;
#endif
}

bool hasSuffix(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
    value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    const std::vector<Point> points = readPcd(cfg.input);

    octomap::OcTree tree(cfg.resolution);
    std::unordered_set<VoxelKey, VoxelKeyHash> support_keys;
    const auto occupied_keys = buildOccupiedKeys(
      points,
      tree,
      cfg.support_dilation_cells,
      support_keys);
    if (occupied_keys.empty()) {
      throw std::runtime_error("PCD contains no finite points within OctoMap bounds");
    }
    const auto directory = std::filesystem::path(cfg.input).parent_path();
    const bool saved_rays = std::filesystem::is_regular_file(directory / "poses.txt") &&
                            std::filesystem::is_regular_file(directory / "scan_origin.txt");
    if (saved_rays) {
      lingtu::maps::PopulateSavedRayOctomap(tree, directory);
    } else {
    for (const auto & key : occupied_keys) {
      octomap::OcTreeKey octo_key;
      octo_key.k[0] = key.x;
      octo_key.k[1] = key.y;
      octo_key.k[2] = key.z;
      tree.updateNode(tree.keyToCoord(octo_key), true);
    }
    for (const auto & key : support_keys) {
      markFreeEnvelope(
        tree,
        key,
        cfg.free_layers_above,
        cfg.free_dilation_cells,
        occupied_keys);
    }
    }
    tree.updateInnerOccupancy();

    const bool ok = hasSuffix(lower(cfg.output), ".bt")
      ? tree.writeBinary(cfg.output)
      : tree.write(cfg.output);
    if (!ok) {
      std::cerr << "failed to write OctoMap: " << cfg.output << std::endl;
      return 3;
    }
    std::cout << "{\"success\":true,\"input\":\"" << cfg.input
              << "\",\"output\":\"" << cfg.output
              << "\",\"resolution\":" << cfg.resolution
              << ",\"support_dilation_cells\":" << cfg.support_dilation_cells
              << ",\"free_layers_above\":" << cfg.free_layers_above
              << ",\"free_dilation_cells\":" << cfg.free_dilation_cells
              << ",\"occupied_voxels\":" << lingtu::maps::OccupiedVoxelCount(tree)
              << ",\"evidence_source\":\"" << (saved_rays ? "saved_rays" : "sampled_points") << "\""
              << ",\"support_voxels\":" << support_keys.size()
              << ",\"points\":" << points.size()
              << ",\"converter\":\"octomap_sampled_pcd\"}" << std::endl;
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << std::endl;
    return 2;
  }
}
