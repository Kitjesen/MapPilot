#include <octomap/OcTree.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(OCTOPLANNER3D_ENABLE_PCD)
#include "pcd2octomap_converter.h"
#endif

namespace {

struct CliConfig {
  std::string input;
  std::string output;
  double resolution = 0.2;
  int support_dilation_cells = 1;
  int free_layers_above = 3;
  int free_dilation_cells = 1;
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

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
        "[--resolution 0.2] [--support-dilation-cells 1] "
        "[--free-layers-above 3] [--free-dilation-cells 1] [--frame map]");
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

std::vector<Point> readAsciiPcd(const std::string & path)
{
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
}

void markFreeEnvelope(
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

std::unordered_set<VoxelKey, VoxelKeyHash> horizontalSupportKeys(
  const std::unordered_set<VoxelKey, VoxelKeyHash> & occupied)
{
  std::unordered_set<VoxelKey, VoxelKeyHash> supports;
  supports.reserve(occupied.size());
  constexpr int kMinimumProbeDistance = 2;
  constexpr int kProbeRadius = 4;
  constexpr int kLateralTolerance = 2;
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

  for (const auto & key : occupied) {
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

std::unordered_set<VoxelKey, VoxelKeyHash> buildFilteredOccupiedKeys(
  const std::vector<Point> & points,
  octomap::OcTree & tree,
  int support_dilation_cells,
  std::unordered_set<VoxelKey, VoxelKeyHash> & support_keys)
{
  std::unordered_map<VoxelKey, int, VoxelKeyHash> counts;
  for (const Point & point : points) {
    octomap::OcTreeKey raw_key;
    if (!tree.coordToKeyChecked(point.x, point.y, point.z, raw_key)) {
      continue;
    }
    ++counts[VoxelKey{raw_key.k[0], raw_key.k[1], raw_key.k[2]}];
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(counts.size());
  // MuJoCo/Livox scan noise can place isolated single-hit voxels in the robot
  // body envelope. Require repeated hits before a voxel becomes occupied.
  constexpr int kMinPointsPerVoxel = 2;
  for (const auto & entry : counts) {
    if (entry.second >= kMinPointsPerVoxel) {
      occupied.insert(entry.first);
    }
  }

  support_keys = horizontalSupportKeys(occupied);
  if (support_dilation_cells <= 0 || occupied.empty()) {
    return occupied;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> dilated = occupied;
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
        dilated.insert(dilated_key);
        support_keys.insert(dilated_key);
      }
    }
  }
  return dilated;
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
#if defined(OCTOPLANNER3D_ENABLE_PCD)
    pcd2octomap::Pcd2OctomapConverter converter;
    converter.setInputPcdFile(cfg.input);
    converter.setOutputBtFile(cfg.output);
    converter.setResolution(cfg.resolution);
    converter.setSupportDilationCells(cfg.support_dilation_cells);
    converter.setFreeEnvelopeLayers(cfg.free_layers_above);
    converter.setFreeEnvelopeDilationCells(cfg.free_dilation_cells);
    if (!converter.convert()) {
      std::cerr << "failed to convert PCD to OctoMap: " << cfg.input << std::endl;
      return 3;
    }
    std::cout << "{\"success\":true,\"input\":\"" << cfg.input
              << "\",\"output\":\"" << cfg.output
              << "\",\"resolution\":" << cfg.resolution
              << ",\"support_dilation_cells\":" << cfg.support_dilation_cells
              << ",\"free_layers_above\":" << cfg.free_layers_above
              << ",\"free_dilation_cells\":" << cfg.free_dilation_cells
              << ",\"converter\":\"octoplanner3d_pcl_pcd\"}" << std::endl;
    return 0;
#else
    const std::vector<Point> points = readAsciiPcd(cfg.input);

    octomap::OcTree tree(cfg.resolution);
    std::unordered_set<VoxelKey, VoxelKeyHash> support_keys;
    const auto occupied_keys = buildFilteredOccupiedKeys(
      points,
      tree,
      cfg.support_dilation_cells,
      support_keys);
    for (const auto & key : occupied_keys) {
      octomap::OcTreeKey octo_key;
      octo_key.k[0] = key.x;
      octo_key.k[1] = key.y;
      octo_key.k[2] = key.z;
      tree.updateNode(tree.keyToCoord(octo_key), true);
    }
    for (const auto & key : occupied_keys) {
      markFreeEnvelope(
        tree,
        key,
        cfg.free_layers_above,
        cfg.free_dilation_cells,
        occupied_keys);
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
              << ",\"occupied_voxels\":" << occupied_keys.size()
              << ",\"support_voxels\":" << support_keys.size()
              << ",\"points\":" << points.size()
              << ",\"converter\":\"octomap_ascii_pcd\"}" << std::endl;
    return 0;
#endif
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << std::endl;
    return 2;
  }
}
