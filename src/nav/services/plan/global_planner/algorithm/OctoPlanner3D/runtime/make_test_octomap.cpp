#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

struct Args {
  std::string output;
  std::string scenario = "two_floor_stairs";
  double resolution = 0.2;
};

struct Scene {
  const char * name = "two_floor_stairs";
  double start_x = -5.0;
  double start_y = -3.0;
  double start_z = 0.3;
  double goal_x = 5.0;
  double goal_y = 3.0;
  double goal_z = 1.5;
};

void usage()
{
  std::cerr
    << "usage: octoplanner3d_make_test_octomap --output map.bt "
    << "[--scenario two_floor_stairs|spiral_stairs] [--resolution 0.2]\n";
}

bool readDouble(const char * text, double & out)
{
  char * end = nullptr;
  out = std::strtod(text, &end);
  return end != text && *end == '\0' && std::isfinite(out);
}

bool parse(int argc, char ** argv, Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--output" && i + 1 < argc) {
      args.output = argv[++i];
    } else if (key == "--scenario" && i + 1 < argc) {
      args.scenario = argv[++i];
    } else if (key == "--resolution" && i + 1 < argc) {
      if (!readDouble(argv[++i], args.resolution)) {
        return false;
      }
    } else {
      return false;
    }
  }
  return !args.output.empty() &&
    (args.scenario == "two_floor_stairs" || args.scenario == "spiral_stairs") &&
    args.resolution > 0.05 && args.resolution <= 1.0;
}

void occupy(octomap::OcTree & tree, double x, double y, double z)
{
  tree.updateNode(octomap::point3d(x, y, z), true);
}

double center(int index, double resolution)
{
  return (static_cast<double>(index) + 0.5) * resolution;
}

void occupyCell(octomap::OcTree & tree, double resolution, int ix, int iy, int iz)
{
  occupy(tree, center(ix, resolution), center(iy, resolution), center(iz, resolution));
}

void addFloor(
  octomap::OcTree & tree,
  double resolution,
  int ix_min,
  int ix_max,
  int iy_min,
  int iy_max,
  int iz)
{
  for (int ix = ix_min; ix <= ix_max; ++ix) {
    for (int iy = iy_min; iy <= iy_max; ++iy) {
      occupyCell(tree, resolution, ix, iy, iz);
    }
  }
}

void addStairs(octomap::OcTree & tree, double resolution)
{
  constexpr int step_count = 7;
  constexpr int start_ix = -6;
  constexpr int cells_per_step = 2;
  for (int step = 0; step < step_count; ++step) {
    const int ix_min = start_ix + step * cells_per_step;
    const int ix_max = ix_min + cells_per_step - 1;
    for (int ix = ix_min; ix <= ix_max; ++ix) {
      for (int iy = -7; iy <= 7; ++iy) {
        occupyCell(tree, resolution, ix, iy, step);
      }
    }
  }
}

void addPillar(
  octomap::OcTree & tree,
  double resolution,
  int ix,
  int iy,
  int iz_min,
  int iz_max)
{
  for (int iz = iz_min; iz <= iz_max; ++iz) {
    occupyCell(tree, resolution, ix, iy, iz);
    occupyCell(tree, resolution, ix + 1, iy, iz);
    occupyCell(tree, resolution, ix, iy + 1, iz);
    occupyCell(tree, resolution, ix + 1, iy + 1, iz);
  }
}

void buildTwoFloorStairs(octomap::OcTree & tree, double resolution)
{
  addFloor(tree, resolution, -30, -7, -20, 20, 0);
  addStairs(tree, resolution);
  addFloor(tree, resolution, 6, 30, -20, 20, 6);
  addPillar(tree, resolution, -18, 12, 1, 5);
  addPillar(tree, resolution, 24, -18, 7, 12);
}

void addSpiralStairs(octomap::OcTree & tree, double resolution)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr int radius_cells = 10;
  constexpr int half_width_cells = 2;
  constexpr int max_step = 18;
  constexpr int cells_per_step = 8;
  constexpr int samples = max_step * cells_per_step;

  addFloor(tree, resolution, -16, -8, -5, 5, 0);
  addFloor(tree, resolution, -4, 4, 7, 15, max_step / 2);
  addFloor(tree, resolution, 8, 16, -5, 5, max_step);

  for (int i = 0; i <= samples; ++i) {
    const double theta = pi + (3.0 * pi * static_cast<double>(i)) /
      static_cast<double>(samples);
    const int iz = std::min(max_step, i / cells_per_step);
    for (int dr = -half_width_cells; dr <= half_width_cells; ++dr) {
      const double radius = static_cast<double>(radius_cells + dr);
      const int ix = static_cast<int>(std::lround(radius * std::cos(theta)));
      const int iy = static_cast<int>(std::lround(radius * std::sin(theta)));
      occupyCell(tree, resolution, ix, iy, iz);
    }
  }

  // Extend the map bounds and make the scene read visually as a spiral stair.
  for (int iz = 0; iz <= max_step + 4; ++iz) {
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        occupyCell(tree, resolution, ix, iy, iz);
      }
    }
  }
}

Scene buildScene(octomap::OcTree & tree, const Args & args)
{
  if (args.scenario == "spiral_stairs") {
    addSpiralStairs(tree, args.resolution);
    return Scene{
      "spiral_stairs",
      center(-14, args.resolution),
      center(0, args.resolution),
      center(1, args.resolution),
      center(14, args.resolution),
      center(0, args.resolution),
      center(19, args.resolution)};
  }

  buildTwoFloorStairs(tree, args.resolution);
  return Scene{};
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse(argc, argv, args)) {
    usage();
    return 2;
  }

  octomap::OcTree tree(args.resolution);
  tree.setProbHit(0.7);
  tree.setProbMiss(0.4);
  tree.setClampingThresMin(0.12);
  tree.setClampingThresMax(0.97);

  const Scene scene = buildScene(tree, args);
  tree.updateInnerOccupancy();

  if (!tree.writeBinary(args.output)) {
    std::cerr << "failed to write output: " << args.output << "\n";
    return 3;
  }

  std::cout
    << "{\"ok\":true,"
    << "\"map_path\":\"" << args.output << "\","
    << "\"scenario\":\"" << scene.name << "\","
    << "\"resolution\":" << args.resolution << ","
    << "\"occupied_leafs\":" << tree.size() << ","
    << "\"start\":[" << scene.start_x << "," << scene.start_y << "," << scene.start_z << "],"
    << "\"goal\":[" << scene.goal_x << "," << scene.goal_y << "," << scene.goal_z << "]}"
    << "\n";
  return 0;
}
