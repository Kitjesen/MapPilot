#include <octomap/OcTree.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) try {
  if (argc != 4) {
    throw std::runtime_error("make_scenes boxes.txt map.bt occupied.xyz");
  }
  constexpr double resolution = 0.05;
  std::ifstream boxes(argv[1]);
  if (!boxes) throw std::runtime_error("cannot read boxes");
  std::set<std::array<int, 3>> occupied;
  std::array<double, 6> box{};
  while (boxes >> box[0] >> box[1] >> box[2] >> box[3] >> box[4] >> box[5]) {
    std::array<int, 3> lower{}, upper{};
    for (int axis = 0; axis < 3; ++axis) {
      lower[axis] = static_cast<int>(std::lround(box[axis] / resolution));
      upper[axis] = static_cast<int>(std::lround(box[axis + 3] / resolution));
      if (lower[axis] >= upper[axis]) throw std::runtime_error("empty box");
    }
    for (int x = lower[0]; x < upper[0]; ++x)
      for (int y = lower[1]; y < upper[1]; ++y)
        for (int z = lower[2]; z < upper[2]; ++z)
          occupied.insert({x, y, z});
  }
  if (!boxes.eof()) throw std::runtime_error("invalid box coordinates");
  octomap::OcTree tree(resolution);
  tree.updateNode(octomap::point3d(-1,-4,-4),false);
  tree.updateNode(octomap::point3d(32,4,12),false);
  std::ofstream points(argv[3]);
  if (!points) throw std::runtime_error("cannot write occupied points");
  points << "# occupied voxel centers, resolution=0.05m\n";
  for (const auto& cell : occupied) {
    const float x = static_cast<float>((cell[0] + .5) * resolution);
    const float y = static_cast<float>((cell[1] + .5) * resolution);
    const float z = static_cast<float>((cell[2] + .5) * resolution);
    tree.updateNode(octomap::point3d(x, y, z), true);
    points << x << ' ' << y << ' ' << z << '\n';
  }
  points.close();
  if (!points) throw std::runtime_error("cannot flush occupied points");
  tree.updateInnerOccupancy();
  if (!tree.writeBinary(argv[2])) throw std::runtime_error("cannot write map");
  std::cout << occupied.size() << '\n';
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
}
