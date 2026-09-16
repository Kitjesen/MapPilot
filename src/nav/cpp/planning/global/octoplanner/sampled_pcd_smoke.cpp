#include <octomap/OcTree.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Point = std::array<float, 3>;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void writePcd(const fs::path& path, const std::vector<Point>& points, bool binary) {
  std::ofstream out(path, std::ios::binary);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
      << points.size() << "\nHEIGHT 1\nPOINTS " << points.size()
      << "\nDATA " << (binary ? "binary" : "ascii") << '\n';
  for (const auto& p : points) {
    if (binary) out.write(reinterpret_cast<const char*>(p.data()), 3 * sizeof(float));
    else out << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
  }
}

int convert(const std::string& executable, const fs::path& input, const fs::path& output,
            int dilation = 0) {
  std::string command = "\"" + executable + "\" --input \"" + input.string() +
      "\" --output \"" + output.string() +
      "\" --resolution 0.2 --support-dilation-cells " + std::to_string(dilation) +
      " --free-layers-above 3 --free-dilation-cells " + std::to_string(dilation);
#if defined(_WIN32)
  command = "\"" + command + "\"";
#endif
  return std::system(command.c_str());
}

int main(int argc, char** argv) {
  try {
    require(argc == 4, "usage: sampled_pcd_smoke CONVERTER WORKDIR HAS_PCL");
    const fs::path root(argv[2]);
    fs::create_directories(root);
    // Keep OcTree's type registered when OctoMap is linked as a static library.
    octomap::OcTree registration(0.2);
    std::vector<Point> points;
    for (int x = -4; x <= 4; ++x)
      for (int y = -4; y <= 4; ++y)
        points.push_back({x * 0.2F + 0.1F, y * 0.2F + 0.1F, -0.3F});
    // A thin isolated obstacle must survive too, without becoming a floor.
    points.push_back({3.1F, 3.1F, 1.1F});
    // An obstacle inside the floor's free envelope must remain occupied.
    points.push_back({0.1F, 0.1F, 0.1F});
    // A vertical wall away from the floor must not acquire support padding.
    for (int y = -4; y <= 4; ++y)
      for (int z = -2; z <= 6; ++z)
        points.push_back({4.1F, y * 0.2F + 0.1F, z * 0.2F + 0.1F});
    // Nearby walls must not collectively masquerade as a horizontal surface.
    for (float x : {6.1F, 6.9F})
      for (int y = -4; y <= 4; ++y)
        for (int z = -2; z <= 6; ++z)
          points.push_back({x, y * 0.2F + 0.1F, z * 0.2F + 0.1F});
    // Adjacent treads can differ by one voxel without becoming a wall.
    for (int x = 0; x < 6; ++x)
      for (int y = -4; y <= 4; ++y)
        points.push_back({10.1F + x * 0.2F, y * 0.2F + 0.1F, -0.3F + (x / 2) * 0.2F});
    // A raised object must not grow sideways over already observed lower floor.
    for (int x = 0; x < 9; ++x)
      for (int y = 0; y < 5; ++y)
        points.push_back({14.1F + x * 0.2F, y * 0.2F + 0.1F, -0.3F});
    for (int x = 0; x < 5; ++x)
      for (int y = 0; y < 5; ++y)
        points.push_back({14.1F + x * 0.2F, y * 0.2F + 0.1F, 0.5F});
    for (bool binary : {false, true}) {
      if (binary && std::string(argv[3]) != "1") continue;
      for (int dilation : {0, 1}) {
        const std::string name = binary ? "sampled_binary" : "sampled_ascii";
        const auto input = root / (name + ".pcd");
        const auto output = root / (name + "_d" + std::to_string(dilation) + ".ot");
        writePcd(input, points, binary);
        require(convert(argv[1], input, output, dilation) == 0, "sampled cloud conversion failed");
        std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(output.string()));
        const auto* tree = dynamic_cast<const octomap::OcTree*>(raw.get());
        require(tree && tree->size() > 0, "sampled map became empty");
        for (const auto& point : points) {
          const auto* node = tree->search(point[0], point[1], point[2]);
          require(node && tree->isNodeOccupied(node), "sampled surface or thin obstacle was erased");
        }
        const auto* free_above_floor = tree->search(0.1, 0.1, -0.1);
        require(free_above_floor && !tree->isNodeOccupied(free_above_floor), "floor free envelope missing");
        require(tree->search(3.1, 3.1, 1.3) == nullptr, "isolated obstacle incorrectly created free space");
        const auto* floor_edge = tree->search(1.1, 0.1, -0.3);
        require(dilation ? floor_edge && tree->isNodeOccupied(floor_edge) : floor_edge == nullptr,
                "floor support padding does not match configured radius");
        require(tree->search(1.3, 0.1, -0.3) == nullptr, "support padding exceeded one cell");
        require(tree->search(4.3, 0.1, 0.1) == nullptr, "vertical wall incorrectly became support");
        require(tree->search(4.1, 0.1, 1.5) == nullptr, "vertical wall incorrectly created free space");
        require(tree->search(6.3, 0.1, 0.1) == nullptr, "nearby walls incorrectly created support padding");
        require(tree->search(6.1, 0.1, 1.5) == nullptr, "nearby walls incorrectly created free space");
        const auto* free_above_tread = tree->search(10.5, 0.1, 0.3);
        require(free_above_tread && !tree->isNodeOccupied(free_above_tread), "stair tread support was lost");
        const auto* invented_shelf = tree->search(15.1, 0.5, 0.5);
        require(!invented_shelf || !tree->isNodeOccupied(invented_shelf),
                "raised support padding created a shelf over observed lower floor");
      }
    }
    const auto bad_input = root / "outside_tree.pcd";
    const auto bad_output = root / "outside_tree.ot";
    fs::remove(bad_output);
    writePcd(bad_input, {{1e30F, 1e30F, 1e30F}}, false);
    require(convert(argv[1], bad_input, bad_output) != 0, "empty OctoMap reported success");
    require(!fs::exists(bad_output), "empty OctoMap was written");
    std::cout << "sampled PCD conversion checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
