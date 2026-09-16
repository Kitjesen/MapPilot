#include "status/planning_map_writer.hpp"

#include <octomap/OcTree.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace {
std::string read(const std::filesystem::path &path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
void expect(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char **argv) {
  try {
    const auto root = std::filesystem::temp_directory_path() /
        ("lingtu-planning-map-test-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root / "fixture-map");
    auto map_path = root / "fixture-map" / "octomap.ot";
    auto status_path = root / "nav.status.json";
    double reference_z = 0.1;
    if (argc == 4) {
      // Optional offline replay of a saved field map using the deployed Go2 envelope.
      map_path = argv[1];
      status_path = argv[3];
      reference_z = std::stod(argv[2]);
    } else {
      octomap::OcTree tree(0.2);
      for (int x = -5; x <= 5; ++x)
        for (int y = -5; y <= 5; ++y) {
          tree.updateNode(octomap::point3d(static_cast<float>((x + 0.5) * 0.2),
              static_cast<float>((y + 0.5) * 0.2), -0.1F), true);
          tree.updateNode(octomap::point3d(static_cast<float>((x + 0.5) * 0.2),
              static_cast<float>((y + 0.5) * 0.2), 0.5F), false);
        }
      expect(tree.write(map_path.string()), "fixture map write failed");
    }
    const lingtu::nav::plan::MapIdentity identity{map_path.parent_path().filename().string(), 1, "map"};
    auto gate = std::make_shared<lingtu::nav::endpoint::ActiveOctomapGate>(identity);
    lingtu::nav::plan::GlobalPlannerOptions options;
    options.robot_radius = argc == 4 ? 0.43 : 0.2;
    options.ground_support_depth_cells = 2;
    const auto sidecar = status_path.string() + ".planning-map.json";
    {
      lingtu::nav::endpoint::PlanningMapWriter writer(
          status_path.string(), "offline-projection-test", gate, map_path.string(), options);
      writer.update(identity, reference_z, 123.0);
      writer.flush();
      const auto first = read(sidecar);
      expect(first.find("\"available\":true") != std::string::npos,
             "native writer did not export the ready map");
      expect(first.find("\"product_session_id\":\"offline-projection-test\"") != std::string::npos &&
             first.find("\"map_content_epoch\":1") != std::string::npos,
             "native projection lost identity");
      writer.update(identity, reference_z + 0.001, 124.0);
      writer.flush();
      expect(read(sidecar) == first, "same-layer update rebuilt the static projection timestamp");
      if (argc != 4) {
        writer.update(identity, std::nullopt, 125.0);
        writer.flush();
        expect(read(sidecar).find("\"available\":false") != std::string::npos,
               "missing localization left a valid projection in the sidecar");
        writer.update(std::nullopt, reference_z, 126.0);
        writer.flush();
        expect(read(sidecar).find("\"reason\":\"active_map_not_ready\"") != std::string::npos,
               "missing map identity was not reflected in sidecar");
      }
    }
    std::filesystem::remove_all(root);
    std::cout << "native planning map writer identity, cache timestamp and invalidation passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
