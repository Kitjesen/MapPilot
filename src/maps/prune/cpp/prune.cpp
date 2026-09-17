#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "cleaner.hpp"

namespace {

void printHelp() {
  std::cout
      << "Usage: prune --map-dir MAP_DIR [options]\n"
      << "\n"
      << "LingTu-owned saved-map dynamic ghost pruner for S100P/MID-360 field maps.\n"
      << "It does not include, link, or require ERASOR2. Multiple later observed rays\n"
      << "must contradict a historical point before it can be removed.\n"
      << "\n"
      << "Options:\n"
      << "  --map-dir PATH              Saved map directory\n"
      << "  --out-clean PATH            Output cleaned PCD (default map.clean.pcd)\n"
      << "  --out-removed PATH          Output removed candidate PCD (default map.removed.pcd)\n"
      << "  --voxel-size M              Evidence voxel size (default 0.20)\n"
      << "  --sensor-origin X Y Z       LiDAR origin in scan coordinates; otherwise read scan_origin.txt\n"
      << "  --min-free-frames N         Later free-space frames required (default 3, minimum 2)\n"
      << "  --ground-z-threshold Z      Local scan ground protection threshold (default -0.45)\n"
      << "  --instance-grid-m M         XY instance score grid size (default 0.8)\n"
      << "  --moving-score-threshold R  Moving candidate ratio threshold (default 0.65)\n"
      << "  --min-frame-support N       Multi-frame support report threshold (default 2)\n"
      << "  --min-hit-support N         Hit support report threshold (default 3)\n"
      << "  --min-instance-points N     Minimum points before an instance can score moving "
         "(default 6)\n"
      << "  --dry-run                   Analyze and print JSON; write no files\n"
      << "  --apply                     Preserve the first map.pcd.preclean backup, then replace map.pcd\n"
      << "  --overwrite                 Replace existing output files\n"
      << "  --help                      Show this message\n";
}

bool nextValue(int &i, int argc, char **argv, std::string &value) {
  if (i + 1 >= argc) {
    return false;
  }
  value = argv[++i];
  return true;
}

}  // namespace

int main(int argc, char **argv) {
  lingtu::map_cleaning::StaticCleanerOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;
    if (arg == "--help" || arg == "-h") {
      printHelp();
      return 0;
    }
    if (arg == "--map-dir" && nextValue(i, argc, argv, value)) {
      options.map_dir = value;
    } else if (arg == "--out-clean" && nextValue(i, argc, argv, value)) {
      options.output_clean_pcd = value;
    } else if (arg == "--out-removed" && nextValue(i, argc, argv, value)) {
      options.output_removed_pcd = value;
    } else if (arg == "--voxel-size" && nextValue(i, argc, argv, value)) {
      options.voxel_size_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--sensor-origin" && i + 3 < argc) {
      std::array<float, 3> origin{};
      try {
        for (auto& coordinate : origin) {
          const std::string component = argv[++i];
          std::size_t parsed = 0;
          coordinate = std::stof(component, &parsed);
          if (parsed != component.size()) throw std::invalid_argument(component);
        }
      } catch (const std::exception&) {
        std::cerr << "--sensor-origin requires three numeric coordinates\n";
        return 2;
      }
      options.sensor_origin = origin;
    } else if (arg == "--min-free-frames" && nextValue(i, argc, argv, value)) {
      options.min_free_frames = static_cast<std::uint32_t>(std::strtoul(value.c_str(), nullptr, 10));
    } else if (arg == "--ground-z-threshold" && nextValue(i, argc, argv, value)) {
      options.ground_z_threshold = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--instance-grid-m" && nextValue(i, argc, argv, value)) {
      options.instance_grid_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--moving-score-threshold" && nextValue(i, argc, argv, value)) {
      options.moving_score_threshold = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--min-frame-support" && nextValue(i, argc, argv, value)) {
      options.min_frame_support =
          static_cast<std::uint32_t>(std::strtoul(value.c_str(), nullptr, 10));
    } else if (arg == "--min-hit-support" && nextValue(i, argc, argv, value)) {
      options.min_hit_support =
          static_cast<std::uint32_t>(std::strtoul(value.c_str(), nullptr, 10));
    } else if (arg == "--min-instance-points" && nextValue(i, argc, argv, value)) {
      options.min_instance_points =
          static_cast<std::uint32_t>(std::strtoul(value.c_str(), nullptr, 10));
    } else if (arg == "--apply") {
      options.apply_to_map = true;
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--overwrite") {
      options.overwrite = true;
    } else {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      printHelp();
      return 2;
    }
  }

  const auto result = lingtu::map_cleaning::cleanStaticMap(options);
  std::cout << lingtu::map_cleaning::toJson(result);
  return result.success ? 0 : 1;
}
