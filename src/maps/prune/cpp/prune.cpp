#include <cstdlib>
#include <iostream>
#include <string>

#include "cleaner.hpp"

namespace {

void printHelp() {
  std::cout
      << "Usage: prune --map-dir MAP_DIR [options]\n"
      << "\n"
      << "LingTu-owned saved-map dynamic ghost pruner for S100P/MID-360 field maps.\n"
      << "It does not include, link, or require ERASOR2. It uses conservative temporal\n"
      << "occupancy voting over map.pcd + patches/*.pcd + poses.txt.\n"
      << "\n"
      << "Options:\n"
      << "  --map-dir PATH              Saved map directory\n"
      << "  --out-clean PATH            Output cleaned PCD (default map.clean.pcd)\n"
      << "  --out-removed PATH          Output removed candidate PCD (default map.removed.pcd)\n"
      << "  --voxel-size M              Evidence voxel size (default 0.20)\n"
      << "  --ground-z-threshold Z      Local scan ground protection threshold (default -0.45)\n"
      << "  --instance-grid-m M         XY instance score grid size (default 0.8)\n"
      << "  --moving-score-threshold R  Moving candidate ratio threshold (default 0.65)\n"
      << "  --min-frame-support N       Keep voxels seen in at least N frames (default 2)\n"
      << "  --min-hit-support N         Keep voxels with at least N hits (default 3)\n"
      << "  --min-instance-points N     Minimum points before an instance can score moving "
         "(default 6)\n"
      << "  --apply                     Backup map.pcd to map.pcd.preclean, then replace map.pcd\n"
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
