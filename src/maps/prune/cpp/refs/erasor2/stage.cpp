#include <cstdlib>
#include <iostream>
#include <string>

#include "stager.hpp"

namespace {

void printHelp() {
  std::cout
      << "Usage: erasor2_stage --map-dir MAP_DIR --out STAGE_DIR [options]\n"
      << "\n"
      << "Converts a LingTu saved-map directory into an ERASOR2-style staging dataset.\n"
      << "This does not link to ERASOR2 or run GPLv3 upstream code.\n"
      << "\n"
      << "Options:\n"
      << "  --map-dir PATH              LingTu saved map with map.pcd, patches/, poses.txt\n"
      << "  --out PATH                  Output staging directory\n"
      << "  --ground-z-threshold Z      Bootstrap ground threshold in lidar frame (default -1.25)\n"
      << "  --instance-grid-m M         Bootstrap non-ground XY instance grid size (default 0.8)\n"
      << "  --range-of-interest M       ERASOR2 range_of_interest config value (default 60)\n"
      << "  --grid-resolution M         ERASOR2 grid_resolution config value (default 2)\n"
      << "  --sensor-height M           ERASOR2 sensor_height config value (default 0.55)\n"
      << "  --overwrite                 Replace an existing staging directory\n"
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
  lingtu::map_cleaning::Erasor2StageOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;
    if (arg == "--help" || arg == "-h") {
      printHelp();
      return 0;
    }
    if (arg == "--map-dir" && nextValue(i, argc, argv, value)) {
      options.map_dir = value;
    } else if (arg == "--out" && nextValue(i, argc, argv, value)) {
      options.output_dir = value;
    } else if (arg == "--ground-z-threshold" && nextValue(i, argc, argv, value)) {
      options.ground_z_threshold = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--instance-grid-m" && nextValue(i, argc, argv, value)) {
      options.instance_grid_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--range-of-interest" && nextValue(i, argc, argv, value)) {
      options.range_of_interest_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--grid-resolution" && nextValue(i, argc, argv, value)) {
      options.grid_resolution_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--sensor-height" && nextValue(i, argc, argv, value)) {
      options.sensor_height_m = std::strtof(value.c_str(), nullptr);
    } else if (arg == "--overwrite") {
      options.overwrite = true;
    } else {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      printHelp();
      return 2;
    }
  }

  const auto result = lingtu::map_cleaning::stageErasor2Dataset(options);
  std::cout << lingtu::map_cleaning::toJson(result);
  return result.success ? 0 : 1;
}
