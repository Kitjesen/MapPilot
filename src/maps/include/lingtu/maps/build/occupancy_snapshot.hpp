#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::maps {

struct OccupancySnapshotResult {
  bool ok{false};
  std::string message;
  std::filesystem::path occupancy_path;
  std::filesystem::path pgm_path;
  std::filesystem::path yaml_path;
  int rows{0};
  int cols{0};
  double resolution{0.2};
  double origin_x{0.0};
  double origin_y{0.0};
  int unknown_count{0};
  int free_count{0};
  int occupied_count{0};
};

OccupancySnapshotResult BuildOccupancyProjectionSnapshot(
    const std::filesystem::path& map_dir,
    bool output_is_staged = false);

}  // namespace lingtu::maps
