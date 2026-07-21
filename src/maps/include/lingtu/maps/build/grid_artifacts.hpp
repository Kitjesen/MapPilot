#pragma once

#include <filesystem>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::maps {

struct GridArtifactResult {
  bool ok{false};
  std::string message;
  std::filesystem::path path;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

struct OccupancyArtifactData {
  std::vector<std::int8_t> grid;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

// Reads the uncompressed NPZ product emitted by the native maps pipeline.
// Compressed or malformed archives fail explicitly instead of taking a
// Python/numpy fallback path.
OccupancyArtifactData LoadOccupancyArtifact(
    const std::filesystem::path& path);

GridArtifactResult BuildEsdfArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged = false);
GridArtifactResult BuildTraversabilityArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged = false);

}  // namespace lingtu::maps
