#pragma once

#include <filesystem>
#include <string>

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

GridArtifactResult BuildEsdfArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged = false);
GridArtifactResult BuildTraversabilityArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged = false);

}  // namespace lingtu::maps
