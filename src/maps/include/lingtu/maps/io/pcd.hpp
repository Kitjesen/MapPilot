#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::maps {

struct PointXyz {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct PcdBounds {
  bool enabled{false};
  double min_x{0.0};
  double min_y{0.0};
  double min_z{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double max_z{0.0};
};

struct PcdFilterOptions {
  double voxel_size{0.0};
  PcdBounds bounds;
  bool invert_bounds{false};
};

struct PcdIoResult {
  bool ok{false};
  std::string message;
  std::vector<PointXyz> points;
};

PcdIoResult LoadPcdXyz(const std::filesystem::path& path);
bool WriteBinaryXyzPcd(
    const std::filesystem::path& path,
    const std::vector<PointXyz>& points,
    std::string* error);
std::vector<PointXyz> FilterPcdPoints(
    const std::vector<PointXyz>& points,
    const PcdFilterOptions& options);

}  // namespace lingtu::maps
