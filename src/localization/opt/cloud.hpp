#pragma once

#include <filesystem>
#include <vector>

namespace lingtu::localization::opt {

struct Point {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

std::vector<Point> read_point_cloud(const std::filesystem::path &path);

std::vector<std::filesystem::path> sorted_point_cloud_files(const std::filesystem::path &directory);

}  // namespace lingtu::localization::opt
