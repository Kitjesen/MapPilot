#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace lingtu::localization::opt {

struct Map {
  std::filesystem::path map_dir;
  std::filesystem::path map_pcd;
  std::filesystem::path poses_txt;
  std::filesystem::path patches_dir;
  std::filesystem::path output_dir;
};

struct Result {
  bool ok = false;
  std::string code;
  std::string message;
  std::size_t patch_count = 0;
  std::size_t pose_count = 0;
  std::size_t factor_count = 0;
  std::size_t iterations = 0;
  bool changed = false;
  std::filesystem::path report_path;
};

Map files(
    const std::filesystem::path& map_dir,
    const std::filesystem::path& output_dir = {});

Result check(const Map& map);

}  // namespace lingtu::localization::opt
