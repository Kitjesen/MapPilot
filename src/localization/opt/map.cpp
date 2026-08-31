#include "localization/opt/map.hpp"

#include <system_error>

namespace lingtu::localization::opt {
namespace {

bool path_is_regular_file(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(path, ec);
}

bool path_is_directory(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_directory(path, ec);
}

std::size_t count_pcd_patches(const std::filesystem::path& patches_dir) {
  std::error_code ec;
  std::size_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(patches_dir, ec)) {
    if (ec) {
      return 0;
    }
    if (entry.is_regular_file(ec) && entry.path().extension() == ".pcd") {
      ++count;
    }
  }
  return count;
}

}  // namespace

Map files(
    const std::filesystem::path& map_dir,
    const std::filesystem::path& output_dir) {
  Map artifacts;
  artifacts.map_dir = map_dir;
  artifacts.map_pcd = map_dir / "map.pcd";
  artifacts.poses_txt = map_dir / "poses.txt";
  artifacts.patches_dir = map_dir / "patches";
  artifacts.patch_bundle_manifest = map_dir / "patch_bundle.manifest";
  artifacts.output_dir = output_dir.empty() ? map_dir : output_dir;
  return artifacts;
}

Result check(const Map& map) {
  Result result;

  if (!path_is_directory(map.map_dir)) {
    result.code = "map_dir_missing";
    result.message = "map directory does not exist";
    return result;
  }
  if (!path_is_regular_file(map.map_pcd)) {
    result.code = "map_pcd_missing";
    result.message = "map.pcd does not exist";
    return result;
  }
  if (!path_is_regular_file(map.poses_txt)) {
    result.code = "poses_txt_missing";
    result.message = "poses.txt does not exist";
    return result;
  }
  if (!path_is_directory(map.patches_dir)) {
    result.code = "patches_dir_missing";
    result.message = "patches directory does not exist";
    return result;
  }

  result.patch_count = count_pcd_patches(map.patches_dir);
  if (result.patch_count == 0) {
    result.code = "patches_empty";
    result.message = "patches directory has no .pcd patches";
    return result;
  }

  result.ok = true;
  result.code = "ready";
  result.message = "native map artifacts are ready";
  return result;
}

}  // namespace lingtu::localization::opt
