#include "lingtu/maps/patch_bundle.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string>

namespace lingtu::maps {
namespace {

bool IsBasename(const std::string &value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  const std::filesystem::path path(value);
  return path == path.filename();
}

bool IsNonEmptyRegularFile(const std::filesystem::path &path) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return false;
  }
  return std::filesystem::file_size(path, error) > 0U && !error;
}

}  // namespace

bool ReadPatchManifest(const std::filesystem::path &path, std::size_t *patch_count) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  const std::array<const char *, 6> expected_keys = {
      "LINGTU_PATCH_BUNDLE_V1", "complete",      "dropped_count",
      "first_sequence",         "last_sequence", "patch_count",
  };
  std::array<std::uint64_t, 5> values{};
  std::string line;
  if (!std::getline(input, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  if (line != expected_keys[0]) {
    return false;
  }
  for (std::size_t index = 1; index < expected_keys.size(); ++index) {
    if (!std::getline(input, line)) {
      return false;
    }
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::istringstream row(line);
    std::string key;
    std::string token;
    std::string extra;
    std::uint64_t parsed = 0U;
    if (!(row >> key >> token) || row >> extra || key != expected_keys[index] || token.empty() ||
        token.front() == '-') {
      return false;
    }
    const auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
      return false;
    }
    values[index - 1] = parsed;
  }
  if (std::getline(input, line)) {
    return false;
  }
  if (values[0] != 1U || values[1] != 0U || values[4] == 0U || values[2] != 0U ||
      values[3] != values[4] - 1U || values[4] > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  *patch_count = static_cast<std::size_t>(values[4]);
  return true;
}

bool HasCompletePatchBundle(const std::filesystem::path &source) {
  const auto patches = source / "patches";
  if (!IsNonEmptyRegularFile(source / "map.pcd") || !IsNonEmptyRegularFile(source / "poses.txt") ||
      !IsNonEmptyRegularFile(source / "patch_bundle.manifest") ||
      !std::filesystem::is_directory(patches)) {
    return false;
  }
  std::size_t manifest_patch_count = 0U;
  if (!ReadPatchManifest(source / "patch_bundle.manifest", &manifest_patch_count)) {
    return false;
  }
  std::set<std::string> disk_patches;
  for (const auto &entry : std::filesystem::directory_iterator(patches)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd" &&
        IsNonEmptyRegularFile(entry.path())) {
      disk_patches.insert(entry.path().filename().string());
    }
  }
  std::set<std::string> pose_patches;
  std::ifstream poses(source / "poses.txt", std::ios::binary);
  std::string line;
  while (std::getline(poses, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::istringstream row(line);
    std::string filename;
    std::array<double, 7> pose{};
    std::string extra;
    if (!(row >> filename) || !IsBasename(filename) ||
        std::filesystem::path(filename).extension() != ".pcd") {
      return false;
    }
    for (double &value : pose) {
      if (!(row >> value) || !std::isfinite(value))
        return false;
    }
    if (row >> extra || !pose_patches.insert(filename).second)
      return false;
  }
  return !pose_patches.empty() && pose_patches == disk_patches &&
         pose_patches.size() == manifest_patch_count;
}

bool HasSavedRays(const std::filesystem::path &dir) {
  // A replay is only navigation-grade when every pose has a corresponding
  // non-empty patch and the manifest proves that no sequence was dropped.
  // Checking only the marker files can publish a map whose geometry has no
  // scans to replay; it then looks ready until the build fails on the field.
  return IsNonEmptyRegularFile(dir / "scan_origin.txt") &&
         HasCompletePatchBundle(dir);
}

}  // namespace lingtu::maps
