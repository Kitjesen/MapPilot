#include "lingtu/maps/version.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "lingtu/maps/hash.hpp"

namespace lingtu::maps {
namespace {

constexpr const char *kManifestFilename = "save_manifest.json";
constexpr const char *kManifestHashFilename = "save_manifest.sha256";

std::string Trim(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
    value.pop_back();
  }
  const auto begin = std::find_if_not(value.begin(), value.end(),
                                      [](unsigned char ch) { return std::isspace(ch) != 0; });
  return std::string(begin, value.end());
}

bool IsSha256(const std::string &value) {
  return value.size() == 64U && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
           return std::isxdigit(ch) != 0;
         });
}

bool SafeRelativePath(const std::filesystem::path &path) {
  if (path.empty() || path.is_absolute() || !path.root_name().empty())
    return false;
  for (const auto &part : path) {
    if (part.empty() || part == "." || part == "..")
      return false;
  }
  return true;
}

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

}  // namespace

std::string BuildArtifactChecksums(const std::vector<std::pair<std::string, std::string>> &hashes) {
  std::ostringstream out;
  for (const auto &[path, hash] : hashes) {
    if (!IsSha256(hash) || path.empty() || path.find('\t') != std::string::npos ||
        path.find('\n') != std::string::npos || !SafeRelativePath(std::filesystem::path(path))) {
      throw std::invalid_argument("invalid artifact checksum entry: " + path);
    }
    out << hash << '\t' << path << '\n';
  }
  return out.str();
}

bool VerifyMapVersion(const std::filesystem::path &version_dir, std::string *error) {
  const auto manifest = version_dir / kManifestFilename;
  const auto manifest_hash = version_dir / kManifestHashFilename;
  const auto artifact_index = version_dir / kArtifactChecksumsFilename;
  if (!std::filesystem::is_regular_file(manifest) ||
      !std::filesystem::is_regular_file(manifest_hash) ||
      !std::filesystem::is_regular_file(artifact_index)) {
    return Fail(error, "version integrity metadata is incomplete");
  }

  std::ifstream root_hashes(manifest_hash, std::ios::binary);
  std::string expected_manifest;
  std::string expected_index;
  std::getline(root_hashes, expected_manifest);
  std::getline(root_hashes, expected_index);
  expected_manifest = Trim(expected_manifest);
  expected_index = Trim(expected_index);
  try {
    if (!IsSha256(expected_manifest) || Sha256File(manifest) != expected_manifest) {
      return Fail(error, "save manifest hash mismatch");
    }
    if (!IsSha256(expected_index) || Sha256File(artifact_index) != expected_index) {
      return Fail(error, "artifact checksum index hash mismatch");
    }
  } catch (const std::exception &exc) {
    return Fail(error, exc.what());
  }

  std::ifstream checksums(artifact_index, std::ios::binary);
  std::unordered_set<std::string> seen;
  bool has_map_pcd = false;
  std::string line;
  while (std::getline(checksums, line)) {
    if (line.empty())
      continue;
    const auto separator = line.find('\t');
    if (separator != 64U)
      return Fail(error, "invalid artifact checksum entry");
    const std::string expected = line.substr(0, separator);
    const std::string relative = line.substr(separator + 1U);
    const std::filesystem::path relative_path(relative);
    if (!IsSha256(expected) || !SafeRelativePath(relative_path) ||
        !seen.insert(relative_path.generic_string()).second) {
      return Fail(error, "invalid or duplicate artifact checksum path");
    }
    const auto artifact = version_dir / relative_path;
    if (!std::filesystem::is_regular_file(artifact)) {
      return Fail(error, "version artifact is missing: " + relative_path.generic_string());
    }
    try {
      if (Sha256File(artifact) != expected) {
        return Fail(error, "version artifact hash mismatch: " + relative_path.generic_string());
      }
    } catch (const std::exception &exc) {
      return Fail(error, exc.what());
    }
    has_map_pcd = has_map_pcd || relative_path.generic_string() == "map.pcd";
  }
  if (!has_map_pcd)
    return Fail(error, "artifact checksum index has no map.pcd");
  if (error != nullptr)
    error->clear();
  return true;
}

}  // namespace lingtu::maps
