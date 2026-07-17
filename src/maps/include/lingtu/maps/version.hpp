#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::maps {

inline constexpr const char *kArtifactChecksumsFilename = "artifact_checksums.sha256";

std::string BuildArtifactChecksums(const std::vector<std::pair<std::string, std::string>> &hashes);

bool VerifyMapVersion(const std::filesystem::path &version_dir, std::string *error = nullptr);

}  // namespace lingtu::maps
