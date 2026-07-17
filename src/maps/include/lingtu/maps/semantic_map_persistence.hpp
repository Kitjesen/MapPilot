#pragma once

#include <filesystem>
#include <string>

#include "lingtu/maps/layers/semantic_occupancy.hpp"

namespace lingtu::maps {

inline constexpr const char* kSemanticMapArtifactFilename = "semantic_map.bin";
inline constexpr std::uint32_t kSemanticMapBinarySchemaVersion = 1U;

struct SemanticMapReadOptions {
  std::uint64_t max_voxels{2'000'000U};
  std::uint64_t max_body_bytes{1024ULL * 1024ULL * 1024ULL};
};

layers::SemanticMapChunk ReadSemanticMapBinary(
    const std::filesystem::path& path,
    const SemanticMapReadOptions& options = {});

void WriteSemanticMapBinaryAtomic(
    const std::filesystem::path& path,
    const layers::SemanticMapChunk& chunk);

bool ValidateSemanticMapBinary(
    const std::filesystem::path& path,
    std::string* error = nullptr,
    const SemanticMapReadOptions& options = {});

}  // namespace lingtu::maps
