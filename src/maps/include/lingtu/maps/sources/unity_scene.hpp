#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "lingtu/maps/layers/semantic_occupancy.hpp"

namespace lingtu::maps::sources {

struct UnitySemanticImportConfig {
  std::filesystem::path categories_path{"environment/Categories.csv"};
  std::filesystem::path objects_path{"object_list.txt"};
  std::filesystem::path taxonomy_path;
  std::string frame_id{"map"};
  float voxel_size_m{0.20F};
  float occupied_probability{0.95F};
  float shell_thickness_voxels{0.75F};
  std::uint64_t generation{1U};
  std::size_t max_objects{100'000U};
  std::size_t max_voxels{2'000'000U};
  std::size_t max_voxel_checks{50'000'000U};
  bool include_unknown_geometry{false};
  bool exclude_dynamic_classes{true};
};

struct UnitySemanticImportStats {
  std::size_t category_rows{0U};
  std::size_t object_rows{0U};
  std::size_t accepted_objects{0U};
  std::size_t skipped_unmapped_objects{0U};
  std::size_t skipped_dynamic_objects{0U};
  std::size_t candidate_voxel_checks{0U};
  std::size_t semantic_conflicts{0U};
  std::size_t output_voxels{0U};
  std::vector<std::string> unmapped_labels;
};

struct UnitySemanticImportResult {
  layers::SemanticMapChunk chunk;
  UnitySemanticImportStats stats;
};

UnitySemanticImportResult BuildUnitySemanticMap(
    const std::filesystem::path& scene_dir,
    const UnitySemanticImportConfig& config);

UnitySemanticImportStats ImportUnitySemanticMap(
    const std::filesystem::path& scene_dir,
    const std::filesystem::path& output_path,
    const UnitySemanticImportConfig& config);

}  // namespace lingtu::maps::sources
