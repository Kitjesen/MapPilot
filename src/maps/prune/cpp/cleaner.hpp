#pragma once

#include <cstdint>
#include <array>
#include <filesystem>
#include <optional>
#include <string>

namespace lingtu::map_cleaning {

struct StaticCleanerOptions {
  std::filesystem::path map_dir;
  std::filesystem::path output_clean_pcd;
  std::filesystem::path output_removed_pcd;
  std::optional<std::array<float, 3>> sensor_origin;
  std::uint32_t min_free_frames{3};
  float ray_tolerance_m{0.05F};
  float endpoint_confirm_radius_m{0.05F};
  float endpoint_margin_m{0.20F};
  float max_ray_length_m{20.0F};
  float voxel_size_m{0.20F};
  bool overwrite{false};
  bool apply_to_map{false};
  bool dry_run{false};
};

struct StaticCleanerResult {
  bool success{false};
  std::string reason_code;
  std::string message;
  std::filesystem::path clean_pcd;
  std::filesystem::path removed_pcd;
  std::filesystem::path backup_pcd;
  bool dry_run{false};
  bool applied{false};
  std::uint64_t unmatched_patch_count{0};
  std::uint64_t patch_count{0};
  std::uint64_t pose_count{0};
  std::uint64_t source_points{0};
  std::uint64_t kept_points{0};
  std::uint64_t removed_points{0};
  std::uint64_t kept_uncontradicted_points{0};
  std::uint64_t kept_supported_surface_points{0};
  std::uint64_t free_space_candidate_points{0};
};

StaticCleanerResult cleanStaticMap(const StaticCleanerOptions &options);

std::string toJson(const StaticCleanerResult &result);

}  // namespace lingtu::map_cleaning
