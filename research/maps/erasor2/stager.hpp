#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::map_cleaning {

struct Erasor2StageOptions {
  std::filesystem::path map_dir;
  std::filesystem::path output_dir;
  float ground_z_threshold{-1.25F};
  float instance_grid_m{0.8F};
  float range_of_interest_m{60.0F};
  float grid_resolution_m{2.0F};
  float sensor_height_m{0.55F};
  bool overwrite{false};
};

struct Erasor2StageFrame {
  std::string source_patch;
  std::filesystem::path scan_bin;
  std::filesystem::path ground_label;
  std::filesystem::path instance_label;
  std::uint64_t point_count{0};
  std::uint64_t ground_points{0};
  std::uint64_t instance_count{0};
};

struct Erasor2StageResult {
  bool success{false};
  std::string reason_code;
  std::string message;
  std::filesystem::path dataset_dir;
  std::filesystem::path config_path;
  std::filesystem::path output_dir;
  std::vector<Erasor2StageFrame> frames;
  std::uint64_t total_points{0};
  std::uint64_t total_ground_points{0};
};

Erasor2StageResult stageErasor2Dataset(const Erasor2StageOptions &options);

std::string toJson(const Erasor2StageResult &result);

}  // namespace lingtu::map_cleaning
