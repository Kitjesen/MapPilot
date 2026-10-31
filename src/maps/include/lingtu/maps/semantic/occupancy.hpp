#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "lingtu/maps/cloud.hpp"

namespace lingtu::maps::layers {

inline constexpr std::uint64_t kAnySemanticMapGeneration =
    std::numeric_limits<std::uint64_t>::max();

class SemanticGenerationMismatch final : public std::runtime_error {
 public:
  SemanticGenerationMismatch(std::uint64_t expected_generation,
                             std::uint64_t actual_generation);

  std::uint64_t expected_generation() const noexcept { return expected_generation_; }
  std::uint64_t actual_generation() const noexcept { return actual_generation_; }

 private:
  std::uint64_t expected_generation_{0U};
  std::uint64_t actual_generation_{0U};
};

struct SemanticLabelView {
  const std::uint16_t *data{nullptr};
  std::size_t size{0};
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::string taxonomy;
  std::uint32_t taxonomy_version{0U};
};

struct SemanticObservationFrame {
  MapCloudFrame frame;
  SemanticLabelView labels;
  std::uint64_t sequence{0U};
  std::uint64_t expected_generation{kAnySemanticMapGeneration};
};

struct SemanticOccupancyConfig {
  float voxel_size_m{0.20F};
  float max_range_m{50.0F};
  float min_z_m{-3.0F};
  float max_z_m{5.0F};
  float hit_log_odds{0.85F};
  float miss_log_odds{-0.40F};
  float min_log_odds{-2.0F};
  float max_log_odds{3.5F};
  float occupied_probability{0.50F};
  bool raycast_free_space{true};
  std::size_t max_rays_per_update{4000U};
  std::size_t max_ray_voxels_per_ray{1024U};
  std::size_t max_voxels{2'000'000U};
  std::size_t max_query_voxel_checks{1'500'000U};
  std::size_t max_query_results{200'000U};
  std::uint16_t unknown_label{0U};
};

struct SemanticVoxel {
  std::int32_t index_x{0};
  std::int32_t index_y{0};
  std::int32_t index_z{0};
  float center_x_m{0.0F};
  float center_y_m{0.0F};
  float center_z_m{0.0F};
  float occupancy_probability{0.5F};
  bool occupied{false};
  std::uint32_t hit_count{0U};
  std::uint32_t miss_count{0U};
  std::uint32_t point_count{0U};
  float mean_x_m{0.0F};
  float mean_y_m{0.0F};
  float mean_z_m{0.0F};
  float covariance_xx{0.0F};
  float covariance_xy{0.0F};
  float covariance_xz{0.0F};
  float covariance_yy{0.0F};
  float covariance_yz{0.0F};
  float covariance_zz{0.0F};
  std::uint16_t dominant_label{0U};
  float semantic_confidence{0.0F};
};

struct SemanticOccupancyUpdateStats {
  std::size_t input_points{0U};
  std::size_t accepted_points{0U};
  std::size_t hit_voxels{0U};
  std::size_t rays_traced{0U};
  std::size_t truncated_rays{0U};
  std::size_t free_voxel_updates{0U};
  std::size_t pruned_voxels{0U};
  std::size_t total_voxels{0U};
  bool replaced_full_map{false};
  bool applied{false};
  bool duplicate_sequence{false};
  bool stale_sequence{false};
  std::uint64_t generation_before{0U};
  std::uint64_t generation_after{0U};
};

struct SemanticMapChunkSoA {
  std::vector<std::int32_t> index_x;
  std::vector<std::int32_t> index_y;
  std::vector<std::int32_t> index_z;
  std::vector<float> center_x_m;
  std::vector<float> center_y_m;
  std::vector<float> center_z_m;
  std::vector<float> occupancy_probability;
  std::vector<std::uint32_t> hit_count;
  std::vector<std::uint32_t> miss_count;
  std::vector<std::uint32_t> point_count;
  std::vector<float> mean_x_m;
  std::vector<float> mean_y_m;
  std::vector<float> mean_z_m;
  std::vector<float> covariance_xx;
  std::vector<float> covariance_xy;
  std::vector<float> covariance_xz;
  std::vector<float> covariance_yy;
  std::vector<float> covariance_yz;
  std::vector<float> covariance_zz;
  std::vector<std::uint16_t> dominant_label;
  std::vector<float> semantic_confidence;
};

struct SemanticMapChunk {
  std::uint64_t generation{0U};
  std::size_t offset{0U};
  std::size_t total_voxels{0U};
  bool complete{true};
  float voxel_size_m{0.0F};
  std::string frame_id;
  std::string taxonomy;
  std::uint32_t taxonomy_version{0U};
  std::shared_ptr<const SemanticMapChunkSoA> data;

  std::size_t Size() const noexcept {
    return data == nullptr ? 0U : data->center_x_m.size();
  }
};

struct SemanticMapMetadata {
  std::uint64_t generation{0U};
  std::size_t voxel_count{0U};
  float voxel_size_m{0.0F};
  std::string frame_id;
  std::string taxonomy;
  std::uint32_t taxonomy_version{0U};
};

class SemanticOccupancyLayerCore final {
 public:
  explicit SemanticOccupancyLayerCore(SemanticOccupancyConfig config = {});
  ~SemanticOccupancyLayerCore();

  SemanticOccupancyLayerCore(const SemanticOccupancyLayerCore &) = delete;
  SemanticOccupancyLayerCore &operator=(const SemanticOccupancyLayerCore &) = delete;
  SemanticOccupancyLayerCore(SemanticOccupancyLayerCore &&) = delete;
  SemanticOccupancyLayerCore &operator=(SemanticOccupancyLayerCore &&) = delete;

  void Reset();
  SemanticOccupancyUpdateStats Update(const SemanticObservationFrame &observation);
  SemanticOccupancyUpdateStats Replace(
      const SemanticMapChunk &chunk,
      std::uint64_t expected_generation = kAnySemanticMapGeneration);

  std::optional<SemanticVoxel> Lookup(float x_m, float y_m, float z_m) const;
  std::vector<SemanticVoxel> QueryRadius(float center_x_m, float center_y_m, float center_z_m,
                                         float radius_m,
                                         float min_occupancy_probability = 0.50F) const;
  SemanticMapChunk QueryRadiusChunk(float center_x_m, float center_y_m, float center_z_m,
                                    float radius_m,
                                    float min_occupancy_probability = 0.50F) const;
  SemanticMapChunk SnapshotChunk(std::size_t offset, std::size_t limit,
                                 float min_occupancy_probability = 0.0F) const;
  SemanticMapMetadata Metadata() const;
  std::uint64_t Generation() const;
  std::size_t VoxelCount() const;
  SemanticOccupancyUpdateStats LastStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::maps::layers
