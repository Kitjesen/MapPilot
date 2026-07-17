#include "lingtu/maps/c_api/semantic_occupancy.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <string>

namespace {

LingtuMapsSemanticConfig Config() {
  LingtuMapsSemanticConfig config{};
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.min_z_m = -3.0F;
  config.max_z_m = 5.0F;
  config.hit_log_odds = 0.85F;
  config.miss_log_odds = -0.40F;
  config.min_log_odds = -2.0F;
  config.max_log_odds = 3.5F;
  config.occupied_probability = 0.50F;
  config.raycast_free_space = 0U;
  config.max_rays_per_update = 4000U;
  config.max_ray_voxels_per_ray = 1024U;
  config.max_voxels = 1000U;
  config.max_query_voxel_checks = 1000U;
  config.max_query_results = 1000U;
  return config;
}

LingtuMapsSemanticChunkBuffers Buffers(
    std::array<std::int32_t, 2U>* index_x,
    std::array<float, 2U>* center_x,
    std::array<float, 2U>* mean_x,
    std::array<float, 2U>* covariance_xx,
    std::array<float, 2U>* occupancy,
    std::array<std::uint16_t, 2U>* labels,
    std::array<float, 2U>* confidence) {
  LingtuMapsSemanticChunkBuffers buffers{};
  buffers.index_x = index_x->data();
  buffers.center_x_m = center_x->data();
  buffers.mean_x_m = mean_x->data();
  buffers.covariance_xx = covariance_xx->data();
  buffers.occupancy_probability = occupancy->data();
  buffers.dominant_label = labels->data();
  buffers.semantic_confidence = confidence->data();
  return buffers;
}

}  // namespace

int main() {
  assert(lingtu_maps_semantic_abi_version() == 1U);
  const auto config = Config();
  LingtuMapsSemanticHandle* handle = lingtu_maps_semantic_create(&config);
  assert(handle != nullptr);

  const std::array<float, 2U> x{0.1F, 1.1F};
  const std::array<float, 2U> y{0.1F, 0.1F};
  const std::array<float, 2U> z{0.1F, 0.1F};
  const std::array<std::uint16_t, 2U> semantic_labels{3U, 4U};
  LingtuMapsSemanticUpdateStats update{};
  assert(lingtu_maps_semantic_update_xyz_soa(
             handle, x.data(), y.data(), z.data(), semantic_labels.data(), x.size(), "map", 10,
             0.0F, 0.0F, 0.0F, 1U, 0U, 0U, "lingtu.semantic", 1U, &update) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(update.applied == 1U);
  assert(update.generation_before == 0U);
  assert(update.generation_after == 1U);

  LingtuMapsSemanticMetadata metadata{};
  assert(lingtu_maps_semantic_metadata(handle, &metadata) == LINGTU_MAPS_SEMANTIC_OK);
  assert(metadata.generation == 1U);
  assert(metadata.voxel_count == 2U);
  assert(metadata.taxonomy_version == 1U);
  std::array<char, 16U> frame_id{};
  std::array<char, 32U> taxonomy{};
  assert(lingtu_maps_semantic_metadata_strings(
             handle, metadata.generation, frame_id.data(), frame_id.size(), taxonomy.data(),
             taxonomy.size()) == LINGTU_MAPS_SEMANTIC_OK);
  assert(std::string(frame_id.data()) == "map");
  assert(std::string(taxonomy.data()) == "lingtu.semantic");

  const auto artifact = std::filesystem::temp_directory_path() /
      "lingtu_semantic_c_api_test.bin";
  std::filesystem::remove(artifact);
  assert(lingtu_maps_semantic_save_file(handle, artifact.string().c_str(), 1U) ==
         LINGTU_MAPS_SEMANTIC_OK);
  std::uint64_t error_size = 0U;
  assert(lingtu_maps_semantic_validate_file(
             artifact.string().c_str(), nullptr, 0U, &error_size) ==
         LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL);
  std::array<char, 64U> validation_error{};
  assert(lingtu_maps_semantic_validate_file(
             artifact.string().c_str(), validation_error.data(), validation_error.size(),
             &error_size) == LINGTU_MAPS_SEMANTIC_OK);
  LingtuMapsSemanticHandle* loaded = lingtu_maps_semantic_create(&config);
  assert(loaded != nullptr);
  LingtuMapsSemanticUpdateStats load_stats{};
  assert(lingtu_maps_semantic_load_file(
             loaded, artifact.string().c_str(), 0U, &load_stats) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(load_stats.replaced_full_map == 1U);
  assert(load_stats.generation_after == 1U);
  lingtu_maps_semantic_destroy(loaded);
  LingtuMapsSemanticHandle* opened =
      lingtu_maps_semantic_open_file(artifact.string().c_str());
  assert(opened != nullptr);
  assert(lingtu_maps_semantic_metadata(opened, &metadata) == LINGTU_MAPS_SEMANTIC_OK);
  assert(metadata.generation == 1U);
  assert(metadata.voxel_count == 2U);
  lingtu_maps_semantic_destroy(opened);
  std::filesystem::remove(artifact);

  assert(lingtu_maps_semantic_update_xyz_soa(
             handle, x.data(), y.data(), z.data(), nullptr, x.size(), "map", 10, 0.0F, 0.0F,
             0.0F, 1U, 1U, 0U, nullptr, 0U, &update) == LINGTU_MAPS_SEMANTIC_OK);
  assert(update.applied == 0U);
  assert(update.duplicate_sequence == 1U);
  assert(update.generation_after == 1U);

  LingtuMapsSemanticQuery query{0.5F, 0.5F, 0.5F, 2.0F, 0.5F};
  std::uint64_t generation = 0U;
  std::uint64_t count = 0U;
  assert(lingtu_maps_semantic_query_radius_count(handle, &query, &generation, &count) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(generation == 1U);
  assert(count == 2U);

  const std::array<float, 1U> next_x{2.1F};
  const std::array<float, 1U> next_y{0.1F};
  const std::array<float, 1U> next_z{0.1F};
  assert(lingtu_maps_semantic_update_xyz_soa(
             handle, next_x.data(), next_y.data(), next_z.data(), nullptr, next_x.size(), "map",
             11, 0.0F, 0.0F, 0.0F, 2U, 1U, 0U, nullptr, 0U, &update) ==
         LINGTU_MAPS_SEMANTIC_OK);

  std::array<std::int32_t, 2U> index_x{};
  std::array<float, 2U> center_x{};
  std::array<float, 2U> mean_x{};
  std::array<float, 2U> covariance_xx{};
  std::array<float, 2U> occupancy{};
  std::array<std::uint16_t, 2U> labels{};
  std::array<float, 2U> confidence{};
  const auto buffers =
      Buffers(&index_x, &center_x, &mean_x, &covariance_xx, &occupancy, &labels, &confidence);
  assert(lingtu_maps_semantic_query_radius_fill(handle, &query, generation, &buffers,
                                                index_x.size(), &count) ==
         LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED);

  assert(lingtu_maps_semantic_query_radius_count(handle, &query, &generation, &count) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(generation == 2U);
  assert(count == 3U);
  assert(lingtu_maps_semantic_query_radius_fill(handle, &query, generation, &buffers,
                                                index_x.size(), &count) ==
         LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL);

  query.radius_m = 1.5F;
  assert(lingtu_maps_semantic_query_radius_count(handle, &query, &generation, &count) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(count == 2U);
  assert(lingtu_maps_semantic_query_radius_fill(handle, &query, generation, &buffers,
                                                index_x.size(), &count) ==
         LINGTU_MAPS_SEMANTIC_OK);
  assert(count == 2U);
  assert(index_x[0] == 0);
  assert(index_x[1] == 1);
  assert(labels[0] == 3U);
  assert(labels[1] == 4U);

  lingtu_maps_semantic_destroy(handle);
  return 0;
}
