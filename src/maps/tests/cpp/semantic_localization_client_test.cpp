#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "lingtu/maps/c_api/semantic_occupancy.h"
#include "semantic_map_client.hpp"

namespace {

LingtuMapsSemanticConfig Config() {
  LingtuMapsSemanticConfig config{};
  config.voxel_size_m = 0.5F;
  config.min_z_m = -3.0F;
  config.max_z_m = 5.0F;
  config.hit_log_odds = 0.85F;
  config.miss_log_odds = -0.40F;
  config.min_log_odds = -2.0F;
  config.max_log_odds = 3.5F;
  config.occupied_probability = 0.50F;
  config.max_rays_per_update = 4000U;
  config.max_ray_voxels_per_ray = 1024U;
  config.max_voxels = 1000U;
  config.max_query_voxel_checks = 1000U;
  config.max_query_results = 1000U;
  return config;
}

void Require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  const auto artifact =
      std::filesystem::temp_directory_path() / "lingtu_semantic_localization_client_test.bin";
  std::filesystem::remove(artifact);

  const auto config = Config();
  LingtuMapsSemanticHandle *handle = lingtu_maps_semantic_create(&config);
  Require(handle != nullptr, "semantic C ABI create failed");
  const std::array<float, 3U> x{0.1F, 1.1F, 2.1F};
  const std::array<float, 3U> y{0.1F, 0.1F, 0.1F};
  const std::array<float, 3U> z{0.1F, 0.1F, 0.1F};
  const std::array<std::uint16_t, 3U> labels{0U, 7U, 9U};
  LingtuMapsSemanticUpdateStats stats{};
  Require(lingtu_maps_semantic_update_xyz_soa(
              handle, x.data(), y.data(), z.data(), labels.data(), x.size(), "map", 10, 0.0F, 0.0F,
              0.0F, 1U, 0U, 0U, "lingtu.semantic", 1U, &stats) == LINGTU_MAPS_SEMANTIC_OK,
          "semantic C ABI update failed");
  Require(stats.generation_after == 1U, "semantic generation mismatch");
  Require(lingtu_maps_semantic_save_file(handle, artifact.string().c_str(), 1U) ==
              LINGTU_MAPS_SEMANTIC_OK,
          "semantic C ABI save failed");
  lingtu_maps_semantic_destroy(handle);

  lingtu::slam::SemanticMapClient client;
  Require(client.available(), "semantic localization ABI client unavailable");
  lingtu::slam::SemanticMapSnapshot snapshot;
  std::string error;
  Require(client.load(artifact.string(), &snapshot, &error),
          "semantic localization ABI load failed");
  Require(error.empty(), "semantic localization ABI returned an error");
  Require(snapshot.generation == 1U, "semantic artifact generation mismatch");
  Require(snapshot.frame_id == "map", "semantic artifact frame mismatch");
  Require(snapshot.taxonomy == "lingtu.semantic", "semantic taxonomy mismatch");
  Require(snapshot.taxonomy_version == 1U, "semantic taxonomy version mismatch");
  Require(snapshot.points.size() == 3U, "semantic localization point count mismatch");
  Require(snapshot.points[0].label == 0U, "geometry-only label was not preserved");
  Require(snapshot.points[1].label == 7U, "semantic label 7 was not preserved");
  Require(snapshot.points[2].label == 9U, "semantic label 9 was not preserved");

  std::filesystem::remove(artifact);
  return 0;
}
