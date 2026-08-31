#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "lingtu/maps/layers/semantic_occupancy.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"
#include "semantic_map_client.hpp"

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::maps::layers::SemanticMapChunk MakeArtifact() {
  using lingtu::maps::layers::SemanticMapChunk;
  using lingtu::maps::layers::SemanticMapChunkSoA;

  auto data = std::make_shared<SemanticMapChunkSoA>();
  data->index_x = {0, 1, 2};
  data->index_y = {0, 0, 0};
  data->index_z = {0, 0, 0};
  data->center_x_m = {0.1F, 1.1F, 2.1F};
  data->center_y_m = {0.1F, 0.1F, 0.1F};
  data->center_z_m = {0.1F, 0.1F, 0.1F};
  data->occupancy_probability = {0.8F, 0.8F, 0.8F};
  data->hit_count = {1U, 1U, 1U};
  data->miss_count = {0U, 0U, 0U};
  data->point_count = {1U, 1U, 1U};
  data->mean_x_m = data->center_x_m;
  data->mean_y_m = data->center_y_m;
  data->mean_z_m = data->center_z_m;
  data->covariance_xx = {0.0F, 0.0F, 0.0F};
  data->covariance_xy = data->covariance_xx;
  data->covariance_xz = data->covariance_xx;
  data->covariance_yy = data->covariance_xx;
  data->covariance_yz = data->covariance_xx;
  data->covariance_zz = data->covariance_xx;
  data->dominant_label = {0U, 7U, 9U};
  data->semantic_confidence = {0.0F, 1.0F, 1.0F};

  SemanticMapChunk chunk;
  chunk.generation = 1U;
  chunk.total_voxels = data->center_x_m.size();
  chunk.voxel_size_m = 0.5F;
  chunk.frame_id = "map";
  chunk.taxonomy = "lingtu.semantic";
  chunk.taxonomy_version = 1U;
  chunk.data = std::move(data);
  return chunk;
}

}  // namespace

int main() {
  const auto artifact =
      std::filesystem::temp_directory_path() / "lingtu_semantic_localization_client_test.bin";
  std::filesystem::remove(artifact);
  lingtu::maps::WriteSemanticMapBinaryAtomic(artifact, MakeArtifact());

  lingtu::slam::SemanticMapClient client;
  Require(client.available(), "semantic localization client unavailable");
  lingtu::slam::SemanticMapSnapshot snapshot;
  std::string error;
  Require(client.load(artifact.string(), &snapshot, &error),
          "semantic localization artifact load failed");
  Require(error.empty(), "semantic localization client returned an error");
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
