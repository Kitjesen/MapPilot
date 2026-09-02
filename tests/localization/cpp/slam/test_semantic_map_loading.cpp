#include "native_relocalizer.hpp"
#include "semantic_map_client.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "lingtu/maps/semantic_map_persistence.hpp"

namespace {

using lingtu::maps::layers::SemanticMapChunk;
using lingtu::maps::layers::SemanticMapChunkSoA;

void Check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class TempDirectory final {
 public:
  TempDirectory() {
    path_ = std::filesystem::temp_directory_path() /
        ("lingtu-semantic-map-loading-" +
         std::to_string(std::filesystem::file_time_type::clock::now()
                            .time_since_epoch()
                            .count()));
    std::filesystem::create_directories(path_);
  }

  ~TempDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void AppendVoxel(
    SemanticMapChunkSoA* data,
    int index,
    float occupancy_probability,
    float x,
    float y,
    float z,
    std::uint32_t point_count = 1U) {
  data->index_x.push_back(index);
  data->index_y.push_back(0);
  data->index_z.push_back(0);
  data->center_x_m.push_back(x + 0.01F);
  data->center_y_m.push_back(y + 0.01F);
  data->center_z_m.push_back(z + 0.01F);
  data->occupancy_probability.push_back(occupancy_probability);
  data->hit_count.push_back(1U);
  data->miss_count.push_back(0U);
  data->point_count.push_back(point_count);
  data->mean_x_m.push_back(x);
  data->mean_y_m.push_back(y);
  data->mean_z_m.push_back(z);
  data->covariance_xx.push_back(0.0F);
  data->covariance_xy.push_back(0.0F);
  data->covariance_xz.push_back(0.0F);
  data->covariance_yy.push_back(0.0F);
  data->covariance_yz.push_back(0.0F);
  data->covariance_zz.push_back(0.0F);
  data->dominant_label.push_back(static_cast<std::uint16_t>(100 + index));
  data->semantic_confidence.push_back(0.75F);
}

SemanticMapChunk MakeSemanticMap() {
  auto data = std::make_shared<SemanticMapChunkSoA>();
  for (int index = 0; index < 32; ++index) {
    const float x = static_cast<float>(index % 8) * 0.5F;
    const float y = static_cast<float>(index / 8) * 0.5F;
    AppendVoxel(data.get(), index, 0.8F, x, y, 0.2F, index == 31 ? 0U : 1U);
  }
  AppendVoxel(data.get(), 32, 0.49F, 10.0F, 0.0F, 0.0F);
  AppendVoxel(
      data.get(),
      33,
      0.8F,
      std::numeric_limits<float>::quiet_NaN(),
      0.0F,
      0.0F);

  SemanticMapChunk chunk;
  chunk.generation = 17U;
  chunk.offset = 0U;
  chunk.total_voxels = data->center_x_m.size();
  chunk.complete = true;
  chunk.voxel_size_m = 0.2F;
  chunk.frame_id = "map";
  chunk.taxonomy = "lingtu-test";
  chunk.taxonomy_version = 3U;
  chunk.data = std::move(data);
  return chunk;
}

void WritePcd(const std::filesystem::path& path) {
  std::ofstream file(path);
  Check(static_cast<bool>(file), "pcd_create_failed");
  file << "# .PCD v0.7\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z intensity\n"
       << "SIZE 4 4 4 4\n"
       << "TYPE F F F F\n"
       << "COUNT 1 1 1 1\n"
       << "WIDTH 32\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS 32\n"
       << "DATA ascii\n";
  for (int index = 0; index < 32; ++index) {
    file << static_cast<float>(index % 8) * 0.5F << ' '
         << static_cast<float>(index / 8) * 0.5F << " 0.2 1\n";
  }
}

void TestSemanticClientOccupancyAndFiniteFiltering(
    const std::filesystem::path& semantic_path) {
  lingtu::slam::SemanticMapClient client;
  lingtu::slam::SemanticMapSnapshot snapshot;
  std::string error;
  Check(client.load(semantic_path.string(), &snapshot, &error), error.c_str());
  Check(snapshot.generation == 17U, "semantic_generation_mismatch");
  Check(snapshot.frame_id == "map", "semantic_frame_mismatch");
  Check(snapshot.taxonomy == "lingtu-test", "semantic_taxonomy_mismatch");
  Check(snapshot.taxonomy_version == 3U, "semantic_taxonomy_version_mismatch");
  Check(snapshot.points.size() == 32U, "semantic_occupancy_or_finite_filter_failed");
  Check(snapshot.points.front().label == 100U, "semantic_label_conversion_failed");
  Check(std::abs(snapshot.points.front().confidence - 0.75F) < 1e-6F,
        "semantic_confidence_conversion_failed");
  Check(std::abs(snapshot.points.back().x - 3.51F) < 1e-6F &&
            std::abs(snapshot.points.back().y - 1.51F) < 1e-6F &&
            std::abs(snapshot.points.back().z - 0.21F) < 1e-6F,
        "semantic_empty_voxel_center_conversion_failed");
}

void TestNativeRelocalizerArtifactSelection(
    const std::filesystem::path& semantic_path,
    const std::filesystem::path& pcd_path) {
  std::string message;
  lingtu::slam::NativeRelocalizer semantic_relocalizer;
  Check(semantic_relocalizer.loadMap(pcd_path.string(), &message), message.c_str());
  Check(message == "native_relocalizer_semantic_map_loaded",
        "valid_semantic_artifact_not_selected");
  Check(semantic_relocalizer.hasMap(), "semantic_relocalizer_map_missing");

  {
    std::ofstream corrupt(semantic_path, std::ios::binary | std::ios::trunc);
    corrupt << "broken";
  }
  lingtu::slam::NativeRelocalizer corrupt_relocalizer;
  Check(!corrupt_relocalizer.loadMap(pcd_path.string(), &message),
        "corrupt_semantic_artifact_fell_back_to_pcd");
  Check(message.find("native_relocalizer_semantic_map_load_failed:") == 0U,
        "corrupt_semantic_failure_not_reported");
  Check(!corrupt_relocalizer.hasMap(), "corrupt_semantic_artifact_loaded_map");

  std::filesystem::remove(semantic_path);
  lingtu::slam::NativeRelocalizer pcd_relocalizer;
  Check(pcd_relocalizer.loadMap(pcd_path.string(), &message), message.c_str());
  Check(message == "native_relocalizer_pcd_map_loaded",
        "missing_semantic_artifact_did_not_use_pcd");
  Check(pcd_relocalizer.hasMap(), "pcd_fallback_map_missing");
}

}  // namespace

int main() {
  TempDirectory temporary;
  const auto semantic_path = temporary.path() / "semantic_map.bin";
  const auto pcd_path = temporary.path() / "map.pcd";
  lingtu::maps::WriteSemanticMapBinaryAtomic(semantic_path, MakeSemanticMap());
  WritePcd(pcd_path);

  TestSemanticClientOccupancyAndFiniteFiltering(semantic_path);
  TestNativeRelocalizerArtifactSelection(semantic_path, pcd_path);
  return 0;
}
