#include "semantic_map_client.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <string>
#include <utility>

#include "lingtu/maps/semantic_map_persistence.hpp"

namespace lingtu::slam {
namespace {

constexpr float kMinimumOccupancyProbability = 0.5F;

bool Fail(std::string* error, std::string message) {
  if (error != nullptr) {
    *error = std::move(message);
  }
  return false;
}

}  // namespace

bool SemanticMapClient::available() const {
  return true;
}

bool SemanticMapClient::load(
    const std::string& path,
    SemanticMapSnapshot* snapshot,
    std::string* error) const {
  if (snapshot == nullptr) {
    return Fail(error, "semantic_map_snapshot_output_required");
  }

  try {
    const auto chunk = lingtu::maps::ReadSemanticMapBinary(std::filesystem::path(path));
    if (chunk.data == nullptr) {
      return Fail(error, "semantic_map_has_no_voxel_data");
    }

    SemanticMapSnapshot loaded;
    loaded.generation = chunk.generation;
    loaded.frame_id = chunk.frame_id;
    loaded.taxonomy = chunk.taxonomy;
    loaded.taxonomy_version = chunk.taxonomy_version;
    loaded.points.reserve(chunk.Size());

    bool has_occupied_voxel = false;
    for (std::size_t index = 0U; index < chunk.Size(); ++index) {
      if (chunk.data->occupancy_probability[index] < kMinimumOccupancyProbability) {
        continue;
      }
      has_occupied_voxel = true;
      const bool has_mean = chunk.data->point_count[index] > 0U;
      const float x = has_mean ? chunk.data->mean_x_m[index] : chunk.data->center_x_m[index];
      const float y = has_mean ? chunk.data->mean_y_m[index] : chunk.data->center_y_m[index];
      const float z = has_mean ? chunk.data->mean_z_m[index] : chunk.data->center_z_m[index];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      loaded.points.push_back({
          x,
          y,
          z,
          chunk.data->dominant_label[index],
          chunk.data->semantic_confidence[index],
      });
    }

    if (loaded.points.empty()) {
      return Fail(
          error,
          has_occupied_voxel ? "semantic_map_has_no_finite_points"
                             : "semantic_map_has_no_occupied_voxels");
    }
    *snapshot = std::move(loaded);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const std::exception& exception) {
    return Fail(error, "semantic_map_open_failed: " + std::string(exception.what()));
  }
}

}  // namespace lingtu::slam
