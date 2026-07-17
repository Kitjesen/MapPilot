#include "lingtu/maps/c_api/semantic_occupancy.h"

#include "lingtu/maps/layers/semantic_occupancy.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <cstring>
#include <limits>
#include <string>

using lingtu::maps::CloudLayout;
using lingtu::maps::layers::SemanticGenerationMismatch;
using lingtu::maps::layers::SemanticMapChunk;
using lingtu::maps::layers::SemanticObservationFrame;
using lingtu::maps::layers::SemanticOccupancyConfig;
using lingtu::maps::layers::SemanticOccupancyLayerCore;
using lingtu::maps::layers::SemanticOccupancyUpdateStats;

struct LingtuMapsSemanticHandle {
  explicit LingtuMapsSemanticHandle(SemanticOccupancyConfig config) : core(config) {}
  explicit LingtuMapsSemanticHandle(const SemanticMapChunk& chunk)
      : core([&chunk]() {
          SemanticOccupancyConfig config;
          config.voxel_size_m = chunk.voxel_size_m;
          config.max_voxels = std::max<std::size_t>(config.max_voxels, chunk.Size());
          config.max_query_results =
              std::max<std::size_t>(config.max_query_results, chunk.Size());
          return config;
        }()) {
    core.Replace(chunk, 0U);
  }
  SemanticOccupancyLayerCore core;
};

namespace {

constexpr std::uint32_t kSemanticAbiVersion = 1U;

SemanticOccupancyConfig ToConfig(const LingtuMapsSemanticConfig* config) {
  if (config == nullptr) {
    return {};
  }
  SemanticOccupancyConfig out;
  out.voxel_size_m = config->voxel_size_m;
  out.max_range_m = config->max_range_m;
  out.min_z_m = config->min_z_m;
  out.max_z_m = config->max_z_m;
  out.hit_log_odds = config->hit_log_odds;
  out.miss_log_odds = config->miss_log_odds;
  out.min_log_odds = config->min_log_odds;
  out.max_log_odds = config->max_log_odds;
  out.occupied_probability = config->occupied_probability;
  out.raycast_free_space = config->raycast_free_space != 0U;
  out.max_rays_per_update = static_cast<std::size_t>(config->max_rays_per_update);
  out.max_ray_voxels_per_ray = static_cast<std::size_t>(config->max_ray_voxels_per_ray);
  out.max_voxels = static_cast<std::size_t>(config->max_voxels);
  out.max_query_voxel_checks = static_cast<std::size_t>(config->max_query_voxel_checks);
  out.max_query_results = static_cast<std::size_t>(config->max_query_results);
  out.unknown_label = config->unknown_label;
  return out;
}

void CopyStats(const SemanticOccupancyUpdateStats& stats,
               LingtuMapsSemanticUpdateStats* out) {
  if (out == nullptr) {
    return;
  }
  out->input_points = static_cast<std::uint64_t>(stats.input_points);
  out->accepted_points = static_cast<std::uint64_t>(stats.accepted_points);
  out->hit_voxels = static_cast<std::uint64_t>(stats.hit_voxels);
  out->rays_traced = static_cast<std::uint64_t>(stats.rays_traced);
  out->truncated_rays = static_cast<std::uint64_t>(stats.truncated_rays);
  out->free_voxel_updates = static_cast<std::uint64_t>(stats.free_voxel_updates);
  out->pruned_voxels = static_cast<std::uint64_t>(stats.pruned_voxels);
  out->total_voxels = static_cast<std::uint64_t>(stats.total_voxels);
  out->generation_before = stats.generation_before;
  out->generation_after = stats.generation_after;
  out->replaced_full_map = stats.replaced_full_map ? 1U : 0U;
  out->applied = stats.applied ? 1U : 0U;
  out->duplicate_sequence = stats.duplicate_sequence ? 1U : 0U;
  out->stale_sequence = stats.stale_sequence ? 1U : 0U;
}

template <typename T>
void CopyOptional(const std::vector<T>& source, T* target) {
  if (target == nullptr) {
    return;
  }
  for (std::size_t i = 0U; i < source.size(); ++i) {
    target[i] = source[i];
  }
}

void CopyChunk(const SemanticMapChunk& chunk, const LingtuMapsSemanticChunkBuffers& buffers) {
  if (chunk.data == nullptr) {
    return;
  }
  const auto& data = *chunk.data;
  CopyOptional(data.index_x, buffers.index_x);
  CopyOptional(data.index_y, buffers.index_y);
  CopyOptional(data.index_z, buffers.index_z);
  CopyOptional(data.center_x_m, buffers.center_x_m);
  CopyOptional(data.center_y_m, buffers.center_y_m);
  CopyOptional(data.center_z_m, buffers.center_z_m);
  CopyOptional(data.occupancy_probability, buffers.occupancy_probability);
  CopyOptional(data.hit_count, buffers.hit_count);
  CopyOptional(data.miss_count, buffers.miss_count);
  CopyOptional(data.point_count, buffers.point_count);
  CopyOptional(data.mean_x_m, buffers.mean_x_m);
  CopyOptional(data.mean_y_m, buffers.mean_y_m);
  CopyOptional(data.mean_z_m, buffers.mean_z_m);
  CopyOptional(data.covariance_xx, buffers.covariance_xx);
  CopyOptional(data.covariance_xy, buffers.covariance_xy);
  CopyOptional(data.covariance_xz, buffers.covariance_xz);
  CopyOptional(data.covariance_yy, buffers.covariance_yy);
  CopyOptional(data.covariance_yz, buffers.covariance_yz);
  CopyOptional(data.covariance_zz, buffers.covariance_zz);
  CopyOptional(data.dominant_label, buffers.dominant_label);
  CopyOptional(data.semantic_confidence, buffers.semantic_confidence);
}

LingtuMapsSemanticStatus StatusForException() {
  try {
    throw;
  } catch (const SemanticGenerationMismatch&) {
    return LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED;
  } catch (const std::invalid_argument&) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  } catch (const std::exception&) {
    return LINGTU_MAPS_SEMANTIC_ERROR;
  } catch (...) {
    return LINGTU_MAPS_SEMANTIC_UNKNOWN_ERROR;
  }
}

bool PointCountFits(std::uint64_t count) {
  return count <= static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max());
}

}  // namespace

extern "C" {

std::uint32_t lingtu_maps_semantic_abi_version(void) {
  return kSemanticAbiVersion;
}

LingtuMapsSemanticHandle* lingtu_maps_semantic_create(
    const LingtuMapsSemanticConfig* config) {
  try {
    return new LingtuMapsSemanticHandle(ToConfig(config));
  } catch (...) {
    return nullptr;
  }
}

LingtuMapsSemanticHandle* lingtu_maps_semantic_open_file(const char* path) {
  if (path == nullptr || path[0] == '\0') {
    return nullptr;
  }
  try {
    return new LingtuMapsSemanticHandle(lingtu::maps::ReadSemanticMapBinary(path));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_maps_semantic_destroy(LingtuMapsSemanticHandle* handle) {
  delete handle;
}

std::int32_t lingtu_maps_semantic_reset(LingtuMapsSemanticHandle* handle) {
  if (handle == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    handle->core.Reset();
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_generation(const LingtuMapsSemanticHandle* handle,
                                             std::uint64_t* out_generation) {
  if (handle == nullptr || out_generation == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    *out_generation = handle->core.Generation();
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_metadata(
    const LingtuMapsSemanticHandle* handle, LingtuMapsSemanticMetadata* out_metadata) {
  if (handle == nullptr || out_metadata == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto metadata = handle->core.Metadata();
    out_metadata->generation = metadata.generation;
    out_metadata->voxel_count = static_cast<std::uint64_t>(metadata.voxel_count);
    out_metadata->voxel_size_m = metadata.voxel_size_m;
    out_metadata->taxonomy_version = metadata.taxonomy_version;
    out_metadata->frame_id_bytes = static_cast<std::uint64_t>(metadata.frame_id.size() + 1U);
    out_metadata->taxonomy_bytes = static_cast<std::uint64_t>(metadata.taxonomy.size() + 1U);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_metadata_strings(
    const LingtuMapsSemanticHandle* handle, std::uint64_t expected_generation,
    char* out_frame_id, std::uint64_t frame_id_capacity, char* out_taxonomy,
    std::uint64_t taxonomy_capacity) {
  if (handle == nullptr || out_frame_id == nullptr || out_taxonomy == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto metadata = handle->core.Metadata();
    if (metadata.generation != expected_generation) {
      return LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED;
    }
    const std::size_t frame_bytes = metadata.frame_id.size() + 1U;
    const std::size_t taxonomy_bytes = metadata.taxonomy.size() + 1U;
    if (frame_id_capacity < frame_bytes || taxonomy_capacity < taxonomy_bytes) {
      return LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL;
    }
    std::copy(metadata.frame_id.c_str(), metadata.frame_id.c_str() + frame_bytes, out_frame_id);
    std::copy(metadata.taxonomy.c_str(), metadata.taxonomy.c_str() + taxonomy_bytes, out_taxonomy);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_load_file(
    LingtuMapsSemanticHandle* handle, const char* path,
    std::uint64_t expected_generation, LingtuMapsSemanticUpdateStats* out_stats) {
  if (handle == nullptr || path == nullptr || path[0] == '\0') {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = lingtu::maps::ReadSemanticMapBinary(path);
    CopyStats(handle->core.Replace(chunk, expected_generation), out_stats);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_save_file(
    const LingtuMapsSemanticHandle* handle, const char* path,
    std::uint64_t expected_generation) {
  if (handle == nullptr || path == nullptr || path[0] == '\0') {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = handle->core.SnapshotChunk(
        0U, std::numeric_limits<std::size_t>::max(), 0.0F);
    if (chunk.generation != expected_generation) {
      return LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED;
    }
    lingtu::maps::WriteSemanticMapBinaryAtomic(path, chunk);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_validate_file(
    const char* path, char* out_error, std::uint64_t error_capacity,
    std::uint64_t* out_error_size) {
  if (path == nullptr || path[0] == '\0' || out_error_size == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  std::string error;
  const bool valid = lingtu::maps::ValidateSemanticMapBinary(path, &error);
  const std::uint64_t needed = static_cast<std::uint64_t>(error.size() + 1U);
  *out_error_size = needed;
  if (out_error == nullptr || error_capacity < needed) {
    return LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL;
  }
  std::memcpy(out_error, error.c_str(), static_cast<std::size_t>(needed));
  return valid ? LINGTU_MAPS_SEMANTIC_OK : LINGTU_MAPS_SEMANTIC_ERROR;
}

std::int32_t lingtu_maps_semantic_update_xyz_soa(
    LingtuMapsSemanticHandle* handle, const float* x, const float* y, const float* z,
    const std::uint16_t* labels, std::uint64_t point_count, const char* frame_id,
    std::int64_t stamp_ns, float origin_x_m, float origin_y_m, float origin_z_m,
    std::uint64_t sequence, std::uint64_t expected_generation, std::uint8_t full_map,
    const char* taxonomy, std::uint32_t taxonomy_version,
    LingtuMapsSemanticUpdateStats* out_stats) {
  if (handle == nullptr || !PointCountFits(point_count) ||
      (point_count > 0U && (x == nullptr || y == nullptr || z == nullptr))) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const std::size_t size = static_cast<std::size_t>(point_count);
    SemanticObservationFrame observation;
    observation.frame.cloud.layout = CloudLayout::kXyzF32SoA;
    observation.frame.cloud.point_count = size;
    observation.frame.cloud.x = {x, size};
    observation.frame.cloud.y = {y, size};
    observation.frame.cloud.z = {z, size};
    observation.frame.cloud.frame_id =
        (frame_id == nullptr || frame_id[0] == '\0') ? "map" : frame_id;
    observation.frame.cloud.stamp_ns = stamp_ns;
    observation.frame.sensor_origin_x_m = origin_x_m;
    observation.frame.sensor_origin_y_m = origin_y_m;
    observation.frame.sensor_origin_z_m = origin_z_m;
    observation.frame.full_map = full_map != 0U;
    observation.frame.incremental = full_map == 0U;
    observation.sequence = sequence;
    observation.expected_generation = expected_generation;
    if (labels != nullptr) {
      observation.labels.data = labels;
      observation.labels.size = size;
      observation.labels.stamp_ns = stamp_ns;
      observation.labels.frame_id = observation.frame.cloud.frame_id;
      observation.labels.taxonomy = taxonomy == nullptr ? "" : taxonomy;
      observation.labels.taxonomy_version = taxonomy_version;
    }
    CopyStats(handle->core.Update(observation), out_stats);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_query_radius_count(
    const LingtuMapsSemanticHandle* handle, const LingtuMapsSemanticQuery* query,
    std::uint64_t* out_generation, std::uint64_t* out_count) {
  if (handle == nullptr || query == nullptr || out_generation == nullptr || out_count == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = handle->core.QueryRadiusChunk(
        query->center_x_m, query->center_y_m, query->center_z_m, query->radius_m,
        query->min_occupancy_probability);
    *out_generation = chunk.generation;
    *out_count = static_cast<std::uint64_t>(chunk.Size());
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_query_radius_fill(
    const LingtuMapsSemanticHandle* handle, const LingtuMapsSemanticQuery* query,
    std::uint64_t expected_generation, const LingtuMapsSemanticChunkBuffers* buffers,
    std::uint64_t capacity, std::uint64_t* out_count) {
  if (handle == nullptr || query == nullptr || buffers == nullptr || out_count == nullptr ||
      !PointCountFits(capacity)) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = handle->core.QueryRadiusChunk(
        query->center_x_m, query->center_y_m, query->center_z_m, query->radius_m,
        query->min_occupancy_probability);
    if (chunk.generation != expected_generation) {
      return LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED;
    }
    *out_count = static_cast<std::uint64_t>(chunk.Size());
    if (capacity < chunk.Size()) {
      return LINGTU_MAPS_SEMANTIC_CAPACITY_TOO_SMALL;
    }
    CopyChunk(chunk, *buffers);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_snapshot_count(
    const LingtuMapsSemanticHandle* handle, float min_occupancy_probability,
    std::uint64_t* out_generation, std::uint64_t* out_count) {
  if (handle == nullptr || out_generation == nullptr || out_count == nullptr) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = handle->core.SnapshotChunk(
        0U, std::numeric_limits<std::size_t>::max(), min_occupancy_probability);
    *out_generation = chunk.generation;
    *out_count = static_cast<std::uint64_t>(chunk.total_voxels);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

std::int32_t lingtu_maps_semantic_snapshot_fill(
    const LingtuMapsSemanticHandle* handle, float min_occupancy_probability,
    std::uint64_t expected_generation, std::uint64_t offset,
    const LingtuMapsSemanticChunkBuffers* buffers, std::uint64_t capacity,
    std::uint64_t* out_count, std::uint8_t* out_complete) {
  if (handle == nullptr || buffers == nullptr || out_count == nullptr || out_complete == nullptr ||
      !PointCountFits(offset) || !PointCountFits(capacity) || capacity == 0U) {
    return LINGTU_MAPS_SEMANTIC_INVALID_ARGUMENT;
  }
  try {
    const auto chunk = handle->core.SnapshotChunk(static_cast<std::size_t>(offset),
                                                   static_cast<std::size_t>(capacity),
                                                   min_occupancy_probability);
    if (chunk.generation != expected_generation) {
      return LINGTU_MAPS_SEMANTIC_GENERATION_CHANGED;
    }
    *out_count = static_cast<std::uint64_t>(chunk.Size());
    *out_complete = chunk.complete ? 1U : 0U;
    CopyChunk(chunk, *buffers);
    return LINGTU_MAPS_SEMANTIC_OK;
  } catch (...) {
    return StatusForException();
  }
}

}  // extern "C"
