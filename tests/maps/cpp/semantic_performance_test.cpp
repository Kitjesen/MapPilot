#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "lingtu/maps/layers/semantic_occupancy.hpp"

namespace {

double ThresholdMs() {
  if (const char *configured = std::getenv("LINGTU_MAPS_SEMANTIC_MAX_MS")) {
    const double value = std::strtod(configured, nullptr);
    if (std::isfinite(value) && value > 0.0) {
      return value;
    }
  }
#if defined(__aarch64__) || defined(_M_ARM64)
  return 750.0;
#else
  return 3000.0;
#endif
}

}  // namespace

int main() {
  using lingtu::maps::CloudLayout;
  using lingtu::maps::OwnedPointCloud;
  using lingtu::maps::layers::SemanticObservationFrame;
  using lingtu::maps::layers::SemanticOccupancyConfig;
  using lingtu::maps::layers::SemanticOccupancyLayerCore;

  constexpr std::size_t kPointsPerFrame = 8000U;
  constexpr std::uint64_t kFrames = 20U;
  OwnedPointCloud cloud;
  cloud.frame_id = "map";
  cloud.layout = CloudLayout::kXyzF32Interleaved;
  cloud.point_count = kPointsPerFrame;
  cloud.interleaved.reserve(kPointsPerFrame * 3U);
  std::vector<std::uint16_t> labels;
  labels.reserve(kPointsPerFrame);
  for (std::size_t i = 0U; i < kPointsPerFrame; ++i) {
    const float x = static_cast<float>(i % 200U) * 0.10F;
    const float y = static_cast<float>((i / 200U) % 40U) * 0.10F;
    const float z = static_cast<float>(i % 12U) * 0.05F;
    cloud.interleaved.insert(cloud.interleaved.end(), {x, y, z});
    labels.push_back(static_cast<std::uint16_t>((i % 16U) + 1U));
  }

  SemanticOccupancyConfig config;
  config.voxel_size_m = 0.10F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;
  config.max_voxels = 250000U;
  SemanticOccupancyLayerCore layer(config);

  const auto started = std::chrono::steady_clock::now();
  for (std::uint64_t sequence = 1U; sequence <= kFrames; ++sequence) {
    cloud.stamp_ns = static_cast<std::int64_t>(sequence * 100000000U);
    SemanticObservationFrame observation;
    observation.frame.cloud = cloud.View();
    observation.sequence = sequence;
    observation.expected_generation = layer.Generation();
    observation.labels.data = labels.data();
    observation.labels.size = labels.size();
    observation.labels.stamp_ns = cloud.stamp_ns;
    observation.labels.frame_id = cloud.frame_id;
    observation.labels.taxonomy = "lingtu.semantic";
    observation.labels.taxonomy_version = 1U;
    const auto stats = layer.Update(observation);
    if (!stats.applied) {
      throw std::runtime_error("semantic performance replay update was rejected");
    }
  }
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  const double threshold_ms = ThresholdMs();
  std::cout << "semantic_update points=" << (kPointsPerFrame * kFrames)
            << " elapsed_ms=" << elapsed_ms << " threshold_ms=" << threshold_ms
            << " voxels=" << layer.VoxelCount() << std::endl;
  if (layer.Generation() != kFrames || layer.VoxelCount() == 0U) {
    throw std::runtime_error("semantic performance replay produced invalid state");
  }
  if (elapsed_ms > threshold_ms) {
    throw std::runtime_error("semantic update performance gate exceeded");
  }
  return 0;
}
