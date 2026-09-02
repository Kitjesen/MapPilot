#include "lingtu/maps/layers/semantic_occupancy.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lingtu::maps::CloudLayout;
using lingtu::maps::MapCloudFrame;
using lingtu::maps::OwnedPointCloud;
using lingtu::maps::layers::SemanticLabelView;
using lingtu::maps::layers::SemanticObservationFrame;
using lingtu::maps::layers::SemanticOccupancyConfig;
using lingtu::maps::layers::SemanticOccupancyLayerCore;

struct ObservationFixture {
  OwnedPointCloud cloud;
  std::vector<std::uint16_t> labels;
  SemanticObservationFrame observation;
};

ObservationFixture MakeObservation(std::vector<float> xyz, std::vector<std::uint16_t> labels = {},
                                   float origin_x_m = 0.0F, float origin_y_m = 0.0F,
                                   float origin_z_m = 0.0F,
                                   std::uint64_t sequence = 0U) {
  ObservationFixture fixture;
  fixture.cloud.frame_id = "map";
  fixture.cloud.stamp_ns = 123456789;
  fixture.cloud.layout = CloudLayout::kXyzF32Interleaved;
  fixture.cloud.point_count = xyz.size() / 3U;
  fixture.cloud.interleaved = std::move(xyz);
  fixture.labels = std::move(labels);

  fixture.observation.frame.cloud = fixture.cloud.View();
  fixture.observation.frame.sensor_origin_x_m = origin_x_m;
  fixture.observation.frame.sensor_origin_y_m = origin_y_m;
  fixture.observation.frame.sensor_origin_z_m = origin_z_m;
  fixture.observation.sequence = sequence;
  fixture.observation.labels.data = fixture.labels.data();
  fixture.observation.labels.size = fixture.labels.size();
  if (!fixture.labels.empty()) {
    fixture.observation.labels.stamp_ns = fixture.cloud.stamp_ns;
    fixture.observation.labels.frame_id = fixture.cloud.frame_id;
    fixture.observation.labels.taxonomy = "lingtu.semantic";
    fixture.observation.labels.taxonomy_version = 1U;
  }
  return fixture;
}

void TestGenerationAndDuplicateSequenceAreStable() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  assert(layer.Generation() == 0U);

  auto first = MakeObservation({0.10F, 0.10F, 0.10F}, {}, 0.0F, 0.0F, 0.0F, 41U);
  first.observation.expected_generation = layer.Generation();
  const auto applied = layer.Update(first.observation);
  assert(applied.applied);
  assert(applied.generation_before == 0U);
  assert(applied.generation_after == 1U);
  assert(layer.Generation() == 1U);

  auto duplicate = MakeObservation({9.10F, 0.10F, 0.10F}, {}, 0.0F, 0.0F, 0.0F, 41U);
  duplicate.observation.expected_generation = layer.Generation();
  const auto ignored = layer.Update(duplicate.observation);
  assert(!ignored.applied);
  assert(ignored.duplicate_sequence);
  assert(!ignored.stale_sequence);
  assert(ignored.generation_before == 1U);
  assert(ignored.generation_after == 1U);
  assert(layer.Generation() == 1U);
  assert(!layer.Lookup(9.10F, 0.10F, 0.10F).has_value());

  auto stale = MakeObservation({8.10F, 0.10F, 0.10F}, {}, 0.0F, 0.0F, 0.0F, 40U);
  stale.observation.expected_generation = layer.Generation();
  const auto stale_stats = layer.Update(stale.observation);
  assert(!stale_stats.applied);
  assert(!stale_stats.duplicate_sequence);
  assert(stale_stats.stale_sequence);
  assert(layer.Generation() == 1U);
}

void TestGenerationMismatchFailsBeforeMutation() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  auto first = MakeObservation({0.10F, 0.10F, 0.10F}, {}, 0.0F, 0.0F, 0.0F, 1U);
  first.observation.expected_generation = 0U;
  layer.Update(first.observation);

  auto stale_writer = MakeObservation({2.10F, 0.10F, 0.10F}, {}, 0.0F, 0.0F, 0.0F, 2U);
  stale_writer.observation.expected_generation = 0U;
  bool rejected = false;
  try {
    static_cast<void>(layer.Update(stale_writer.observation));
  } catch (const lingtu::maps::layers::SemanticGenerationMismatch &error) {
    rejected = true;
    assert(error.expected_generation() == 0U);
    assert(error.actual_generation() == 1U);
  }
  assert(rejected);
  assert(layer.Generation() == 1U);
  assert(!layer.Lookup(2.10F, 0.10F, 0.10F).has_value());
}

void TestSoAChunkCarriesOneConsistentGeneration() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation(
      {0.10F, 0.10F, 0.10F, 1.10F, 0.10F, 0.10F}, {5U, 7U}, 0.0F, 0.0F, 0.0F,
      1U);
  layer.Update(observation.observation);

  const auto chunk = layer.QueryRadiusChunk(0.5F, 0.5F, 0.5F, 2.0F);
  assert(chunk.generation == layer.Generation());
  assert(chunk.frame_id == "map");
  assert(chunk.taxonomy == "lingtu.semantic");
  assert(chunk.taxonomy_version == 1U);
  assert(chunk.voxel_size_m == 1.0F);
  assert(chunk.Size() == 2U);
  assert(chunk.data != nullptr);
  assert(chunk.data->center_x_m.size() == chunk.Size());
  assert(chunk.data->mean_x_m.size() == chunk.Size());
  assert(chunk.data->covariance_xx.size() == chunk.Size());
  assert(chunk.data->occupancy_probability.size() == chunk.Size());
  assert(chunk.data->dominant_label.size() == chunk.Size());
  assert(chunk.data->semantic_confidence.size() == chunk.Size());

  const auto first_page = layer.SnapshotChunk(0U, 1U);
  assert(first_page.generation == chunk.generation);
  assert(first_page.Size() == 1U);
  assert(!first_page.complete);
  const auto second_page = layer.SnapshotChunk(1U, 1U);
  assert(second_page.generation == chunk.generation);
  assert(second_page.Size() == 1U);
  assert(second_page.complete);
}

void TestCompleteChunkCanReplaceMapWithGenerationGuard() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;
  SemanticOccupancyLayerCore source(config);
  auto observation = MakeObservation({0.10F, 0.10F, 0.10F}, {5U}, 0.0F, 0.0F, 0.0F, 1U);
  source.Update(observation.observation);
  const auto chunk = source.SnapshotChunk(0U, 10U);

  SemanticOccupancyLayerCore target(config);
  const auto replaced = target.Replace(chunk, 0U);

  assert(replaced.applied);
  assert(replaced.replaced_full_map);
  assert(target.Generation() == chunk.generation);
  const auto voxel = target.Lookup(0.10F, 0.10F, 0.10F);
  assert(voxel.has_value());
  assert(voxel->dominant_label == 5U);

  bool mismatch = false;
  try {
    static_cast<void>(target.Replace(chunk, 0U));
  } catch (const lingtu::maps::layers::SemanticGenerationMismatch&) {
    mismatch = true;
  }
  assert(mismatch);
  assert(target.Generation() == chunk.generation);
}

void TestGeometryAndSemanticStatistics() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation(
      {
          0.10F,
          0.10F,
          0.10F,
          0.30F,
          0.10F,
          0.10F,
          0.20F,
          0.30F,
          0.10F,
      },
      {7U, 7U, 3U});
  layer.Update(observation.observation);

  const auto voxel = layer.Lookup(0.2F, 0.2F, 0.1F);
  assert(voxel.has_value());
  assert(voxel->occupied);
  assert(voxel->hit_count == 1U);
  assert(voxel->point_count == 3U);
  assert(std::fabs(voxel->mean_x_m - 0.20F) < 1e-5F);
  assert(std::fabs(voxel->mean_y_m - (0.50F / 3.0F)) < 1e-5F);
  assert(voxel->covariance_xx > 0.0F);
  assert(voxel->dominant_label == 7U);
  assert(voxel->semantic_confidence > 0.60F);
}

void TestEvidenceIsDeduplicatedPerFrame() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = true;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation(
      {
          3.20F,
          0.10F,
          0.10F,
          3.20F,
          0.20F,
          0.10F,
      },
      {2U, 2U}, 0.10F, 0.10F, 0.10F);
  layer.Update(observation.observation);

  const auto free_voxel = layer.Lookup(1.20F, 0.10F, 0.10F);
  const auto hit_voxel = layer.Lookup(3.20F, 0.10F, 0.10F);
  assert(free_voxel.has_value());
  assert(free_voxel->miss_count == 1U);
  assert(hit_voxel.has_value());
  assert(hit_voxel->hit_count == 1U);
  assert(hit_voxel->point_count == 2U);
}

void TestRaycastingClearsStaleObstacle() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = true;
  config.hit_log_odds = 0.85F;
  config.miss_log_odds = -0.50F;

  SemanticOccupancyLayerCore layer(config);
  auto obstacle = MakeObservation({2.20F, 0.10F, 0.10F}, {4U}, 0.10F, 0.10F, 0.10F);
  layer.Update(obstacle.observation);
  const auto occupied = layer.Lookup(2.20F, 0.10F, 0.10F);
  assert(occupied.has_value());
  assert(occupied->occupied);

  for (int i = 0; i < 3; ++i) {
    auto clear_ray = MakeObservation({4.20F, 0.10F, 0.10F}, {1U}, 0.10F, 0.10F, 0.10F);
    layer.Update(clear_ray.observation);
  }

  const auto cleared = layer.Lookup(2.20F, 0.10F, 0.10F);
  assert(cleared.has_value());
  assert(!cleared->occupied);
  assert(cleared->miss_count >= 2U);
  assert(cleared->point_count == 0U);
}

void TestFullMapReplacesPreviousState() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = true;

  SemanticOccupancyLayerCore layer(config);
  auto incremental = MakeObservation({1.10F, 0.10F, 0.10F});
  layer.Update(incremental.observation);
  assert(layer.Lookup(1.10F, 0.10F, 0.10F).has_value());

  auto replacement = MakeObservation({5.10F, 0.10F, 0.10F});
  replacement.observation.frame.full_map = true;
  replacement.observation.frame.incremental = false;
  layer.Update(replacement.observation);

  assert(!layer.Lookup(1.10F, 0.10F, 0.10F).has_value());
  assert(layer.Lookup(5.10F, 0.10F, 0.10F).has_value());
  assert(layer.VoxelCount() == 1U);
  assert(layer.LastStats().rays_traced == 0U);
}

void TestMemoryLimitIsEnforced() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;
  config.max_voxels = 3U;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation({
      0.10F,
      0.10F,
      0.10F,
      1.10F,
      0.10F,
      0.10F,
      2.10F,
      0.10F,
      0.10F,
      3.10F,
      0.10F,
      0.10F,
      4.10F,
      0.10F,
      0.10F,
  });
  layer.Update(observation.observation);

  assert(layer.VoxelCount() == 3U);
  assert(layer.LastStats().pruned_voxels == 2U);
}

void TestQueryIsBounded() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;
  config.max_query_voxel_checks = 200U;
  config.max_query_results = 2U;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation({
      0.10F,
      0.10F,
      0.10F,
      1.10F,
      0.10F,
      0.10F,
  });
  layer.Update(observation.observation);
  assert(layer.QueryRadius(0.5F, 0.5F, 0.5F, 2.0F).size() == 2U);

  bool rejected = false;
  try {
    static_cast<void>(layer.QueryRadius(0.0F, 0.0F, 0.0F, 10.0F));
  } catch (const std::length_error &) {
    rejected = true;
  }
  assert(rejected);
}

void TestSemanticMetadataMustMatch() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  auto first = MakeObservation({0.10F, 0.10F, 0.10F}, {2U});
  layer.Update(first.observation);

  auto wrong_stamp = MakeObservation({1.10F, 0.10F, 0.10F}, {2U});
  wrong_stamp.observation.labels.stamp_ns += 1;
  bool stamp_rejected = false;
  try {
    static_cast<void>(layer.Update(wrong_stamp.observation));
  } catch (const std::invalid_argument &) {
    stamp_rejected = true;
  }
  assert(stamp_rejected);

  auto wrong_taxonomy = MakeObservation({1.10F, 0.10F, 0.10F}, {2U});
  wrong_taxonomy.observation.labels.taxonomy_version = 2U;
  bool taxonomy_rejected = false;
  try {
    static_cast<void>(layer.Update(wrong_taxonomy.observation));
  } catch (const std::invalid_argument &) {
    taxonomy_rejected = true;
  }
  assert(taxonomy_rejected);
}

void TestLargeCoordinatesFailClosed() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 0.01F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = false;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation({std::numeric_limits<float>::max(), 0.10F, 0.10F});
  observation.observation.frame.full_map = true;
  observation.observation.frame.incremental = false;
  bool rejected = false;
  try {
    static_cast<void>(layer.Update(observation.observation));
  } catch (const std::out_of_range &) {
    rejected = true;
  }
  assert(rejected);
}

void TestTruncatedRayIsReturnedToCaller() {
  SemanticOccupancyConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.raycast_free_space = true;
  config.max_ray_voxels_per_ray = 2U;

  SemanticOccupancyLayerCore layer(config);
  auto observation = MakeObservation({10.20F, 0.10F, 0.10F}, {}, 0.10F, 0.10F, 0.10F);
  const auto stats = layer.Update(observation.observation);
  assert(stats.rays_traced == 1U);
  assert(stats.truncated_rays == 1U);
  assert(stats.free_voxel_updates == 2U);
}

}  // namespace

int main() {
  TestGenerationAndDuplicateSequenceAreStable();
  TestGenerationMismatchFailsBeforeMutation();
  TestSoAChunkCarriesOneConsistentGeneration();
  TestCompleteChunkCanReplaceMapWithGenerationGuard();
  TestGeometryAndSemanticStatistics();
  TestEvidenceIsDeduplicatedPerFrame();
  TestRaycastingClearsStaleObstacle();
  TestFullMapReplacesPreviousState();
  TestMemoryLimitIsEnforced();
  TestQueryIsBounded();
  TestSemanticMetadataMustMatch();
  TestLargeCoordinatesFailClosed();
  TestTruncatedRayIsReturnedToCaller();
  return 0;
}
