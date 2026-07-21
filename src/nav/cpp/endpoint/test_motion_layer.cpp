#include "motion_layer.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

std::vector<float> point(float x, float y, float z, float h) {
  return {x, y, z, h};
}

std::vector<float> cluster(float x) {
  return {
      x, 0.00f, 0.0f, 0.4f,
      x, 0.10f, 0.0f, 0.4f,
      x, 0.20f, 0.0f, 0.4f,
      x, 0.30f, 0.0f, 0.4f,
  };
}

std::vector<float> freeRays(const std::vector<float>& target_xs) {
  std::vector<float> out;
  constexpr float endpoint_x = 3.0f;
  for (const float target_x : target_xs) {
    for (int i = 0; i < 4; ++i) {
      const float target_y = 0.10f * static_cast<float>(i);
      out.insert(out.end(), {
          endpoint_x,
          target_y * endpoint_x / target_x,
          0.0f,
          0.4f,
      });
    }
  }
  return out;
}

void primeFree(
    lingtu::nav::endpoint::MotionLayer& layer,
    const lingtu::nav::endpoint::SensorOrigin& origin,
    const std::vector<float>& target_xs) {
  const auto rays = freeRays(target_xs);
  layer.updateFromScan(origin, rays, 0.1);
  layer.updateFromScan(origin, rays, 0.2);
  layer.updateFromScan(origin, rays, 0.3);
}

bool containsNear(const std::vector<float>& xyzh, float x, float y, float eps) {
  const std::size_t n = xyzh.size() / 4;
  for (std::size_t i = 0; i < n; ++i) {
    const float dx = xyzh[i * 4 + 0] - x;
    const float dy = xyzh[i * 4 + 1] - y;
    if (std::hypot(dx, dy) <= eps) {
      return true;
    }
  }
  return false;
}

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::nav::endpoint::MotionLayerConfig baseConfig() {
  lingtu::nav::endpoint::MotionLayerConfig cfg;
  cfg.voxel_size_m = 0.10;
  cfg.decay_s = 10.0;
  cfg.inflation_radius_m = 0.0;
  cfg.ray_clear_max_range_m = 4.0;
  cfg.ray_clearing_interval_s = 0.0;
  cfg.max_clearing_rays = 100;
  cfg.min_hits = 1;
  cfg.static_min_frames = 3;
  cfg.free_min_frames = 1;
  cfg.static_free_min_frames = 3;
  cfg.ray_clearing_enabled = true;
  return cfg;
}

void testRayClearsResidueWithoutInventingMovingObject() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  require(
      layer.query(1.0f, 0.0f, 0.0f, 1.0).state ==
          lingtu::nav::endpoint::MotionCellState::Unknown,
      "absent sparse voxel must query as explicit unknown");
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  require(
      layer.query(1.0f, 0.0f, 0.0f, 1.0).state ==
          lingtu::nav::endpoint::MotionCellState::Occupied,
      "first endpoint must query as occupied");
  require(
      containsNear(layer.snapshot(100, 1.0), 1.0f, 0.0f, 0.05f),
      "first endpoint must be occupied");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.1);
  const auto after_clear = layer.snapshot(100, 1.1);
  require(!containsNear(after_clear, 1.0f, 0.0f, 0.05f), "ray free evidence must clear old residue");
  require(containsNear(after_clear, 2.0f, 0.0f, 0.05f), "new endpoint must stay occupied");
  const auto stats = layer.stats();
  require(stats.cleared_cells >= 1, "cleared residue must be tracked as cleared evidence");
  require(stats.dynamic_cells == 0, "cleared residue must not be reported as a moving object");
  require(stats.ray_cleared_cells >= 1, "ray-cleared transition must be counted");
  const auto clusters = layer.dynamicClusters(8, 1.1);
  require(clusters.empty(), "cleared residue must not create a dynamic track");
  require(
      layer.query(1.0f, 0.0f, 0.0f, 1.1).state ==
          lingtu::nav::endpoint::MotionCellState::Cleared,
      "ray-cleared voxel must have explicit Cleared state");
}

void testEndpointHitWinsOverRayFree() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.1);
  require(
      containsNear(layer.snapshot(100, 1.1), 1.0f, 0.0f, 0.05f),
      "current endpoint must not be removed by its own ray");
}

void testOcclusionDoesNotClearBehindEndpoint() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.0);
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.1);
  const auto snapshot = layer.snapshot(100, 1.1);
  require(containsNear(snapshot, 1.0f, 0.0f, 0.05f), "near endpoint must be occupied");
  require(
      containsNear(snapshot, 2.0f, 0.0f, 0.05f),
      "ray must not clear cells hidden behind a nearer endpoint");
}

void testStaticNeedsRepeatedFreeEvidence() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 2;
  cfg.free_min_frames = 1;
  cfg.static_free_min_frames = 3;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.1);
  require(
      containsNear(layer.snapshot(100, 1.1), 1.0f, 0.0f, 0.05f),
      "repeated hit must become static obstacle");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.2);
  require(
      containsNear(layer.snapshot(100, 1.2), 1.0f, 0.0f, 0.05f),
      "static obstacle must survive first free ray");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.3);
  require(
      containsNear(layer.snapshot(100, 1.3), 1.0f, 0.0f, 0.05f),
      "static obstacle must survive second free ray");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.4);
  require(
      !containsNear(layer.snapshot(100, 1.4), 1.0f, 0.0f, 0.05f),
      "static obstacle should clear only after repeated free evidence");
}

void testStationaryClusterDoesNotBecomeDynamic() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_confirm_frames = 2;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  require(layer.dynamicClusters(8, 1.0).empty(), "first observation must stay tentative");
  layer.updateFromScan(origin, cluster(1.0f), 1.1);
  require(
      layer.dynamicClusters(8, 1.1).empty(),
      "stationary non-static cluster must not be reported as moving");
}

void testMovingCurrentClusterProducesVelocityTrack() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_max_speed_mps = 2.0;
  cfg.dynamic_max_z_speed_mps = 0.5;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  cfg.dynamic_min_dir_cos = 0.5;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  primeFree(layer, origin, {1.0f, 1.1f, 1.2f, 1.3f});
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  require(layer.dynamicClusters(8, 1.0).empty(), "first moving observation must stay tentative");
  layer.updateFromScan(origin, cluster(1.1f), 1.1);
  require(layer.dynamicClusters(8, 1.1).empty(), "one motion step must stay tentative");
  layer.updateFromScan(origin, cluster(1.2f), 1.2);
  const auto tracks = layer.dynamicClusters(8, 1.2);
  require(!tracks.empty(), "moving current cluster must create a confirmed dynamic track");
  require(tracks.front().id > 0, "dynamic track must have stable product id");
  require(tracks.front().x > 1.1, "track centroid must describe the current object position");
  require(tracks.front().vx > 0.5, "track velocity must describe current forward motion");
  require(tracks.front().cells >= 3, "dynamic track must report current occupied cells");
  const auto dynamic_points = layer.snapshotDynamic(100, 1.2);
  require(
      containsNear(dynamic_points, 1.2f, 0.15f, 0.15f),
      "dynamic point snapshot must contain the current object, not its cleared residue");
  const auto expired = layer.dynamicClusters(8, 2.0);
  require(expired.empty(), "dynamic tracks must expire when no newer scan arrives");
  require(layer.stats().dynamic_cells == 0, "expired tracks must clear dynamic cell telemetry");
}

void testPriorFreeEvidenceIsRequired() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  layer.dynamicClusters(8, 1.0);
  layer.updateFromScan(origin, cluster(1.1f), 1.1);
  layer.dynamicClusters(8, 1.1);
  layer.updateFromScan(origin, cluster(1.2f), 1.2);
  require(
      layer.dynamicClusters(8, 1.2).empty(),
      "motion without prior confirmed free space must remain an ordinary obstacle");
}

void testAlternatingJitterDoesNotBecomeDynamic() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 20;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_max_speed_mps = 2.0;
  cfg.dynamic_min_dir_cos = 0.5;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  primeFree(layer, origin, {1.0f, 1.12f});
  for (int i = 0; i < 8; ++i) {
    const float x = i % 2 == 0 ? 1.0f : 1.12f;
    const double stamp = 1.0 + 0.1 * static_cast<double>(i);
    layer.updateFromScan(origin, cluster(x), stamp);
    require(
        layer.dynamicClusters(8, stamp).empty(),
        "alternating registration jitter must not become a moving object");
  }
}

void testImplausibleSpeedDoesNotBecomeDynamic() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 2;
  cfg.dynamic_max_speed_mps = 2.0;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  primeFree(layer, origin, {1.0f, 1.6f});
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  layer.dynamicClusters(8, 1.0);
  layer.updateFromScan(origin, cluster(1.6f), 1.1);
  require(
      layer.dynamicClusters(8, 1.1).empty(),
      "an implausible centroid jump must not become a moving object");
}

void testSameFrameQueriesReusePrunePass() {
  auto cfg = baseConfig();
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  const auto passes = layer.stats().prune_passes;
  require(passes > 0, "scan update must execute one prune pass");
  layer.dynamicClusters(8, 1.0);
  layer.snapshot(100, 1.0);
  require(
      layer.stats().prune_passes == passes,
      "same-frame dynamic and snapshot queries must reuse the completed prune pass");
  layer.prune(1.1);
  require(layer.stats().prune_passes == passes + 1, "a later frame must prune again");
}

void testBoundedSnapshotUsesFullBudget() {
  auto cfg = baseConfig();
  cfg.ray_clearing_enabled = false;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  std::vector<float> points;
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      points.insert(points.end(), {
          0.10f * static_cast<float>(x),
          0.10f * static_cast<float>(y),
          0.0f,
          0.4f,
      });
    }
  }
  layer.update(points, 1.0);
  const auto snapshot = layer.snapshot(37, 1.0);
  require(snapshot.size() / 4 == 37, "bounded snapshot must use its full point budget");
}

void testBoundedSnapshotPreservesNearestHazard() {
  auto cfg = baseConfig();
  cfg.ray_clearing_enabled = false;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{10.0, -4.0, 0.0, true};
  std::vector<float> points;
  for (int i = 0; i < 250; ++i) {
    points.insert(points.end(), {
        30.0f + 0.10f * static_cast<float>(i),
        -4.0f,
        0.0f,
        0.4f,
    });
  }
  points.insert(points.end(), {10.35f, -4.0f, 0.0f, 0.4f});
  layer.updateFromScan(origin, points, 1.0);

  const auto snapshot = layer.snapshot(1, 1.0);
  require(snapshot.size() == 4, "one-point budget must emit one obstacle");
  require(
      containsNear(snapshot, 10.35f, -4.0f, 0.05f),
      "bounded snapshot must retain the nearest obstacle to the current sensor origin");
}

}  // namespace

int main() {
  testRayClearsResidueWithoutInventingMovingObject();
  testEndpointHitWinsOverRayFree();
  testOcclusionDoesNotClearBehindEndpoint();
  testStaticNeedsRepeatedFreeEvidence();
  testStationaryClusterDoesNotBecomeDynamic();
  testMovingCurrentClusterProducesVelocityTrack();
  testPriorFreeEvidenceIsRequired();
  testAlternatingJitterDoesNotBecomeDynamic();
  testImplausibleSpeedDoesNotBecomeDynamic();
  testSameFrameQueriesReusePrunePass();
  testBoundedSnapshotUsesFullBudget();
  testBoundedSnapshotPreservesNearestHazard();
  std::cout << "test_motion_layer passed\n";
  return 0;
}
