#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#include "input/obstacle.hpp"

namespace {

std::vector<float> point(float x, float y, float z, float h) {
  return {x, y, z, h};
}

std::vector<float> cluster(float x) {
  return {
      x, 0.00f, 0.0f, 0.4f, x, 0.10f, 0.0f, 0.4f, x, 0.20f, 0.0f, 0.4f, x, 0.30f, 0.0f, 0.4f,
  };
}

std::vector<float> freeRays(const std::vector<float> &target_xs) {
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

void primeFree(lingtu::nav::endpoint::MotionLayer &layer,
               const lingtu::nav::endpoint::SensorOrigin &origin,
               const std::vector<float> &target_xs) {
  const auto rays = freeRays(target_xs);
  layer.updateFromScan(origin, rays, 0.1);
  layer.updateFromScan(origin, rays, 0.2);
  layer.updateFromScan(origin, rays, 0.3);
}

bool containsNear(const std::vector<float> &xyzh, float x, float y, float eps) {
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

bool containsBeyondX(const std::vector<float> &xyzh, float x) {
  const std::size_t n = xyzh.size() / 4;
  for (std::size_t i = 0; i < n; ++i) {
    if (xyzh[i * 4] > x) {
      return true;
    }
  }
  return false;
}

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
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
  require(layer.query(1.0f, 0.0f, 0.0f, 1.0).state ==
              lingtu::nav::endpoint::MotionCellState::Unknown,
          "absent sparse voxel must query as explicit unknown");
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  require(layer.query(1.0f, 0.0f, 0.0f, 1.0).state ==
              lingtu::nav::endpoint::MotionCellState::Occupied,
          "first endpoint must query as occupied");
  require(containsNear(layer.snapshot(100, 1.0), 1.0f, 0.0f, 0.05f),
          "first endpoint must be occupied");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.1);
  const auto after_clear = layer.snapshot(100, 1.1);
  require(!containsNear(after_clear, 1.0f, 0.0f, 0.05f),
          "ray free evidence must clear old residue");
  require(containsNear(after_clear, 2.0f, 0.0f, 0.05f), "new endpoint must stay occupied");
  const auto stats = layer.stats();
  require(stats.cleared_cells >= 1, "cleared residue must be tracked as cleared evidence");
  require(stats.dynamic_cells == 0, "cleared residue must not be reported as a moving object");
  require(stats.ray_cleared_cells >= 1, "ray-cleared transition must be counted");
  const auto clusters = layer.dynamicClusters(8, 1.1);
  require(clusters.empty(), "cleared residue must not create a dynamic track");
  require(layer.query(1.0f, 0.0f, 0.0f, 1.1).state ==
              lingtu::nav::endpoint::MotionCellState::Cleared,
          "ray-cleared voxel must have explicit Cleared state");
}

void testEndpointHitWinsOverRayFree() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.1);
  require(containsNear(layer.snapshot(100, 1.1), 1.0f, 0.0f, 0.05f),
          "current endpoint must not be removed by its own ray");
}

void testOcclusionDoesNotClearBehindEndpoint() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.0);
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.1);
  const auto snapshot = layer.snapshot(100, 1.1);
  require(containsNear(snapshot, 1.0f, 0.0f, 0.05f), "near endpoint must be occupied");
  require(containsNear(snapshot, 2.0f, 0.0f, 0.05f),
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
  require(containsNear(layer.snapshot(100, 1.1), 1.0f, 0.0f, 0.05f),
          "repeated hit must become static obstacle");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.2);
  require(containsNear(layer.snapshot(100, 1.2), 1.0f, 0.0f, 0.05f),
          "static obstacle must survive first free ray");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.3);
  require(containsNear(layer.snapshot(100, 1.3), 1.0f, 0.0f, 0.05f),
          "static obstacle must survive second free ray");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.4);
  require(!containsNear(layer.snapshot(100, 1.4), 1.0f, 0.0f, 0.05f),
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
  require(layer.dynamicClusters(8, 1.1).empty(),
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
  require(containsNear(dynamic_points, 1.2f, 0.15f, 0.15f),
          "dynamic point snapshot must contain the current object, not its cleared residue");
  const auto expired = layer.dynamicClusters(8, 2.0);
  require(expired.empty(), "dynamic tracks must expire when no newer scan arrives");
  require(layer.stats().dynamic_cells == 0, "expired tracks must clear dynamic cell telemetry");
}

void testConfirmedMovingClusterAddsFutureOccupancyToSnapshot() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_max_speed_mps = 2.0;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  primeFree(layer, origin, {1.0f, 1.1f, 1.2f});
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  layer.dynamicClusters(8, 1.0);
  layer.updateFromScan(origin, cluster(1.1f), 1.1);
  layer.dynamicClusters(8, 1.1);
  layer.updateFromScan(origin, cluster(1.2f), 1.2);

  const auto current = layer.snapshot(100, 1.2);
  require(containsNear(current, 1.2f, 0.15f, 0.15f),
          "current snapshot must retain the measured moving obstacle");
  require(!containsNear(current, 1.45f, 0.15f, 0.20f),
          "current snapshot must not disguise predicted occupancy as measurement");
  const auto predicted = layer.snapshotPredictedDynamic(100, 1.2);
  const auto volumes = layer.predictedVolumes(1.2);
  require(volumes.size() == 1, "confirmed moving cluster must feed one swept volume to SCAN");
  require(volumes[0].end.x > volumes[0].start.x + .8,
          "SCAN volume must continuously cover the prediction horizon");
  require(volumes[0].minZ <= 0 && volumes[0].maxZ >= 0,
          "prediction vertical bounds must contain the measured cluster");
  require(containsNear(predicted, 1.45f, 0.15f, 0.20f),
          "confirmed moving obstacle must add bounded future occupancy");
  require(layer.stats().predicted_points == predicted.size() / 4,
          "motion-layer telemetry must expose the applied prediction point count");
  require(layer.stats().prediction_clusters == 1,
          "motion-layer telemetry must expose the applied prediction cluster count");
  require(std::abs(layer.stats().prediction_horizon_s -
                   lingtu::nav::endpoint::kDynamicPredictionHorizonS) < 1e-9,
          "motion-layer telemetry must expose the prediction horizon");
  const auto bounded = layer.snapshotPredictedDynamic(7, 1.2);
  require(bounded.size() / 4 == 7, "predicted occupancy must obey its point budget exactly");
  for (std::size_t offset = 0; offset + 3 < bounded.size(); offset += 4) {
    require(std::abs(bounded[offset + 3] - 0.4f) < 1e-5f,
            "predicted occupancy height must come from the observed cluster cells");
  }
}

void testConfirmedTrackPredictsThroughBriefOcclusion() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_max_speed_mps = 2.0;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  cfg.dynamic_track_ttl_s = 0.6;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  primeFree(layer, origin, {1.0f, 1.1f, 1.2f});
  layer.updateFromScan(origin, cluster(1.0f), 1.0);
  layer.dynamicClusters(8, 1.0);
  layer.updateFromScan(origin, cluster(1.1f), 1.1);
  layer.dynamicClusters(8, 1.1);
  layer.updateFromScan(origin, cluster(1.2f), 1.2);
  const auto confirmed = layer.dynamicClusters(8, 1.2);
  require(!confirmed.empty(), "test fixture must confirm a moving track");
  const auto confirmed_id = confirmed.front().id;

  layer.updateFromScan(origin, point(3.0f, -1.0f, 0.0f, 0.4f), 1.3);
  const auto occluded = layer.dynamicClusters(8, 1.3);
  require(!occluded.empty() && occluded.front().id == confirmed_id,
          "a confirmed track must survive one briefly occluded scan");
  require(containsBeyondX(layer.snapshotPredictedDynamic(100, 1.3), 1.4f),
          "brief occlusion must not make future dynamic occupancy disappear");

  layer.updateFromScan(origin, point(3.0f, -1.0f, 0.0f, 0.4f), 2.0);
  require(layer.snapshotPredictedDynamic(100, 2.0).empty(),
          "an occluded dynamic track must disappear after its TTL");
}

void testUnconfirmedStationaryAndExpiredTracksDoNotPredict() {
  auto cfg = baseConfig();
  cfg.static_min_frames = 10;
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_min_speed_mps = 0.2;
  cfg.dynamic_max_speed_mps = 2.0;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  cfg.dynamic_track_ttl_s = 0.6;
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};

  lingtu::nav::endpoint::MotionLayer stationary(cfg);
  primeFree(stationary, origin, {1.0f});
  stationary.updateFromScan(origin, cluster(1.0f), 1.0);
  require(stationary.snapshotPredictedDynamic(100, 1.0).empty(),
          "first stationary observation must not predict");
  stationary.updateFromScan(origin, cluster(1.0f), 1.1);
  require(stationary.snapshotPredictedDynamic(100, 1.1).empty(),
          "stationary cluster must not create future occupancy");

  lingtu::nav::endpoint::MotionLayer moving(cfg);
  primeFree(moving, origin, {1.0f, 1.1f, 1.2f});
  moving.updateFromScan(origin, cluster(1.0f), 1.0);
  require(moving.snapshotPredictedDynamic(100, 1.0).empty(),
          "first moving observation must not predict");
  moving.updateFromScan(origin, cluster(1.1f), 1.1);
  require(moving.snapshotPredictedDynamic(100, 1.1).empty(),
          "tentative moving cluster must not create future occupancy");
  moving.updateFromScan(origin, cluster(1.2f), 1.2);
  require(containsBeyondX(moving.snapshotPredictedDynamic(100, 1.2), 1.3f),
          "confirmed moving cluster must predict before expiry");
  require(moving.snapshotPredictedDynamic(100, 2.0).empty(),
          "expired dynamic track must remove all future occupancy");
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
  require(layer.dynamicClusters(8, 1.2).empty(),
          "motion without prior confirmed free space must remain an ordinary obstacle");
}

void testClearingCadenceAtUnixEpochStamps() {
  auto cfg = baseConfig();
  cfg.ray_clearing_interval_s = 0.1;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  for (int frame = 0; frame < 10; ++frame) {
    layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f),
                         1789920000.0 + 0.1 * frame);
  }
  require(layer.stats().raycast_rays == 10,
          "10 Hz source timestamps must not lose alternate clearing frames to rounding");
}

void testSparseMovingReturnsUseRepeatedObjectLevelFreeEvidence() {
  auto cfg = baseConfig();
  cfg.dynamic_min_cells = 4;
  cfg.dynamic_confirm_frames = 4;
  cfg.dynamic_free_min_frames = 3;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  // Only two rows of the object were observed free, each in a single scan.
  std::vector<float> rays;
  for (float x : {1.05f, 1.15f, 1.25f, 1.35f}) {
    rays.insert(rays.end(), {3.0f, 0.05f * 3.0f / x, 0.0f, 0.4f,
                            3.0f, 0.25f * 3.0f / x, 0.0f, 0.4f});
  }
  layer.updateFromScan(origin, rays, 0.1);
  std::vector<lingtu::nav::endpoint::DynamicCluster> tracks;
  for (int frame = 0; frame < 4; ++frame) {
    const float x = 1.05f + 0.1f * frame;
    std::vector<float> returns;
    for (float y : {0.05f, 0.25f, 0.45f, 0.65f}) {
      returns.insert(returns.end(), {x, y, 0.0f, 0.4f});
    }
    const double stamp = 1.0 + frame * 0.1;
    layer.updateFromScan(origin, returns, stamp);
    tracks = layer.dynamicClusters(8, stamp);
    if (frame < 3) require(tracks.empty(), "sparse motion still requires temporal confirmation");
  }
  require(tracks.size() == 1, "sparse moving object must not require prior rays in every voxel");
  require(tracks.front().vx > 0.5, "sparse object must retain its observed forward velocity");
}

void testSubVoxelMovingSurfaceRetainsVelocityEvidence() {
  auto cfg = baseConfig();
  cfg.voxel_size_m = 0.08;
  cfg.max_clearing_rays = 512;
  cfg.dynamic_min_cells = 8;
  cfg.dynamic_confirm_frames = 4;
  cfg.dynamic_free_min_frames = 3;
  cfg.dynamic_min_speed_mps = 0.25;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.5, true};
  std::vector<float> background;
  for (int y = -35; y <= 35; ++y) {
    for (int z = 0; z < 10; ++z) {
      const float target_y = 0.04f * y;
      const float target_z = 0.13f + 0.08f * z;
      background.insert(background.end(),
          {3.5f, target_y * 3.5f / 2.35f,
           0.5f + (target_z - 0.5f) * 3.5f / 2.35f, 0.0f});
    }
  }
  for (int frame = 0; frame < 4; ++frame)
    layer.updateFromScan(origin, background, 0.1 * (frame + 1));

  bool confirmed = false;
  for (int frame = 0; frame < 20; ++frame) {
    const float center_y = 0.83f - (2.2f / 6.0f) * 0.1f * frame;
    std::vector<float> returns = background;
    for (int y = 0; y < 9; ++y) for (int z = 0; z < 9; ++z) {
      const float height = 0.15f + 0.08f * z;
      returns.insert(returns.end(), {2.35f, center_y - 0.25f + 0.06f * y,
                                    height, height});
    }
    const double stamp = 1.0 + 0.1 * frame;
    layer.updateFromScan(origin, returns, stamp);
    const auto tracks = layer.dynamicClusters(8, stamp);
    for (const auto &track : tracks)
      if (track.x < 2.6 && track.vy < -0.25) confirmed = true;
  }
  require(confirmed,
          "a 0.3667 m/s surface at 10 Hz must confirm motion below the 8 cm voxel stride");
}

void testChangingWallVisibilityDoesNotCreateVelocityTrack() {
  auto cfg = baseConfig();
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  // A sliding visible patch on a fixed wall has a moving centroid, but no
  // repeated free-to-hit evidence at the wall itself.
  for (int frame = 0; frame < 12; ++frame) {
    std::vector<float> patch;
    for (int y = 0; y < 8; ++y) {
      patch.insert(patch.end(), {2.0f, 0.1f * frame + 0.2f * y, 0.0f, 0.4f});
    }
    const double stamp = 1.0 + frame * 0.1;
    layer.updateFromScan(origin, patch, stamp);
    require(layer.dynamicClusters(8, stamp).empty(),
            "changing visibility on a static wall must not predict wall motion");
  }
}

void testConfirmedTrackSurvivesRejectedVisibleHeightFitOnlyUntilTtl() {
  auto cfg = baseConfig();
  cfg.dynamic_min_cells = 3;
  const lingtu::nav::endpoint::SensorOrigin origin{0, 0, 0, true};
  for (const bool expires : {false, true}) {
    lingtu::nav::endpoint::MotionLayer layer(cfg);
    primeFree(layer, origin, {1.05f, 1.15f, 1.25f, 1.35f});
    std::vector<lingtu::nav::endpoint::DynamicCluster> tracks;
    for (int frame = 0; frame < 4; ++frame) {
      const double stamp = 1 + .1 * frame;
      layer.updateFromScan(origin, cluster(1.05f + .1f * frame), stamp);
      tracks = layer.dynamicClusters(8, stamp);
    }
    require(tracks.size() == 1, "height-fit regression requires an actually confirmed target");
    const auto confirmed = tracks.front();
    for (int frame = 4; frame < (expires ? 10 : 7); ++frame) {
      const double stamp = 1 + .1 * frame;
      auto points = cluster(1.05f + .1f * frame);
      for (std::size_t i = 0; i < points.size(); i += 4) points[i + 2] = expires ? -.49f : -.18f;
      layer.updateFromScan(origin, points, stamp);
      tracks = layer.dynamicClusters(8, stamp);
      require(tracks.size() == 1 && tracks.front().id == confirmed.id,
              "a rejected visible-height fit must retain confirmed motion within existing TTL");
      require(!layer.predictedVolumes(stamp).empty(), "bounded fallback must retain future geometry");
      if (frame < 6) {
        require(tracks.front().vz == confirmed.vz && tracks.front().confidence < confirmed.confidence,
                "rejected vertical fit must not refresh confidence or adopt its velocity");
      }
      const auto measured = layer.snapshot(2000, stamp);
      bool observed = false;
      for (std::size_t i = 0; i < measured.size(); i += 4)
        observed |= std::abs(measured[i] - points[0]) < .01f && measured[i + 2] == points[2];
      require(observed, "rejected velocity fit must never discard the current measured obstacle");
    }
    if (expires) {
      auto points = cluster(1.96f);
      for (std::size_t i = 0; i < points.size(); i += 4) points[i + 2] = -.49f;
      layer.updateFromScan(origin, points, 1.91);
      require(layer.dynamicClusters(8, 1.91).empty() && layer.predictedVolumes(1.91).empty(),
              "repeated rejected height fits must not extend the existing motion TTL");
      require(!layer.snapshot(2000, 1.91).empty(), "expired motion must remain measured geometry");
    } else {
      require(tracks.front().confidence >= confirmed.confidence,
              "plausible observed motion must resume the existing confirmed track");
    }
  }
}

void testRejectedHeightDoesNotPreserveStoppedReversedOrTooFastMotion() {
  auto cfg = baseConfig();
  cfg.dynamic_min_cells = 3;
  const lingtu::nav::endpoint::SensorOrigin origin{0, 0, 0, true};
  for (const float target_x : {1.35f, 1.25f, 1.75f}) {
    lingtu::nav::endpoint::MotionLayer layer(cfg);
    primeFree(layer, origin, {1.05f, 1.15f, 1.25f, 1.35f});
    for (int frame = 0; frame < 4; ++frame) {
      const double stamp = 1 + .1 * frame;
      layer.updateFromScan(origin, cluster(1.05f + .1f * frame), stamp);
      (void)layer.dynamicClusters(8, stamp);
    }
    require(!layer.predictedVolumes(1.3).empty(), "rejection fixture must first confirm motion");
    auto points = cluster(target_x);
    for (std::size_t i = 0; i < points.size(); i += 4) points[i + 2] = -.18f;
    layer.updateFromScan(origin, points, 1.4);
    require(layer.dynamicClusters(8, 1.4).empty() && layer.predictedVolumes(1.4).empty(),
            "a height anomaly must not excuse stopped, reversed or excessive planar motion");
    require(!layer.snapshot(2000, 1.4).empty(), "rejected motion remains a measured obstacle");
  }
}

void testMovingSensorDoesNotTurnNoisyStaticWallIntoDynamicObject() {
  lingtu::nav::endpoint::MotionLayer layer;
  std::mt19937 random(123);
  std::normal_distribution<float> noise(0.0f, 0.015f);
  for (int frame = 0; frame < 20; ++frame) {
    const float sensor_x = 0.1f * frame;
    std::vector<float> wall;
    for (int x = 0; x < 40; ++x) {
      for (int z = 0; z < 8; ++z) {
        const float height = 0.15f + 0.12f * z;
        wall.insert(wall.end(), {sensor_x - 1.0f + 0.06f * x + noise(random),
                                 -1.45f + noise(random), height + noise(random), height});
      }
    }
    const double stamp = 1.0 + 0.1 * frame;
    layer.updateFromScan({sensor_x, -0.4, 0.5, true}, wall, stamp);
    require(layer.dynamicClusters(8, stamp).empty(),
            "rays grazing a noisy static wall must not turn moving visibility into object velocity");
    require(!layer.snapshot(1000, stamp).empty(),
            "suppressing false wall motion must preserve its measured obstacle geometry");
  }
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
    require(layer.dynamicClusters(8, stamp).empty(),
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
  require(layer.dynamicClusters(8, 1.1).empty(),
          "an implausible centroid jump must not become a moving object");
}

void testOldFreeEvidenceDoesNotConfirmChangingWallVisibility() {
  auto cfg = baseConfig();
  cfg.dynamic_min_cells = 3;
  cfg.dynamic_free_min_frames = 2;
  cfg.dynamic_confirm_frames = 3;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  std::vector<float> columns;
  for (int i = 0; i < 11; ++i) columns.push_back(1.0f + 0.1f * i);
  primeFree(layer, origin, columns);
  for (int frame = 0; frame < 8; ++frame) {
    std::vector<float> visible;
    const int first_column = frame < 3 ? 0 : frame - 2;
    for (int i = first_column; i < 11; ++i) {
      const auto points = cluster(columns[i]);
      visible.insert(visible.end(), points.begin(), points.end());
    }
    const double stamp = 1.0 + 0.1 * frame;
    // Occlusion changes the visible wall centroid, not the wall's position.
    layer.update(visible, stamp);
    require(layer.dynamicClusters(8, stamp).empty(),
            "one historical free observation must not confirm moving wall fragments");
    require(!layer.snapshot(1000, stamp).empty(),
            "unconfirmed wall must remain in measured obstacle geometry");
  }
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
  require(layer.stats().prune_passes == passes,
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
  require(containsNear(snapshot, 10.35f, -4.0f, 0.05f),
          "bounded snapshot must retain the nearest obstacle to the current sensor origin");
}

void testInflatedBoundedSnapshotPreservesNearestHazard() {
  auto cfg = baseConfig();
  cfg.ray_clearing_enabled = false;
  cfg.inflation_radius_m = 0.12;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  std::vector<float> points;
  for (int i = 0; i < 64; ++i) {
    points.insert(points.end(), {
                                    10.0f + 2.0f * static_cast<float>(i),
                                    5.0f,
                                    0.0f,
                                    0.4f,
                                });
  }
  points.insert(points.end(), {0.35f, 0.0f, 0.0f, 0.4f});
  layer.updateFromScan(origin, points, 1.0);

  const auto snapshot = layer.snapshot(1, 1.0);
  require(snapshot.size() == 4, "inflated one-point budget must emit one obstacle");
  require(std::hypot(snapshot[0], snapshot[1]) < 0.6f,
          "inflated bounded snapshot must retain the nearest obstacle cell");
}

void testScanHitsKeepPointCountsAndWinOverCrossingRays() {
  lingtu::nav::endpoint::MotionLayer layer(baseConfig());
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  const std::vector<float> scan{
      2.0f, 0.0f, 0.0f, 0.4f,
      1.01f, 0.0f, 0.0f, 0.4f,
      1.02f, 0.0f, 0.0f, 0.7f,
      1.03f, 0.0f, 0.0f, 0.5f,
  };
  for (int frame = 0; frame < 3; ++frame) {
    const double stamp = 1.0 + 0.1 * frame;
    layer.updateFromScan(origin, scan, stamp);
    const auto hit = layer.query(1.01f, 0.0f, 0.0f, stamp);
    require(hit.hits == 3 * (frame + 1) &&
                hit.hit_frames == static_cast<std::uint32_t>(frame + 1),
            "same-voxel points must each count as hits but only one observation frame");
    require(hit.x == 1.02f && hit.height == 0.7f,
            "the highest hit must remain the voxel representative");
    require(hit.free_frames == 0 && hit.last_free_s < 0.0,
            "another endpoint's ray must not clear a current hit");
    require(hit.state == (frame == 2 ? lingtu::nav::endpoint::MotionCellState::Static
                                     : lingtu::nav::endpoint::MotionCellState::Occupied),
            "static confirmation must advance once per scan");
  }
}

void testBoundedSnapshotReranksAfterSensorMoves() {
  auto cfg = baseConfig();
  cfg.ray_clearing_enabled = false;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const std::vector<float> points{
      10.0f, -4.0f, 1.0f, 0.2f,
      12.0f, -4.0f, 1.0f, 0.4f,
      14.0f, -4.0f, 1.0f, 0.6f,
      16.0f, -4.0f, 1.0f, 0.8f,
      10.0f, -4.0f, 9.0f, 1.0f,
  };
  layer.updateFromScan({9.0, -4.0, 1.0, true}, points, 1.0);
  const auto before = layer.snapshot(2, 1.0);
  require(before.size() == 8 && before[0] == 10.0f && before[4] == 12.0f &&
              before[2] == 1.0f && before[6] == 1.0f,
          "bounded snapshot must rank by full 3D distance and emit nearest first");

  layer.updateFromScan({17.0, -4.0, 1.0, true}, points, 1.1);
  const auto after = layer.snapshot(2, 1.1);
  require(after.size() == 8 && after[0] == 16.0f && after[4] == 14.0f &&
              after[3] == 0.8f && after[7] == 0.6f,
          "distance ranking must follow the new sensor origin and preserve obstacle height");
  require(layer.snapshot(0, 1.1).size() == points.size(),
          "unbounded snapshot after a bounded query must still include every obstacle");
}

void testInflatedCoarseReductionCannotReplaceNearHazardWithTallerFarPoint() {
  auto cfg = baseConfig();
  cfg.ray_clearing_enabled = false;
  cfg.inflation_radius_m = 0.12;
  lingtu::nav::endpoint::MotionLayer layer(cfg);
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  std::vector<float> points{{0.35f, 0.0f, 0.0f, 0.2f}};
  for (int index = 0; index < 180; ++index) {
    const float angle = 0.17f * static_cast<float>(index);
    const float radius = 2.0f + 0.005f * static_cast<float>(index);
    points.insert(points.end(), {
                                    radius * std::cos(angle),
                                    radius * std::sin(angle),
                                    0.0f,
                                    1.0f,
                                });
  }
  layer.updateFromScan(origin, points, 1.0);

  const auto snapshot = layer.snapshot(1, 1.0);

  require(snapshot.size() == 4, "coarse-reduced snapshot must obey its point budget");
  require(std::hypot(snapshot[0], snapshot[1]) < 0.6f,
          "coarse reduction must rank distance before obstacle height");
}

}  // namespace

int main() {
  testRayClearsResidueWithoutInventingMovingObject();
  testEndpointHitWinsOverRayFree();
  testOcclusionDoesNotClearBehindEndpoint();
  testScanHitsKeepPointCountsAndWinOverCrossingRays();
  testStaticNeedsRepeatedFreeEvidence();
  testStationaryClusterDoesNotBecomeDynamic();
  testMovingCurrentClusterProducesVelocityTrack();
  testConfirmedMovingClusterAddsFutureOccupancyToSnapshot();
  testConfirmedTrackPredictsThroughBriefOcclusion();
  testUnconfirmedStationaryAndExpiredTracksDoNotPredict();
  testPriorFreeEvidenceIsRequired();
  testClearingCadenceAtUnixEpochStamps();
  testSparseMovingReturnsUseRepeatedObjectLevelFreeEvidence();
  testSubVoxelMovingSurfaceRetainsVelocityEvidence();
  testChangingWallVisibilityDoesNotCreateVelocityTrack();
  testMovingSensorDoesNotTurnNoisyStaticWallIntoDynamicObject();
  testConfirmedTrackSurvivesRejectedVisibleHeightFitOnlyUntilTtl();
  testRejectedHeightDoesNotPreserveStoppedReversedOrTooFastMotion();
  testAlternatingJitterDoesNotBecomeDynamic();
  testImplausibleSpeedDoesNotBecomeDynamic();
  testOldFreeEvidenceDoesNotConfirmChangingWallVisibility();
  testSameFrameQueriesReusePrunePass();
  testBoundedSnapshotUsesFullBudget();
  testBoundedSnapshotPreservesNearestHazard();
  testBoundedSnapshotReranksAfterSensorMoves();
  testInflatedBoundedSnapshotPreservesNearestHazard();
  testInflatedCoarseReductionCannotReplaceNearHazardWithTallerFarPoint();
  std::cout << "test_motion_layer passed\n";
  return 0;
}
