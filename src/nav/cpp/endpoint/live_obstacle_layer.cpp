#include "live_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {
namespace {

double positiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

MotionLayerConfig toMotionConfig(LiveObstacleLayerConfig config) {
  MotionLayerConfig out;
  out.voxel_size_m = positiveOr(config.voxel_size_m, 0.08);
  out.decay_s = std::max(0.0, config.decay_s);
  out.inflation_radius_m = std::max(0.0, config.inflation_radius_m);
  out.ray_clear_max_range_m = std::max(0.0, config.ray_clear_max_range_m);
  out.ray_clearing_interval_s = std::max(0.0, config.ray_clearing_interval_s);
  out.max_clearing_rays = std::max<std::size_t>(1, config.max_clearing_rays);
  out.min_hits = std::max(1, config.min_hits);
  out.ray_clearing_enabled = config.ray_clearing_enabled;
  return out;
}

}  // namespace

LiveObstacleLayer::LiveObstacleLayer(LiveObstacleLayerConfig config) {
  configure(config);
}

void LiveObstacleLayer::configure(LiveObstacleLayerConfig config) {
  layer_.configure(toMotionConfig(config));
}

void LiveObstacleLayer::update(const std::vector<float>& xyzh, double stamp_s) {
  layer_.update(xyzh, stamp_s);
}

void LiveObstacleLayer::updateFromScan(
    const SensorOrigin& origin,
    const std::vector<float>& xyzh,
    double stamp_s) {
  layer_.updateFromScan(origin, xyzh, stamp_s);
}

std::vector<float> LiveObstacleLayer::snapshot(std::size_t max_points, double now_s) {
  return layer_.snapshot(max_points, now_s);
}

std::vector<float> LiveObstacleLayer::snapshotDynamic(std::size_t max_points, double now_s) {
  return layer_.snapshotDynamic(max_points, now_s);
}

std::vector<DynamicCluster> LiveObstacleLayer::dynamicClusters(
    std::size_t max_clusters,
    double now_s) {
  return layer_.dynamicClusters(max_clusters, now_s);
}

MotionCellQuery LiveObstacleLayer::query(float x, float y, float z, double now_s) const {
  return layer_.query(x, y, z, now_s);
}

void LiveObstacleLayer::prune(double now_s) {
  layer_.prune(now_s);
}

void LiveObstacleLayer::clear() {
  layer_.clear();
}

std::size_t LiveObstacleLayer::size() const {
  return layer_.size();
}

LiveObstacleLayerStats LiveObstacleLayer::stats() const {
  return layer_.stats();
}

}  // namespace lingtu::nav::endpoint
