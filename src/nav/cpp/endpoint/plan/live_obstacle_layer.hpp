#pragma once

#include <vector>

#include "motion/motion_layer.hpp"

namespace lingtu::nav::endpoint {

struct LiveObstacleLayerConfig {
  double voxel_size_m{0.08};
  double decay_s{0.45};
  double inflation_radius_m{0.12};
  double ray_clear_max_range_m{3.5};
  double ray_clearing_interval_s{0.33};
  std::size_t max_clearing_rays{160};
  int min_hits{1};
  bool ray_clearing_enabled{true};
};

using LiveObstacleLayerStats = MotionLayerStats;

class LiveObstacleLayer {
 public:
  explicit LiveObstacleLayer(LiveObstacleLayerConfig config = {});

  void configure(LiveObstacleLayerConfig config);
  void update(const std::vector<float> &xyzh, double stamp_s);
  void updateFromScan(const SensorOrigin &origin, const std::vector<float> &xyzh, double stamp_s);
  std::vector<float> snapshot(std::size_t max_points, double now_s);
  std::vector<float> snapshotDynamic(std::size_t max_points, double now_s);
  std::vector<DynamicCluster> dynamicClusters(std::size_t max_clusters, double now_s);
  MotionCellQuery query(float x, float y, float z, double now_s) const;
  void prune(double now_s);
  void clear();
  std::size_t size() const;
  LiveObstacleLayerStats stats() const;

 private:
  MotionLayer layer_;
};

}  // namespace lingtu::nav::endpoint
