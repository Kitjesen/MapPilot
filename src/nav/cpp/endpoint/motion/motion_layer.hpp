#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lingtu::nav::endpoint {

struct MotionLayerConfig {
  double voxel_size_m{0.08};
  double decay_s{0.45};
  double inflation_radius_m{0.12};
  double ray_clear_max_range_m{3.5};
  double ray_clearing_interval_s{0.33};
  std::size_t max_clearing_rays{160};
  int min_hits{1};
  std::uint32_t static_min_frames{3};
  std::uint32_t free_min_frames{2};
  std::uint32_t static_free_min_frames{4};
  std::size_t dynamic_min_cells{8};
  std::uint32_t dynamic_free_min_frames{3};
  double dynamic_min_speed_mps{0.25};
  double dynamic_max_speed_mps{2.50};
  double dynamic_max_z_speed_mps{0.75};
  double dynamic_min_dir_cos{0.30};
  double dynamic_min_height_m{0.10};
  double dynamic_max_height_m{2.20};
  std::uint32_t dynamic_confirm_frames{4};
  double dynamic_track_ttl_s{0.60};
  double dynamic_match_distance_m{0.50};
  bool ray_clearing_enabled{true};
};

struct MotionLayerStats {
  std::size_t cells{0};
  std::size_t unknown_cells{0};
  std::size_t free_cells{0};
  std::size_t occupied_cells{0};
  std::size_t static_cells{0};
  std::size_t dynamic_cells{0};
  std::size_t cleared_cells{0};
  std::size_t obstacle_cells{0};
  std::size_t raycast_rays{0};
  std::size_t raycast_voxels{0};
  std::size_t ray_cleared_cells{0};
  std::size_t prune_passes{0};
};

struct SensorOrigin {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool valid{false};
};

enum class MotionCellState {
  Unknown,
  Free,
  Occupied,
  Static,
  Cleared,
};

struct MotionCellQuery {
  MotionCellState state{MotionCellState::Unknown};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float height{0.0f};
  int hits{0};
  std::uint32_t hit_frames{0};
  std::uint32_t free_frames{0};
  double last_hit_s{-1.0};
  double last_free_s{-1.0};
};

struct DynamicCluster {
  std::uint32_t id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double age_s{0.0};
  double confidence{0.0};
  std::size_t cells{0};
};

class MotionLayer {
 public:
  explicit MotionLayer(MotionLayerConfig config = {});

  void configure(MotionLayerConfig config);
  void update(const std::vector<float> &xyzh, double stamp_s);
  void updateFromScan(const SensorOrigin &origin, const std::vector<float> &xyzh, double stamp_s);
  std::vector<float> snapshot(std::size_t max_points, double now_s);
  std::vector<float> snapshotDynamic(std::size_t max_points, double now_s);
  std::vector<DynamicCluster> dynamicClusters(std::size_t max_clusters, double now_s);
  MotionCellQuery query(float x, float y, float z, double now_s) const;
  void prune(double now_s);
  void clear();
  std::size_t size() const;
  MotionLayerStats stats() const;

 private:
  using CellState = MotionCellState;

  struct VoxelKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const VoxelKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const;
  };

  struct Cell {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float height{0.0f};
    double last_hit_s{-1.0};
    double last_free_s{-1.0};
    double last_update_s{0.0};
    int hits{0};
    std::uint32_t hit_frames{0};
    std::uint32_t free_frames{0};
    CellState state{CellState::Free};
    std::uint32_t free_observations{0};
  };

  struct ClusterTrack {
    std::uint32_t id{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double last_seen_s{0.0};
    double age_s{0.0};
    std::uint32_t observations{0};
    std::uint32_t moving_frames{0};
    bool confirmed{false};
  };

  VoxelKey makeKey(float x, float y, float z) const;
  VoxelKey makeKeyForSize(float x, float y, float z, double voxel_size_m) const;
  Cell cellAtKey(const VoxelKey &key, double stamp_s) const;
  void collectRayFreeKeys(const SensorOrigin &origin, const Cell &endpoint,
                          std::unordered_set<VoxelKey, VoxelKeyHash> &keys) const;
  void markHit(const Cell &sample, double stamp_s);
  void markFree(const VoxelKey &key, double stamp_s);
  void refreshStats();
  static bool isObstacle(CellState state);
  bool isStale(const Cell &cell, double now_s) const;

  MotionLayerConfig config_{};
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells_;
  MotionLayerStats stats_{};
  double last_ray_clearing_s_{-1.0};
  double last_prune_s_{-1.0};
  double current_stamp_s_{-1.0};
  std::unordered_set<VoxelKey, VoxelKeyHash> current_hit_keys_;
  std::unordered_set<VoxelKey, VoxelKeyHash> current_dynamic_keys_;
  std::vector<ClusterTrack> cluster_tracks_;
  std::vector<DynamicCluster> last_dynamic_clusters_;
  double last_cluster_update_s_{-1.0};
  std::uint32_t next_cluster_id_{1};
  SensorOrigin last_sensor_origin_{};
};

using LiveVoxelLayer = MotionLayer;

}  // namespace lingtu::nav::endpoint
