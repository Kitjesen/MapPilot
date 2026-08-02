#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "lingtu/maps/block_grid.hpp"
#include "lingtu/maps/cloud.hpp"
#include "lingtu/maps/layers/grid.hpp"
#include "lingtu/maps/layers/rolling_occupancy.hpp"
#include "lingtu/maps/layers/voxel.hpp"

namespace lingtu::maps::mapd {

struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct Observation {
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::int64_t stamp_ns{0};
  std::string map_frame{"map"};
  std::string sensor_frame;
  Pose map_sensor;
  float sensor_origin_x_m{0.0F};
  float sensor_origin_y_m{0.0F};
  float sensor_origin_z_m{0.0F};
  float pose_quality{0.0F};
  std::string pose_state;
  std::string pose_reason;
  OwnedPointCloud scan;
};

struct Config {
  std::size_t max_points_per_observation{300000U};
  float min_range_m{0.25F};
  float max_range_m{30.0F};
  float min_height_from_sensor_m{-2.0F};
  float max_height_from_sensor_m{3.0F};
  float column_carving_min_height_from_sensor_m{-0.7F};
  float column_carving_max_height_from_sensor_m{1.8F};
  float occupancy_min_height_from_sensor_m{-1.0F};
  float occupancy_max_height_from_sensor_m{2.0F};
  std::chrono::milliseconds decay_period{250};
  std::chrono::milliseconds stale_after{1000};
  float accumulated_decay_factor{0.995F};
  bool accumulated_column_carving{true};
  bool accept_degraded_pose{true};
  float accumulated_snapshot_radius_m{30.0F};
  float accumulated_snapshot_min_z_from_sensor_m{-3.0F};
  float accumulated_snapshot_max_z_from_sensor_m{5.0F};
  std::size_t max_accumulated_snapshot_cells{200000U};
  float voxel_snapshot_radius_m{30.0F};
  float voxel_snapshot_min_z_from_sensor_m{-3.0F};
  float voxel_snapshot_max_z_from_sensor_m{5.0F};
  std::size_t max_voxel_snapshot_points{200000U};
  layers::VoxelLayerConfig voxel = [] {
    layers::VoxelLayerConfig value;
    value.max_range_m = 0.0F;
    value.min_z_m = -10000.0F;
    value.max_z_m = 10000.0F;
    return value;
  }();
  layers::RollingOccupancyConfig occupancy = [] {
    layers::RollingOccupancyConfig value;
    value.decay_after_ns = 1000000000LL;
    value.decay_factor = 0.5F;
    return value;
  }();
  BlockGridConfig accumulated;
};

enum class SubmitCode {
  kAccepted,
  kReplacedPending,
  kInvalid,
  kStale,
  kStopped,
};

struct SubmitResult {
  SubmitCode code{SubmitCode::kInvalid};
  std::string reason;

  bool accepted() const {
    return code == SubmitCode::kAccepted || code == SubmitCode::kReplacedPending;
  }
};

struct State {
  bool running{false};
  bool live{false};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::uint64_t generation{0U};
  std::uint64_t accepted_observations{0U};
  std::uint64_t processed_observations{0U};
  std::uint64_t replaced_observations{0U};
  std::uint64_t stale_observations{0U};
  std::uint64_t invalid_observations{0U};
  std::uint64_t epoch_resets{0U};
  std::size_t queue_depth{0U};
  std::size_t live_points{0U};
  std::size_t voxel_points{0U};
  std::size_t voxel_cells{0U};
  std::size_t voxel_snapshot_omitted_cells{0U};
  std::uint64_t voxel_capacity_rejections{0U};
  std::size_t accumulated_cells{0U};
  std::size_t accumulated_snapshot_cells{0U};
  std::uint64_t accumulated_capacity_rejections{0U};
  bool capacity_limited{false};
  float pose_quality{0.0F};
  std::string pose_state;
  std::string pose_reason;
  std::string last_error;
};

struct Snapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::uint64_t generation{0U};
  Pose map_sensor;
  OwnedPointCloud live_cloud;
  OwnedPointCloud voxel_cloud;
  BlockGridSnapshot accumulated_cloud;
  layers::Grid2D occupancy;
  layers::ElevationMapResult elevation;
  layers::EsdfResult esdf;
};

struct EngineView {
  State state;
  Snapshot snapshot;
};

class LiveMapEngine final {
 public:
  explicit LiveMapEngine(Config config = {});
  ~LiveMapEngine();

  LiveMapEngine(const LiveMapEngine&) = delete;
  LiveMapEngine& operator=(const LiveMapEngine&) = delete;

  void Start();
  void Stop();
  SubmitResult Submit(Observation observation);

  State GetState() const;
  Snapshot GetSnapshot() const;
  EngineView GetView() const;
  bool WaitUntilProcessed(
      std::uint64_t reset_epoch,
      std::uint64_t sequence,
      std::chrono::milliseconds timeout) const;

 private:
  static bool ValidateObservation(
      const Observation& observation,
      const Config& config,
      std::string* reason);
  static OwnedPointCloud TransformObservation(
      const Observation& observation,
      const Config& config);
  static layers::Grid2D ProjectOccupancy(
      const layers::RollingOccupancySnapshot& occupancy,
      float sensor_z_m,
      const Config& config);
  static layers::ElevationMapResult ProjectElevation(
      const PointCloudView& cloud,
      const layers::Grid2D& geometry);

  void Run();
  void Process(Observation observation);
  void Decay(std::int64_t now_ns);
  void ResetForEpoch(const Observation& observation);
  void RefreshSnapshotLocked();

  Config config_;
  layers::VoxelLayerCore voxel_;
  layers::RollingOccupancyGrid occupancy_;
  PersistentBlockGrid accumulated_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool running_{false};
  bool stop_requested_{false};
  bool has_pending_{false};
  Observation pending_;
  std::uint64_t highest_epoch_{0U};
  std::uint64_t highest_sequence_{0U};
  std::uint64_t accepted_observations_{0U};
  std::uint64_t replaced_observations_{0U};
  std::uint64_t stale_observations_{0U};
  std::uint64_t invalid_observations_{0U};
  std::thread worker_;

  mutable std::mutex data_mutex_;
  mutable std::condition_variable processed_cv_;
  std::uint64_t processed_epoch_{0U};
  std::uint64_t processed_sequence_{0U};
  std::uint64_t generation_{0U};
  std::uint64_t processed_observations_{0U};
  std::uint64_t epoch_resets_{0U};
  std::size_t accumulated_total_cells_{0U};
  std::size_t voxel_total_cells_{0U};
  std::size_t voxel_snapshot_omitted_cells_{0U};
  std::uint64_t voxel_capacity_rejections_{0U};
  std::uint64_t accumulated_capacity_rejections_{0U};
  bool capacity_limited_{false};
  std::int64_t last_processed_steady_ns_{0};
  float pose_quality_{0.0F};
  std::string pose_state_;
  std::string pose_reason_;
  std::string last_error_;
  Snapshot snapshot_;
};

}  // namespace lingtu::maps::mapd
