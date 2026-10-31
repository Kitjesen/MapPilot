#pragma once
#include "lingtu/maps/layers/elevation.hpp"
#include "lingtu/maps/layers/esdf.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "lingtu/maps/layers/block_grid.hpp"
#include "lingtu/maps/cloud.hpp"
#include "lingtu/maps/layers/grid.hpp"
#include "lingtu/maps/layers/ground_surface.hpp"
#include "lingtu/maps/layers/rolling_occupancy.hpp"
#include "lingtu/maps/layers/voxel.hpp"
#include "lingtu/maps/mapd/activation.hpp"

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
  std::string reference_map_id;
  std::int64_t reference_map_content_epoch{0};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::int64_t stamp_ns{0};
  std::string map_frame{"map"};
  std::string sensor_frame;
  Pose map_sensor;
  double sensor_origin_x_m{0.0};
  double sensor_origin_y_m{0.0};
  double sensor_origin_z_m{0.0};
  float pose_quality{0.0F};
  std::string pose_state;
  std::string pose_reason;
  OwnedPointCloud scan;
};

struct GeometrySnapshot {
  std::uint64_t reset_epoch{0U};
  std::uint64_t graph_revision{0U};
  std::uint64_t captured_sequence{0U};
  std::string frame_id{"map"};
  std::string frozen_dir;
  std::uint64_t keyframe_count{0U};
  std::int64_t captured_stamp_ns{0};
  Pose map_body;
  double sensor_origin_z_m{0.0};
};

struct GeometryApplied {
  std::uint64_t reset_epoch{0U};
  std::uint64_t graph_revision{0U};
  bool success{false};
  std::string message;
};

struct Config {
  bool build_extended_layers{true};
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
  std::chrono::milliseconds occupancy_update_period{50};
  std::chrono::milliseconds stale_after{1000};
  float accumulated_decay_factor{0.995F};
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
    // Surface hits do not prove that other heights in the column are free.
    // The independent occupancy layer owns ray-based clearing.
    value.column_carving = false;
    // Observed surfaces persist within the local window, including sparse hits.
    value.decay_rate = 0.0F;
    return value;
  }();
  layers::RollingOccupancyConfig occupancy = [] {
    layers::RollingOccupancyConfig value;
    return value;
  }();
  layers::GroundSurfaceConfig ground_surface;
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

struct Timing {
  std::uint64_t samples{0U};
  double last_ms{0.0};
  double total_ms{0.0};
  double max_ms{0.0};
  std::uint64_t max_sequence{0U};
  std::uint64_t max_generation{0U};

  void Observe(double elapsed_ms, std::uint64_t sequence, std::uint64_t generation) noexcept {
    ++samples;
    last_ms = elapsed_ms;
    total_ms += elapsed_ms;
    if (elapsed_ms > max_ms) {
      max_ms = elapsed_ms;
      max_sequence = sequence;
      max_generation = generation;
    }
  }
};

struct State {
  bool running{false};
  bool live{false};
  bool extended_layers_enabled{true};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::uint64_t generation{0U};
  std::uint64_t realtime_snapshot_generation{0U};
  std::uint64_t complete_snapshot_generation{0U};
  std::uint64_t realtime_snapshot_builds{0U};
  std::uint64_t complete_snapshot_builds{0U};
  std::uint64_t collision_snapshot_builds{0U};
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
  Timing process_timing;
  Timing occupancy_update_timing;
  Timing view_timing;
  Timing collision_build_timing;
};

struct Snapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::uint64_t generation{0U};
  Pose map_sensor;
  double sensor_origin_z_m{0.0};
  OwnedPointCloud live_cloud;
  OwnedPointCloud voxel_cloud;
  struct CollisionLayer {
    std::uint64_t generation{0U};
    double resolution_m{0.0};
    std::int32_t size_x{0};
    std::int32_t size_y{0};
    std::int32_t size_z{0};
    double min_x_m{0.0};
    double min_y_m{0.0};
    double min_z_m{0.0};
    double max_x_m{0.0};
    double max_y_m{0.0};
    double max_z_m{0.0};
    std::size_t occupied_cells{0U};
    bool complete{false};
    std::vector<std::uint8_t> occupied_bits;
    std::vector<std::uint8_t> measured_occupied_bits;
    std::vector<std::uint8_t> known_free_bits;
    std::vector<layers::CollisionHitBounds> hit_bounds;
  } collision;
  BlockGridSnapshot accumulated_cloud;
  layers::Grid2D occupancy;
  layers::Grid2D surface_projection;
  layers::GroundSurfaceResult ground_surface;
  layers::ElevationMapResult elevation;
  layers::EsdfResult esdf;
};

struct EngineView {
  State state;
  Snapshot snapshot;
};

enum class SnapshotDetail {
  kRealtime,
  kComplete,
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
  SubmitResult SubmitGeometry(GeometrySnapshot snapshot);
  std::vector<GeometryApplied> TakeGeometryAcks();
  void SetReferenceMap(MapIdentity identity);

  State GetState() const;
  Snapshot GetSnapshot() const;
  EngineView GetView(
      SnapshotDetail detail = SnapshotDetail::kComplete) const;
  bool WaitUntilProcessed(
      std::uint64_t reset_epoch,
      std::uint64_t sequence,
      std::chrono::milliseconds timeout) const;

 private:
  struct TransformedObservation {
    OwnedPointCloud cloud;
    std::vector<double> precise_xyz;
  };
  struct GeometryBuild {
    GeometrySnapshot source;
    PersistentBlockGrid grid;
    std::string error;
    std::uint64_t replayed_sequence{0U};
  };
  struct GeometrySuffix {
    std::shared_ptr<const OwnedPointCloud> cloud;
    double origin_x{0},origin_y{0},origin_z{0};
    std::int64_t stamp_ns{0};
    std::uint64_t sequence{0U};
  };

  struct QueueState {
    bool running{false};
    bool stop_requested{false};
    bool has_pending{false};
    std::uint64_t accepted_observations{0U};
    std::uint64_t replaced_observations{0U};
    std::uint64_t stale_observations{0U};
    std::uint64_t invalid_observations{0U};
  };

  static bool ValidateObservation(
      const Observation& observation,
      const Config& config,
      std::string* reason);
  static TransformedObservation TransformObservation(
      const Observation& observation);
  static void FilterExtendedObservation(
      OwnedPointCloud* cloud,
      const Observation& observation,
      const Config& config);
  static layers::Grid2D ProjectOccupancy(
      const layers::RollingOccupancySnapshot& occupancy,
      double sensor_z_m,
      const Config& config);
  static layers::ElevationMapResult ProjectElevation(
      const PointCloudView& cloud,
      const layers::Grid2D& geometry);

  void Run();
  void Process(Observation observation);
  static GeometryBuild RebuildGeometry(GeometrySnapshot snapshot, const Config& config);
  static GeometryBuild ReplayGeometrySuffix(GeometryBuild result, std::vector<GeometrySuffix> suffix);
  void InstallGeometry(GeometryBuild result);
  void Decay(std::int64_t now_ns);
  void ResetForEpoch(const Observation& observation);
  QueueState QueueStateLocked() const;
  State BuildStateLocked(std::int64_t now_ns, const QueueState& queue) const;
  void EnsureRealtimeSnapshotLocked() const;
  void EnsureCompleteSnapshotLocked() const;
  void BuildRealtimeSnapshotLocked() const;
  Snapshot RealtimeSnapshotLocked() const;

  Config config_;
  MapIdentity reference_map_;
  layers::VoxelLayerCore voxel_;
  layers::RollingOccupancyGrid occupancy_;
  PersistentBlockGrid accumulated_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool running_{false};
  bool stop_requested_{false};
  bool has_pending_{false};
  Observation pending_;
  std::optional<GeometrySnapshot> pending_geometry_;
  std::uint64_t highest_graph_revision_{0U};
  std::uint64_t highest_epoch_{0U};
  std::uint64_t highest_sequence_{0U};
  std::uint64_t accepted_observations_{0U};
  std::uint64_t replaced_observations_{0U};
  std::uint64_t stale_observations_{0U};
  std::uint64_t invalid_observations_{0U};
  std::thread worker_;
  std::future<GeometryBuild> geometry_job_;
  std::deque<GeometrySuffix> geometry_suffix_;
  std::size_t geometry_suffix_points_{0U};
  std::uint64_t geometry_discarded_sequence_{0U};

  mutable std::mutex data_mutex_;
  mutable std::condition_variable processed_cv_;
  std::uint64_t processed_epoch_{0U};
  std::uint64_t processed_sequence_{0U};
  std::uint64_t generation_{0U};
  std::uint64_t applied_graph_revision_{0U};
  std::uint64_t fresh_scan_cutoff_{0U};
  std::vector<GeometryApplied> geometry_acks_;
  std::uint64_t processed_observations_{0U};
  std::uint64_t epoch_resets_{0U};
  std::size_t accumulated_total_cells_{0U};
  std::size_t voxel_total_cells_{0U};
  mutable std::size_t voxel_snapshot_omitted_cells_{0U};
  std::uint64_t voxel_capacity_rejections_{0U};
  std::uint64_t accumulated_capacity_rejections_{0U};
  bool capacity_limited_{false};
  std::int64_t last_processed_steady_ns_{0};
  float pose_quality_{0.0F};
  std::string pose_state_;
  std::string pose_reason_;
  std::string last_error_;
  std::string geometry_error_;
  std::uint64_t failed_geometry_epoch_{0U};
  std::uint64_t failed_geometry_revision_{0U};
  mutable Snapshot snapshot_;
  mutable std::uint64_t realtime_snapshot_generation_{0U};
  mutable std::uint64_t complete_snapshot_generation_{0U};
  mutable std::uint64_t realtime_snapshot_builds_{0U};
  mutable std::uint64_t complete_snapshot_builds_{0U};
  mutable std::uint64_t collision_snapshot_builds_{0U};
  Timing process_timing_;
  Timing occupancy_update_timing_;
  mutable Timing view_timing_;
  mutable Timing collision_build_timing_;
};

}  // namespace lingtu::maps::mapd
