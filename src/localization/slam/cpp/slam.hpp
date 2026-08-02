#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::slam {

struct Status {
  bool ok = false;
  std::string message;

  static Status Ok(std::string message = "ok") {
    return Status{true, std::move(message)};
  }

  static Status Error(std::string message) {
    return Status{false, std::move(message)};
  }
};

enum class SlamMode {
  Mapping,
  Localization,
};

enum class SlamState {
  Unconfigured,
  Initializing,
  Mapping,
  Localizing,
  Tracking,
  Degraded,
  Lost,
  Failed,
};

struct SlamConfig {
  std::string backend = "fastlio2";
  std::string config_path;
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string body_frame = "body";
  bool publish_state_estimation_at_scan = false;
};

struct Pose3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

struct Transform3d {
  std::string frame_id = "map";
  std::string child_frame_id = "odom";
  Pose3d pose;
};

struct ImuSample {
  double stamp_s = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
};

struct PointXYZIT {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  std::int64_t offset_time_ns = 0;
  std::uint8_t line = 0;
  std::uint8_t tag = 0;
};

struct LidarFrame {
  double stamp_s = 0.0;
  std::string frame_id = "lidar";
  std::vector<PointXYZIT> points;
};

struct GnssSample {
  double stamp_s = 0.0;
  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  double ve = 0.0;
  double vn = 0.0;
  double vu = 0.0;
  double cov_e = 99.0;
  double cov_n = 99.0;
  double cov_u = 99.0;
  std::string fix_type = "NO_FIX";
};

struct OdomSample {
  double stamp_s = 0.0;
  Pose3d odom_body;
  bool has_velocity = false;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
};

struct Cloud {
  double stamp_s = 0.0;
  std::string frame_id = "map";
  std::vector<PointXYZIT> points;
};

struct GnssFusionHealth {
  bool enabled = false;
  bool alignment_locked = false;
  std::string last_fix_type = "NONE";
  double last_gnss_age_s = 0.0;
  double last_residual_m = 0.0;
  int relock_count = 0;
};

struct FastLioLidarUpdateDiagnostics {
  bool attempted = false;
  bool accepted = false;
  std::uint64_t attempt_sequence = 0U;
  std::string rejection_reason = "none";
  std::string previous_rejection_reason = "none";
  std::uint64_t consecutive_rejections = 0U;
  std::uint64_t downsampled_points = 0U;
  std::uint64_t effective_points = 0U;

  double candidate_translation_m = 0.0;
  double candidate_rotation_rad = 0.0;
  double candidate_velocity_mps = 0.0;
  double candidate_velocity_delta_mps = 0.0;
  double max_update_translation_m = 0.0;
  double max_update_rotation_rad = 0.0;
  double max_update_velocity_mps = 0.0;
  double max_update_velocity_delta_mps = 0.0;

  bool information_ldlt_evaluated = false;
  bool information_ldlt_decomposition_success = false;
  bool information_ldlt_positive = false;
  bool candidate_covariance_evaluated = false;
  bool candidate_covariance_finite = false;
  bool candidate_covariance_positive_diagonal = false;
  bool posterior_covariance_evaluated = false;
  bool posterior_covariance_finite = false;
  bool posterior_covariance_positive_diagonal = false;
};

struct SlamOutputs {
  SlamState state = SlamState::Unconfigured;
  double stamp_s = 0.0;
  double confidence = 0.0;
  std::string reason;

  std::optional<Pose3d> odometry_odom_body;
  std::optional<Pose3d> state_estimation_at_scan;
  std::optional<Cloud> registered_cloud_body;
  std::optional<Cloud> map_cloud_map;
  std::optional<Cloud> saved_map_cloud_map;
  std::optional<Transform3d> map_odom_tf;
  std::uint64_t observation_sequence = 0U;
  std::uint64_t source_epoch = 0U;

  int saved_map_points = 0;
  bool alive = false;
  bool map_loaded = false;
  bool map_frame_jump = false;
  std::uint64_t map_frame_jump_sequence = 0U;
  bool relocalization_supported = false;
  bool saved_map_relocalization_supported = false;
  std::string relocalization_state = "unsupported";
  std::string last_relocalization_message;
  double relocalization_quality = -1.0;
  std::optional<Pose3d> relocalization_map_body;
  std::string relocalization_refine_backend;
  int relocalization_refine_iterations = -1;
  int relocalization_refine_inliers = -1;
  int relocalization_min_inliers = 0;
  int relocalization_min_evaluated_points = 0;
  int relocalization_refine_input_points = 0;
  int relocalization_refine_evaluated_points = 0;
  double relocalization_refine_support_ratio = -1.0;
  double relocalization_refine_overlap_inlier_ratio = -1.0;
  bool relocalization_refine_converged = false;
  double relocalization_refine_pos_cov_trace = -1.0;

  double localization_quality = 0.0;
  GnssFusionHealth gnss_fusion_health;
  std::string scene_mode = "unknown";

  double scan_start_s = 0.0;
  double scan_end_s = 0.0;
  double last_imu_s = 0.0;
  int imu_batch = 0;
  int sync_wait_count = 0;
  int imu_rollback_count = 0;
  int lidar_rollback_count = 0;

  int imu_buffer = 0;
  int lidar_buffer = 0;
  int dropped_lidar_frames = 0;
  int dropped_imu_frames = 0;

  bool odom_prior_enabled = false;
  bool odom_prior_active = false;
  double odom_prior_age_s = -1.0;
  double odom_prior_error_xy_m = -1.0;
  int odom_prior_map_points = 0;

  std::string map_optimization_status = "not_run";
  std::string map_optimization_backend;
  std::string map_optimization_refine_backend;
  bool map_optimization_enabled = false;
  bool map_optimization_loop_closure_enabled = false;
  bool map_optimization_loop_closure_applied = false;
  bool map_optimization_refine_enabled = false;
  bool map_optimization_refine_applied = false;
  bool map_optimization_hba_refine_enabled = false;
  bool map_optimization_hba_refine_applied = false;
  int map_optimization_patch_count = 0;
  int map_optimization_pose_count = 0;
  int map_optimization_optimized_pose_count = 0;
  int map_optimization_loop_count = 0;
  int map_optimization_raw_map_points = 0;
  int map_optimization_optimized_map_points = 0;
  double map_optimization_loop_error_m = -1.0;

  double fastlio_velocity_x = 0.0;
  double fastlio_velocity_y = 0.0;
  double fastlio_velocity_z = 0.0;
  bool fastlio_degeneracy_detected = false;
  int fastlio_degenerate_dof_count = 0;
  double fastlio_condition_number = 0.0;
  double fastlio_min_eigenvalue = 0.0;
  double fastlio_max_eigenvalue = 0.0;
  double fastlio_effective_ratio = 0.0;
  double fastlio_pos_cov_trace = 0.0;
  int fastlio_iter_num = 0;
  bool fastlio_converged = true;
  FastLioLidarUpdateDiagnostics fastlio_lidar_update;
};

class ISlamBackend {
 public:
  virtual ~ISlamBackend() = default;

  virtual Status configure(const SlamConfig&) = 0;
  virtual Status setMode(SlamMode mode, const std::string& map_path) = 0;

  virtual Status feedImu(const ImuSample&) = 0;
  virtual Status feedLidar(const LidarFrame&) = 0;
  virtual Status feedGnss(const GnssSample&) = 0;
  virtual Status feedVisualOdom(const OdomSample&) = 0;

  virtual Status setInitialPose(const Pose3d&) = 0;
  virtual Status relocalize(const std::optional<Pose3d>& guess) = 0;

  // Periodic saved-map tracking must not run ICP on the sensor/tick thread.
  // Backends that support it snapshot their inputs in startRelocalizeAsync(),
  // perform only the independent registration work off-thread, and commit a
  // completed result from pollRelocalizeAsync() on the owning runtime thread.
  virtual Status startRelocalizeAsync(const std::optional<Pose3d>&) {
    return Status::Error("async_relocalization_unsupported");
  }
  virtual std::optional<Status> pollRelocalizeAsync() { return std::nullopt; }
  virtual bool relocalizeAsyncInFlight() const { return false; }

  virtual Status tick() = 0;
  virtual Status saveMap(const std::string& pcd_path) = 0;
  virtual Status loadMap(const std::string& pcd_path) = 0;
  virtual SlamOutputs outputs() const = 0;
  virtual Status reset() = 0;
};

std::string toString(SlamState state);
SlamMode modeFromString(const std::string& value);
std::string toString(SlamMode mode);

std::uint64_t newSourceEpoch() noexcept;
std::unique_ptr<ISlamBackend> makeContractBackend(std::string backend_name);
std::unique_ptr<ISlamBackend> makeFastLioBackend();
std::unique_ptr<ISlamBackend> makePointLioBackend();

}  // namespace lingtu::slam
