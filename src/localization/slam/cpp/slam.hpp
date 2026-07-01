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

  bool alive = false;
  bool map_loaded = false;
  bool map_frame_jump = false;

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

  virtual Status tick() = 0;
  virtual Status saveMap(const std::string& pcd_path) = 0;
  virtual Status loadMap(const std::string& pcd_path) = 0;
  virtual SlamOutputs outputs() const = 0;
  virtual Status reset() = 0;
};

std::string toString(SlamState state);
SlamMode modeFromString(const std::string& value);
std::string toString(SlamMode mode);

std::unique_ptr<ISlamBackend> makeContractBackend(std::string backend_name);
std::unique_ptr<ISlamBackend> makeFastLioBackend();
std::unique_ptr<ISlamBackend> makePointLioBackend();

}  // namespace lingtu::slam
