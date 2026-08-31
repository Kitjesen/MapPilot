#include "slam.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lingtu::slam {
namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

bool finite(double value) {
  return std::isfinite(value);
}

bool validPose(const Pose3d& pose) {
  return finite(pose.x) && finite(pose.y) && finite(pose.z) &&
         finite(pose.qx) && finite(pose.qy) && finite(pose.qz) &&
         finite(pose.qw);
}

std::filesystem::path mapPcdPath(const std::string& path) {
  std::filesystem::path p(path);
  if (p.extension() == ".pcd") {
    return p;
  }
  return p / "map.pcd";
}

Status writePcd(const std::filesystem::path& path, const Cloud& cloud) {
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    return Status::Error("create_map_dir_failed: " + ec.message());
  }

  std::ofstream out(path);
  if (!out) {
    return Status::Error("open_map_pcd_failed");
  }

  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH " << cloud.points.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << cloud.points.size() << "\n";
  out << "DATA ascii\n";
  out << std::setprecision(9);
  for (const auto& point : cloud.points) {
    out << point.x << ' ' << point.y << ' ' << point.z << ' '
        << point.intensity << '\n';
  }
  return Status::Ok("map_pcd_written");
}

Status writePcdBinary(const std::filesystem::path& path, const Cloud& cloud) {
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    return Status::Error("create_patch_dir_failed: " + ec.message());
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return Status::Error("open_patch_pcd_failed");
  }

  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH " << cloud.points.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << cloud.points.size() << "\n";
  out << "DATA binary\n";
  for (const auto& point : cloud.points) {
    const float values[4] = {point.x, point.y, point.z, point.intensity};
    out.write(reinterpret_cast<const char*>(values), sizeof(values));
  }
  return Status::Ok("patch_pcd_written");
}

Status writeTrajectory(
    const std::filesystem::path& map_dir,
    const std::vector<OdomSample>& pose_history) {
  if (pose_history.empty()) {
    return Status::Ok("no_pose_history");
  }

  std::ofstream out(map_dir / "trajectory.txt");
  if (!out) {
    return Status::Error("open_trajectory_txt_failed");
  }
  out << std::setprecision(12);
  for (const auto& sample : pose_history) {
    const auto& p = sample.odom_body;
    out << sample.stamp_s << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' '
        << p.qx << ' ' << p.qy << ' ' << p.qz << ' ' << p.qw << '\n';
  }
  return Status::Ok("trajectory_txt_written");
}

Status writePatchIndex(
    const std::filesystem::path& map_dir,
    const std::string& patch_name,
    const Pose3d& pose) {
  std::ofstream out(map_dir / "poses.txt");
  if (!out) {
    return Status::Error("open_poses_txt_failed");
  }
  out << std::setprecision(12)
      << patch_name << ' ' << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
      << pose.qw << ' ' << pose.qx << ' ' << pose.qy << ' ' << pose.qz << '\n';
  return Status::Ok("poses_txt_written");
}

Status writeLatestPatch(
    const std::filesystem::path& map_dir,
    const std::optional<Cloud>& registered_cloud_body,
    const std::optional<Pose3d>& odometry_odom_body) {
  if (!registered_cloud_body.has_value() || registered_cloud_body->points.empty()) {
    return Status::Ok("no_registered_cloud_patch");
  }
  if (!odometry_odom_body.has_value()) {
    return Status::Ok("no_odometry_for_patch");
  }
  const std::string patch_name = "latest_scan.pcd";
  const Status patch_status =
      writePcdBinary(map_dir / "patches" / patch_name, *registered_cloud_body);
  if (!patch_status.ok) {
    return patch_status;
  }
  return writePatchIndex(map_dir, patch_name, *odometry_odom_body);
}

class ContractBackend final : public ISlamBackend {
 public:
  explicit ContractBackend(std::string backend_name)
      : backend_name_(std::move(backend_name)),
        source_epoch_(newSourceEpoch()) {}

  Status configure(const SlamConfig& config) override {
    config_ = config;
    if (!backend_name_.empty()) {
      config_.backend = backend_name_;
    }
    alive_ = true;
    state_ = SlamState::Initializing;
    reason_ = "configured";
    return Status::Ok(reason_);
  }

  Status setMode(SlamMode mode, const std::string& map_path) override {
    mode_ = mode;
    map_path_ = map_path;
    state_ =
        mode_ == SlamMode::Mapping ? SlamState::Mapping : SlamState::Localizing;
    reason_ = "mode_set";
    return Status::Ok(reason_);
  }

  Status feedImu(const ImuSample& sample) override {
    if (!finite(sample.stamp_s) || !finite(sample.gx) || !finite(sample.gy) ||
        !finite(sample.gz) || !finite(sample.ax) || !finite(sample.ay) ||
        !finite(sample.az)) {
      ++dropped_imu_frames_;
      return Status::Error("invalid_imu_sample");
    }
    ++imu_buffer_;
    last_stamp_s_ = sample.stamp_s;
    if (last_imu_s_ > 0.0 && sample.stamp_s < last_imu_s_) {
      ++imu_rollback_count_;
      reason_ = "imu_time_rollback";
    }
    last_imu_s_ = sample.stamp_s;
    return Status::Ok("imu_accepted");
  }

  Status feedLidar(const LidarFrame& frame) override {
    if (!finite(frame.stamp_s)) {
      ++dropped_lidar_frames_;
      return Status::Error("invalid_lidar_frame");
    }
    if (last_lidar_s_ > 0.0 && frame.stamp_s < last_lidar_s_) {
      ++lidar_rollback_count_;
      reason_ = "lidar_time_rollback";
    }
    last_lidar_s_ = frame.stamp_s;
    scan_start_s_ = frame.stamp_s;
    std::int64_t max_offset_ns = 0;
    for (const auto& point : frame.points) {
      max_offset_ns = std::max(max_offset_ns, point.offset_time_ns);
    }
    scan_end_s_ = frame.stamp_s + static_cast<double>(max_offset_ns) * 1e-9;
    imu_batch_ = imu_buffer_;
    if (last_imu_s_ > 0.0 && last_imu_s_ < scan_end_s_) {
      ++sync_wait_count_;
    }
    ++lidar_buffer_;
    last_stamp_s_ = frame.stamp_s;
    Cloud registered;
    registered.stamp_s = frame.stamp_s;
    registered.frame_id = config_.body_frame;
    registered.points = frame.points;
    registered_cloud_body_ = registered;
    ++observation_sequence_;

    Cloud map_cloud;
    map_cloud.stamp_s = frame.stamp_s;
    map_cloud.frame_id = config_.map_frame;
    map_cloud.points = frame.points;
    map_cloud_map_ = map_cloud;
    return Status::Ok("lidar_accepted");
  }

  Status feedGnss(const GnssSample& sample) override {
    gnss_health_.enabled = true;
    gnss_health_.last_fix_type = sample.fix_type;
    gnss_health_.last_gnss_age_s = std::max(0.0, nowSeconds() - sample.stamp_s);
    gnss_health_.alignment_locked =
        sample.fix_type == "RTK_FIXED" || sample.fix_type == "RTK_FLOAT";
    return Status::Ok("gnss_accepted");
  }

  Status feedVisualOdom(const OdomSample& sample) override {
    if (!finite(sample.stamp_s) || !validPose(sample.odom_body)) {
      return Status::Error("invalid_visual_odom");
    }
    odometry_odom_body_ = sample.odom_body;
    state_estimation_at_scan_ = sample.odom_body;
    pose_history_.push_back(sample);
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }
    return Status::Ok("visual_odom_accepted");
  }

  Status setInitialPose(const Pose3d& pose) override {
    if (!validPose(pose)) {
      return Status::Error("invalid_initial_pose");
    }
    odometry_odom_body_ = pose;
    state_estimation_at_scan_ = pose;
    pose_history_.push_back(OdomSample{nowSeconds(), pose});
    reason_ = "initial_pose_set";
    return Status::Ok(reason_);
  }

  Status relocalize(const std::optional<Pose3d>& guess) override {
    if (guess.has_value()) {
      const Status status = setInitialPose(*guess);
      if (!status.ok) {
        return status;
      }
    }
    if (!map_loaded_ && mode_ == SlamMode::Localization) {
      state_ = SlamState::Lost;
      reason_ = "map_not_loaded";
      return Status::Error(reason_);
    }
    state_ = SlamState::Tracking;
    reason_ = "relocalized";
    return Status::Ok(reason_);
  }

  Status tick() override {
    if (!alive_) {
      state_ = SlamState::Failed;
      reason_ = "not_alive";
      return Status::Error(reason_);
    }
    if (odometry_odom_body_.has_value()) {
      state_ = SlamState::Tracking;
      confidence_ = 0.8;
      reason_ = "tracking";
    } else {
      confidence_ = 0.0;
      reason_ = "waiting_for_sensor_data";
    }
    return Status::Ok(reason_);
  }

  Status saveMap(const std::string& pcd_path) override {
    const auto pcd = mapPcdPath(pcd_path);
    Cloud cloud;
    cloud.stamp_s = nowSeconds();
    cloud.frame_id = config_.map_frame;
    if (map_cloud_map_.has_value()) {
      cloud = *map_cloud_map_;
    }

    Status status = writePcd(pcd, cloud);
    if (!status.ok) {
      return status;
    }

    status = writeTrajectory(pcd.parent_path(), pose_history_);
    if (!status.ok) {
      return status;
    }

    std::error_code ec;
    std::filesystem::create_directories(pcd.parent_path() / "patches", ec);
    if (ec) {
      return Status::Error("create_patches_dir_failed: " + ec.message());
    }

    status = writeLatestPatch(pcd.parent_path(), registered_cloud_body_, odometry_odom_body_);
    if (!status.ok) {
      return status;
    }

    saved_map_cloud_map_ = cloud;
    saved_map_points_ = static_cast<int>(cloud.points.size());
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    reason_ = "map_saved";
    return Status::Ok(reason_);
  }

  Status loadMap(const std::string& pcd_path) override {
    const auto pcd = mapPcdPath(pcd_path);
    if (!std::filesystem::exists(pcd)) {
      return Status::Error("map_pcd_missing");
    }
    Cloud cloud;
    cloud.stamp_s = nowSeconds();
    cloud.frame_id = config_.map_frame;
    saved_map_cloud_map_ = cloud;
    saved_map_points_ = static_cast<int>(cloud.points.size());
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    reason_ = "map_loaded";
    return Status::Ok(reason_);
  }

  SlamOutputs outputs() const override {
    SlamOutputs out;
    out.state = state_;
    out.stamp_s = last_stamp_s_ > 0.0 ? last_stamp_s_ : nowSeconds();
    out.confidence = confidence_;
    out.reason = reason_;
    out.odometry_odom_body = odometry_odom_body_;
    out.state_estimation_at_scan = state_estimation_at_scan_;
    out.registered_cloud_body = registered_cloud_body_;
    out.map_cloud_map = map_cloud_map_;
    out.saved_map_cloud_map = saved_map_cloud_map_;
    out.map_odom_tf = Transform3d{config_.map_frame, config_.odom_frame, Pose3d{}};
    out.observation_sequence = observation_sequence_;
    out.source_epoch = source_epoch_;
    out.saved_map_points = saved_map_points_;
    out.alive = alive_;
    out.map_loaded = map_loaded_;
    out.map_frame_jump = map_frame_jump_;
    out.relocalization_supported = true;
    out.saved_map_relocalization_supported = true;
    out.relocalization_state = map_loaded_ ? "idle" : "map_not_loaded";
    out.last_relocalization_message = reason_;
    out.relocalization_quality = confidence_;
    out.localization_quality = confidence_;
    out.gnss_fusion_health = gnss_health_;
    out.scene_mode = scene_mode_;
    out.scan_start_s = scan_start_s_;
    out.scan_end_s = scan_end_s_;
    out.last_imu_s = last_imu_s_;
    out.imu_batch = imu_batch_;
    out.sync_wait_count = sync_wait_count_;
    out.imu_rollback_count = imu_rollback_count_;
    out.lidar_rollback_count = lidar_rollback_count_;
    out.imu_buffer = imu_buffer_;
    out.lidar_buffer = lidar_buffer_;
    out.dropped_lidar_frames = dropped_lidar_frames_;
    out.dropped_imu_frames = dropped_imu_frames_;
    return out;
  }

  Status reset() override {
    *this = ContractBackend(backend_name_);
    return Status::Ok("reset");
  }

 private:
  std::string backend_name_;
  SlamConfig config_;
  SlamMode mode_ = SlamMode::Mapping;
  SlamState state_ = SlamState::Unconfigured;
  std::string reason_ = "unconfigured";
  std::string scene_mode_ = "unknown";
  std::string map_path_;
  std::string last_map_path_;
  bool alive_ = false;
  bool map_loaded_ = false;
  bool map_frame_jump_ = false;
  double confidence_ = 0.0;
  double last_stamp_s_ = 0.0;
  double last_imu_s_ = 0.0;
  double last_lidar_s_ = 0.0;
  double scan_start_s_ = 0.0;
  double scan_end_s_ = 0.0;
  int imu_batch_ = 0;
  int sync_wait_count_ = 0;
  int imu_rollback_count_ = 0;
  int lidar_rollback_count_ = 0;
  int imu_buffer_ = 0;
  int lidar_buffer_ = 0;
  std::uint64_t observation_sequence_ = 0U;
  std::uint64_t source_epoch_ = 0U;
  int dropped_lidar_frames_ = 0;
  int dropped_imu_frames_ = 0;
  std::optional<Pose3d> odometry_odom_body_;
  std::optional<Pose3d> state_estimation_at_scan_;
  std::optional<Cloud> registered_cloud_body_;
  std::optional<Cloud> map_cloud_map_;
  std::optional<Cloud> saved_map_cloud_map_;
  int saved_map_points_ = 0;
  GnssFusionHealth gnss_health_;
  std::vector<OdomSample> pose_history_;
};

}  // namespace

std::uint64_t newSourceEpoch() noexcept {
  constexpr std::uint64_t kResetCounterBits = 16U;
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  const std::uint64_t wall_epoch =
      static_cast<std::uint64_t>(std::max<std::int64_t>(1, now_ms))
      << kResetCounterBits;
  static std::atomic<std::uint64_t> last_epoch{0U};
  std::uint64_t observed = last_epoch.load(std::memory_order_relaxed);
  for (;;) {
    const std::uint64_t candidate = std::max(wall_epoch, observed + 1U);
    if (last_epoch.compare_exchange_weak(
            observed,
            candidate,
            std::memory_order_relaxed,
            std::memory_order_relaxed)) {
      return candidate;
    }
  }
}

std::string toString(SlamState state) {
  switch (state) {
    case SlamState::Unconfigured:
      return "UNCONFIGURED";
    case SlamState::Initializing:
      return "INITIALIZING";
    case SlamState::Mapping:
      return "MAPPING";
    case SlamState::Localizing:
      return "LOCALIZING";
    case SlamState::Tracking:
      return "TRACKING";
    case SlamState::Degraded:
      return "DEGRADED";
    case SlamState::Lost:
      return "LOST";
    case SlamState::Failed:
      return "FAILED";
  }
  return "FAILED";
}

SlamMode modeFromString(const std::string& value) {
  if (value == "mapping") {
    return SlamMode::Mapping;
  }
  if (value == "localization") {
    return SlamMode::Localization;
  }
  throw std::invalid_argument("unsupported SLAM mode: " + value);
}

std::string toString(SlamMode mode) {
  return mode == SlamMode::Localization ? "localization" : "mapping";
}

std::unique_ptr<ISlamBackend> makeContractBackend(std::string backend_name) {
  return std::make_unique<ContractBackend>(std::move(backend_name));
}

}  // namespace lingtu::slam
