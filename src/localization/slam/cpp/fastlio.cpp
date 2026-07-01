#include "slam.hpp"

#if defined(LINGTU_HAS_FASTLIO2_BACKEND) && LINGTU_HAS_FASTLIO2_BACKEND

#include "map_builder/map_builder.h"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <utility>

namespace lingtu::slam {
namespace {

struct RuntimeConfig {
  double acc_scale = 1.0;
  double time_diff_lidar_to_imu = 0.0;
  double max_imu_gap_s = 0.25;
  std::size_t max_imu_buffer = 4000;
  std::size_t max_lidar_buffer = 64;
};

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

template <typename T>
void readIfPresent(const YAML::Node& node, const char* key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

Status loadYamlConfig(
    const std::string& path,
    SlamConfig& slam_config,
    Config& builder_config,
    RuntimeConfig& runtime_config) {
  if (path.empty()) {
    return Status::Ok("default_fastlio2_config");
  }
  if (!std::filesystem::exists(path)) {
    return Status::Error("fastlio2_config_missing: " + path);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  } catch (const YAML::Exception& exc) {
    return Status::Error(std::string("fastlio2_config_parse_failed: ") + exc.what());
  }
  if (!config) {
    return Status::Error("fastlio2_config_empty: " + path);
  }

  readIfPresent(config, "body_frame", slam_config.body_frame);
  readIfPresent(config, "world_frame", slam_config.odom_frame);
  readIfPresent(config, "acc_scale", runtime_config.acc_scale);
  readIfPresent(config, "time_diff_lidar_to_imu", runtime_config.time_diff_lidar_to_imu);
  readIfPresent(config, "max_imu_gap_s", runtime_config.max_imu_gap_s);

  readIfPresent(config, "lidar_filter_num", builder_config.lidar_filter_num);
  readIfPresent(config, "lidar_min_range", builder_config.lidar_min_range);
  readIfPresent(config, "lidar_max_range", builder_config.lidar_max_range);
  readIfPresent(config, "scan_resolution", builder_config.scan_resolution);
  readIfPresent(config, "map_resolution", builder_config.map_resolution);
  readIfPresent(config, "cube_len", builder_config.cube_len);
  readIfPresent(config, "det_range", builder_config.det_range);
  readIfPresent(config, "move_thresh", builder_config.move_thresh);
  readIfPresent(config, "max_map_points", builder_config.max_map_points);
  readIfPresent(config, "stationary_thresh", builder_config.stationary_thresh);
  readIfPresent(config, "na", builder_config.na);
  readIfPresent(config, "ng", builder_config.ng);
  readIfPresent(config, "nba", builder_config.nba);
  readIfPresent(config, "nbg", builder_config.nbg);
  readIfPresent(config, "imu_init_num", builder_config.imu_init_num);
  readIfPresent(config, "near_search_num", builder_config.near_search_num);
  readIfPresent(config, "ieskf_max_iter", builder_config.ieskf_max_iter);
  readIfPresent(config, "degeneracy_max_update_dof", builder_config.degeneracy_max_update_dof);
  readIfPresent(config, "degeneracy_max_condition", builder_config.degeneracy_max_condition);
  readIfPresent(config, "max_update_translation_m", builder_config.max_update_translation_m);
  readIfPresent(config, "max_update_rotation_rad", builder_config.max_update_rotation_rad);
  readIfPresent(config, "reject_nonconverged_update", builder_config.reject_nonconverged_update);
  readIfPresent(config, "reject_degenerate_nonconverged_update", builder_config.reject_degenerate_nonconverged_update);
  readIfPresent(config, "gravity_align", builder_config.gravity_align);
  readIfPresent(config, "esti_il", builder_config.esti_il);
  readIfPresent(config, "lidar_cov_inv", builder_config.lidar_cov_inv);
  readIfPresent(config, "imu_static_acc_thresh", builder_config.imu_static_acc_thresh);
  readIfPresent(config, "imu_static_gyro_thresh", builder_config.imu_static_gyro_thresh);
  readIfPresent(config, "zupt_min_static_frames", builder_config.zupt_min_static_frames);
  readIfPresent(config, "zupt_sigma_v", builder_config.zupt_sigma_v);
  readIfPresent(config, "zupt_sigma_pos", builder_config.zupt_sigma_pos);
  readIfPresent(config, "vertical_velocity_constraint_enabled", builder_config.vertical_velocity_constraint_enabled);
  readIfPresent(config, "vertical_velocity_sigma_v", builder_config.vertical_velocity_sigma_v);

  if (config["t_il"]) {
    const auto t = config["t_il"].as<std::vector<double>>();
    if (t.size() != 3) {
      return Status::Error("fastlio2_t_il_must_have_3_values");
    }
    builder_config.t_il << t[0], t[1], t[2];
  }
  if (config["r_il"]) {
    const auto r = config["r_il"].as<std::vector<double>>();
    if (r.size() != 9) {
      return Status::Error("fastlio2_r_il_must_have_9_values");
    }
    builder_config.r_il << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }

  return Status::Ok("fastlio2_config_loaded");
}

bool validPoint(const PointXYZIT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

CloudType::Ptr toPclCloud(const LidarFrame& frame, const Config& config) {
  auto cloud = std::make_shared<CloudType>();
  const int filter_num = std::max(1, config.lidar_filter_num);
  cloud->reserve(frame.points.size() / static_cast<std::size_t>(filter_num) + 1);
  const double min2 = config.lidar_min_range * config.lidar_min_range;
  const double max2 = config.lidar_max_range * config.lidar_max_range;

  for (std::size_t i = 0; i < frame.points.size(); i += static_cast<std::size_t>(filter_num)) {
    const auto& src = frame.points[i];
    if (!validPoint(src)) {
      continue;
    }
    const double dist2 =
        static_cast<double>(src.x) * src.x +
        static_cast<double>(src.y) * src.y +
        static_cast<double>(src.z) * src.z;
    if (dist2 < min2 || dist2 > max2) {
      continue;
    }
    if (src.line >= 4 || !(((src.tag & 0x30U) == 0x10U) || ((src.tag & 0x30U) == 0x00U))) {
      continue;
    }

    PointType point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.normal_x = 0.0F;
    point.normal_y = 0.0F;
    point.normal_z = 0.0F;
    point.intensity = src.intensity;
    point.curvature = static_cast<float>(static_cast<double>(src.offset_time_ns) * 1e-6);
    cloud->push_back(point);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

Cloud toContractCloud(const CloudType::Ptr& cloud, double stamp_s, std::string frame_id) {
  Cloud out;
  out.stamp_s = stamp_s;
  out.frame_id = std::move(frame_id);
  if (!cloud) {
    return out;
  }
  out.points.reserve(cloud->points.size());
  for (const auto& src : cloud->points) {
    PointXYZIT point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.intensity = src.intensity;
    point.offset_time_ns = static_cast<std::int64_t>(std::llround(src.curvature * 1e6));
    out.points.push_back(point);
  }
  return out;
}

Pose3d poseFromState(const State& state) {
  Eigen::Quaterniond q(state.r_wi);
  q.normalize();
  Pose3d pose;
  pose.x = state.t_wi.x();
  pose.y = state.t_wi.y();
  pose.z = state.t_wi.z();
  pose.qx = q.x();
  pose.qy = q.y();
  pose.qz = q.z();
  pose.qw = q.w();
  return pose;
}

void applyPose(State& state, const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  state.t_wi = V3D(pose.x, pose.y, pose.z);
  state.r_wi = q.toRotationMatrix();
}

Status writePoses(
    const std::filesystem::path& map_dir,
    const std::vector<OdomSample>& pose_history) {
  std::ofstream out(map_dir / "poses.txt");
  if (!out) {
    return Status::Error("open_poses_txt_failed");
  }
  for (const auto& sample : pose_history) {
    const auto& p = sample.odom_body;
    out << sample.stamp_s << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' '
        << p.qx << ' ' << p.qy << ' ' << p.qz << ' ' << p.qw << '\n';
  }
  return Status::Ok("poses_txt_written");
}

class FastLioBackend final : public ISlamBackend {
 public:
  Status configure(const SlamConfig& config) override {
    config_ = config;
    config_.backend = "fastlio2";
    builder_config_ = Config{};
    runtime_config_ = RuntimeConfig{};
    const Status status = loadYamlConfig(
        config_.config_path, config_, builder_config_, runtime_config_);
    if (!status.ok) {
      state_ = SlamState::Failed;
      reason_ = status.message;
      return status;
    }
    resetCore();
    alive_ = true;
    state_ = SlamState::Initializing;
    reason_ = status.message;
    return Status::Ok(reason_);
  }

  Status setMode(SlamMode mode, const std::string& map_path) override {
    mode_ = mode;
    map_path_ = map_path;
    if (mode_ == SlamMode::Localization && !map_path.empty()) {
      const Status status = loadMap(map_path);
      if (!status.ok) {
        state_ = SlamState::Lost;
        reason_ = status.message;
        return status;
      }
    }
    state_ = mode_ == SlamMode::Mapping ? SlamState::Mapping : SlamState::Localizing;
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
    double timestamp = sample.stamp_s - runtime_config_.time_diff_lidar_to_imu;
    if (timestamp < last_imu_time_) {
      dropped_imu_frames_ += static_cast<int>(imu_buffer_.size());
      imu_buffer_.clear();
      ++imu_rollback_count_;
      reason_ = "imu_time_rollback";
    }
    V3D acc(sample.ax * runtime_config_.acc_scale,
            sample.ay * runtime_config_.acc_scale,
            sample.az * runtime_config_.acc_scale);
    V3D gyro(sample.gx, sample.gy, sample.gz);
    imu_buffer_.emplace_back(acc, gyro, timestamp);
    last_imu_time_ = timestamp;
    while (imu_buffer_.size() > runtime_config_.max_imu_buffer) {
      imu_buffer_.pop_front();
      ++dropped_imu_frames_;
    }
    return Status::Ok("imu_accepted");
  }

  Status feedLidar(const LidarFrame& frame) override {
    if (!finite(frame.stamp_s)) {
      ++dropped_lidar_frames_;
      return Status::Error("invalid_lidar_frame");
    }
    if (frame.points.empty()) {
      ++dropped_lidar_frames_;
      return Status::Error("empty_lidar_frame");
    }
    if (frame.stamp_s < last_lidar_time_) {
      dropped_lidar_frames_ += static_cast<int>(lidar_buffer_.size());
      lidar_buffer_.clear();
      lidar_pushed_ = false;
      ++lidar_rollback_count_;
      reason_ = "lidar_time_rollback";
    }
    lidar_buffer_.push_back(frame);
    last_lidar_time_ = frame.stamp_s;
    last_stamp_s_ = frame.stamp_s;
    while (lidar_buffer_.size() > runtime_config_.max_lidar_buffer) {
      lidar_buffer_.pop_front();
      ++dropped_lidar_frames_;
    }
    return Status::Ok("lidar_accepted");
  }

  Status feedGnss(const GnssSample& sample) override {
    gnss_health_.enabled = true;
    gnss_health_.last_fix_type = sample.fix_type;
    gnss_health_.alignment_locked =
        sample.fix_type == "RTK_FIXED" || sample.fix_type == "RTK_FLOAT";
    gnss_health_.last_gnss_age_s = 0.0;
    return Status::Ok("gnss_accepted");
  }

  Status feedVisualOdom(const OdomSample& sample) override {
    if (!finite(sample.stamp_s) || !validPose(sample.odom_body)) {
      return Status::Error("invalid_visual_odom");
    }
    pose_history_.push_back(sample);
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }
    return Status::Ok("visual_odom_accepted");
  }

  Status setInitialPose(const Pose3d& pose) override {
    if (!validPose(pose) || !kf_) {
      return Status::Error("invalid_initial_pose");
    }
    applyPose(kf_->x(), pose);
    odometry_odom_body_ = pose;
    state_estimation_at_scan_ = pose;
    pose_history_.push_back(OdomSample{last_stamp_s_, pose});
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
    if (mode_ == SlamMode::Localization) {
      state_ = SlamState::Lost;
      reason_ = "fastlio2_relocalization_not_implemented";
      return Status::Error(reason_);
    }
    state_ = SlamState::Tracking;
    reason_ = "tracking";
    return Status::Ok(reason_);
  }

  Status tick() override {
    if (!alive_ || !builder_ || !kf_) {
      state_ = SlamState::Failed;
      reason_ = "not_configured";
      return Status::Error(reason_);
    }
    if (!prepareFastLioPackage()) {
      updateWaitingReason();
      return Status::Ok(reason_);
    }

    // Fast-LIO2 core: IMU init/propagation, point undistortion, and map update.
    builder_->process(package_);
    last_stamp_s_ = package_.cloud_end_time;
    if (builder_->status() != BuilderStatus::MAPPING) {
      updateBuilderState();
      return Status::Ok(reason_);
    }

    odometry_odom_body_ = poseFromState(kf_->x());
    state_estimation_at_scan_ = odometry_odom_body_;
    pose_history_.push_back(OdomSample{package_.cloud_end_time, *odometry_odom_body_});
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }

    auto body_cloud = LidarProcessor::transformCloud(
        package_.cloud, kf_->x().r_il, kf_->x().t_il);
    registered_cloud_body_ =
        toContractCloud(body_cloud, package_.cloud_end_time, config_.body_frame);

    auto world_cloud = LidarProcessor::transformCloud(
        package_.cloud, builder_->lidar_processor()->r_wl(), builder_->lidar_processor()->t_wl());
    map_cloud_map_ =
        toContractCloud(world_cloud, package_.cloud_end_time, config_.map_frame);

    state_ = SlamState::Tracking;
    confidence_ = std::max(0.0, std::min(1.0, kf_->degeneracy().effective_ratio));
    reason_ = "tracking";
    return Status::Ok(reason_);
  }

  Status saveMap(const std::string& pcd_path) override {
    if (!builder_) {
      return Status::Error("not_configured");
    }
    if (builder_->status() != BuilderStatus::MAPPING) {
      return Status::Error("map_not_ready");
    }
    const auto pcd = mapPcdPath(pcd_path);
    std::error_code ec;
    std::filesystem::create_directories(pcd.parent_path(), ec);
    if (ec) {
      return Status::Error("create_map_dir_failed: " + ec.message());
    }
    builder_->saveMap(pcd.string());
    if (!std::filesystem::exists(pcd)) {
      return Status::Error("map_pcd_write_failed");
    }
    const Status pose_status = writePoses(pcd.parent_path(), pose_history_);
    if (!pose_status.ok) {
      return pose_status;
    }
    std::filesystem::create_directories(pcd.parent_path() / "patches", ec);
    if (ec) {
      return Status::Error("create_patches_dir_failed: " + ec.message());
    }
    saved_map_cloud_map_ = map_cloud_map_;
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
    CloudType cloud;
    if (pcl::io::loadPCDFile<PointType>(pcd.string(), cloud) < 0) {
      return Status::Error("map_pcd_load_failed");
    }
    saved_map_cloud_map_ = toContractCloud(
        std::make_shared<CloudType>(cloud), last_stamp_s_, config_.map_frame);
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    reason_ = "map_loaded";
    return Status::Ok(reason_);
  }

  SlamOutputs outputs() const override {
    SlamOutputs out;
    out.state = state_;
    out.stamp_s = last_stamp_s_;
    out.confidence = confidence_;
    out.reason = reason_;
    out.odometry_odom_body = odometry_odom_body_;
    out.state_estimation_at_scan = state_estimation_at_scan_;
    out.registered_cloud_body = registered_cloud_body_;
    out.map_cloud_map = map_cloud_map_;
    out.saved_map_cloud_map = saved_map_cloud_map_;
    out.map_odom_tf = Transform3d{config_.map_frame, config_.odom_frame, Pose3d{}};
    out.alive = alive_;
    out.map_loaded = map_loaded_;
    out.map_frame_jump = map_frame_jump_;
    out.localization_quality = confidence_;
    out.gnss_fusion_health = gnss_health_;
    out.scene_mode = scene_mode_;
    out.scan_start_s = scan_start_s_;
    out.scan_end_s = scan_end_s_;
    out.last_imu_s = last_imu_time_;
    out.imu_batch = imu_batch_;
    out.sync_wait_count = sync_wait_count_;
    out.imu_rollback_count = imu_rollback_count_;
    out.lidar_rollback_count = lidar_rollback_count_;
    out.imu_buffer = static_cast<int>(imu_buffer_.size());
    out.lidar_buffer = static_cast<int>(lidar_buffer_.size());
    out.dropped_lidar_frames = dropped_lidar_frames_;
    out.dropped_imu_frames = dropped_imu_frames_;
    return out;
  }

  Status reset() override {
    resetCore();
    alive_ = true;
    map_loaded_ = false;
    map_frame_jump_ = false;
    confidence_ = 0.0;
    reason_ = "reset";
    state_ = SlamState::Initializing;
    return Status::Ok(reason_);
  }

 private:
  void resetCore() {
    kf_ = std::make_shared<IESKF>();
    builder_ = std::make_unique<MapBuilder>(builder_config_, kf_);
    imu_buffer_.clear();
    lidar_buffer_.clear();
    lidar_pushed_ = false;
    package_ = SyncPackage{};
    last_lidar_time_ = -1.0;
    last_imu_time_ = -1.0;
    last_stamp_s_ = 0.0;
    scan_start_s_ = 0.0;
    scan_end_s_ = 0.0;
    imu_batch_ = 0;
    sync_wait_count_ = 0;
    imu_rollback_count_ = 0;
    lidar_rollback_count_ = 0;
    dropped_lidar_frames_ = 0;
    dropped_imu_frames_ = 0;
    odometry_odom_body_.reset();
    state_estimation_at_scan_.reset();
    registered_cloud_body_.reset();
    map_cloud_map_.reset();
    saved_map_cloud_map_.reset();
    pose_history_.clear();
  }

  bool prepareFastLioPackage() {
    if (imu_buffer_.empty() || lidar_buffer_.empty()) {
      return false;
    }
    if (!lidar_pushed_) {
      package_.cloud = toPclCloud(lidar_buffer_.front(), builder_config_);
      if (!package_.cloud || package_.cloud->points.empty()) {
        lidar_buffer_.pop_front();
        ++dropped_lidar_frames_;
        return false;
      }
      std::sort(
          package_.cloud->points.begin(),
          package_.cloud->points.end(),
          [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
      package_.cloud_start_time = lidar_buffer_.front().stamp_s;
      package_.cloud_end_time =
          package_.cloud_start_time + package_.cloud->points.back().curvature / 1000.0;
      scan_start_s_ = package_.cloud_start_time;
      scan_end_s_ = package_.cloud_end_time;
      lidar_pushed_ = true;
    }
    if (last_imu_time_ < package_.cloud_end_time) {
      ++sync_wait_count_;
      if (runtime_config_.max_imu_gap_s > 0.0 && last_imu_time_ >= 0.0 &&
          package_.cloud_end_time - last_imu_time_ > runtime_config_.max_imu_gap_s) {
        lidar_buffer_.pop_front();
        lidar_pushed_ = false;
        package_ = SyncPackage{};
        ++dropped_lidar_frames_;
        reason_ = "imu_gap_drop";
      }
      return false;
    }

    Vec<IMUData>().swap(package_.imus);
    while (!imu_buffer_.empty() && imu_buffer_.front().time < package_.cloud_end_time) {
      package_.imus.emplace_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    imu_batch_ = static_cast<int>(package_.imus.size());
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
  }

  void updateWaitingReason() {
    if (imu_buffer_.empty()) {
      reason_ = "waiting_for_imu";
    } else if (lidar_buffer_.empty()) {
      reason_ = "waiting_for_lidar";
    } else {
      reason_ = "waiting_for_imu_to_cover_scan";
    }
    updateBuilderState();
  }

  void updateBuilderState() {
    if (!builder_) {
      state_ = SlamState::Failed;
      return;
    }
    if (builder_->status() == BuilderStatus::IMU_INIT) {
      state_ = SlamState::Initializing;
      confidence_ = 0.0;
      reason_ = reason_.empty() ? "imu_initializing" : reason_;
    } else if (builder_->status() == BuilderStatus::MAP_INIT) {
      state_ = SlamState::Mapping;
      confidence_ = 0.2;
      reason_ = "map_initializing";
    } else {
      state_ = SlamState::Tracking;
      confidence_ = std::max(0.0, std::min(1.0, kf_->degeneracy().effective_ratio));
    }
  }

  SlamConfig config_;
  Config builder_config_;
  RuntimeConfig runtime_config_;
  SlamMode mode_ = SlamMode::Mapping;
  SlamState state_ = SlamState::Unconfigured;
  std::string reason_ = "unconfigured";
  std::string scene_mode_ = "unknown";
  std::string map_path_;
  std::string last_map_path_;
  std::shared_ptr<IESKF> kf_;
  std::unique_ptr<MapBuilder> builder_;
  std::deque<IMUData> imu_buffer_;
  std::deque<LidarFrame> lidar_buffer_;
  bool lidar_pushed_ = false;
  SyncPackage package_;
  double last_lidar_time_ = -1.0;
  double last_imu_time_ = -1.0;
  double last_stamp_s_ = 0.0;
  double scan_start_s_ = 0.0;
  double scan_end_s_ = 0.0;
  bool alive_ = false;
  bool map_loaded_ = false;
  bool map_frame_jump_ = false;
  double confidence_ = 0.0;
  int imu_batch_ = 0;
  int sync_wait_count_ = 0;
  int imu_rollback_count_ = 0;
  int lidar_rollback_count_ = 0;
  int dropped_lidar_frames_ = 0;
  int dropped_imu_frames_ = 0;
  std::optional<Pose3d> odometry_odom_body_;
  std::optional<Pose3d> state_estimation_at_scan_;
  std::optional<Cloud> registered_cloud_body_;
  std::optional<Cloud> map_cloud_map_;
  std::optional<Cloud> saved_map_cloud_map_;
  GnssFusionHealth gnss_health_;
  std::vector<OdomSample> pose_history_;
};

}  // namespace

std::unique_ptr<ISlamBackend> makeFastLioBackend() {
  return std::make_unique<FastLioBackend>();
}

}  // namespace lingtu::slam

#else

namespace lingtu::slam {

std::unique_ptr<ISlamBackend> makeFastLioBackend() {
  return makeContractBackend("fastlio2");
}

}  // namespace lingtu::slam

#endif
