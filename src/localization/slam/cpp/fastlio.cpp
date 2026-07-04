#include "slam.hpp"

#if defined(LINGTU_HAS_FASTLIO2_BACKEND) && LINGTU_HAS_FASTLIO2_BACKEND

#include "map_builder/map_builder.h"
#include "native_relocalizer.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

namespace lingtu::slam {
namespace {

struct RuntimeConfig {
  double acc_scale = 1.0;
  double time_diff_lidar_to_imu = 0.0;
  double max_imu_gap_s = 0.25;
  double relocalization_max_fitness = 0.5;
  double relocalization_map_bounds_margin_m = 2.0;
  std::size_t max_imu_buffer = 4000;
  std::size_t max_lidar_buffer = 64;
};

struct PatchSnapshot {
  std::string name;
  double stamp_s = 0.0;
  Pose3d pose;
  Cloud cloud;
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
  readIfPresent(config, "relocalization_max_fitness", runtime_config.relocalization_max_fitness);
  readIfPresent(
      config,
      "relocalization_map_bounds_margin_m",
      runtime_config.relocalization_map_bounds_margin_m);

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

Pose3d composePoses(const Pose3d& lhs, const Pose3d& rhs) {
  Eigen::Quaterniond lq(lhs.qw, lhs.qx, lhs.qy, lhs.qz);
  Eigen::Quaterniond rq(rhs.qw, rhs.qx, rhs.qy, rhs.qz);
  if (!std::isfinite(lq.norm()) || lq.norm() <= 0.0) {
    lq = Eigen::Quaterniond::Identity();
  }
  if (!std::isfinite(rq.norm()) || rq.norm() <= 0.0) {
    rq = Eigen::Quaterniond::Identity();
  }
  lq.normalize();
  rq.normalize();
  const Eigen::Vector3d translated =
      Eigen::Vector3d(lhs.x, lhs.y, lhs.z) +
      lq * Eigen::Vector3d(rhs.x, rhs.y, rhs.z);
  const Eigen::Quaterniond q = (lq * rq).normalized();
  Pose3d out;
  out.x = translated.x();
  out.y = translated.y();
  out.z = translated.z();
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

void applyPose(State& state, const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  state.t_wi = V3D(pose.x, pose.y, pose.z);
  state.r_wi = q.toRotationMatrix();
}

Status writeTrajectory(
    const std::filesystem::path& map_dir,
    const std::vector<OdomSample>& pose_history) {
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

Status writeContractPcdBinary(const std::filesystem::path& path, const Cloud& cloud) {
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

Status writePatchIndex(
    const std::filesystem::path& map_dir,
    const std::vector<PatchSnapshot>& patches) {
  std::ofstream out(map_dir / "poses.txt");
  if (!out) {
    return Status::Error("open_poses_txt_failed");
  }
  out << std::setprecision(12);
  for (const auto& patch : patches) {
    const auto& pose = patch.pose;
    out << patch.name << ' ' << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
        << pose.qw << ' ' << pose.qx << ' ' << pose.qy << ' ' << pose.qz << '\n';
  }
  return Status::Ok("poses_txt_written");
}

Status writePatchBundle(
    const std::filesystem::path& map_dir,
    const std::vector<PatchSnapshot>& patches) {
  if (patches.empty()) {
    return Status::Ok("no_patches");
  }
  for (const auto& patch : patches) {
    if (patch.cloud.points.empty()) {
      continue;
    }
    const Status patch_status =
        writeContractPcdBinary(map_dir / "patches" / patch.name, patch.cloud);
    if (!patch_status.ok) {
      return patch_status;
    }
  }
  return writePatchIndex(map_dir, patches);
}

double planarDistance(const Pose3d& a, const Pose3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::string patchName(std::uint64_t sequence) {
  std::ostringstream out;
  out << "scan_" << std::setw(6) << std::setfill('0') << sequence << ".pcd";
  return out.str();
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
    if (mode_ == SlamMode::Mapping) {
      map_loaded_ = false;
      map_bounds_ready_ = false;
    }
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
    last_lidar_time_ = frame.stamp_s;
    last_stamp_s_ = frame.stamp_s;
    pushLidarFrame(frame);
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
    if (mode_ != SlamMode::Localization) {
      if (guess.has_value()) {
        const Status status = setInitialPose(*guess);
        if (!status.ok) {
          return status;
        }
      }
      state_ = SlamState::Tracking;
      relocalization_state_ = "tracking";
      last_relocalization_message_ = "tracking";
      reason_ = "tracking";
      return Status::Ok(reason_);
    }

    relocalization_state_ = "requested";
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      state_ = SlamState::Lost;
      reason_ = "map_not_loaded";
      relocalization_state_ = "map_not_loaded";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }
    std::optional<Pose3d> effective_guess = guess;
    if (!effective_guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value()) {
      effective_guess = composePoses(map_odom_pose_, *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      state_ = SlamState::Lost;
      reason_ = "initial_pose_required";
      relocalization_state_ = "initial_pose_required";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }
    if (!registered_cloud_body_.has_value() || registered_cloud_body_->points.empty()) {
      state_ = SlamState::Lost;
      reason_ = "registered_cloud_unavailable";
      relocalization_state_ = "waiting_for_scan";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }
    if (!odometry_odom_body_.has_value()) {
      state_ = SlamState::Lost;
      reason_ = "odometry_unavailable";
      relocalization_state_ = "waiting_for_odometry";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }

    const NativeRelocalizationResult result = effective_guess.has_value()
        ? relocalizer_->relocalize(*registered_cloud_body_, *effective_guess, *odometry_odom_body_)
        : relocalizer_->globalRelocalize(*registered_cloud_body_, *odometry_odom_body_);
    relocalization_quality_ = result.quality;
    last_relocalization_message_ = result.message;
    if (!result.success) {
      state_ = SlamState::Lost;
      relocalization_state_ = "failed";
      reason_ = result.message;
      return Status::Error(reason_);
    }
    if (result.quality >= 0.0 && result.quality > runtime_config_.relocalization_max_fitness) {
      state_ = SlamState::Lost;
      relocalization_state_ = "rejected";
      reason_ = "relocalization_fitness_rejected";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }
    if (!poseInsideMapBounds(result.map_body)) {
      state_ = SlamState::Lost;
      relocalization_state_ = "rejected";
      reason_ = "relocalization_outside_map_bounds";
      last_relocalization_message_ = reason_;
      return Status::Error(reason_);
    }

    resetTrackingCoreAtPose(*odometry_odom_body_);
    map_odom_pose_ = result.map_odom;
    map_body_pose_at_relocalization_ = result.map_body;
    relocalization_refine_backend_ = result.refine_backend;
    relocalization_refine_iterations_ = result.refine_iterations;
    relocalization_refine_inliers_ = result.refine_inliers;
    relocalization_refine_converged_ = result.refine_converged;
    relocalization_refine_pos_cov_trace_ = result.refine_pos_cov_trace;
    has_map_odom_pose_ = true;
    map_frame_jump_ = true;
    confidence_ = result.quality >= 0.0
        ? std::max(0.0, std::min(1.0, 1.0 - result.quality))
        : confidence_;
    localization_quality_ = confidence_;
    state_ = SlamState::Tracking;
    relocalization_state_ = "completed";
    reason_ = "relocalized";
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

    recordPatchSnapshot();

    state_ = SlamState::Tracking;
    confidence_ = std::max(0.0, std::min(1.0, kf_->degeneracy().effective_ratio));
    localization_quality_ = confidence_;
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
    CloudType saved_cloud;
    if (pcl::io::loadPCDFile<PointType>(pcd.string(), saved_cloud) >= 0) {
      updateMapBounds(saved_cloud);
    }
    const Status trajectory_status = writeTrajectory(pcd.parent_path(), pose_history_);
    if (!trajectory_status.ok) {
      return trajectory_status;
    }
    std::filesystem::create_directories(pcd.parent_path() / "patches", ec);
    if (ec) {
      return Status::Error("create_patches_dir_failed: " + ec.message());
    }
    const std::vector<PatchSnapshot> patches(
        patch_history_.begin(), patch_history_.end());
    const Status patch_status = writePatchBundle(pcd.parent_path(), patches);
    if (!patch_status.ok) {
      return patch_status;
    }
    saved_map_cloud_map_ = map_cloud_map_;
    saved_map_points_ = saved_map_cloud_map_.has_value()
        ? static_cast<int>(saved_map_cloud_map_->points.size())
        : 0;
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    if (!relocalizer_) {
      relocalizer_ = std::make_unique<NativeRelocalizer>();
    }
    std::string relocalizer_message;
    relocalizer_->loadMap(pcd.string(), &relocalizer_message);
    last_relocalization_message_ = relocalizer_message;
    relocalization_state_ = relocalizer_->hasMap() ? "idle" : "map_load_failed";
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
    updateMapBounds(cloud);
    if (!relocalizer_) {
      relocalizer_ = std::make_unique<NativeRelocalizer>();
    }
    std::string relocalizer_message;
    if (!relocalizer_->loadMap(pcd.string(), &relocalizer_message)) {
      relocalization_state_ = "map_load_failed";
      last_relocalization_message_ = relocalizer_message;
      return Status::Error(relocalizer_message);
    }
    saved_map_cloud_map_.reset();
    saved_map_points_ = static_cast<int>(cloud.size());
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    relocalization_state_ = "idle";
    last_relocalization_message_ = relocalizer_message;
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
    out.map_odom_tf = Transform3d{
        config_.map_frame,
        config_.odom_frame,
        has_map_odom_pose_ ? map_odom_pose_ : Pose3d{}};
    out.saved_map_points = saved_map_points_;
    out.alive = alive_;
    out.map_loaded = map_loaded_;
    out.map_frame_jump = map_frame_jump_;
    out.relocalization_supported = relocalizer_ &&
        (relocalizer_->supportsSeededRelocalization() ||
         relocalizer_->supportsGlobalRelocalization());
    out.saved_map_relocalization_supported =
        out.relocalization_supported && map_loaded_ && relocalizer_ && relocalizer_->hasMap();
    out.relocalization_state = relocalization_state_;
    out.last_relocalization_message = last_relocalization_message_;
    out.relocalization_quality = relocalization_quality_;
    out.relocalization_map_body = map_body_pose_at_relocalization_;
    out.relocalization_refine_backend = relocalization_refine_backend_;
    out.relocalization_refine_iterations = relocalization_refine_iterations_;
    out.relocalization_refine_inliers = relocalization_refine_inliers_;
    out.relocalization_refine_converged = relocalization_refine_converged_;
    out.relocalization_refine_pos_cov_trace = relocalization_refine_pos_cov_trace_;
    out.localization_quality = localization_quality_;
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
    has_map_odom_pose_ = false;
    map_bounds_ready_ = false;
    map_odom_pose_ = Pose3d{};
    map_body_pose_at_relocalization_.reset();
    saved_map_points_ = 0;
    confidence_ = 0.0;
    localization_quality_ = 0.0;
    relocalization_quality_ = -1.0;
    relocalization_refine_backend_.clear();
    relocalization_refine_iterations_ = -1;
    relocalization_refine_inliers_ = -1;
    relocalization_refine_converged_ = false;
    relocalization_refine_pos_cov_trace_ = -1.0;
    relocalization_state_ = "idle";
    last_relocalization_message_ = "reset";
    reason_ = "reset";
    state_ = SlamState::Initializing;
    return Status::Ok(reason_);
  }

 private:
  void resetCore() {
    kf_ = std::make_shared<IESKF>();
    builder_ = std::make_unique<MapBuilder>(builder_config_, kf_);
    relocalizer_ = std::make_unique<NativeRelocalizer>();
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
    patch_history_.clear();
    patch_sequence_ = 0;
    last_patch_stamp_s_ = 0.0;
    has_last_patch_pose_ = false;
  }

  void resetTrackingCoreAtPose(const Pose3d& pose) {
    kf_ = std::make_shared<IESKF>();
    builder_ = std::make_unique<MapBuilder>(builder_config_, kf_);
    applyPose(kf_->x(), pose);
    imu_buffer_.clear();
    lidar_buffer_.clear();
    lidar_pushed_ = false;
    package_ = SyncPackage{};
    last_lidar_time_ = -1.0;
    last_imu_time_ = -1.0;
    scan_start_s_ = 0.0;
    scan_end_s_ = 0.0;
    imu_batch_ = 0;
    sync_wait_count_ = 0;
    odometry_odom_body_ = pose;
    state_estimation_at_scan_ = pose;
    registered_cloud_body_.reset();
    map_cloud_map_.reset();
    pose_history_.clear();
    pose_history_.push_back(OdomSample{last_stamp_s_, pose});
    patch_history_.clear();
    patch_sequence_ = 0;
    last_patch_stamp_s_ = 0.0;
    has_last_patch_pose_ = false;
  }

  void pushLidarFrame(LidarFrame frame) {
    lidar_buffer_.push_back(std::move(frame));
    while (lidar_buffer_.size() > runtime_config_.max_lidar_buffer) {
      lidar_buffer_.pop_front();
      ++dropped_lidar_frames_;
    }
  }

  void updateMapBounds(const CloudType& cloud) {
    map_bounds_ready_ = false;
    map_min_x_ = map_min_y_ = map_min_z_ = std::numeric_limits<double>::infinity();
    map_max_x_ = map_max_y_ = map_max_z_ = -std::numeric_limits<double>::infinity();
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      map_min_x_ = std::min(map_min_x_, static_cast<double>(point.x));
      map_min_y_ = std::min(map_min_y_, static_cast<double>(point.y));
      map_min_z_ = std::min(map_min_z_, static_cast<double>(point.z));
      map_max_x_ = std::max(map_max_x_, static_cast<double>(point.x));
      map_max_y_ = std::max(map_max_y_, static_cast<double>(point.y));
      map_max_z_ = std::max(map_max_z_, static_cast<double>(point.z));
      map_bounds_ready_ = true;
    }
  }

  bool poseInsideMapBounds(const Pose3d& pose) const {
    if (!map_bounds_ready_) {
      return true;
    }
    const double margin = std::max(0.0, runtime_config_.relocalization_map_bounds_margin_m);
    return pose.x >= map_min_x_ - margin && pose.x <= map_max_x_ + margin &&
           pose.y >= map_min_y_ - margin && pose.y <= map_max_y_ + margin &&
           pose.z >= map_min_z_ - margin && pose.z <= map_max_z_ + margin;
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
    if (state_ == SlamState::Tracking &&
        odometry_odom_body_.has_value() &&
        map_cloud_map_.has_value()) {
      reason_ = "tracking";
      updateBuilderState();
      return;
    }
    if (imu_buffer_.empty()) {
      reason_ = "waiting_for_imu";
    } else if (lidar_buffer_.empty()) {
      reason_ = "waiting_for_lidar";
    } else {
      reason_ = "waiting_for_imu_to_cover_scan";
    }
    updateBuilderState();
  }

  void recordPatchSnapshot() {
    if (!registered_cloud_body_.has_value() || registered_cloud_body_->points.empty()) {
      return;
    }
    if (!odometry_odom_body_.has_value()) {
      return;
    }
    const double stamp = registered_cloud_body_->stamp_s > 0.0
        ? registered_cloud_body_->stamp_s
        : last_stamp_s_;
    const Pose3d pose = *odometry_odom_body_;
    const bool time_ready =
        last_patch_stamp_s_ <= 0.0 || (stamp - last_patch_stamp_s_) >= 1.0;
    const bool motion_ready =
        !has_last_patch_pose_ || planarDistance(pose, last_patch_pose_) >= 0.20;
    if (!patch_history_.empty() && !time_ready && !motion_ready) {
      return;
    }

    PatchSnapshot patch;
    patch.name = patchName(patch_sequence_++);
    patch.stamp_s = stamp;
    patch.pose = pose;
    patch.cloud = *registered_cloud_body_;
    patch_history_.push_back(std::move(patch));
    while (patch_history_.size() > 300) {
      patch_history_.pop_front();
    }
    last_patch_stamp_s_ = stamp;
    last_patch_pose_ = pose;
    has_last_patch_pose_ = true;
  }

  void updateBuilderState() {
    if (!builder_) {
      state_ = SlamState::Failed;
      return;
    }
    if (builder_->status() == BuilderStatus::IMU_INIT) {
      state_ = SlamState::Initializing;
      confidence_ = 0.0;
      localization_quality_ = confidence_;
      reason_ = reason_.empty() ? "imu_initializing" : reason_;
    } else if (builder_->status() == BuilderStatus::MAP_INIT) {
      state_ = SlamState::Mapping;
      confidence_ = 0.2;
      localization_quality_ = confidence_;
      reason_ = "map_initializing";
    } else {
      state_ = SlamState::Tracking;
      confidence_ = std::max(0.0, std::min(1.0, kf_->degeneracy().effective_ratio));
      localization_quality_ = confidence_;
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
  bool map_bounds_ready_ = false;
  double map_min_x_ = 0.0;
  double map_min_y_ = 0.0;
  double map_min_z_ = 0.0;
  double map_max_x_ = 0.0;
  double map_max_y_ = 0.0;
  double map_max_z_ = 0.0;
  std::shared_ptr<IESKF> kf_;
  std::unique_ptr<MapBuilder> builder_;
  std::unique_ptr<NativeRelocalizer> relocalizer_;
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
  bool has_map_odom_pose_ = false;
  Pose3d map_odom_pose_;
  std::optional<Pose3d> map_body_pose_at_relocalization_;
  double confidence_ = 0.0;
  double localization_quality_ = 0.0;
  double relocalization_quality_ = -1.0;
  std::string relocalization_refine_backend_;
  int relocalization_refine_iterations_ = -1;
  int relocalization_refine_inliers_ = -1;
  bool relocalization_refine_converged_ = false;
  double relocalization_refine_pos_cov_trace_ = -1.0;
  std::string relocalization_state_ = "idle";
  std::string last_relocalization_message_;
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
  int saved_map_points_ = 0;
  GnssFusionHealth gnss_health_;
  std::vector<OdomSample> pose_history_;
  std::deque<PatchSnapshot> patch_history_;
  std::uint64_t patch_sequence_ = 0;
  double last_patch_stamp_s_ = 0.0;
  Pose3d last_patch_pose_;
  bool has_last_patch_pose_ = false;
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
