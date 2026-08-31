#include "slam.hpp"

#if defined(LINGTU_HAS_FASTLIO2_BACKEND) && LINGTU_HAS_FASTLIO2_BACKEND

#include "map_builder/map_builder.h"
#include "native_relocalizer.hpp"
#include "relocalization_gate.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

namespace lingtu::slam {
namespace {

struct RuntimeConfig {
  double acc_scale = 1.0;
  double time_diff_lidar_to_imu = 0.0;
  double max_imu_gap_s = 0.25;
  double max_sensor_time_jump_s = 5.0;
  double relocalization_max_fitness = 0.5;
  int relocalization_min_inliers = 30;
  int relocalization_min_evaluated_points = 30;
  double relocalization_max_pos_cov_trace = 1.0;
  // A 100 m^2 position-covariance trace means aggregate 1-sigma uncertainty
  // has reached 10 m. At that point local odometry is not safe to publish or
  // to revive through a map-only relocalization result.
  double fastlio_max_pos_cov_trace = 100.0;
  double relocalization_map_bounds_margin_m = 2.0;
  double track_against_map_max_translation_m = 1.0;
  double track_against_map_max_yaw_deg = 15.0;
  double track_against_map_max_tilt_deg = 5.0;
  bool track_against_map_require_degeneracy_metrics = true;
  bool odom_prior_enabled = false;
  bool odom_prior_bypass_fastlio = false;
  double odom_prior_max_age_s = 0.20;
  double odom_prior_map_resolution = 0.20;
  double odom_prior_snap_translation_m = 0.25;
  std::size_t max_odom_prior_buffer = 4000;
  std::size_t max_patch_snapshots = 3000;
  double patch_min_interval_s = 1.0;
  double patch_min_translation_m = 0.20;
  double patch_min_rotation_rad = 0.0872664626;
  std::size_t max_imu_buffer = 4000;
  std::size_t max_lidar_buffer = 64;
  Eigen::Matrix3d navigation_body_from_imu_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d navigation_body_from_imu_translation = Eigen::Vector3d::Zero();
};

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    const std::uint64_t ux = static_cast<std::uint64_t>(key.x);
    const std::uint64_t uy = static_cast<std::uint64_t>(key.y);
    const std::uint64_t uz = static_cast<std::uint64_t>(key.z);
    std::uint64_t h = ux * 73856093ULL;
    h ^= uy * 19349663ULL;
    h ^= uz * 83492791ULL;
    return static_cast<std::size_t>(h);
  }
};

struct PatchSnapshot {
  std::string name;
  std::uint64_t sequence = 0;
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
  readIfPresent(config, "max_sensor_time_jump_s", runtime_config.max_sensor_time_jump_s);
  readIfPresent(config, "relocalization_max_fitness", runtime_config.relocalization_max_fitness);
  readIfPresent(config, "relocalization_min_inliers", runtime_config.relocalization_min_inliers);
  readIfPresent(
      config,
      "relocalization_min_evaluated_points",
      runtime_config.relocalization_min_evaluated_points);
  readIfPresent(
      config,
      "relocalization_max_pos_cov_trace",
      runtime_config.relocalization_max_pos_cov_trace);
  readIfPresent(
      config,
      "fastlio_max_pos_cov_trace",
      runtime_config.fastlio_max_pos_cov_trace);
  readIfPresent(
      config,
      "relocalization_map_bounds_margin_m",
      runtime_config.relocalization_map_bounds_margin_m);
  readIfPresent(
      config,
      "track_against_map_max_translation_m",
      runtime_config.track_against_map_max_translation_m);
  readIfPresent(
      config,
      "track_against_map_max_yaw_deg",
      runtime_config.track_against_map_max_yaw_deg);
  readIfPresent(
      config,
      "track_against_map_max_tilt_deg",
      runtime_config.track_against_map_max_tilt_deg);
  readIfPresent(
      config,
      "track_against_map_require_degeneracy_metrics",
      runtime_config.track_against_map_require_degeneracy_metrics);
  readIfPresent(config, "odom_prior_enabled", runtime_config.odom_prior_enabled);
  readIfPresent(
      config,
      "odom_prior_bypass_fastlio",
      runtime_config.odom_prior_bypass_fastlio);
  readIfPresent(config, "odom_prior_max_age_s", runtime_config.odom_prior_max_age_s);
  readIfPresent(config, "odom_prior_map_resolution", runtime_config.odom_prior_map_resolution);
  readIfPresent(
      config,
      "odom_prior_snap_translation_m",
      runtime_config.odom_prior_snap_translation_m);
  readIfPresent(
      config,
      "max_odom_prior_buffer",
      runtime_config.max_odom_prior_buffer);
  readIfPresent(config, "max_patch_snapshots", runtime_config.max_patch_snapshots);
  readIfPresent(config, "patch_min_interval_s", runtime_config.patch_min_interval_s);
  readIfPresent(config, "patch_min_translation_m", runtime_config.patch_min_translation_m);
  readIfPresent(config, "patch_min_rotation_rad", runtime_config.patch_min_rotation_rad);
  readIfPresent(config, "max_imu_buffer", runtime_config.max_imu_buffer);
  readIfPresent(config, "max_lidar_buffer", runtime_config.max_lidar_buffer);

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
  readIfPresent(config, "max_update_velocity_mps", builder_config.max_update_velocity_mps);
  readIfPresent(config, "max_update_velocity_delta_mps", builder_config.max_update_velocity_delta_mps);
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
  if (config["navigation_body_from_imu_translation"]) {
    const auto t = config["navigation_body_from_imu_translation"].as<std::vector<double>>();
    if (t.size() != 3) {
      return Status::Error("fastlio2_navigation_body_from_imu_translation_must_have_3_values");
    }
    runtime_config.navigation_body_from_imu_translation << t[0], t[1], t[2];
  }
  if (config["navigation_body_from_imu_rotation"]) {
    const auto r = config["navigation_body_from_imu_rotation"].as<std::vector<double>>();
    if (r.size() != 9) {
      return Status::Error("fastlio2_navigation_body_from_imu_rotation_must_have_9_values");
    }
    runtime_config.navigation_body_from_imu_rotation <<
        r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }

  return Status::Ok("fastlio2_config_loaded");
}

bool validPoint(const PointXYZIT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

CloudType::Ptr toPclCloud(const LidarFrame& frame, const Config& config) {
  CloudType::Ptr cloud(new CloudType);
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

Pose3d inversePose(const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  if (!std::isfinite(q.norm()) || q.norm() <= 0.0) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  const Eigen::Quaterniond inverse_q = q.conjugate();
  const Eigen::Vector3d inverse_t =
      -(inverse_q * Eigen::Vector3d(pose.x, pose.y, pose.z));
  Pose3d out;
  out.x = inverse_t.x();
  out.y = inverse_t.y();
  out.z = inverse_t.z();
  out.qx = inverse_q.x();
  out.qy = inverse_q.y();
  out.qz = inverse_q.z();
  out.qw = inverse_q.w();
  return out;
}

Pose3d navigationBodyFromImuTransform(const RuntimeConfig& config) {
  Eigen::Quaterniond q(config.navigation_body_from_imu_rotation);
  q.normalize();
  Pose3d out;
  out.x = config.navigation_body_from_imu_translation.x();
  out.y = config.navigation_body_from_imu_translation.y();
  out.z = config.navigation_body_from_imu_translation.z();
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

Pose3d navigationBodyPoseFromImuPose(
    const Pose3d& odom_imu,
    const RuntimeConfig& config) {
  return composePoses(odom_imu, inversePose(navigationBodyFromImuTransform(config)));
}

Pose3d imuPoseFromNavigationBodyPose(
    const Pose3d& odom_body,
    const RuntimeConfig& config) {
  return composePoses(odom_body, navigationBodyFromImuTransform(config));
}

void anchorInitialNavigationBodyFrame(
    State& state,
    const RuntimeConfig& config) {
  const Pose3d initial_body =
      navigationBodyPoseFromImuPose(poseFromState(state), config);
  Eigen::Quaterniond q_wb(
      initial_body.qw,
      initial_body.qx,
      initial_body.qy,
      initial_body.qz);
  if (!std::isfinite(q_wb.norm()) || q_wb.norm() <= 0.0) {
    q_wb = Eigen::Quaterniond::Identity();
  }
  q_wb.normalize();
  const Eigen::Matrix3d r_wb = q_wb.toRotationMatrix();
  const double initial_yaw = std::atan2(r_wb(1, 0), r_wb(0, 0));
  const Eigen::Matrix3d r_anchor =
      Eigen::AngleAxisd(-initial_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d t_wb(initial_body.x, initial_body.y, initial_body.z);
  const Eigen::Vector3d t_anchor = -(r_anchor * t_wb);

  // Re-express the just-initialized filter state in an odom frame whose
  // origin and heading are the initial navigation body pose.  A yaw-only
  // rotation preserves the gravity-aligned world Z axis.  This runs before
  // MAP_INIT, while the state covariance is still the isotropic IMU-init
  // covariance and before any map point is inserted.
  state.r_wi = r_anchor * state.r_wi;
  state.t_wi = r_anchor * state.t_wi + t_anchor;
  state.v = r_anchor * state.v;
}

CloudType::Ptr transformLidarCloudToNavigationBody(
    const CloudType::Ptr& cloud,
    const Config& config,
    const RuntimeConfig& runtime_config) {
  const Eigen::Matrix3d r_bi = runtime_config.navigation_body_from_imu_rotation;
  const Eigen::Vector3d t_bi = runtime_config.navigation_body_from_imu_translation;
  const Eigen::Matrix3d r_bl = r_bi * config.r_il;
  const Eigen::Vector3d t_bl = t_bi + r_bi * config.t_il;
  return LidarProcessor::transformCloud(cloud, r_bl, t_bl);
}

void applyPose(State& state, const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  state.t_wi = V3D(pose.x, pose.y, pose.z);
  state.r_wi = q.toRotationMatrix();
}

Eigen::Matrix3d rotationFromPose(const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  if (!std::isfinite(q.norm()) || q.norm() <= 0.0) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q.toRotationMatrix();
}

CloudType::Ptr transformLidarCloudWithBodyPose(
    const CloudType::Ptr& cloud,
    const Pose3d& body_pose,
    const Config& config,
    const RuntimeConfig& runtime_config) {
  const Eigen::Matrix3d r_wb = rotationFromPose(body_pose);
  const Eigen::Vector3d t_wb(body_pose.x, body_pose.y, body_pose.z);
  const Eigen::Matrix3d r_bi = runtime_config.navigation_body_from_imu_rotation;
  const Eigen::Vector3d t_bi = runtime_config.navigation_body_from_imu_translation;
  const Eigen::Matrix3d r_bl = r_bi * config.r_il;
  const Eigen::Vector3d t_bl = t_bi + r_bi * config.t_il;
  const Eigen::Matrix3d r_wl = r_wb * r_bl;
  const Eigen::Vector3d t_wl = t_wb + r_wb * t_bl;
  return LidarProcessor::transformCloud(cloud, r_wl, t_wl);
}

VoxelKey voxelKeyForPoint(const PointType& point, double resolution) {
  const double res = std::max(1e-3, resolution);
  return VoxelKey{
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / res)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / res)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.z) / res))};
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
    const std::vector<PatchSnapshot>& patches,
    std::uint64_t dropped_count) {
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
  const Status index_status = writePatchIndex(map_dir, patches);
  if (!index_status.ok) {
    return index_status;
  }
  std::ofstream manifest(map_dir / "patch_bundle.manifest");
  if (!manifest) {
    return Status::Error("open_patch_bundle_manifest_failed");
  }
  manifest << "LINGTU_PATCH_BUNDLE_V1\n"
           << "complete " << (dropped_count == 0 ? 1 : 0) << '\n'
           << "dropped_count " << dropped_count << '\n'
           << "first_sequence " << patches.front().sequence << '\n'
           << "last_sequence " << patches.back().sequence << '\n'
           << "patch_count " << patches.size() << '\n';
  manifest.flush();
  if (!manifest) {
    return Status::Error("write_patch_bundle_manifest_failed");
  }
  return Status::Ok("patch_bundle_written");
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

std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(ch));
        } else {
          out << ch;
        }
    }
  }
  return out.str();
}

double wrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromPose(const Pose3d& pose) {
  const Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  const Eigen::Vector3d ypr = q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
  return ypr[0];
}

Pose3d planarSeedWithOdomHeightAndTilt(
    const Pose3d& seed,
    const Pose3d& odom_body) {
  const Eigen::Matrix3d odom_rotation = rotationFromPose(odom_body);
  const double odom_pitch = std::asin(std::clamp(-odom_rotation(2, 0), -1.0, 1.0));
  const double odom_roll = std::atan2(odom_rotation(2, 1), odom_rotation(2, 2));
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(yawFromPose(seed), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(odom_pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(odom_roll, Eigen::Vector3d::UnitX());
  Pose3d out = seed;
  out.z = odom_body.z;
  out.qx = rotation.x();
  out.qy = rotation.y();
  out.qz = rotation.z();
  out.qw = rotation.w();
  return out;
}

double yawDistance(const Pose3d& a, const Pose3d& b) {
  return std::abs(wrapAngle(yawFromPose(a) - yawFromPose(b)));
}

struct AsyncRelocalizationComputeState {
  NativeRelocalizationResult result;
  std::atomic<bool> ready{false};
};

struct AsyncRelocalizationJob {
  std::uint64_t job_sequence = 0U;
  std::uint64_t map_epoch = 0U;
  std::uint64_t alignment_sequence = 0U;
  std::uint64_t observation_sequence = 0U;
  bool map_alignment_update = false;
  bool preserve_tracking_on_failure = false;
  std::shared_ptr<AsyncRelocalizationComputeState> compute;
};

class FastLioBackend final : public ISlamBackend {
 public:
  Status configure(const SlamConfig& config) override {
    advanceRelocalizationMapEpoch();
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
      advanceRelocalizationMapEpoch();
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
    if (isSensorTimeJump(last_imu_time_, timestamp)) {
      handleSensorTimeJump(timestamp);
    }
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
    if (isSensorTimeJump(last_lidar_time_, frame.stamp_s)) {
      handleSensorTimeJump(frame.stamp_s);
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
    if (runtime_config_.odom_prior_enabled) {
      const auto insert_at = std::lower_bound(
          odom_prior_buffer_.begin(),
          odom_prior_buffer_.end(),
          sample.stamp_s,
          [](const OdomSample& buffered, double stamp_s) {
            return buffered.stamp_s < stamp_s;
          });
      if (insert_at != odom_prior_buffer_.end() &&
          std::abs(insert_at->stamp_s - sample.stamp_s) <= 1e-9) {
        *insert_at = sample;
      } else {
        odom_prior_buffer_.insert(insert_at, sample);
      }
      const std::size_t max_buffer =
          std::max<std::size_t>(1, runtime_config_.max_odom_prior_buffer);
      while (odom_prior_buffer_.size() > max_buffer) {
        odom_prior_buffer_.pop_front();
      }
      latest_odom_prior_ = odom_prior_buffer_.back();
      odom_prior_age_s_ = 0.0;
      reason_ = "odom_prior_accepted";
      return Status::Ok(reason_);
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
    applyPose(kf_->x(), imuPoseFromNavigationBodyPose(pose, runtime_config_));
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

    abandonAsyncRelocalization();
    relocalization_state_ = "requested";
    const bool preserve_tracking_on_relocalization_failure =
        state_ == SlamState::Tracking && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const bool map_alignment_update =
        !guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const std::string reason_before_relocalization = reason_;
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      return failRelocalization(
          "map_not_loaded", "map_not_loaded", false, reason_before_relocalization);
    }
    std::optional<Pose3d> effective_guess = guess;
    if (!effective_guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value()) {
      effective_guess = composePoses(map_odom_pose_, *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      return failRelocalization(
          "initial_pose_required", "initial_pose_required", false, reason_before_relocalization);
    }
    if (!registered_cloud_body_.has_value() || registered_cloud_body_->points.empty()) {
      return failRelocalization(
          "waiting_for_scan",
          "registered_cloud_unavailable",
          preserve_tracking_on_relocalization_failure,
          reason_before_relocalization);
    }
    if (!odometry_odom_body_.has_value()) {
      return failRelocalization(
          "waiting_for_odometry", "odometry_unavailable", false, reason_before_relocalization);
    }

    const NativeRelocalizationResult result = effective_guess.has_value()
        ? relocalizer_->relocalize(*registered_cloud_body_, *effective_guess, *odometry_odom_body_)
        : relocalizer_->globalRelocalize(*registered_cloud_body_, *odometry_odom_body_);
    return applyRelocalizationResult(
        result,
        map_alignment_update,
        preserve_tracking_on_relocalization_failure,
        reason_before_relocalization);
  }

  Status startRelocalizeAsync(const std::optional<Pose3d>& guess) override {
    if (mode_ != SlamMode::Localization) {
      return Status::Error("localization_mode_required");
    }
    if (async_relocalization_job_.has_value()) {
      return Status::Error("async_relocalization_in_progress");
    }
    const bool preserve_tracking_on_failure =
        state_ == SlamState::Tracking && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const bool map_alignment_update =
        !guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const std::string reason_before_relocalization = reason_;
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      return failRelocalization(
          "map_not_loaded", "map_not_loaded", false, reason_before_relocalization);
    }
    std::optional<Pose3d> effective_guess = guess;
    if (runtime_config_.odom_prior_bypass_fastlio &&
        guess.has_value() && odometry_odom_body_.has_value()) {
      effective_guess = planarSeedWithOdomHeightAndTilt(
          *guess, *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value()) {
      effective_guess = composePoses(map_odom_pose_, *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      return failRelocalization(
          "initial_pose_required", "initial_pose_required", false, reason_before_relocalization);
    }
    if (!registered_cloud_body_.has_value() || registered_cloud_body_->points.empty()) {
      return failRelocalization(
          "waiting_for_scan",
          "registered_cloud_unavailable",
          preserve_tracking_on_failure,
          reason_before_relocalization);
    }
    if (!odometry_odom_body_.has_value()) {
      return failRelocalization(
          "waiting_for_odometry", "odometry_unavailable", false, reason_before_relocalization);
    }

    const Cloud scan = *registered_cloud_body_;
    const Pose3d odom_body = *odometry_odom_body_;
    const auto relocalizer = relocalizer_;
    const auto compute = std::make_shared<AsyncRelocalizationComputeState>();
    AsyncRelocalizationJob job;
    job.job_sequence = ++async_relocalization_sequence_;
    job.map_epoch = relocalization_map_epoch_;
    job.alignment_sequence = map_frame_jump_sequence_;
    job.observation_sequence = observation_sequence_;
    job.map_alignment_update = map_alignment_update;
    job.preserve_tracking_on_failure = preserve_tracking_on_failure;
    job.compute = compute;

    try {
      std::thread worker([
          compute,
          relocalizer,
          scan,
          effective_guess,
          odom_body]() mutable {
        try {
          compute->result = effective_guess.has_value()
              ? relocalizer->relocalize(scan, *effective_guess, odom_body)
              : relocalizer->globalRelocalize(scan, odom_body);
        } catch (const std::exception& error) {
          compute->result.message =
              std::string("async_relocalization_exception: ") + error.what();
        } catch (...) {
          compute->result.message = "async_relocalization_unknown_exception";
        }
        compute->ready.store(true, std::memory_order_release);
      });
      worker.detach();
    } catch (const std::exception& error) {
      return Status::Error(
          std::string("async_relocalization_start_failed: ") + error.what());
    }

    async_relocalization_job_ = std::move(job);
    relocalization_state_ = preserve_tracking_on_failure ? "tracking" : "running";
    last_relocalization_message_ = "async_relocalization_running";
    return Status::Ok("async_relocalization_started");
  }

  std::optional<Status> pollRelocalizeAsync() override {
    if (!async_relocalization_job_.has_value()) {
      return std::nullopt;
    }
    const auto& pending = *async_relocalization_job_;
    if (!pending.compute->ready.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    AsyncRelocalizationJob completed = std::move(*async_relocalization_job_);
    async_relocalization_job_.reset();
    const bool preserve_tracking =
        completed.preserve_tracking_on_failure ||
        (state_ == SlamState::Tracking && has_map_odom_pose_ && odometry_odom_body_.has_value());
    const std::string reason_before_relocalization = reason_;
    if (completed.job_sequence != async_relocalization_sequence_) {
      return failRelocalization(
          "rejected",
          "async_relocalization_sequence_stale",
          preserve_tracking,
          reason_before_relocalization);
    }
    if (completed.map_epoch != relocalization_map_epoch_) {
      return failRelocalization(
          "rejected",
          "async_relocalization_map_changed",
          preserve_tracking,
          reason_before_relocalization);
    }
    if (completed.alignment_sequence != map_frame_jump_sequence_) {
      return failRelocalization(
          "rejected",
          "async_relocalization_alignment_stale",
          preserve_tracking,
          reason_before_relocalization);
    }
    if (completed.observation_sequence == 0U) {
      return failRelocalization(
          "rejected",
          "async_relocalization_observation_invalid",
          preserve_tracking,
          reason_before_relocalization);
    }
    return applyRelocalizationResult(
        completed.compute->result,
        completed.map_alignment_update,
        preserve_tracking,
        reason_before_relocalization);
  }

  bool relocalizeAsyncInFlight() const override {
    return async_relocalization_job_.has_value();
  }

  Status tick() override {
    if (!alive_ || !builder_ || !kf_) {
      state_ = SlamState::Failed;
      reason_ = "not_configured";
      return Status::Error(reason_);
    }
    // Relocalization publishes the jump in the same runtime cycle. Clear it
    // before processing the next cycle so consumers observe an event, not a
    // permanently latched state.
    map_frame_jump_ = false;
    if (catastrophic_health_fault_latched_ && mode_ == SlamMode::Mapping) {
      enforceCatastrophicHealthFault();
      return Status::Ok(reason_);
    }
    if (!prepareFastLioPackage()) {
      updateWaitingReason();
      return Status::Ok(reason_);
    }

    std::optional<OdomSample> odom_prior = freshOdomPrior(package_.cloud_end_time);
    if (runtime_config_.odom_prior_bypass_fastlio) {
      if (odom_prior.has_value()) {
        return processOdomPriorPackage(*odom_prior);
      }
      reason_ = "waiting_for_time_aligned_odom_prior";
      return Status::Ok(reason_);
    }

    // Fast-LIO2 core: IMU init/propagation, point undistortion, and map update.
    const BuilderStatus builder_status_before = builder_->status();
    builder_->process(package_);
    if (mode_ == SlamMode::Mapping &&
        builder_status_before == BuilderStatus::IMU_INIT &&
        builder_->status() == BuilderStatus::MAP_INIT) {
      anchorInitialNavigationBodyFrame(kf_->x(), runtime_config_);
    }
    last_stamp_s_ = package_.cloud_end_time;
    if (builder_->status() != BuilderStatus::MAPPING) {
      updateBuilderState();
      if (relocalization_required_after_time_jump_) {
        state_ = SlamState::Lost;
        confidence_ = 0.0;
        localization_quality_ = 0.0;
        reason_ = "sensor_time_jump_relocalization_required";
      }
      return Status::Ok(reason_);
    }

    const Pose3d fastlio_pose =
        navigationBodyPoseFromImuPose(poseFromState(kf_->x()), runtime_config_);
    if (odom_prior.has_value()) {
      if (!validPose(odom_prior->odom_body)) {
        setFastLioHealthFault(SlamState::Lost, "fastlio_state_nonfinite", true);
        return Status::Ok(reason_);
      }
      if (odom_prior->has_velocity) {
        const V3D prior_velocity(odom_prior->vx, odom_prior->vy, odom_prior->vz);
        if (!prior_velocity.allFinite()) {
          setFastLioHealthFault(SlamState::Lost, "fastlio_state_nonfinite", true);
          return Status::Ok(reason_);
        }
        if (velocityOutOfBounds(prior_velocity)) {
          setFastLioHealthFault(SlamState::Lost, "fastlio_velocity_out_of_bounds", true);
          return Status::Ok(reason_);
        }
      }
    } else {
      const std::string numeric_fault = fastLioNumericHealthFaultReason();
      if (!numeric_fault.empty()) {
        setFastLioHealthFault(SlamState::Lost, numeric_fault, true);
        return Status::Ok(reason_);
      }
    }
    if (!odom_prior.has_value() &&
        builder_->lastLidarUpdateAttempted() && !builder_->lastLidarUpdateAccepted()) {
      const bool repeated = builder_->consecutiveLidarUpdateRejections() >= 2U;
      setFastLioHealthFault(
          repeated ? SlamState::Lost : SlamState::Degraded,
          repeated
              ? "fastlio_lidar_update_rejected_streak"
              : "fastlio_lidar_update_rejected");
      return Status::Ok(reason_);
    }

    if (odom_prior.has_value()) {
      odom_prior_error_xy_m_ = validPose(fastlio_pose)
          ? planarDistance(fastlio_pose, odom_prior->odom_body)
          : -1.0;
      applyPose(
          kf_->x(),
          imuPoseFromNavigationBodyPose(odom_prior->odom_body, runtime_config_));
      if (odom_prior->has_velocity) {
        kf_->x().v = V3D(odom_prior->vx, odom_prior->vy, odom_prior->vz);
      } else if (!kf_->x().v.allFinite() || velocityOutOfBounds(kf_->x().v)) {
        kf_->x().v.setZero();
      }
      odometry_odom_body_ = odom_prior->odom_body;
      state_estimation_at_scan_ = odom_prior->odom_body;
    } else {
      odometry_odom_body_ = fastlio_pose;
      state_estimation_at_scan_ = odometry_odom_body_;
    }

    pose_history_.push_back(OdomSample{package_.cloud_end_time, *odometry_odom_body_});
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }

    auto body_cloud = transformLidarCloudToNavigationBody(
        package_.cloud, builder_config_, runtime_config_);
    registered_cloud_body_ =
        toContractCloud(body_cloud, package_.cloud_end_time, config_.body_frame);
    ++observation_sequence_;

    auto world_cloud = odom_prior.has_value()
        ? transformLidarCloudWithBodyPose(
            package_.cloud,
            odom_prior->odom_body,
            builder_config_,
            runtime_config_)
        : LidarProcessor::transformCloud(
            package_.cloud,
            builder_->lidar_processor()->r_wl(),
            builder_->lidar_processor()->t_wl());
    map_cloud_map_ =
        toContractCloud(world_cloud, package_.cloud_end_time, config_.map_frame);
    if (odom_prior.has_value()) {
      addOdomPriorMapCloud(world_cloud);
    }

    recordPatchSnapshot();

    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return Status::Ok(reason_);
    }
    fastlio_health_fault_active_ = false;
    if (relocalization_required_after_time_jump_) {
      state_ = SlamState::Lost;
      confidence_ = 0.0;
      localization_quality_ = 0.0;
      reason_ = "sensor_time_jump_relocalization_required";
      return Status::Ok(reason_);
    }
    state_ = SlamState::Tracking;
    confidence_ = std::max(0.0, std::min(1.0, kf_->degeneracy().effective_ratio));
    localization_quality_ = confidence_;
    reason_ = odom_prior.has_value() ? "tracking_with_odom_prior" : "tracking";
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
    CloudType saved_cloud;
    const bool save_odom_prior_map =
        runtime_config_.odom_prior_enabled && !odom_prior_map_.empty();
    if (save_odom_prior_map) {
      saved_cloud = odomPriorMapPclCloud();
      if (pcl::io::savePCDFileBinary(pcd.string(), saved_cloud) != 0) {
        return Status::Error("map_pcd_write_failed");
      }
    } else {
      builder_->saveMap(pcd.string());
      if (!std::filesystem::exists(pcd)) {
        return Status::Error("map_pcd_write_failed");
      }
      if (pcl::io::loadPCDFile<PointType>(pcd.string(), saved_cloud) < 0) {
        saved_cloud.clear();
      }
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
    const Status patch_status =
        writePatchBundle(pcd.parent_path(), patches, patch_history_dropped_count_);
    if (!patch_status.ok) {
      return patch_status;
    }
    saved_map_cloud_map_ = save_odom_prior_map
        ? std::optional<Cloud>{odomPriorMapContractCloud(last_stamp_s_)}
        : toContractCloud(
              CloudType::Ptr(new CloudType(saved_cloud)),
              last_stamp_s_,
              config_.map_frame);
    if (!saved_cloud.empty()) {
      updateMapBounds(saved_cloud);
    }
    saved_map_points_ = saved_map_cloud_map_.has_value()
        ? static_cast<int>(saved_map_cloud_map_->points.size())
        : 0;
    map_loaded_ = true;
    last_map_path_ = pcd.string();
    if (!relocalizer_) {
      relocalizer_ = std::make_shared<NativeRelocalizer>();
    }
    std::string relocalizer_message;
    const bool relocalizer_loaded =
        relocalizer_->loadMap(pcd.string(), &relocalizer_message);
    if (relocalizer_loaded) {
      advanceRelocalizationMapEpoch();
    }
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
      relocalizer_ = std::make_shared<NativeRelocalizer>();
    }
    std::string relocalizer_message;
    if (!relocalizer_->loadMap(pcd.string(), &relocalizer_message)) {
      relocalization_state_ = "map_load_failed";
      last_relocalization_message_ = relocalizer_message;
      return Status::Error(relocalizer_message);
    }
    advanceRelocalizationMapEpoch();
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
    if (!fastlio_health_fault_active_ && !relocalization_required_after_time_jump_) {
      out.odometry_odom_body = odometry_odom_body_;
      out.state_estimation_at_scan = state_estimation_at_scan_;
      out.registered_cloud_body = registered_cloud_body_;
      out.map_cloud_map = map_cloud_map_;
    }
    out.saved_map_cloud_map = saved_map_cloud_map_;
    if (mode_ != SlamMode::Localization || has_map_odom_pose_) {
      out.map_odom_tf = Transform3d{
          config_.map_frame,
          config_.odom_frame,
          has_map_odom_pose_ ? map_odom_pose_ : Pose3d{}};
    }
    out.observation_sequence = observation_sequence_;
    out.source_epoch = source_epoch_;
    out.saved_map_points = saved_map_points_;
    out.alive = alive_;
    out.map_loaded = map_loaded_;
    out.map_frame_jump = map_frame_jump_;
    out.map_frame_jump_sequence = map_frame_jump_sequence_;
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
    out.relocalization_min_inliers =
        runtime_config_.relocalization_min_inliers;
    out.relocalization_min_evaluated_points =
        runtime_config_.relocalization_min_evaluated_points;
    out.relocalization_refine_input_points =
        relocalization_refine_input_points_;
    out.relocalization_refine_evaluated_points =
        relocalization_refine_evaluated_points_;
    out.relocalization_refine_support_ratio =
        relocalization_refine_support_ratio_;
    out.relocalization_refine_overlap_inlier_ratio =
        relocalization_refine_overlap_inlier_ratio_;
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
    out.odom_prior_enabled = runtime_config_.odom_prior_enabled;
    out.odom_prior_active = odom_prior_active_;
    out.odom_prior_age_s = odom_prior_age_s_;
    out.odom_prior_error_xy_m = odom_prior_error_xy_m_;
    out.odom_prior_map_points = static_cast<int>(odom_prior_map_.size());
    if (kf_) {
      const auto& state = kf_->x();
      const auto& degeneracy = kf_->degeneracy();
      out.fastlio_velocity_x = state.v.x();
      out.fastlio_velocity_y = state.v.y();
      out.fastlio_velocity_z = state.v.z();
      out.fastlio_degeneracy_detected = degeneracy.detected;
      out.fastlio_degenerate_dof_count = degeneracy.degenerate_dof_count;
      out.fastlio_condition_number = degeneracy.condition_number;
      out.fastlio_min_eigenvalue = degeneracy.min_eigenvalue;
      out.fastlio_max_eigenvalue = degeneracy.max_eigenvalue;
      out.fastlio_effective_ratio = degeneracy.effective_ratio;
      out.fastlio_pos_cov_trace = degeneracy.pos_cov_trace;
      out.fastlio_iter_num = degeneracy.iter_num;
      out.fastlio_converged = degeneracy.converged;
    }
    if (builder_) {
      const auto& diagnostics = builder_->lastLidarUpdateDiagnostics();
      out.fastlio_lidar_update.attempted = diagnostics.attempted;
      out.fastlio_lidar_update.accepted = diagnostics.accepted;
      out.fastlio_lidar_update.attempt_sequence = diagnostics.attempt_sequence;
      out.fastlio_lidar_update.rejection_reason =
          lidarUpdateRejectionReasonName(diagnostics.rejection_reason);
      out.fastlio_lidar_update.previous_rejection_reason =
          lidarUpdateRejectionReasonName(
              diagnostics.previous_rejection_reason);
      out.fastlio_lidar_update.consecutive_rejections =
          static_cast<std::uint64_t>(diagnostics.consecutive_rejections);
      out.fastlio_lidar_update.downsampled_points =
          static_cast<std::uint64_t>(diagnostics.downsampled_points);
      out.fastlio_lidar_update.effective_points =
          static_cast<std::uint64_t>(diagnostics.effective_points);
      out.fastlio_lidar_update.candidate_translation_m =
          diagnostics.candidate_translation_m;
      out.fastlio_lidar_update.candidate_rotation_rad =
          diagnostics.candidate_rotation_rad;
      out.fastlio_lidar_update.candidate_velocity_mps =
          diagnostics.candidate_velocity_mps;
      out.fastlio_lidar_update.candidate_velocity_delta_mps =
          diagnostics.candidate_velocity_delta_mps;
      out.fastlio_lidar_update.max_update_translation_m =
          diagnostics.max_update_translation_m;
      out.fastlio_lidar_update.max_update_rotation_rad =
          diagnostics.max_update_rotation_rad;
      out.fastlio_lidar_update.max_update_velocity_mps =
          diagnostics.max_update_velocity_mps;
      out.fastlio_lidar_update.max_update_velocity_delta_mps =
          diagnostics.max_update_velocity_delta_mps;
      out.fastlio_lidar_update.information_ldlt_evaluated =
          diagnostics.information_ldlt_evaluated;
      out.fastlio_lidar_update.information_ldlt_decomposition_success =
          diagnostics.information_ldlt_decomposition_success;
      out.fastlio_lidar_update.information_ldlt_positive =
          diagnostics.information_ldlt_positive;
      out.fastlio_lidar_update.candidate_covariance_evaluated =
          diagnostics.candidate_covariance_evaluated;
      out.fastlio_lidar_update.candidate_covariance_finite =
          diagnostics.candidate_covariance_finite;
      out.fastlio_lidar_update.candidate_covariance_positive_diagonal =
          diagnostics.candidate_covariance_positive_diagonal;
      out.fastlio_lidar_update.posterior_covariance_evaluated =
          diagnostics.posterior_covariance_evaluated;
      out.fastlio_lidar_update.posterior_covariance_finite =
          diagnostics.posterior_covariance_finite;
      out.fastlio_lidar_update.posterior_covariance_positive_diagonal =
          diagnostics.posterior_covariance_positive_diagonal;
    }
    return out;
  }

  Status reset() override {
    const bool preserve_catastrophic_fault =
        mode_ == SlamMode::Localization && catastrophic_health_fault_latched_;
    const std::string preserved_catastrophic_reason = catastrophic_health_fault_reason_;
    advanceRelocalizationMapEpoch();
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
    relocalization_refine_input_points_ = 0;
    relocalization_refine_evaluated_points_ = 0;
    relocalization_refine_support_ratio_ = -1.0;
    relocalization_refine_overlap_inlier_ratio_ = -1.0;
    relocalization_refine_converged_ = false;
    relocalization_refine_pos_cov_trace_ = -1.0;
    relocalization_state_ = "idle";
    last_relocalization_message_ = "reset";
    reason_ = "reset";
    state_ = SlamState::Initializing;
    if (preserve_catastrophic_fault) {
      catastrophic_health_fault_latched_ = true;
      catastrophic_health_fault_reason_ = preserved_catastrophic_reason;
      relocalization_state_ = "required";
      last_relocalization_message_ = "catastrophic_health_fault_relocalization_required";
      enforceCatastrophicHealthFault();
    }
    return Status::Ok(reason_);
  }

 private:
  void invalidateAsyncRelocalization() {
    ++async_relocalization_sequence_;
  }

  void abandonAsyncRelocalization() {
    invalidateAsyncRelocalization();
    async_relocalization_job_.reset();
  }

  void advanceRelocalizationMapEpoch() {
    ++relocalization_map_epoch_;
    abandonAsyncRelocalization();
  }

  Status failRelocalization(
      const std::string& relocalization_state,
      const std::string& message,
      bool preserve_tracking,
      const std::string& reason_before_relocalization) {
    last_relocalization_message_ = message;
    if (catastrophic_health_fault_latched_) {
      relocalization_state_ = relocalization_state;
      enforceCatastrophicHealthFault();
      return Status::Error(message);
    }
    relocalization_state_ = preserve_tracking ? "tracking" : relocalization_state;
    if (preserve_tracking) {
      reason_ = reason_before_relocalization.empty() ? "tracking" : reason_before_relocalization;
    } else {
      state_ = SlamState::Lost;
      reason_ = message;
    }
    return Status::Error(message);
  }

  Status applyRelocalizationResult(
      const NativeRelocalizationResult& result,
      bool map_alignment_update,
      bool preserve_tracking_on_failure,
      const std::string& reason_before_relocalization) {
    relocalization_quality_ = result.quality;
    last_relocalization_message_ = result.message;
    relocalization_refine_backend_ = result.refine_backend;
    relocalization_refine_iterations_ = result.refine_iterations;
    relocalization_refine_inliers_ = result.refine_inliers;
    relocalization_refine_input_points_ = result.input_points;
    relocalization_refine_evaluated_points_ = result.evaluated_points;
    relocalization_refine_support_ratio_ = result.support_ratio;
    relocalization_refine_overlap_inlier_ratio_ =
        result.overlap_inlier_ratio;
    relocalization_refine_converged_ = result.refine_converged;
    relocalization_refine_pos_cov_trace_ = result.refine_pos_cov_trace;
    if (!result.success) {
      return failRelocalization(
          "failed",
          result.message,
          preserve_tracking_on_failure,
          reason_before_relocalization);
    }
    if (!poseInsideMapBounds(result.map_body)) {
      return failRelocalization(
          "rejected",
          "relocalization_outside_map_bounds",
          preserve_tracking_on_failure,
          reason_before_relocalization);
    }

    RelocalizationGateConfig gate_config;
    gate_config.max_fitness = runtime_config_.relocalization_max_fitness;
    gate_config.min_inliers = runtime_config_.relocalization_min_inliers;
    gate_config.min_evaluated_points =
        runtime_config_.relocalization_min_evaluated_points;
    gate_config.max_pos_cov_trace = runtime_config_.relocalization_max_pos_cov_trace;
    gate_config.max_alignment_translation_m =
        runtime_config_.track_against_map_max_translation_m;
    gate_config.max_alignment_yaw_rad =
        runtime_config_.track_against_map_max_yaw_deg * 3.14159265358979323846 / 180.0;
    gate_config.max_alignment_tilt_rad =
        runtime_config_.track_against_map_max_tilt_deg * 3.14159265358979323846 / 180.0;
    gate_config.require_alignment_degeneracy_metrics =
        runtime_config_.track_against_map_require_degeneracy_metrics;
    RelocalizationGateInput gate_input;
    gate_input.converged = result.refine_converged;
    gate_input.fitness = result.quality;
    gate_input.inliers = result.refine_inliers;
    gate_input.evaluated_points = result.evaluated_points;
    gate_input.pos_cov_trace = result.refine_pos_cov_trace;
    gate_input.alignment_update = map_alignment_update;
    gate_input.current_map_odom = map_odom_pose_;
    gate_input.candidate_map_odom = result.map_odom;
    const RelocalizationGateDecision gate =
        EvaluateRelocalizationGate(gate_config, gate_input);
    if (!gate.accepted) {
      return failRelocalization(
          "rejected", gate.reason, preserve_tracking_on_failure, reason_before_relocalization);
    }

    const std::string numeric_fault = fastLioNumericHealthFaultReason();
    if (!numeric_fault.empty()) {
      setFastLioHealthFault(SlamState::Lost, numeric_fault, true);
      relocalization_state_ = "rejected";
      last_relocalization_message_ = "fastlio_numeric_health_unrecovered";
      return Status::Error(last_relocalization_message_);
    }

    // Only the owning runtime thread reaches this commit. Fast-LIO odometry
    // stays continuous while the independently computed map alignment changes.
    constexpr double kPathInvalidatingTranslationM = 0.05;
    constexpr double kPathInvalidatingYawRad = 0.017453292519943295;
    const bool path_invalidating_jump =
        !has_map_odom_pose_ || !map_alignment_update ||
        gate.correction_translation_m > kPathInvalidatingTranslationM ||
        gate.correction_yaw_rad > kPathInvalidatingYawRad;
    state_estimation_at_scan_ = odometry_odom_body_;
    map_odom_pose_ = result.map_odom;
    map_body_pose_at_relocalization_ = result.map_body;
    has_map_odom_pose_ = true;
    map_frame_jump_ = path_invalidating_jump;
    if (path_invalidating_jump) {
      ++map_frame_jump_sequence_;
      // Map observations accumulated before this alignment were transformed
      // through the old map<-odom relation. Advance the published epoch so
      // mapd drops those mixed-frame layers before accepting the next scan.
      ++source_epoch_;
    }
    confidence_ = result.quality >= 0.0
        ? std::max(0.0, std::min(1.0, 1.0 - result.quality))
        : confidence_;
    localization_quality_ = confidence_;
    fastlio_health_fault_active_ = false;
    catastrophic_health_fault_latched_ = false;
    catastrophic_health_fault_reason_.clear();
    relocalization_required_after_time_jump_ = false;
    state_ = SlamState::Tracking;
    relocalization_state_ = "completed";
    reason_ = "relocalized";
    return Status::Ok(reason_);
  }

  bool isSensorTimeJump(double previous_s, double current_s) const {
    return runtime_config_.max_sensor_time_jump_s > 0.0 && previous_s >= 0.0 &&
           current_s - previous_s > runtime_config_.max_sensor_time_jump_s;
  }

  bool velocityOutOfBounds(const V3D& velocity) const {
    return builder_config_.max_update_velocity_mps > 0.0 &&
           velocity.norm() > builder_config_.max_update_velocity_mps;
  }

  std::string fastLioNumericHealthFaultReason() const {
    if (!kf_) {
      return "fastlio_state_unavailable";
    }
    const auto& state = kf_->x();
    if (!state.r_wi.allFinite() || !state.t_wi.allFinite() ||
        !state.r_il.allFinite() || !state.t_il.allFinite() ||
        !state.v.allFinite() || !state.bg.allFinite() ||
        !state.ba.allFinite() || !state.g.allFinite()) {
      return "fastlio_state_nonfinite";
    }
    const Pose3d pose =
        navigationBodyPoseFromImuPose(poseFromState(state), runtime_config_);
    if (!validPose(pose)) {
      return "fastlio_state_nonfinite";
    }
    if (velocityOutOfBounds(state.v)) {
      return "fastlio_velocity_out_of_bounds";
    }
    const auto& covariance = kf_->P();
    if (!covariance.allFinite()) {
      return "fastlio_covariance_nonfinite";
    }
    const double pos_cov_trace = covariance.diagonal().segment<3>(3).sum();
    if (!finite(pos_cov_trace) || pos_cov_trace < 0.0) {
      return "fastlio_covariance_nonfinite";
    }
    if (runtime_config_.fastlio_max_pos_cov_trace > 0.0 &&
        pos_cov_trace > runtime_config_.fastlio_max_pos_cov_trace) {
      return "fastlio_position_covariance_out_of_bounds";
    }
    return {};
  }

  void enforceCatastrophicHealthFault() {
    fastlio_health_fault_active_ = true;
    state_ = SlamState::Lost;
    confidence_ = 0.0;
    localization_quality_ = 0.0;
    reason_ = catastrophic_health_fault_reason_.empty()
        ? "fastlio_catastrophic_health_fault"
        : catastrophic_health_fault_reason_;
  }

  void setFastLioHealthFault(
      SlamState state,
      const std::string& reason,
      bool catastrophic = false) {
    if (catastrophic && !catastrophic_health_fault_latched_) {
      catastrophic_health_fault_latched_ = true;
      catastrophic_health_fault_reason_ = reason;
    }
    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return;
    }
    fastlio_health_fault_active_ = true;
    state_ = state;
    confidence_ = 0.0;
    localization_quality_ = 0.0;
    reason_ = reason;
  }

  void handleSensorTimeJump(double new_timestamp_s) {
    if (mode_ == SlamMode::Mapping) {
      reset();
      last_stamp_s_ = new_timestamp_s;
      state_ = SlamState::Initializing;
      confidence_ = 0.0;
      localization_quality_ = 0.0;
      reason_ = "sensor_time_jump_reset_mapping";
      return;
    }

    advanceRelocalizationMapEpoch();
    const bool preserve_catastrophic_fault = catastrophic_health_fault_latched_;
    const std::string preserved_catastrophic_reason = catastrophic_health_fault_reason_;
    const auto saved_relocalizer = relocalizer_;
    resetCore();
    relocalizer_ = saved_relocalizer;
    last_stamp_s_ = new_timestamp_s;
    has_map_odom_pose_ = false;
    map_body_pose_at_relocalization_.reset();
    map_frame_jump_ = false;
    confidence_ = 0.0;
    localization_quality_ = 0.0;
    relocalization_state_ = "required";
    last_relocalization_message_ = "sensor_time_jump_relocalization_required";
    relocalization_required_after_time_jump_ = true;
    if (preserve_catastrophic_fault) {
      catastrophic_health_fault_latched_ = true;
      catastrophic_health_fault_reason_ = preserved_catastrophic_reason;
      enforceCatastrophicHealthFault();
      return;
    }
    state_ = SlamState::Lost;
    reason_ = "sensor_time_jump_relocalization_required";
  }

  void resetCore() {
    ++source_epoch_;
    kf_ = std::make_shared<IESKF>();
    builder_ = std::make_unique<MapBuilder>(builder_config_, kf_);
    relocalizer_ = std::make_shared<NativeRelocalizer>();
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
    observation_sequence_ = 0U;
    map_cloud_map_.reset();
    saved_map_cloud_map_.reset();
    pose_history_.clear();
    patch_history_.clear();
    patch_history_dropped_count_ = 0;
    latest_odom_prior_.reset();
    odom_prior_buffer_.clear();
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    odom_prior_error_xy_m_ = -1.0;
    odom_prior_map_.clear();
    patch_sequence_ = 0;
    last_patch_stamp_s_ = 0.0;
    has_last_patch_pose_ = false;
    fastlio_health_fault_active_ = false;
    catastrophic_health_fault_latched_ = false;
    catastrophic_health_fault_reason_.clear();
    relocalization_required_after_time_jump_ = false;
  }

  void resetTrackingCoreAtPose(const Pose3d& pose) {
    kf_ = std::make_shared<IESKF>();
    builder_ = std::make_unique<MapBuilder>(builder_config_, kf_);
    applyPose(kf_->x(), imuPoseFromNavigationBodyPose(pose, runtime_config_));
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
    patch_history_dropped_count_ = 0;
    latest_odom_prior_.reset();
    odom_prior_buffer_.clear();
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    odom_prior_error_xy_m_ = -1.0;
    odom_prior_map_.clear();
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

  Status processOdomPriorPackage(const OdomSample& odom_prior) {
    if (!validPose(odom_prior.odom_body)) {
      setFastLioHealthFault(SlamState::Lost, "fastlio_state_nonfinite", true);
      return Status::Ok(reason_);
    }
    if (odom_prior.has_velocity) {
      const V3D prior_velocity(odom_prior.vx, odom_prior.vy, odom_prior.vz);
      if (!prior_velocity.allFinite()) {
        setFastLioHealthFault(SlamState::Lost, "fastlio_state_nonfinite", true);
        return Status::Ok(reason_);
      }
      if (velocityOutOfBounds(prior_velocity)) {
        setFastLioHealthFault(SlamState::Lost, "fastlio_velocity_out_of_bounds", true);
        return Status::Ok(reason_);
      }
    }

    last_stamp_s_ = package_.cloud_end_time;
    odom_prior_error_xy_m_ = -1.0;
    applyPose(
        kf_->x(),
        imuPoseFromNavigationBodyPose(odom_prior.odom_body, runtime_config_));
    if (odom_prior.has_velocity) {
      kf_->x().v = V3D(odom_prior.vx, odom_prior.vy, odom_prior.vz);
    } else {
      kf_->x().v.setZero();
    }
    odometry_odom_body_ = odom_prior.odom_body;
    state_estimation_at_scan_ = odom_prior.odom_body;
    pose_history_.push_back(OdomSample{package_.cloud_end_time, odom_prior.odom_body});
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }

    auto body_cloud = transformLidarCloudToNavigationBody(
        package_.cloud, builder_config_, runtime_config_);
    registered_cloud_body_ =
        toContractCloud(body_cloud, package_.cloud_end_time, config_.body_frame);
    ++observation_sequence_;
    auto world_cloud = transformLidarCloudWithBodyPose(
        package_.cloud,
        odom_prior.odom_body,
        builder_config_,
        runtime_config_);
    map_cloud_map_ =
        toContractCloud(world_cloud, package_.cloud_end_time, config_.map_frame);
    addOdomPriorMapCloud(world_cloud);
    recordPatchSnapshot();

    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return Status::Ok(reason_);
    }
    if (relocalization_required_after_time_jump_) {
      state_ = SlamState::Lost;
      confidence_ = 0.0;
      localization_quality_ = 0.0;
      reason_ = "sensor_time_jump_relocalization_required";
      return Status::Ok(reason_);
    }
    fastlio_health_fault_active_ = false;
    state_ = SlamState::Tracking;
    confidence_ = 1.0;
    localization_quality_ = 1.0;
    reason_ = "tracking_with_odom_prior_bypass";
    return Status::Ok(reason_);
  }

  void updateWaitingReason() {
    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return;
    }
    if (fastlio_health_fault_active_ || relocalization_required_after_time_jump_) {
      return;
    }
    if (runtime_config_.odom_prior_bypass_fastlio &&
        odom_prior_active_ &&
        state_ == SlamState::Tracking &&
        odometry_odom_body_.has_value() &&
        map_cloud_map_.has_value()) {
      confidence_ = 1.0;
      localization_quality_ = 1.0;
      reason_ = "tracking_with_odom_prior_bypass";
      return;
    }
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
    const double min_interval_s = std::max(0.0, runtime_config_.patch_min_interval_s);
    const double min_translation_m = std::max(0.0, runtime_config_.patch_min_translation_m);
    const double min_rotation_rad = std::max(0.0, runtime_config_.patch_min_rotation_rad);
    const bool time_ready =
        last_patch_stamp_s_ <= 0.0 || (stamp - last_patch_stamp_s_) >= min_interval_s;
    const bool motion_ready =
        !has_last_patch_pose_ || planarDistance(pose, last_patch_pose_) >= min_translation_m;
    const bool rotation_ready =
        !has_last_patch_pose_ || yawDistance(pose, last_patch_pose_) >= min_rotation_rad;
    if (!patch_history_.empty()) {
      if (!time_ready) {
        return;
      }
      if (!motion_ready && !rotation_ready) {
        return;
      }
    }

    PatchSnapshot patch;
    patch.sequence = patch_sequence_++;
    patch.name = patchName(patch.sequence);
    patch.stamp_s = stamp;
    patch.pose = pose;
    patch.cloud = *registered_cloud_body_;
    patch_history_.push_back(std::move(patch));
    const std::size_t max_snapshots =
        std::max<std::size_t>(1, runtime_config_.max_patch_snapshots);
    while (patch_history_.size() > max_snapshots) {
      patch_history_.pop_front();
      ++patch_history_dropped_count_;
    }
    last_patch_stamp_s_ = stamp;
    last_patch_pose_ = pose;
    has_last_patch_pose_ = true;
  }

  std::optional<OdomSample> freshOdomPrior(double stamp_s) {
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    if (!runtime_config_.odom_prior_enabled || odom_prior_buffer_.empty()) {
      return std::nullopt;
    }
    auto after = std::lower_bound(
        odom_prior_buffer_.begin(),
        odom_prior_buffer_.end(),
        stamp_s,
        [](const OdomSample& buffered, double requested_stamp_s) {
          return buffered.stamp_s < requested_stamp_s;
        });
    auto selected = after;
    if (after == odom_prior_buffer_.end()) {
      selected = std::prev(odom_prior_buffer_.end());
    } else if (after != odom_prior_buffer_.begin()) {
      const auto before = std::prev(after);
      if (std::abs(before->stamp_s - stamp_s) <=
          std::abs(after->stamp_s - stamp_s)) {
        selected = before;
      }
    }
    const double age_s = std::abs(stamp_s - selected->stamp_s);
    odom_prior_age_s_ = age_s;
    if (runtime_config_.odom_prior_max_age_s >= 0.0 &&
        age_s > runtime_config_.odom_prior_max_age_s) {
      return std::nullopt;
    }
    odom_prior_active_ = true;
    return *selected;
  }

  void addOdomPriorMapCloud(const CloudType::Ptr& world_cloud) {
    if (!world_cloud) {
      return;
    }
    const double resolution = runtime_config_.odom_prior_map_resolution > 0.0
        ? runtime_config_.odom_prior_map_resolution
        : builder_config_.map_resolution;
    for (const auto& point : world_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      odom_prior_map_[voxelKeyForPoint(point, resolution)] = point;
    }
  }

  CloudType odomPriorMapPclCloud() const {
    CloudType cloud;
    cloud.reserve(odom_prior_map_.size());
    for (const auto& item : odom_prior_map_) {
      cloud.push_back(item.second);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
    return cloud;
  }

  Cloud odomPriorMapContractCloud(double stamp_s) const {
    CloudType::Ptr cloud(new CloudType(odomPriorMapPclCloud()));
    return toContractCloud(cloud, stamp_s, config_.map_frame);
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
  std::shared_ptr<NativeRelocalizer> relocalizer_;
  std::optional<AsyncRelocalizationJob> async_relocalization_job_;
  std::uint64_t async_relocalization_sequence_ = 0U;
  std::uint64_t relocalization_map_epoch_ = 0U;
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
  bool fastlio_health_fault_active_ = false;
  bool catastrophic_health_fault_latched_ = false;
  std::string catastrophic_health_fault_reason_;
  bool relocalization_required_after_time_jump_ = false;
  std::uint64_t map_frame_jump_sequence_ = 0U;
  bool has_map_odom_pose_ = false;
  Pose3d map_odom_pose_;
  std::optional<Pose3d> map_body_pose_at_relocalization_;
  double confidence_ = 0.0;
  double localization_quality_ = 0.0;
  double relocalization_quality_ = -1.0;
  std::string relocalization_refine_backend_;
  int relocalization_refine_iterations_ = -1;
  int relocalization_refine_inliers_ = -1;
  int relocalization_refine_input_points_ = 0;
  int relocalization_refine_evaluated_points_ = 0;
  double relocalization_refine_support_ratio_ = -1.0;
  double relocalization_refine_overlap_inlier_ratio_ = -1.0;
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
  std::uint64_t observation_sequence_ = 0U;
  std::uint64_t source_epoch_ = newSourceEpoch() - 1U;
  std::optional<Cloud> map_cloud_map_;
  std::optional<Cloud> saved_map_cloud_map_;
  int saved_map_points_ = 0;
  std::optional<OdomSample> latest_odom_prior_;
  std::deque<OdomSample> odom_prior_buffer_;
  bool odom_prior_active_ = false;
  double odom_prior_age_s_ = -1.0;
  double odom_prior_error_xy_m_ = -1.0;
  std::unordered_map<VoxelKey, PointType, VoxelKeyHash> odom_prior_map_;
  GnssFusionHealth gnss_health_;
  std::vector<OdomSample> pose_history_;
  std::deque<PatchSnapshot> patch_history_;
  std::uint64_t patch_history_dropped_count_ = 0;
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
