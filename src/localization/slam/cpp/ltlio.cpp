#include "slam.hpp"

#if defined(LINGTU_HAS_LTLIO_BACKEND) && LINGTU_HAS_LTLIO_BACKEND

#include "ltlio/frontend.hpp"
#include "profile.hpp"
#include "native_relocalizer.hpp"
#include "relocalization_gate.hpp"
#include "localization/online_mapping/worker.hpp"
#include "localization/map_optimization/pose_math.hpp"
#include "localization/online_mapping/config_yaml.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
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
namespace map_optimization = lingtu::localization::map_optimization;
namespace online_mapping = lingtu::localization::online_mapping;
namespace lio = lingtu::ltlio;
namespace slam_defaults = lingtu::settings::slam::slam;
using Config = lio::Config;
using CloudType = lio::Cloud;
using PointType = pcl::PointXYZINormal;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

// External-pose replay uses the same scan input and publication contract,
// without constructing a second estimator.
struct BufferedImu {
  V3D gyro;
  double time;
};
struct SyncedScan {
  std::vector<BufferedImu> imus;
  CloudType::Ptr cloud;
  double cloud_start_time = 0;
  double cloud_end_time = 0;
};

map_optimization::Pose graphPose(const Pose3d& p) { return {p.x, p.y, p.z, p.qw, p.qx, p.qy, p.qz}; }
Pose3d slamPose(const map_optimization::Pose& p) { return {p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw}; }

struct RuntimeConfig {
  std::string elevator_mode = slam_defaults::elevator_mode;
  double max_update_velocity_mps = slam_defaults::max_update_velocity_mps;
  online_mapping::Config online_mapping;
  double time_diff_lidar_to_imu = 0.0;
  double max_imu_gap_s = slam_defaults::max_imu_gap_s;
  double max_sensor_time_jump_s = slam_defaults::max_sensor_time_jump_s;
  double relocalization_max_fitness = slam_defaults::relocalization_max_fitness;
  int relocalization_min_inliers = slam_defaults::relocalization_min_inliers;
  int relocalization_min_evaluated_points = slam_defaults::relocalization_min_evaluated_points;
  double relocalization_max_pos_cov_trace = slam_defaults::relocalization_max_pos_cov_trace;
  // A 100 m^2 position-covariance trace means aggregate 1-sigma uncertainty
  // has reached 10 m. At that point local odometry is not safe to publish or
  // to revive through a map-only relocalization result.
  double ltlio_max_pos_cov_trace = slam_defaults::ltlio_max_pos_cov_trace;
  double relocalization_map_bounds_margin_m = slam_defaults::relocalization_map_bounds_margin_m;
  double track_against_map_max_translation_m = slam_defaults::track_against_map_max_translation_m;
  double track_against_map_max_yaw_deg = slam_defaults::track_against_map_max_yaw_deg;
  double track_against_map_max_tilt_deg = slam_defaults::track_against_map_max_tilt_deg;
  bool track_against_map_require_degeneracy_metrics = slam_defaults::track_against_map_require_degeneracy_metrics;
  bool odom_prior_enabled = slam_defaults::odom_prior_enabled;
  bool odom_prior_bypass_ltlio = slam_defaults::odom_prior_bypass_ltlio;
  double odom_prior_max_age_s = slam_defaults::odom_prior_max_age_s;
  double odom_prior_map_resolution = slam_defaults::odom_prior_map_resolution;
  double odom_prior_snap_translation_m = slam_defaults::odom_prior_snap_translation_m;
  std::size_t max_odom_prior_buffer = slam_defaults::max_odom_prior_buffer;
  std::size_t max_patch_snapshots = slam_defaults::max_patch_snapshots;
  // Keep bounded recording/save memory by default. Explicit zero enables
  // spatial-only sampling, whose full long-map save still needs qualification.
  double patch_min_interval_s = slam_defaults::patch_min_interval_s;
  double patch_min_translation_m = slam_defaults::patch_min_translation_m;
  double patch_min_rotation_rad = slam_defaults::patch_min_rotation_rad;
  std::size_t max_imu_buffer = slam_defaults::max_imu_buffer;
  std::size_t max_lidar_buffer = slam_defaults::max_lidar_buffer;
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
  std::uint64_t observation_sequence = 0;
  double stamp_s = 0.0;
  Pose3d pose;
  Cloud cloud;
};

struct SavedPatch {
  std::shared_ptr<const PatchSnapshot> source;
  Pose3d pose;
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
    Config& frontend_config,
    RuntimeConfig& runtime_config) {
  if (path.empty()) {
    return Status::Ok("default_ltlio_config");
  }
  if (!std::filesystem::exists(path)) {
    return Status::Error("ltlio_config_missing: " + path);
  }

  YAML::Node config;
  try {
    config = readProfile(path);
  } catch (const std::exception& exc) {
    return Status::Error(std::string("ltlio_config_parse_failed: ") + exc.what());
  }
  if (!config) {
    return Status::Error("ltlio_config_empty: " + path);
  }

  if (config["estimator"])
    return Status::Error("estimator_selection_removed: LTLIO has one core");
  readIfPresent(config, "body_frame", slam_config.body_frame);
  try {
    runtime_config.online_mapping = online_mapping::readConfig(config["online_mapping"]);
  } catch (const std::exception& error) {
    return Status::Error(std::string("invalid_online_mapping_config: ") + error.what());
  }
  readIfPresent(config, "world_frame", slam_config.odom_frame);
  readIfPresent(config, "acc_scale", frontend_config.acc_scale);
  // Kalibr densities are in SI units, independently of raw IMU acc_scale.
  // The continuous-time covariance propagation consumes density squared.
  for (const auto& [key, variance] : {
           std::pair{"na", &frontend_config.acc_noise_variance},
           std::pair{"ng", &frontend_config.gyro_noise_variance},
           std::pair{"nba", &frontend_config.acc_bias_noise_variance},
           std::pair{"nbg", &frontend_config.gyro_bias_noise_variance}}) {
    if (!config[key]) continue;
    const double density = config[key].as<double>();
    if (!finite(density) || density < 0.0 || !finite(density * density))
      return Status::Error(std::string("invalid_imu_noise_density: ") + key);
    *variance = density * density;
  }
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
      "ltlio_max_pos_cov_trace",
      runtime_config.ltlio_max_pos_cov_trace);
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
      "odom_prior_bypass_ltlio",
      runtime_config.odom_prior_bypass_ltlio);
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

  readIfPresent(config, "lidar_filter_num", frontend_config.point_filter_num);
  readIfPresent(config, "lidar_min_range", frontend_config.blind_m);
  readIfPresent(config, "lidar_max_range", frontend_config.max_range_m);
  readIfPresent(config, "scan_resolution", frontend_config.scan_resolution);
  readIfPresent(config, "scale_aware_sampling_enabled", frontend_config.scale_aware_sampling_enabled);
  readIfPresent(config, "hybrid_enabled", frontend_config.hybrid_enabled);
  readIfPresent(config, "correspondence_pruning_enabled", frontend_config.correspondence_pruning_enabled);
  readIfPresent(config, "hybrid_point_weight", frontend_config.hybrid_point_weight);
  readIfPresent(config, "hybrid_point_initial_threshold_m", frontend_config.hybrid_point_initial_threshold_m);
  readIfPresent(config, "map_resolution", frontend_config.map_resolution);
  readIfPresent(config, "cube_len", frontend_config.cube_len);
  readIfPresent(config, "det_range", frontend_config.det_range);
  readIfPresent(config, "move_thresh", frontend_config.move_thresh);
  readIfPresent(config, "max_map_points", frontend_config.max_map_points);
  readIfPresent(config, "stationary_thresh", frontend_config.stationary_thresh);
  readIfPresent(config, "imu_init_num", frontend_config.imu_init_samples);
  readIfPresent(config, "near_search_num", frontend_config.near_search_num);
  readIfPresent(config, "ieskf_max_iter", frontend_config.ieskf_max_iter);
  readIfPresent(config, "max_update_translation_m", frontend_config.max_update_translation_m);
  readIfPresent(config, "max_update_rotation_rad", frontend_config.max_update_rotation_rad);
  readIfPresent(config, "max_update_velocity_mps", runtime_config.max_update_velocity_mps);
  readIfPresent(config, "max_update_velocity_delta_mps", frontend_config.max_update_velocity_delta_mps);
  readIfPresent(config, "esti_il", frontend_config.estimate_extrinsics);
  readIfPresent(config, "lidar_cov_inv", frontend_config.lidar_cov_inv);
  readIfPresent(config, "imu_static_acc_thresh", frontend_config.imu_static_acc_thresh);
  readIfPresent(config, "imu_static_gyro_thresh", frontend_config.imu_static_gyro_thresh);
  readIfPresent(config, "zupt_min_static_frames", frontend_config.zupt_min_static_frames);
  readIfPresent(config, "zupt_sigma_v", frontend_config.zupt_sigma_v);
  readIfPresent(config, "elevator_mode", runtime_config.elevator_mode);
  if (const auto entry = config["elevator_entry"]) {
    readIfPresent(entry, "range_m", frontend_config.elevator_entry_range_m);
    readIfPresent(entry, "hold_s", frontend_config.elevator_entry_hold_s);
    readIfPresent(entry, "motion_m", frontend_config.elevator_entry_motion_m);
  }
  if (runtime_config.elevator_mode != "off" && runtime_config.elevator_mode != "auto" &&
      runtime_config.elevator_mode != "manual") return Status::Error("invalid_elevator_mode");

  if (config["t_il"]) {
    const auto t = config["t_il"].as<std::vector<double>>();
    if (t.size() != 3) {
      return Status::Error("ltlio_t_il_must_have_3_values");
    }
    frontend_config.imu_from_lidar_translation << t[0], t[1], t[2];
  }
  if (config["r_il"]) {
    const auto r = config["r_il"].as<std::vector<double>>();
    if (r.size() != 9) {
      return Status::Error("ltlio_r_il_must_have_9_values");
    }
    frontend_config.imu_from_lidar_rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }
  if (config["navigation_body_from_imu_translation"]) {
    const auto t = config["navigation_body_from_imu_translation"].as<std::vector<double>>();
    if (t.size() != 3) {
      return Status::Error("ltlio_navigation_body_from_imu_translation_must_have_3_values");
    }
    runtime_config.navigation_body_from_imu_translation << t[0], t[1], t[2];
  }
  if (config["navigation_body_from_imu_rotation"]) {
    const auto r = config["navigation_body_from_imu_rotation"].as<std::vector<double>>();
    if (r.size() != 9) {
      return Status::Error("ltlio_navigation_body_from_imu_rotation_must_have_9_values");
    }
    runtime_config.navigation_body_from_imu_rotation <<
        r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }

  if (frontend_config.estimate_extrinsics)
    return Status::Error("online_extrinsic_estimation_removed: use calibrated RobotConfig extrinsics");
  if (runtime_config.odom_prior_bypass_ltlio && !runtime_config.odom_prior_enabled)
    return Status::Error("odom_prior_bypass_requires_enabled_input");
  return Status::Ok("ltlio_config_loaded");
}

bool validPoint(const PointXYZIT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

lio::TimedCloud toNativeCloud(const LidarFrame& frame) {
  lio::TimedCloud cloud;
  cloud.stamp_s = frame.stamp_s;
  cloud.end_stamp_s = frame.stamp_s +
      static_cast<double>(std::max<std::int64_t>(0, frame.points.back().offset_time_ns)) / 1e9;
  cloud.points.reserve(frame.points.size());
  for (const auto& point : frame.points) {
    const bool valid = validPoint(point) && point.offset_time_ns >= 0;
    cloud.points.push_back({valid ? point.x : std::numeric_limits<float>::quiet_NaN(),
        point.y, point.z, point.intensity,
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, point.offset_time_ns)),
        point.line, point.tag});
  }
  return cloud;
}

CloudType::Ptr transformCloud(const CloudType::ConstPtr& cloud,
                             const M3D& rotation, const V3D& translation) {
  auto result = cloud->makeShared();
  for (auto& point : result->points) {
    const V3D transformed = rotation * V3D(point.x, point.y, point.z) + translation;
    point.x = static_cast<float>(transformed.x());
    point.y = static_cast<float>(transformed.y());
    point.z = static_cast<float>(transformed.z());
  }
  return result;
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



CloudType::Ptr transformLidarCloudToNavigationBody(
    const CloudType::ConstPtr& cloud,
    const Config& config,
    const RuntimeConfig& runtime_config) {
  const Eigen::Matrix3d r_bi = runtime_config.navigation_body_from_imu_rotation;
  const Eigen::Vector3d t_bi = runtime_config.navigation_body_from_imu_translation;
  const Eigen::Matrix3d r_bl = r_bi * config.imu_from_lidar_rotation;
  const Eigen::Vector3d t_bl = t_bi + r_bi * config.imu_from_lidar_translation;
  return transformCloud(cloud, r_bl, t_bl);
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
    const CloudType::ConstPtr& cloud,
    const Pose3d& body_pose,
    const Config& config,
    const RuntimeConfig& runtime_config) {
  const Eigen::Matrix3d r_wb = rotationFromPose(body_pose);
  const Eigen::Vector3d t_wb(body_pose.x, body_pose.y, body_pose.z);
  const Eigen::Matrix3d r_bi = runtime_config.navigation_body_from_imu_rotation;
  const Eigen::Vector3d t_bi = runtime_config.navigation_body_from_imu_translation;
  const Eigen::Matrix3d r_bl = r_bi * config.imu_from_lidar_rotation;
  const Eigen::Vector3d t_bl = t_bi + r_bi * config.imu_from_lidar_translation;
  const Eigen::Matrix3d r_wl = r_wb * r_bl;
  const Eigen::Vector3d t_wl = t_wb + r_wb * t_bl;
  return transformCloud(cloud, r_wl, t_wl);
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
    const std::vector<OdomSample>& pose_history,
    const char* filename = "trajectory.txt") {
  std::ofstream out(map_dir / filename);
  if (!out) {
    return Status::Error("open_trajectory_txt_failed");
  }
  out << std::setprecision(12);
  for (const auto& sample : pose_history) {
    const auto& p = sample.odom_body;
    out << sample.stamp_s << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' '
        << p.qx << ' ' << p.qy << ' ' << p.qz << ' ' << p.qw << '\n';
  }
  out.flush();
  if (!out) return Status::Error("write_trajectory_txt_failed");
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
  out.flush();
  if (!out) return Status::Error("write_patch_pcd_failed");
  out.close();
  if (!out) return Status::Error("close_patch_pcd_failed");
  return Status::Ok("patch_pcd_written");
}

Status writePatchIndex(
    const std::filesystem::path& map_dir,
    const std::vector<SavedPatch>& patches,
    const char* filename = "poses.txt") {
  std::ofstream out(map_dir / filename);
  if (!out) {
    return Status::Error("open_poses_txt_failed");
  }
  out << std::setprecision(12);
  for (const auto& patch : patches) {
    const auto& pose = patch.pose;
    out << patch.source->name << ' ' << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
        << pose.qw << ' ' << pose.qx << ' ' << pose.qy << ' ' << pose.qz << '\n';
  }
  out.flush();
  if (!out) return Status::Error("write_poses_txt_failed");
  return Status::Ok("poses_txt_written");
}

Status writePatchBundle(
    const std::filesystem::path& map_dir,
    const std::vector<SavedPatch>& patches,
    std::uint64_t dropped_count,
    const Eigen::Vector3d& lidar_origin_in_patch) {
  if (patches.empty()) {
    return Status::Ok("no_patches");
  }
  for (const auto& patch : patches) {
    if (patch.source->cloud.points.empty()) {
      continue;
    }
    const Status patch_status =
        writeContractPcdBinary(map_dir / "patches" / patch.source->name, patch.source->cloud);
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
           << "first_sequence " << patches.front().source->sequence << '\n'
           << "last_sequence " << patches.back().source->sequence << '\n'
           << "patch_count " << patches.size() << '\n';
  manifest.flush();
  if (!manifest) {
    return Status::Error("write_patch_bundle_manifest_failed");
  }
  std::ofstream calibration(map_dir / "scan_origin.txt");
  calibration << std::setprecision(12) << "lidar_origin_in_patch "
              << lidar_origin_in_patch.x() << ' ' << lidar_origin_in_patch.y()
              << ' ' << lidar_origin_in_patch.z() << '\n';
  calibration.flush();
  if (!calibration) return Status::Error("write_scan_origin_failed");
  return Status::Ok("patch_bundle_written");
}

struct SaveCapture {
  std::filesystem::path pcd;
  std::filesystem::path semantic_map;
  CloudType cloud;
  std::vector<OdomSample> trajectory;
  std::vector<SavedPatch> patches;
  std::shared_ptr<const online_mapping::Snapshot> correction;
  std::shared_ptr<const online_mapping::Snapshot> evidence;
  online_mapping::Config online_mapping_config;
  bool sam_snapshot = false;
  std::uint64_t dropped_patches = 0;
  Eigen::Vector3d lidar_origin;
  double stamp_s = 0.0;
  std::string frame_id;
};

struct TempDir {
  std::filesystem::path path;
  ~TempDir() {
    if (!path.empty()) {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  }
};

struct SaveComputeState {
  Status status = Status::Error("map_snapshot_in_progress");
  std::shared_ptr<Cloud> cloud;
  std::string pcd_path;
  TempDir staging;
  std::atomic<bool> ready{false};
};

Status writeCapturedCloud(SaveCapture& capture, SaveComputeState& result) {
  std::size_t count = capture.cloud.size();
  for (const auto& patch : capture.patches) count += patch.source->cloud.points.size();
  if (count > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    return Status::Error("map_point_count_exceeds_supported_range");
  result.cloud = std::make_shared<Cloud>();
  result.cloud->stamp_s = capture.stamp_s;
  result.cloud->frame_id = capture.frame_id;
  result.cloud->points.reserve(count);
  std::ofstream out(capture.pcd, std::ios::binary);
  if (!out) return Status::Error("open_map_pcd_failed");
  // Use PCL's field layout and header, omitting the same alignment padding as
  // savePCDFileBinary. Only the bounded write buffer uses the PCL point layout.
  out << pcl::PCDWriter::generateHeader<PointType>(capture.cloud, static_cast<int>(count))
      << "DATA binary\n";
  struct Field {
    std::size_t offset;
    std::size_t bytes;
  };
  std::vector<Field> fields;
  std::size_t point_bytes = 0;
  for (const auto& field : pcl::getFields<PointType>()) {
    if (field.name == "_") continue;
    const auto bytes = field.count * pcl::getFieldSize(field.datatype);
    fields.push_back({field.offset, bytes});
    point_bytes += bytes;
  }
  std::array<char, 65536> buffer;
  std::size_t used = 0;
  const auto append = [&](const PointType& point) {
    if (used + point_bytes > buffer.size()) {
      out.write(buffer.data(), static_cast<std::streamsize>(used));
      if (!out) return false;
      used = 0;
    }
    for (const auto& field : fields) {
      std::memcpy(buffer.data() + used,
                  reinterpret_cast<const char*>(&point) + field.offset, field.bytes);
      used += field.bytes;
    }
    PointXYZIT compact;
    compact.x = point.x;
    compact.y = point.y;
    compact.z = point.z;
    compact.intensity = point.intensity;
    compact.offset_time_ns = static_cast<std::int64_t>(std::llround(point.curvature * 1e6));
    result.cloud->points.push_back(compact);
    return true;
  };
  for (const auto& point : capture.cloud.points)
    if (!append(point)) return Status::Error("map_pcd_write_failed");
  for (const auto& patch : capture.patches) {
    const auto pose = graphPose(patch.pose);
    for (const auto& point : patch.source->cloud.points) {
      const auto xyz = map_optimization::rotate_vector(pose, point.x, point.y, point.z);
      PointType transformed;
      transformed.x = static_cast<float>(xyz[0] + pose.x);
      transformed.y = static_cast<float>(xyz[1] + pose.y);
      transformed.z = static_cast<float>(xyz[2] + pose.z);
      transformed.intensity = point.intensity;
      if (!append(transformed)) return Status::Error("map_pcd_write_failed");
    }
  }
  if (used) out.write(buffer.data(), static_cast<std::streamsize>(used));
  out.flush();
  if (!out) return Status::Error("map_pcd_write_failed");
  out.close();
  if (!out) return Status::Error("map_pcd_close_failed");
  capture.cloud = CloudType{};
  return Status::Ok("map_pcd_written");
}

Status writeCapturedMapFiles(SaveCapture& capture, SaveComputeState& result) {
  std::error_code ec;
  std::filesystem::create_directories(capture.pcd.parent_path(), ec);
  if (ec) return Status::Error("create_map_dir_failed: " + ec.message());

  const auto raw_trajectory = writeTrajectory(capture.pcd.parent_path(), capture.trajectory,
                                               "trajectory.raw.txt");
  if (!raw_trajectory.ok) return raw_trajectory;
  const auto raw_poses = writePatchIndex(capture.pcd.parent_path(), capture.patches, "poses.raw.txt");
  if (!raw_poses.ok) return raw_poses;
  std::ofstream timestamps(capture.pcd.parent_path() / "keyframes.timestamps.txt");
  timestamps << std::setprecision(17);
  for (const auto& patch : capture.patches)
    timestamps << patch.source->name << ' ' << patch.source->stamp_s << '\n';
  timestamps.flush();
  if (!timestamps) return Status::Error("write_keyframe_timestamps_failed");


  if (capture.correction) {
    capture.cloud.clear();
    for (std::size_t i = 0; i < capture.patches.size(); ++i) {
      const auto& pose = capture.correction->keyframes[i].pose;
      capture.patches[i].pose = slamPose(pose);
    }
  }
  // The localization tree is downsampled. Preserve recorded surface returns
  // even when pose-graph optimization did not run. Without a correction keep
  // its snapshot too, since scans after the last keyframe can add geometry.
  if (capture.correction && capture.patches.empty())
    return Status::Error("corrected_map_pcd_write_failed");
  if (capture.correction) {
    std::size_t anchor = 0;
    for (auto& sample : capture.trajectory) {
      while (anchor + 1 < capture.patches.size() &&
             capture.patches[anchor + 1].source->stamp_s <= sample.stamp_s)
        ++anchor;
      const auto correction = map_optimization::compose_pose(
          graphPose(capture.patches[anchor].pose),
          map_optimization::inverse_pose(graphPose(capture.patches[anchor].source->pose)));
      sample.odom_body = slamPose(map_optimization::compose_pose(correction, graphPose(sample.odom_body)));
    }
  }
  const auto cloud_status = writeCapturedCloud(capture, result);
  if (!cloud_status.ok) return cloud_status;

  const Status trajectory_status = writeTrajectory(capture.pcd.parent_path(), capture.trajectory);
  if (!trajectory_status.ok) return trajectory_status;
  std::filesystem::create_directories(capture.pcd.parent_path() / "patches", ec);
  if (ec) return Status::Error("create_patches_dir_failed: " + ec.message());
  const Status patch_status = writePatchBundle(
      capture.pcd.parent_path(), capture.patches, capture.dropped_patches,
      capture.lidar_origin);
  if (!patch_status.ok) return patch_status;
  if (capture.sam_snapshot) {
    std::ofstream loops(capture.pcd.parent_path() / "sam_loops.json");
    online_mapping::writeLoopEvidence(loops, capture.online_mapping_config,
        capture.evidence ? capture.evidence->loop_records
                         : std::vector<online_mapping::LoopResult>{});
    loops.flush();
    if (!loops) return Status::Error("write_sam_loop_evidence_failed");
    std::ofstream report(capture.pcd.parent_path() / "map_optimization.json");
    report << "{\"schema\":\"lingtu.map_optimization.v1\",\"backend\":\"lio_sam_isam2\","
           << "\"success\":true,\"performed\":" << (capture.correction ? "true" : "false")
           << ",\"code\":\"" << (capture.correction ? "online_graph_snapshot" : "lio_snapshot_without_loop_correction") << "\","
           << "\"pose_count\":" << capture.patches.size()
           << ",\"loop_count\":" << (capture.correction ? capture.correction->loop_constraints : 0)
           << ",\"evidence_pose_count\":" << (capture.evidence ? capture.evidence->keyframes.size() : 0)
           << ",\"evidence_complete\":" << (capture.evidence &&
               capture.evidence->keyframes.size()==capture.patches.size() ? "true" : "false")
           << ",\"worker_ms\":" << (capture.evidence ? capture.evidence->worker_ms : 0)
           << ",\"preview_ms\":" << (capture.evidence ? capture.evidence->preview_ms : 0)
           << ",\"cloud_bytes\":" << (capture.evidence ? capture.evidence->cloud_bytes : 0)
           << ",\"pending_high_water\":" << (capture.evidence ? capture.evidence->pending_high_water : 0) << "}\n";
    report.flush();
    if (!report) return Status::Error("write_online_graph_report_failed");
  }


  return Status::Ok("map_snapshot_staged");
}

Status stageCapturedMap(SaveCapture& capture, SaveComputeState& result) {
  const auto destination = capture.pcd;
  capture.semantic_map = destination.parent_path() / "semantic_map.bin";
  auto staging = destination;
  staging += ".tmp";
  std::error_code ec;
  std::filesystem::create_directories(destination.parent_path(), ec);
  if (ec) return Status::Error("create_map_dir_failed: " + ec.message());
  if (!std::filesystem::create_directory(staging, ec))
    return Status::Error("create_map_staging_failed: " +
                         (ec ? ec.message() : "staging_exists"));
  // Match map optimization's temporary-directory save pattern. Back up only
  // map-owned entries when replacing an existing bundle in a shared directory.
  TempDir cleanup{staging};
  capture.pcd = staging / destination.filename();
  const auto status = writeCapturedMapFiles(capture, result);
  if (!status.ok) return status;
  result.pcd_path = destination.string();
  result.staging.path = std::exchange(cleanup.path, {});
  return status;
}

Status commitCapturedMap(SaveComputeState& result) {
  std::error_code ec;
  const std::filesystem::path destination(result.pcd_path);
  const auto staging = std::exchange(result.staging.path, {});
  TempDir cleanup{staging};
  // The worker has completed. Only the owning poll may publish its files,
  // after checking that the captured source is still the current epoch.
  std::vector<std::filesystem::path> names;
  for (const auto& entry : std::filesystem::directory_iterator(staging))
    names.push_back(entry.path().filename());
  const auto previous = staging / ".previous";
  std::filesystem::create_directory(previous);
  std::vector<const std::filesystem::path*> backed_up;
  std::vector<const std::filesystem::path*> installed;
  backed_up.reserve(names.size());
  installed.reserve(names.size());
  try {
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto& name = names[i];
      const auto target = destination.parent_path() / name;
      if (std::filesystem::exists(target)) {
        std::filesystem::rename(target, previous / name);
        backed_up.push_back(&name);
      }
      std::filesystem::rename(staging / name, target);
      installed.push_back(&name);
    }
  } catch (const std::exception& error) {
    // Leave recovery files in place if the filesystem also refuses rollback.
    cleanup.path.clear();
    for (auto it = installed.rbegin(); it != installed.rend(); ++it) {
      std::filesystem::rename(destination.parent_path() / **it, staging / **it, ec);
      if (ec) return Status::Error("map_save_restore_failed: " + ec.message());
    }
    for (auto it = backed_up.rbegin(); it != backed_up.rend(); ++it) {
      std::filesystem::rename(previous / **it, destination.parent_path() / **it, ec);
      if (ec) return Status::Error("map_save_restore_failed: " + ec.message());
    }
    cleanup.path = staging;
    return Status::Error("map_save_commit_failed: " + std::string(error.what()));
  }
  return Status::Ok("map_snapshot_ready");
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

double yawFromPose(const Pose3d& pose) {
  const Eigen::Matrix3d rotation = rotationFromPose(pose);
  return std::atan2(rotation(1, 0), rotation(0, 0));
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

double rotationDistance(const Pose3d& a, const Pose3d& b) {
  const Eigen::Quaterniond qa(rotationFromPose(a));
  const Eigen::Quaterniond qb(rotationFromPose(b));
  return qa.angularDistance(qb);
}

bool pathInvalidatingCorrection(double translation_m, double yaw_rad) {
  constexpr double kPathInvalidatingTranslationM = 0.05;
  constexpr double kPathInvalidatingYawRad = 0.017453292519943295;
  return translation_m > kPathInvalidatingTranslationM ||
      std::abs(yaw_rad) > kPathInvalidatingYawRad;
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
  std::uint64_t elevator_entries = 0U;
  std::uint64_t elevator_exits = 0U;
  bool map_alignment_update = false;
  bool preserve_tracking_on_failure = false;
  std::shared_ptr<AsyncRelocalizationComputeState> compute;
};

struct AsyncSaveJob {
  std::uint64_t source_epoch = 0;
  std::uint64_t observation_sequence = 0;
  std::shared_ptr<SaveComputeState> compute;
};

class LtlioBackend final : public ISlamBackend {
 public:
  ~LtlioBackend() override { clearGeometryExports(); }

  std::optional<MapGeometrySnapshot> mapGeometrySnapshot() override {
    if (mode_ != SlamMode::Mapping) {
      clearGeometryExports();
      return std::nullopt;
    }
    if ((geometry_pending_ && geometry_pending_->reset_epoch != source_epoch_) ||
        (geometry_confirmed_ && geometry_confirmed_->reset_epoch != source_epoch_))
      clearGeometryExports();
    if (geometry_export_.valid() && geometry_export_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto completed = geometry_export_.get();
      if (completed.first.ok && completed.second.reset_epoch == source_epoch_) {
        geometry_pending_ = std::move(completed.second);
        geometry_pending_optimizations_ = geometry_export_optimizations_;
      } else {
        geometry_failed_export_optimizations_=geometry_export_optimizations_;
        std::fprintf(stderr,"map_geometry_export_failed epoch=%llu revision=%llu reason=%s\n",
            static_cast<unsigned long long>(completed.second.reset_epoch),
            static_cast<unsigned long long>(completed.second.graph_revision),completed.first.message.c_str());
        removeGeometryDirectory(completed.second.frozen_dir);
      }
    }
    if (!geometry_pending_ && !geometry_export_.valid() && global_mapping_ &&
        global_mapping_->optimizations > 0 && patch_history_dropped_count_ == 0 &&
        online_mapping_.dropped_frames() == 0 &&
        global_mapping_->optimizations > std::max(geometry_confirmed_optimizations_,geometry_failed_export_optimizations_)) {
      const auto& keyframes = global_mapping_->keyframes;
      bool complete = !keyframes.empty() && keyframes.size() <= patch_history_.size();
      for (std::size_t i = 0; complete && i < keyframes.size(); ++i)
        complete = patch_history_[i]->name == keyframes[i].patch_name;
      if (complete) {
        std::vector<SavedPatch> patches;
        for (std::size_t i = 0; i < keyframes.size(); ++i) {
          const auto map_pose = composePoses(has_map_lio_pose_ ? map_lio_pose_ : Pose3d{},
                                             slamPose(keyframes[i].pose));
          patches.push_back({patch_history_[i], map_pose});
        }
        if (geometry_root_.empty())
          geometry_root_ = std::filesystem::temp_directory_path() /
              ("lingtu-map-geometry-" + std::to_string(newSourceEpoch()));
        MapGeometrySnapshot descriptor{source_epoch_, global_mapping_->revision, patches.back().source->observation_sequence,
            config_.map_frame, (geometry_root_ / std::to_string(global_mapping_->revision)).string(),
            static_cast<std::uint64_t>(patches.size())};
        const Eigen::Vector3d origin = runtime_config_.navigation_body_from_imu_translation +
            runtime_config_.navigation_body_from_imu_rotation * frontend_config_.imu_from_lidar_translation;
        descriptor.captured_stamp_ns = static_cast<std::int64_t>(std::llround(patches.back().source->stamp_s*1e9));
        descriptor.map_body = patches.back().pose;
        const auto& anchor=descriptor.map_body;
        descriptor.sensor_origin_z_m = (rotationFromPose(anchor)*origin).z()+anchor.z;
        geometry_export_optimizations_ = global_mapping_->optimizations;
        geometry_export_ = std::async(std::launch::async,
            [descriptor, patches = std::move(patches), origin]() mutable {
          Status status;
          try {
            std::filesystem::create_directories(std::filesystem::path(descriptor.frozen_dir) / "patches");
            status = writePatchBundle(descriptor.frozen_dir, patches, 0U, origin);
          } catch (const std::exception& error) { status = Status::Error(error.what()); }
          return std::make_pair(std::move(status), std::move(descriptor));
        });
      }
    }
    return geometry_pending_ ? geometry_pending_ : geometry_confirmed_;
  }

  void acknowledgeMapGeometry(std::uint64_t epoch, std::uint64_t revision, bool success,
                              const std::string& message = {}) override {
    if (!geometry_pending_ || geometry_pending_->reset_epoch != epoch ||
        geometry_pending_->graph_revision != revision) return;
    if (!success) std::fprintf(stderr,"map_geometry_rejected epoch=%llu revision=%llu reason=%s\n",
        static_cast<unsigned long long>(epoch),static_cast<unsigned long long>(revision),message.c_str());
    const bool terminal = message == "saved_map_active" || message == "geometry_source_superseded" ||
                          message == "stale_geometry" || message.rfind("geometry_build_failed:",0)==0;
    if (!success && !terminal) return; // Keep the same frozen revision available for retry.
    // A terminal rejection also releases the pending slot. Retain its advertised
    // bundle so a restarted consumer can retry; a later accepted loop supersedes it.
    if (geometry_confirmed_) removeGeometryDirectory(geometry_confirmed_->frozen_dir);
    geometry_confirmed_optimizations_ = geometry_pending_optimizations_;
    geometry_confirmed_ = std::move(geometry_pending_);
    geometry_pending_.reset();
  }
  Status configure(const SlamConfig& config) override {
    advanceRelocalizationMapEpoch();
    config_ = config;
    config_.backend = "ltlio";
    frontend_config_ = Config{};
    runtime_config_ = RuntimeConfig{};
    const Status status = loadYamlConfig(
        config_.config_path, config_, frontend_config_, runtime_config_);
    if (!status.ok) {
      state_ = SlamState::Failed;
      reason_ = status.message;
      return status;
    }
    try {
      online_mapping_.configure(runtime_config_.online_mapping);
    } catch (const std::exception& error) {
      state_ = SlamState::Failed;
      reason_ = std::string("invalid_online_mapping_config: ") + error.what();
      return Status::Error(reason_);
    }
    try {
      resetCore();
    } catch (const std::exception& error) {
      state_ = SlamState::Failed;
      reason_ = std::string("invalid_ltlio_config: ") + error.what();
      return Status::Error(reason_);
    }
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
        !finite(sample.gz) || !finite(sample.ax) || !finite(sample.ay) || !finite(sample.az)) {
      ++dropped_imu_frames_;
      return Status::Error("invalid_imu_sample");
    }
    const double timestamp = sample.stamp_s - runtime_config_.time_diff_lidar_to_imu;
    if (isSensorTimeJump(last_imu_time_, timestamp) ||
        (last_imu_time_ >= 0 && timestamp < last_imu_time_)) {
      const bool rollback = timestamp < last_imu_time_;
      handleSensorTimeJump(timestamp);
      if (rollback) ++imu_rollback_count_;
    }
    const V3D gyro(sample.gx, sample.gy, sample.gz);
    if (runtime_config_.odom_prior_bypass_ltlio) {
      imu_buffer_.push_back({gyro, timestamp});
      while (imu_buffer_.size() > runtime_config_.max_imu_buffer) {
        imu_buffer_.pop_front();
        ++dropped_imu_frames_;
      }
    } else {
      frontend_->feedImu({timestamp, V3D(sample.ax, sample.ay, sample.az), gyro});
    }
    last_imu_time_ = timestamp;
    return Status::Ok("imu_accepted");
  }

  Status feedLidar(const LidarFrame& frame) override {
    if (!finite(frame.stamp_s) || frame.points.empty()) {
      ++dropped_lidar_frames_;
      return Status::Error(frame.points.empty() ? "empty_lidar_frame" : "invalid_lidar_frame");
    }
    if (isSensorTimeJump(last_lidar_time_, frame.stamp_s) ||
        (last_lidar_time_ >= 0 && frame.stamp_s < last_lidar_time_)) {
      const bool rollback = frame.stamp_s < last_lidar_time_;
      handleSensorTimeJump(frame.stamp_s);
      if (rollback) ++lidar_rollback_count_;
    }
    last_lidar_time_ = frame.stamp_s;
    if (runtime_config_.odom_prior_bypass_ltlio) pushLidarFrame(frame);
    else frontend_->feedCloud(toNativeCloud(frame));
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
      // Reject unusable external velocity at intake, before any scan can
      // commit a map or observation using that source.
      if (sample.has_velocity) {
        const V3D velocity(sample.vx, sample.vy, sample.vz);
        if (!velocity.allFinite() || velocityOutOfBounds(velocity)) {
          const std::string fault = velocity.allFinite()
              ? "ltlio_velocity_out_of_bounds" : "ltlio_state_nonfinite";
          setLtlioHealthFault(SlamState::Lost, fault, true);
          return Status::Error(fault);
        }
      }
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
    OdomSample lio_sample = sample;
    if (has_lio_odom_correction_) {
      lio_sample.odom_body = composePoses(lio_from_odom_, sample.odom_body);
      if (sample.has_velocity) {
        const V3D velocity = rotationFromPose(lio_from_odom_) * V3D(sample.vx, sample.vy, sample.vz);
        lio_sample.vx = velocity.x();
        lio_sample.vy = velocity.y();
        lio_sample.vz = velocity.z();
      }
    }
    recordTrajectory(lio_sample);
    return Status::Ok("visual_odom_accepted");
  }

  Status setInitialPose(const Pose3d& pose) override {
    if (!validPose(pose) || !alive_) return Status::Error("invalid_initial_pose");
    if (frontend_result_ && frontend_result_->accepted) {
      // An initialized pose correction changes the current state in the existing
      // map frame, not the frame of all previously saved scan patches.
      if (!correctFrontendPose(pose, std::nullopt))
        return Status::Error("initial_pose_outside_imu_history");
      odometry_lio_body_ = composePoses(frontend_anchor_, frontendBodyPose(*frontend_result_));
      if (frontend_result_->deskewed_cloud) {
        const auto world = transformLidarCloudWithBodyPose(
            frontend_result_->deskewed_cloud->makeShared(), mapBodyPose(*odometry_lio_body_),
            frontend_config_, runtime_config_);
        map_cloud_map_ = std::make_shared<Cloud>(toContractCloud(world, last_stamp_s_, config_.map_frame));
      }
    } else {
      frontend_initial_pose_ = pose;
      frontend_anchored_ = false;
      odometry_lio_body_ = pose;
    }
    odometry_odom_body_ = pose;
    state_estimation_at_scan_ = pose;
    odometry_twist_body_.reset();
    recordTrajectory({last_stamp_s_, *odometry_lio_body_});
    reason_ = "initial_pose_set";
    return Status::Ok(reason_);
  }

  Status relocalize(const std::optional<Pose3d>& guess,
                    RelocalizationSearch search) override {
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

    if (elevatorTransportActive()) {
      return waitForRelocalizationInput("waiting_for_elevator_exit");
    }
    if (relocalizationNeedsPostTransportScan()) {
      return waitForRelocalizationInput("waiting_for_scan");
    }

    abandonAsyncRelocalization();
    relocalization_state_ = "requested";
    const bool preserve_tracking_on_relocalization_failure =
        state_ == SlamState::Tracking && has_map_lio_pose_ && odometry_odom_body_.has_value();
    const bool map_alignment_update =
        (search == RelocalizationSearch::Global || !guess.has_value()) &&
        has_map_lio_pose_ && odometry_odom_body_.has_value();
    const std::string reason_before_relocalization = reason_;
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      return failRelocalization(
          "map_not_loaded", "map_not_loaded", false, reason_before_relocalization);
    }
    const bool global_search = search == RelocalizationSearch::Global;
    std::optional<Pose3d> effective_guess = global_search ? std::nullopt : guess;
    if (!global_search && !effective_guess.has_value() && has_map_lio_pose_ &&
        odometry_odom_body_.has_value()) {
      effective_guess = composePoses(mapOdomPose(), *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      return failRelocalization(
          "initial_pose_required",
          global_search ? "global_relocalization_unavailable" : "initial_pose_required",
          preserve_tracking_on_relocalization_failure, reason_before_relocalization);
    }
    if (!(registered_cloud_body_ != nullptr) || registered_cloud_body_->points.empty()) {
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
        ? relocalizer_->relocalize(*registered_cloud_body_, *effective_guess,
                                  *odometry_odom_body_, map_alignment_update)
        : relocalizer_->globalRelocalize(*registered_cloud_body_, *odometry_odom_body_);
    return applyRelocalizationResult(
        result,
        observation_sequence_,
        map_alignment_update,
        preserve_tracking_on_relocalization_failure,
        reason_before_relocalization);
  }

  Status startRelocalizeAsync(const std::optional<Pose3d>& guess,
                              RelocalizationSearch search) override {
    if (mode_ != SlamMode::Localization) {
      return Status::Error("localization_mode_required");
    }
    if (async_relocalization_job_.has_value()) {
      return Status::Error("async_relocalization_in_progress");
    }
    if (elevatorTransportActive()) {
      return waitForRelocalizationInput("waiting_for_elevator_exit");
    }
    if (relocalizationNeedsPostTransportScan()) {
      return waitForRelocalizationInput("waiting_for_scan");
    }
    const bool preserve_tracking_on_failure =
        state_ == SlamState::Tracking && has_map_lio_pose_ && odometry_odom_body_.has_value();
    const bool map_alignment_update =
        (search == RelocalizationSearch::Global || !guess.has_value()) &&
        has_map_lio_pose_ && odometry_odom_body_.has_value();
    const std::string reason_before_relocalization = reason_;
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      return failRelocalization(
          "map_not_loaded", "map_not_loaded", false, reason_before_relocalization);
    }
    const bool global_search = search == RelocalizationSearch::Global;
    std::optional<Pose3d> effective_guess = global_search ? std::nullopt : guess;
    if (!global_search && runtime_config_.odom_prior_bypass_ltlio &&
        guess.has_value() && odometry_odom_body_.has_value()) {
      effective_guess = planarSeedWithOdomHeightAndTilt(
          *guess, *odometry_odom_body_);
    }
    if (!global_search && !effective_guess.has_value() && has_map_lio_pose_ &&
        odometry_odom_body_.has_value()) {
      effective_guess = composePoses(mapOdomPose(), *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      return failRelocalization(
          "initial_pose_required",
          global_search ? "global_relocalization_unavailable" : "initial_pose_required",
          preserve_tracking_on_failure, reason_before_relocalization);
    }
    if (!(registered_cloud_body_ != nullptr) || registered_cloud_body_->points.empty()) {
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
    job.elevator_entries = elevatorEntries();
    job.elevator_exits = elevatorExits();
    job.map_alignment_update = map_alignment_update;
    job.preserve_tracking_on_failure = preserve_tracking_on_failure;
    job.compute = compute;

    try {
      std::thread worker([
          compute,
          relocalizer,
          scan,
          effective_guess,
          odom_body,
          map_alignment_update]() mutable {
        try {
          compute->result = effective_guess.has_value()
              ? relocalizer->relocalize(scan, *effective_guess, odom_body, map_alignment_update)
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
        (state_ == SlamState::Tracking && has_map_lio_pose_ && odometry_odom_body_.has_value());
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
    if (elevatorTransportActive()) {
      return waitForRelocalizationInput("waiting_for_elevator_exit");
    }
    if (completed.elevator_entries != elevatorEntries() ||
        completed.elevator_exits != elevatorExits()) {
      return waitForRelocalizationInput("async_relocalization_transport_changed");
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
        completed.observation_sequence,
        completed.map_alignment_update,
        preserve_tracking,
        reason_before_relocalization);
  }

  bool relocalizeAsyncInFlight() const override {
    return async_relocalization_job_.has_value();
  }

  Status setElevator(bool active) override {
    if (!frontend_) return Status::Error("not_configured");
    if (runtime_config_.elevator_mode == "off") return Status::Error("elevator_disabled");
    if (runtime_config_.odom_prior_bypass_ltlio || !frontend_->initialized())
      return Status::Error("elevator_not_initialized");
    frontend_->requestElevator(active);
    return Status::Ok("elevator_request_queued");
  }

  Status tick() override {
    if (!alive_ || !frontend_) {
      state_ = SlamState::Failed;
      reason_ = "not_configured";
      return Status::Error(reason_);
    }
    // Relocalization publishes the jump in the same runtime cycle. Clear it
    // before processing the next cycle so consumers observe an event, not a
    // permanently latched state.
    map_frame_jump_ = false;
    updateOnlineMapping();
    if (catastrophic_health_fault_latched_ && mode_ == SlamMode::Mapping) {
      enforceCatastrophicHealthFault();
      return Status::Ok(reason_);
    }
    if (!runtime_config_.odom_prior_bypass_ltlio) return tickFrontend();
    if (!prepareExternalScan()) {
      updateWaitingReason();
      return Status::Ok(reason_);
    }
    auto prior = freshOdomPrior(package_.cloud_end_time);
    if (!prior) {
      reason_ = "waiting_for_time_aligned_odom_prior";
      return Status::Ok(reason_);
    }
    return processOdomPriorPackage(*prior);
  }

  Status saveMap(const std::string& pcd_path) override {
    Status started = startSaveMapAsync(pcd_path);
    while (!started.ok && started.message == "online_mapping_busy_retry_save") {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      started = startSaveMapAsync(pcd_path);
    }
    if (!started.ok) return started;
    while (true) {
      if (auto completed = pollSaveMapAsync()) return *completed;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  Status startSaveMapAsync(const std::string& pcd_path) override {
    if (save_job_) return Status::Error("map_snapshot_in_progress");
    if (!frontend_) return Status::Error("not_configured");
    if (runtime_config_.odom_prior_bypass_ltlio ? !registered_cloud_body_
                                             : (!frontend_result_ || !frontend_result_->map_ready))
      return Status::Error("map_not_ready");
    updateOnlineMapping();
    if (online_mapping_state_.find("online_mapping_failed:") == 0)
      return Status::Error("online_mapping_failed_cannot_save");
    // The first loop can still be in flight even when the last published
    // snapshot has no corrections. Finish that prefix before choosing a save.
    if (mode_ == SlamMode::Mapping && online_mapping_.busy())
      return Status::Error("online_mapping_busy_retry_save");
    const bool corrected_save = mode_ == SlamMode::Mapping && global_mapping_ &&
        global_mapping_->optimizations > 0;
    if (corrected_save && (online_mapping_.dropped_frames() != 0 ||
        patch_history_dropped_count_ != 0 ||
        global_mapping_->keyframes.size() != patch_history_.size()))
      return Status::Error("online_mapping_incomplete_cannot_save_corrected_map");
    if (corrected_save) {
      for (std::size_t i = 0; i < patch_history_.size(); ++i)
        if (patch_history_[i]->name != global_mapping_->keyframes[i].patch_name)
          return Status::Error("online_mapping_patch_identity_mismatch");
    }

    SaveCapture capture;
    capture.pcd = mapPcdPath(pcd_path);
    capture.sam_snapshot = mode_ == SlamMode::Mapping;
    capture.online_mapping_config = online_mapping_.config();
    if (capture.sam_snapshot) capture.evidence = global_mapping_;
    capture.trajectory = pose_history_;
    capture.patches.reserve(patch_history_.size());
    for (const auto& patch : patch_history_)
      capture.patches.push_back({patch, patch->pose});
    capture.dropped_patches = patch_history_dropped_count_;
    capture.lidar_origin = runtime_config_.navigation_body_from_imu_translation
        + runtime_config_.navigation_body_from_imu_rotation * frontend_config_.imu_from_lidar_translation;
    capture.stamp_s = last_stamp_s_;
    capture.frame_id = config_.map_frame;
    const bool capacity_reached = mappingCapacityReached();
    if (capacity_reached) {
      // The map archive is frozen, while LIO continues matching new scans.
      capture.stamp_s = patch_history_.back()->stamp_s;
      capture.trajectory.erase(std::remove_if(capture.trajectory.begin(), capture.trajectory.end(),
          [&](const OdomSample& sample) { return sample.stamp_s > capture.stamp_s; }),
          capture.trajectory.end());
    }
    if (corrected_save) {
      capture.correction = global_mapping_;
    } else if (!capacity_reached && runtime_config_.odom_prior_enabled && !odom_prior_map_.empty()) {
      capture.cloud = odomPriorMapPclCloud();
    } else if (!capacity_reached && frontend_) {
      // Static scan patches remain authoritative after any transport: the
      // upstream local matching tree may still contain cabin surfaces.
      if (frontend_result_ && !frontend_result_->state.in_elevator &&
          frontend_result_->elevator.entries == 0) {
        const auto cloud = frontend_->mapSnapshot();
        capture.cloud = *transformCloud(cloud,
            rotationFromPose(frontend_anchor_),
            V3D(frontend_anchor_.x, frontend_anchor_.y, frontend_anchor_.z));
      }
    }
    // During transport, static patches are the save source. The cabin-relative
    // matching tree must not leak into the building map through this fallback.
    // Keep the full-posterior LIO trajectory and patches for the existing SAM
    // save path. The continuous-odom correction must not transform old floors.

    auto compute = std::make_shared<SaveComputeState>();
    AsyncSaveJob job;
    job.source_epoch = source_epoch_;
    job.observation_sequence = capacity_reached
        ? patch_history_.back()->observation_sequence : observation_sequence_;
    job.compute = compute;
    try {
      std::thread worker([compute, capture = std::move(capture)]() mutable {
        try {
          compute->status = stageCapturedMap(capture, *compute);
        } catch (const std::exception& error) {
          compute->status = Status::Error(std::string("map_snapshot_failed: ") + error.what());
        } catch (...) {
          compute->status = Status::Error("map_snapshot_unknown_failure");
        }
        compute->ready.store(true, std::memory_order_release);
      });
      worker.detach();
    } catch (const std::exception& error) {
      return Status::Error(std::string("map_snapshot_start_failed: ") + error.what());
    }
    save_job_ = std::move(job);
    return Status::Ok("map_snapshot_started");
  }

  bool saveMapAsyncInFlight() const override { return save_job_.has_value(); }

  std::optional<Status> pollSaveMapAsync() override {
    if (!save_job_ || !save_job_->compute->ready.load(std::memory_order_acquire))
      return std::nullopt;
    AsyncSaveJob job = std::move(*save_job_);
    save_job_.reset();
    if (job.source_epoch != source_epoch_)
      return Status::Error("map_snapshot_source_changed");
    auto& completed = *job.compute;
    if (!completed.status.ok) return completed.status;
    try {
      const auto committed = commitCapturedMap(completed);
      if (!committed.ok) return committed;
    } catch (const std::exception& error) {
      return Status::Error(std::string("map_snapshot_failed: ") + error.what());
    }

    saved_map_cloud_map_ = std::move(completed.cloud);
    saved_map_observation_sequence_ = job.observation_sequence;
    ++saved_map_revision_;
    saved_map_points_ = static_cast<int>(saved_map_cloud_map_->points.size());
    return Status::Ok("map_snapshot_ready");
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
    auto candidate = std::make_shared<NativeRelocalizer>();
    std::string relocalizer_message;
    if (!candidate->loadMap(pcd.string(), &relocalizer_message)) {
      return Status::Error(relocalizer_message);
    }
    // A failed candidate must leave the current map, bounds and alignment usable.
    relocalizer_ = std::move(candidate);
    updateMapBounds(cloud);
    advanceRelocalizationMapEpoch();
    saved_map_cloud_map_.reset();
    saved_map_observation_sequence_ = 0U;
    ++saved_map_revision_;
    saved_map_points_ = static_cast<int>(cloud.size());
    map_loaded_ = true;
    relocalization_state_ = "idle";
    last_relocalization_message_ = relocalizer_message;
    relocalization_engine_.clear();
    if (mode_ == SlamMode::Localization) {
      if (has_map_lio_pose_) ++source_epoch_;
      has_map_lio_pose_ = false;
      map_lio_pose_ = Pose3d{};
      map_body_pose_at_relocalization_.reset();
      map_cloud_map_.reset();
      map_frame_jump_ = false;
      relocalization_state_ = "required";
      relocalization_quality_ = -1.0;
      if (!ltlio_health_fault_active_ && !relocalization_required_after_time_jump_) {
        state_ = SlamState::Localizing;
        confidence_ = localization_quality_ = 0.0;
        reason_ = "waiting_for_relocalization";
      }
    } else {
      reason_ = "map_loaded";
    }
    return Status::Ok("map_loaded");
  }

  SlamOutputs outputs() const override {
    SlamOutputs out;
    out.state = state_;
    out.stamp_s = last_stamp_s_;
    out.confidence = confidence_;
    out.reason = reason_;
    if (!ltlio_health_fault_active_ && !relocalization_required_after_time_jump_) {
      out.odometry_odom_body = odometry_odom_body_;
      out.state_estimation_at_scan = state_estimation_at_scan_;
      out.odometry_twist_body = odometry_twist_body_;
      out.registered_cloud_body = registered_cloud_body_;
      out.map_cloud_map = map_cloud_map_;
    }
    out.saved_map_cloud_map = saved_map_cloud_map_;
    out.saved_map_revision = saved_map_revision_;
    out.saved_map_observation_sequence = saved_map_observation_sequence_;
    if (mode_ == SlamMode::Mapping) {
      out.global_map_cloud = global_map_cloud_;
      out.global_map_busy = online_mapping_.busy();
      out.global_map_dropped_frames = online_mapping_.dropped_frames();
      out.global_map_optimization_failures = online_mapping_.optimization_failures();
      out.global_map_state = mappingCapacityReached() &&
              online_mapping_state_.find("online_mapping_failed:") != 0
          ? "mapping_keyframe_capacity_reached" : online_mapping_state_;
      if (global_mapping_) {
        out.global_map_revision = global_mapping_->revision;
        out.global_map_keyframes = global_mapping_->keyframes.size();
        out.global_map_registered_keyframes = global_mapping_->registered_keyframes;
        out.global_map_rejected_keyframes = global_mapping_->registration_rejections;
        out.global_map_loops = global_mapping_->loop_constraints;
        out.global_map_optimizations = global_mapping_->optimizations;
      }
    }
    if (mode_ != SlamMode::Localization || has_map_lio_pose_) {
      out.map_odom_tf = Transform3d{
          config_.map_frame,
          config_.odom_frame,
          mapOdomPose()};
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
    out.relocalization_engine = relocalization_engine_;
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
    if (!runtime_config_.odom_prior_bypass_ltlio) fillFrontendDiagnostics(out);
    else {
      out.ltlio_converged = false;
      if (external_velocity_world_) {
        out.ltlio_velocity_x = external_velocity_world_->x();
        out.ltlio_velocity_y = external_velocity_world_->y();
        out.ltlio_velocity_z = external_velocity_world_->z();
      }
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
    has_map_lio_pose_ = false;
    map_bounds_ready_ = false;
    map_lio_pose_ = Pose3d{};
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
    relocalization_engine_.clear();
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
  Pose3d mapOdomPose() const {
    const Pose3d map_lio = has_map_lio_pose_ ? map_lio_pose_ : Pose3d{};
    return has_lio_odom_correction_ ? composePoses(map_lio, lio_from_odom_) : map_lio;
  }

  Pose3d mapBodyPose(const Pose3d& lio_body) const {
    return has_map_lio_pose_ ? composePoses(map_lio_pose_, lio_body) : lio_body;
  }

  Pose3d lioOdomCorrection(const lio::Result& result) const {
    if (result.world_from_odom.matrix().isIdentity(0.0)) return Pose3d{};
    const auto& t = result.world_from_odom.translation();
    const Eigen::Quaterniond q(result.world_from_odom.rotation());
    const Pose3d correction{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
    return composePoses(composePoses(frontend_anchor_, correction), inversePose(frontend_anchor_));
  }

  Pose3d continuousBodyPose(const lio::Result& result, const Pose3d& lio_body) const {
    return result.world_from_odom.matrix().isIdentity(0.0) ? lio_body
        : composePoses(inversePose(lioOdomCorrection(result)), lio_body);
  }

  void commitOdomCorrection(const lio::Result& result) {
    if ((result.world_from_odom.matrix().array() == frontend_world_from_odom_.matrix().array()).all())
      return;
    const Pose3d previous = mapOdomPose();
    frontend_world_from_odom_ = result.world_from_odom;
    lio_from_odom_ = lioOdomCorrection(result);
    has_lio_odom_correction_ = !result.world_from_odom.matrix().isIdentity(0.0);
    // The estimator and its map keep the full posterior. Only their relation
    // to continuous navigation odometry changes; this is not a new map source.
    const Pose3d current = mapOdomPose();
    const double translation_m = std::hypot(current.x - previous.x,
        current.y - previous.y, current.z - previous.z);
    const double yaw_rad = relocalization_gate_detail::AngleDistance(
        yawFromPose(current), yawFromPose(previous));
    if (pathInvalidatingCorrection(translation_m, yaw_rad)) {
      map_frame_jump_ = true;
      ++map_frame_jump_sequence_;
    }
  }

  void recordTrajectory(const OdomSample& sample) {
    if (mappingCapacityReached()) return;
    pose_history_.push_back(sample);
    if (pose_history_.size() > 10000) pose_history_.erase(pose_history_.begin());
  }

  void publishScan(const CloudType::Ptr& body_cloud,
                   const CloudType::Ptr& world_cloud, double stamp_s) {
    registered_cloud_body_ = std::make_shared<Cloud>(
        toContractCloud(body_cloud, stamp_s, config_.body_frame));
    map_cloud_map_ = std::make_shared<Cloud>(
        toContractCloud(world_cloud, stamp_s, config_.map_frame));
    last_stamp_s_ = stamp_s;
    ++observation_sequence_;
    recordTrajectory({stamp_s, *odometry_lio_body_});
  }

  void resetFrontend() {
    frontend_.reset();
    frontend_result_.reset();
    frontend_anchor_ = Pose3d{};
    frontend_world_from_odom_ = Eigen::Isometry3d::Identity();
    lio_from_odom_ = Pose3d{};
    has_lio_odom_correction_ = false;
    odometry_lio_body_.reset();
    frontend_initial_pose_.reset();
    frontend_anchored_ = false;
    frontend_attempts_ = frontend_rejections_ = 0;
    frontend_previous_rejection_ = "none";
    last_published_elevator_exits_ = 0U;
    elevator_exit_observation_sequence_ = 0U;
    frontend_config_.elevator_enabled = runtime_config_.elevator_mode != "off";
    frontend_config_.automatic_door = runtime_config_.elevator_mode == "auto";
    frontend_config_.max_imu_buffer = runtime_config_.max_imu_buffer;
    frontend_config_.max_lidar_buffer = runtime_config_.max_lidar_buffer;
    frontend_ = std::make_unique<lio::Frontend>(frontend_config_);
  }

  Pose3d frontendBodyPose(const lio::Result& result) const {
    const auto& q = result.state.orientation;
    const auto& p = result.world_position;
    return navigationBodyPoseFromImuPose(
        Pose3d{p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()}, runtime_config_);
  }

  void anchorFrontendOutput() {
    if (frontend_anchored_) return;
    const auto initial = frontend_->initialState();
    if (!initial) return;
    const auto& q = initial->orientation;
    const auto& p = initial->position;
    const Pose3d body = navigationBodyPoseFromImuPose(
        Pose3d{p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()}, runtime_config_);
    if (frontend_initial_pose_) {
      frontend_anchor_ = composePoses(*frontend_initial_pose_, inversePose(body));
    } else if (mode_ == SlamMode::Mapping) {
      const auto r = rotationFromPose(body);
      const Eigen::Quaterniond anchor_q(Eigen::AngleAxisd(
          -std::atan2(r(1, 0), r(0, 0)), V3D::UnitZ()));
      const V3D t = -(anchor_q * V3D(body.x, body.y, body.z));
      frontend_anchor_ = {t.x(), t.y(), t.z(), anchor_q.x(), anchor_q.y(),
                          anchor_q.z(), anchor_q.w()};
    }
    frontend_anchored_ = true;
  }

  std::string frontendNumericFault() const {
    if (!frontend_result_) return "ltlio_state_unavailable";
    const auto& result = *frontend_result_;
    const auto& state = result.state;
    if (!state.orientation.coeffs().allFinite() || !result.world_position.allFinite() ||
        !result.world_velocity.allFinite() || !state.gyro_bias.allFinite() ||
        !state.accel_bias.allFinite() || !state.gravity.allFinite() ||
        !finite(state.transport_acceleration) || !result.world_from_odom.matrix().allFinite())
      return "ltlio_state_nonfinite";
    if (!state.covariance.allFinite() || !result.world_position_covariance.allFinite() ||
        result.world_position_covariance.trace() < 0) return "ltlio_covariance_nonfinite";
    if (velocityOutOfBounds(result.world_velocity)) return "ltlio_velocity_out_of_bounds";
    if (runtime_config_.ltlio_max_pos_cov_trace > 0 &&
        result.world_position_covariance.trace() > runtime_config_.ltlio_max_pos_cov_trace)
      return "ltlio_position_covariance_out_of_bounds";
    return {};
  }

  lio::PosePrior frontendPrior(const Pose3d& pose, const std::optional<V3D>& body_velocity,
                              const lio::Result& result) const {
    const bool corrected = !result.world_from_odom.matrix().isIdentity(0.0);
    const Pose3d correction = lioOdomCorrection(result);
    const Pose3d lio_body = corrected ? composePoses(correction, pose) : pose;
    const Pose3d local_imu = composePoses(inversePose(frontend_anchor_),
        imuPoseFromNavigationBodyPose(lio_body, runtime_config_));
    lio::PosePrior prior;
    prior.world_imu_position = V3D(local_imu.x, local_imu.y, local_imu.z);
    prior.world_imu_rotation = Eigen::Quaterniond(local_imu.qw, local_imu.qx, local_imu.qy, local_imu.qz);
    if (body_velocity) {
      const V3D angular_body = runtime_config_.navigation_body_from_imu_rotation *
          (result.causal_gyro_rps - result.state.gyro_bias);
      const V3D lio_velocity = corrected ? rotationFromPose(correction) * *body_velocity : *body_velocity;
      prior.world_imu_velocity = rotationFromPose(frontend_anchor_).transpose() *
          (lio_velocity + rotationFromPose(lio_body) *
           angular_body.cross(runtime_config_.navigation_body_from_imu_translation));
    }
    return prior;
  }

  bool correctFrontendPose(const Pose3d& pose, const std::optional<V3D>& body_velocity) {
    if (!frontend_result_) return false;
    auto& result = *frontend_result_;
    const auto prior = frontendPrior(pose, body_velocity, result);
    if (!frontend_->correctState(result.scan_end_s, prior.world_imu_rotation,
                                prior.world_imu_position, prior.world_imu_velocity)) return false;
    result.state.orientation = prior.world_imu_rotation;
    result.world_position = prior.world_imu_position;
    result.state.position = result.world_position;
    result.state.position.z() -= result.state.transport_height;
    if (prior.world_imu_velocity) {
      result.world_velocity = *prior.world_imu_velocity;
      result.state.velocity = result.world_velocity;
      result.state.velocity.z() -= result.state.transport_velocity;
    }
    return true;
  }

  Status tickFrontend() {
    std::optional<OdomSample> prior;
    auto next = frontend_->processNext([&](const lio::Result& candidate) -> std::optional<lio::PosePrior> {
      prior = freshOdomPrior(candidate.scan_end_s);
      if (!prior) return std::nullopt;
      anchorFrontendOutput();
      odom_prior_error_xy_m_ = planarDistance(
          continuousBodyPose(candidate, composePoses(frontend_anchor_, frontendBodyPose(candidate))),
          prior->odom_body);
      const std::optional<V3D> velocity = prior->has_velocity
          ? std::optional<V3D>(V3D(prior->vx, prior->vy, prior->vz)) : std::nullopt;
      return frontendPrior(prior->odom_body, velocity, candidate);
    });
    anchorFrontendOutput();
    if (!next) {
      if (updateWaitingHealth()) return Status::Ok(reason_);
      if (!frontend_->initialized()) reason_ = "imu_initializing";
      else if (!frontend_result_ || !frontend_result_->map_ready) reason_ = "map_initializing";
      return Status::Ok(reason_);
    }
    // A filtered-empty scan is a lifecycle notification, not a new estimate.
    // Keep the last real state instead of displaying its zero-filled fields.
    if (next->map_ready && !next->attempted && !next->deskewed_cloud) {
      scan_start_s_ = next->scan_start_s;
      scan_end_s_ = next->scan_end_s;
      updateWaitingHealth();
      return Status::Ok(reason_);
    }
    frontend_result_ = std::move(next);
    auto& result = *frontend_result_;
    scan_start_s_ = result.scan_start_s;
    scan_end_s_ = result.scan_end_s;
    if (result.attempted) {
      ++frontend_attempts_;
      frontend_rejections_ = result.accepted ? 0 : frontend_rejections_ + 1;
      if (!result.accepted) frontend_previous_rejection_ = result.rejection_reason;
    }
    if (!result.map_ready) {
      state_ = SlamState::Initializing;
      reason_ = result.initialized ? "map_initializing" : "imu_initializing";
      if (relocalization_required_after_time_jump_) {
        state_ = SlamState::Lost;
        confidence_ = localization_quality_ = 0;
        reason_ = "sensor_time_jump_relocalization_required";
      }
      return Status::Ok(reason_);
    }
    const auto fault = frontendNumericFault();
    if (!fault.empty()) {
      setLtlioHealthFault(SlamState::Lost, fault, true);
      return Status::Ok(reason_);
    }
    if (!prior && result.attempted && !result.accepted) {
      const bool repeated = frontend_rejections_ >= 2;
      setLtlioHealthFault(repeated ? SlamState::Lost : SlamState::Degraded,
          repeated ? "ltlio_lidar_update_rejected_streak" : "ltlio_lidar_update_rejected");
      return Status::Ok(reason_);
    }
    if ((!result.accepted && !prior) || !result.deskewed_cloud) return Status::Ok(reason_);
    odometry_lio_body_ = composePoses(frontend_anchor_, frontendBodyPose(result));
    odometry_odom_body_ = continuousBodyPose(result, *odometry_lio_body_);
    commitOdomCorrection(result);
    state_estimation_at_scan_ = odometry_odom_body_;
    odometry_twist_body_.reset();
    const double gyro_age = result.scan_end_s - result.causal_gyro_stamp_s;
    if (gyro_age >= 0 && (runtime_config_.max_imu_gap_s <= 0 || gyro_age <= runtime_config_.max_imu_gap_s)) {
      const V3D angular = runtime_config_.navigation_body_from_imu_rotation *
          (result.causal_gyro_rps - result.state.gyro_bias);
      const V3D velocity = rotationFromPose(frontend_anchor_) * result.world_velocity;
      const V3D linear = rotationFromPose(*odometry_lio_body_).transpose() * velocity -
          angular.cross(runtime_config_.navigation_body_from_imu_translation);
      odometry_twist_body_ = BodyTwist{linear.x(), linear.y(), linear.z(),
                                      angular.x(), angular.y(), angular.z()};
    }
    // Both transforms create their own output and leave the estimator's
    // immutable scan untouched; no intermediate clone is needed.
    const auto& lidar_cloud = result.deskewed_cloud;
    const auto body_cloud = transformLidarCloudToNavigationBody(
        lidar_cloud, frontend_config_, runtime_config_);
    const auto world_cloud = transformLidarCloudWithBodyPose(
        lidar_cloud, mapBodyPose(*odometry_lio_body_), frontend_config_, runtime_config_);
    const bool exited_transport = result.elevator.exits > last_published_elevator_exits_;
    publishScan(body_cloud, world_cloud, result.scan_end_s);
    last_published_elevator_exits_ = result.elevator.exits;
    if (exited_transport) elevator_exit_observation_sequence_ = observation_sequence_;
    if (prior && !result.state.in_elevator) {
      const auto lio_cloud = has_map_lio_pose_
          ? transformLidarCloudWithBodyPose(lidar_cloud, *odometry_lio_body_, frontend_config_, runtime_config_)
          : world_cloud;
      addOdomPriorMapCloud(lio_cloud);
    }
    if (!result.state.in_elevator) recordPatchSnapshot();
    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return Status::Ok(reason_);
    }
    ltlio_health_fault_active_ = false;
    if (relocalization_required_after_time_jump_) {
      state_ = SlamState::Lost;
      confidence_ = localization_quality_ = 0;
      reason_ = "sensor_time_jump_relocalization_required";
    } else if (mode_ == SlamMode::Localization && map_loaded_ && !has_map_lio_pose_) {
      state_ = SlamState::Localizing;
      confidence_ = localization_quality_ = 0.0;
      reason_ = "waiting_for_relocalization";
    } else {
      state_ = SlamState::Tracking;
      confidence_ = localization_quality_ = result.sampled_points > 0
          ? std::min(1.0, static_cast<double>(result.effective_points) / result.sampled_points) : 0;
      reason_ = prior ? "tracking_with_odom_prior" : "tracking";
    }
    return Status::Ok(reason_);
  }

  void fillFrontendDiagnostics(SlamOutputs& out) const {
    out.ltlio_iter_num = 0;
    out.ltlio_converged = false;
    out.ltlio_geometry_available = false;
    out.ltlio_degeneracy_detected = false;
    out.ltlio_degenerate_dof_count = 0;
    out.ltlio_condition_number = 0.0;
    out.ltlio_min_eigenvalue = 0.0;
    out.ltlio_max_eigenvalue = 0.0;
    out.ltlio_rotation_scale_m = 0.0;
    out.ltlio_geometry_eigenvalues = {};
    out.ltlio_principal_directions = {};
    out.ltlio_geometry_categories = {};
    out.ltlio_geometry_support_ratios = {};
    out.ltlio_geometry_update_applied = false;
    out.ltlio_geometry_guidance = "awaiting_lidar_geometry";
    out.imu_buffer = static_cast<int>(frontend_->bufferedImus());
    out.lidar_buffer = static_cast<int>(frontend_->bufferedClouds());
    out.dropped_imu_frames += static_cast<int>(frontend_->droppedImus());
    out.dropped_lidar_frames += static_cast<int>(frontend_->droppedClouds());
    out.elevator.mode = runtime_config_.elevator_mode;
    if (!frontend_result_) return;
    const auto& r = *frontend_result_;
    const auto& geometry = r.geometry;
    out.ltlio_geometry_available = geometry.available;
    out.ltlio_degeneracy_detected = geometry.detected;
    out.ltlio_degenerate_dof_count = geometry.degenerate_dof_count;
    out.ltlio_condition_number = geometry.condition_number;
    out.ltlio_min_eigenvalue = geometry.min_eigenvalue;
    out.ltlio_max_eigenvalue = geometry.max_eigenvalue;
    out.ltlio_rotation_scale_m = geometry.rotation_scale_m;
    out.ltlio_geometry_eigenvalues = geometry.eigenvalues;
    out.ltlio_principal_directions = geometry.principal_directions;
    out.ltlio_geometry_categories = geometry.categories;
    out.ltlio_geometry_support_ratios = geometry.support_ratios;
    out.ltlio_geometry_update_applied = geometry.update_applied;
    out.ltlio_geometry_guidance = geometry.guidance;
    out.ltlio_iter_num = r.iterations;
    out.ltlio_converged = r.converged;
    const V3D lio_velocity = rotationFromPose(frontend_anchor_) * r.world_velocity;
    const V3D velocity = r.world_from_odom.matrix().isIdentity(0.0) ? lio_velocity
        : rotationFromPose(lioOdomCorrection(r)).transpose() * lio_velocity;
    out.ltlio_velocity_x = velocity.x();
    out.ltlio_velocity_y = velocity.y();
    out.ltlio_velocity_z = velocity.z();
    out.ltlio_pos_cov_trace = r.world_position_covariance.trace();
    out.ltlio_effective_ratio = r.sampled_points > 0
        ? static_cast<double>(r.effective_points) / r.sampled_points : 0;
    out.elevator.active = r.state.in_elevator;
    out.elevator.height_m = r.state.transport_height;
    out.elevator.velocity_mps = r.state.transport_velocity;
    out.elevator.acceleration_mps2 = r.state.transport_acceleration;
    constexpr const char* phases[] = {"idle", "accelerating", "cruising", "decelerating", "stop_check"};
    out.elevator.phase = r.elevator.phase >= 0 && r.elevator.phase < 5 ? phases[r.elevator.phase] : "idle";
    out.elevator.range_quantile_m = r.elevator.range_quantile_m;
    out.elevator.velocity_variance = r.elevator.velocity_variance;
    out.elevator.entries = r.elevator.entries;
    out.elevator.exits = r.elevator.exits;
    auto& d = out.ltlio_lidar_update;
    d.attempted = r.attempted;
    d.accepted = r.accepted;
    d.attempt_sequence = frontend_attempts_;
    d.consecutive_rejections = frontend_rejections_;
    d.rejection_reason = r.rejection_reason;
    d.previous_rejection_reason = frontend_previous_rejection_;
    d.candidate_translation_m = r.candidate_translation_m;
    d.candidate_rotation_rad = r.candidate_rotation_rad;
    d.candidate_velocity_mps = r.world_velocity.norm();
    d.candidate_velocity_delta_mps = r.candidate_velocity_delta_mps;
    d.max_update_translation_m = frontend_config_.max_update_translation_m;
    d.max_update_rotation_rad = frontend_config_.max_update_rotation_rad;
    d.max_update_velocity_mps = runtime_config_.max_update_velocity_mps;
    d.max_update_velocity_delta_mps = frontend_config_.max_update_velocity_delta_mps;
    d.downsampled_points = r.sampled_points;
    d.effective_points = r.effective_points;
    d.plane_matches = r.plane_matches;
    d.point_matches = r.point_matches;
    d.mapping_points = r.mapping_points;
    d.sampling_leaf_size_m = r.sampling_leaf_size_m;
    d.sampling_scale_m = r.sampling_scale_m;
  }

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

  bool elevatorTransportActive() const {
    return frontend_result_.has_value() && frontend_result_->state.in_elevator;
  }

  bool relocalizationNeedsPostTransportScan() const {
    return elevator_exit_observation_sequence_ != 0U &&
        observation_sequence_ <= elevator_exit_observation_sequence_;
  }

  std::uint64_t elevatorEntries() const {
    return frontend_result_.has_value() ? frontend_result_->elevator.entries : 0U;
  }

  std::uint64_t elevatorExits() const {
    return frontend_result_.has_value() ? frontend_result_->elevator.exits : 0U;
  }

  Status waitForRelocalizationInput(const std::string& message) {
    relocalization_state_ = "waiting";
    last_relocalization_message_ = message;
    return Status::Error(message);
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
      std::uint64_t observation_sequence,
      bool map_alignment_update,
      bool preserve_tracking_on_failure,
      const std::string& reason_before_relocalization) {
    relocalization_quality_ = result.quality;
    last_relocalization_message_ = result.message;
    relocalization_engine_ = result.engine;
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
    gate_input.current_map_odom = mapOdomPose();
    gate_input.candidate_map_odom = result.map_odom;
    const RelocalizationGateDecision gate =
        EvaluateRelocalizationGate(gate_config, gate_input);
    if (!gate.accepted) {
      if (gate.reason == "relocalization_translation_jump_rejected" ||
          gate.reason == "relocalization_yaw_jump_rejected") {
        // Preserve one candidate and its gate context together. Public map-body
        // status retains the last accepted pose, while quality describes this result.
        const auto pose_text = [](const Pose3d& pose) {
          std::ostringstream text;
          text << std::fixed << std::setprecision(9)
               << '[' << pose.x << ',' << pose.y << ',' << pose.z << ','
               << pose.qx << ',' << pose.qy << ',' << pose.qz << ',' << pose.qw << ']';
          return text.str();
        };
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        std::fprintf(
            stderr,
            "relocalization_gate reason=%s engine=%s scan_sequence=%llu "
            "translation_m=%.9f limit_translation_m=%.9f "
            "yaw_deg=%.9f limit_yaw_deg=%.9f tilt_deg=%.9f limit_tilt_deg=%.9f "
            "fitness=%.9f inliers=%d evaluated=%d "
            "current_map_odom=%s candidate_map_odom=%s candidate_map_body=%s config=%s\n",
            gate.reason.c_str(), result.engine.c_str(),
            static_cast<unsigned long long>(observation_sequence),
            gate.correction_translation_m, gate_config.max_alignment_translation_m,
            gate.correction_yaw_rad * kRadToDeg, runtime_config_.track_against_map_max_yaw_deg,
            gate.alignment_tilt_rad * kRadToDeg, runtime_config_.track_against_map_max_tilt_deg,
            result.quality, result.refine_inliers, result.evaluated_points,
            pose_text(mapOdomPose()).c_str(), pose_text(result.map_odom).c_str(),
            pose_text(result.map_body).c_str(), config_.config_path.c_str());
      }
      return failRelocalization(
          "rejected", gate.reason, preserve_tracking_on_failure, reason_before_relocalization);
    }

    const std::string numeric_fault = ltlioNumericHealthFaultReason();
    if (!numeric_fault.empty()) {
      setLtlioHealthFault(SlamState::Lost, numeric_fault, true);
      relocalization_state_ = "rejected";
      last_relocalization_message_ = "ltlio_numeric_health_unrecovered";
      return Status::Error(last_relocalization_message_);
    }

    // Only the owning runtime thread reaches this commit. LTLIO odometry
    // stays continuous while the independently computed map alignment changes.
    const bool path_invalidating_jump =
        !has_map_lio_pose_ || !map_alignment_update ||
        pathInvalidatingCorrection(gate.correction_translation_m, gate.correction_yaw_rad);
    state_estimation_at_scan_ = odometry_odom_body_;
    map_lio_pose_ = has_lio_odom_correction_
        ? composePoses(result.map_odom, inversePose(lio_from_odom_)) : result.map_odom;
    map_body_pose_at_relocalization_ = result.map_body;
    has_map_lio_pose_ = true;
    if (registered_cloud_body_ && odometry_lio_body_) {
      auto map_cloud = std::make_shared<Cloud>(*registered_cloud_body_);
      map_cloud->frame_id = config_.map_frame;
      const Pose3d map_body = mapBodyPose(*odometry_lio_body_);
      const M3D rotation = rotationFromPose(map_body);
      const V3D translation(map_body.x, map_body.y, map_body.z);
      for (auto& point : map_cloud->points) {
        const V3D xyz = rotation * V3D(point.x, point.y, point.z) + translation;
        point.x = static_cast<float>(xyz.x());
        point.y = static_cast<float>(xyz.y());
        point.z = static_cast<float>(xyz.z());
      }
      map_cloud_map_ = std::move(map_cloud);
    }
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
    ltlio_health_fault_active_ = false;
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
    return runtime_config_.max_update_velocity_mps > 0.0 &&
           velocity.norm() > runtime_config_.max_update_velocity_mps;
  }

  std::string ltlioNumericHealthFaultReason() const {
    return runtime_config_.odom_prior_bypass_ltlio ? std::string{} : frontendNumericFault();
  }

  void enforceCatastrophicHealthFault() {
    ltlio_health_fault_active_ = true;
    state_ = SlamState::Lost;
    confidence_ = 0.0;
    localization_quality_ = 0.0;
    reason_ = catastrophic_health_fault_reason_.empty()
        ? "ltlio_catastrophic_health_fault"
        : catastrophic_health_fault_reason_;
  }

  void setLtlioHealthFault(
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
    ltlio_health_fault_active_ = true;
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
    has_map_lio_pose_ = false;
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
    clearGeometryExports();
    ++source_epoch_;
    online_mapping_.reset(source_epoch_);
    global_mapping_.reset();
    global_map_cloud_.reset();
    online_mapping_state_ = "waiting_for_keyframes";
    resetFrontend();
    relocalizer_ = std::make_shared<NativeRelocalizer>();
    imu_buffer_.clear();
    lidar_buffer_.clear();
    lidar_pushed_ = false;
    package_ = SyncedScan{};
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
    odometry_twist_body_.reset();
    external_velocity_world_.reset();
    registered_cloud_body_.reset();
    observation_sequence_ = 0U;
    map_cloud_map_.reset();
    saved_map_cloud_map_.reset();
    saved_map_observation_sequence_ = 0U;
    pose_history_.clear();
    patch_history_.clear();
    patch_history_dropped_count_ = 0;
    next_patch_to_queue_ = 0;
    latest_odom_prior_.reset();
    odom_prior_buffer_.clear();
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    odom_prior_error_xy_m_ = -1.0;
    odom_prior_map_.clear();
    patch_sequence_ = 0;
    last_patch_stamp_s_ = 0.0;
    has_last_patch_pose_ = false;
    ltlio_health_fault_active_ = false;
    catastrophic_health_fault_latched_ = false;
    catastrophic_health_fault_reason_.clear();
    relocalization_required_after_time_jump_ = false;
  }

  void pushLidarFrame(LidarFrame frame) {
    lidar_buffer_.push_back(std::move(frame));
    while (lidar_buffer_.size() > runtime_config_.max_lidar_buffer) {
      lidar_buffer_.pop_front();
      ++dropped_lidar_frames_;
    }
  }

  template <typename MapCloud>
  void updateMapBounds(const MapCloud& cloud) {
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

  bool prepareExternalScan() {
    if (imu_buffer_.empty() || lidar_buffer_.empty()) {
      return false;
    }
    if (!lidar_pushed_) {
      const auto input = toNativeCloud(lidar_buffer_.front());
      package_.cloud = lio::selectPoints(input, frontend_config_);
      if (!package_.cloud || package_.cloud->points.empty()) {
        lidar_buffer_.pop_front();
        ++dropped_lidar_frames_;
        return false;
      }
      std::sort(
          package_.cloud->points.begin(),
          package_.cloud->points.end(),
          [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
      package_.cloud_start_time = input.stamp_s;
      package_.cloud_end_time = input.end_stamp_s;
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
        package_ = SyncedScan{};
        ++dropped_lidar_frames_;
        reason_ = "imu_gap_drop";
      }
      return false;
    }

    package_.imus.clear();
    while (!imu_buffer_.empty() && imu_buffer_.front().time <= package_.cloud_end_time) {
      package_.imus.emplace_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    imu_batch_ = static_cast<int>(package_.imus.size());
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
  }

  void recordOdometryTwist(const OdomSample* prior) {
    odometry_twist_body_.reset();
    if (!odometry_odom_body_ || !prior || !prior->has_velocity || package_.imus.empty()) return;
    const auto& imu = package_.imus.back();
    const double age = package_.cloud_end_time - imu.time;
    if (age < 0 || (runtime_config_.max_imu_gap_s > 0 && age > runtime_config_.max_imu_gap_s)) return;
    const V3D angular = runtime_config_.navigation_body_from_imu_rotation * imu.gyro;
    const V3D linear = rotationFromPose(*odometry_odom_body_).transpose() *
        V3D(prior->vx, prior->vy, prior->vz);
    if (!linear.allFinite() || !angular.allFinite()) return;
    odometry_twist_body_ = BodyTwist{linear.x(), linear.y(), linear.z(),
                                    angular.x(), angular.y(), angular.z()};
  }

  Status processOdomPriorPackage(const OdomSample& odom_prior) {
    if (!validPose(odom_prior.odom_body)) {
      setLtlioHealthFault(SlamState::Lost, "ltlio_state_nonfinite", true);
      return Status::Ok(reason_);
    }
    if (odom_prior.has_velocity) {
      const V3D prior_velocity(odom_prior.vx, odom_prior.vy, odom_prior.vz);
      if (!prior_velocity.allFinite()) {
        setLtlioHealthFault(SlamState::Lost, "ltlio_state_nonfinite", true);
        return Status::Ok(reason_);
      }
      if (velocityOutOfBounds(prior_velocity)) {
        setLtlioHealthFault(SlamState::Lost, "ltlio_velocity_out_of_bounds", true);
        return Status::Ok(reason_);
      }
    }

    odom_prior_error_xy_m_ = -1.0;
    external_velocity_world_ = odom_prior.has_velocity
        ? std::optional<V3D>(V3D(odom_prior.vx, odom_prior.vy, odom_prior.vz)) : std::nullopt;
    odometry_odom_body_ = odom_prior.odom_body;
    odometry_lio_body_ = odom_prior.odom_body;
    state_estimation_at_scan_ = odom_prior.odom_body;
    recordOdometryTwist(&odom_prior);
    auto body_cloud = transformLidarCloudToNavigationBody(
        package_.cloud, frontend_config_, runtime_config_);
    auto lio_cloud = transformLidarCloudWithBodyPose(
        package_.cloud, odom_prior.odom_body, frontend_config_, runtime_config_);
    auto world_cloud = has_map_lio_pose_
        ? transformLidarCloudWithBodyPose(package_.cloud, mapBodyPose(odom_prior.odom_body),
                                        frontend_config_, runtime_config_)
        : lio_cloud;
    publishScan(body_cloud, world_cloud, package_.cloud_end_time);
    addOdomPriorMapCloud(lio_cloud);
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
    ltlio_health_fault_active_ = false;
    if (mode_ == SlamMode::Localization && map_loaded_ && !has_map_lio_pose_) {
      state_ = SlamState::Localizing;
      confidence_ = localization_quality_ = 0.0;
      reason_ = "waiting_for_relocalization";
      return Status::Ok(reason_);
    }
    state_ = SlamState::Tracking;
    confidence_ = 1.0;
    localization_quality_ = 1.0;
    reason_ = "tracking_with_odom_prior_bypass";
    return Status::Ok(reason_);
  }

  bool updateWaitingHealth() {
    if (catastrophic_health_fault_latched_) {
      enforceCatastrophicHealthFault();
      return true;
    }
    if (ltlio_health_fault_active_ || relocalization_required_after_time_jump_) {
      return true;
    }
    if (runtime_config_.max_imu_gap_s > 0.0 && last_imu_time_ >= 0.0 &&
        last_lidar_time_ - last_imu_time_ > runtime_config_.max_imu_gap_s) {
      state_ = SlamState::Degraded;
      confidence_ = 0.0;
      localization_quality_ = 0.0;
      reason_ = "imu_gap_waiting_for_scan";
      return true;
    }
    return false;
  }

  void updateWaitingReason() {
    if (updateWaitingHealth()) return;
    if (runtime_config_.odom_prior_bypass_ltlio &&
        odom_prior_active_ &&
        state_ == SlamState::Tracking &&
        odometry_odom_body_.has_value() &&
        (map_cloud_map_ != nullptr)) {
      confidence_ = 1.0;
      localization_quality_ = 1.0;
      reason_ = "tracking_with_odom_prior_bypass";
      return;
    }
    if (state_ == SlamState::Tracking &&
        odometry_odom_body_.has_value() &&
        (map_cloud_map_ != nullptr)) {
      reason_ = "tracking";
      return;
    }
    if (imu_buffer_.empty()) {
      reason_ = "waiting_for_imu";
    } else if (lidar_buffer_.empty()) {
      reason_ = "waiting_for_lidar";
    } else {
      reason_ = "waiting_for_imu_to_cover_scan";
    }
  }

  void recordPatchSnapshot() {
    if (mappingCapacityReached()) return;
    if (!(registered_cloud_body_ != nullptr) || registered_cloud_body_->points.empty()) {
      return;
    }
    if (!odometry_lio_body_.has_value()) {
      return;
    }
    const double stamp = registered_cloud_body_->stamp_s > 0.0
        ? registered_cloud_body_->stamp_s
        : last_stamp_s_;
    const Pose3d pose = *odometry_lio_body_;
    const double min_interval_s = std::max(0.0, runtime_config_.patch_min_interval_s);
    const double min_translation_m = std::max(0.0, runtime_config_.patch_min_translation_m);
    const double min_rotation_rad = std::max(0.0, runtime_config_.patch_min_rotation_rad);
    const bool time_ready =
        last_patch_stamp_s_ <= 0.0 || (stamp - last_patch_stamp_s_) >= min_interval_s;
    const bool motion_ready =
        !has_last_patch_pose_ ||
        std::hypot(pose.x - last_patch_pose_.x,
                   pose.y - last_patch_pose_.y,
                   pose.z - last_patch_pose_.z) >= min_translation_m;
    const bool rotation_ready =
        !has_last_patch_pose_ || rotationDistance(pose, last_patch_pose_) >= min_rotation_rad;
    if (!patch_history_.empty()) {
      if (!time_ready) {
        return;
      }
      if (!motion_ready && !rotation_ready) {
        return;
      }
    }

    auto patch = std::make_shared<PatchSnapshot>();
    patch->sequence = patch_sequence_++;
    patch->observation_sequence = observation_sequence_;
    patch->name = patchName(patch->sequence);
    patch->stamp_s = stamp;
    patch->pose = pose;
    patch->cloud = *registered_cloud_body_;
    patch_history_.push_back(std::move(patch));
    pumpOnlineMappingFrames();
    const std::size_t max_snapshots =
        std::max<std::size_t>(1, runtime_config_.max_patch_snapshots);
    while (patch_history_.size() > max_snapshots) {
      patch_history_.pop_front();
      ++patch_history_dropped_count_;
      if (next_patch_to_queue_ > 0) --next_patch_to_queue_;
    }
    last_patch_stamp_s_ = stamp;
    last_patch_pose_ = pose;
    has_last_patch_pose_ = true;
  }

  void updateOnlineMapping() {
    if (mode_ != SlamMode::Mapping) return;
    auto update = online_mapping_.update();
    if (update) {
      online_mapping_state_ = update->code;
      if (!update->keyframes.empty()) {
        auto cloud = std::make_shared<Cloud>();
        cloud->frame_id = config_.map_frame;
        cloud->stamp_s = update->stamp_s;
        cloud->points.reserve(update->cloud.size());
        const Pose3d tf = has_map_lio_pose_ ? map_lio_pose_ : Pose3d{};
        // One transform per snapshot; keep double precision until the output.
        const Eigen::Matrix3d rotation =
            Eigen::Quaterniond(tf.qw, tf.qx, tf.qy, tf.qz).toRotationMatrix();
        const Eigen::Vector3d translation(tf.x, tf.y, tf.z);
        for (const auto& point : update->cloud) {
          PointXYZIT value;
          const Eigen::Vector3d xyz =
              rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
          value.x = static_cast<float>(xyz.x());
          value.y = static_cast<float>(xyz.y());
          value.z = static_cast<float>(xyz.z());
          value.intensity = point.intensity;
          cloud->points.push_back(value);
        }
        global_mapping_ = std::move(update);
        global_map_cloud_ = std::move(cloud);
      }
    }
    pumpOnlineMappingFrames();
  }

  void removeGeometryDirectory(const std::filesystem::path& path) {
    if (path.empty() || geometry_root_.empty() || path.parent_path() != geometry_root_) return;
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  void clearGeometryExports() {
    if (geometry_export_.valid()) {
      auto completed = geometry_export_.get();
      removeGeometryDirectory(completed.second.frozen_dir);
    }
    if (geometry_pending_) removeGeometryDirectory(geometry_pending_->frozen_dir);
    if (geometry_confirmed_) removeGeometryDirectory(geometry_confirmed_->frozen_dir);
    geometry_pending_.reset();
    geometry_confirmed_.reset();
    geometry_confirmed_optimizations_ = 0U;
    geometry_pending_optimizations_ = 0U;
    geometry_export_optimizations_ = 0U;
    geometry_failed_export_optimizations_=0U;
    if (!geometry_root_.empty()) {
      std::error_code error;
      std::filesystem::remove(geometry_root_, error);
      geometry_root_.clear();
    }
  }

  bool mappingCapacityReached() const {
    return mode_ == SlamMode::Mapping &&
        (patch_history_.size() >= std::min(
             std::max<std::size_t>(1, runtime_config_.max_patch_snapshots),
             online_mapping_.capacity()) ||
         online_mapping_.capacityReached());
  }

  void pumpOnlineMappingFrames() {
    if (mode_ != SlamMode::Mapping) return;
    while (next_patch_to_queue_ < patch_history_.size() && online_mapping_.canEnqueue()) {
      const auto& patch = *patch_history_[next_patch_to_queue_];
      online_mapping::MappingFrame frame;
      frame.keyframe = {patch.name, graphPose(patch.pose)};
      frame.stamp_s = patch.stamp_s;
      frame.body_cloud.reserve(patch.cloud.points.size());
      for (const auto& point : patch.cloud.points)
        frame.body_cloud.push_back({point.x, point.y, point.z, point.intensity});
      const auto queued = online_mapping_.enqueue(std::move(frame));
      if (!queued.ok) {
        online_mapping_state_ = queued.code;
        return;
      }
      ++next_patch_to_queue_;
    }
    if (next_patch_to_queue_ < patch_history_.size() && online_mapping_.capacityReached())
      online_mapping_state_ = "online_mapping_capacity_reached";
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
    if (!world_cloud || mappingCapacityReached()) {
      return;
    }
    const double resolution = runtime_config_.odom_prior_map_resolution > 0.0
        ? runtime_config_.odom_prior_map_resolution
        : frontend_config_.map_resolution;
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



  SlamConfig config_;
  Config frontend_config_;
  RuntimeConfig runtime_config_;
  SlamMode mode_ = SlamMode::Mapping;
  SlamState state_ = SlamState::Unconfigured;
  std::string reason_ = "unconfigured";
  std::string scene_mode_ = "unknown";
  std::string map_path_;
  bool map_bounds_ready_ = false;
  double map_min_x_ = 0.0;
  double map_min_y_ = 0.0;
  double map_min_z_ = 0.0;
  double map_max_x_ = 0.0;
  double map_max_y_ = 0.0;
  double map_max_z_ = 0.0;
  std::unique_ptr<lio::Frontend> frontend_;
  std::optional<lio::Result> frontend_result_;
  Pose3d frontend_anchor_;
  Eigen::Isometry3d frontend_world_from_odom_ = Eigen::Isometry3d::Identity();
  Pose3d lio_from_odom_;
  bool has_lio_odom_correction_ = false;
  std::optional<Pose3d> frontend_initial_pose_;
  bool frontend_anchored_ = false;
  std::uint64_t frontend_attempts_ = 0;
  std::uint64_t frontend_rejections_ = 0;
  std::uint64_t last_published_elevator_exits_ = 0U;
  std::uint64_t elevator_exit_observation_sequence_ = 0U;
  std::string frontend_previous_rejection_ = "none";
  std::shared_ptr<NativeRelocalizer> relocalizer_;
  std::optional<AsyncSaveJob> save_job_;
  std::optional<AsyncRelocalizationJob> async_relocalization_job_;
  std::uint64_t async_relocalization_sequence_ = 0U;
  std::uint64_t relocalization_map_epoch_ = 0U;
  std::deque<BufferedImu> imu_buffer_;
  std::deque<LidarFrame> lidar_buffer_;
  bool lidar_pushed_ = false;
  SyncedScan package_;
  double last_lidar_time_ = -1.0;
  double last_imu_time_ = -1.0;
  double last_stamp_s_ = 0.0;
  double scan_start_s_ = 0.0;
  double scan_end_s_ = 0.0;
  bool alive_ = false;
  bool map_loaded_ = false;
  bool map_frame_jump_ = false;
  bool ltlio_health_fault_active_ = false;
  bool catastrophic_health_fault_latched_ = false;
  std::string catastrophic_health_fault_reason_;
  bool relocalization_required_after_time_jump_ = false;
  std::uint64_t map_frame_jump_sequence_ = 0U;
  bool has_map_lio_pose_ = false;
  // Map alignment consumes full-posterior LIO poses; public map<-odom also
  // composes lio_from_odom_. Saved patches and SAM stay in the LIO frame.
  Pose3d map_lio_pose_;
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
  std::string relocalization_engine_;
  int imu_batch_ = 0;
  int sync_wait_count_ = 0;
  int imu_rollback_count_ = 0;
  int lidar_rollback_count_ = 0;
  int dropped_lidar_frames_ = 0;
  int dropped_imu_frames_ = 0;
  std::optional<Pose3d> odometry_odom_body_;
  std::optional<Pose3d> odometry_lio_body_;
  std::optional<Pose3d> state_estimation_at_scan_;
  std::optional<BodyTwist> odometry_twist_body_;
  std::optional<V3D> external_velocity_world_;
  std::shared_ptr<const Cloud> registered_cloud_body_;
  std::uint64_t observation_sequence_ = 0U;
  std::uint64_t source_epoch_ = newSourceEpoch() - 1U;
  std::shared_ptr<const Cloud> map_cloud_map_;
  std::shared_ptr<const Cloud> saved_map_cloud_map_;
  std::uint64_t saved_map_observation_sequence_ = 0U;
  std::uint64_t saved_map_revision_ = 0;
  online_mapping::Worker online_mapping_;
  std::shared_ptr<const online_mapping::Snapshot> global_mapping_;
  std::filesystem::path geometry_root_;
  std::optional<MapGeometrySnapshot> geometry_pending_;
  std::optional<MapGeometrySnapshot> geometry_confirmed_;
  std::future<std::pair<Status, MapGeometrySnapshot>> geometry_export_;
  std::size_t geometry_confirmed_optimizations_ = 0U;
  std::size_t geometry_pending_optimizations_ = 0U;
  std::size_t geometry_export_optimizations_ = 0U;
  std::size_t geometry_failed_export_optimizations_ = 0U;
  std::shared_ptr<const Cloud> global_map_cloud_;
  std::string online_mapping_state_ = "waiting_for_keyframes";
  int saved_map_points_ = 0;
  std::optional<OdomSample> latest_odom_prior_;
  std::deque<OdomSample> odom_prior_buffer_;
  bool odom_prior_active_ = false;
  double odom_prior_age_s_ = -1.0;
  double odom_prior_error_xy_m_ = -1.0;
  std::unordered_map<VoxelKey, PointType, VoxelKeyHash> odom_prior_map_;
  GnssFusionHealth gnss_health_;
  std::vector<OdomSample> pose_history_;
  std::deque<std::shared_ptr<const PatchSnapshot>> patch_history_;
  std::uint64_t patch_history_dropped_count_ = 0;
  std::uint64_t patch_sequence_ = 0;
  std::size_t next_patch_to_queue_ = 0;
  double last_patch_stamp_s_ = 0.0;
  Pose3d last_patch_pose_;
  bool has_last_patch_pose_ = false;
};

}  // namespace

std::unique_ptr<ISlamBackend> makeLtlioBackend() {
  return std::make_unique<LtlioBackend>();
}

}  // namespace lingtu::slam

#else

namespace lingtu::slam {

std::unique_ptr<ISlamBackend> makeLtlioBackend() {
  return makeContractBackend("ltlio");
}

}  // namespace lingtu::slam

#endif
