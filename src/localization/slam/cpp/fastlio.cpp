#include "slam.hpp"

#if defined(LINGTU_HAS_FASTLIO2_BACKEND) && LINGTU_HAS_FASTLIO2_BACKEND

#include "map_builder/map_builder.h"
#include "native_relocalizer.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace lingtu::slam {
namespace {

struct RuntimeConfig {
  double acc_scale = 1.0;
  double time_diff_lidar_to_imu = 0.0;
  double max_imu_gap_s = 0.25;
  double relocalization_max_fitness = 0.5;
  double relocalization_map_bounds_margin_m = 2.0;
  bool odom_prior_enabled = false;
  double odom_prior_max_age_s = 0.20;
  double odom_prior_map_resolution = 0.20;
  double odom_prior_snap_translation_m = 0.25;
  bool map_optimization_enabled = true;
  double map_optimization_resolution = 0.20;
  double loop_closure_max_error_m = 1.25;
  double loop_closure_min_path_m = 4.0;
  std::size_t max_imu_buffer = 4000;
  std::size_t max_lidar_buffer = 64;
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
  double stamp_s = 0.0;
  Pose3d pose;
  Cloud cloud;
};

struct LoopClosureCandidate {
  bool found = false;
  std::size_t start_index = 0;
  std::size_t end_index = 0;
  double error_m = -1.0;
  double path_m = 0.0;
};

struct MapOptimizationReport {
  std::string status = "not_run";
  std::string backend = "native_patch_pose_graph";
  std::string refine_backend = "native_voxel_refine";
  bool enabled = true;
  bool loop_closure_enabled = true;
  bool loop_closure_applied = false;
  bool refine_enabled = true;
  bool refine_applied = false;
  int patch_count = 0;
  int pose_count = 0;
  int optimized_pose_count = 0;
  int loop_count = 0;
  int raw_map_points = 0;
  int optimized_map_points = 0;
  double loop_closure_error_m = -1.0;
  std::string raw_map_path;
  std::string optimized_map_path;
  std::string metadata_path;
};

struct OptimizedMapResult {
  Cloud cloud;
  MapOptimizationReport report;
};

struct VoxelAccum {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  int hits = 0;
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
  readIfPresent(config, "odom_prior_enabled", runtime_config.odom_prior_enabled);
  readIfPresent(config, "odom_prior_max_age_s", runtime_config.odom_prior_max_age_s);
  readIfPresent(config, "odom_prior_map_resolution", runtime_config.odom_prior_map_resolution);
  readIfPresent(
      config,
      "odom_prior_snap_translation_m",
      runtime_config.odom_prior_snap_translation_m);
  readIfPresent(config, "map_optimization_enabled", runtime_config.map_optimization_enabled);
  readIfPresent(config, "map_optimization_resolution", runtime_config.map_optimization_resolution);
  readIfPresent(config, "loop_closure_max_error_m", runtime_config.loop_closure_max_error_m);
  readIfPresent(config, "loop_closure_min_path_m", runtime_config.loop_closure_min_path_m);

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
    const Config& config) {
  const Eigen::Matrix3d r_wb = rotationFromPose(body_pose);
  const Eigen::Vector3d t_wb(body_pose.x, body_pose.y, body_pose.z);
  const Eigen::Matrix3d r_wl = r_wb * config.r_il;
  const Eigen::Vector3d t_wl = t_wb + r_wb * config.t_il;
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

Pose3d poseWithYaw(const Pose3d& pose, double yaw) {
  Pose3d out = pose;
  const double half = yaw * 0.5;
  out.qx = 0.0;
  out.qy = 0.0;
  out.qz = std::sin(half);
  out.qw = std::cos(half);
  return out;
}

std::vector<double> cumulativePlanarDistance(const std::vector<PatchSnapshot>& patches) {
  std::vector<double> distance(patches.size(), 0.0);
  for (std::size_t i = 1; i < patches.size(); ++i) {
    distance[i] = distance[i - 1] + planarDistance(patches[i - 1].pose, patches[i].pose);
  }
  return distance;
}

LoopClosureCandidate detectLoopClosure(
    const std::vector<PatchSnapshot>& patches,
    double max_error_m,
    double min_path_m) {
  LoopClosureCandidate best;
  if (patches.size() < 8) {
    return best;
  }
  const std::vector<double> distance = cumulativePlanarDistance(patches);
  const std::size_t end = patches.size() - 1;
  for (std::size_t i = 0; i + 5 < end; ++i) {
    const double path_m = distance[end] - distance[i];
    if (path_m < min_path_m) {
      continue;
    }
    const double error_m = planarDistance(patches[i].pose, patches[end].pose);
    if (error_m > max_error_m) {
      continue;
    }
    if (!best.found || error_m < best.error_m) {
      best.found = true;
      best.start_index = i;
      best.end_index = end;
      best.error_m = error_m;
      best.path_m = path_m;
    }
  }
  return best;
}

std::vector<Pose3d> correctedPatchPoses(
    const std::vector<PatchSnapshot>& patches,
    const LoopClosureCandidate& loop) {
  std::vector<Pose3d> poses;
  poses.reserve(patches.size());
  for (const auto& patch : patches) {
    poses.push_back(patch.pose);
  }
  if (!loop.found || loop.end_index >= patches.size() ||
      loop.start_index >= patches.size() || loop.start_index >= loop.end_index) {
    return poses;
  }
  const std::vector<double> distance = cumulativePlanarDistance(patches);
  const double span = std::max(1e-6, distance[loop.end_index] - distance[loop.start_index]);
  const Pose3d& start = patches[loop.start_index].pose;
  const Pose3d& end = patches[loop.end_index].pose;
  const double dx = start.x - end.x;
  const double dy = start.y - end.y;
  const double dz = start.z - end.z;
  const double dyaw = wrapAngle(yawFromPose(start) - yawFromPose(end));
  for (std::size_t i = loop.start_index; i <= loop.end_index; ++i) {
    const double alpha = std::max(
        0.0,
        std::min(1.0, (distance[i] - distance[loop.start_index]) / span));
    poses[i].x += alpha * dx;
    poses[i].y += alpha * dy;
    poses[i].z += alpha * dz;
    poses[i] = poseWithYaw(poses[i], yawFromPose(poses[i]) + alpha * dyaw);
  }
  return poses;
}

PointXYZIT transformPointWithBodyPose(const PointXYZIT& point, const Pose3d& pose) {
  const Eigen::Matrix3d r_wb = rotationFromPose(pose);
  const Eigen::Vector3d t_wb(pose.x, pose.y, pose.z);
  const Eigen::Vector3d p_body(point.x, point.y, point.z);
  const Eigen::Vector3d p_map = r_wb * p_body + t_wb;
  PointXYZIT out = point;
  out.x = static_cast<float>(p_map.x());
  out.y = static_cast<float>(p_map.y());
  out.z = static_cast<float>(p_map.z());
  out.offset_time_ns = 0;
  return out;
}

CloudType cloudToPclCloud(const Cloud& cloud) {
  CloudType out;
  out.reserve(cloud.points.size());
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    PointType pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.intensity;
    out.push_back(pcl_point);
  }
  out.width = static_cast<std::uint32_t>(out.points.size());
  out.height = 1;
  out.is_dense = false;
  return out;
}

Cloud refinePatchMap(
    const std::vector<PatchSnapshot>& patches,
    const std::vector<Pose3d>& poses,
    double resolution,
    const std::string& frame_id,
    double stamp_s) {
  Cloud out;
  out.stamp_s = stamp_s;
  out.frame_id = frame_id;
  if (patches.empty() || patches.size() != poses.size()) {
    return out;
  }
  const double res = std::max(0.05, resolution);
  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
  for (std::size_t i = 0; i < patches.size(); ++i) {
    for (const auto& point : patches[i].cloud.points) {
      const PointXYZIT map_point = transformPointWithBodyPose(point, poses[i]);
      if (!std::isfinite(map_point.x) ||
          !std::isfinite(map_point.y) ||
          !std::isfinite(map_point.z)) {
        continue;
      }
      const VoxelKey key{
          static_cast<std::int64_t>(std::floor(static_cast<double>(map_point.x) / res)),
          static_cast<std::int64_t>(std::floor(static_cast<double>(map_point.y) / res)),
          static_cast<std::int64_t>(std::floor(static_cast<double>(map_point.z) / res))};
      auto& accum = voxels[key];
      accum.x += map_point.x;
      accum.y += map_point.y;
      accum.z += map_point.z;
      accum.intensity += map_point.intensity;
      ++accum.hits;
    }
  }
  const int min_hits = patches.size() >= 4 ? 2 : 1;
  out.points.reserve(voxels.size());
  for (const auto& item : voxels) {
    const auto& accum = item.second;
    if (accum.hits < min_hits) {
      continue;
    }
    PointXYZIT point;
    point.x = static_cast<float>(accum.x / static_cast<double>(accum.hits));
    point.y = static_cast<float>(accum.y / static_cast<double>(accum.hits));
    point.z = static_cast<float>(accum.z / static_cast<double>(accum.hits));
    point.intensity = static_cast<float>(accum.intensity / static_cast<double>(accum.hits));
    out.points.push_back(point);
  }
  return out;
}

OptimizedMapResult optimizePatchMapForSave(
    const std::filesystem::path& pcd,
    const std::filesystem::path& raw_pcd,
    const std::vector<PatchSnapshot>& patches,
    const RuntimeConfig& runtime_config,
    const Config& builder_config,
    const std::string& frame_id,
    double stamp_s,
    int raw_map_points) {
  OptimizedMapResult result;
  result.report.enabled = runtime_config.map_optimization_enabled;
  result.report.raw_map_path = raw_pcd.string();
  result.report.optimized_map_path = pcd.string();
  result.report.metadata_path = (pcd.parent_path() / "map_optimization.json").string();
  result.report.patch_count = static_cast<int>(patches.size());
  result.report.pose_count = static_cast<int>(patches.size());
  result.report.raw_map_points = raw_map_points;
  if (!runtime_config.map_optimization_enabled) {
    result.report.status = "disabled";
    return result;
  }
  if (patches.empty()) {
    result.report.status = "no_patches";
    return result;
  }

  const LoopClosureCandidate loop = detectLoopClosure(
      patches,
      runtime_config.loop_closure_max_error_m,
      runtime_config.loop_closure_min_path_m);
  result.report.loop_closure_error_m = loop.error_m;
  result.report.loop_closure_applied = loop.found;
  result.report.loop_count = loop.found ? 1 : 0;
  result.report.optimized_pose_count = loop.found ? static_cast<int>(patches.size()) : 0;

  const std::vector<Pose3d> poses = correctedPatchPoses(patches, loop);
  const double resolution = runtime_config.map_optimization_resolution > 0.0
      ? runtime_config.map_optimization_resolution
      : builder_config.map_resolution;
  result.cloud = refinePatchMap(patches, poses, resolution, frame_id, stamp_s);
  result.report.optimized_map_points = static_cast<int>(result.cloud.points.size());
  result.report.refine_applied = !result.cloud.points.empty();
  if (result.cloud.points.empty()) {
    result.report.status = "raw_fallback_empty_optimized_map";
  } else if (loop.found) {
    result.report.status = "optimized_loop_closed";
  } else {
    result.report.status = "optimized_refined_no_loop";
  }
  return result;
}

Status writeMapOptimizationMetadata(
    const std::filesystem::path& map_dir,
    const MapOptimizationReport& report) {
  const auto path = map_dir / "map_optimization.json";
  std::ofstream out(path);
  if (!out) {
    return Status::Error("open_map_optimization_metadata_failed");
  }
  out << std::setprecision(12);
  out << "{\n";
  out << "  \"schema_version\": \"lingtu.slam.map_optimization.v1\",\n";
  out << "  \"status\": \"" << jsonEscape(report.status) << "\",\n";
  out << "  \"enabled\": " << (report.enabled ? "true" : "false") << ",\n";
  out << "  \"backend\": \"" << jsonEscape(report.backend) << "\",\n";
  out << "  \"refine_backend\": \"" << jsonEscape(report.refine_backend) << "\",\n";
  out << "  \"raw_map_path\": \"" << jsonEscape(report.raw_map_path) << "\",\n";
  out << "  \"optimized_map_path\": \"" << jsonEscape(report.optimized_map_path) << "\",\n";
  out << "  \"loop_closure_enabled\": " << (report.loop_closure_enabled ? "true" : "false") << ",\n";
  out << "  \"loop_closure_applied\": " << (report.loop_closure_applied ? "true" : "false") << ",\n";
  out << "  \"loop_count\": " << report.loop_count << ",\n";
  out << "  \"loop_closure_error_m\": " << report.loop_closure_error_m << ",\n";
  out << "  \"refine_enabled\": " << (report.refine_enabled ? "true" : "false") << ",\n";
  out << "  \"refine_applied\": " << (report.refine_applied ? "true" : "false") << ",\n";
  out << "  \"hba_refine_enabled\": " << (report.refine_enabled ? "true" : "false") << ",\n";
  out << "  \"hba_refine_applied\": " << (report.refine_applied ? "true" : "false") << ",\n";
  out << "  \"patch_count\": " << report.patch_count << ",\n";
  out << "  \"pose_count\": " << report.pose_count << ",\n";
  out << "  \"optimized_pose_count\": " << report.optimized_pose_count << ",\n";
  out << "  \"raw_map_points\": " << report.raw_map_points << ",\n";
  out << "  \"optimized_map_points\": " << report.optimized_map_points << "\n";
  out << "}\n";
  return Status::Ok("map_optimization_metadata_written");
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
    if (runtime_config_.odom_prior_enabled) {
      latest_odom_prior_ = sample;
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
    const bool preserve_tracking_on_relocalization_failure =
        state_ == SlamState::Tracking && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const bool map_alignment_update =
        !guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value();
    const std::string reason_before_relocalization = reason_;
    auto fail_relocalization = [&](
        const std::string& relocalization_state,
        const std::string& message,
        bool preserve_tracking) {
      relocalization_state_ = preserve_tracking ? "tracking" : relocalization_state;
      last_relocalization_message_ = message;
      if (preserve_tracking) {
        reason_ = reason_before_relocalization.empty() ? "tracking" : reason_before_relocalization;
      } else {
        state_ = SlamState::Lost;
        reason_ = message;
      }
      return Status::Error(message);
    };
    if (!map_loaded_ || !relocalizer_ || !relocalizer_->hasMap()) {
      return fail_relocalization("map_not_loaded", "map_not_loaded", false);
    }
    std::optional<Pose3d> effective_guess = guess;
    if (!effective_guess.has_value() && has_map_odom_pose_ && odometry_odom_body_.has_value()) {
      effective_guess = composePoses(map_odom_pose_, *odometry_odom_body_);
    }
    if (!effective_guess.has_value() && !relocalizer_->supportsGlobalRelocalization()) {
      return fail_relocalization("initial_pose_required", "initial_pose_required", false);
    }
    if (!registered_cloud_body_.has_value() || registered_cloud_body_->points.empty()) {
      return fail_relocalization(
          "waiting_for_scan",
          "registered_cloud_unavailable",
          preserve_tracking_on_relocalization_failure);
    }
    if (!odometry_odom_body_.has_value()) {
      return fail_relocalization("waiting_for_odometry", "odometry_unavailable", false);
    }

    const NativeRelocalizationResult result = effective_guess.has_value()
        ? relocalizer_->relocalize(*registered_cloud_body_, *effective_guess, *odometry_odom_body_)
        : relocalizer_->globalRelocalize(*registered_cloud_body_, *odometry_odom_body_);
    relocalization_quality_ = result.quality;
    last_relocalization_message_ = result.message;
    if (!result.success) {
      return fail_relocalization(
          "failed",
          result.message,
          preserve_tracking_on_relocalization_failure);
    }
    if (result.quality >= 0.0 && result.quality > runtime_config_.relocalization_max_fitness) {
      return fail_relocalization(
          "rejected",
          "relocalization_fitness_rejected",
          preserve_tracking_on_relocalization_failure);
    }
    if (!poseInsideMapBounds(result.map_body)) {
      return fail_relocalization(
          "rejected",
          "relocalization_outside_map_bounds",
          preserve_tracking_on_relocalization_failure);
    }

    if (map_alignment_update) {
      // Periodic map tracking refines map->odom; resetting Fast-LIO would make
      // odometry briefly drop through INITIALIZING on every correction.
      state_estimation_at_scan_ = odometry_odom_body_;
    } else {
      resetTrackingCoreAtPose(*odometry_odom_body_);
    }
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

    const Pose3d fastlio_pose = poseFromState(kf_->x());
    odometry_odom_body_ = fastlio_pose;
    state_estimation_at_scan_ = odometry_odom_body_;

    std::optional<OdomSample> odom_prior = freshOdomPrior(package_.cloud_end_time);
    if (odom_prior.has_value()) {
      odom_prior_error_xy_m_ = planarDistance(fastlio_pose, odom_prior->odom_body);
      applyPose(kf_->x(), odom_prior->odom_body);
      if (odom_prior->has_velocity &&
          finite(odom_prior->vx) && finite(odom_prior->vy) && finite(odom_prior->vz)) {
        kf_->x().v = V3D(odom_prior->vx, odom_prior->vy, odom_prior->vz);
      }
      odometry_odom_body_ = odom_prior->odom_body;
      state_estimation_at_scan_ = odom_prior->odom_body;
    }

    pose_history_.push_back(OdomSample{package_.cloud_end_time, *odometry_odom_body_});
    if (pose_history_.size() > 10000) {
      pose_history_.erase(pose_history_.begin());
    }

    auto body_cloud = LidarProcessor::transformCloud(
        package_.cloud, kf_->x().r_il, kf_->x().t_il);
    registered_cloud_body_ =
        toContractCloud(body_cloud, package_.cloud_end_time, config_.body_frame);

    auto world_cloud = odom_prior.has_value()
        ? transformLidarCloudWithBodyPose(package_.cloud, odom_prior->odom_body, builder_config_)
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
    const auto raw_pcd = pcd.parent_path() / "map.raw.pcd";
    CloudType saved_cloud;
    const bool save_odom_prior_map =
        runtime_config_.odom_prior_enabled && !odom_prior_map_.empty();
    if (save_odom_prior_map) {
      saved_cloud = odomPriorMapPclCloud();
      if (pcl::io::savePCDFileBinary(raw_pcd.string(), saved_cloud) != 0) {
        return Status::Error("map_pcd_write_failed");
      }
    } else {
      builder_->saveMap(raw_pcd.string());
      if (!std::filesystem::exists(raw_pcd)) {
        return Status::Error("map_pcd_write_failed");
      }
      if (pcl::io::loadPCDFile<PointType>(raw_pcd.string(), saved_cloud) < 0) {
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
    const Status patch_status = writePatchBundle(pcd.parent_path(), patches);
    if (!patch_status.ok) {
      return patch_status;
    }
    OptimizedMapResult optimized = optimizePatchMapForSave(
        pcd,
        raw_pcd,
        patches,
        runtime_config_,
        builder_config_,
        config_.map_frame,
        last_stamp_s_,
        static_cast<int>(saved_cloud.size()));
    last_map_optimization_ = optimized.report;
    if (!optimized.cloud.points.empty()) {
      const Status optimized_write_status = writeContractPcdBinary(pcd, optimized.cloud);
      if (!optimized_write_status.ok) {
        return optimized_write_status;
      }
      saved_cloud = cloudToPclCloud(optimized.cloud);
      saved_map_cloud_map_ = optimized.cloud;
    } else {
      std::filesystem::copy_file(
          raw_pcd,
          pcd,
          std::filesystem::copy_options::overwrite_existing,
          ec);
      if (ec) {
        return Status::Error("copy_raw_map_failed: " + ec.message());
      }
      saved_map_cloud_map_ = save_odom_prior_map
          ? std::optional<Cloud>{odomPriorMapContractCloud(last_stamp_s_)}
          : toContractCloud(std::make_shared<CloudType>(saved_cloud), last_stamp_s_, config_.map_frame);
    }
    if (saved_map_cloud_map_.has_value()) {
      last_map_optimization_.optimized_map_points =
          static_cast<int>(saved_map_cloud_map_->points.size());
    }
    const Status metadata_status =
        writeMapOptimizationMetadata(pcd.parent_path(), last_map_optimization_);
    if (!metadata_status.ok) {
      return metadata_status;
    }
    if (!saved_cloud.empty()) {
      updateMapBounds(saved_cloud);
    }
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
    out.odom_prior_enabled = runtime_config_.odom_prior_enabled;
    out.odom_prior_active = odom_prior_active_;
    out.odom_prior_age_s = odom_prior_age_s_;
    out.odom_prior_error_xy_m = odom_prior_error_xy_m_;
    out.odom_prior_map_points = static_cast<int>(odom_prior_map_.size());
    out.map_optimization_status = last_map_optimization_.status;
    out.map_optimization_backend = last_map_optimization_.backend;
    out.map_optimization_refine_backend = last_map_optimization_.refine_backend;
    out.map_optimization_enabled = last_map_optimization_.enabled;
    out.map_optimization_loop_closure_enabled = last_map_optimization_.loop_closure_enabled;
    out.map_optimization_loop_closure_applied = last_map_optimization_.loop_closure_applied;
    out.map_optimization_refine_enabled = last_map_optimization_.refine_enabled;
    out.map_optimization_refine_applied = last_map_optimization_.refine_applied;
    out.map_optimization_hba_refine_enabled = last_map_optimization_.refine_enabled;
    out.map_optimization_hba_refine_applied = last_map_optimization_.refine_applied;
    out.map_optimization_patch_count = last_map_optimization_.patch_count;
    out.map_optimization_pose_count = last_map_optimization_.pose_count;
    out.map_optimization_optimized_pose_count = last_map_optimization_.optimized_pose_count;
    out.map_optimization_loop_count = last_map_optimization_.loop_count;
    out.map_optimization_raw_map_points = last_map_optimization_.raw_map_points;
    out.map_optimization_optimized_map_points = last_map_optimization_.optimized_map_points;
    out.map_optimization_loop_error_m = last_map_optimization_.loop_closure_error_m;
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
    last_map_optimization_ = MapOptimizationReport{};
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
    latest_odom_prior_.reset();
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    odom_prior_error_xy_m_ = -1.0;
    odom_prior_map_.clear();
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
    latest_odom_prior_.reset();
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

  std::optional<OdomSample> freshOdomPrior(double stamp_s) {
    odom_prior_active_ = false;
    odom_prior_age_s_ = -1.0;
    if (!runtime_config_.odom_prior_enabled || !latest_odom_prior_.has_value()) {
      return std::nullopt;
    }
    const double age_s = std::abs(stamp_s - latest_odom_prior_->stamp_s);
    odom_prior_age_s_ = age_s;
    if (runtime_config_.odom_prior_max_age_s >= 0.0 &&
        age_s > runtime_config_.odom_prior_max_age_s) {
      return std::nullopt;
    }
    odom_prior_active_ = true;
    return latest_odom_prior_;
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
    auto cloud = std::make_shared<CloudType>(odomPriorMapPclCloud());
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
  MapOptimizationReport last_map_optimization_;
  std::optional<OdomSample> latest_odom_prior_;
  bool odom_prior_active_ = false;
  double odom_prior_age_s_ = -1.0;
  double odom_prior_error_xy_m_ = -1.0;
  std::unordered_map<VoxelKey, PointType, VoxelKeyHash> odom_prior_map_;
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
