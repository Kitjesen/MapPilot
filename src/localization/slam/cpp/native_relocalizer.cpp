#include "native_relocalizer.hpp"

#include "bbs3d_global_localizer.h"
#include "map_icp.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <vector>

namespace lingtu::slam {
namespace {

using LocalizerCloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Matrix4f poseToMatrix(const Pose3d& pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  if (!std::isfinite(q.norm()) || q.norm() <= 0.0) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
  out(0, 3) = static_cast<float>(pose.x);
  out(1, 3) = static_cast<float>(pose.y);
  out(2, 3) = static_cast<float>(pose.z);
  return out;
}

Eigen::Matrix3d bodyToGravityFrame(const Pose3d& odom_body) {
  const Eigen::Matrix3d odom_body_rotation =
      poseToMatrix(odom_body).block<3, 3>(0, 0).cast<double>();
  const double yaw = std::atan2(odom_body_rotation(1, 0), odom_body_rotation(0, 0));
  return Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
      * odom_body_rotation;
}

Eigen::Matrix4f withMeasuredTilt(Eigen::Matrix4f map_body, const Pose3d& odom_body) {
  const double yaw = std::atan2(map_body(1, 0), map_body(0, 0));
  map_body.block<3, 3>(0, 0) = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
      * bodyToGravityFrame(odom_body)).cast<float>();
  return map_body;
}

Pose3d matrixToPose(const Eigen::Matrix4f& matrix) {
  Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond q(rotation);
  q.normalize();
  Pose3d out;
  out.x = matrix(0, 3);
  out.y = matrix(1, 3);
  out.z = matrix(2, 3);
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

LocalizerCloud::Ptr toLocalizerCloud(const Cloud& cloud) {
  LocalizerCloud::Ptr out(new LocalizerCloud);
  out->reserve(cloud.points.size());
  for (const auto& src : cloud.points) {
    if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z)) {
      continue;
    }
    pcl::PointXYZI point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.intensity = src.intensity;
    out->push_back(point);
  }
  out->width = static_cast<std::uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = false;
  return out;
}

std::vector<Pose3d> savedPoseCandidates(const std::filesystem::path& map_path) {
  std::ifstream input(map_path.parent_path() / "poses.txt");
  if (!input) return {};
  std::vector<Pose3d> endpoints;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    std::string name;
    Pose3d pose;
    if (!(row >> name >> pose.x >> pose.y >> pose.z >> pose.qw >> pose.qx
              >> pose.qy >> pose.qz) ||
        !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.z) || !std::isfinite(pose.qw) ||
        !std::isfinite(pose.qx) || !std::isfinite(pose.qy) ||
        !std::isfinite(pose.qz)) {
      return {};
    }
    if (endpoints.empty()) endpoints.push_back(pose);
    else if (endpoints.size() == 1U) endpoints.push_back(pose);
    else endpoints.back() = pose;
  }
  if (!input.eof()) return {};
  if (endpoints.size() == 2U) std::swap(endpoints.front(), endpoints.back());
  return endpoints;
}

void fillMapIcpDiagnostics(
    NativeRelocalizationResult& result,
    const MapIcpDiagnostics& diagnostics) {
  result.quality = diagnostics.quality;
  result.refine_backend = diagnostics.refine_backend;
  result.refine_iterations = diagnostics.refine_iterations;
  result.refine_inliers = diagnostics.refine_inliers;
  result.input_points = diagnostics.input_points;
  result.evaluated_points = diagnostics.evaluated_points;
  result.support_ratio = diagnostics.support_ratio;
  result.overlap_inlier_ratio = diagnostics.overlap_inlier_ratio;
  result.refine_converged = diagnostics.refine_converged;
  result.refine_pos_cov_trace = diagnostics.refine_pos_cov_trace;
}

}  // namespace

struct NativeRelocalizer::Impl {
  MapIcp map_icp{ICPConfig{}};
  BBS3DGlobalLocalizer bbs3d{};
  mutable std::mutex bbs3d_mutex;
  std::atomic<bool> map_loaded{false};
  std::atomic<bool> global_supported{false};
  bool bbs3d_map_loaded = false;
  std::vector<Pose3d> saved_pose_candidates;
};

NativeRelocalizer::NativeRelocalizer() : impl_(std::make_unique<Impl>()) {}

NativeRelocalizer::~NativeRelocalizer() = default;

bool NativeRelocalizer::loadMap(const std::string& pcd_path, std::string* message) {
  if (!impl_) {
    if (message) {
      *message = "native_relocalizer_not_initialized";
    }
    return false;
  }
  impl_->map_loaded.store(false, std::memory_order_release);
  impl_->global_supported.store(false, std::memory_order_release);
  const std::filesystem::path pcd(pcd_path);
  const std::filesystem::path semantic_map = pcd.parent_path() / "semantic_map.bin";
  const bool has_semantic_map = std::filesystem::is_regular_file(semantic_map);
  const bool loaded = has_semantic_map
      ? impl_->map_icp.loadSemanticMap(semantic_map.string())
      : impl_->map_icp.loadMap(pcd_path);
  if (!loaded) {
    if (message) {
      *message = has_semantic_map
          ? "native_relocalizer_semantic_map_load_failed: " + impl_->map_icp.lastError()
          : "native_relocalizer_pcd_map_load_failed: " + impl_->map_icp.lastError();
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->bbs3d_mutex);
  impl_->saved_pose_candidates = savedPoseCandidates(pcd);
  impl_->bbs3d_map_loaded = false;
  if (impl_->bbs3d.available()) {
    LocalizerCloud::Ptr cloud(new LocalizerCloud);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) >= 0) {
      impl_->bbs3d_map_loaded = impl_->bbs3d.set_map(cloud);
    }
  }
  impl_->global_supported.store(
      !impl_->saved_pose_candidates.empty() || impl_->bbs3d_map_loaded,
      std::memory_order_release);
  impl_->map_loaded.store(true, std::memory_order_release);
  if (message) {
    if (impl_->map_loaded) {
      *message = has_semantic_map ? "native_relocalizer_semantic_map_loaded"
                                  : "native_relocalizer_pcd_map_loaded";
    }
  }
  return impl_->map_loaded;
}

bool NativeRelocalizer::hasMap() const {
  // Runtime status must not wait on the worker's ICP/BBS computation locks.
  return impl_ && impl_->map_loaded.load(std::memory_order_acquire);
}

bool NativeRelocalizer::supportsSeededRelocalization() const {
  return impl_ != nullptr;
}

bool NativeRelocalizer::supportsGlobalRelocalization() const {
  return hasMap() && impl_->global_supported.load(std::memory_order_acquire);
}

NativeRelocalizationResult NativeRelocalizer::relocalize(
    const Cloud& scan_body,
    const Pose3d& map_body_guess,
    const Pose3d& odom_body,
    bool refine_prediction) const {
  NativeRelocalizationResult result;
  result.engine = "seeded_gicp";
  if (!impl_ || !impl_->map_loaded.load(std::memory_order_acquire)) {
    result.message = "native_relocalizer_map_not_loaded";
    return result;
  }
  auto scan = toLocalizerCloud(scan_body);
  if (!scan || scan->size() < 20) {
    result.message = "native_relocalizer_scan_too_small";
    return result;
  }

  const std::uint64_t map_generation = impl_->map_icp.mapGeneration();
  Eigen::Matrix4f guess = poseToMatrix(map_body_guess);
  if (!refine_prediction && std::abs(map_body_guess.qx) < 1e-12 &&
      std::abs(map_body_guess.qy) < 1e-12) {
    // The public XY(Z)/yaw hint omits body tilt. Map and odometry are gravity
    // aligned; use measured roll/pitch without copying the odometry altitude
    // into a possibly different map floor. Full-attitude seeds stay intact.
    guess = withMeasuredTilt(guess, odom_body);
  }
  // A prediction from an accepted map<-odom alignment needs local drift
  // correction. Initial poses use bounded, gravity-preserving initialization.
  const MapIcpResult icp_result = refine_prediction
      ? impl_->map_icp.refine(scan, guess, map_generation)
      : impl_->map_icp.verifySeed(scan, guess, map_generation);
  if (!icp_result.success) {
    fillMapIcpDiagnostics(result, icp_result.diagnostics);
    result.message = icp_result.message == "map_icp_generation_mismatch"
        ? "native_relocalizer_map_generation_mismatch"
        : icp_result.message == "map_icp_seed_outside_search"
            ? "native_relocalizer_seed_outside_search"
            : "native_relocalizer_icp_failed";
    return result;
  }

  const Eigen::Matrix4f odom_body_matrix = poseToMatrix(odom_body);
  const Eigen::Matrix4f map_odom = icp_result.map_body * odom_body_matrix.inverse();
  result.success = true;
  result.message = icp_result.message == "map_icp_seed_planar_refined"
      ? "native_relocalized_seed_planar_refined"
      : "native_relocalized";
  result.map_body = matrixToPose(icp_result.map_body);
  result.map_odom = matrixToPose(map_odom);
  fillMapIcpDiagnostics(result, icp_result.diagnostics);
  return result;
}

NativeRelocalizationResult NativeRelocalizer::globalRelocalize(
    const Cloud& scan_body,
    const Pose3d& odom_body) const {
  NativeRelocalizationResult result;
  if (!impl_ || !impl_->map_loaded.load(std::memory_order_acquire)) {
    result.message = "native_relocalizer_map_not_loaded";
    return result;
  }
  auto scan = toLocalizerCloud(scan_body);
  if (!scan || scan->size() < 20) {
    result.message = "native_relocalizer_scan_too_small";
    return result;
  }

  std::vector<Pose3d> saved_candidates;
  {
    std::lock_guard<std::mutex> lock(impl_->bbs3d_mutex);
    saved_candidates = impl_->saved_pose_candidates;
  }
  std::optional<NativeRelocalizationResult> saved_match;
  result.engine = saved_candidates.empty() ? "bbs3d_gicp" : "saved_pose_icp";
  for (const Pose3d& candidate : saved_candidates) {
    // A saved endpoint suggests position and heading, not the current stance.
    const Pose3d current_stance = matrixToPose(
        withMeasuredTilt(poseToMatrix(candidate), odom_body));
    auto verified = relocalize(scan_body, current_stance, odom_body);
    verified.engine = "saved_pose_icp";
    if (!verified.success) {
      if (verified.message == "native_relocalizer_map_generation_mismatch") return verified;
      continue;
    }
    if (saved_match) {
      const double separation = std::hypot(
          verified.map_body.x - saved_match->map_body.x,
          verified.map_body.y - saved_match->map_body.y);
      const double orientation_dot = std::abs(
          verified.map_body.qw * saved_match->map_body.qw +
          verified.map_body.qx * saved_match->map_body.qx +
          verified.map_body.qy * saved_match->map_body.qy +
          verified.map_body.qz * saved_match->map_body.qz);
      if (separation > 0.75 ||
          std::abs(verified.map_body.z - saved_match->map_body.z) > 0.2 ||
          orientation_dot < std::cos(0.17453292519943295)) {
        result.message = "native_saved_pose_ambiguous";
        return result;
      }
    }
    if (!saved_match || verified.quality < saved_match->quality) {
      saved_match = std::move(verified);
    }
  }
  if (saved_match) {
    saved_match->message = "native_relocalized_saved_pose_verified";
    saved_match->engine = "saved_pose_icp";
    return *saved_match;
  }

  const std::uint64_t map_generation = impl_->map_icp.mapGeneration();
  result.engine = "bbs3d_gicp";
  BBS3DGlobalLocalizer::Result coarse;
  {
    std::lock_guard<std::mutex> lock(impl_->bbs3d_mutex);
    if (!impl_->bbs3d.available()) {
      result.message = "native_global_relocalizer_unavailable";
      return result;
    }
    if (!impl_->bbs3d_map_loaded) {
      result.message = "native_global_relocalizer_map_not_loaded";
      return result;
    }
    coarse = impl_->bbs3d.localize(scan, bodyToGravityFrame(odom_body));
  }
  if (!coarse.success) {
    result.message = std::string("native_global_relocalizer_failed: ") + coarse.message;
    return result;
  }

  const MapIcpResult icp_result =
      impl_->map_icp.refine(scan, coarse.pose, map_generation);
  if (!icp_result.success) {
    fillMapIcpDiagnostics(result, icp_result.diagnostics);
    result.message = icp_result.message == "map_icp_generation_mismatch"
        ? "native_global_relocalizer_map_generation_mismatch"
        : "native_global_relocalizer_icp_refine_failed";
    return result;
  }
  if (icp_result.diagnostics.quality < 0.0 ||
      icp_result.diagnostics.quality > 0.0144 ||
      icp_result.diagnostics.overlap_inlier_ratio < 0.80) {
    fillMapIcpDiagnostics(result, icp_result.diagnostics);
    result.message = "native_global_relocalizer_quality_rejected";
    return result;
  }

  const Eigen::Matrix4f odom_body_matrix = poseToMatrix(odom_body);
  const Eigen::Matrix4f map_odom = icp_result.map_body * odom_body_matrix.inverse();
  result.success = true;
  result.message = "native_global_relocalized";
  result.map_body = matrixToPose(icp_result.map_body);
  result.map_odom = matrixToPose(map_odom);
  fillMapIcpDiagnostics(result, icp_result.diagnostics);
  return result;
}

}  // namespace lingtu::slam
