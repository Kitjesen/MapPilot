#include "native_relocalizer.hpp"

#include "bbs3d_global_localizer.h"
#include "map_icp.hpp"

#include <Eigen/Geometry>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <pcl/io/pcd_io.h>

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
  bool bbs3d_map_loaded = false;
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
  impl_->map_loaded.store(true, std::memory_order_release);
  std::lock_guard<std::mutex> lock(impl_->bbs3d_mutex);
  impl_->bbs3d_map_loaded = false;
  if (impl_->map_loaded && impl_->bbs3d.available()) {
    LocalizerCloud::Ptr cloud(new LocalizerCloud);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) >= 0) {
      impl_->bbs3d_map_loaded = impl_->bbs3d.set_map(cloud);
    }
  }
  if (message) {
    if (impl_->map_loaded) {
      *message = has_semantic_map ? "native_relocalizer_semantic_map_loaded"
                                  : "native_relocalizer_pcd_map_loaded";
    }
  }
  return impl_->map_loaded;
}

bool NativeRelocalizer::hasMap() const {
  return impl_ && impl_->map_loaded.load(std::memory_order_acquire) &&
      impl_->map_icp.hasMap();
}

bool NativeRelocalizer::supportsSeededRelocalization() const {
  return impl_ != nullptr;
}

bool NativeRelocalizer::supportsGlobalRelocalization() const {
  if (!impl_ || !impl_->map_loaded.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->bbs3d_mutex);
  return impl_->bbs3d.available() && impl_->bbs3d_map_loaded;
}

NativeRelocalizationResult NativeRelocalizer::relocalize(
    const Cloud& scan_body,
    const Pose3d& map_body_guess,
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

  const std::uint64_t map_generation = impl_->map_icp.mapGeneration();
  const Eigen::Matrix4f guess = poseToMatrix(map_body_guess);
  const MapIcpResult icp_result =
      impl_->map_icp.verifySeed(scan, guess, map_generation);
  if (!icp_result.success) {
    fillMapIcpDiagnostics(result, icp_result.diagnostics);
    result.message = icp_result.message == "map_icp_generation_mismatch"
        ? "native_relocalizer_map_generation_mismatch"
        : "native_relocalizer_icp_failed";
    return result;
  }

  const Eigen::Matrix4f odom_body_matrix = poseToMatrix(odom_body);
  const Eigen::Matrix4f map_odom = icp_result.map_body * odom_body_matrix.inverse();
  result.success = true;
  result.message = icp_result.message == "map_icp_seed_verified"
      ? "native_relocalized_seed_verified"
      : icp_result.message == "map_icp_seed_planar_refined"
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

  const std::uint64_t map_generation = impl_->map_icp.mapGeneration();
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
    coarse = impl_->bbs3d.localize(scan);
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
