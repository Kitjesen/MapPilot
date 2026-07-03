#include "native_relocalizer.hpp"

#include "bbs3d_global_localizer.h"
#include "icp_localizer.h"

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
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
  auto out = std::make_shared<LocalizerCloud>();
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

}  // namespace

struct NativeRelocalizer::Impl {
  ICPLocalizer icp{ICPConfig{}};
  BBS3DGlobalLocalizer bbs3d{};
  bool map_loaded = false;
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
  impl_->map_loaded = impl_->icp.loadMap(pcd_path);
  impl_->bbs3d_map_loaded = false;
  if (impl_->map_loaded && impl_->bbs3d.available()) {
    auto cloud = std::make_shared<LocalizerCloud>();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) >= 0) {
      impl_->bbs3d_map_loaded = impl_->bbs3d.set_map(cloud);
    }
  }
  if (message) {
    *message = impl_->map_loaded ? "native_relocalizer_map_loaded"
                                 : "native_relocalizer_map_load_failed";
  }
  return impl_->map_loaded;
}

bool NativeRelocalizer::hasMap() const {
  return impl_ && impl_->map_loaded;
}

bool NativeRelocalizer::supportsSeededRelocalization() const {
  return impl_ != nullptr;
}

bool NativeRelocalizer::supportsGlobalRelocalization() const {
  return impl_ && impl_->bbs3d.available();
}

NativeRelocalizationResult NativeRelocalizer::relocalize(
    const Cloud& scan_body,
    const Pose3d& map_body_guess,
    const Pose3d& odom_body) const {
  NativeRelocalizationResult result;
  if (!impl_ || !impl_->map_loaded) {
    result.message = "native_relocalizer_map_not_loaded";
    return result;
  }
  auto scan = toLocalizerCloud(scan_body);
  if (!scan || scan->size() < 20) {
    result.message = "native_relocalizer_scan_too_small";
    return result;
  }

  M4F map_body = poseToMatrix(map_body_guess);
  impl_->icp.setInput(scan);
  if (!impl_->icp.align(map_body)) {
    result.quality = impl_->icp.getLastFitnessScore();
    result.message = "native_relocalizer_icp_failed";
    return result;
  }

  const Eigen::Matrix4f odom_body_matrix = poseToMatrix(odom_body);
  const Eigen::Matrix4f map_odom = map_body * odom_body_matrix.inverse();
  result.success = true;
  result.message = "native_relocalized";
  result.map_body = matrixToPose(map_body);
  result.map_odom = matrixToPose(map_odom);
  result.quality = impl_->icp.getLastFitnessScore();
  return result;
}

NativeRelocalizationResult NativeRelocalizer::globalRelocalize(
    const Cloud& scan_body,
    const Pose3d& odom_body) const {
  NativeRelocalizationResult result;
  if (!impl_ || !impl_->map_loaded) {
    result.message = "native_relocalizer_map_not_loaded";
    return result;
  }
  if (!impl_->bbs3d.available()) {
    result.message = "native_global_relocalizer_unavailable";
    return result;
  }
  if (!impl_->bbs3d_map_loaded) {
    result.message = "native_global_relocalizer_map_not_loaded";
    return result;
  }
  auto scan = toLocalizerCloud(scan_body);
  if (!scan || scan->size() < 20) {
    result.message = "native_relocalizer_scan_too_small";
    return result;
  }

  const auto coarse = impl_->bbs3d.localize(scan);
  if (!coarse.success) {
    result.message = std::string("native_global_relocalizer_failed: ") + coarse.message;
    return result;
  }

  Eigen::Matrix4f map_body = coarse.pose;
  impl_->icp.setInput(scan);
  if (!impl_->icp.align(map_body)) {
    result.quality = impl_->icp.getLastFitnessScore();
    result.message = "native_global_relocalizer_icp_refine_failed";
    return result;
  }

  const Eigen::Matrix4f odom_body_matrix = poseToMatrix(odom_body);
  const Eigen::Matrix4f map_odom = map_body * odom_body_matrix.inverse();
  result.success = true;
  result.message = "native_global_relocalized";
  result.map_body = matrixToPose(map_body);
  result.map_odom = matrixToPose(map_odom);
  result.quality = impl_->icp.getLastFitnessScore();
  return result;
}

}  // namespace lingtu::slam
