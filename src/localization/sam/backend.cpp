// Derived from Tixiao Shan's LIO-SAM mapOptmization.cpp (BSD-3-Clause).
// Pinned source and copyright: upstream/LICENSE and upstream/UPSTREAM.md.
#include "backend.hpp"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <iomanip>

namespace lingtu::localization::sam {
namespace {
gtsam::ISAM2Params parameters() {
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.1;
  p.relinearizeSkip = 1;
  return p;
}
}

Backend::Backend(Config config) : config_(config), isam_(parameters()) {
  if (!std::isfinite(config.radius_m) || config.radius_m <= 0 ||
      !std::isfinite(config.min_time_s) || config.min_time_s < 0 ||
      config.submap_half_window < 0 || config.submap_half_window > 100 ||
      !std::isfinite(config.voxel_m) || config.voxel_m <= 0 ||
      !std::isfinite(config.max_fitness) || config.max_fitness <= 0)
    throw std::invalid_argument("invalid LIO-SAM configuration");
  for (double value : {config.correspondence_m, config.inlier_distance_m,
       config.translation_sigma_m, config.rotation_sigma_rad, config.huber_k})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid loop verification configuration");
  if (!std::isfinite(config.min_overlap) || config.min_overlap <= 0 ||
      config.min_overlap > 1 || config.inlier_distance_m > config.correspondence_m)
    throw std::invalid_argument("invalid loop overlap configuration");
  for (double variance : config.odom_variance)
    if (!std::isfinite(variance) || variance <= 0)
      throw std::invalid_argument("invalid LIO-SAM odometry variance");
}

LoopResult Backend::append(Frame frame) {
  if (!std::isfinite(frame.stamp_s) || !frame.odom.matrix().allFinite() ||
      !frame.cloud || frame.cloud->empty() ||
      (!frames_.empty() && frame.stamp_s <= frames_.back().stamp_s))
    throw std::invalid_argument("invalid or discontinuous LIO-SAM frame");
  const auto index = frames_.size();
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  if (index == 0) {
    const auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector6() << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, frame.odom, noise));
    initial.insert(0, frame.odom);
  } else {
    // Adapt the upstream continuous mapping odometry input to Fast-LIO odom.
    // Never remeasure adjacent clouds or difference separately corrected poses.
    const auto delta = frames_.back().odom.between(frame.odom);
    gtsam::Vector6 variance;
    for (int i = 0; i < 6; ++i) variance[i] = config_.odom_variance[i];
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1, index, delta,
        gtsam::noiseModel::Diagonal::Variances(variance)));
    initial.insert(index, poses_.back().compose(delta));
  }
  isam_.update(factors, initial);
  isam_.update();
  frames_.push_back(std::move(frame));
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(config_.voxel_m, config_.voxel_m, config_.voxel_m);
  voxel.setInputCloud(frames_.back().cloud);
  voxel.filter(*filtered);
  filtered_.push_back(filtered);
  updateEstimates();
  const auto started = std::chrono::steady_clock::now();
  auto result = closeLoop();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-started).count();
  records_.push_back(result);
  return result;
}

void Backend::updateEstimates() {
  const auto estimate = isam_.calculateEstimate();
  poses_.resize(frames_.size());
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    poses_[i] = estimate.at<gtsam::Pose3>(i);
    if (!poses_[i].matrix().allFinite()) throw std::runtime_error("nonfinite iSAM2 estimate");
  }
}

Cloud::Ptr Backend::submap(const std::vector<std::size_t>& indices) const {
  Cloud::Ptr accumulated(new Cloud), filtered(new Cloud);
  for (auto i : indices) {
    Cloud transformed;
    const Eigen::Matrix4f transform = poses_[i].matrix().cast<float>();
    pcl::transformPointCloud(*filtered_[i], transformed, transform);
    *accumulated += transformed;
  }
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(config_.voxel_m, config_.voxel_m, config_.voxel_m);
  voxel.setInputCloud(accumulated);
  voxel.filter(*filtered);
  return filtered;
}

LoopResult Backend::closeLoop() {
  LoopResult result;
  const auto current = frames_.size()-1;
  result.current = current;
  result.stamp_s = frames_.back().stamp_s;
  double nearest = config_.radius_m;
  for (std::size_t i = 0; i < current; ++i) {
    const double distance = (poses_[i].translation()-poses_.back().translation()).norm();
    if (distance < nearest && frames_.back().stamp_s-frames_[i].stamp_s > config_.min_time_s) {
      nearest = distance;
      result.previous = static_cast<int>(i);
    }
  }
  if (result.previous < 0) return result;
  for (int i = std::max(0, result.previous-config_.submap_half_window);
       i <= std::min(static_cast<int>(current)-1, result.previous+config_.submap_half_window); ++i)
    if (frames_.back().stamp_s-frames_[i].stamp_s > config_.min_time_s)
      result.target_frames.push_back(static_cast<std::size_t>(i));
  const auto source = submap({current});
  const auto target = submap(result.target_frames);
  result.source_points = source->size(); result.target_points = target->size();
  result.reason = "insufficient_geometry";
  if (source->size() < 300 || target->size() < 1000) return result;
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(config_.correspondence_m);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(source); icp.setInputTarget(target);
  Cloud unused;
  icp.align(unused);
  result.fitness = icp.getFitnessScore();
  result.reason = "icp_not_converged";
  if (!icp.hasConverged()) return result;
  result.reason = "nonfinite_transform";
  if (!icp.getFinalTransformation().allFinite()) return result;
  result.reason = "fitness_rejected";
  if (!std::isfinite(result.fitness) || result.fitness > config_.max_fitness)
    return result;
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1); std::vector<float> distances(1);
  std::size_t inliers = 0; double squared_error = 0;
  for (const auto& point : unused) {
    if (tree.nearestKSearch(point, 1, indices, distances) == 1 &&
        distances[0] <= config_.inlier_distance_m*config_.inlier_distance_m) {
      ++inliers; squared_error += distances[0];
    }
  }
  result.overlap = static_cast<double>(inliers)/source->size();
  result.inlier_rmse_m = inliers ? std::sqrt(squared_error/inliers) : -1;
  result.reason = "overlap_rejected";
  if (result.overlap < config_.min_overlap) return result;
  const Eigen::Matrix4d corrected = icp.getFinalTransformation().cast<double>() * poses_.back().matrix();
  const gtsam::Pose3 correction(icp.getFinalTransformation().cast<double>());
  result.correction_translation_m =
      (corrected.block<3,1>(0,3)-poses_.back().translation()).norm();
  result.correction_rotation_rad = gtsam::Rot3::Logmap(correction.rotation()).norm();
  // Separate physical units. Fixed angular uncertainty is deliberately not
  // inferred from Euclidean ICP fitness. Residual inflates translation only.
  const double translation_variance = std::max(
      config_.translation_sigma_m*config_.translation_sigma_m,
      result.inlier_rmse_m*result.inlier_rmse_m);
  for (int i=0;i<6;++i) result.variance[i] = i<3
      ? config_.rotation_sigma_rad*config_.rotation_sigma_rad : translation_variance;
  const auto noise = loopNoise(config_, result.variance);
  result.measurement = gtsam::Pose3(corrected).between(poses_[result.previous]);
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(current, result.previous,
      result.measurement, noise));
  isam_.update(factors, gtsam::Values());
  isam_.update();
  for (int i = 0; i < 5; ++i) isam_.update();
  updateEstimates();
  ++loops_;
  result.accepted = true;
  result.reason = "accepted";
  return result;
}

double Backend::error() const {
  return isam_.getFactorsUnsafe().error(isam_.calculateEstimate());
}

gtsam::SharedNoiseModel loopNoise(const Config& config, const std::array<double,6>& variance) {
  gtsam::Vector6 values;
  for (int i=0;i<6;++i) values[i]=variance[i];
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(config.huber_k),
      gtsam::noiseModel::Diagonal::Variances(values));
}

std::size_t Backend::cloudBytes() const {
  std::size_t bytes = 0;
  for (std::size_t i=0;i<frames_.size();++i)
    bytes += (frames_[i].cloud->points.capacity()+filtered_[i]->points.capacity())*sizeof(pcl::PointXYZI);
  return bytes;
}

void writeLoopEvidence(std::ostream& out, const Config& c, const std::vector<LoopResult>& records) {
  out << std::setprecision(17)
      << "{\"schema\":\"lingtu.sam_loops.v1\",\"config\":{\"radius_m\":" << c.radius_m
      << ",\"min_time_s\":" << c.min_time_s << ",\"submap_half_window\":" << c.submap_half_window
      << ",\"voxel_m\":" << c.voxel_m << ",\"max_fitness\":" << c.max_fitness
      << ",\"correspondence_m\":" << c.correspondence_m << ",\"inlier_distance_m\":" << c.inlier_distance_m
      << ",\"min_overlap\":" << c.min_overlap << ",\"translation_sigma_m\":" << c.translation_sigma_m
      << ",\"rotation_sigma_rad\":" << c.rotation_sigma_rad << ",\"huber_k\":" << c.huber_k
      << ",\"odom_variance\":[";
  for (int i=0;i<6;++i) out << (i?",":"") << c.odom_variance[i];
  out << "]},\"records\":[";
  for (std::size_t i=0;i<records.size();++i) {
    const auto& r=records[i];
    out << (i?",":"") << "{\"current\":" << r.current << ",\"previous\":" << r.previous
        << ",\"stamp_s\":" << r.stamp_s << ",\"accepted\":" << (r.accepted?"true":"false")
        << ",\"reason\":\"" << r.reason << "\",\"fitness\":";
    if (std::isfinite(r.fitness)) out << r.fitness; else out << "null";
    out << ",\"overlap\":" << r.overlap << ",\"inlier_rmse_m\":" << r.inlier_rmse_m
        << ",\"source_points\":" << r.source_points << ",\"target_points\":" << r.target_points
        << ",\"elapsed_ms\":" << r.elapsed_ms
        << ",\"correction_translation_m\":" << r.correction_translation_m
        << ",\"correction_rotation_rad\":" << r.correction_rotation_rad << ",\"target_frames\":[";
    for (std::size_t j=0;j<r.target_frames.size();++j) out << (j?",":"") << r.target_frames[j];
    out << "],\"variance\":[";
    for (int j=0;j<6;++j) out << (j?",":"") << r.variance[j];
    const auto q=r.measurement.rotation().toQuaternion();
    out << "],\"measurement_xyz_qwxyz\":[" << r.measurement.x() << ',' << r.measurement.y() << ','
        << r.measurement.z() << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << "]}";
  }
  out << "]}\n";
}
}  // namespace lingtu::localization::sam
