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
#include <limits>
#include <Eigen/SVD>

namespace lingtu::localization::online_mapping {
namespace {
gtsam::ISAM2Params parameters() {
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.1;
  p.relinearizeSkip = 1;
  // Preserve the upstream weak global prior without Cholesky's loss of precision.
  p.factorization = gtsam::ISAM2Params::QR;
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
       config.translation_sigma_m, config.rotation_sigma_rad, config.huber_k,
       config.confirm_distance_m, config.consistency_m, config.consistency_rad})
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
  return append(std::move(frame), nullptr);
}

void Backend::restore(std::vector<Frame> frames, const std::vector<LoopResult>& records) {
  if (!frames_.empty() || frames.empty() || frames.size() != records.size())
    throw std::invalid_argument("incomplete mapping archive");
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto& r = records[i];
    if (r.current != i || r.stamp_s != frames[i].stamp_s ||
        (r.accepted && (r.previous < 0 || static_cast<std::size_t>(r.previous) >= i ||
                        !r.measurement.matrix().allFinite())))
      throw std::invalid_argument("invalid archived loop identity");
    if (r.accepted)
      for (double variance : r.variance)
        if (!std::isfinite(variance) || variance <= 0)
          throw std::invalid_argument("invalid archived loop variance");
    append(std::move(frames[i]), &r);
  }
}

LoopResult Backend::append(Frame frame, const LoopResult* recorded) {
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
    // Adapt the upstream continuous mapping odometry input to LTLIO odom.
    // Never remeasure adjacent clouds or difference separately corrected poses.
    const auto delta = frames_.back().odom.between(frame.odom);
    gtsam::Vector6 variance;
    for (int i = 0; i < 6; ++i) variance[i] = config_.odom_variance[i];
    // Across a restart the link is measured by map registration, not LIO.
    if (frame.relocalized) {
      variance.head<3>().setConstant(config_.rotation_sigma_rad * config_.rotation_sigma_rad);
      variance.tail<3>().setConstant(config_.translation_sigma_m * config_.translation_sigma_m);
    }
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
  auto result = recorded ? *recorded : closeLoop();
  if (recorded && recorded->accepted) {
    gtsam::NonlinearFactorGraph loops;
    loops.add(gtsam::BetweenFactor<gtsam::Pose3>(index, recorded->previous,
        recorded->measurement, loopNoise(config_, recorded->variance)));
    isam_.update(loops, gtsam::Values());
    for (int i = 0; i < 6; ++i) isam_.update();
    updateEstimates();
    ++loops_;
  }
  if (!recorded) result.elapsed_ms = std::chrono::duration<double, std::milli>(
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
  std::vector<std::pair<double,std::size_t>> anchors;
  for(std::size_t i=0;i<current;++i) {
    const double distance=(poses_[i].translation()-poses_.back().translation()).norm();
    if(distance<config_.radius_m && frames_.back().stamp_s-frames_[i].stamp_s>config_.min_time_s)
      anchors.push_back({distance,i});
  }
  std::sort(anchors.begin(),anchors.end());
  if(anchors.empty())return result;
  const auto source=submap({current});
  result.source_points=source->size();
  if(source->size()<300){result.reason="insufficient_geometry";return result;}
  struct Hypothesis {
    std::size_t previous;
    std::vector<std::size_t> indices;
    Cloud::Ptr target;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
    double coarse=0;
    Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();
    gtsam::Pose3 mode;
  };
  // One coarse history union per query; local target windows remain authoritative
  // only during the final full ICP and overlap verification.
  std::vector<std::size_t> coarse_indices;
  for(const auto& anchor:anchors)coarse_indices.push_back(anchor.second);
  const auto coarse_target=submap(coarse_indices);
  if(coarse_target->empty()) {
    result.previous=static_cast<int>(anchors.front().second);
    for(int i=std::max(0,result.previous-config_.submap_half_window);
        i<=std::min(static_cast<int>(current)-1,result.previous+config_.submap_half_window);++i)
      if(frames_.back().stamp_s-frames_[i].stamp_s>config_.min_time_s)result.target_frames.push_back(i);
    result.target_points=coarse_target->size();result.reason="insufficient_geometry";return result;
  }
  pcl::search::KdTree<pcl::PointXYZI>::Ptr coarse_tree(new pcl::search::KdTree<pcl::PointXYZI>);
  coarse_tree->setInputCloud(coarse_target);
  std::vector<Hypothesis> hypotheses;
  double discarded_score=std::numeric_limits<double>::infinity();
  for(const auto& anchor:anchors) {
    const auto previous=anchor.second;
    Hypothesis h;h.previous=previous;
    for(int i=std::max(0,static_cast<int>(previous)-config_.submap_half_window);
        i<=std::min(static_cast<int>(current)-1,static_cast<int>(previous)+config_.submap_half_window);++i)
      if(frames_.back().stamp_s-frames_[i].stamp_s>config_.min_time_s)h.indices.push_back(i);
    h.target=coarse_target;h.tree=coarse_tree;
    h.guess.block<3,1>(0,3)=(poses_[previous].translation()-poses_.back().translation()).cast<float>();
    const std::size_t count=std::min(std::size_t(256),source->size());
    std::vector<int> neighbors(1);std::vector<float> distances(1);
    // Three bounded rigid SVD steps are candidate retrieval only. A coarse
    // match never contributes a loop factor or replaces fine registration.
    for(int iteration=0;iteration<3;++iteration) {
      std::vector<Eigen::Vector3f> xs,ys;
      Eigen::Vector3f mx=Eigen::Vector3f::Zero(),my=Eigen::Vector3f::Zero();
      for(std::size_t i=0;i<count;++i) {
        const auto& point=(*source)[i*source->size()/count];
        const Eigen::Vector3f x=h.guess.block<3,3>(0,0)*point.getVector3fMap()+h.guess.block<3,1>(0,3);
        pcl::PointXYZI q;q.getVector3fMap()=x;
        if(h.tree->nearestKSearch(q,1,neighbors,distances)!=1 || distances[0]>config_.correspondence_m*config_.correspondence_m)continue;
        const Eigen::Vector3f y=(*h.target)[neighbors[0]].getVector3fMap();
        xs.push_back(x);ys.push_back(y);mx+=x;my+=y;
      }
      if(xs.size()<6)break;
      mx/=xs.size();my/=ys.size();Eigen::Matrix3f cross=Eigen::Matrix3f::Zero();
      for(std::size_t i=0;i<xs.size();++i)cross+=(xs[i]-mx)*(ys[i]-my).transpose();
      Eigen::JacobiSVD<Eigen::Matrix3f> svd(cross,Eigen::ComputeFullU|Eigen::ComputeFullV);
      if(svd.singularValues()[1]<=1e-6F*svd.singularValues()[0])break;
      Eigen::Matrix3f sign=Eigen::Matrix3f::Identity();
      if((svd.matrixV()*svd.matrixU().transpose()).determinant()<0)sign(2,2)=-1;
      const Eigen::Matrix3f rotation=svd.matrixV()*sign*svd.matrixU().transpose();
      Eigen::Matrix4f step=Eigen::Matrix4f::Identity();step.block<3,3>(0,0)=rotation;step.block<3,1>(0,3)=my-rotation*mx;
      h.guess=(step*h.guess).eval();
    }
    for(std::size_t i=0;i<count;++i) {
      const auto& point=(*source)[i*source->size()/count];pcl::PointXYZI q;
      q.getVector3fMap()=h.guess.block<3,3>(0,0)*point.getVector3fMap()+h.guess.block<3,1>(0,3);
      if(h.tree->nearestKSearch(q,1,neighbors,distances)==1)h.coarse+=distances[0];
    }
    h.coarse/=count;h.mode=gtsam::Pose3(h.guess.cast<double>()*poses_.back().matrix());
    bool represented=false;
    for(auto& existing:hypotheses) {
      const auto delta=existing.mode.between(h.mode);
      if(delta.translation().norm()<config_.consistency_m && gtsam::Rot3::Logmap(delta.rotation()).norm()<config_.consistency_rad) {
        if(h.coarse<existing.coarse)existing=std::move(h);
        represented=true;break;
      }
    }
    if(represented)continue;
    hypotheses.push_back(std::move(h));
    if(hypotheses.size()>32){std::sort(hypotheses.begin(),hypotheses.end(),[](const auto&a,const auto&b){return a.coarse<b.coarse;});discarded_score=std::min(discarded_score,hypotheses.back().coarse);hypotheses.pop_back();}
  }
  result.candidate_count=hypotheses.size();
  if(hypotheses.empty()){result.reason="insufficient_geometry";return result;}
  std::sort(hypotheses.begin(),hypotheses.end(),[](const auto&a,const auto&b){return a.coarse<b.coarse;});
  const double indistinguishable=config_.translation_sigma_m*config_.translation_sigma_m;
  if(discarded_score<=hypotheses[0].coarse+indistinguishable || (hypotheses.size()>2 && hypotheses[2].coarse<=hypotheses[0].coarse+indistinguishable)) {
    result.reason="candidate_budget_pending";
    return result;
  }
  auto verify=[&](const Hypothesis& hypothesis) {
    const auto previous=hypothesis.previous;
    LoopResult result;result.current=current;result.stamp_s=frames_.back().stamp_s;
    result.previous=static_cast<int>(previous);
  result.target_frames=hypothesis.indices;
  const auto target=submap(hypothesis.indices);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr fine_tree(new pcl::search::KdTree<pcl::PointXYZI>);
  fine_tree->setInputCloud(target);
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
  icp.setSearchMethodTarget(fine_tree,true);
  icp.align(unused,hypothesis.guess);
  result.fitness = icp.getFitnessScore();
  result.reason = "icp_not_converged";
  if (!icp.hasConverged()) return result;
  result.reason = "nonfinite_transform";
  if (!icp.getFinalTransformation().allFinite()) return result;
  result.reason = "fitness_rejected";
  if (!std::isfinite(result.fitness) || result.fitness > config_.max_fitness)
    return result;
  const auto& tree=*fine_tree;
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
  result.measurement = gtsam::Pose3(corrected).between(poses_[result.previous]);

    result.reason="valid_geometry";
    return result;
  };

  std::vector<LoopResult> matches;
  const auto count=std::min(std::size_t(2),hypotheses.size());
  for(std::size_t i=0;i<count;++i) {
    auto candidate=verify(hypotheses[i]);
    if(candidate.reason=="valid_geometry")matches.push_back(std::move(candidate));
    else if(matches.empty())result=std::move(candidate);
  }
  if(matches.empty()){result.candidate_count=hypotheses.size();result.verified_candidates=count;return result;}
  std::sort(matches.begin(),matches.end(),[](const auto&a,const auto&b){return a.fitness<b.fitness;});
  result=matches.front();result.candidate_count=hypotheses.size();result.verified_candidates=count;
  const auto best_pose=poses_[result.previous].compose(result.measurement.inverse());
  for(std::size_t i=1;i<matches.size();++i) {
    const auto& other=matches[i];
    if(other.fitness>result.fitness+indistinguishable)continue;
    const auto other_pose=poses_[other.previous].compose(other.measurement.inverse());
    const auto difference=best_pose.between(other_pose);
    if(difference.translation().norm()>config_.consistency_m ||
       gtsam::Rot3::Logmap(difference.rotation()).norm()>config_.consistency_rad) {
      result.reason="ambiguous_place";
      return result;
    }
  }
  const auto noise=loopNoise(config_,result.variance);
  // Two independently moved source/target anchors must close the same SE(3)
  // cycle using raw odometry. Optimized poses must not confirm their own loop.
  result.reason = "confirmation_pending";
  bool confirmed = false;
  for (const auto& earlier : pending_) {
    // The earlier source cannot be recycled as this candidate's historical
    // evidence; confirmation needs four distinct observations.
    if (earlier.current == static_cast<std::size_t>(result.previous)) continue;
    const auto source_delta = frames_[earlier.current].odom.between(frames_[current].odom);
    const auto target_delta = frames_[earlier.previous].odom.between(frames_[result.previous].odom);
    if (source_delta.translation().norm() < config_.confirm_distance_m ||
        target_delta.translation().norm() < config_.confirm_distance_m)
      continue;
    const auto predicted = source_delta.inverse().compose(earlier.measurement).compose(target_delta);
    const auto cycle = predicted.between(result.measurement);
    const double translation = cycle.translation().norm();
    const double rotation = gtsam::Rot3::Logmap(cycle.rotation()).norm();
    result.reason = "confirmation_inconsistent";
    result.consistency_translation_m = translation;
    result.consistency_rotation_rad = rotation;
    if (translation <= config_.consistency_m && rotation <= config_.consistency_rad) {
      result.support_current = static_cast<int>(earlier.current);
      result.support_previous = earlier.previous;
      confirmed = true;
      break;
    }
  }
  // Keep only spatially independent representatives, so dense/stationary
  // keyframes cannot evict all useful confirmation history.
  bool represented = false;
  for (const auto& earlier : pending_) {
    if ((frames_[earlier.current].odom.translation() - frames_[current].odom.translation()).norm()
            < config_.confirm_distance_m &&
        (frames_[earlier.previous].odom.translation() - frames_[result.previous].odom.translation()).norm()
            < config_.confirm_distance_m) {
      represented = true;
      break;
    }
  }
  if (!represented) {
    if (pending_.size() == 32) pending_.erase(pending_.begin());
    pending_.push_back(result);
  }
  if (!confirmed) return result;
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

std::vector<std::size_t> Backend::segmentStarts() const {
  std::vector<std::size_t> result;
  for (std::size_t i = 1; i < frames_.size(); ++i)
    if (frames_[i].relocalized) result.push_back(i);
  return result;
}

void writeLoopEvidence(std::ostream& out, const Config& c, const std::vector<LoopResult>& records,
                       const std::vector<std::size_t>& segment_starts) {
  out << std::setprecision(17)
      << "{\"schema\":\"lingtu.sam_loops.v1\",\"config\":{\"radius_m\":" << c.radius_m
      << ",\"min_time_s\":" << c.min_time_s << ",\"submap_half_window\":" << c.submap_half_window
      << ",\"voxel_m\":" << c.voxel_m << ",\"max_fitness\":" << c.max_fitness
      << ",\"correspondence_m\":" << c.correspondence_m << ",\"inlier_distance_m\":" << c.inlier_distance_m
      << ",\"min_overlap\":" << c.min_overlap << ",\"translation_sigma_m\":" << c.translation_sigma_m
      << ",\"rotation_sigma_rad\":" << c.rotation_sigma_rad << ",\"huber_k\":" << c.huber_k
      << ",\"confirm_distance_m\":" << c.confirm_distance_m
      << ",\"consistency_m\":" << c.consistency_m
      << ",\"consistency_rad\":" << c.consistency_rad
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
        << ",\"support_current\":" << r.support_current
        << ",\"support_previous\":" << r.support_previous
        << ",\"consistency_translation_m\":" << r.consistency_translation_m
        << ",\"consistency_rotation_rad\":" << r.consistency_rotation_rad
        << ",\"source_points\":" << r.source_points << ",\"target_points\":" << r.target_points
        << ",\"candidate_count\":" << r.candidate_count << ",\"verified_candidates\":" << r.verified_candidates
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
  out << "],\"segment_starts\":[";
  for (std::size_t i = 0; i < segment_starts.size(); ++i)
    out << (i ? "," : "") << segment_starts[i];
  out << "]}\n";
}
}  // namespace lingtu::localization::online_mapping
