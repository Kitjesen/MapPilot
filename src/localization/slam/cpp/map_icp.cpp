#include "map_icp.hpp"

#include <cmath>
#include <limits>
#include <memory>

#include "semantic_map_client.hpp"

namespace lingtu::slam {

MapIcp::MapIcp(const ICPConfig &config)
    : config_(config), icp_(std::make_unique<ICPLocalizer>(config_)) {}

bool MapIcp::loadMap(const std::string &pcd_path) {
  auto staged = std::make_unique<ICPLocalizer>(config_);
  if (!staged->loadMap(pcd_path)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = "map_icp_pcd_load_failed";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (map_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    last_error_ = "map_icp_generation_exhausted";
    return false;
  }
  icp_.swap(staged);
  map_loaded_ = true;
  ++map_generation_;
  artifact_generation_ = 0U;
  map_source_ = "pcd";
  last_error_.clear();
  return true;
}

bool MapIcp::loadSemanticMap(const std::string &semantic_map_path) {
  SemanticMapClient client;
  SemanticMapSnapshot snapshot;
  std::string error;
  if (!client.load(semantic_map_path, &snapshot, &error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error.empty() ? "semantic_map_load_failed" : error;
    return false;
  }
  auto cloud = std::make_shared<CloudType>();
  cloud->reserve(snapshot.points.size());
  for (const auto &source : snapshot.points) {
    if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
      continue;
    }
    PointType point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.intensity = static_cast<float>(source.label);
    cloud->push_back(point);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = false;

  auto staged = std::make_unique<ICPLocalizer>(config_);
  if (!staged->setMap(cloud)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = "semantic_map_icp_target_empty";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (map_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    last_error_ = "map_icp_generation_exhausted";
    return false;
  }
  icp_.swap(staged);
  map_loaded_ = true;
  ++map_generation_;
  artifact_generation_ = snapshot.generation;
  map_source_ = "semantic_map";
  last_error_.clear();
  return true;
}

bool MapIcp::hasMap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_loaded_;
}

std::uint64_t MapIcp::mapGeneration() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_generation_;
}

std::string MapIcp::lastError() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

std::string MapIcp::mapSource() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_source_;
}

MapIcpResult MapIcp::refine(const CloudType::Ptr &scan_body, const M4F &map_body_guess,
                            std::uint64_t expected_generation) {
  MapIcpResult result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.map_generation = map_generation_;
  if (!map_loaded_) {
    result.message = "map_icp_map_not_loaded";
    return result;
  }
  if (expected_generation != map_generation_) {
    result.message = "map_icp_generation_mismatch";
    return result;
  }
  if (!scan_body || scan_body->empty()) {
    result.message = "map_icp_scan_empty";
    return result;
  }

  M4F refined = map_body_guess;
  icp_->setInput(scan_body);
  if (!icp_->align(refined)) {
    fillDiagnostics(result, *icp_);
    result.message = "map_icp_failed";
    return result;
  }

  result.success = true;
  result.message = "map_icp_refined";
  result.map_body = refined;
  fillDiagnostics(result, *icp_);
  return result;
}

MapIcpResult MapIcp::verifySeed(
    const CloudType::Ptr &scan_body,
    const M4F &map_body_guess,
    std::uint64_t expected_generation) {
  MapIcpResult result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.map_generation = map_generation_;
  if (!map_loaded_) {
    result.message = "map_icp_map_not_loaded";
    return result;
  }
  if (expected_generation != map_generation_) {
    result.message = "map_icp_generation_mismatch";
    return result;
  }
  if (!scan_body || scan_body->empty() || !map_body_guess.allFinite()) {
    result.message = "map_icp_scan_or_seed_invalid";
    return result;
  }

  icp_->setInput(scan_body);
  const double correspondence_distance = std::max(
      0.15,
      2.0 * std::max(
          config_.refine_scan_resolution,
          config_.refine_map_resolution));
  if (!icp_->evaluate(map_body_guess, correspondence_distance)) {
    fillDiagnostics(result, *icp_);
    result.message = "map_icp_seed_unverified";
    return result;
  }
  fillDiagnostics(result, *icp_);
  result.diagnostics.refine_backend = "fixed_transform_seed_check";
  const int evaluated_points = icp_->getLastEvaluatedPoints();
  const double inlier_ratio = evaluated_points > 0
      ? static_cast<double>(result.diagnostics.refine_inliers) /
          static_cast<double>(evaluated_points)
      : 0.0;
  constexpr double kMaxSeedRmseM = 0.12;
  const double max_seed_mse = std::min(
      kMaxSeedRmseM * kMaxSeedRmseM,
      config_.refine_score_thresh);
  if (result.diagnostics.quality < 0.0 ||
      result.diagnostics.quality > max_seed_mse ||
      inlier_ratio < 0.80) {
    result.message = "map_icp_seed_quality_rejected";
    return result;
  }
  M4F refined = map_body_guess;
  if (icp_->alignPlanar(refined, correspondence_distance)) {
    fillDiagnostics(result, *icp_);
    const int refined_points = icp_->getLastEvaluatedPoints();
    const double refined_inlier_ratio = refined_points > 0
        ? static_cast<double>(result.diagnostics.refine_inliers) /
            static_cast<double>(refined_points)
        : 0.0;
    if (result.diagnostics.quality >= 0.0 &&
        result.diagnostics.quality <= max_seed_mse &&
        refined_inlier_ratio >= 0.80) {
      result.success = true;
      result.message = "map_icp_seed_planar_refined";
      result.map_body = refined;
      result.diagnostics.refine_backend = "fixed_seed_planar_icp";
      return result;
    }
    icp_->evaluate(map_body_guess, correspondence_distance);
    fillDiagnostics(result, *icp_);
    result.diagnostics.refine_backend = "fixed_transform_seed_check";
  }
  result.success = true;
  result.message = "map_icp_seed_verified";
  result.map_body = map_body_guess;
  return result;
}

void MapIcp::fillDiagnostics(MapIcpResult &result, const ICPLocalizer &icp) {
  result.diagnostics.quality = icp.getLastFitnessScore();
  result.diagnostics.refine_backend = icp.getBackendName();
  result.diagnostics.refine_iterations = icp.getLastIterations();
  result.diagnostics.refine_inliers = icp.getLastInliers();
  result.diagnostics.input_points = icp.getLastInputPoints();
  result.diagnostics.evaluated_points = icp.getLastEvaluatedPoints();
  result.diagnostics.support_ratio =
      result.diagnostics.input_points > 0
      ? static_cast<double>(result.diagnostics.evaluated_points) /
          static_cast<double>(result.diagnostics.input_points)
      : -1.0;
  result.diagnostics.overlap_inlier_ratio =
      result.diagnostics.evaluated_points > 0 &&
          result.diagnostics.refine_inliers >= 0
      ? static_cast<double>(result.diagnostics.refine_inliers) /
          static_cast<double>(result.diagnostics.evaluated_points)
      : -1.0;
  result.diagnostics.refine_converged = icp.getLastConverged();
  result.diagnostics.refine_pos_cov_trace = icp.getLastPosCovTrace();
}

}  // namespace lingtu::slam
