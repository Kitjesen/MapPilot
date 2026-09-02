#include "navigation/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

bool finitePoint(const nav_kernel::Vec3& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double normalizeAngle(double angle) {
  return std::remainder(angle, 2.0 * M_PI);
}

const char* safetyFailureName(nav_kernel::SafetyFailure failure) {
  switch (failure) {
    case nav_kernel::SafetyFailure::None:
      return "none";
    case nav_kernel::SafetyFailure::InvalidInput:
      return "invalid_input";
    case nav_kernel::SafetyFailure::ObstacleCollision:
      return "obstacle_collision";
    case nav_kernel::SafetyFailure::TraversabilityUnavailable:
      return "traversability_unavailable";
    case nav_kernel::SafetyFailure::TraversabilityOutOfBounds:
      return "traversability_out_of_bounds";
    case nav_kernel::SafetyFailure::TraversabilityNonFinite:
      return "traversability_non_finite";
    case nav_kernel::SafetyFailure::TraversabilityInvalidValue:
      return "traversability_invalid_value";
    case nav_kernel::SafetyFailure::TraversabilityBlocked:
      return "traversability_blocked";
  }
  return "invalid_input";
}

nav_kernel::RecoveryPlannerParams recoveryPlannerParams(
    const nav_kernel::LocalPlannerParams& params,
    const RecoveryConfig& config) {
  nav_kernel::RecoveryPlannerParams recovery;
  recovery.vehicleLength = params.vehicleLength;
  recovery.vehicleWidth = params.vehicleWidth;
  recovery.footprintPadding = params.footprintPadding;
  recovery.obstacleHeightThreshold = params.obstacleHeightThre;
  recovery.useTerrainAnalysis = params.useTerrainAnalysis;
  recovery.checkObstacles = params.checkObstacle;
  recovery.requireTraversability = params.useTraversabilityCost;
  recovery.traversabilityHardCost = params.traversabilityHardCost;

  const double configured_range =
      std::isfinite(params.adjacentRange)
          ? std::max(0.0, params.adjacentRange)
          : recovery.searchRadius;
  recovery.searchRadius = std::max(
      recovery.minTranslationDistance + recovery.latticeResolution,
      std::min(recovery.searchRadius, configured_range));
  recovery.minRotationRad = config.min_rotation_rad;
  recovery.maxRotationRad = config.max_rotation_rad;
  recovery.rotationCandidateStepRad = config.rotation_candidate_step_rad;
  recovery.rotationSampleStepRad = config.rotation_sample_step_rad;
  recovery.rotationRate = config.rotation_rate_rad_s;
  return recovery;
}

RecoveryConfig normalizeRecoveryConfig(RecoveryConfig config) {
  std::vector<nav_kernel::RecoveryAction> order;
  order.reserve(config.behavior_order.size());
  for (const nav_kernel::RecoveryAction action : config.behavior_order) {
    if (action != nav_kernel::RecoveryAction::Translate &&
        action != nav_kernel::RecoveryAction::Rotate) {
      continue;
    }
    if (std::find(order.begin(), order.end(), action) == order.end()) {
      order.push_back(action);
    }
  }
  if (order.empty()) {
    order = {
        nav_kernel::RecoveryAction::Translate,
        nav_kernel::RecoveryAction::Rotate,
    };
  }
  config.behavior_order = std::move(order);
  return config;
}

}  // namespace

class Recovery::Impl {
 public:
  Impl(nav_kernel::LocalPlannerParams params, RecoveryConfig config)
      : params_(std::move(params)),
        config_(normalizeRecoveryConfig(std::move(config))),
        planner_(recoveryPlannerParams(params_, config_)) {}

  RecoveryOutput step(const nav_kernel::LocalPlanRequest& request) {
    RecoveryOutput output;
    double goal_direction_body_rad = 0.0;
    std::string invalid_reason;
    if (!prepareInput(request, &goal_direction_body_rad, &invalid_reason)) {
      output.reason = invalid_reason.empty() ? "recovery_invalid_input"
                                             : std::move(invalid_reason);
      return output;
    }

    const int max_attempts = std::max(0, config_.max_attempts);
    if (max_attempts == 0 || cycle_count_ >= max_attempts) {
      markExhausted(output, "recovery_exhausted");
      populate(output);
      return output;
    }

    nav_kernel::RecoveryPlannerInput planner_input =
        makePlannerInput(request, goal_direction_body_rad);
    if (state_ != 0) {
      std::string failure_reason;
      const Update update =
          updateActive(request, planner_input, output, &failure_reason);
      if (update == Update::Active || update == Update::Completed) {
        populate(output);
        return output;
      }

      rejectActive(failure_reason);
      if (cycle_count_ >= max_attempts) {
        markExhausted(output, failure_reason + "_exhausted");
        populate(output);
        return output;
      }
      planner_input = makePlannerInput(request, goal_direction_body_rad);
    }

    session_active_ = true;
    const CandidateSelection selection = selectCandidate(planner_input);
    const nav_kernel::RecoveryPlanResult& candidate = selection.result;
    candidate_count_ = selection.candidate_count;
    if (!candidate.found() || !activate(candidate, request)) {
      advanceBehavior(selection.behavior_index);
      ++cycle_count_;
      reason_ = std::string("recovery_no_safe_candidate_") +
                safetyFailureName(candidate.safetyFailure);
      if (cycle_count_ >= max_attempts) {
        markExhausted(output, reason_ + "_exhausted");
      }
      populate(output);
      return output;
    }
    behavior_index_ = selection.behavior_index;

    planner_input = makePlannerInput(request, goal_direction_body_rad);
    std::string failure_reason;
    if (updateActive(request, planner_input, output, &failure_reason) ==
        Update::Failed) {
      rejectActive(failure_reason);
      if (cycle_count_ >= max_attempts) {
        markExhausted(output, failure_reason + "_exhausted");
      }
    }
    populate(output);
    return output;
  }

  void reset() {
    session_active_ = false;
    state_ = 0;
    cycle_count_ = 0;
    action_ = nav_kernel::RecoveryAction::None;
    world_path_.clear();
    path_cumulative_.clear();
    path_length_ = 0.0;
    target_yaw_delta_ = 0.0;
    start_pose_ = {};
    last_progress_ = 0.0;
    last_progress_time_ = -1.0;
    candidate_count_ = 0;
    rotation_direction_ = 0;
    direction_bin_ = -1;
    rejected_translation_mask_ = 0;
    rejected_rotation_mask_ = 0;
    behavior_cursor_ = 0;
    behavior_index_ = -1;
    reason_.clear();
  }

  [[nodiscard]] bool active() const { return session_active_; }

 private:
  enum class Update { Active, Completed, Failed };

  struct CandidateSelection {
    nav_kernel::RecoveryPlanResult result{};
    int candidate_count{0};
    int behavior_index{-1};
  };

  struct PathProjection {
    double progress{0.0};
    double along_distance{0.0};
    double cross_track_distance{std::numeric_limits<double>::infinity()};
    std::size_t segment_index{0};
  };

  bool prepareInput(const nav_kernel::LocalPlanRequest& request,
                    double* goal_direction_body_rad,
                    std::string* failure_reason) {
    obstacle_x_.clear();
    obstacle_y_.clear();
    obstacle_height_.clear();
    const nav_kernel::LocalRouteView* route = request.route();
    const bool collision_authoritative =
        params_.backend == nav_kernel::LocalPlannerBackend::Scan &&
        request.environment.collision.present();

    if (route == nullptr || !route->valid() ||
        !finitePoint(request.robot.pose.position) ||
        !std::isfinite(request.robot.pose.yaw) ||
        !std::isfinite(request.clock.timestampS) ||
        request.clock.timestampS < 0.0 ||
        (!collision_authoritative &&
         (request.environment.obstacles.count < 0 ||
          (request.environment.obstacles.count > 0 &&
           request.environment.obstacles.xyzh == nullptr)))) {
      *failure_reason = "recovery_invalid_input";
      return false;
    }

    const nav_kernel::Vec3 target = route->target();
    if (!finitePoint(target)) {
      *failure_reason = "recovery_invalid_route";
      return false;
    }
    const double c = std::cos(request.robot.pose.yaw);
    const double s = std::sin(request.robot.pose.yaw);
    const double target_dx = target.x - request.robot.pose.position.x;
    const double target_dy = target.y - request.robot.pose.position.y;
    *goal_direction_body_rad = std::atan2(
        -s * target_dx + c * target_dy,
        c * target_dx + s * target_dy);

    const nav_kernel::RecoveryPlannerParams& planner_params = planner_.params();
    const double half_length =
        0.5 * planner_params.vehicleLength + planner_params.footprintPadding;
    const double half_width =
        0.5 * planner_params.vehicleWidth + planner_params.footprintPadding;
    const double obstacle_range =
        planner_params.searchRadius + std::hypot(half_length, half_width);
    const double obstacle_range_sq = obstacle_range * obstacle_range;

    const auto append_planning_obstacle =
        [&](double x, double y, double height) -> bool {
      if (!std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(height)) {
        return false;
      }
      const double dx = x - request.robot.pose.position.x;
      const double dy = y - request.robot.pose.position.y;
      if (dx * dx + dy * dy > obstacle_range_sq) return true;
      obstacle_x_.push_back(static_cast<float>(c * dx + s * dy));
      obstacle_y_.push_back(static_cast<float>(-s * dx + c * dy));
      obstacle_height_.push_back(static_cast<float>(height));
      return true;
    };

    if (params_.checkObstacle) {
      if (!collision_authoritative) {
        for (int index = 0; index < request.environment.obstacles.count;
             ++index) {
          const float* point = request.environment.obstacles.xyzh + index * 4;
          if (!append_planning_obstacle(point[0], point[1], point[3])) {
            *failure_reason = "recovery_obstacle_nonfinite";
            return false;
          }
        }
      }

      if (collision_authoritative) {
        if (!validateCollisionMap(request, obstacle_range, failure_reason)) {
          return false;
        }
        const double min_relative_z =
            params_.backend == nav_kernel::LocalPlannerBackend::Scan
                ? -std::max(0.0, params_.scan.bodyClearanceBelow)
                : params_.minRelZ;
        const double max_relative_z =
            params_.backend == nav_kernel::LocalPlannerBackend::Scan
                ? std::max(0.0, params_.scan.bodyClearanceAbove)
                : params_.maxRelZ;
        const double occupied_height =
            std::max(0.0, params_.obstacleHeightThre);
        const auto& collision = request.environment.collision;
        for (std::size_t index = 0; index < collision.cellCount(); ++index) {
          if (!collision.occupiedLinear(index)) {
            continue;
          }
          const nav_kernel::Vec3 point = collision.planningCellCenter(index);
          const double relative_z =
              point.z - request.robot.pose.position.z;
          if (relative_z < min_relative_z || relative_z > max_relative_z) {
            continue;
          }
          if (!append_planning_obstacle(point.x, point.y, occupied_height)) {
            *failure_reason = "recovery_collision_map_nonfinite";
            return false;
          }
        }
      }
    }

    if (obstacle_x_.size() >
        static_cast<std::size_t>(planner_params.maxObstaclePoints)) {
      *failure_reason = "recovery_obstacle_capacity_exceeded";
      return false;
    }
    return true;
  }

  bool validateCollisionMap(const nav_kernel::LocalPlanRequest& request,
                            double obstacle_range,
                            std::string* failure_reason) const {
    const nav_kernel::LocalCollisionMapView& collision =
        request.environment.collision;
    const bool structural_valid = collision.valid() &&
        collision.resetEpoch > 0 && collision.observationSequence > 0 &&
        collision.generation > 0 && std::isfinite(collision.stampS) &&
        collision.stampS > 0.0 &&
        std::isfinite(collision.receiveStampS) &&
        collision.receiveStampS >= 0.0;
    if (!structural_valid) {
      *failure_reason = "recovery_collision_map_invalid";
      return false;
    }
    if (!collision.complete) {
      *failure_reason = "recovery_collision_map_incomplete";
      return false;
    }
    if (!collision.live) {
      *failure_reason = "recovery_collision_map_not_live";
      return false;
    }
    const double freshness_stamp =
        collision.receiveStampS > 0.0
            ? collision.receiveStampS
            : collision.stampS;
    const double age = request.clock.timestampS - freshness_stamp;
    if (!std::isfinite(age) || age < -0.10 ||
        age > std::max(0.10, params_.scan.collisionMaxAge)) {
      *failure_reason = "recovery_collision_map_stale";
      return false;
    }

    const double lower =
        params_.backend == nav_kernel::LocalPlannerBackend::Scan
            ? std::max(0.0, params_.scan.bodyClearanceBelow)
            : std::max(0.0, -params_.minRelZ);
    const double upper =
        params_.backend == nav_kernel::LocalPlannerBackend::Scan
            ? std::max(0.0, params_.scan.bodyClearanceAbove)
            : std::max(0.0, params_.maxRelZ);
    const double tolerance = collision.resolution;
    const nav_kernel::Vec3 required_min{
        request.robot.pose.position.x - obstacle_range,
        request.robot.pose.position.y - obstacle_range,
        request.robot.pose.position.z - lower,
    };
    const nav_kernel::Vec3 required_max{
        request.robot.pose.position.x + obstacle_range,
        request.robot.pose.position.y + obstacle_range,
        request.robot.pose.position.z + upper,
    };
    for (const double x : {required_min.x, required_max.x}) {
      for (const double y : {required_min.y, required_max.y}) {
        for (const double z : {required_min.z, required_max.z}) {
          if (!collision.covers({x, y, z}, tolerance)) {
            *failure_reason = "recovery_collision_map_roi_uncovered";
            return false;
          }
        }
      }
    }
    return true;
  }

  nav_kernel::RecoveryPlannerInput makePlannerInput(
      const nav_kernel::LocalPlanRequest& request,
      double goal_direction_body_rad) const {
    nav_kernel::RecoveryPlannerInput recovery;
    recovery.vehiclePose = request.robot.pose;
    if (!obstacle_x_.empty()) {
      recovery.obstacleX = obstacle_x_.data();
      recovery.obstacleY = obstacle_y_.data();
      recovery.obstacleHeight = obstacle_height_.data();
      recovery.obstacleCount = static_cast<int>(obstacle_x_.size());
    }
    if (request.environment.traversability.valid()) {
      recovery.traversabilityGrid = request.environment.traversability.values;
      recovery.traversabilityRows = request.environment.traversability.rows;
      recovery.traversabilityCols = request.environment.traversability.cols;
      recovery.traversabilityResolution =
          request.environment.traversability.resolution;
      recovery.traversabilityOriginX =
          request.environment.traversability.originX;
      recovery.traversabilityOriginY =
          request.environment.traversability.originY;
    }
    recovery.goalDirectionBodyRad = goal_direction_body_rad;
    recovery.rejectedTranslationDirectionMask = rejected_translation_mask_;
    recovery.rejectedRotationDirectionMask = rejected_rotation_mask_;
    return recovery;
  }

  CandidateSelection selectCandidate(
      const nav_kernel::RecoveryPlannerInput& base_input) const {
    CandidateSelection selection;
    const std::size_t behavior_count = config_.behavior_order.size();
    for (std::size_t offset = 0; offset < behavior_count; ++offset) {
      const std::size_t index =
          (behavior_cursor_ + offset) % behavior_count;
      const nav_kernel::RecoveryAction action = config_.behavior_order[index];
      nav_kernel::RecoveryPlannerInput input = base_input;
      if (action == nav_kernel::RecoveryAction::Translate) {
        input.rejectedRotationDirectionMask |= 0x3;
      } else {
        input.rejectedTranslationDirectionMask |= 0xFFFFU;
      }

      nav_kernel::RecoveryPlanResult candidate = planner_.plan(input);
      selection.candidate_count +=
          candidate.diagnostics.candidateCount +
          candidate.diagnostics.rotationCandidateCount;
      selection.result = std::move(candidate);
      selection.behavior_index = static_cast<int>(index);
      if (selection.result.found()) {
        return selection;
      }
      if (selection.result.status == nav_kernel::PlanStatus::InvalidInput) {
        return selection;
      }
    }
    return selection;
  }

  void advanceBehavior(int behavior_index) {
    if (behavior_index < 0 || config_.behavior_order.empty()) return;
    behavior_cursor_ =
        (static_cast<std::size_t>(behavior_index) + 1U) %
        config_.behavior_order.size();
  }

  void populate(RecoveryOutput& output) const {
    output.state = state_;
    output.active = session_active_;
    output.verified = state_ != 0;
    output.direct_command =
        output.verified && action_ == nav_kernel::RecoveryAction::Rotate;
    output.action = output.verified ? action_
                                    : nav_kernel::RecoveryAction::None;
    output.reason = reason_.empty() ? "inactive" : reason_;
    output.progress = std::clamp(last_progress_, 0.0, 1.0);
    output.attempt = state_ != 0 ? cycle_count_ + 1 : cycle_count_;
    output.candidate_count = candidate_count_;
    output.rotation_direction = state_ == 1 ? rotation_direction_ : 0;
    output.rotation_target_rad = state_ == 1 ? target_yaw_delta_ : 0.0;
  }

  void clearActive() {
    state_ = 0;
    action_ = nav_kernel::RecoveryAction::None;
    world_path_.clear();
    path_cumulative_.clear();
    path_length_ = 0.0;
    target_yaw_delta_ = 0.0;
    rotation_direction_ = 0;
    direction_bin_ = -1;
    behavior_index_ = -1;
  }

  void finish(double timestamp_s, const char* reason) {
    const int completed_behavior_index = behavior_index_;
    clearActive();
    session_active_ = false;
    ++cycle_count_;
    rejected_translation_mask_ = 0;
    rejected_rotation_mask_ = 0;
    advanceBehavior(completed_behavior_index);
    last_progress_ = 1.0;
    last_progress_time_ = timestamp_s;
    reason_ = reason;
  }

  void rejectActive(const std::string& reason) {
    if (action_ == nav_kernel::RecoveryAction::Translate &&
        direction_bin_ >= 0 && direction_bin_ < 16) {
      rejected_translation_mask_ |=
          std::uint32_t{1} << static_cast<std::uint32_t>(direction_bin_);
    } else if (action_ == nav_kernel::RecoveryAction::Rotate) {
      rejected_rotation_mask_ |= rotation_direction_ > 0 ? 0x1 : 0x2;
    }
    advanceBehavior(behavior_index_);
    ++cycle_count_;
    clearActive();
    reason_ = reason;
  }

  void markExhausted(RecoveryOutput& output, const std::string& reason) {
    clearActive();
    session_active_ = false;
    output.path_body.clear();
    output.exhausted = true;
    behavior_cursor_ = 0;
    reason_ = reason;
  }

  void buildPathCumulative() {
    path_cumulative_.assign(world_path_.size(), 0.0);
    for (std::size_t index = 1; index < world_path_.size(); ++index) {
      path_cumulative_[index] =
          path_cumulative_[index - 1] +
          nav_kernel::distance2D(world_path_[index - 1], world_path_[index]);
    }
    path_length_ = path_cumulative_.empty() ? 0.0 : path_cumulative_.back();
  }

  PathProjection projectPath(const nav_kernel::Pose& vehicle) const {
    PathProjection projection;
    if (world_path_.size() < 2 ||
        path_cumulative_.size() != world_path_.size() ||
        path_length_ <= 1e-9) {
      return projection;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index + 1 < world_path_.size(); ++index) {
      const nav_kernel::Vec3& a = world_path_[index];
      const nav_kernel::Vec3& b = world_path_[index + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double segment_length_sq = dx * dx + dy * dy;
      if (segment_length_sq <= 1e-12) continue;

      const double ratio = std::clamp(
          ((vehicle.position.x - a.x) * dx +
           (vehicle.position.y - a.y) * dy) /
              segment_length_sq,
          0.0,
          1.0);
      const double nearest_x = a.x + ratio * dx;
      const double nearest_y = a.y + ratio * dy;
      const double error_x = vehicle.position.x - nearest_x;
      const double error_y = vehicle.position.y - nearest_y;
      const double distance_sq = error_x * error_x + error_y * error_y;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        projection.segment_index = index;
        projection.along_distance =
            path_cumulative_[index] + ratio * std::sqrt(segment_length_sq);
      }
    }
    projection.cross_track_distance = std::sqrt(best_distance_sq);
    projection.progress = std::clamp(
        projection.along_distance / path_length_, 0.0, 1.0);
    return projection;
  }

  std::vector<nav_kernel::Vec3> remainingPathBody(
      const nav_kernel::Pose& vehicle,
      std::size_t segment_index) const {
    const std::vector<nav_kernel::Vec3> body =
        nav_kernel::RecoveryPlanner::worldPathToBody(world_path_, vehicle);
    std::vector<nav_kernel::Vec3> remaining;
    remaining.reserve(body.size() + 1);
    remaining.push_back({0.0, 0.0, 0.0});
    if (body.empty()) return remaining;

    const std::size_t first = std::min(segment_index + 1, body.size() - 1);
    for (std::size_t index = first; index < body.size(); ++index) {
      if (nav_kernel::distance2D(remaining.back(), body[index]) > 1e-4) {
        remaining.push_back(body[index]);
      }
    }
    return remaining;
  }

  bool activate(const nav_kernel::RecoveryPlanResult& candidate,
                const nav_kernel::LocalPlanRequest& request) {
    if (!candidate.found()) return false;

    action_ = candidate.action;
    start_pose_ = request.robot.pose;
    last_progress_ = 0.0;
    last_progress_time_ = request.clock.timestampS;
    direction_bin_ = candidate.diagnostics.selectedDirectionBin;

    if (candidate.action == nav_kernel::RecoveryAction::Translate) {
      world_path_ = candidate.pathWorld;
      if (world_path_.empty()) {
        world_path_ = nav_kernel::RecoveryPlanner::bodyPathToWorld(
            candidate.pathBody, request.robot.pose);
      }
      if (world_path_.empty() ||
          nav_kernel::distance2D(
              world_path_.front(), request.robot.pose.position) > 1e-4) {
        world_path_.insert(world_path_.begin(), request.robot.pose.position);
      }
      buildPathCumulative();
      if (world_path_.size() < 2 || path_length_ <= 1e-6 ||
          direction_bin_ < 0 || direction_bin_ >= 16) {
        clearActive();
        return false;
      }
      state_ = 2;
      rotation_direction_ = 0;
      reason_ = "recovery_translation_active";
      return true;
    }

    if (candidate.action == nav_kernel::RecoveryAction::Rotate &&
        std::isfinite(candidate.rotationDeltaRad) &&
        std::fabs(candidate.rotationDeltaRad) > 1e-6) {
      state_ = 1;
      target_yaw_delta_ = candidate.rotationDeltaRad;
      rotation_direction_ = candidate.rotationDeltaRad > 0.0 ? 1 : -1;
      reason_ = "recovery_rotation_active";
      return true;
    }

    clearActive();
    return false;
  }

  Update updateActive(const nav_kernel::LocalPlanRequest& request,
                      const nav_kernel::RecoveryPlannerInput& planner_input,
                      RecoveryOutput& output,
                      std::string* failure_reason) {
    if (last_progress_time_ > request.clock.timestampS) {
      last_progress_time_ = request.clock.timestampS;
    }

    if (action_ == nav_kernel::RecoveryAction::Translate) {
      const PathProjection projection = projectPath(request.robot.pose);
      const double cross_track_limit = std::max(
          0.20,
          0.5 * params_.vehicleWidth + params_.footprintPadding);
      const double observed_progress =
          projection.cross_track_distance <= cross_track_limit
              ? projection.progress
              : last_progress_;
      if (observed_progress > last_progress_ + 0.01) {
        last_progress_ = observed_progress;
        last_progress_time_ = request.clock.timestampS;
      }

      const double completion_distance =
          std::max(0.08, planner_.params().latticeResolution * 1.25);
      if (path_length_ - projection.along_distance <= completion_distance) {
        output.observation_refresh_required = true;
        finish(request.clock.timestampS, "recovery_translation_complete");
        return Update::Completed;
      }

      std::vector<nav_kernel::Vec3> remaining =
          remainingPathBody(request.robot.pose, projection.segment_index);
      nav_kernel::SafetyFailure failure = nav_kernel::SafetyFailure::None;
      if (remaining.size() < 2 ||
          !planner_.validateBodyPath(planner_input, remaining, &failure)) {
        *failure_reason = std::string("recovery_translation_unsafe_") +
                          safetyFailureName(failure);
        return Update::Failed;
      }

      const double timeout =
          std::isfinite(config_.translation_timeout_s)
              ? std::max(0.05, config_.translation_timeout_s)
              : 0.05;
      if (request.clock.timestampS - last_progress_time_ >= timeout) {
        *failure_reason = "recovery_translation_no_progress";
        return Update::Failed;
      }

      output.path_body = std::move(remaining);
      reason_ = "recovery_translation_active";
      return Update::Active;
    }

    if (action_ == nav_kernel::RecoveryAction::Rotate) {
      const double target = std::fabs(target_yaw_delta_);
      const double signed_travel =
          normalizeAngle(request.robot.pose.yaw - start_pose_.yaw) *
          static_cast<double>(rotation_direction_);
      const double observed_progress =
          target > 1e-9
              ? std::clamp(std::max(0.0, signed_travel) / target, 0.0, 1.0)
              : 0.0;
      if (observed_progress > last_progress_ + 0.01) {
        last_progress_ = observed_progress;
        last_progress_time_ = request.clock.timestampS;
      }

      const double remaining_magnitude =
          std::max(0.0, target - std::max(0.0, signed_travel));
      if (remaining_magnitude <= 0.05) {
        output.observation_refresh_required = true;
        finish(request.clock.timestampS, "recovery_rotation_complete");
        return Update::Completed;
      }

      const double remaining_delta =
          static_cast<double>(rotation_direction_) * remaining_magnitude;
      nav_kernel::SafetyFailure failure = nav_kernel::SafetyFailure::None;
      if (!planner_.validateRotation(
              planner_input, remaining_delta, &failure)) {
        *failure_reason = std::string("recovery_rotation_unsafe_") +
                          safetyFailureName(failure);
        return Update::Failed;
      }

      const double timeout =
          std::isfinite(config_.rotation_timeout_s)
              ? std::max(0.05, config_.rotation_timeout_s)
              : 0.05;
      if (request.clock.timestampS - last_progress_time_ >= timeout) {
        *failure_reason = "recovery_rotation_no_progress";
        return Update::Failed;
      }

      output.path_body.clear();
      reason_ = "recovery_rotation_active";
      return Update::Active;
    }

    *failure_reason = "recovery_invalid_active_action";
    return Update::Failed;
  }

  nav_kernel::LocalPlannerParams params_;
  RecoveryConfig config_;
  nav_kernel::RecoveryPlanner planner_;
  std::vector<float> obstacle_x_;
  std::vector<float> obstacle_y_;
  std::vector<float> obstacle_height_;

  bool session_active_{false};
  int state_{0};
  int cycle_count_{0};
  nav_kernel::RecoveryAction action_{nav_kernel::RecoveryAction::None};
  std::vector<nav_kernel::Vec3> world_path_;
  std::vector<double> path_cumulative_;
  double path_length_{0.0};
  double target_yaw_delta_{0.0};
  nav_kernel::Pose start_pose_{};
  double last_progress_{0.0};
  double last_progress_time_{-1.0};
  int candidate_count_{0};
  int rotation_direction_{0};
  int direction_bin_{-1};
  std::uint32_t rejected_translation_mask_{0};
  int rejected_rotation_mask_{0};
  std::size_t behavior_cursor_{0};
  int behavior_index_{-1};
  std::string reason_;
};

Recovery::Recovery(const nav_kernel::LocalPlannerParams& planner_params,
                   const RecoveryConfig& config)
    : impl_(std::make_unique<Impl>(planner_params, config)) {}

Recovery::~Recovery() = default;
Recovery::Recovery(Recovery&&) noexcept = default;
Recovery& Recovery::operator=(Recovery&&) noexcept = default;

RecoveryOutput Recovery::step(const nav_kernel::LocalPlanRequest& request) {
  return impl_->step(request);
}

void Recovery::reset() {
  impl_->reset();
}

bool Recovery::active() const {
  return impl_->active();
}

}  // namespace lingtu::nav::navigation
