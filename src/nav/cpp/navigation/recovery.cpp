#include "navigation/recovery.hpp"
#include "navigation/recovery_search.hpp"
#include "planning/local/scan/grid.hpp"

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

const char* safetyFailureName(detail::RecoverySafetyFailure failure) {
  switch (failure) {
    case detail::RecoverySafetyFailure::None:
      return "none";
    case detail::RecoverySafetyFailure::InvalidInput:
      return "invalid_input";
    case detail::RecoverySafetyFailure::ObstacleCollision:
      return "obstacle_collision";
    case detail::RecoverySafetyFailure::TraversabilityUnavailable:
      return "traversability_unavailable";
    case detail::RecoverySafetyFailure::TraversabilityOutOfBounds:
      return "traversability_out_of_bounds";
    case detail::RecoverySafetyFailure::TraversabilityNonFinite:
      return "traversability_non_finite";
    case detail::RecoverySafetyFailure::TraversabilityInvalidValue:
      return "traversability_invalid_value";
    case detail::RecoverySafetyFailure::TraversabilityBlocked:
      return "traversability_blocked";
  }
  return "invalid_input";
}

detail::RecoverySearchParams recoverySearchParams(
    const nav_kernel::LocalPlannerParams& params,
    const nav_kernel::LocalPlannerCapabilities& capabilities,
    const RecoveryConfig& config) {
  detail::RecoverySearchParams recovery;
  recovery.vehicleLength = params.vehicleLength;
  recovery.vehicleWidth = params.vehicleWidth;
  recovery.footprintPadding = params.footprintPadding;
  recovery.obstacleHeightThreshold = params.obstacleHeightThre;
  recovery.useTerrainAnalysis = params.useTerrainAnalysis;
  recovery.checkObstacles = params.checkObstacle;
  // SCAN consumes the 3D collision map. Its map-frame requests deliberately
  // omit the odom-only 2D traversability view, just like normal SCAN planning.
  recovery.requireTraversability = capabilities.consumesTraversability;
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
        capabilities_(nav_kernel::plannerCapabilities(params_)),
        search_(recoverySearchParams(params_, capabilities_, config_)) {}

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

    detail::RecoverySearchInput search_input =
        makeSearchInput(request, goal_direction_body_rad);
    if (state_ != 0) {
      std::string failure_reason;
      const Update update =
          updateActive(request, search_input, output, &failure_reason);
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
      search_input = makeSearchInput(request, goal_direction_body_rad);
    }

    session_active_ = true;
    const CandidateSelection selection = selectCandidate(search_input);
    const detail::RecoverySearchResult& candidate = selection.result;
    candidate_count_ = selection.candidate_count;
    if (!activate(candidate, request)) {
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

    std::string failure_reason;
    if (updateActive(request, search_input, output, &failure_reason) ==
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
    clearActive();
    cycle_count_ = 0;
    start_pose_ = {};
    last_progress_ = 0.0;
    last_progress_time_ = -1.0;
    candidate_count_ = 0;
    rejected_translation_mask_ = 0;
    rejected_rotation_mask_ = 0;
    behavior_cursor_ = 0;
    reason_.clear();
  }

  [[nodiscard]] bool active() const { return session_active_; }

 private:
  enum class Update { Active, Completed, Failed };

  struct CandidateSelection {
    detail::RecoverySearchResult result{};
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
    collision_grid_.reset();
    obstacle_x_.clear();
    obstacle_y_.clear();
    obstacle_height_.clear();
    const nav_kernel::LocalRouteView* route = request.route();
    const bool collision_authoritative = capabilities_.consumesCollisionMap;

    if (route == nullptr || !route->valid() ||
        !finitePoint(request.robot.pose.position) ||
        !std::isfinite(request.robot.pose.yaw) ||
        !std::isfinite(request.clock.timestampS) ||
        request.clock.timestampS < 0.0 ||
        (request.environment.obstacles.count < 0 ||
         (request.environment.obstacles.count > 0 &&
          request.environment.obstacles.xyzh == nullptr))) {
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

    const detail::RecoverySearchParams& search_params = search_.params();
    const double half_length =
        0.5 * search_params.vehicleLength + search_params.footprintPadding;
    const double half_width =
        0.5 * search_params.vehicleWidth + search_params.footprintPadding;
    const double obstacle_range =
        search_params.searchRadius + std::hypot(half_length, half_width);
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

    const bool needs_geometry = !capabilities_.consumesCollisionMap || state_ == 1 ||
        config_.behavior_order[behavior_cursor_] == nav_kernel::RecoveryAction::Rotate ||
        rejected_translation_mask_ == 0xFFFFU;
    if (needs_geometry && (params_.checkObstacle || request.body || request.observedBody)) {
      // The point cloud is the direct DDS-derived live observation and is a
      // useful recovery fallback when the optional packed collision snapshot
      // is between publications. Keep it for SCAN too; a valid 3D Grid still
      // takes precedence and preserves SCAN's exact collision semantics.
      for (int index = 0; index < request.environment.obstacles.count;
           ++index) {
        const float* point = request.environment.obstacles.xyzh + index * 4;
        if (!append_planning_obstacle(point[0], point[1], point[3])) {
          *failure_reason = "recovery_obstacle_nonfinite";
          return false;
        }
      }

      if (collision_authoritative && (request.body || request.observedBody ||
                                     request.environment.collision.valid())) {
        // Recovery checks its candidate's actual heading with the same body.
        // A missing body bitmap cannot fall back to a different 2D footprint.
        auto collision_request = request;
        collision_request.objective = nav_kernel::RouteTarget{*route};
        collision_request.bodyHeading.reset();
        collision_grid_.emplace(params_, collision_request);
        if (!collision_grid_->valid()) {
          if (request.body || request.observedBody) {
            *failure_reason = "recovery_" + collision_grid_->reason();
            return false;
          }
          collision_grid_.reset();
        }
      }
    }

    if (obstacle_x_.size() >
        static_cast<std::size_t>(search_params.maxObstaclePoints)) {
      *failure_reason = "recovery_obstacle_capacity_exceeded";
      return false;
    }
    return true;
  }

  detail::RecoverySearchInput makeSearchInput(
      const nav_kernel::LocalPlanRequest& request,
      double goal_direction_body_rad) const {
    detail::RecoverySearchInput recovery;
    recovery.vehiclePose = request.robot.pose;
    recovery.collisionGrid = collision_grid_ ? &*collision_grid_ : nullptr;
    if (!obstacle_x_.empty()) {
      recovery.obstacleX = obstacle_x_.data();
      recovery.obstacleY = obstacle_y_.data();
      recovery.obstacleHeight = obstacle_height_.data();
      recovery.obstacleCount = static_cast<int>(obstacle_x_.size());
    }
    if (capabilities_.consumesTraversability &&
        request.environment.traversability.valid()) {
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
      const detail::RecoverySearchInput& base_input) const {
    CandidateSelection selection;
    const std::size_t behavior_count = config_.behavior_order.size();
    for (std::size_t offset = 0; offset < behavior_count; ++offset) {
      const std::size_t index =
          (behavior_cursor_ + offset) % behavior_count;
      const nav_kernel::RecoveryAction action = config_.behavior_order[index];
      detail::RecoverySearchInput input = base_input;
      if (action == nav_kernel::RecoveryAction::Translate) {
        input.rejectedRotationDirectionMask |= 0x3;
      } else {
        input.rejectedTranslationDirectionMask |= 0xFFFFU;
      }

      detail::RecoverySearchResult candidate;
      if (capabilities_.consumesCollisionMap &&
          action == nav_kernel::RecoveryAction::Translate) {
        const int goal_bin = static_cast<int>(std::lround(
            input.goalDirectionBodyRad * 8.0 / M_PI));
        // These are temporary goals. SCAN decides whether a trajectory exists.
        for (const int offset : {4, -4, 8, 2, -2, 6, -6, 0,
                                 3, -3, 5, -5, 7, -7, 1, -1}) {
          const int bin = (goal_bin + offset + 32) % 16;
          if (input.rejectedTranslationDirectionMask & (std::uint32_t{1} << bin)) continue;
          const double angle = bin * M_PI / 8.0;
          const double distance = search_.params().searchRadius;
          candidate.status = detail::RecoverySearchStatus::TranslationReady;
          candidate.action = action;
          candidate.pathBody = {{}, {distance * std::cos(angle), distance * std::sin(angle), 0.0}};
          candidate.pathWorld = detail::RecoverySearch::bodyPathToWorld(
              candidate.pathBody, input.vehiclePose);
          candidate.diagnostics.selectedDirectionBin = bin;
          candidate.diagnostics.candidateCount = 1;
          break;
        }
      } else {
        candidate = search_.plan(input);
      }
      selection.candidate_count +=
          candidate.diagnostics.candidateCount +
          candidate.diagnostics.rotationCandidateCount;
      selection.result = std::move(candidate);
      selection.behavior_index = static_cast<int>(index);
      if (selection.result.found() ||
          selection.result.action == nav_kernel::RecoveryAction::Translate) {
        return selection;
      }
      if (selection.result.status == detail::RecoverySearchStatus::InvalidInput) {
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
    output.verified = state_ != 0 &&
        !(capabilities_.consumesCollisionMap && action_ == nav_kernel::RecoveryAction::Translate);
    output.direct_command =
        output.verified && action_ == nav_kernel::RecoveryAction::Rotate;
    output.action = state_ != 0 ? action_ : nav_kernel::RecoveryAction::None;
    if (state_ == 2) output.path_world = world_path_;
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
        detail::RecoverySearch::worldPathToBody(world_path_, vehicle);
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

  bool activate(const detail::RecoverySearchResult& candidate,
                const nav_kernel::LocalPlanRequest& request) {
    if (!candidate.found() &&
        !(capabilities_.consumesCollisionMap &&
          candidate.action == nav_kernel::RecoveryAction::Translate)) return false;

    action_ = candidate.action;
    start_pose_ = request.robot.pose;
    last_progress_ = 0.0;
    last_progress_time_ = request.clock.timestampS;
    direction_bin_ = candidate.diagnostics.selectedDirectionBin;

    if (candidate.action == nav_kernel::RecoveryAction::Translate) {
      world_path_ = candidate.pathWorld;
      if (world_path_.empty()) {
        world_path_ = detail::RecoverySearch::bodyPathToWorld(
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
                      const detail::RecoverySearchInput& search_input,
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
      if (!capabilities_.consumesCollisionMap &&
          projection.cross_track_distance > cross_track_limit) {
        *failure_reason = "recovery_translation_off_path";
        return Update::Failed;
      }
      const double observed_progress = projection.progress;
      if (observed_progress > last_progress_ + 0.01) {
        last_progress_ = observed_progress;
        last_progress_time_ = request.clock.timestampS;
      }

      const double completion_distance =
          std::max(0.08, search_.params().latticeResolution * 1.25);
      if (path_length_ - projection.along_distance <= completion_distance &&
          nav_kernel::distance2D(request.robot.pose.position, world_path_.back()) <=
              completion_distance) {
        finish(request.clock.timestampS, "recovery_translation_complete");
        return Update::Completed;
      }

      std::vector<nav_kernel::Vec3> remaining =
          remainingPathBody(request.robot.pose, projection.segment_index);
      detail::RecoverySafetyFailure failure = detail::RecoverySafetyFailure::None;
      if (remaining.size() < 2 ||
          (!capabilities_.consumesCollisionMap &&
           !search_.validateBodyPath(search_input, remaining, &failure))) {
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
        finish(request.clock.timestampS, "recovery_rotation_complete");
        return Update::Completed;
      }

      const double remaining_delta =
          static_cast<double>(rotation_direction_) * remaining_magnitude;
      detail::RecoverySafetyFailure failure = detail::RecoverySafetyFailure::None;
      if (!search_.validateRotation(
              search_input, remaining_delta, &failure)) {
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
  nav_kernel::LocalPlannerCapabilities capabilities_;
  detail::RecoverySearch search_;
  std::optional<nav_kernel::local::scan::Grid> collision_grid_;
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

Recovery::Recovery(const nav_kernel::LocalPlannerParams& search_params,
                   const RecoveryConfig& config)
    : impl_(std::make_unique<Impl>(search_params, config)) {}

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
