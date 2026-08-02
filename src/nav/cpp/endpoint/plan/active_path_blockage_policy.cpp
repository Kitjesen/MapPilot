#include "plan/active_path_blockage_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

constexpr double kMaximumPersistenceS = 300.0;
constexpr std::size_t kMaximumFreshObservations = 10000U;
constexpr double kMaximumLookaheadM = 1000.0;
constexpr double kMaximumCorridorRadiusM = 10.0;
constexpr double kMaximumVerticalToleranceM = 20.0;
constexpr double kMaximumOverlayRadiusM = 10.0;
constexpr double kMaximumOverlayHalfHeightM = 10.0;
constexpr std::size_t kMaximumMinimumObstaclePoints = 1000000U;

bool finiteInRange(double value, double minimum, double maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

double safeDistance3D(const nav_kernel::Vec3 &left, const nav_kernel::Vec3 &right) {
  return std::hypot(std::hypot(left.x - right.x, left.y - right.y), left.z - right.z);
}

}  // namespace

ActivePathBlockagePolicy::ActivePathBlockagePolicy(ActivePathBlockagePolicyConfig config)
    : config_(config) {
  if (!finiteInRange(config_.persistence_s, std::numeric_limits<double>::min(),
                     kMaximumPersistenceS)) {
    throw std::invalid_argument("active_path_blockage_persistence_invalid");
  }
  if (config_.minimum_fresh_observations < 2U ||
      config_.minimum_fresh_observations > kMaximumFreshObservations) {
    throw std::invalid_argument("active_path_blockage_observation_count_invalid");
  }
  if (!finiteInRange(config_.lookahead_m, std::numeric_limits<double>::min(), kMaximumLookaheadM)) {
    throw std::invalid_argument("active_path_blockage_lookahead_invalid");
  }
  if (!finiteInRange(config_.corridor_radius_m, std::numeric_limits<double>::min(),
                     kMaximumCorridorRadiusM)) {
    throw std::invalid_argument("active_path_blockage_corridor_radius_invalid");
  }
  if (!finiteInRange(config_.corridor_vertical_tolerance_m, std::numeric_limits<double>::min(),
                     kMaximumVerticalToleranceM)) {
    throw std::invalid_argument("active_path_blockage_vertical_tolerance_invalid");
  }
  if (!finiteInRange(config_.overlay_radius_m, std::numeric_limits<double>::min(),
                     kMaximumOverlayRadiusM)) {
    throw std::invalid_argument("active_path_blockage_overlay_radius_invalid");
  }
  if (!finiteInRange(config_.overlay_half_height_m, std::numeric_limits<double>::min(),
                     kMaximumOverlayHalfHeightM)) {
    throw std::invalid_argument("active_path_blockage_overlay_height_invalid");
  }
  if (config_.max_regions == 0U || config_.max_regions > kMaximumOverlayRegions) {
    throw std::invalid_argument("active_path_blockage_region_count_invalid");
  }
  if (config_.minimum_obstacle_points == 0U ||
      config_.minimum_obstacle_points > kMaximumMinimumObstaclePoints) {
    throw std::invalid_argument("active_path_blockage_obstacle_points_invalid");
  }
}

bool ActivePathBlockagePolicy::validPoint(const nav_kernel::Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool ActivePathBlockagePolicy::sameBinding(const GoalReplanIdentity &goal,
                                           std::uint64_t frame_epoch) const {
  return goal_.has_value() && frame_epoch_ == frame_epoch && sameGoalReplanIdentity(*goal_, goal);
}

void ActivePathBlockagePolicy::bind(const GoalReplanIdentity &goal, std::uint64_t frame_epoch) {
  goal_ = goal;
  frame_epoch_ = frame_epoch;
  last_cloud_generation_ = 0U;
  last_traversability_generation_ = 0U;
  fresh_blocked_observations_ = 0U;
  current_blocker_count_ = 0U;
  first_blocked_s_ = -1.0;
  last_now_s_ = -1.0;
  trigger_emitted_ = false;
  reason_ = "identity_bound";
}

void ActivePathBlockagePolicy::clearAccumulation(const char *reason) {
  fresh_blocked_observations_ = 0U;
  current_blocker_count_ = 0U;
  first_blocked_s_ = -1.0;
  reason_ = reason;
}

void ActivePathBlockagePolicy::setGenerationBaseline(std::uint64_t cloud_generation,
                                                     std::uint64_t traversability_generation) {
  last_cloud_generation_ = cloud_generation;
  last_traversability_generation_ = traversability_generation;
}

std::vector<ActivePathBlockagePolicy::CorridorBlocker>
ActivePathBlockagePolicy::corridorBlockers(const ActivePathBlockageObservation &observation) const {
  std::vector<CorridorBlocker> result;
  const auto &path = *observation.active_global_path;
  const auto &obstacles = *observation.live_obstacles_xyzh;
  if (path.empty() || obstacles.empty()) {
    return result;
  }

  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.size(); ++index) {
    const double distance = safeDistance3D(observation.robot_position, path[index]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  if (!std::isfinite(nearest_distance)) {
    return result;
  }

  result.reserve(obstacles.size() / 4U);
  for (std::size_t offset = 0U; offset + 3U < obstacles.size(); offset += 4U) {
    const double x = static_cast<double>(obstacles[offset]);
    const double y = static_cast<double>(obstacles[offset + 1U]);
    const double z = static_cast<double>(obstacles[offset + 2U]);
    const double height = static_cast<double>(obstacles[offset + 3U]);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(height)) {
      continue;
    }

    double best_along = std::numeric_limits<double>::infinity();
    double cumulative = 0.0;
    for (std::size_t index = nearest_index; index < path.size(); ++index) {
      const nav_kernel::Vec3 &start = path[index];
      if (index + 1U >= path.size()) {
        if (cumulative <= config_.lookahead_m) {
          const double xy_distance = std::hypot(x - start.x, y - start.y);
          if (xy_distance <= config_.corridor_radius_m &&
              std::abs(z - start.z) <= config_.corridor_vertical_tolerance_m) {
            best_along = std::min(best_along, cumulative);
          }
        }
        break;
      }

      const nav_kernel::Vec3 &end = path[index + 1U];
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double dz = end.z - start.z;
      const double segment_length = std::hypot(std::hypot(dx, dy), dz);
      if (!std::isfinite(segment_length)) {
        return {};
      }

      if (segment_length <= std::numeric_limits<double>::epsilon()) {
        if (cumulative <= config_.lookahead_m) {
          const double xy_distance = std::hypot(x - start.x, y - start.y);
          if (xy_distance <= config_.corridor_radius_m &&
              std::abs(z - start.z) <= config_.corridor_vertical_tolerance_m) {
            best_along = std::min(best_along, cumulative);
          }
        }
        continue;
      }

      const double remaining = config_.lookahead_m - cumulative;
      if (remaining < 0.0) {
        break;
      }
      const double usable_length = std::min(segment_length, remaining);
      const double ux = dx / segment_length;
      const double uy = dy / segment_length;
      const double uz = dz / segment_length;
      double projected = (x - start.x) * ux + (y - start.y) * uy + (z - start.z) * uz;
      projected = std::clamp(projected, 0.0, usable_length);
      const double closest_x = start.x + ux * projected;
      const double closest_y = start.y + uy * projected;
      const double closest_z = start.z + uz * projected;
      const double xy_distance = std::hypot(x - closest_x, y - closest_y);
      if (xy_distance <= config_.corridor_radius_m &&
          std::abs(z - closest_z) <= config_.corridor_vertical_tolerance_m) {
        best_along = std::min(best_along, cumulative + projected);
      }

      cumulative += usable_length;
      if (usable_length < segment_length || cumulative >= config_.lookahead_m) {
        break;
      }
    }

    if (!std::isfinite(best_along)) {
      continue;
    }
    const double min_z = z - config_.overlay_half_height_m;
    const double max_z = z + config_.overlay_half_height_m;
    const double min_x = x - config_.overlay_radius_m;
    const double max_x = x + config_.overlay_radius_m;
    const double min_y = y - config_.overlay_radius_m;
    const double max_y = y + config_.overlay_radius_m;
    if (!std::isfinite(min_z) || !std::isfinite(max_z) || !std::isfinite(min_x) ||
        !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y)) {
      continue;
    }

    CorridorBlocker blocker;
    blocker.along_path_m = best_along;
    blocker.height = height;
    blocker.region.center = {x, y, z};
    blocker.region.radius_xy_m = config_.overlay_radius_m;
    blocker.region.min_z = min_z;
    blocker.region.max_z = max_z;
    result.push_back(std::move(blocker));
  }

  std::sort(result.begin(), result.end(),
            [](const CorridorBlocker &left, const CorridorBlocker &right) {
              if (left.along_path_m != right.along_path_m) {
                return left.along_path_m < right.along_path_m;
              }
              if (left.region.center.x != right.region.center.x) {
                return left.region.center.x < right.region.center.x;
              }
              if (left.region.center.y != right.region.center.y) {
                return left.region.center.y < right.region.center.y;
              }
              if (left.region.center.z != right.region.center.z) {
                return left.region.center.z < right.region.center.z;
              }
              return left.height < right.height;
            });
  return result;
}

std::optional<GoalReplanTrigger>
ActivePathBlockagePolicy::observe(const ActivePathBlockageObservation &observation) {
  if (!observation.external_active_goal || !observation.goal.valid() ||
      observation.frame_epoch == 0U) {
    reset();
    reason_ = "external_goal_inactive";
    return std::nullopt;
  }
  if (!sameBinding(observation.goal, observation.frame_epoch)) {
    bind(observation.goal, observation.frame_epoch);
  }

  if (!std::isfinite(observation.now_s) || observation.now_s < 0.0 ||
      !validPoint(observation.robot_position) || observation.active_global_path == nullptr ||
      observation.live_obstacles_xyzh == nullptr ||
      observation.live_obstacles_xyzh->size() % 4U != 0U) {
    clearAccumulation("invalid_observation");
    return std::nullopt;
  }

  if (last_now_s_ >= 0.0 && observation.now_s < last_now_s_) {
    clearAccumulation("clock_rollback");
    last_now_s_ = observation.now_s;
    setGenerationBaseline(observation.cloud_generation, observation.traversability_generation);
    return std::nullopt;
  }
  last_now_s_ = observation.now_s;

  if (observation.cloud_generation == 0U || observation.traversability_generation == 0U) {
    reason_ = "generation_missing";
    return std::nullopt;
  }
  if ((last_cloud_generation_ != 0U && observation.cloud_generation < last_cloud_generation_) ||
      (last_traversability_generation_ != 0U &&
       observation.traversability_generation < last_traversability_generation_)) {
    clearAccumulation("generation_rollback");
    setGenerationBaseline(observation.cloud_generation, observation.traversability_generation);
    return std::nullopt;
  }

  const bool fresh_pair = (last_cloud_generation_ == 0U && last_traversability_generation_ == 0U) ||
                          (observation.cloud_generation > last_cloud_generation_ &&
                           observation.traversability_generation > last_traversability_generation_);

  const auto &path = *observation.active_global_path;
  if (path.empty() || std::any_of(path.begin(), path.end(), [](const nav_kernel::Vec3 &point) {
        return !validPoint(point);
      })) {
    clearAccumulation(path.empty() ? "active_path_empty" : "active_path_invalid");
    if (fresh_pair) {
      setGenerationBaseline(observation.cloud_generation, observation.traversability_generation);
    }
    return std::nullopt;
  }

  std::vector<CorridorBlocker> blockers = corridorBlockers(observation);
  current_blocker_count_ = blockers.size();
  if (!fresh_pair) {
    reason_ = "generation_pair_stale";
    return std::nullopt;
  }
  setGenerationBaseline(observation.cloud_generation, observation.traversability_generation);
  if (blockers.size() < config_.minimum_obstacle_points) {
    const std::size_t blocker_count = blockers.size();
    clearAccumulation(blockers.empty() ? "corridor_clear" : "corridor_sparse");
    current_blocker_count_ = blocker_count;
    return std::nullopt;
  }

  if (trigger_emitted_) {
    reason_ = "trigger_already_emitted";
    return std::nullopt;
  }
  if (fresh_blocked_observations_ == 0U) {
    first_blocked_s_ = observation.now_s;
  }
  if (fresh_blocked_observations_ < std::numeric_limits<std::size_t>::max()) {
    ++fresh_blocked_observations_;
  }
  reason_ = "persistent_blockage_watching";

  const double blocked_for_s = observation.now_s - first_blocked_s_;
  if (fresh_blocked_observations_ < config_.minimum_fresh_observations ||
      blocked_for_s < config_.persistence_s) {
    return std::nullopt;
  }

  GoalReplanTrigger trigger;
  trigger.kind = GoalReplanTriggerKind::kPersistentPathObstruction;
  trigger.reason = "persistent_path_obstruction";
  trigger.goal = *goal_;
  trigger.temporary_overlay.revision = next_overlay_revision_;
  trigger.temporary_overlay.frame_epoch = frame_epoch_;
  trigger.temporary_overlay.obstacle_generation = observation.cloud_generation;
  trigger.temporary_overlay.traversability_generation = observation.traversability_generation;
  trigger.temporary_overlay.blocked_regions.reserve(std::min(config_.max_regions, blockers.size()));
  const double dedupe_distance_squared = config_.overlay_radius_m * config_.overlay_radius_m;
  for (const CorridorBlocker &blocker : blockers) {
    const bool duplicate =
        std::any_of(trigger.temporary_overlay.blocked_regions.begin(),
                    trigger.temporary_overlay.blocked_regions.end(), [&](const auto &region) {
                      const double dx = blocker.region.center.x - region.center.x;
                      const double dy = blocker.region.center.y - region.center.y;
                      return dx * dx + dy * dy <= dedupe_distance_squared;
                    });
    if (duplicate) {
      continue;
    }
    trigger.temporary_overlay.blocked_regions.push_back(blocker.region);
    if (trigger.temporary_overlay.blocked_regions.size() >= config_.max_regions) {
      break;
    }
  }

  if (next_overlay_revision_ != std::numeric_limits<std::uint64_t>::max()) {
    ++next_overlay_revision_;
  }
  trigger_emitted_ = true;
  reason_ = trigger.reason;
  return trigger;
}

ActivePathBlockagePolicySnapshot ActivePathBlockagePolicy::snapshot() const {
  ActivePathBlockagePolicySnapshot result;
  result.goal = goal_;
  result.frame_epoch = frame_epoch_;
  result.last_cloud_generation = last_cloud_generation_;
  result.last_traversability_generation = last_traversability_generation_;
  result.fresh_blocked_observations = fresh_blocked_observations_;
  result.current_blocker_count = current_blocker_count_;
  result.first_blocked_s = first_blocked_s_;
  result.trigger_emitted = trigger_emitted_;
  result.reason = reason_;
  return result;
}

void ActivePathBlockagePolicy::reset() {
  goal_.reset();
  frame_epoch_ = 0U;
  last_cloud_generation_ = 0U;
  last_traversability_generation_ = 0U;
  fresh_blocked_observations_ = 0U;
  current_blocker_count_ = 0U;
  first_blocked_s_ = -1.0;
  last_now_s_ = -1.0;
  trigger_emitted_ = false;
  reason_ = "reset";
}

}  // namespace lingtu::nav::endpoint
