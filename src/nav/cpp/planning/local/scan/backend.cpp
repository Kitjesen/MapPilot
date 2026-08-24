#include "planning/local/scan/backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/search.hpp"
#include "planning/local/scan/spline.hpp"

namespace nav_kernel::local::scan {
namespace {

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Vec3 planningPointToBody(const Pose &vehicle, const Vec3 &point) {
  const double dx = point.x - vehicle.position.x;
  const double dy = point.y - vehicle.position.y;
  const double c = std::cos(vehicle.yaw);
  const double s = std::sin(vehicle.yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      point.z - vehicle.position.z,
  };
}

Vec3 planningVectorToBody(const Pose &vehicle, const Vec3 &value) {
  const double c = std::cos(vehicle.yaw);
  const double s = std::sin(vehicle.yaw);
  return {
      c * value.x + s * value.y,
      -s * value.x + c * value.y,
      value.z,
  };
}

double remainingDistance(const std::vector<Vec3> &path, std::size_t index) {
  double distance = 0.0;
  for (std::size_t i = index; i + 1 < path.size(); ++i) {
    distance += distance3D(path[i], path[i + 1]);
  }
  return distance;
}

}  // namespace

class Backend::Impl {
 public:
  explicit Impl(LocalPlannerParams params) : params_(std::move(params)) {}

  LocalPlanResult plan(const LocalPlanInput &input) { return plan(input, params_, nullptr); }

  LocalPlanResult planIntent(const LocalPlanInput &input, const LocalMotionIntent &intent) {
    LocalPlannerParams params = params_;
    params.autonomySpeed = params_.autonomySpeed * std::clamp(intent.speedNormalized, 0.0, 1.0);
    return plan(input, params, &intent);
  }

  void reset() {
    previous_.clear();
    previousTimestampS_ = -1.0;
    previousFrameEpoch_ = 0;
    previousRouteGeneration_ = 0;
    previousObstacleGeneration_ = 0;
    previousTraversabilityGeneration_ = 0;
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
  }

  LocalPlannerDebugSnapshot debugSnapshot() const { return debug_; }

 private:
  LocalPlanResult plan(const LocalPlanInput &input, const LocalPlannerParams &params,
                       const LocalMotionIntent *intent) {
    const auto started = std::chrono::steady_clock::now();
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    debug_.timestampS = input.timestampS;

    LocalPlanResult result;
    result.backend = LocalPlannerBackend::Scan;
    if (intent != nullptr &&
        (!std::isfinite(intent->directionBodyDeg) || !std::isfinite(intent->speedNormalized) ||
         !std::isfinite(intent->horizonM) || !std::isfinite(intent->maxDirectionDeviationDeg) ||
         intent->horizonM <= 0.0)) {
      result.status = LocalPlanStatus::InvalidInput;
      result.reason = "intent_invalid";
      finishDebug(started);
      return result;
    }
    if (intent != nullptr && intent->speedNormalized <= 1e-6) {
      result.status = LocalPlanStatus::NoPath;
      result.reason = "intent_idle";
      finishDebug(started);
      return result;
    }
    if (!input.route.valid() || !finitePoint(input.vehicle.position) ||
        !std::isfinite(input.vehicle.yaw) || !std::isfinite(input.timestampS)) {
      result.status = LocalPlanStatus::InvalidInput;
      result.reason = "route_invalid";
      finishDebug(started);
      return result;
    }

    const auto grid_started = std::chrono::steady_clock::now();
    Grid grid(params, input);
    debug_.gridTimeMs = elapsedMs(grid_started);
    debug_.occupiedCellCount = grid.occupiedCellCount();
    debug_.collisionPointCount = grid.collisionPointCount();
    debug_.legacyObstaclePointCount = grid.legacyObstaclePointCount();
    if (!grid.valid()) {
      result.status = LocalPlanStatus::InvalidInput;
      result.reason = grid.reason();
      debug_.searchReason = grid.reason();
      finishDebug(started);
      return result;
    }

    if (intent == nullptr) {
      const auto reuse_started = std::chrono::steady_clock::now();
      if (auto reused = reuseSafeTrajectory(grid, input, params)) {
        debug_.reuseTimeMs += elapsedMs(reuse_started);
        finishDebug(started);
        return *reused;
      }
      debug_.reuseTimeMs += elapsedMs(reuse_started);
    }

    const auto search_started = std::chrono::steady_clock::now();
    SearchResult searched = search(grid, input.vehicle.position, input.vehicle.yaw, params);
    debug_.searchTimeMs = elapsedMs(search_started);
    debug_.searchReason = searched.reason;
    debug_.expandedNodes = searched.expandedNodes;
    if (!searched.found()) {
      const bool blocked =
          searched.reason == "start_or_goal_blocked" || searched.reason == "start_obstacle_blocked";
      result.status = blocked ? LocalPlanStatus::Blocked : LocalPlanStatus::NoPath;
      result.reason = searched.reason;
      result.nearFieldStop = result.status == LocalPlanStatus::Blocked;
      clearPreviousIfIdentityChanged(input);
      finishDebug(started);
      return result;
    }

    bool continuity_reused = false;
    std::vector<Vec3> planning_path =
        reusePreviousPrefix(grid, input, searched.path, params, &continuity_reused);
    const auto spline_started = std::chrono::steady_clock::now();
    SplineResult spline = buildSpline(grid, planning_path, input, params);
    debug_.splineTimeMs = elapsedMs(spline_started);
    if (!spline.valid()) {
      result.status = LocalPlanStatus::NoPath;
      result.reason = "trajectory_collision";
      clearPreviousIfIdentityChanged(input);
      finishDebug(started);
      return result;
    }

    result.status = LocalPlanStatus::Ready;
    result.reason =
        searched.reason == "route_clear"
            ? "scan_route_ready"
            : (searched.boundaryFallback ? "scan_boundary_fallback_ready" : "scan_search_ready");
    result.pathFound = true;
    result.path.reserve(spline.path.size());
    for (const auto &point : spline.path) {
      result.path.push_back(planningPointToBody(input.vehicle, point));
    }
    result.trajectory.reserve(spline.trajectory.size());
    for (const auto &sample : spline.trajectory) {
      TrajectoryPoint body;
      body.position = planningPointToBody(input.vehicle, sample.position);
      body.velocity = planningVectorToBody(input.vehicle, sample.velocity);
      body.acceleration = planningVectorToBody(input.vehicle, sample.acceleration);
      body.yaw = normalizeAngle(sample.yaw - input.vehicle.yaw);
      body.yawRate = sample.yawRate;
      body.timeFromStartS = sample.timeFromStartS;
      result.trajectory.push_back(body);
    }

    if (intent != nullptr && !intentDirectionAllowed(result.path, *intent)) {
      result.status = LocalPlanStatus::NoPath;
      result.reason = "intent_direction_rejected";
      result.pathFound = false;
      result.path.clear();
      result.trajectory.clear();
      previous_.clear();
      previousTimestampS_ = -1.0;
      finishDebug(started);
      return result;
    }

    previous_ = spline.trajectory;
    previousTimestampS_ = input.timestampS;
    previousFrameEpoch_ = input.identity.frameEpoch;
    previousRouteGeneration_ = input.route.generation;
    previousObstacleGeneration_ = input.identity.obstacleGeneration;
    previousTraversabilityGeneration_ = input.identity.traversabilityGeneration;
    if (intent != nullptr)
      result.reason = "scan_intent_ready";

    debug_.valid = true;
    debug_.continuityReused = continuity_reused;
    debug_.splineFallback = spline.fallback;
    debug_.trajectoryPointCount = static_cast<int>(result.trajectory.size());
    finishDebug(started);
    return result;
  }

  static bool intentDirectionAllowed(const std::vector<Vec3> &body_path,
                                     const LocalMotionIntent &intent) {
    const double allowed = std::clamp(intent.maxDirectionDeviationDeg, 0.0, 180.0) * M_PI / 180.0;
    const double requested = std::remainder(intent.directionBodyDeg, 360.0) * M_PI / 180.0;
    for (std::size_t index = body_path.size(); index > 1U; --index) {
      const Vec3 &from = body_path[index - 2U];
      const Vec3 &to = body_path[index - 1U];
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      if (std::hypot(dx, dy) <= 1e-5)
        continue;
      const double error = normalizeAngle(std::atan2(dy, dx) - requested);
      return std::abs(error) <= allowed + 1e-9;
    }
    return false;
  }

  std::optional<LocalPlanResult> reuseSafeTrajectory(const Grid &grid, const LocalPlanInput &input,
                                                     const LocalPlannerParams &params) {
    const double age = input.timestampS - previousTimestampS_;
    if (previous_.size() < 2 || previousFrameEpoch_ != input.identity.frameEpoch ||
        previousRouteGeneration_ != input.route.generation || !std::isfinite(age) || age < 0.0 ||
        age > std::max(0.05, params.scan.continuityHorizon)) {
      return std::nullopt;
    }
    return reusedTrajectory(input, &grid,
                            input.timestampS + std::max(0.05, params.scan.continuityHorizon),
                            "scan_safe_trajectory_reused");
  }

  std::optional<LocalPlanResult> reusedTrajectory(const LocalPlanInput &input, const Grid *grid,
                                                  double absoluteEndS, const char *reason) {
    LocalPlanResult result;
    result.backend = LocalPlannerBackend::Scan;
    result.status = LocalPlanStatus::Ready;
    result.reason = reason;
    result.pathFound = true;
    result.path.push_back({0.0, 0.0, 0.0});

    TrajectoryPoint origin;
    origin.velocity = input.kinematics.valid
                          ? planningVectorToBody(input.vehicle, input.kinematics.linearVelocity)
                          : Vec3{};
    origin.timeFromStartS = 0.0;
    result.trajectory.push_back(origin);

    Vec3 previous_position = input.vehicle.position;
    for (const auto &sample : previous_) {
      const double absolute_time = previousTimestampS_ + sample.timeFromStartS;
      const double relative_time = absolute_time - input.timestampS;
      if (relative_time <= 1e-6)
        continue;
      if (absolute_time > absoluteEndS)
        break;
      if (grid != nullptr && !grid->segmentFree(previous_position, sample.position)) {
        break;
      }
      TrajectoryPoint body;
      body.position = planningPointToBody(input.vehicle, sample.position);
      body.velocity = planningVectorToBody(input.vehicle, sample.velocity);
      body.acceleration = planningVectorToBody(input.vehicle, sample.acceleration);
      body.yaw = normalizeAngle(sample.yaw - input.vehicle.yaw);
      body.yawRate = sample.yawRate;
      body.timeFromStartS = relative_time;
      result.path.push_back(body.position);
      result.trajectory.push_back(body);
      previous_position = sample.position;
    }
    if (result.trajectory.size() < 2 || result.trajectory.back().timeFromStartS < 0.05) {
      return std::nullopt;
    }

    debug_.valid = true;
    debug_.continuityReused = true;
    debug_.trajectoryPointCount = static_cast<int>(result.trajectory.size());
    return result;
  }

  void clearPreviousIfIdentityChanged(const LocalPlanInput &input) {
    if (input.identity.frameEpoch != previousFrameEpoch_ ||
        input.route.generation != previousRouteGeneration_) {
      previous_.clear();
      previousTimestampS_ = -1.0;
    }
  }

  std::vector<Vec3> reusePreviousPrefix(const Grid &grid, const LocalPlanInput &input,
                                        const std::vector<Vec3> &fresh,
                                        const LocalPlannerParams &params, bool *reused) const {
    *reused = false;
    if (previous_.size() < 2 || previousFrameEpoch_ != input.identity.frameEpoch ||
        previousRouteGeneration_ != input.route.generation ||
        input.timestampS < previousTimestampS_ ||
        input.timestampS - previousTimestampS_ > std::max(0.05, params.scan.continuityHorizon)) {
      return fresh;
    }

    std::vector<Vec3> prefix;
    prefix.push_back(input.vehicle.position);
    const double horizon = input.timestampS + std::max(0.05, params.scan.continuityHorizon);
    for (const auto &sample : previous_) {
      const double absolute_time = previousTimestampS_ + sample.timeFromStartS;
      if (absolute_time <= input.timestampS + 1e-6)
        continue;
      if (absolute_time > horizon)
        break;
      if (!grid.segmentFree(prefix.back(), sample.position))
        break;
      if (distance3D(prefix.back(), sample.position) > 1e-4) {
        prefix.push_back(sample.position);
      }
    }
    if (prefix.size() < 2)
      return fresh;

    std::size_t join = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    const double prefix_remaining = distance3D(prefix.back(), fresh.back());
    for (std::size_t i = 0; i < fresh.size(); ++i) {
      const double distance = distance3D(prefix.back(), fresh[i]);
      if (remainingDistance(fresh, i) <= prefix_remaining + grid.resolution() &&
          distance < best_distance && grid.segmentFree(prefix.back(), fresh[i])) {
        best_distance = distance;
        join = i;
      }
    }
    if (!std::isfinite(best_distance))
      return fresh;
    for (std::size_t i = join; i < fresh.size(); ++i) {
      if (distance3D(prefix.back(), fresh[i]) > 1e-4)
        prefix.push_back(fresh[i]);
    }
    if (prefix.size() < 2)
      return fresh;
    *reused = true;
    return prefix;
  }

  void finishDebug(std::chrono::steady_clock::time_point started) {
    debug_.planningTimeMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
  }

  static double elapsedMs(std::chrono::steady_clock::time_point started) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
        .count();
  }

  LocalPlannerParams params_;
  std::vector<TrajectorySample> previous_;
  double previousTimestampS_{-1.0};
  std::uint64_t previousFrameEpoch_{0};
  std::uint64_t previousRouteGeneration_{0};
  std::uint64_t previousObstacleGeneration_{0};
  std::uint64_t previousTraversabilityGeneration_{0};
  LocalPlannerDebugSnapshot debug_{};
};

Backend::Backend(const LocalPlannerParams &params) : impl_(std::make_unique<Impl>(params)) {}

Backend::~Backend() = default;
Backend::Backend(Backend &&) noexcept = default;
Backend &Backend::operator=(Backend &&) noexcept = default;

LocalPlanResult Backend::plan(const LocalPlanInput &input) {
  return impl_->plan(input);
}

LocalPlanResult Backend::planIntent(const LocalPlanInput &input, const LocalMotionIntent &intent) {
  return impl_->planIntent(input, intent);
}

void Backend::reset() {
  impl_->reset();
}

LocalPlannerDebugSnapshot Backend::debugSnapshot() const {
  return impl_->debugSnapshot();
}

}  // namespace nav_kernel::local::scan
