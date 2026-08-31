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

double executionTime(const LocalPlanRequest &input) {
  return std::isfinite(input.clock.executionTimeS) && input.clock.executionTimeS >= 0.0
             ? input.clock.executionTimeS
             : input.clock.timestampS;
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

TrajectorySample interpolateTrajectory(const std::vector<TrajectorySample> &trajectory,
                                       double time_from_start_s) {
  if (trajectory.empty())
    return {};
  const auto right = std::lower_bound(
      trajectory.begin(), trajectory.end(), time_from_start_s,
      [](const TrajectorySample &sample, double time) {
        return sample.timeFromStartS < time;
      });
  if (right == trajectory.begin())
    return *right;
  if (right == trajectory.end())
    return trajectory.back();

  const auto left = std::prev(right);
  const double span = right->timeFromStartS - left->timeFromStartS;
  const double ratio = span > 1e-9
                           ? std::clamp((time_from_start_s - left->timeFromStartS) / span,
                                        0.0, 1.0)
                           : 0.0;
  const auto blend = [ratio](const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a.x + ratio * (b.x - a.x),
        a.y + ratio * (b.y - a.y),
        a.z + ratio * (b.z - a.z),
    };
  };
  TrajectorySample sample;
  sample.position = blend(left->position, right->position);
  sample.velocity = blend(left->velocity, right->velocity);
  sample.acceleration = blend(left->acceleration, right->acceleration);
  sample.yaw = normalizeAngle(
      left->yaw + ratio * normalizeAngle(right->yaw - left->yaw));
  sample.yawRate = left->yawRate + ratio * (right->yawRate - left->yawRate);
  sample.timeFromStartS = time_from_start_s;
  return sample;
}

}  // namespace

class Backend::Impl {
 public:
  explicit Impl(LocalPlannerParams params) : params_(std::move(params)) {}

  LocalPlan plan(const LocalPlanRequest &input, const LocalPlanCancel &cancel = {}) {
    LocalPlannerParams params = params_;
    const auto *intent = input.intent();
    if (intent != nullptr) {
      params.autonomySpeed =
          params_.autonomySpeed * std::clamp(intent->speedNormalized, 0.0, 1.0);
    }
    return plan(input, params, intent, cancel);
  }

  void reset() {
    previous_.clear();
    previousSpline_ = {};
    previousTimestampS_ = -1.0;
    previousFrameEpoch_ = 0;
    previousRouteGeneration_ = 0;
    previousObstacleGeneration_ = 0;
    previousTraversabilityGeneration_ = 0;
    continuousFailures_ = 0;
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
  }

  LocalPlannerDebugSnapshot debugSnapshot() const { return debug_; }

  bool pathSafe(const LocalPlanRequest &input, const std::vector<Vec3> &planning_path) const {
    const auto *route = input.route();
    if (planning_path.size() < 2U || route == nullptr || !route->valid())
      return false;
    Grid grid(params_, input);
    if (!grid.valid())
      return false;
    if (!grid.free(input.robot.pose.position, input.robot.pose.yaw))
      return false;
    for (std::size_t index = 0U; index + 1U < planning_path.size(); ++index) {
      if (!grid.segmentFree(planning_path[index], planning_path[index + 1U]))
        return false;
    }
    return true;
  }

 private:
  LocalPlan plan(const LocalPlanRequest &input, const LocalPlannerParams &params,
                       const LocalMotionIntent *intent, const LocalPlanCancel &cancel) {
    const auto started = std::chrono::steady_clock::now();
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    debug_.timestampS = input.clock.timestampS;

    const auto cancelled = [&]() { return cancel && cancel(); };
    if (cancelled()) {
      debug_.searchReason = "planning_cancelled";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::Cancelled);
    }
    if (intent != nullptr &&
        (!std::isfinite(intent->directionBodyDeg) || !std::isfinite(intent->speedNormalized) ||
         !std::isfinite(intent->horizonM) || !std::isfinite(intent->maxDirectionDeviationDeg) ||
         intent->horizonM <= 0.0)) {
      debug_.searchReason = "intent_invalid";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::InvalidInput);
    }
    if (intent != nullptr && intent->speedNormalized <= 1e-6) {
      debug_.searchReason = "intent_idle";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::NoPath);
    }
    const auto *route = input.route();
    const auto &vehicle = input.robot.pose;
    if (route == nullptr || !route->valid() || !finitePoint(vehicle.position) ||
        !std::isfinite(vehicle.yaw) || !std::isfinite(input.clock.timestampS)) {
      debug_.searchReason = "route_invalid";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::InvalidInput);
    }

    const auto grid_started = std::chrono::steady_clock::now();
    Grid grid(params, input);
    debug_.gridTimeMs = elapsedMs(grid_started);
    debug_.occupiedCellCount = grid.occupiedCellCount();
    debug_.collisionPointCount = grid.collisionPointCount();
    if (!grid.valid()) {
      debug_.searchReason = grid.reason();
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::InvalidInput);
    }
    if (cancelled()) {
      debug_.searchReason = "planning_cancelled";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::Cancelled);
    }

    const double current_execution_time = executionTime(input);
    const double previous_age = current_execution_time - previousTimestampS_;
    if (intent == nullptr && previousSpline_.valid() &&
        previousFrameEpoch_ == input.identity.frameEpoch &&
        previousRouteGeneration_ == route->generation &&
        std::isfinite(previous_age) && previous_age >= 0.0 && previous_age <= 1e-6 &&
        previousTracksVehicle(input, params, previous_age)) {
      const auto reuse_started = std::chrono::steady_clock::now();
      if (auto reused = reuseSafeTrajectory(grid, input, params)) {
        debug_.searchReason = "scan_frozen_trajectory_reused";
        debug_.reuseTimeMs += elapsedMs(reuse_started);
        finishDebug(started);
        return *reused;
      }
      debug_.reuseTimeMs += elapsedMs(reuse_started);
    }

    const auto safeFallback = [&]() -> std::optional<LocalPlan> {
      if (intent != nullptr)
        return std::nullopt;
      const auto reuse_started = std::chrono::steady_clock::now();
      auto reused = reuseSafeTrajectory(grid, input, params);
      debug_.reuseTimeMs += elapsedMs(reuse_started);
      return reused;
    };

    const auto search_started = std::chrono::steady_clock::now();
    SearchResult searched = search(grid, vehicle.position, vehicle.yaw, params, cancel);
    debug_.searchTimeMs = elapsedMs(search_started);
    debug_.searchReason = searched.reason;
    debug_.expandedNodes = searched.expandedNodes;
    if (!searched.found()) {
      const bool blocked =
          searched.reason == "start_or_goal_blocked" || searched.reason == "start_obstacle_blocked";
      if (searched.reason != "planning_cancelled")
        ++continuousFailures_;
      if (searched.reason != "planning_cancelled") {
        if (auto reused = safeFallback()) {
          finishDebug(started);
          return *reused;
        }
      }
      clearPreviousIfIdentityChanged(input);
      finishDebug(started);
      return LocalPlan::stopped(blocked ? LocalPlanStatus::NearFieldStop
                                        : (searched.reason == "planning_cancelled"
                                               ? LocalPlanStatus::Cancelled
                                               : LocalPlanStatus::NoPath));
    }

    SeedHistory history;
    if (previousSpline_.valid() && previousFrameEpoch_ == input.identity.frameEpoch &&
        previousRouteGeneration_ == route->generation && std::isfinite(previous_age) &&
        previous_age >= 0.0 &&
        previous_age <= previousSpline_.duration() + 1e-6 &&
        previousTracksVehicle(input, params, previous_age)) {
      history = {&previousSpline_, previousTimestampS_};
    }
    const auto spline_started = std::chrono::steady_clock::now();
    if (cancelled()) {
      debug_.searchReason = "planning_cancelled";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::Cancelled);
    }
    SplineResult spline =
        buildSpline(grid, searched.path, input, params, cancel, history, continuousFailures_);
    debug_.splineTimeMs = elapsedMs(spline_started);
    debug_.reboundRestarts = spline.reboundRestarts;
    debug_.optimizerEvaluations = spline.optimizerEvaluations;
    debug_.collisionSegments = spline.collisionSegments;
    debug_.anchorSearches = spline.anchorSearches;
    if (cancelled()) {
      debug_.searchReason = "planning_cancelled";
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::Cancelled);
    }
    if (!spline.valid()) {
      debug_.searchReason = spline.reason;
      ++continuousFailures_;
      if (auto reused = safeFallback()) {
        finishDebug(started);
        return *reused;
      }
      clearPreviousIfIdentityChanged(input);
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::NoPath);
    }

    debug_.searchReason =
        searched.reason == "route_clear"
            ? "scan_route_ready"
            : (searched.boundaryFallback ? "scan_boundary_fallback_ready" : "scan_search_ready");
    std::vector<Vec3> body_path;
    body_path.reserve(spline.path.size());
    for (const auto &point : spline.path) {
      body_path.push_back(planningPointToBody(vehicle, point));
    }
    SplineTarget target;
    target.controls.reserve(spline.controls.size());
    for (const Vec3 &control : spline.controls) {
      target.controls.push_back(planningPointToBody(vehicle, control));
    }
    target.degree = 3;
    target.intervalS = spline.interval;
    target.startTimeS =
        spline.seedMode == SeedMode::Previous ? current_execution_time : input.clock.timestampS;
    target.timeS = 0.0;

    if (intent != nullptr && !intentDirectionAllowed(body_path, *intent)) {
      debug_.searchReason = "intent_direction_rejected";
      previous_.clear();
      previousSpline_ = {};
      previousTimestampS_ = -1.0;
      finishDebug(started);
      return LocalPlan::stopped(LocalPlanStatus::NoPath);
    }

    previous_ = spline.trajectory;
    previousSpline_ = UniformSpline(spline.controls, 3, spline.interval);
    previousTimestampS_ = current_execution_time;
    previousFrameEpoch_ = input.identity.frameEpoch;
    previousRouteGeneration_ = route->generation;
    previousObstacleGeneration_ = input.identity.obstacleGeneration;
    previousTraversabilityGeneration_ = input.identity.traversabilityGeneration;
    continuousFailures_ = 0;
    if (intent != nullptr)
      debug_.searchReason = "scan_intent_ready";

    debug_.valid = true;
    debug_.continuityReused = spline.seedMode == SeedMode::Previous;
    debug_.splineFallback = spline.fallback;
    debug_.trajectoryPointCount = static_cast<int>(spline.trajectory.size());
    finishDebug(started);
    return LocalPlan::spline(std::move(target));
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

  std::optional<LocalPlan> reuseSafeTrajectory(const Grid &grid, const LocalPlanRequest &input,
                                                     const LocalPlannerParams &params) {
    const auto *route = input.route();
    const double current_execution_time = executionTime(input);
    const double age = current_execution_time - previousTimestampS_;
    const double previous_duration =
        previous_.empty() ? 0.0 : previous_.back().timeFromStartS;
    if (route == nullptr || previous_.size() < 2 ||
        previousFrameEpoch_ != input.identity.frameEpoch ||
        previousRouteGeneration_ != route->generation || !std::isfinite(age) || age < 0.0 ||
        !std::isfinite(previous_duration) || age + 0.05 > previous_duration ||
        !previousTracksVehicle(input, params, age)) {
      return std::nullopt;
    }
    return reusedTrajectory(input, &grid,
                            current_execution_time +
                                std::max(0.05, params.scan.continuityHorizon),
                            "scan_safe_trajectory_reused");
  }

  std::optional<LocalPlan> reusedTrajectory(const LocalPlanRequest &input, const Grid *grid,
                                                  double absoluteEndS, const char *reason) {
    const auto &vehicle = input.robot.pose;
    const double current_execution_time = executionTime(input);
    const double previous_age = current_execution_time - previousTimestampS_;
    SplineTarget target;
    target.controls.reserve(previousSpline_.controls().size());
    for (const Vec3 &control : previousSpline_.controls()) {
      target.controls.push_back(planningPointToBody(vehicle, control));
    }
    target.degree = previousSpline_.degree();
    target.intervalS = previousSpline_.interval();
    target.startTimeS = current_execution_time;
    target.timeS = std::max(0.0, previous_age);
    const TrajectorySample desired_sample =
        interpolateTrajectory(previous_, std::max(0.0, previous_age));
    if (grid != nullptr &&
        !grid->segmentFree(vehicle.position, desired_sample.position)) {
      return std::nullopt;
    }

    Vec3 previous_position = desired_sample.position;
    std::size_t future_samples = 1U;
    double final_relative_time = 0.0;
    for (const auto &sample : previous_) {
      const double absolute_time = previousTimestampS_ + sample.timeFromStartS;
      const double relative_time = absolute_time - current_execution_time;
      if (relative_time <= 1e-6)
        continue;
      if (absolute_time > absoluteEndS)
        break;
      if (grid != nullptr && !grid->segmentFree(previous_position, sample.position)) {
        break;
      }
      previous_position = sample.position;
      final_relative_time = relative_time;
      ++future_samples;
    }
    if (future_samples < 2U || final_relative_time < 0.05) {
      return std::nullopt;
    }

    debug_.valid = true;
    debug_.continuityReused = true;
    debug_.trajectoryPointCount = static_cast<int>(future_samples);
    debug_.searchReason = reason;
    return LocalPlan::spline(std::move(target));
  }

  bool previousTracksVehicle(const LocalPlanRequest &input,
                             const LocalPlannerParams &params, double age) const {
    if (!previousSpline_.valid() || !std::isfinite(age) || age < 0.0 ||
        age > previousSpline_.duration() + 1e-6) {
      return false;
    }
    const Vec3 expected = previousSpline_.evaluate(age);
    const double tolerance = std::max({
        0.15,
        0.5 * std::max(0.0, params.vehicleLength),
        1.5 * std::max(0.05, params.scan.controlPointSpacing),
    });
    return std::hypot(expected.x - input.robot.pose.position.x,
                      expected.y - input.robot.pose.position.y) <= tolerance;
  }

  void clearPreviousIfIdentityChanged(const LocalPlanRequest &input) {
    const auto *route = input.route();
    if (input.identity.frameEpoch != previousFrameEpoch_ ||
        route == nullptr || route->generation != previousRouteGeneration_) {
      previous_.clear();
      previousSpline_ = {};
      previousTimestampS_ = -1.0;
    }
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
  UniformSpline previousSpline_;
  double previousTimestampS_{-1.0};
  std::uint64_t previousFrameEpoch_{0};
  std::uint64_t previousRouteGeneration_{0};
  std::uint64_t previousObstacleGeneration_{0};
  std::uint64_t previousTraversabilityGeneration_{0};
  int continuousFailures_{0};
  LocalPlannerDebugSnapshot debug_{};
};

Backend::Backend(const LocalPlannerParams &params) : impl_(std::make_unique<Impl>(params)) {}

Backend::~Backend() = default;
Backend::Backend(Backend &&) noexcept = default;
Backend &Backend::operator=(Backend &&) noexcept = default;

LocalPlan Backend::plan(const LocalPlanRequest &input) {
  return impl_->plan(input);
}

LocalPlan Backend::plan(const LocalPlanRequest &input, const LocalPlanCancel &cancel) {
  return impl_->plan(input, cancel);
}

bool Backend::pathSafe(const LocalPlanRequest &input, const std::vector<Vec3> &planningPath) const {
  return impl_->pathSafe(input, planningPath);
}

void Backend::reset() {
  impl_->reset();
}

LocalPlannerDebugSnapshot Backend::debugSnapshot() const {
  return impl_->debugSnapshot();
}

}  // namespace nav_kernel::local::scan
