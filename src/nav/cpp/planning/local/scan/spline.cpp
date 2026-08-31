// SCAN planner-manager rebound/refine flow ported from upstream 348e8a5.
// Modified for LingTu's ROS-free trajectory and local-planner contracts.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/spline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "planning/local/scan/optimizer.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel::local::scan {
namespace {

Vec3 add(const Vec3 &left, const Vec3 &right) {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 subtract(const Vec3 &left, const Vec3 &right) {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 scale(const Vec3 &value, double factor) {
  return {factor * value.x, factor * value.y, factor * value.z};
}

double norm(const Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double pointChordDistance(const Vec3 &point, const Vec3 &start, const Vec3 &end) {
  const Vec3 chord = subtract(end, start);
  const double chord_squared = chord.x * chord.x + chord.y * chord.y + chord.z * chord.z;
  if (chord_squared <= 1e-12)
    return norm(subtract(point, start));
  const Vec3 delta = subtract(point, start);
  const double ratio = std::clamp(
      (delta.x * chord.x + delta.y * chord.y + delta.z * chord.z) / chord_squared,
      0.0, 1.0);
  return norm(subtract(point, add(start, scale(chord, ratio))));
}

bool guideHasMaterialShape(const std::vector<Vec3> &guide,
                           const LocalPlannerParams &params) {
  if (guide.size() <= 2U)
    return false;
  const double threshold = std::max(
      {0.15, 0.5 * std::max(0.0, params.vehicleWidth),
       2.0 * std::max(0.02, params.scan.voxelResolution)});
  for (std::size_t index = 1U; index + 1U < guide.size(); ++index) {
    if (pointChordDistance(guide[index], guide.front(), guide.back()) > threshold)
      return true;
  }
  return false;
}

double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double derivativeNormBound(const UniformSpline &spline) {
  double bound = 0.0;
  for (const Vec3 &control : spline.controls()) {
    bound = std::max(bound, norm(control));
  }
  return bound;
}

std::vector<Vec3> samplePath(const UniformSpline &spline, double spacing, double max_chord_error) {
  if (!spline.valid())
    return {};
  const double duration = spline.duration();
  const UniformSpline velocity = spline.derivative();
  const double speed_bound = derivativeNormBound(velocity);
  const double acceleration_bound = derivativeNormBound(velocity.derivative());
  double sample_dt = duration;
  if (speed_bound > 1e-9) {
    sample_dt = std::min(sample_dt, std::max(0.02, spacing) / speed_bound);
  }
  if (acceleration_bound > 1e-9) {
    // Linear interpolation of a twice-differentiable curve has error at most
    // max(|p''|) * dt^2 / 8. Bound p'' by the derivative spline's control hull.
    sample_dt =
        std::min(sample_dt, std::sqrt(8.0 * std::max(1e-6, max_chord_error) / acceleration_bound));
  }
  sample_dt = std::clamp(sample_dt, 1e-4, std::max(1e-4, duration));
  const std::size_t count =
      std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(duration / sample_dt)) + 1U);
  std::vector<Vec3> path;
  path.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    path.push_back(
        spline.evaluate(duration * static_cast<double>(index) / static_cast<double>(count - 1U)));
  }
  return path;
}

bool collisionFree(const Grid &grid, const std::vector<Vec3> &path) {
  if (path.size() < 2U)
    return false;
  for (std::size_t index = 0U; index + 1U < path.size(); ++index) {
    if (!grid.segmentFree(path[index], path[index + 1U]))
      return false;
  }
  return true;
}

double verticalScale(const UniformSpline &spline, double max_vertical_speed) {
  if (!(max_vertical_speed > 0.0))
    return 1.0;
  double observed = 0.0;
  for (const Vec3 &velocity : spline.derivative().controls()) {
    observed = std::max(observed, std::abs(velocity.z));
  }
  return std::max(1.0, observed / max_vertical_speed);
}

double vectorScale(const UniformSpline &spline, double max_speed,
                   double max_acceleration) {
  if (!spline.valid() || !(max_speed > 0.0) || !(max_acceleration > 0.0))
    return std::numeric_limits<double>::infinity();
  double observed_speed = 0.0;
  double observed_acceleration = 0.0;
  const UniformSpline velocity = spline.derivative();
  const UniformSpline acceleration = velocity.derivative();
  for (const Vec3 &value : velocity.controls())
    observed_speed = std::max(observed_speed, norm(value));
  for (const Vec3 &value : acceleration.controls())
    observed_acceleration = std::max(observed_acceleration, norm(value));
  return std::max({1.0, observed_speed / max_speed,
                   std::sqrt(observed_acceleration / max_acceleration)});
}

std::vector<Vec3> sampleReference(const UniformSpline &spline) {
  if (!spline.valid() || spline.controls().size() < 6U)
    return {};
  const std::size_t segments = spline.controls().size() - 3U;
  std::vector<Vec3> reference;
  reference.reserve(segments + 1U);
  for (std::size_t i = 0U; i <= segments; ++i) {
    reference.push_back(spline.evaluate(
        spline.duration() * static_cast<double>(i) /
        static_cast<double>(segments)));
  }
  return reference;
}

bool reparameterize(const UniformSpline &source,
                    const std::array<Vec3, 4> &boundary, double ratio,
                    double *interval, std::vector<Vec3> *controls,
                    std::vector<Vec3> *reference) {
  if (interval == nullptr || controls == nullptr || reference == nullptr ||
      !source.valid() || !std::isfinite(ratio) || ratio < 1.0)
    return false;
  const std::size_t segments = source.controls().size() - 3U;
  if (segments == 0U)
    return false;
  const UniformSpline slowed(source.controls(), 3, source.interval() * ratio);
  *interval = slowed.duration() / static_cast<double>(segments);
  std::vector<Vec3> samples;
  samples.reserve(segments + 1U);
  for (std::size_t i = 0U; i <= segments; ++i) {
    samples.push_back(slowed.evaluate(
        slowed.duration() * static_cast<double>(i) /
        static_cast<double>(segments)));
  }
  *controls = UniformSpline::parameterize(*interval, samples, boundary);
  const UniformSpline parameterized(*controls, 3, *interval);
  *reference = sampleReference(parameterized);
  return parameterized.valid() && reference->size() + 2U >= controls->size();
}

bool dynamicFeasible(const UniformSpline &spline, double command_speed,
                     const ScanPlannerParams &params) {
  if (!spline.valid())
    return false;
  const UniformSpline velocity = spline.derivative();
  const UniformSpline acceleration = velocity.derivative();
  const double duration = spline.duration();
  const double dt = std::clamp(duration / 50.0, 0.01, 0.05);
  const double fail_safe_speed =
      command_speed + std::max(0.0, params.velocityTolerance);
  const double fail_safe_acceleration =
      std::max(0.05, params.maxAcceleration) +
      std::max(0.0, params.accelerationTolerance);
  for (double time = 0.0; time < duration + 1e-6; time += dt) {
    const double bounded = std::min(time, duration);
    const Vec3 speed = velocity.evaluate(bounded);
    const Vec3 accel = acceleration.evaluate(bounded);
    if (norm(speed) > fail_safe_speed + 1e-4 ||
        norm(speed) > command_speed + 1e-3 ||
        norm(accel) > fail_safe_acceleration + 1e-4 ||
        std::abs(speed.z) > std::max(0.02, params.maxVerticalSpeed) + 1e-4) {
      return false;
    }
  }
  return true;
}

std::vector<TrajectorySample> sampleTrajectory(const UniformSpline &position, double sample_spacing,
                                               double max_speed, double initial_yaw) {
  if (!position.valid())
    return {};
  const UniformSpline velocity = position.derivative();
  const UniformSpline acceleration = velocity.derivative();
  const double duration = position.duration();
  const double requested_dt = sample_spacing / std::max(0.05, max_speed);
  const double sample_dt = std::clamp(requested_dt, 0.01, 0.05);
  const std::size_t count =
      std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(duration / sample_dt)) + 1U);
  std::vector<TrajectorySample> trajectory(count);
  double yaw = initial_yaw;
  for (std::size_t index = 0U; index < count; ++index) {
    const double time = duration * static_cast<double>(index) / static_cast<double>(count - 1U);
    TrajectorySample &sample = trajectory[index];
    sample.timeFromStartS = time;
    sample.position = position.evaluate(time);
    sample.velocity = velocity.evaluate(time);
    sample.acceleration = acceleration.evaluate(time);
    if (std::hypot(sample.velocity.x, sample.velocity.y) > 1e-5) {
      yaw = std::atan2(sample.velocity.y, sample.velocity.x);
    }
    sample.yaw = yaw;
    if (index > 0U) {
      const double dt = time - trajectory[index - 1U].timeFromStartS;
      sample.yawRate = wrap(yaw - trajectory[index - 1U].yaw) / dt;
    }
  }
  if (trajectory.size() > 1U)
    trajectory.front().yawRate = trajectory[1U].yawRate;
  return trajectory;
}

}  // namespace

SplineResult buildSpline(const Grid &grid, const std::vector<Vec3> &path,
                         const LocalPlanRequest &input, const LocalPlannerParams &params,
                         const LocalPlanCancel &cancel, const SeedHistory &history,
                         int failures) {
  SplineResult result;
  if (path.size() < 2U || (cancel && cancel())) {
    result.reason = "spline_input_invalid";
    return result;
  }

  const double max_speed = std::clamp(params.autonomySpeed, 0.05, std::max(0.05, params.maxSpeed));
  const double collision_spacing =
      std::min(std::max(0.02, params.scan.sampleSpacing), 0.5 * grid.resolution());
  const double max_chord_error = 0.05 * grid.resolution();
  std::vector<SeedMode> modes;
  if (history.trajectory != nullptr && history.trajectory->valid())
    modes.push_back(SeedMode::Previous);
  const bool shaped_guide = guideHasMaterialShape(path, params);
  if (shaped_guide)
    modes.push_back(SeedMode::Guide);
  modes.push_back(SeedMode::Polynomial);
  if (!shaped_guide)
    modes.push_back(SeedMode::Guide);
  if (failures > 0)
    modes.push_back(SeedMode::RandomPolynomial);

  for (const SeedMode mode : modes) {
    if (cancel && cancel())
      return {};
    const SeedResult seed = buildSeed(path, input, params, mode, history, failures);
    if (!seed.valid()) {
      result.reason = "seed_invalid";
      continue;
    }
    double interval = seed.interval;
    std::vector<Vec3> controls =
        UniformSpline::parameterize(interval, seed.samples, seed.boundary);
    if (controls.size() < 6U) {
      result.reason = "parameterization_failed";
      continue;
    }

    LocalPlannerParams optimizer_params = params;
    ReboundState rebound_state;
    bool collision_resolved = false;
    const int max_restarts = std::clamp(params.scan.maxReboundRestarts, 0, 8);
    for (int attempt = 0; attempt <= max_restarts; ++attempt) {
      if (cancel && cancel())
        return {};
      ReboundOptimizationResult optimized =
          optimizeRebound(grid, controls, interval, optimizer_params,
                          &rebound_state, cancel);
      if (cancel && cancel())
        return {};
      result.optimizerUsed = result.optimizerUsed || optimized.attempted;
      result.zGradientSuppressed = result.zGradientSuppressed || optimized.attempted;
      result.optimizerStatus = optimized.status;
      result.optimizerEvaluations += optimized.evaluations;
      result.optimizerInitialCost += optimized.initialCost;
      result.optimizerFinalCost += optimized.finalCost;
      result.collisionSegments += optimized.collisionSegments;
      result.anchorSearches += optimized.anchorSearches;
      result.reboundRestarts += optimized.reboundPasses;
      if (!optimized.anchorsComplete) {
        result.reason = optimized.failureReason.empty()
                            ? "anchor_search_failed"
                            : "anchor_search_failed_" + optimized.failureReason;
        break;
      }
      controls = std::move(optimized.controls);
      const UniformSpline candidate(controls, 3, interval);
      if (candidate.valid() &&
          collisionFree(grid, samplePath(candidate, collision_spacing, max_chord_error))) {
        collision_resolved = true;
        break;
      }
      ++result.reboundRestarts;
      optimizer_params.scan.collisionWeight *= 2.0;
    }
    if (!collision_resolved) {
      if (result.reason.rfind("anchor_search_failed", 0U) != 0U)
        result.reason = "rebound_collision";
      continue;
    }

    UniformSpline spline(controls, 3, interval);
    double feasibility_scale =
        spline.feasibilityRatio(max_speed, std::max(0.05, params.scan.maxAcceleration),
                                std::max(0.0, params.scan.feasibilityTolerance));
    double vertical_scale =
        verticalScale(spline, std::max(0.02, params.scan.maxVerticalSpeed));
    double command_scale =
        vectorScale(spline, max_speed, std::max(0.05, params.scan.maxAcceleration));
    double time_scale = std::max({feasibility_scale, vertical_scale, command_scale});
    if (!std::isfinite(time_scale)) {
      result.reason = "time_scale_invalid";
      continue;
    }
    if (time_scale > 1.0 + 1e-6) {
      const std::vector<Vec3> rebound_controls = controls;
      const double rebound_interval = interval;
      std::vector<Vec3> reference;
      if (!reparameterize(spline, seed.boundary, time_scale, &interval,
                          &controls, &reference)) {
        result.reason = "reparameterization_failed";
        continue;
      }
      LocalPlannerParams refine_params = params;
      RefineOptimizationResult refined =
          optimizeRefine(controls, reference, interval, refine_params, cancel);
      result.refineUsed = result.refineUsed || refined.attempted;
      result.zGradientSuppressed = result.zGradientSuppressed || refined.attempted;
      result.refineStatus = refined.status;
      result.refineEvaluations += refined.evaluations;
      result.refineInitialCost += refined.initialCost;
      result.refineFinalCost += refined.finalCost;
      controls = std::move(refined.controls);
      spline = UniformSpline(controls, 3, interval);
      result.timeReallocated = true;
      if (!spline.valid()) {
        result.reason = "refine_invalid";
        continue;
      }

      const std::vector<Vec3> refined_path =
          samplePath(spline, collision_spacing, max_chord_error);
      if (!collisionFree(grid, refined_path)) {
        // Refinement is allowed to improve fitness only while preserving the
        // collision-free rebound geometry. Uniform time extension is the safe
        // fallback and still satisfies the requested dynamic reallocation.
        controls = rebound_controls;
        interval = rebound_interval * time_scale;
        spline = UniformSpline(controls, 3, interval);
      }

      feasibility_scale =
          spline.feasibilityRatio(max_speed,
                                  std::max(0.05, params.scan.maxAcceleration),
                                  0.0);
      vertical_scale =
          verticalScale(spline, std::max(0.02, params.scan.maxVerticalSpeed));
      command_scale =
          vectorScale(spline, max_speed,
                      std::max(0.05, params.scan.maxAcceleration));
      time_scale = std::max({feasibility_scale, vertical_scale, command_scale});
      if (!std::isfinite(time_scale)) {
        result.reason = "refine_time_scale_invalid";
        continue;
      }
      if (time_scale > 1.0 + 1e-6) {
        // The official refine objective may trade a small amount of dynamic
        // margin for fitness. Uniformly extending the final knot interval keeps
        // its collision-free geometry while restoring the hard command limit.
        interval *= time_scale;
        spline = UniformSpline(controls, 3, interval);
      }
    }

    std::vector<Vec3> sampled = samplePath(spline, collision_spacing, max_chord_error);
    if (!collisionFree(grid, sampled)) {
      result.reason = "final_collision";
      continue;
    }
    if (!dynamicFeasible(spline, max_speed, params.scan)) {
      result.reason = "dynamic_infeasible";
      continue;
    }
    std::vector<TrajectorySample> trajectory =
        sampleTrajectory(spline, std::max(0.02, params.scan.sampleSpacing), max_speed,
                         input.robot.pose.yaw);
    if (trajectory.size() < 2U) {
      result.reason = "trajectory_sampling_failed";
      continue;
    }
    result.seedMode = mode;
    result.fallback = mode == SeedMode::RandomPolynomial;
    result.controls = std::move(controls);
    result.interval = interval;
    result.dynamicFeasible = true;
    result.reason = "spline_ready";
    result.path = std::move(sampled);
    result.trajectory = std::move(trajectory);
    return result;
  }
  return result;
}

}  // namespace nav_kernel::local::scan
