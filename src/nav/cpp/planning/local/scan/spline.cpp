#include "planning/local/scan/spline.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "planning/local/scan/optimizer.hpp"
#include "planning/local/scan/uniform.hpp"

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

Vec3 normalized(const Vec3 &value) {
  const double length = norm(value);
  return length > 1e-9 ? scale(value, 1.0 / length) : Vec3{};
}

double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<Vec3> resample(const std::vector<Vec3> &path, double requested_spacing,
                           std::size_t minimum_count) {
  if (path.size() < 2U)
    return path;
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    arc[index] = arc[index - 1U] + distance3D(path[index - 1U], path[index]);
  }
  const double total = arc.back();
  if (total <= 1e-9)
    return {};
  const double spacing = std::max(0.02, requested_spacing);
  const std::size_t count =
      std::max(minimum_count, static_cast<std::size_t>(std::ceil(total / spacing)) + 1U);

  std::vector<Vec3> samples;
  samples.reserve(count);
  std::size_t segment = 1U;
  for (std::size_t index = 0U; index < count; ++index) {
    const double target = total * static_cast<double>(index) / static_cast<double>(count - 1U);
    while (segment + 1U < arc.size() && arc[segment] < target)
      ++segment;
    const double segment_length = arc[segment] - arc[segment - 1U];
    const double ratio =
        segment_length > 1e-12 ? (target - arc[segment - 1U]) / segment_length : 0.0;
    samples.push_back(
        add(path[segment - 1U], scale(subtract(path[segment], path[segment - 1U]), ratio)));
  }
  samples.front() = path.front();
  samples.back() = path.back();
  return samples;
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
                         const LocalPlanInput &input, const LocalPlannerParams &params) {
  SplineResult result;
  if (path.size() < 2U)
    return result;

  const double max_speed = std::clamp(params.autonomySpeed, 0.05, std::max(0.05, params.maxSpeed));
  const double control_spacing =
      std::max(grid.resolution(), std::max(0.05, params.scan.controlPointSpacing));
  const std::vector<Vec3> samples = resample(path, control_spacing, 7U);
  if (samples.size() < 4U)
    return result;

  const Vec3 start_velocity = input.kinematics.valid ? input.kinematics.linearVelocity : Vec3{};
  const Vec3 start_acceleration =
      input.kinematics.valid ? input.kinematics.linearAcceleration : Vec3{};
  const Vec3 end_tangent = normalized(subtract(samples.back(), samples[samples.size() - 2U]));
  const bool stops_at_local_frontier =
      distance3D(path.back(), grid.route().back()) > grid.resolution();
  const Vec3 end_velocity =
      input.route.reachesGoal || stops_at_local_frontier ? Vec3{} : scale(end_tangent, max_speed);
  const std::array<Vec3, 4> boundary_derivatives{
      start_velocity,
      end_velocity,
      start_acceleration,
      Vec3{},
  };

  // SCAN upstream initializes ts as control-point distance / max velocity * 1.2.
  double interval = std::max(0.03, 1.2 * control_spacing / max_speed);
  std::vector<Vec3> controls = UniformSpline::parameterize(interval, samples, boundary_derivatives);
  if (controls.size() < 6U)
    return result;

  const double collision_spacing =
      std::min(std::max(0.02, params.scan.sampleSpacing), 0.5 * grid.resolution());
  const double max_chord_error = 0.05 * grid.resolution();
  LocalPlannerParams optimizer_params = params;
  bool collision_resolved = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    ReboundOptimizationResult optimized =
        optimizeRebound(grid, controls, path, interval, optimizer_params);
    result.optimizerUsed = result.optimizerUsed || optimized.attempted;
    result.zGradientSuppressed = result.zGradientSuppressed || optimized.attempted;
    result.optimizerStatus = optimized.status;
    result.optimizerEvaluations += optimized.evaluations;
    result.optimizerInitialCost += optimized.initialCost;
    result.optimizerFinalCost += optimized.finalCost;
    controls = std::move(optimized.controls);
    const UniformSpline candidate(controls, 3, interval);
    if (candidate.valid() &&
        collisionFree(grid, samplePath(candidate, collision_spacing, max_chord_error))) {
      collision_resolved = true;
      break;
    }
    optimizer_params.scan.collisionWeight *= 2.0;
  }
  if (!collision_resolved)
    return result;

  UniformSpline spline(controls, 3, interval);
  const double feasibility_scale =
      spline.feasibilityRatio(max_speed, std::max(0.05, params.scan.maxAcceleration),
                              std::max(0.0, params.scan.feasibilityTolerance));
  const double vertical_scale = verticalScale(spline, std::max(0.02, params.scan.maxVerticalSpeed));
  const double time_scale = std::max(feasibility_scale, vertical_scale);
  if (!std::isfinite(time_scale))
    return result;
  if (time_scale > 1.0 + 1e-6) {
    interval *= time_scale;
    spline = UniformSpline(controls, 3, interval);
  }

  result.path = samplePath(spline, collision_spacing, max_chord_error);
  if (!collisionFree(grid, result.path)) {
    result.path.clear();
    return result;
  }
  result.trajectory = sampleTrajectory(spline, std::max(0.02, params.scan.sampleSpacing), max_speed,
                                       input.vehicle.yaw);
  if (result.trajectory.size() < 2U)
    result.path.clear();
  return result;
}

}  // namespace nav_kernel::local::scan
