// Adapted from SCAN-Planner PolynomialTraj/planner_manager at upstream 348e8a5.
// Modified for LingTu: ROS/Eigen-free dense solves and deterministic retry sampling.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/seed.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace nav_kernel::local::scan {
namespace {

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(const Vec3 &value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

double norm(const Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double pointSegmentDistance(const Vec3 &point, const Vec3 &start,
                            const Vec3 &end) {
  const Vec3 segment = subtract(end, start);
  const double length_squared = segment.x * segment.x + segment.y * segment.y +
                                segment.z * segment.z;
  if (length_squared <= 1e-12)
    return norm(subtract(point, start));
  const Vec3 delta = subtract(point, start);
  const double ratio = std::clamp(
      (delta.x * segment.x + delta.y * segment.y + delta.z * segment.z) /
          length_squared,
      0.0, 1.0);
  return norm(subtract(point, add(start, scale(segment, ratio))));
}

void simplifyGuideRange(const std::vector<Vec3> &guide, std::size_t first,
                        std::size_t last, double tolerance,
                        std::vector<unsigned char> *keep) {
  if (keep == nullptr || last <= first + 1U)
    return;
  double greatest = 0.0;
  std::size_t split = first;
  for (std::size_t index = first + 1U; index < last; ++index) {
    const double distance =
        pointSegmentDistance(guide[index], guide[first], guide[last]);
    if (distance > greatest) {
      greatest = distance;
      split = index;
    }
  }
  if (greatest <= tolerance)
    return;
  (*keep)[split] = 1U;
  simplifyGuideRange(guide, first, split, tolerance, keep);
  simplifyGuideRange(guide, split, last, tolerance, keep);
}

std::vector<Vec3> seedWaypoints(const std::vector<Vec3> &guide,
                                double control_spacing) {
  if (guide.size() <= 2U)
    return guide;
  constexpr std::size_t kMaxWaypoints = 12U;
  double tolerance = 0.5 * std::max(0.05, control_spacing);
  std::vector<Vec3> simplified;
  for (int attempt = 0; attempt < 8; ++attempt) {
    std::vector<unsigned char> keep(guide.size(), 0U);
    keep.front() = 1U;
    keep.back() = 1U;
    simplifyGuideRange(guide, 0U, guide.size() - 1U, tolerance, &keep);
    simplified.clear();
    for (std::size_t index = 0U; index < guide.size(); ++index) {
      if (keep[index] != 0U &&
          (simplified.empty() || distance3D(simplified.back(), guide[index]) > 1e-5)) {
        simplified.push_back(guide[index]);
      }
    }
    if (simplified.size() <= kMaxWaypoints)
      return simplified;
    tolerance *= 1.5;
  }
  return {guide.front(), guide.back()};
}

Vec3 normalized(const Vec3 &value) {
  const double length = norm(value);
  return length > 1e-9 ? scale(value, 1.0 / length) : Vec3{};
}

bool finite(const Vec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double executionTime(const LocalPlanRequest &input) {
  return std::isfinite(input.clock.executionTimeS) && input.clock.executionTimeS >= 0.0
             ? input.clock.executionTimeS
             : input.clock.timestampS;
}

std::array<Vec3, 2> initialBoundaryState(const std::vector<Vec3> &guide,
                                        const LocalPlanRequest &input) {
  Vec3 velocity = input.robot.kinematics.valid ? input.robot.kinematics.linearVelocity : Vec3{};
  // Projected ground-following owns the vertical profile.  Feeding body heave
  // from odometry into the initial derivative creates a near-vertical spline
  // at standstill and makes an otherwise clear route fail the slope check.
  velocity.z = 0.0;
  // SCAN initializes acceleration from odometry as zero. Replans that retain
  // an active spline obtain acceleration from that spline in previousSeed().
  Vec3 acceleration{};
  const Vec3 to_target = subtract(guide.back(), input.robot.pose.position);
  if (velocity.x * to_target.x + velocity.y * to_target.y < 0.0) {
    velocity = {};
    acceleration = {};
  }
  return {velocity, acceleration};
}

double component(const Vec3 &value, int axis) {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

void setComponent(Vec3 *value, int axis, double component_value) {
  if (axis == 0) {
    value->x = component_value;
  } else if (axis == 1) {
    value->y = component_value;
  } else {
    value->z = component_value;
  }
}

struct PolynomialSegment {
  std::array<Vec3, 6> coefficients{};  // Ascending power order.
  double duration{0.0};
};

class Polynomial {
 public:
  explicit Polynomial(std::vector<PolynomialSegment> segments)
      : segments_(std::move(segments)) {
    for (const auto &segment : segments_)
      duration_ += segment.duration;
  }

  [[nodiscard]] bool valid() const {
    return !segments_.empty() && duration_ > 0.0 && std::isfinite(duration_) &&
           std::all_of(segments_.begin(), segments_.end(), [](const auto &segment) {
             return segment.duration > 0.0 && std::isfinite(segment.duration) &&
                    std::all_of(segment.coefficients.begin(), segment.coefficients.end(), finite);
           });
  }

  [[nodiscard]] double duration() const { return duration_; }

  [[nodiscard]] Vec3 position(double time) const { return evaluate(time, 0); }
  [[nodiscard]] Vec3 velocity(double time) const { return evaluate(time, 1); }
  [[nodiscard]] Vec3 acceleration(double time) const { return evaluate(time, 2); }

 private:
  [[nodiscard]] Vec3 evaluate(double time, int derivative) const {
    if (!valid() || !std::isfinite(time))
      return {};
    double local = std::clamp(time, 0.0, duration_);
    const PolynomialSegment *segment = &segments_.back();
    for (const auto &candidate : segments_) {
      segment = &candidate;
      if (local <= candidate.duration + 1e-12)
        break;
      local -= candidate.duration;
    }
    local = std::clamp(local, 0.0, segment->duration);
    Vec3 result{};
    for (int power = derivative; power < 6; ++power) {
      double factor = 1.0;
      for (int order = 0; order < derivative; ++order)
        factor *= static_cast<double>(power - order);
      result = add(result, scale(segment->coefficients[static_cast<std::size_t>(power)],
                                 factor * std::pow(local, power - derivative)));
    }
    return result;
  }

  std::vector<PolynomialSegment> segments_;
  double duration_{0.0};
};

Polynomial quintic(const Vec3 &start, const Vec3 &start_velocity, const Vec3 &start_acceleration,
                    const Vec3 &target, const Vec3 &target_velocity,
                    const Vec3 &target_acceleration, double duration) {
  if (!(duration > 0.0) || !std::isfinite(duration))
    return Polynomial({});
  const double t2 = duration * duration;
  const double t3 = t2 * duration;
  const double t4 = t3 * duration;
  const double t5 = t4 * duration;
  PolynomialSegment segment;
  segment.duration = duration;
  segment.coefficients[0] = start;
  segment.coefficients[1] = start_velocity;
  segment.coefficients[2] = scale(start_acceleration, 0.5);
  const Vec3 position_residual =
      subtract(target, add(add(start, scale(start_velocity, duration)),
                           scale(start_acceleration, 0.5 * t2)));
  const Vec3 velocity_residual =
      subtract(target_velocity, add(start_velocity, scale(start_acceleration, duration)));
  const Vec3 acceleration_residual = subtract(target_acceleration, start_acceleration);
  segment.coefficients[3] =
      add(add(scale(position_residual, 10.0 / t3), scale(velocity_residual, -4.0 / t2)),
          scale(acceleration_residual, 0.5 / duration));
  segment.coefficients[4] =
      add(add(scale(position_residual, -15.0 / t4), scale(velocity_residual, 7.0 / t3)),
          scale(acceleration_residual, -1.0 / t2));
  segment.coefficients[5] =
      add(add(scale(position_residual, 6.0 / t5), scale(velocity_residual, -3.0 / t4)),
          scale(acceleration_residual, 0.5 / t3));
  return Polynomial({segment});
}

bool solveDense(std::vector<double> matrix, std::vector<double> rhs, int size,
                std::vector<double> *solution) {
  if (solution == nullptr || size <= 0 ||
      matrix.size() != static_cast<std::size_t>(size * size) ||
      rhs.size() != static_cast<std::size_t>(size)) {
    return false;
  }
  for (int column = 0; column < size; ++column) {
    int pivot = column;
    double pivot_value = std::abs(matrix[static_cast<std::size_t>(column * size + column)]);
    for (int row = column + 1; row < size; ++row) {
      const double candidate = std::abs(matrix[static_cast<std::size_t>(row * size + column)]);
      if (candidate > pivot_value) {
        pivot = row;
        pivot_value = candidate;
      }
    }
    if (pivot_value <= 1e-11 || !std::isfinite(pivot_value))
      return false;
    if (pivot != column) {
      for (int entry = column; entry < size; ++entry) {
        std::swap(matrix[static_cast<std::size_t>(column * size + entry)],
                  matrix[static_cast<std::size_t>(pivot * size + entry)]);
      }
      std::swap(rhs[static_cast<std::size_t>(column)], rhs[static_cast<std::size_t>(pivot)]);
    }
    const double diagonal = matrix[static_cast<std::size_t>(column * size + column)];
    for (int row = column + 1; row < size; ++row) {
      const double factor = matrix[static_cast<std::size_t>(row * size + column)] / diagonal;
      if (std::abs(factor) <= 1e-18)
        continue;
      matrix[static_cast<std::size_t>(row * size + column)] = 0.0;
      for (int entry = column + 1; entry < size; ++entry) {
        matrix[static_cast<std::size_t>(row * size + entry)] -=
            factor * matrix[static_cast<std::size_t>(column * size + entry)];
      }
      rhs[static_cast<std::size_t>(row)] -= factor * rhs[static_cast<std::size_t>(column)];
    }
  }
  solution->assign(static_cast<std::size_t>(size), 0.0);
  for (int row = size - 1; row >= 0; --row) {
    double value = rhs[static_cast<std::size_t>(row)];
    for (int column = row + 1; column < size; ++column) {
      value -= matrix[static_cast<std::size_t>(row * size + column)] *
               (*solution)[static_cast<std::size_t>(column)];
    }
    const double diagonal = matrix[static_cast<std::size_t>(row * size + row)];
    if (std::abs(diagonal) <= 1e-11)
      return false;
    (*solution)[static_cast<std::size_t>(row)] = value / diagonal;
  }
  return std::all_of(solution->begin(), solution->end(),
                     [](double value) { return std::isfinite(value); });
}

double derivativeCoefficient(int power, int derivative, double time) {
  if (power < derivative)
    return 0.0;
  double factor = 1.0;
  for (int order = 0; order < derivative; ++order)
    factor *= static_cast<double>(power - order);
  return factor * std::pow(time, power - derivative);
}

Polynomial minimumSnap(const std::vector<Vec3> &waypoints, const Vec3 &start_velocity,
                       const Vec3 &target_velocity, const Vec3 &start_acceleration,
                       const Vec3 &target_acceleration, const std::vector<double> &times) {
  const int segments = static_cast<int>(times.size());
  if (segments <= 0 || waypoints.size() != static_cast<std::size_t>(segments + 1) ||
      std::any_of(times.begin(), times.end(),
                  [](double value) { return !(value > 0.0) || !std::isfinite(value); })) {
    return Polynomial({});
  }
  if (segments == 1) {
    return quintic(waypoints.front(), start_velocity, start_acceleration, waypoints.back(),
                   target_velocity, target_acceleration, times.front());
  }

  const int variables = 6 * segments;
  const int constraints = 4 * segments + 2;
  const int system_size = variables + constraints;
  std::vector<double> system(static_cast<std::size_t>(system_size * system_size), 0.0);
  const auto at = [system_size](int row, int column) {
    return static_cast<std::size_t>(row * system_size + column);
  };
  for (int segment = 0; segment < segments; ++segment) {
    const double duration = times[static_cast<std::size_t>(segment)];
    for (int left = 3; left < 6; ++left) {
      for (int right = 3; right < 6; ++right) {
        const double value = derivativeCoefficient(left, 3, 1.0) *
                             derivativeCoefficient(right, 3, 1.0) *
                             std::pow(duration, left + right - 5) /
                             static_cast<double>(left + right - 5);
        system[at(6 * segment + left, 6 * segment + right)] = 2.0 * value;
      }
    }
  }

  struct Constraint {
    std::vector<std::pair<int, double>> terms;
    Vec3 value{};
  };
  std::vector<Constraint> rows;
  rows.reserve(static_cast<std::size_t>(constraints));
  const auto endpointTerms = [](int segment, int derivative, double time) {
    std::vector<std::pair<int, double>> terms;
    for (int power = derivative; power < 6; ++power) {
      terms.emplace_back(6 * segment + power,
                         derivativeCoefficient(power, derivative, time));
    }
    return terms;
  };
  for (int segment = 0; segment < segments; ++segment) {
    rows.push_back({endpointTerms(segment, 0, 0.0), waypoints[static_cast<std::size_t>(segment)]});
    rows.push_back({endpointTerms(segment, 0, times[static_cast<std::size_t>(segment)]),
                    waypoints[static_cast<std::size_t>(segment + 1)]});
  }
  rows.push_back({endpointTerms(0, 1, 0.0), start_velocity});
  rows.push_back({endpointTerms(0, 2, 0.0), start_acceleration});
  rows.push_back({endpointTerms(segments - 1, 1, times.back()), target_velocity});
  rows.push_back({endpointTerms(segments - 1, 2, times.back()), target_acceleration});
  for (int segment = 0; segment + 1 < segments; ++segment) {
    for (int derivative = 1; derivative <= 2; ++derivative) {
      auto terms = endpointTerms(segment, derivative, times[static_cast<std::size_t>(segment)]);
      for (const auto &[index, value] : endpointTerms(segment + 1, derivative, 0.0))
        terms.emplace_back(index, -value);
      rows.push_back({std::move(terms), {}});
    }
  }
  if (rows.size() != static_cast<std::size_t>(constraints))
    return Polynomial({});
  for (int row = 0; row < constraints; ++row) {
    for (const auto &[column, value] : rows[static_cast<std::size_t>(row)].terms) {
      system[at(column, variables + row)] = value;
      system[at(variables + row, column)] = value;
    }
  }

  std::array<std::vector<double>, 3> coefficients;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> rhs(static_cast<std::size_t>(system_size), 0.0);
    for (int row = 0; row < constraints; ++row) {
      rhs[static_cast<std::size_t>(variables + row)] =
          component(rows[static_cast<std::size_t>(row)].value, axis);
    }
    std::vector<double> solution;
    if (!solveDense(system, std::move(rhs), system_size, &solution))
      return Polynomial({});
    coefficients[static_cast<std::size_t>(axis)].assign(solution.begin(),
                                                        solution.begin() + variables);
  }

  std::vector<PolynomialSegment> output(static_cast<std::size_t>(segments));
  for (int segment = 0; segment < segments; ++segment) {
    output[static_cast<std::size_t>(segment)].duration = times[static_cast<std::size_t>(segment)];
    for (int power = 0; power < 6; ++power) {
      Vec3 coefficient;
      for (int axis = 0; axis < 3; ++axis) {
        setComponent(&coefficient, axis,
                     coefficients[static_cast<std::size_t>(axis)]
                                 [static_cast<std::size_t>(6 * segment + power)]);
      }
      output[static_cast<std::size_t>(segment)].coefficients[static_cast<std::size_t>(power)] =
          coefficient;
    }
  }
  return Polynomial(std::move(output));
}

void applyGuideHeight(std::vector<Vec3> *points,
                      const std::vector<Vec3> &guide) {
  if (points == nullptr || points->empty() || guide.size() < 2U)
    return;
  if (points->size() == 1U) {
    points->front().z = guide.front().z;
    return;
  }
  std::vector<double> sample_arc(points->size(), 0.0);
  for (std::size_t index = 1U; index < points->size(); ++index) {
    sample_arc[index] = sample_arc[index - 1U] +
                        distance3D((*points)[index - 1U], (*points)[index]);
  }
  std::vector<double> guide_arc(guide.size(), 0.0);
  for (std::size_t index = 1U; index < guide.size(); ++index)
    guide_arc[index] = guide_arc[index - 1U] + distance3D(guide[index - 1U], guide[index]);
  std::size_t guide_segment = 1U;
  for (std::size_t index = 0U; index < points->size(); ++index) {
    const double ratio = sample_arc.back() > 1e-6
                             ? sample_arc[index] / sample_arc.back()
                             : static_cast<double>(index) /
                                   static_cast<double>(points->size() - 1U);
    const double target = ratio * guide_arc.back();
    while (guide_segment + 1U < guide_arc.size() &&
           guide_arc[guide_segment] < target) {
      ++guide_segment;
    }
    const double span = guide_arc[guide_segment] - guide_arc[guide_segment - 1U];
    const double local = span > 1e-9
                             ? (target - guide_arc[guide_segment - 1U]) / span
                             : 0.0;
    (*points)[index].z = guide[guide_segment - 1U].z +
                         local * (guide[guide_segment].z - guide[guide_segment - 1U].z);
  }
  points->front().z = guide.front().z;
  points->back().z = guide.back().z;
}

std::vector<Vec3> resampleArc(const std::vector<Vec3> &points, double spacing) {
  if (points.size() < 2U || !(spacing > 0.0))
    return {};
  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t index = 1U; index < points.size(); ++index)
    arc[index] = arc[index - 1U] + distance3D(points[index - 1U], points[index]);
  if (arc.back() <= 1e-9)
    return {};
  std::vector<Vec3> samples;
  samples.reserve(static_cast<std::size_t>(std::ceil(arc.back() / spacing)) + 1U);
  std::size_t segment = 1U;
  for (double target = 0.0; target < arc.back(); target += spacing) {
    while (segment + 1U < arc.size() && arc[segment] < target)
      ++segment;
    const double length = arc[segment] - arc[segment - 1U];
    const double ratio = length > 1e-12 ? (target - arc[segment - 1U]) / length : 0.0;
    samples.push_back(add(points[segment - 1U],
                          scale(subtract(points[segment], points[segment - 1U]), ratio)));
  }
  if (samples.empty() || distance3D(samples.back(), points.back()) > 1e-8)
    samples.push_back(points.back());
  return samples;
}

double travelTime(double distance, double max_speed, double max_acceleration) {
  const double speed = std::max(0.05, max_speed);
  const double acceleration = std::max(0.05, max_acceleration);
  const double switch_distance = speed * speed / acceleration;
  const double time = switch_distance > distance
                          ? std::sqrt(std::max(0.0, distance) / acceleration)
                          : (distance - switch_distance) / speed + 2.0 * speed / acceleration;
  return std::max(0.10, time);
}

Vec3 targetVelocity(const std::vector<Vec3> &guide, const LocalPlanRequest &input,
                    const LocalPlannerParams &params) {
  const LocalRouteView *route = input.route();
  if (route == nullptr || guide.size() < 2U || route->reachesGoal ||
      distance3D(guide.back(), route->target()) >
          std::max(0.05, params.scan.voxelResolution)) {
    return {};
  }
  return scale(normalized(subtract(guide.back(), guide[guide.size() - 2U])),
               std::clamp(params.autonomySpeed, 0.05, std::max(0.05, params.maxSpeed)));
}

SeedResult samplePolynomial(const Polynomial &trajectory, const std::vector<Vec3> &guide,
                            const LocalPlanRequest &input, const LocalPlannerParams &params,
                            SeedMode mode, double initial_interval) {
  SeedResult result;
  result.mode = mode;
  if (!trajectory.valid() || guide.size() < 2U)
    return result;
  const double control_spacing = std::max(0.05, params.scan.controlPointSpacing);
  double interval = initial_interval * 1.5;
  for (int retry = 0; retry < 24; ++retry) {
    interval /= 1.5;
    std::vector<Vec3> samples;
    bool too_far = false;
    Vec3 previous = trajectory.position(0.0);
    for (double time = 0.0; time < trajectory.duration(); time += interval) {
      const Vec3 point = trajectory.position(time);
      if (!finite(point) || distance3D(previous, point) > 1.5 * control_spacing) {
        too_far = true;
        break;
      }
      samples.push_back(point);
      previous = point;
    }
    if (!too_far) {
      const Vec3 target = guide.back();
      if (distance3D(previous, target) > 1.5 * control_spacing) {
        too_far = true;
      } else if (samples.empty() || distance3D(samples.back(), target) > 1e-8) {
        samples.push_back(target);
      }
    }
    if (!too_far && samples.size() >= 7U) {
      applyGuideHeight(&samples, guide);
      result.samples = std::move(samples);
      result.interval = interval;
      result.boundary = {
          trajectory.velocity(0.0),
          targetVelocity(guide, input, params),
          trajectory.acceleration(0.0),
          trajectory.acceleration(trajectory.duration()),
      };
      return result;
    }
  }
  return result;
}

std::uint64_t mix(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

double retryNoise(std::uint64_t *state) {
  *state = mix(*state);
  return static_cast<double>(*state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5;
}

SeedResult previousSeed(const std::vector<Vec3> &guide, const LocalPlanRequest &input,
                        const LocalPlannerParams &params, const SeedHistory &history,
                        double interval) {
  SeedResult result;
  result.mode = SeedMode::Previous;
  if (history.trajectory == nullptr || !history.trajectory->valid() ||
      !std::isfinite(history.startedS)) {
    return result;
  }
  const UniformSpline &previous = *history.trajectory;
  const double elapsed = executionTime(input) - history.startedS;
  if (!std::isfinite(elapsed) || elapsed < 0.0 || elapsed > previous.duration() + 1e-6)
    return result;

  // Upstream replanning always anchors the regenerated point set at start_pt.
  // Using only the time-predicted point from the previous spline makes pose
  // error accumulate whenever the robot first rotates in place or executes
  // more slowly than the planned trajectory.
  std::vector<Vec3> joined{input.robot.pose.position};
  for (double time = elapsed; time < previous.duration(); time += interval) {
    const Vec3 point = previous.evaluate(time);
    if (distance3D(joined.back(), point) > 1e-8)
      joined.push_back(point);
  }
  if (joined.empty() || distance3D(joined.back(), previous.evaluate(previous.duration())) > 1e-8)
    joined.push_back(previous.evaluate(previous.duration()));
  if (joined.empty())
    return result;

  const Vec3 end_velocity = targetVelocity(guide, input, params);
  const Vec3 previous_end = joined.back();
  const UniformSpline previous_velocity = previous.derivative();
  const UniformSpline previous_acceleration = previous_velocity.derivative();
  const double join_time = 2.0 * distance3D(previous_end, guide.back()) /
                           std::max(0.05, params.maxSpeed);
  if (join_time > interval) {
    const Polynomial join = quintic(previous_end, previous_velocity.evaluate(previous.duration()),
                                    previous_acceleration.evaluate(previous.duration()), guide.back(),
                                    end_velocity, {}, join_time);
    for (double time = interval; time < join_time; time += interval)
      joined.push_back(join.position(time));
  }
  if (distance3D(joined.back(), guide.back()) > 1e-8)
    joined.push_back(guide.back());

  double spacing = std::max(0.05, params.scan.controlPointSpacing) * 1.5;
  for (int retry = 0; retry < 24; ++retry) {
    spacing /= 1.5;
    result.samples = resampleArc(joined, spacing);
    if (result.samples.size() >= 7U)
      break;
  }
  const double maximum_samples =
      std::max(1.0, params.scan.horizontalRange) /
      std::max(0.05, params.scan.controlPointSpacing) * 3.0;
  if (result.samples.size() < 7U ||
      static_cast<double>(result.samples.size()) > maximum_samples) {
    result.samples.clear();
    return result;
  }
  applyGuideHeight(&result.samples, guide);
  result.interval = interval;
  Vec3 start_velocity = previous_velocity.evaluate(elapsed);
  Vec3 start_acceleration = previous_acceleration.evaluate(elapsed);
  const Vec3 to_target = subtract(guide.back(), input.robot.pose.position);
  if (start_velocity.x * to_target.x + start_velocity.y * to_target.y < 0.0) {
    start_velocity = {};
    start_acceleration = {};
  }
  result.boundary = {
      start_velocity,
      end_velocity,
      start_acceleration,
      Vec3{},
  };
  return result;
}

}  // namespace

const char *seedModeName(SeedMode mode) noexcept {
  switch (mode) {
    case SeedMode::Previous:
      return "previous";
    case SeedMode::Guide:
      return "guide";
    case SeedMode::Polynomial:
      return "polynomial";
    case SeedMode::RandomPolynomial:
      return "random_min_snap";
  }
  return "polynomial";
}

SeedResult buildSeed(const std::vector<Vec3> &guide, const LocalPlanRequest &input,
                     const LocalPlannerParams &params, SeedMode mode,
                     const SeedHistory &history, int failures) {
  SeedResult invalid;
  invalid.mode = mode;
  if (guide.size() < 2U || !finite(guide.front()) || !finite(guide.back()))
    return invalid;
  const double max_speed =
      std::clamp(params.autonomySpeed, 0.05, std::max(0.05, params.maxSpeed));
  const double interval = std::max(
      0.03, 1.2 * std::max(0.05, params.scan.controlPointSpacing) / max_speed);
  if (mode == SeedMode::Previous)
    return previousSeed(guide, input, params, history, interval);

  std::vector<Vec3> anchored_guide = guide;
  anchored_guide.front() = input.robot.pose.position;
  const auto initial_state = initialBoundaryState(anchored_guide, input);
  const Vec3 start_velocity = initial_state[0];
  const Vec3 start_acceleration = initial_state[1];
  const Vec3 end_velocity = targetVelocity(anchored_guide, input, params);
  const std::vector<Vec3> waypoints =
      seedWaypoints(anchored_guide, params.scan.controlPointSpacing);
  double guide_length = 0.0;
  for (std::size_t index = 1U; index < waypoints.size(); ++index)
    guide_length += distance3D(waypoints[index - 1U], waypoints[index]);
  const double duration = travelTime(guide_length, max_speed,
                                     std::max(0.05, params.scan.maxAcceleration));
  Polynomial trajectory({});
  if (mode == SeedMode::RandomPolynomial) {
    const Vec3 delta = subtract(anchored_guide.front(), anchored_guide.back());
    Vec3 horizontal = normalized({delta.y, -delta.x, 0.0});
    if (norm(horizontal) <= 1e-9)
      horizontal = {0.0, 1.0, 0.0};
    const Vec3 vertical = normalized({-delta.x * delta.z, -delta.y * delta.z,
                                      delta.x * delta.x + delta.y * delta.y});
    std::uint64_t state = static_cast<std::uint64_t>(std::max(1, failures));
    state ^= static_cast<std::uint64_t>(std::llround(guide.back().x * 1000.0)) << 1U;
    state ^= static_cast<std::uint64_t>(std::llround(guide.back().y * 1000.0)) << 17U;
    const double gain = -0.978 / (static_cast<double>(std::max(0, failures)) + 0.989) + 0.989;
    const double distance = distance3D(anchored_guide.front(), anchored_guide.back());
    Vec3 middle = scale(add(anchored_guide.front(), anchored_guide.back()), 0.5);
    middle = add(middle, scale(horizontal, retryNoise(&state) * distance * 0.8 * gain));
    middle = add(middle, scale(vertical, retryNoise(&state) * distance * 0.4 * gain));
    trajectory = minimumSnap({anchored_guide.front(), middle, anchored_guide.back()},
                             start_velocity, end_velocity, start_acceleration, {},
                             {duration * 0.5, duration * 0.5});
  } else if (mode == SeedMode::Guide) {
    if (waypoints.size() > 2U && guide_length > 1e-6) {
      std::vector<double> times;
      times.reserve(waypoints.size() - 1U);
      for (std::size_t index = 1U; index < waypoints.size(); ++index) {
        times.push_back(duration *
                        distance3D(waypoints[index - 1U], waypoints[index]) /
                        guide_length);
      }
      trajectory = minimumSnap(waypoints, start_velocity, end_velocity,
                               start_acceleration, {}, times);
    } else {
      return invalid;
    }
  } else {
    trajectory = quintic(anchored_guide.front(), start_velocity, start_acceleration,
                         anchored_guide.back(), end_velocity, {}, duration);
  }
  return samplePolynomial(trajectory, anchored_guide, input, params, mode, interval);
}

}  // namespace nav_kernel::local::scan
