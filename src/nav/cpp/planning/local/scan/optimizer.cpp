#include "planning/local/scan/optimizer.hpp"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4100 4701 4702)
#endif
#include "planning/local/scan/vendor/lbfgs.hpp"
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace nav_kernel::local::scan {
namespace {

struct ReboundReference {
  Vec3 guide{};
  Vec3 base{};
  double directionX{0.0};
  double directionY{0.0};
  double clearance{0.0};
  bool active{false};
};

struct Objective {
  const Grid *grid{nullptr};
  std::vector<Vec3> seed;
  std::vector<Vec3> guide;
  std::vector<double> guideArc;
  std::vector<ReboundReference> references;
  std::vector<Vec3> controlsScratch;
  std::vector<Vec3> gradientScratch;
  std::size_t fixedControls{1U};
  double interval{0.1};
  double maxSpeed{1.0};
  double maxAcceleration{1.0};
  double smoothWeight{1.0};
  double collisionWeight{1.0};
  double feasibilityWeight{0.1};
  double guideWeight{8.0};
  int evaluations{0};
};

double norm2D(double x, double y) {
  return std::hypot(x, y);
}

Vec3 interpolate(const Vec3 &a, const Vec3 &b, double ratio) {
  return {
      a.x + ratio * (b.x - a.x),
      a.y + ratio * (b.y - a.y),
      a.z + ratio * (b.z - a.z),
  };
}

std::vector<double> horizontalArc(const std::vector<Vec3> &path) {
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1U] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return arc;
}

Vec3 pointAtFraction(const std::vector<Vec3> &path, const std::vector<double> &arc,
                     double fraction) {
  if (path.empty())
    return {};
  if (path.size() == 1U)
    return path.front();
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double total = arc.back();
  if (total <= 1e-8) {
    const double scaled = fraction * static_cast<double>(path.size() - 1U);
    const std::size_t index =
        std::min(static_cast<std::size_t>(std::floor(scaled)), path.size() - 2U);
    return interpolate(path[index], path[index + 1U], scaled - index);
  }
  const double target = fraction * total;
  const auto upper = std::lower_bound(arc.begin() + 1, arc.end(), target);
  if (upper == arc.end())
    return path.back();
  const std::size_t i = static_cast<std::size_t>(upper - arc.begin());
  const double segment = arc[i] - arc[i - 1U];
  return segment > 1e-9 ? interpolate(path[i - 1U], path[i], (target - arc[i - 1U]) / segment)
                        : path[i];
}

double controlYaw(const std::vector<Vec3> &controls, std::size_t index) {
  const std::size_t previous = index > 0U ? index - 1U : index;
  const std::size_t next = std::min(index + 1U, controls.size() - 1U);
  const double dx = controls[next].x - controls[previous].x;
  const double dy = controls[next].y - controls[previous].y;
  return std::hypot(dx, dy) > 1e-9 ? std::atan2(dy, dx) : 0.0;
}

bool pointFree(const Grid &grid, const Vec3 &point, double yaw) {
  const GridIndex index = grid.index(point);
  return grid.contains(index) && grid.free(index, yaw);
}

std::vector<ReboundReference> buildReferences(const Grid &grid, const std::vector<Vec3> &controls,
                                              const std::vector<Vec3> &guide,
                                              const std::vector<double> &guide_arc,
                                              const LocalPlannerParams &params) {
  std::vector<ReboundReference> references(controls.size());
  if (controls.size() < 2U || guide.size() < 2U)
    return references;

  const double sample_step = std::max(0.02, grid.resolution());
  const double clearance = std::max(grid.resolution(), params.scan.collisionDistance);
  for (std::size_t i = 0; i < controls.size(); ++i) {
    ReboundReference &reference = references[i];
    const double fraction = static_cast<double>(i) / static_cast<double>(controls.size() - 1U);
    reference.guide = pointAtFraction(guide, guide_arc, fraction);
    reference.base = controls[i];

    // Upstream initControlPoints intersects the A* guide with the plane whose
    // normal is the local control-point tangent. Use the closest valid
    // intersection instead of assigning anchors by path index.
    const std::size_t previous = i > 0U ? i - 1U : i;
    const std::size_t next = std::min(i + 1U, controls.size() - 1U);
    const double tangent_x = controls[next].x - controls[previous].x;
    const double tangent_y = controls[next].y - controls[previous].y;
    if (std::hypot(tangent_x, tangent_y) > 1e-9) {
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t segment = 0U; segment + 1U < guide.size(); ++segment) {
        const double first_value = (guide[segment].x - controls[i].x) * tangent_x +
                                   (guide[segment].y - controls[i].y) * tangent_y;
        const double second_value = (guide[segment + 1U].x - controls[i].x) * tangent_x +
                                    (guide[segment + 1U].y - controls[i].y) * tangent_y;
        if (first_value * second_value > 0.0 ||
            (std::abs(first_value) <= 1e-12 && std::abs(second_value) <= 1e-12)) {
          continue;
        }
        const double denominator = first_value - second_value;
        if (std::abs(denominator) <= 1e-12)
          continue;
        const double ratio = std::clamp(first_value / denominator, 0.0, 1.0);
        const Vec3 intersection = interpolate(guide[segment], guide[segment + 1U], ratio);
        const double distance =
            std::hypot(intersection.x - controls[i].x, intersection.y - controls[i].y);
        if (distance < best_distance) {
          best_distance = distance;
          reference.guide = intersection;
        }
      }
    }

    const double dx = reference.guide.x - controls[i].x;
    const double dy = reference.guide.y - controls[i].y;
    const double distance = norm2D(dx, dy);
    if (distance <= 0.25 * grid.resolution())
      continue;

    reference.directionX = dx / distance;
    reference.directionY = dy / distance;
    reference.clearance = clearance;
    const double yaw = controlYaw(controls, i);
    bool found_boundary = false;
    for (double travel = distance; travel >= 0.0; travel -= sample_step) {
      const Vec3 sample =
          interpolate(controls[i], reference.guide, std::clamp(travel / distance, 0.0, 1.0));
      const bool occupied = !pointFree(grid, sample, yaw);
      if (!occupied)
        continue;
      const double anchor_travel = std::min(distance, travel + sample_step);
      reference.base =
          interpolate(controls[i], reference.guide, std::clamp(anchor_travel / distance, 0.0, 1.0));
      found_boundary = true;
      break;
    }
    reference.active = found_boundary;
  }
  return references;
}

std::size_t variableBegin(const Objective &objective) {
  return objective.fixedControls;
}

std::size_t variableEnd(const Objective &objective) {
  return objective.seed.size() - objective.fixedControls;
}

std::size_t variableOffset(const Objective &objective, std::size_t control_index) {
  return (control_index - variableBegin(objective)) * 2U;
}

bool updateControls(Objective &objective, const double *variables, int count) {
  const std::size_t begin = variableBegin(objective);
  const std::size_t end = variableEnd(objective);
  const std::size_t expected = end > begin ? (end - begin) * 2U : 0U;
  if (variables == nullptr || count < 0 || static_cast<std::size_t>(count) != expected) {
    return false;
  }
  objective.controlsScratch = objective.seed;
  for (std::size_t i = begin; i < end; ++i) {
    const std::size_t offset = variableOffset(objective, i);
    objective.controlsScratch[i].x = variables[offset];
    objective.controlsScratch[i].y = variables[offset + 1U];
  }
  return true;
}

double signedExcess(double value, double limit) {
  if (value > limit)
    return value - limit;
  if (value < -limit)
    return value + limit;
  return 0.0;
}

double evaluateObjective(void *instance, const double *variables, double *gradient, int count) {
  auto *objective = static_cast<Objective *>(instance);
  if (objective == nullptr || gradient == nullptr || count <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  ++objective->evaluations;
  std::memset(gradient, 0, static_cast<std::size_t>(count) * sizeof(double));
  if (!updateControls(*objective, variables, count)) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &controls = objective->controlsScratch;
  auto &point_gradient = objective->gradientScratch;
  std::fill(point_gradient.begin(), point_gradient.end(), Vec3{});
  double cost = 0.0;

  // Upstream SCAN smooths the third finite difference (jerk), not curvature.
  for (std::size_t i = 0U; i + 3U < controls.size(); ++i) {
    const double jerk_x =
        controls[i + 3U].x - 3.0 * controls[i + 2U].x + 3.0 * controls[i + 1U].x - controls[i].x;
    const double jerk_y =
        controls[i + 3U].y - 3.0 * controls[i + 2U].y + 3.0 * controls[i + 1U].y - controls[i].y;
    cost += objective->smoothWeight * (jerk_x * jerk_x + jerk_y * jerk_y);
    const double gradient_x = 2.0 * objective->smoothWeight * jerk_x;
    const double gradient_y = 2.0 * objective->smoothWeight * jerk_y;
    point_gradient[i].x -= gradient_x;
    point_gradient[i].y -= gradient_y;
    point_gradient[i + 1U].x += 3.0 * gradient_x;
    point_gradient[i + 1U].y += 3.0 * gradient_y;
    point_gradient[i + 2U].x -= 3.0 * gradient_x;
    point_gradient[i + 2U].y -= 3.0 * gradient_y;
    point_gradient[i + 3U].x += gradient_x;
    point_gradient[i + 3U].y += gradient_y;
  }

  for (std::size_t i = 1U; i + 1U < controls.size(); ++i) {
    const ReboundReference &reference = objective->references[i];
    const double guide_dx = controls[i].x - reference.guide.x;
    const double guide_dy = controls[i].y - reference.guide.y;
    cost += objective->guideWeight * (guide_dx * guide_dx + guide_dy * guide_dy);
    point_gradient[i].x += 2.0 * objective->guideWeight * guide_dx;
    point_gradient[i].y += 2.0 * objective->guideWeight * guide_dy;

    if (reference.active) {
      const double signed_distance = (controls[i].x - reference.base.x) * reference.directionX +
                                     (controls[i].y - reference.base.y) * reference.directionY;
      const double violation = reference.clearance - signed_distance;
      if (violation > 0.0) {
        double penalty = 0.0;
        double derivative = 0.0;
        if (violation < reference.clearance) {
          penalty = violation * violation * violation;
          derivative = 3.0 * violation * violation;
        } else {
          const double clearance = reference.clearance;
          const double a = 3.0 * clearance;
          const double b = -3.0 * clearance * clearance;
          const double c = clearance * clearance * clearance;
          penalty = a * violation * violation + b * violation + c;
          derivative = 2.0 * a * violation + b;
        }
        cost += objective->collisionWeight * penalty;
        const double scale = -objective->collisionWeight * derivative;
        point_gradient[i].x += scale * reference.directionX;
        point_gradient[i].y += scale * reference.directionY;
      }
    }
  }

  const double inverse_interval = 1.0 / objective->interval;
  const double inverse_interval_squared = inverse_interval * inverse_interval;
  for (std::size_t i = 0U; i + 1U < controls.size(); ++i) {
    const double velocity_x = (controls[i + 1U].x - controls[i].x) * inverse_interval;
    const double velocity_y = (controls[i + 1U].y - controls[i].y) * inverse_interval;
    const double excess_x = signedExcess(velocity_x, objective->maxSpeed);
    const double excess_y = signedExcess(velocity_y, objective->maxSpeed);
    cost += objective->feasibilityWeight * inverse_interval_squared *
            (excess_x * excess_x + excess_y * excess_y);
    const double gradient_x =
        2.0 * objective->feasibilityWeight * excess_x * inverse_interval * inverse_interval_squared;
    const double gradient_y =
        2.0 * objective->feasibilityWeight * excess_y * inverse_interval * inverse_interval_squared;
    point_gradient[i].x -= gradient_x;
    point_gradient[i].y -= gradient_y;
    point_gradient[i + 1U].x += gradient_x;
    point_gradient[i + 1U].y += gradient_y;
  }

  for (std::size_t i = 1U; i + 1U < controls.size(); ++i) {
    const double ddx = controls[i - 1U].x - 2.0 * controls[i].x + controls[i + 1U].x;
    const double ddy = controls[i - 1U].y - 2.0 * controls[i].y + controls[i + 1U].y;
    const double acceleration_x = ddx * inverse_interval_squared;
    const double acceleration_y = ddy * inverse_interval_squared;
    const double excess_x = signedExcess(acceleration_x, objective->maxAcceleration);
    const double excess_y = signedExcess(acceleration_y, objective->maxAcceleration);
    cost += objective->feasibilityWeight * (excess_x * excess_x + excess_y * excess_y);
    const double gradient_x =
        2.0 * objective->feasibilityWeight * excess_x * inverse_interval_squared;
    const double gradient_y =
        2.0 * objective->feasibilityWeight * excess_y * inverse_interval_squared;
    point_gradient[i - 1U].x += gradient_x;
    point_gradient[i - 1U].y += gradient_y;
    point_gradient[i].x -= 2.0 * gradient_x;
    point_gradient[i].y -= 2.0 * gradient_y;
    point_gradient[i + 1U].x += gradient_x;
    point_gradient[i + 1U].y += gradient_y;
  }

  for (std::size_t i = variableBegin(*objective); i < variableEnd(*objective); ++i) {
    const std::size_t offset = variableOffset(*objective, i);
    gradient[offset] = point_gradient[i].x;
    gradient[offset + 1U] = point_gradient[i].y;
  }
  return cost;
}

bool finiteControls(const std::vector<Vec3> &controls) {
  return std::all_of(controls.begin(), controls.end(), [](const Vec3 &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  });
}

}  // namespace

ReboundOptimizationResult optimizeRebound(const Grid &grid, const std::vector<Vec3> &seed_controls,
                                          const std::vector<Vec3> &projected_guide, double interval,
                                          const LocalPlannerParams &params) {
  ReboundOptimizationResult result;
  result.controls = seed_controls;
  if (seed_controls.size() < 4U || projected_guide.size() < 2U || !finiteControls(seed_controls) ||
      !finiteControls(projected_guide)) {
    return result;
  }

  Objective objective;
  objective.grid = &grid;
  objective.seed = seed_controls;
  objective.guide = projected_guide;
  objective.guideArc = horizontalArc(projected_guide);
  objective.controlsScratch = seed_controls;
  objective.gradientScratch.resize(seed_controls.size());
  objective.fixedControls = seed_controls.size() >= 7U ? 3U : 1U;
  objective.references =
      buildReferences(grid, seed_controls, projected_guide, objective.guideArc, params);
  objective.smoothWeight = std::max(0.0, params.scan.smoothWeight);
  objective.collisionWeight = std::max(0.0, params.scan.collisionWeight);
  objective.feasibilityWeight = std::max(0.0, params.scan.feasibilityWeight);
  objective.guideWeight = std::max(1.0, 4.0 * params.scan.guideWeight);
  objective.maxSpeed = std::max(0.05, params.maxSpeed);
  objective.maxAcceleration = std::max(0.05, params.scan.maxAcceleration);
  objective.interval = std::max(0.03, interval);

  const std::size_t begin = variableBegin(objective);
  const std::size_t end = variableEnd(objective);
  const std::size_t variable_count = (end - begin) * 2U;
  std::vector<double> variables(variable_count, 0.0);
  for (std::size_t i = begin; i < end; ++i) {
    const std::size_t offset = variableOffset(objective, i);
    variables[offset] = seed_controls[i].x;
    variables[offset + 1U] = seed_controls[i].y;
  }
  std::vector<double> initial_gradient(variable_count, 0.0);
  result.initialCost = evaluateObjective(&objective, variables.data(), initial_gradient.data(),
                                         static_cast<int>(variable_count));
  result.finalCost = result.initialCost;
  result.attempted = true;

  lbfgs::lbfgs_parameter_t options;
  lbfgs::lbfgs_load_default_parameters(&options);
  options.mem_size = 16;
  options.g_epsilon = 0.01;
  options.max_iterations = std::clamp(std::max(16, params.scan.smoothingIterations), 16, 200);
  options.max_linesearch = 16;
  double final_cost = result.initialCost;
  const int status =
      lbfgs::lbfgs_optimize(static_cast<int>(variable_count), variables.data(), &final_cost,
                            evaluateObjective, nullptr, nullptr, &objective, &options);
  result.status = status;
  result.evaluations = objective.evaluations;
  result.finalCost = final_cost;

  const bool controls_updated =
      updateControls(objective, variables.data(), static_cast<int>(variable_count));
  const bool optimizer_finished = status >= 0 || status == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                                  status == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
                                  status == lbfgs::LBFGSERR_ROUNDING_ERROR;
  if (optimizer_finished && controls_updated && finiteControls(objective.controlsScratch) &&
      std::isfinite(final_cost) && final_cost <= result.initialCost + 1e-8) {
    result.controls = objective.controlsScratch;
    result.improved = final_cost + 1e-8 < result.initialCost;
  }
  return result;
}

}  // namespace nav_kernel::local::scan
