// Adapted from SCAN-Planner's rebound/refine B-spline objectives at upstream
// commit 348e8a5. Modified for LingTu's ROS/Eigen-free Vec3 representation.
// SPDX-License-Identifier: Apache-2.0
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
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include "planning/local/scan/anchors.hpp"

namespace nav_kernel::local::scan {
namespace {

enum class ObjectiveMode { Rebound, Refine };

struct Objective {
  ObjectiveMode mode{ObjectiveMode::Rebound};
  std::vector<Vec3> seed;
  std::vector<Vec3> reference;
  AnchorSet anchors;
  std::vector<Vec3> controls;
  std::vector<Vec3> gradients;
  std::size_t fixedControls{3U};
  double interval{0.1};
  double maxSpeed{1.0};
  double maxAcceleration{1.0};
  double smoothWeight{1.0};
  double collisionWeight{1.0};
  double feasibilityWeight{0.1};
  double fitnessWeight{1.0};
  int evaluations{0};
  const Grid *grid{nullptr};
  bool reboundRequested{false};
  LocalPlanCancel cancel;
};

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(const Vec3 &value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double length(const Vec3 &value) {
  return std::sqrt(dot(value, value));
}

Vec3 interpolate(const Vec3 &a, const Vec3 &b, double ratio) {
  return add(a, scale(subtract(b, a), ratio));
}

bool finite(const std::vector<Vec3> &points) {
  return std::all_of(points.begin(), points.end(), [](const Vec3 &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
  });
}

void appendAnchors(AnchorSet *target, AnchorSet source) {
  if (target == nullptr)
    return;
  if (target->controls.size() < source.controls.size())
    target->controls.resize(source.controls.size());
  for (std::size_t index = 0U; index < source.controls.size(); ++index) {
    auto &destination = target->controls[index];
    auto &incoming = source.controls[index];
    destination.insert(destination.end(),
                       std::make_move_iterator(incoming.begin()),
                       std::make_move_iterator(incoming.end()));
  }
  target->paths.insert(target->paths.end(),
                       std::make_move_iterator(source.paths.begin()),
                       std::make_move_iterator(source.paths.end()));
  target->collisionSegments += source.collisionSegments;
  target->searches += source.searches;
  target->complete = target->complete && source.complete;
}

std::size_t variableBegin(const Objective &objective) {
  return objective.fixedControls;
}

std::size_t variableEnd(const Objective &objective) {
  return objective.seed.size() - objective.fixedControls;
}

std::size_t variableOffset(const Objective &objective, std::size_t index) {
  return (index - variableBegin(objective)) * 2U;
}

bool updateControls(Objective &objective, const double *variables, int count) {
  const std::size_t begin = variableBegin(objective);
  const std::size_t end = variableEnd(objective);
  const std::size_t expected = end > begin ? (end - begin) * 2U : 0U;
  if (variables == nullptr || count < 0 || static_cast<std::size_t>(count) != expected)
    return false;
  objective.controls = objective.seed;
  for (std::size_t i = begin; i < end; ++i) {
    const std::size_t offset = variableOffset(objective, i);
    objective.controls[i].x = variables[offset];
    objective.controls[i].y = variables[offset + 1U];
  }
  return true;
}

void addSmoothness(Objective &objective, double *cost) {
  for (std::size_t i = 0U; i + 3U < objective.controls.size(); ++i) {
    const Vec3 jerk = add(subtract(objective.controls[i + 3U],
                                   scale(objective.controls[i + 2U], 3.0)),
                          subtract(scale(objective.controls[i + 1U], 3.0),
                                   objective.controls[i]));
    *cost += objective.smoothWeight * dot(jerk, jerk);
    const Vec3 derivative = scale(jerk, 2.0 * objective.smoothWeight);
    objective.gradients[i] = subtract(objective.gradients[i], derivative);
    objective.gradients[i + 1U] =
        add(objective.gradients[i + 1U], scale(derivative, 3.0));
    objective.gradients[i + 2U] =
        subtract(objective.gradients[i + 2U], scale(derivative, 3.0));
    objective.gradients[i + 3U] = add(objective.gradients[i + 3U], derivative);
  }
}

double controlYaw(const std::vector<Vec3> &controls, std::size_t index) {
  if (controls.size() < 2U)
    return 0.0;
  const std::size_t previous = index > 0U ? index - 1U : index;
  const std::size_t next = std::min(controls.size() - 1U, index + 1U);
  return std::atan2(controls[next].y - controls[previous].y,
                    controls[next].x - controls[previous].x);
}

bool needsRebound(const Objective &objective, double smoothness_cost) {
  if (objective.mode != ObjectiveMode::Rebound || objective.grid == nullptr ||
      objective.evaluations <= 3) {
    return false;
  }
  const std::size_t variable_count =
      variableEnd(objective) > variableBegin(objective)
          ? variableEnd(objective) - variableBegin(objective)
          : 1U;
  if (smoothness_cost / static_cast<double>(variable_count) >= 0.1)
    return false;

  const std::size_t end = variableEnd(objective);
  const std::size_t check_end =
      end > variableBegin(objective)
          ? end - (end - variableBegin(objective)) / 3U
          : end;
  const std::size_t begin = variableBegin(objective);
  for (std::size_t index = begin;
       index <= check_end && index < objective.controls.size(); ++index) {
    if (objective.grid->obstacleFree(objective.controls[index],
                                     controlYaw(objective.controls, index))) {
      continue;
    }
    bool covered_by_existing_anchor = false;
    if (index < objective.anchors.controls.size()) {
      for (const Anchor &anchor : objective.anchors.controls[index]) {
        const double signed_distance =
            dot(subtract(objective.controls[index], anchor.base), anchor.direction);
        if (signed_distance < objective.grid->resolution()) {
          covered_by_existing_anchor = true;
          break;
        }
      }
    }
    if (!covered_by_existing_anchor)
      return true;
  }
  return false;
}

void addRebound(Objective &objective, double *cost) {
  for (std::size_t i = variableBegin(objective); i < variableEnd(objective); ++i) {
    if (i >= objective.anchors.controls.size())
      continue;
    for (const Anchor &anchor : objective.anchors.controls[i]) {
      const double signed_distance =
          dot(subtract(objective.controls[i], anchor.base), anchor.direction);
      const double violation = anchor.clearance - signed_distance;
      if (violation < 0.0)
        continue;
      const double demarcation = anchor.clearance;
      double penalty = 0.0;
      double derivative = 0.0;
      if (violation < demarcation) {
        penalty = violation * violation * violation;
        derivative = 3.0 * violation * violation;
      } else {
        const double a = 3.0 * demarcation;
        const double b = -3.0 * demarcation * demarcation;
        const double c = demarcation * demarcation * demarcation;
        penalty = a * violation * violation + b * violation + c;
        derivative = 2.0 * a * violation + b;
      }
      *cost += objective.collisionWeight * penalty;
      objective.gradients[i] =
          subtract(objective.gradients[i],
                   scale(anchor.direction,
                         objective.collisionWeight * derivative));
    }
  }
}

void addFitness(Objective &objective, double *cost) {
  const std::size_t end = objective.controls.size() - objective.fixedControls;
  if (objective.reference.size() + 2U < objective.controls.size())
    return;
  for (std::size_t i = objective.fixedControls - 1U; i < end + 1U; ++i) {
    const Vec3 curve = scale(add(add(objective.controls[i - 1U],
                                     scale(objective.controls[i], 4.0)),
                                 objective.controls[i + 1U]),
                             1.0 / 6.0);
    const Vec3 error = subtract(curve, objective.reference[i - 1U]);
    const Vec3 tangent_delta =
        subtract(objective.reference[i], objective.reference[i - 2U]);
    const double tangent_length = length(tangent_delta);
    if (tangent_length <= 1e-9)
      continue;
    const Vec3 tangent = scale(tangent_delta, 1.0 / tangent_length);
    const double parallel = dot(error, tangent);
    const double fitness = dot(error, error) - parallel * parallel +
                           parallel * parallel / 25.0;
    *cost += objective.fitnessWeight * fitness;
    const Vec3 derivative = scale(
        add(error, scale(tangent, parallel * (1.0 / 25.0 - 1.0))),
        2.0 * objective.fitnessWeight);
    objective.gradients[i - 1U] =
        add(objective.gradients[i - 1U], scale(derivative, 1.0 / 6.0));
    objective.gradients[i] =
        add(objective.gradients[i], scale(derivative, 4.0 / 6.0));
    objective.gradients[i + 1U] =
        add(objective.gradients[i + 1U], scale(derivative, 1.0 / 6.0));
  }
}

double signedExcess(double value, double limit) {
  if (value > limit)
    return value - limit;
  if (value < -limit)
    return value + limit;
  return 0.0;
}

void addFeasibility(Objective &objective, double *cost) {
  const double inverse_interval = 1.0 / objective.interval;
  const double inverse_interval_squared = inverse_interval * inverse_interval;
  for (std::size_t i = 0U; i + 1U < objective.controls.size(); ++i) {
    const Vec3 velocity = scale(subtract(objective.controls[i + 1U],
                                         objective.controls[i]),
                                inverse_interval);
    for (int axis = 0; axis < 2; ++axis) {
      const double value = axis == 0 ? velocity.x : velocity.y;
      const double excess = signedExcess(value, objective.maxSpeed);
      *cost += objective.feasibilityWeight * inverse_interval_squared *
               excess * excess;
      const double derivative = 2.0 * objective.feasibilityWeight * excess *
                                inverse_interval * inverse_interval_squared;
      double &before = axis == 0 ? objective.gradients[i].x
                                 : objective.gradients[i].y;
      double &after = axis == 0 ? objective.gradients[i + 1U].x
                                : objective.gradients[i + 1U].y;
      before -= derivative;
      after += derivative;
    }
  }
  for (std::size_t i = 1U; i + 1U < objective.controls.size(); ++i) {
    const Vec3 acceleration =
        scale(add(subtract(objective.controls[i - 1U],
                           scale(objective.controls[i], 2.0)),
                  objective.controls[i + 1U]),
              inverse_interval_squared);
    for (int axis = 0; axis < 2; ++axis) {
      const double value = axis == 0 ? acceleration.x : acceleration.y;
      const double excess = signedExcess(value, objective.maxAcceleration);
      *cost += objective.feasibilityWeight * excess * excess;
      const double derivative = 2.0 * objective.feasibilityWeight * excess *
                                inverse_interval_squared;
      double &before = axis == 0 ? objective.gradients[i - 1U].x
                                 : objective.gradients[i - 1U].y;
      double &current = axis == 0 ? objective.gradients[i].x
                                  : objective.gradients[i].y;
      double &after = axis == 0 ? objective.gradients[i + 1U].x
                                : objective.gradients[i + 1U].y;
      before += derivative;
      current -= 2.0 * derivative;
      after += derivative;
    }
  }
}

double evaluate(void *instance, const double *variables, double *gradient,
                int count) {
  auto *objective = static_cast<Objective *>(instance);
  if (objective == nullptr || gradient == nullptr || count <= 0 ||
      !updateControls(*objective, variables, count)) {
    return std::numeric_limits<double>::infinity();
  }
  ++objective->evaluations;
  std::memset(gradient, 0, static_cast<std::size_t>(count) * sizeof(double));
  std::fill(objective->gradients.begin(), objective->gradients.end(), Vec3{});
  double cost = 0.0;
  double smoothness_cost = 0.0;
  addSmoothness(*objective, &smoothness_cost);
  cost += smoothness_cost;
  if (needsRebound(*objective, smoothness_cost))
    objective->reboundRequested = true;
  if (objective->mode == ObjectiveMode::Rebound)
    addRebound(*objective, &cost);
  else
    addFitness(*objective, &cost);
  addFeasibility(*objective, &cost);

  for (std::size_t i = variableBegin(*objective); i < variableEnd(*objective); ++i) {
    const std::size_t offset = variableOffset(*objective, i);
    gradient[offset] = objective->gradients[i].x;
    gradient[offset + 1U] = objective->gradients[i].y;
  }
  return cost;
}

int cancelOptimization(void *instance, const double *, const double *, double,
                       double, double, double, int, int, int) {
  const auto *objective = static_cast<const Objective *>(instance);
  if (objective == nullptr)
    return 0;
  return objective->reboundRequested || (objective->cancel && objective->cancel()) ? 1 : 0;
}

OptimizationResult optimize(Objective objective, int iterations) {
  OptimizationResult result;
  result.controls = objective.seed;
  result.anchorsComplete = objective.anchors.complete;
  result.collisionSegments = objective.anchors.collisionSegments;
  result.anchorSearches = objective.anchors.searches;
  const std::size_t begin = variableBegin(objective);
  const std::size_t end = variableEnd(objective);
  if (end <= begin || !finite(objective.seed) || (objective.cancel && objective.cancel()))
    return result;
  const std::size_t count = (end - begin) * 2U;
  std::vector<double> variables(count, 0.0);
  for (std::size_t i = begin; i < end; ++i) {
    const std::size_t offset = variableOffset(objective, i);
    variables[offset] = objective.seed[i].x;
    variables[offset + 1U] = objective.seed[i].y;
  }
  std::vector<double> initial_gradient(count, 0.0);
  result.initialCost = evaluate(&objective, variables.data(),
                                initial_gradient.data(), static_cast<int>(count));
  result.finalCost = result.initialCost;
  result.attempted = true;

  lbfgs::lbfgs_parameter_t options;
  lbfgs::lbfgs_load_default_parameters(&options);
  options.mem_size = 16;
  options.g_epsilon = objective.mode == ObjectiveMode::Refine ? 0.001 : 0.01;
  options.max_iterations = std::clamp(iterations, 16, 200);
  options.max_linesearch = 16;
  double final_cost = result.initialCost;
  result.status = lbfgs::lbfgs_optimize(
      static_cast<int>(count), variables.data(), &final_cost, evaluate, nullptr,
      cancelOptimization, &objective, &options);
  result.evaluations = objective.evaluations;
  result.finalCost = final_cost;
  result.reboundRequested = objective.reboundRequested;

  const bool updated =
      updateControls(objective, variables.data(), static_cast<int>(count));
  const bool finished = result.status >= 0 ||
                        result.status == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                        result.status == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
                        result.status == lbfgs::LBFGSERR_ROUNDING_ERROR ||
                        (result.status == lbfgs::LBFGSERR_CANCELED &&
                         result.reboundRequested);
  if (finished && updated && finite(objective.controls) &&
      std::isfinite(final_cost) &&
      (result.reboundRequested || final_cost <= result.initialCost + 1e-8)) {
    result.controls = std::move(objective.controls);
    result.improved = final_cost + 1e-8 < result.initialCost;
  }
  return result;
}

Objective baseObjective(const std::vector<Vec3> &controls, double interval,
                        const LocalPlannerParams &params,
                        const LocalPlanCancel &cancel) {
  Objective objective;
  objective.seed = controls;
  objective.controls = controls;
  objective.gradients.resize(controls.size());
  objective.fixedControls = 3U;
  objective.interval = std::max(0.03, interval);
  objective.maxSpeed = std::max(0.05, params.maxSpeed);
  objective.maxAcceleration = std::max(0.05, params.scan.maxAcceleration);
  objective.smoothWeight = std::max(0.0, params.scan.smoothWeight);
  objective.collisionWeight = std::max(0.0, params.scan.collisionWeight);
  objective.feasibilityWeight = std::max(0.0, params.scan.feasibilityWeight);
  objective.fitnessWeight = std::max(0.0, params.scan.fitnessWeight);
  objective.cancel = cancel;
  return objective;
}

}  // namespace

ReboundOptimizationResult optimizeRebound(
    const Grid &grid, const std::vector<Vec3> &seed_controls,
    double interval, const LocalPlannerParams &params,
    ReboundState *state, const LocalPlanCancel &cancel) {
  ReboundOptimizationResult result;
  result.controls = seed_controls;
  ReboundState local_state;
  ReboundState &rebound = state != nullptr ? *state : local_state;
  if (rebound.anchors.controls.size() != seed_controls.size()) {
    rebound.anchors = {};
    rebound.anchors.controls.resize(seed_controls.size());
  }
  constexpr int kMaxDynamicRebounds = 20;
  for (int pass = 0; pass <= kMaxDynamicRebounds; ++pass) {
    if (cancel && cancel())
      return result;
    Objective objective = baseObjective(result.controls, interval, params, cancel);
    objective.mode = ObjectiveMode::Rebound;
    objective.grid = &grid;
    const bool had_anchors = !rebound.anchors.paths.empty();
    AnchorSet fresh = buildAnchors(grid, result.controls, params, cancel);
    result.collisionSegments += fresh.collisionSegments;
    result.anchorSearches += fresh.searches;
    if (!fresh.complete) {
      // The official post-collision restart retains earlier directions even
      // when the newly optimized controls no longer provide a clean A* entry.
      // Dynamic rebounds still require a fresh path because they represent a
      // previously unseen collision.
      if (pass != 0 || !had_anchors) {
        result.anchorsComplete = false;
        result.failureReason = std::move(fresh.failureReason);
        return result;
      }
    } else {
      appendAnchors(&rebound.anchors, std::move(fresh));
    }
    objective.anchors = rebound.anchors;

    OptimizationResult optimized =
        optimize(std::move(objective), params.scan.smoothingIterations);
    if (!result.attempted)
      result.initialCost = optimized.initialCost;
    result.attempted = result.attempted || optimized.attempted;
    result.improved = result.improved || optimized.improved;
    result.status = optimized.status;
    result.evaluations += optimized.evaluations;
    result.finalCost = optimized.finalCost;
    result.controls = std::move(optimized.controls);
    result.reboundRequested = optimized.reboundRequested;
    if (!optimized.reboundRequested)
      return result;
    ++result.reboundPasses;
  }
  return result;
}

RefineOptimizationResult optimizeRefine(
    const std::vector<Vec3> &seed_controls,
    const std::vector<Vec3> &reference_points, double interval,
    const LocalPlannerParams &params, const LocalPlanCancel &cancel) {
  Objective objective = baseObjective(seed_controls, interval, params, cancel);
  objective.mode = ObjectiveMode::Refine;
  objective.reference = reference_points;
  return optimize(std::move(objective), params.scan.smoothingIterations);
}

}  // namespace nav_kernel::local::scan
