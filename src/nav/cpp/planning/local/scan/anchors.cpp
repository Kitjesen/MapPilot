// Adapted from SCAN-Planner BsplineOptimizer::initControlPoints at upstream
// commit 348e8a5. Modified for LingTu's projected A* and yaw-aware collision map.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/anchors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "planning/local/scan/search.hpp"

namespace nav_kernel::local::scan {
namespace {

constexpr int kOrder = 3;
constexpr int kStableSamples = 2;

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

double yaw(const std::vector<Vec3> &controls, std::size_t index) {
  const std::size_t before = index > 0U ? index - 1U : index;
  const std::size_t after = std::min(index + 1U, controls.size() - 1U);
  const Vec3 tangent = subtract(controls[after], controls[before]);
  return std::hypot(tangent.x, tangent.y) > 1e-9
             ? std::atan2(tangent.y, tangent.x)
             : 0.0;
}

bool occupied(const Grid &grid, const Vec3 &point, double heading) {
  return !grid.free(point, heading);
}

std::vector<std::pair<int, int>> collisionSegments(
    const Grid &grid, const std::vector<Vec3> &controls) {
  std::vector<std::pair<int, int>> segments;
  if (controls.size() < 2U * kOrder + 1U)
    return segments;

  double total_distance = 0.0;
  for (std::size_t i = 1U; i < controls.size(); ++i)
    total_distance += length(subtract(controls[i], controls[i - 1U]));
  const double average_distance =
      std::max(grid.resolution(), total_distance / static_cast<double>(controls.size() - 1U));
  const double ratio_step = std::clamp(grid.resolution() / average_distance / 2.0, 0.02, 1.0);
  const int count = static_cast<int>(controls.size());
  const int end = count - kOrder - (count - 2 * kOrder) / 3;

  int in = -1;
  int out = -1;
  int same_state = kStableSamples + 1;
  bool last_occupied = false;
  bool have_start = false;
  bool possible_end = false;
  for (int i = kOrder; i <= end; ++i) {
    const double heading = yaw(controls, static_cast<std::size_t>(i));
    for (double ratio = 0.0; ratio <= 1.0 + 1e-9; ratio += ratio_step) {
      const bool is_occupied =
          occupied(grid, interpolate(controls[static_cast<std::size_t>(i - 1)],
                                     controls[static_cast<std::size_t>(i)],
                                     std::min(1.0, ratio)),
                   heading);
      if (is_occupied && !last_occupied) {
        if (same_state > kStableSamples || i == kOrder) {
          in = i - 1;
          have_start = true;
        }
        same_state = 0;
        possible_end = false;
      } else if (!is_occupied && last_occupied) {
        out = i;
        possible_end = true;
        same_state = 0;
      } else {
        ++same_state;
      }
      if (have_start && possible_end && same_state > kStableSamples) {
        if (in >= 0 && out > in && out < count)
          segments.emplace_back(in, out);
        have_start = false;
        possible_end = false;
      }
      last_occupied = is_occupied;
    }
  }

  if (have_start) {
    for (int i = std::max(in + 1, end + 1); i < count; ++i) {
      if (!occupied(grid, controls[static_cast<std::size_t>(i)],
                    yaw(controls, static_cast<std::size_t>(i)))) {
        out = i;
        break;
      }
    }
    if (in >= 0 && out > in && out < count)
      segments.emplace_back(in, out);
  }
  return segments;
}

bool planeIntersection(const Vec3 &control, const Vec3 &tangent,
                       const std::vector<Vec3> &path, Vec3 *intersection) {
  if (intersection == nullptr || path.size() < 2U || length(tangent) <= 1e-9)
    return false;

  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
    const double first = dot(subtract(path[i], control), tangent);
    const double second = dot(subtract(path[i + 1U], control), tangent);
    if (first * second > 0.0 ||
        (std::abs(first) <= 1e-12 && std::abs(second) <= 1e-12)) {
      continue;
    }
    const double denominator = first - second;
    if (std::abs(denominator) <= 1e-12)
      continue;
    const Vec3 candidate = interpolate(path[i], path[i + 1U],
                                       std::clamp(first / denominator, 0.0, 1.0));
    const double distance = length(subtract(candidate, control));
    if (distance < best) {
      best = distance;
      *intersection = candidate;
      found = true;
    }
  }
  return found;
}

bool makeAnchor(const Grid &grid, const std::vector<Vec3> &controls,
                std::size_t index, const Vec3 &reference, double clearance,
                Anchor *anchor) {
  if (anchor == nullptr)
    return false;
  const Vec3 delta = subtract(reference, controls[index]);
  const double distance = length(delta);
  if (distance <= 1e-5)
    return false;

  const Vec3 direction = scale(delta, 1.0 / distance);
  const double step = grid.resolution();
  double base_distance = 0.0;
  for (double travel = distance; travel >= 0.0; travel -= step) {
    const Vec3 sample = add(controls[index], scale(direction, travel));
    if (occupied(grid, sample, yaw(controls, index))) {
      base_distance = std::min(distance, travel + step);
      break;
    }
    if (travel < step) {
      base_distance = 0.0;
      break;
    }
  }
  anchor->base = add(controls[index], scale(direction, base_distance));
  anchor->direction = direction;
  anchor->clearance = clearance;
  return true;
}

}  // namespace

AnchorSet buildAnchors(const Grid &grid, const std::vector<Vec3> &controls,
                       const LocalPlannerParams &params,
                       const LocalPlanCancel &cancel) {
  AnchorSet result;
  result.controls.resize(controls.size());
  auto segments = collisionSegments(grid, controls);
  for (auto &segment : segments) {
    bool expanded = true;
    while (expanded) {
      expanded = false;
      const Vec3 &entry = controls[static_cast<std::size_t>(segment.first)];
      const Vec3 &exit = controls[static_cast<std::size_t>(segment.second)];
      const double segment_yaw = std::atan2(exit.y - entry.y, exit.x - entry.x);
      if (segment.first > 0 && occupied(grid, entry, segment_yaw)) {
        --segment.first;
        expanded = true;
      }
      if (segment.second + 1 < static_cast<int>(controls.size()) &&
          occupied(grid, exit, segment_yaw)) {
        ++segment.second;
        expanded = true;
      }
    }
  }
  result.collisionSegments = static_cast<int>(segments.size());
  if (segments.empty())
    return result;

  std::vector<std::pair<int, int>> bounds(segments.size());
  const int last_variable = static_cast<int>(controls.size()) - kOrder - 1;
  for (std::size_t i = 0U; i < segments.size(); ++i) {
    const int low = i == 0U
                        ? kOrder
                        : (segments[i].first + segments[i - 1U].second + 1) / 2;
    const int high = i + 1U == segments.size()
                         ? last_variable
                         : (segments[i].second + segments[i + 1U].first - 1) / 2;
    bounds[i] = {std::clamp(low, kOrder, last_variable),
                 std::clamp(high, kOrder, last_variable)};
  }

  const double clearance = std::max(grid.resolution(), params.scan.collisionDistance);
  for (std::size_t segment_index = 0U; segment_index < segments.size(); ++segment_index) {
    if (cancel && cancel()) {
      result.complete = false;
      result.failureReason = "planning_cancelled";
      return result;
    }
    const int first = segments[segment_index].first;
    const int second = segments[segment_index].second;
    const Vec3 &entry = controls[static_cast<std::size_t>(first)];
    const Vec3 &exit = controls[static_cast<std::size_t>(second)];
    const double segment_yaw = std::atan2(exit.y - entry.y, exit.x - entry.x);
    if (occupied(grid, entry, segment_yaw) || occupied(grid, exit, segment_yaw)) {
      result.complete = false;
      result.failureReason = "segment_endpoint_blocked";
      return result;
    }
    SearchResult searched = searchSegment(
        grid, entry, exit, segment_yaw, params, cancel);
    ++result.searches;
    if (!searched.found()) {
      result.complete = false;
      result.failureReason = searched.reason;
      return result;
    }
    result.paths.push_back(searched.path);

    std::vector<unsigned char> assigned(controls.size(), 0U);
    int pivot = -1;
    for (int control = first + 1; control < second; ++control) {
      const Vec3 tangent = subtract(controls[static_cast<std::size_t>(control + 1)],
                                    controls[static_cast<std::size_t>(control - 1)]);
      Vec3 intersection{};
      Anchor anchor;
      if (planeIntersection(controls[static_cast<std::size_t>(control)], tangent,
                            searched.path, &intersection) &&
          makeAnchor(grid, controls, static_cast<std::size_t>(control), intersection,
                     clearance, &anchor)) {
        result.controls[static_cast<std::size_t>(control)].push_back(anchor);
        assigned[static_cast<std::size_t>(control)] = 1U;
        pivot = control;
      }
    }

    if (pivot < 0 && second - first == 1) {
      const Vec3 middle = scale(add(entry, exit), 0.5);
      const Vec3 tangent = subtract(exit, entry);
      Vec3 intersection{};
      if (planeIntersection(middle, tangent, searched.path, &intersection)) {
        const Vec3 direction_delta = subtract(intersection, middle);
        const double direction_length = length(direction_delta);
        if (direction_length > 0.01) {
          result.controls[static_cast<std::size_t>(first)].push_back(
              {entry, scale(direction_delta, 1.0 / direction_length), clearance});
          assigned[static_cast<std::size_t>(first)] = 1U;
          pivot = first;
        }
      }
    }

    if (pivot < 0)
      continue;
    for (int control = pivot + 1; control <= bounds[segment_index].second; ++control) {
      if (assigned[static_cast<std::size_t>(control)] == 0U) {
        result.controls[static_cast<std::size_t>(control)].push_back(
            result.controls[static_cast<std::size_t>(control - 1)].back());
      }
    }
    for (int control = pivot - 1; control >= bounds[segment_index].first; --control) {
      if (assigned[static_cast<std::size_t>(control)] == 0U) {
        result.controls[static_cast<std::size_t>(control)].push_back(
            result.controls[static_cast<std::size_t>(control + 1)].back());
      }
    }
  }
  return result;
}

}  // namespace nav_kernel::local::scan
