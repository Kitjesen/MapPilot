#include "planning/local/scan/search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>

namespace nav_kernel::local::scan {
namespace {

struct QueueItem {
  double f{0.0};
  int linear{0};
};

struct QueueGreater {
  bool operator()(const QueueItem &a, const QueueItem &b) const noexcept { return a.f > b.f; }
};

class OpenQueue : public std::priority_queue<QueueItem, std::vector<QueueItem>, QueueGreater> {
 public:
  void reset() { this->c.clear(); }
};

struct SearchScratch {
  std::vector<double> cost;
  std::vector<int> parent;
  std::vector<unsigned char> closed;
  OpenQueue open;

  void reset(int count) {
    const auto size = static_cast<std::size_t>(count);
    cost.resize(size);
    parent.resize(size);
    closed.resize(size);
    std::fill(cost.begin(), cost.end(), std::numeric_limits<double>::infinity());
    std::fill(parent.begin(), parent.end(), -1);
    std::fill(closed.begin(), closed.end(), static_cast<unsigned char>(0));
    open.reset();
  }
};

SearchScratch &searchScratch() {
  thread_local SearchScratch scratch;
  return scratch;
}

double yawBetween(const Vec3 &from, const Vec3 &to, double fallback) {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return std::hypot(dx, dy) > 1e-9 ? std::atan2(dy, dx) : fallback;
}

bool routeIsFree(const Grid &grid) {
  const auto &route = grid.route();
  if (route.size() < 2)
    return false;
  for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    if (!grid.segmentFree(route[i], route[i + 1]))
      return false;
  }
  return true;
}

bool routeSlopeAllowed(const Grid &grid, const LocalPlannerParams &params) {
  const auto &route = grid.route();
  for (std::size_t i = 0; i + 1U < route.size(); ++i) {
    const double horizontal =
        std::hypot(route[i + 1U].x - route[i].x, route[i + 1U].y - route[i].y);
    const double vertical = std::abs(route[i + 1U].z - route[i].z);
    if (vertical > 1e-6 &&
        (horizontal <= 1e-6 || vertical / horizontal > std::max(0.0, params.scan.maxSlope))) {
      return false;
    }
  }
  return true;
}

Vec3 projectedPoint(const Grid &grid, int x, int y, const Vec3 &plane_start,
                    const Vec3 &plane_end) {
  GridIndex sample{x, y, grid.index(plane_start).z};
  Vec3 point = grid.point(sample);
  const double dx = plane_end.x - plane_start.x;
  const double dy = plane_end.y - plane_start.y;
  const double length_sq = dx * dx + dy * dy;
  double ratio = 0.0;
  if (length_sq > 1e-8) {
    ratio = std::clamp(
        ((point.x - plane_start.x) * dx + (point.y - plane_start.y) * dy) / length_sq, 0.0, 1.0);
  }
  point.z = plane_start.z + ratio * (plane_end.z - plane_start.z);
  return point;
}

GridIndex projectedIndex(const Grid &grid, int x, int y, const Vec3 &plane_start,
                         const Vec3 &plane_end) {
  return grid.index(projectedPoint(grid, x, y, plane_start, plane_end));
}

GridIndex nearestFreeGoal(const Grid &grid, const GridIndex &requested, const Vec3 &plane_start,
                          const Vec3 &plane_end, double yaw, int max_radius_cells) {
  const GridIndex projected =
      projectedIndex(grid, requested.x, requested.y, plane_start, plane_end);
  if (grid.free(projected, yaw))
    return projected;
  for (int radius = 1; radius <= std::max(1, max_radius_cells); ++radius) {
    GridIndex best = projected;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius)
          continue;
        const GridIndex candidate =
            projectedIndex(grid, requested.x + dx, requested.y + dy, plane_start, plane_end);
        if (!grid.contains(candidate) || !grid.free(candidate, yaw))
          continue;
        const double distance = grid.routeDistance(
            projectedPoint(grid, candidate.x, candidate.y, plane_start, plane_end));
        if (distance < best_distance) {
          best_distance = distance;
          best = candidate;
        }
      }
    }
    if (best_distance < std::numeric_limits<double>::infinity())
      return best;
  }
  return {-1, -1, -1};
}

std::vector<Vec3> simplify(const Grid &grid, const std::vector<Vec3> &path) {
  if (path.size() <= 2)
    return path;
  std::vector<Vec3> out;
  out.reserve(path.size());
  out.push_back(path.front());
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t next = path.size() - 1;
    while (next > anchor + 1 && !grid.segmentFree(path[anchor], path[next])) {
      --next;
    }
    out.push_back(path[next]);
    anchor = next;
  }
  return out;
}

SearchResult searchInside(const Grid &grid, const Vec3 &start, double start_yaw,
                          const Vec3 &requested_goal, const GridIndex &start_index,
                          const GridIndex &goal_index, const LocalPlannerParams &params) {
  SearchResult result;
  const double goal_yaw = yawBetween(start, requested_goal, start_yaw);
  if (!grid.contains(goal_index) || !grid.free(goal_index, goal_yaw)) {
    result.reason = "start_or_goal_blocked";
    return result;
  }

  const int count = grid.cellCount();
  auto &scratch = searchScratch();
  scratch.reset(count);
  auto &cost = scratch.cost;
  auto &parent = scratch.parent;
  auto &closed = scratch.closed;
  auto &open = scratch.open;

  const int start_linear = grid.linear(start_index);
  const int goal_linear = grid.linear(goal_index);
  const Vec3 projected_goal =
      projectedPoint(grid, goal_index.x, goal_index.y, start, requested_goal);
  cost[static_cast<std::size_t>(start_linear)] = 0.0;
  open.push({distance3D(start, projected_goal), start_linear});

  const int max_nodes = std::clamp(params.scan.maxSearchNodes, 100, count);
  while (!open.empty() && result.expandedNodes < max_nodes) {
    const QueueItem item = open.top();
    open.pop();
    if (closed[static_cast<std::size_t>(item.linear)] != 0U)
      continue;
    closed[static_cast<std::size_t>(item.linear)] = 1U;
    ++result.expandedNodes;
    if (item.linear == goal_linear)
      break;

    const GridIndex current_index = grid.indexFromLinear(item.linear);
    const Vec3 current =
        item.linear == start_linear
            ? start
            : projectedPoint(grid, current_index.x, current_index.y, start, requested_goal);
    double current_yaw = start_yaw;
    const int parent_linear = parent[static_cast<std::size_t>(item.linear)];
    if (parent_linear >= 0) {
      const GridIndex parent_index = grid.indexFromLinear(parent_linear);
      current_yaw =
          yawBetween(projectedPoint(grid, parent_index.x, parent_index.y, start, requested_goal),
                     current, start_yaw);
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0)
          continue;
        const GridIndex next_index =
            projectedIndex(grid, current_index.x + dx, current_index.y + dy, start, requested_goal);
        if (!grid.contains(next_index))
          continue;
        const int next_linear = grid.linear(next_index);
        if (closed[static_cast<std::size_t>(next_linear)] != 0U)
          continue;
        const Vec3 next = projectedPoint(grid, next_index.x, next_index.y, start, requested_goal);
        const double yaw = yawBetween(current, next, current_yaw);
        if (!grid.free(next_index, yaw) || !grid.segmentFree(current, next)) {
          continue;
        }

        const double step = distance3D(current, next);
        const double guide = grid.routeDistance(next);
        const double tentative =
            cost[static_cast<std::size_t>(item.linear)] + step + params.scan.guideWeight * guide;
        if (tentative >= cost[static_cast<std::size_t>(next_linear)])
          continue;
        cost[static_cast<std::size_t>(next_linear)] = tentative;
        parent[static_cast<std::size_t>(next_linear)] = item.linear;
        const double heuristic = distance3D(next, projected_goal);
        open.push({tentative + heuristic, next_linear});
      }
    }
  }

  if (closed[static_cast<std::size_t>(goal_linear)] == 0U) {
    result.reason = result.expandedNodes >= max_nodes ? "search_budget_exhausted" : "no_path";
    return result;
  }

  std::vector<Vec3> reversed;
  for (int cursor = goal_linear; cursor >= 0; cursor = parent[static_cast<std::size_t>(cursor)]) {
    const GridIndex cursor_index = grid.indexFromLinear(cursor);
    reversed.push_back(projectedPoint(grid, cursor_index.x, cursor_index.y, start, requested_goal));
    if (cursor == start_linear)
      break;
  }
  if (reversed.empty() || grid.linear(grid.index(reversed.back())) != start_linear) {
    result.reason = "search_parent_chain_invalid";
    return result;
  }
  std::reverse(reversed.begin(), reversed.end());
  reversed.front() = start;
  if (grid.contains(grid.index(requested_goal)) &&
      grid.segmentFree(reversed.back(), requested_goal)) {
    reversed.back() = requested_goal;
  }
  result.path = simplify(grid, reversed);
  result.reason = "search_ready";
  return result;
}

std::vector<std::pair<std::size_t, std::size_t>> collisionEdgeGroups(const Grid &grid) {
  std::vector<std::pair<std::size_t, std::size_t>> groups;
  const auto &route = grid.route();
  std::size_t edge = 0U;
  while (edge + 1U < route.size()) {
    if (grid.segmentFree(route[edge], route[edge + 1U])) {
      ++edge;
      continue;
    }
    const std::size_t first = edge;
    while (edge + 1U < route.size() && !grid.segmentFree(route[edge], route[edge + 1U])) {
      ++edge;
    }
    groups.emplace_back(first, edge - 1U);
  }
  return groups;
}

void appendWithoutDuplicate(std::vector<Vec3> &output, const Vec3 &point) {
  if (output.empty() || distance3D(output.back(), point) > 1e-5) {
    output.push_back(point);
  }
}

SearchResult searchCollisionSegments(const Grid &grid, double start_yaw,
                                     const LocalPlannerParams &params) {
  SearchResult result;
  const auto &route = grid.route();
  const auto groups = collisionEdgeGroups(grid);
  if (groups.empty()) {
    result.path = route;
    result.reason = "route_clear";
    return result;
  }

  std::vector<Vec3> combined;
  std::size_t route_cursor = 0U;
  for (const auto &group : groups) {
    const std::size_t entry_index = group.first;
    const std::size_t exit_index = group.second + 1U;
    for (; route_cursor <= entry_index; ++route_cursor) {
      appendWithoutDuplicate(combined, route[route_cursor]);
    }

    const Vec3 &entry = route[entry_index];
    const Vec3 &exit = route[exit_index];
    const double segment_yaw = yawBetween(entry, exit, start_yaw);
    const double snap_distance = 0.5 * std::max(params.vehicleLength, params.vehicleWidth) +
                                 std::max(0.0, params.footprintPadding) +
                                 std::max(0.0, params.scan.obstacleClearance);
    const int snap_radius_cells =
        std::clamp(static_cast<int>(std::ceil(snap_distance / grid.resolution())) + 2, 4, 16);
    const GridIndex entry_grid = grid.index(entry);
    const GridIndex exit_grid =
        nearestFreeGoal(grid, grid.index(exit), entry, exit, segment_yaw, snap_radius_cells);
    if (!grid.contains(entry_grid)) {
      result.reason = "collision_segment_entry_outside_grid";
      return result;
    }
    if (entry_index != 0U && !grid.free(entry, segment_yaw)) {
      result.reason = "collision_segment_entry_blocked";
      return result;
    }
    if (!grid.contains(exit_grid)) {
      result.reason = "collision_segment_exit_unreachable";
      return result;
    }

    SearchResult segment =
        searchInside(grid, entry, segment_yaw, exit, entry_grid, exit_grid, params);
    result.expandedNodes += segment.expandedNodes;
    if (!segment.found()) {
      result.reason = std::string{"collision_segment_"} + segment.reason;
      return result;
    }
    for (const Vec3 &point : segment.path) {
      appendWithoutDuplicate(combined, point);
    }
    route_cursor = exit_index + 1U;
    start_yaw = segment_yaw;
  }
  for (; route_cursor < route.size(); ++route_cursor) {
    appendWithoutDuplicate(combined, route[route_cursor]);
  }

  for (std::size_t i = 0U; i + 1U < combined.size(); ++i) {
    if (!grid.segmentFree(combined[i], combined[i + 1U])) {
      result.reason = "collision_segment_splice_invalid";
      result.path.clear();
      return result;
    }
  }
  result.path = std::move(combined);
  result.reason = "segment_search_ready";
  return result;
}

SearchResult searchBoundaryFallback(const Grid &grid, const Vec3 &start, double start_yaw,
                                    const Vec3 &requested_goal, const GridIndex &start_index,
                                    const LocalPlannerParams &params) {
  SearchResult result;
  if (!grid.contains(start_index)) {
    result.reason = "boundary_start_outside_grid";
    return result;
  }

  // Paper Sec. V-C: augment the XY map with exactly one virtual free voxel
  // outside each real boundary. The layer is hypothesis-only and is never
  // returned to execution.
  const int width = grid.sizeX() + 2;
  const int height = grid.sizeY() + 2;
  const int count = width * height;
  const auto linear = [width](int x, int y) { return y * width + x; };
  const auto xFromLinear = [width](int value) { return value % width; };
  const auto yFromLinear = [width](int value) { return value / width; };
  const auto real = [width, height](int x, int y) {
    return x > 0 && x < width - 1 && y > 0 && y < height - 1;
  };
  const auto point = [&](int augmented_x, int augmented_y) {
    return projectedPoint(grid, augmented_x - 1, augmented_y - 1, start, requested_goal);
  };

  const int start_x = start_index.x + 1;
  const int start_y = start_index.y + 1;
  const int start_linear = linear(start_x, start_y);
  const GridIndex requested_index = grid.index(requested_goal);
  int goal_x = std::clamp(requested_index.x + 1, 0, width - 1);
  int goal_y = std::clamp(requested_index.y + 1, 0, height - 1);
  const double requested_yaw = yawBetween(start, requested_goal, start_yaw);
  if (!grid.contains(requested_index) || !grid.free(requested_goal, requested_yaw)) {
    double best_distance = std::numeric_limits<double>::infinity();
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (real(x, y))
          continue;
        const double distance = distance3D(point(x, y), requested_goal);
        if (distance < best_distance) {
          best_distance = distance;
          goal_x = x;
          goal_y = y;
        }
      }
    }
  }
  const int goal_linear = linear(goal_x, goal_y);

  auto &scratch = searchScratch();
  scratch.reset(count);
  auto &cost = scratch.cost;
  auto &parent = scratch.parent;
  auto &closed = scratch.closed;
  auto &open = scratch.open;
  cost[static_cast<std::size_t>(start_linear)] = 0.0;
  open.push({distance3D(start, point(goal_x, goal_y)), start_linear});
  const int max_nodes = std::clamp(params.scan.maxSearchNodes, 100, count);

  while (!open.empty() && result.expandedNodes < max_nodes) {
    const QueueItem item = open.top();
    open.pop();
    if (closed[static_cast<std::size_t>(item.linear)] != 0U)
      continue;
    closed[static_cast<std::size_t>(item.linear)] = 1U;
    ++result.expandedNodes;
    if (item.linear == goal_linear)
      break;

    const int current_x = xFromLinear(item.linear);
    const int current_y = yFromLinear(item.linear);
    const Vec3 current = item.linear == start_linear ? start : point(current_x, current_y);
    double current_yaw = start_yaw;
    const int parent_linear = parent[static_cast<std::size_t>(item.linear)];
    if (parent_linear >= 0) {
      current_yaw = yawBetween(point(xFromLinear(parent_linear), yFromLinear(parent_linear)),
                               current, start_yaw);
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0)
          continue;
        const int next_x = current_x + dx;
        const int next_y = current_y + dy;
        if (next_x < 0 || next_x >= width || next_y < 0 || next_y >= height)
          continue;
        const int next_linear = linear(next_x, next_y);
        if (closed[static_cast<std::size_t>(next_linear)] != 0U)
          continue;
        const Vec3 next = point(next_x, next_y);
        const double yaw = yawBetween(current, next, current_yaw);
        if (real(next_x, next_y)) {
          if (!grid.hypothesisFree(next, yaw))
            continue;
        }
        const double tentative = cost[static_cast<std::size_t>(item.linear)] +
                                 distance3D(current, next) +
                                 params.scan.guideWeight * grid.routeDistance(next);
        if (tentative >= cost[static_cast<std::size_t>(next_linear)])
          continue;
        cost[static_cast<std::size_t>(next_linear)] = tentative;
        parent[static_cast<std::size_t>(next_linear)] = item.linear;
        open.push({tentative + distance3D(next, point(goal_x, goal_y)), next_linear});
      }
    }
  }

  if (closed[static_cast<std::size_t>(goal_linear)] == 0U) {
    result.reason = result.expandedNodes >= max_nodes ? "boundary_search_budget_exhausted"
                                                      : "boundary_hypothesis_failed";
    return result;
  }

  std::vector<int> hypothesis;
  for (int cursor = goal_linear; cursor >= 0; cursor = parent[static_cast<std::size_t>(cursor)]) {
    hypothesis.push_back(cursor);
    if (cursor == start_linear)
      break;
  }
  if (hypothesis.empty() || hypothesis.back() != start_linear) {
    result.reason = "boundary_parent_chain_invalid";
    return result;
  }
  std::reverse(hypothesis.begin(), hypothesis.end());

  std::size_t first_virtual = hypothesis.size();
  for (std::size_t index = 1U; index < hypothesis.size(); ++index) {
    const int x = xFromLinear(hypothesis[index]);
    const int y = yFromLinear(hypothesis[index]);
    if (!real(x, y)) {
      first_virtual = index;
      break;
    }
  }
  if (first_virtual == hypothesis.size() || first_virtual <= 1U) {
    result.reason = "boundary_no_exit";
    return result;
  }

  // The body footprint can extend beyond the center voxel. Walk inward from
  // the hypothesis exit until the strict real-map search accepts a bounded
  // executable target; no virtual node is ever returned.
  for (std::size_t candidate = first_virtual; candidate-- > 1U;) {
    const int fallback_linear = hypothesis[candidate];
    const int fallback_x = xFromLinear(fallback_linear);
    const int fallback_y = yFromLinear(fallback_linear);
    if (!real(fallback_x, fallback_y))
      continue;
    const Vec3 fallback_target = point(fallback_x, fallback_y);
    const GridIndex fallback_index =
        projectedIndex(grid, fallback_x - 1, fallback_y - 1, start, requested_goal);
    SearchResult executable =
        searchInside(grid, start, start_yaw, fallback_target, start_index, fallback_index, params);
    result.expandedNodes += executable.expandedNodes;
    if (!executable.found())
      continue;
    result.path = std::move(executable.path);
    result.boundaryFallback = true;
    result.fallbackTarget = fallback_target;
    result.reason = "boundary_fallback";
    return result;
  }
  result.reason = "boundary_executable_no_path";
  return result;
}

}  // namespace

SearchResult search(const Grid &grid, const Vec3 &start, double startYaw,
                    const LocalPlannerParams &params) {
  SearchResult result;
  if (!grid.valid() || grid.route().size() < 2) {
    result.reason = "grid_invalid";
    return result;
  }
  if (!routeSlopeAllowed(grid, params)) {
    result.reason = "route_slope_exceeded";
    return result;
  }
  if (routeIsFree(grid)) {
    result.path = grid.route();
    result.reason = "route_clear";
    return result;
  }

  const Vec3 requested_goal = grid.route().back();
  const GridIndex start_index = grid.index(start);
  if (!grid.contains(start_index)) {
    result.reason = "start_outside_grid";
    return result;
  }
  const Vec3 start_cell = grid.point(start_index);
  if (!grid.routeHeightAllowed(start_cell)) {
    result.reason = "start_height_blocked";
    return result;
  }
  if (!grid.obstacleFree(start, startYaw)) {
    result.reason = "start_obstacle_blocked";
    return result;
  }
  SearchResult inside = searchCollisionSegments(grid, startYaw, params);
  if (inside.found())
    return inside;

  SearchResult fallback =
      searchBoundaryFallback(grid, start, startYaw, requested_goal, start_index, params);
  fallback.expandedNodes += inside.expandedNodes;
  if (fallback.found())
    return fallback;
  return fallback;
}

}  // namespace nav_kernel::local::scan
