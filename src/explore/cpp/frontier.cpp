#include "frontier.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>

namespace lingtu::explore::detail {
namespace {

constexpr int kNeighbor4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
constexpr int kNeighbor8[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1},   {1, -1}, {1, 0},  {1, 1}};
constexpr double kBoundaryTolerance = 1e-5;
constexpr double kDiagonalCost = 1.4142135623730951;

bool IsFree(const Grid2D& grid, int row, int col) {
  return grid.inBounds(row, col) && grid.at(row, col) == kFree;
}

bool CanTraverse(
    const Grid2D& grid,
    int row,
    int col,
    int next_row,
    int next_col) {
  if (!IsFree(grid, next_row, next_col)) {
    return false;
  }
  const int dr = next_row - row;
  const int dc = next_col - col;
  if (dr != 0 && dc != 0) {
    return IsFree(grid, row + dr, col) && IsFree(grid, row, col + dc);
  }
  return true;
}

std::uint64_t Mix(std::uint64_t value) {
  value ^= value >> 30U;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27U;
  value *= 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

std::uint64_t ClusterId(
    const Grid2D& grid,
    const Cell& centroid,
    std::size_t size) {
  const auto [x, y] = CellToWorld(grid, centroid.row, centroid.col);
  const auto qx = static_cast<std::int64_t>(
      std::llround(x / std::max(grid.resolution, 1e-9)));
  const auto qy = static_cast<std::int64_t>(
      std::llround(y / std::max(grid.resolution, 1e-9)));
  return Mix(static_cast<std::uint64_t>(qx)) ^
      (Mix(static_cast<std::uint64_t>(qy)) << 1U) ^
      Mix(static_cast<std::uint64_t>(size));
}

Cell CentroidCell(const std::vector<Cell>& cluster) {
  double row_sum = 0.0;
  double col_sum = 0.0;
  for (const Cell& cell : cluster) {
    row_sum += static_cast<double>(cell.row);
    col_sum += static_cast<double>(cell.col);
  }
  const double center_row = row_sum / static_cast<double>(cluster.size());
  const double center_col = col_sum / static_cast<double>(cluster.size());

  Cell best = cluster.front();
  double best_cost = std::numeric_limits<double>::infinity();
  for (const Cell& cell : cluster) {
    const double dr = static_cast<double>(cell.row) - center_row;
    const double dc = static_cast<double>(cell.col) - center_col;
    const double cost = dr * dr + dc * dc;
    if (cost < best_cost) {
      best_cost = cost;
      best = cell;
    }
  }
  return best;
}

}  // namespace

bool WorldToCell(
    const Grid2D& grid,
    double x,
    double y,
    Cell* out) {
  if (!grid.valid() || out == nullptr || !std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  auto stable_floor = [](double value) {
    const double nearest = std::round(value);
    if (std::abs(value - nearest) <= kBoundaryTolerance) {
      value = nearest;
    }
    return static_cast<int>(std::floor(value));
  };
  const int col = stable_floor((x - grid.origin_x) / grid.resolution);
  const int row = stable_floor((y - grid.origin_y) / grid.resolution);
  if (!grid.inBounds(row, col)) {
    return false;
  }
  out->row = row;
  out->col = col;
  return true;
}

std::pair<double, double> CellToWorld(
    const Grid2D& grid,
    int row,
    int col) {
  return {
      grid.origin_x + (static_cast<double>(col) + 0.5) * grid.resolution,
      grid.origin_y + (static_cast<double>(row) + 0.5) * grid.resolution,
  };
}

bool HasFreeLineOfSight(
    const Grid2D& grid,
    const Pose2D& from,
    const Pose2D& to) {
  Cell start;
  Cell goal;
  if (!WorldToCell(grid, from.x, from.y, &start) ||
      !WorldToCell(grid, to.x, to.y, &goal) ||
      !IsFree(grid, start.row, start.col) ||
      !IsFree(grid, goal.row, goal.col)) {
    return false;
  }

  int x = start.col;
  int y = start.row;
  const int goal_x = goal.col;
  const int goal_y = goal.row;
  const int dx = std::abs(goal_x - x);
  const int sx = x < goal_x ? 1 : -1;
  const int dy = -std::abs(goal_y - y);
  const int sy = y < goal_y ? 1 : -1;
  int error = dx + dy;

  while (true) {
    if (!IsFree(grid, y, x)) {
      return false;
    }
    if (x == goal_x && y == goal_y) {
      return true;
    }
    const int previous_x = x;
    const int previous_y = y;
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y += sy;
    }
    if (x != previous_x && y != previous_y &&
        (!IsFree(grid, previous_y, x) || !IsFree(grid, y, previous_x))) {
      return false;
    }
  }
}

FrontierAnalysis AnalyzeFrontiers(
    const Grid2D& grid,
    const Pose2D& robot,
    int min_frontier_size,
    std::size_t max_frontier_cells,
    std::size_t max_frontier_clusters,
    const ExploreCancelCheck& cancel) {
  FrontierAnalysis result;
  if (!grid.valid()) {
    result.reason = "missing_grid";
    return result;
  }
  if (!WorldToCell(grid, robot.x, robot.y, &result.robot_cell) ||
      !IsFree(grid, result.robot_cell.row, result.robot_cell.col)) {
    result.reason = "robot_not_in_free_space";
    return result;
  }

  const std::size_t cell_count = grid.cells.size();
  result.reachable.assign(cell_count, 0U);
  result.travel_distance_m.assign(
      cell_count,
      std::numeric_limits<double>::infinity());
  using QueueEntry = std::tuple<double, int, int>;
  std::priority_queue<
      QueueEntry,
      std::vector<QueueEntry>,
      std::greater<QueueEntry>> queue;

  const int start_index =
      grid.index(result.robot_cell.row, result.robot_cell.col);
  result.travel_distance_m[static_cast<std::size_t>(start_index)] = 0.0;
  queue.emplace(0.0, result.robot_cell.row, result.robot_cell.col);

  std::size_t iterations = 0U;
  while (!queue.empty()) {
    const auto [distance, row, col] = queue.top();
    queue.pop();
    const std::size_t position =
        static_cast<std::size_t>(grid.index(row, col));
    if (distance > result.travel_distance_m[position] + 1e-9) {
      continue;
    }
    if (result.reachable[position] == 0U) {
      result.reachable[position] = 1U;
      ++result.reachable_free_cells;
    }
    if ((++iterations & 1023U) == 0U && cancel && cancel()) {
      result.reason = "cancelled";
      return result;
    }

    for (const auto& offset : kNeighbor8) {
      const int next_row = row + offset[0];
      const int next_col = col + offset[1];
      if (!CanTraverse(grid, row, col, next_row, next_col)) {
        continue;
      }
      const std::size_t next_position =
          static_cast<std::size_t>(grid.index(next_row, next_col));
      const double step =
          (offset[0] != 0 && offset[1] != 0 ? kDiagonalCost : 1.0) *
          grid.resolution;
      const double next_distance = distance + step;
      if (next_distance + 1e-9 < result.travel_distance_m[next_position]) {
        result.travel_distance_m[next_position] = next_distance;
        queue.emplace(next_distance, next_row, next_col);
      }
    }
  }

  std::vector<std::uint8_t> frontier(cell_count, 0U);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const std::size_t position =
          static_cast<std::size_t>(grid.index(row, col));
      if (result.reachable[position] == 0U) {
        continue;
      }
      for (const auto& offset : kNeighbor4) {
        const int next_row = row + offset[0];
        const int next_col = col + offset[1];
        if (grid.inBounds(next_row, next_col) &&
            grid.at(next_row, next_col) == kUnknown) {
          frontier[position] = 1U;
          ++result.frontier_cells;
          if (result.frontier_cells > max_frontier_cells) {
            result.resource_limited = true;
            result.reason = "resource_limit_frontier_cells";
            return result;
          }
          break;
        }
      }
    }
  }

  std::vector<std::uint8_t> seen(cell_count, 0U);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const std::size_t start =
          static_cast<std::size_t>(grid.index(row, col));
      if (frontier[start] == 0U || seen[start] != 0U) {
        continue;
      }
      std::vector<Cell> cells;
      std::deque<Cell> pending;
      seen[start] = 1U;
      pending.push_back({row, col});
      while (!pending.empty()) {
        const Cell current = pending.front();
        pending.pop_front();
        cells.push_back(current);
        if ((++iterations & 1023U) == 0U && cancel && cancel()) {
          result.reason = "cancelled";
          return result;
        }
        for (const auto& offset : kNeighbor8) {
          const int next_row = current.row + offset[0];
          const int next_col = current.col + offset[1];
          if (!grid.inBounds(next_row, next_col)) {
            continue;
          }
          const std::size_t position =
              static_cast<std::size_t>(grid.index(next_row, next_col));
          if (frontier[position] == 0U || seen[position] != 0U) {
            continue;
          }
          seen[position] = 1U;
          pending.push_back({next_row, next_col});
        }
      }
      if (static_cast<int>(cells.size()) < std::max(1, min_frontier_size)) {
        continue;
      }
      if (result.clusters.size() >= max_frontier_clusters) {
        result.resource_limited = true;
        result.reason = "resource_limit_frontier_clusters";
        return result;
      }
      FrontierCluster cluster;
      cluster.centroid = CentroidCell(cells);
      cluster.id = ClusterId(grid, cluster.centroid, cells.size());
      cluster.cells = std::move(cells);
      result.clusters.push_back(std::move(cluster));
    }
  }

  result.valid = true;
  result.reason = result.clusters.empty() ? "no_frontiers" : "ok";
  return result;
}

}  // namespace lingtu::explore::detail
