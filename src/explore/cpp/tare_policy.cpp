#include "tare_policy.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace lingtu::explore {
namespace {

constexpr int kNeighbor4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
constexpr int kNeighbor8[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1},   {1, -1}, {1, 0},  {1, 1}};
constexpr double kPi = 3.14159265358979323846;

struct Cell {
  int row = 0;
  int col = 0;
};

double angleDelta(double a, double b) {
  double d = std::fmod(a - b + kPi, 2.0 * kPi);
  if (d < 0.0) {
    d += 2.0 * kPi;
  }
  return d - kPi;
}

bool isFree(const Grid2D& grid, int row, int col) {
  return grid.inBounds(row, col) && grid.at(row, col) == kFree;
}

bool worldToCell(const Grid2D& grid, double x, double y, Cell* out) {
  if (!grid.valid()) {
    return false;
  }
  const int col = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  const int row = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  if (!grid.inBounds(row, col)) {
    return false;
  }
  out->row = row;
  out->col = col;
  return true;
}

std::pair<double, double> cellToWorld(const Grid2D& grid, int row, int col) {
  return {
      grid.origin_x + (static_cast<double>(col) + 0.5) * grid.resolution,
      grid.origin_y + (static_cast<double>(row) + 0.5) * grid.resolution};
}

std::vector<std::uint8_t> reachableFree(const Grid2D& grid, Cell start) {
  std::vector<std::uint8_t> reachable(static_cast<std::size_t>(grid.width * grid.height), 0);
  std::deque<Cell> q;
  reachable[static_cast<std::size_t>(grid.index(start.row, start.col))] = 1;
  q.push_back(start);

  while (!q.empty()) {
    const Cell cur = q.front();
    q.pop_front();
    for (const auto& n : kNeighbor8) {
      const int row = cur.row + n[0];
      const int col = cur.col + n[1];
      if (!isFree(grid, row, col)) {
        continue;
      }
      const int idx = grid.index(row, col);
      if (reachable[static_cast<std::size_t>(idx)] != 0) {
        continue;
      }
      reachable[static_cast<std::size_t>(idx)] = 1;
      q.push_back({row, col});
    }
  }
  return reachable;
}

std::vector<std::uint8_t> frontierMask(
    const Grid2D& grid,
    const std::vector<std::uint8_t>& reachable) {
  std::vector<std::uint8_t> frontier(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const int idx = grid.index(row, col);
      if (reachable[static_cast<std::size_t>(idx)] == 0) {
        continue;
      }
      for (const auto& n : kNeighbor4) {
        const int nr = row + n[0];
        const int nc = col + n[1];
        if (grid.inBounds(nr, nc) && grid.at(nr, nc) == kUnknown) {
          frontier[static_cast<std::size_t>(idx)] = 1;
          break;
        }
      }
    }
  }
  return frontier;
}

std::vector<std::vector<Cell>> clusters(
    const Grid2D& grid,
    const std::vector<std::uint8_t>& frontier) {
  std::vector<std::uint8_t> seen(static_cast<std::size_t>(grid.width * grid.height), 0);
  std::vector<std::vector<Cell>> out;

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const int start_idx = grid.index(row, col);
      const auto start_pos = static_cast<std::size_t>(start_idx);
      if (frontier[start_pos] == 0 || seen[start_pos] != 0) {
        continue;
      }

      std::vector<Cell> cluster;
      std::deque<Cell> q;
      seen[start_pos] = 1;
      q.push_back({row, col});
      while (!q.empty()) {
        const Cell cur = q.front();
        q.pop_front();
        cluster.push_back(cur);
        for (const auto& n : kNeighbor8) {
          const int nr = cur.row + n[0];
          const int nc = cur.col + n[1];
          if (!grid.inBounds(nr, nc)) {
            continue;
          }
          const int idx = grid.index(nr, nc);
          const auto pos = static_cast<std::size_t>(idx);
          if (frontier[pos] == 0 || seen[pos] != 0) {
            continue;
          }
          seen[pos] = 1;
          q.push_back({nr, nc});
        }
      }
      out.push_back(std::move(cluster));
    }
  }
  return out;
}

Cell clusterCentroidCell(const std::vector<Cell>& cluster) {
  double row_sum = 0.0;
  double col_sum = 0.0;
  for (const Cell& cell : cluster) {
    row_sum += static_cast<double>(cell.row);
    col_sum += static_cast<double>(cell.col);
  }
  const double row_center = row_sum / static_cast<double>(cluster.size());
  const double col_center = col_sum / static_cast<double>(cluster.size());

  Cell best = cluster.front();
  double best_cost = std::numeric_limits<double>::infinity();
  for (const Cell& cell : cluster) {
    const double dr = static_cast<double>(cell.row) - row_center;
    const double dc = static_cast<double>(cell.col) - col_center;
    const double cost = dr * dr + dc * dc;
    if (cost < best_cost) {
      best_cost = cost;
      best = cell;
    }
  }
  return best;
}

int nearbyCount(const Cell& cell, const std::vector<Cell>& cluster, double range_m, double resolution) {
  const double radius_cells = range_m / std::max(resolution, 1e-6);
  const double radius_sq = radius_cells * radius_cells;
  int count = 0;
  for (const Cell& item : cluster) {
    const double dr = static_cast<double>(item.row - cell.row);
    const double dc = static_cast<double>(item.col - cell.col);
    if (dr * dr + dc * dc <= radius_sq) {
      ++count;
    }
  }
  return count;
}

bool recentlyVisited(double x, double y, const std::vector<Pose2D>& visited, double radius_m) {
  for (const Pose2D& pose : visited) {
    if (std::hypot(x - pose.x, y - pose.y) < radius_m) {
      return true;
    }
  }
  return false;
}

}  // namespace

TarePolicy::TarePolicy(TarePolicyConfig config) : config_(config) {}

const char* TarePolicy::name() const {
  return "frontier_viewpoint";
}

ExploreDecision TarePolicy::plan(const ExploreInput& input) const {
  return select(input.exploration_grid, input.robot_pose, input.visited_goals);
}

TareDecision TarePolicy::select(
    const Grid2D& grid,
    const Pose2D& robot,
    const std::vector<Pose2D>& visited_goals) const {
  TareDecision decision;
  if (!grid.valid()) {
    decision.reason = "missing_grid";
    return decision;
  }

  Cell start;
  if (!worldToCell(grid, robot.x, robot.y, &start) || !isFree(grid, start.row, start.col)) {
    decision.reason = "robot_not_in_free_space";
    return decision;
  }

  const std::vector<std::uint8_t> reachable = reachableFree(grid, start);
  const std::vector<std::uint8_t> frontiers = frontierMask(grid, reachable);
  const std::vector<std::vector<Cell>> frontier_clusters = clusters(grid, frontiers);

  bool saw_frontier = false;
  for (const auto& cluster : frontier_clusters) {
    if (!cluster.empty()) {
      saw_frontier = true;
      break;
    }
  }
  if (!saw_frontier) {
    decision.done = true;
    decision.reason = "no_frontiers";
    return decision;
  }

  std::vector<TareCandidate> candidates;
  for (const std::vector<Cell>& cluster : frontier_clusters) {
    if (static_cast<int>(cluster.size()) < config_.min_frontier_size) {
      continue;
    }

    const Cell centroid = clusterCentroidCell(cluster);
    const auto [frontier_x, frontier_y] = cellToWorld(grid, centroid.row, centroid.col);
    const int radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(config_.candidate_radius_m / grid.resolution)));

    bool has_best = false;
    TareCandidate best;
    for (int row = std::max(0, centroid.row - radius_cells);
         row <= std::min(grid.height - 1, centroid.row + radius_cells);
         ++row) {
      for (int col = std::max(0, centroid.col - radius_cells);
           col <= std::min(grid.width - 1, centroid.col + radius_cells);
           ++col) {
        const int idx = grid.index(row, col);
        if (reachable[static_cast<std::size_t>(idx)] == 0) {
          continue;
        }
        const auto [x, y] = cellToWorld(grid, row, col);
        const double distance = std::hypot(x - robot.x, y - robot.y);
        if (distance < config_.min_goal_distance_m) {
          continue;
        }
        if (recentlyVisited(x, y, visited_goals, config_.novelty_radius_m)) {
          continue;
        }
        const int coverage = nearbyCount({row, col}, cluster, config_.sensor_range_m, grid.resolution);
        if (coverage <= 0) {
          continue;
        }
        const double heading = std::atan2(y - robot.y, x - robot.x);
        const double momentum = (1.0 + std::cos(angleDelta(heading, robot.yaw))) * 0.5;
        const double frontier_distance = std::hypot(x - frontier_x, y - frontier_y);
        const double score =
            static_cast<double>(coverage) - distance * 0.15 + momentum * 2.0 - frontier_distance * 0.05;

        TareCandidate candidate;
        candidate.x = x;
        candidate.y = y;
        candidate.score = score;
        candidate.distance_m = distance;
        candidate.frontier_size = static_cast<int>(cluster.size());
        candidate.covered_frontier_cells = coverage;
        if (!has_best || candidate.score > best.score) {
          best = candidate;
          has_best = true;
        }
      }
    }
    if (has_best) {
      candidates.push_back(best);
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const TareCandidate& a, const TareCandidate& b) {
    return a.score > b.score;
  });
  if (config_.max_candidates > 0 && static_cast<int>(candidates.size()) > config_.max_candidates) {
    candidates.resize(static_cast<std::size_t>(config_.max_candidates));
  }

  if (candidates.empty()) {
    decision.reason = "no_reachable_viewpoint";
    return decision;
  }

  decision.candidates = std::move(candidates);
  decision.has_goal = true;
  decision.goal_x = decision.candidates.front().x;
  decision.goal_y = decision.candidates.front().y;
  decision.goal_z = 0.0;
  decision.reason = "selected_viewpoint";
  return decision;
}

}  // namespace lingtu::explore
