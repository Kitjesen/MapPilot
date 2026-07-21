#include "tare_policy.hpp"

#include "frontier.hpp"
#include "keypose_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lingtu::explore {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

double AngleDelta(double lhs, double rhs) {
  double delta = std::fmod(lhs - rhs + kPi, 2.0 * kPi);
  if (delta < 0.0) {
    delta += 2.0 * kPi;
  }
  return delta - kPi;
}

double PoseDistance(const Pose2D& lhs, const Pose2D& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool FinitePose(const Pose2D& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
      std::isfinite(pose.yaw);
}

std::uint64_t CoverageKey(double x, double y, double resolution_m) {
  const auto qx = static_cast<std::int64_t>(
      std::floor(x / resolution_m));
  const auto qy = static_cast<std::int64_t>(
      std::floor(y / resolution_m));
  return (static_cast<std::uint64_t>(
              static_cast<std::uint32_t>(qx)) << 32U) |
      static_cast<std::uint32_t>(qy);
}

bool RecentlyVisited(
    double x,
    double y,
    const std::vector<Pose2D>& visited,
    double radius_m) {
  return std::any_of(
      visited.begin(),
      visited.end(),
      [x, y, radius_m](const Pose2D& pose) {
        return std::hypot(x - pose.x, y - pose.y) < radius_m;
      });
}

bool GridContractValid(const Grid2D& grid) {
  if (!grid.valid() || !std::isfinite(grid.resolution) ||
      !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y)) {
    return false;
  }
  return std::all_of(
      grid.cells.begin(),
      grid.cells.end(),
      [](std::int8_t value) {
        return value == kUnknown || value == kFree || value == kOccupied;
      });
}

double RouteLength(
    const Pose2D& robot,
    const std::vector<Pose2D>& route) {
  double length = 0.0;
  Pose2D current = robot;
  for (const Pose2D& target : route) {
    length += PoseDistance(current, target);
    current = target;
  }
  return length;
}

std::vector<std::size_t> BuildRouteOrder(
    const std::vector<ExploreCandidate>& candidates,
    const Pose2D& robot,
    const TarePolicyConfig& config,
    bool* local_phase) {
  std::vector<std::size_t> pool;
  pool.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].route_cost_m <= config.local_route_radius_m) {
      pool.push_back(index);
    }
  }
  *local_phase = !pool.empty();
  if (pool.empty()) {
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      pool.push_back(index);
    }
  }
  if (pool.size() > config.max_route_targets) {
    pool.resize(config.max_route_targets);
  }
  if (pool.empty()) {
    return {};
  }

  std::vector<std::size_t> order;
  order.reserve(pool.size());
  order.push_back(pool.front());
  pool.erase(pool.begin());
  while (!pool.empty()) {
    const ExploreCandidate& current = candidates[order.back()];
    const auto best = std::min_element(
        pool.begin(),
        pool.end(),
        [&](std::size_t lhs, std::size_t rhs) {
          const double lhs_distance = std::hypot(
              candidates[lhs].x - current.x,
              candidates[lhs].y - current.y);
          const double rhs_distance = std::hypot(
              candidates[rhs].x - current.x,
              candidates[rhs].y - current.y);
          const double lhs_cost =
              lhs_distance - 0.02 * std::max(0.0, candidates[lhs].score);
          const double rhs_cost =
              rhs_distance - 0.02 * std::max(0.0, candidates[rhs].score);
          return lhs_cost < rhs_cost;
        });
    order.push_back(*best);
    pool.erase(best);
  }

  auto segment_cost = [&](std::size_t lhs, std::size_t rhs) {
    return std::hypot(
        candidates[lhs].x - candidates[rhs].x,
        candidates[lhs].y - candidates[rhs].y);
  };
  for (int iteration = 0;
       iteration < config.route_2opt_iterations && order.size() > 3U;
       ++iteration) {
    bool improved = false;
    for (std::size_t first = 1U; first + 2U < order.size(); ++first) {
      for (std::size_t second = first + 1U;
           second + 1U < order.size();
           ++second) {
        const double before =
            segment_cost(order[first - 1U], order[first]) +
            segment_cost(order[second], order[second + 1U]);
        const double after =
            segment_cost(order[first - 1U], order[second]) +
            segment_cost(order[first], order[second + 1U]);
        if (after + 1e-6 < before) {
          std::reverse(
              order.begin() + static_cast<std::ptrdiff_t>(first),
              order.begin() + static_cast<std::ptrdiff_t>(second + 1U));
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }

  (void)robot;
  return order;
}

}  // namespace

struct TarePolicy::Impl {
  explicit Impl(const TarePolicyConfig& config)
      : keyposes(detail::KeyposeGraphConfig{
            config.keypose_min_distance_m,
            config.keypose_connect_distance_m,
            config.max_keyposes,
            config.max_keypose_edges,
            config.max_keypose_neighbor_links}) {}

  void ResetState() {
    identity.reset();
    accepted_generation = 0U;
    coverage.clear();
    selected_goals.clear();
    keyposes.Reset();
  }

  detail::KeyposeGraph keyposes;
  std::optional<ExploreMapIdentity> identity;
  std::unordered_map<std::uint64_t, std::uint64_t> coverage;
  std::vector<Pose2D> selected_goals;
  std::uint64_t accepted_generation{0U};
  std::size_t reset_count{0U};
  ExploreDiagnostics last;
};

TarePolicy::TarePolicy(TarePolicyConfig config)
    : impl_(std::make_unique<Impl>(config)),
      config_(std::move(config)) {
  if (config_.min_frontier_size < 1 ||
      !(config_.sensor_range_m > 0.0) ||
      !(config_.candidate_radius_m > 0.0) ||
      !(config_.min_goal_distance_m >= 0.0) ||
      !(config_.novelty_radius_m >= 0.0) ||
      !(config_.local_route_radius_m > 0.0) ||
      !(config_.return_home_distance_m >= 0.0) ||
      !(config_.coverage_resolution_m > 0.0) ||
      !(config_.max_plan_time_ms > 0.0) ||
      config_.max_candidates < 1 ||
      config_.max_grid_cells == 0U ||
      config_.max_frontier_cells == 0U ||
      config_.max_frontier_clusters == 0U ||
      config_.max_coverage_cells == 0U ||
      config_.max_route_targets == 0U) {
    throw std::invalid_argument("invalid TARE policy configuration");
  }
}

TarePolicy::~TarePolicy() = default;
TarePolicy::TarePolicy(TarePolicy&&) noexcept = default;
TarePolicy& TarePolicy::operator=(TarePolicy&&) noexcept = default;

const char* TarePolicy::name() const {
  return "tare_hierarchical";
}

ExploreDecision TarePolicy::plan(const ExploreInput& input) {
  return plan(input, {});
}

ExploreDecision TarePolicy::plan(
    const ExploreInput& input,
    const ExploreCancelCheck& external_cancel) {
  const auto started = Clock::now();
  const auto deadline =
      started + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double, std::milli>(
              config_.max_plan_time_ms));
  auto cancelled = [&]() {
    return (external_cancel && external_cancel()) || Clock::now() >= deadline;
  };

  ExploreDecision decision;
  decision.diagnostics.frame_id = input.map.frame_id;
  decision.diagnostics.session_id = input.map.session_id;
  decision.diagnostics.map_id = input.map.map_id;
  decision.diagnostics.map_version = input.map.map_version;
  decision.diagnostics.reset_epoch = input.map.reset_epoch;
  decision.diagnostics.generation = input.map.generation;
  decision.diagnostics.accepted_generation = impl_->accepted_generation;
  decision.diagnostics.reset_count = impl_->reset_count;

  auto finish_without_commit = [&](std::string reason) {
    decision.reason = std::move(reason);
    decision.diagnostics.phase = "rejected";
    decision.diagnostics.planning_time_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - started).count();
    impl_->last = decision.diagnostics;
    return decision;
  };

  if (!input.map.valid()) {
    return finish_without_commit("invalid_map_identity");
  }
  if (input.map_frame.empty() || input.map_frame != input.map.frame_id) {
    return finish_without_commit("map_frame_mismatch");
  }
  if (!GridContractValid(input.exploration_grid)) {
    return finish_without_commit("invalid_grid_contract");
  }
  if (input.exploration_grid.cells.size() > config_.max_grid_cells) {
    return finish_without_commit("resource_limit_grid_cells");
  }
  if (!FinitePose(input.robot_pose) || !std::isfinite(input.stamp_s) ||
      input.stamp_s < 0.0) {
    return finish_without_commit("invalid_robot_pose_or_stamp");
  }
  if (cancelled()) {
    return finish_without_commit("cancelled");
  }

  Impl working = *impl_;
  bool reset = false;
  if (!working.identity.has_value() ||
      !working.identity->sameSource(input.map)) {
    working.ResetState();
    reset = true;
  } else if (input.map.reset_epoch < working.identity->reset_epoch) {
    return finish_without_commit("stale_reset_epoch");
  } else if (input.map.reset_epoch > working.identity->reset_epoch) {
    working.ResetState();
    reset = true;
  } else if (input.map.generation <= working.accepted_generation) {
    return finish_without_commit("stale_map_generation");
  }
  if (reset) {
    ++working.reset_count;
  }
  working.identity = input.map;

  const detail::FrontierAnalysis analysis = detail::AnalyzeFrontiers(
      input.exploration_grid,
      input.robot_pose,
      config_.min_frontier_size,
      config_.max_frontier_cells,
      config_.max_frontier_clusters,
      cancelled);
  decision.diagnostics.reachable_free_cells =
      analysis.reachable_free_cells;
  decision.diagnostics.frontier_cells = analysis.frontier_cells;
  decision.diagnostics.frontier_clusters = analysis.clusters.size();
  if (!analysis.valid) {
    return finish_without_commit(
        analysis.reason.empty() ? "frontier_analysis_failed" : analysis.reason);
  }

  std::string keypose_reason;
  if (!working.keyposes.Update(
          input.robot_pose,
          input.exploration_grid,
          input.map.generation,
          cancelled,
          &keypose_reason)) {
    return finish_without_commit(
        keypose_reason.empty() ? "keypose_update_failed" : keypose_reason);
  }

  std::vector<ExploreCandidate> candidates;
  candidates.reserve(analysis.clusters.size());
  const Grid2D& grid = input.exploration_grid;
  for (const detail::FrontierCluster& cluster : analysis.clusters) {
    const int radius_cells = std::max(
        1,
        static_cast<int>(
            std::ceil(config_.candidate_radius_m / grid.resolution)));
    bool found = false;
    ExploreCandidate best;
    for (int row = std::max(0, cluster.centroid.row - radius_cells);
         row <= std::min(
             grid.height - 1,
             cluster.centroid.row + radius_cells);
         ++row) {
      for (int col = std::max(0, cluster.centroid.col - radius_cells);
           col <= std::min(
               grid.width - 1,
               cluster.centroid.col + radius_cells);
           ++col) {
        const std::size_t position =
            static_cast<std::size_t>(grid.index(row, col));
        if (analysis.reachable[position] == 0U ||
            !std::isfinite(analysis.travel_distance_m[position]) ||
            analysis.travel_distance_m[position] <
                config_.min_goal_distance_m) {
          continue;
        }
        const auto [x, y] = detail::CellToWorld(grid, row, col);
        if (RecentlyVisited(
                x,
                y,
                input.visited_goals,
                config_.novelty_radius_m) ||
            RecentlyVisited(
                x,
                y,
                working.selected_goals,
                config_.novelty_radius_m)) {
          continue;
        }

        int visible_frontier = 0;
        int already_covered = 0;
        const double sensor_range_squared =
            config_.sensor_range_m * config_.sensor_range_m;
        for (const detail::Cell& frontier_cell : cluster.cells) {
          const auto [frontier_x, frontier_y] = detail::CellToWorld(
              grid,
              frontier_cell.row,
              frontier_cell.col);
          const double dx = frontier_x - x;
          const double dy = frontier_y - y;
          if (dx * dx + dy * dy > sensor_range_squared) {
            continue;
          }
          ++visible_frontier;
          if (working.coverage.find(CoverageKey(
                  frontier_x,
                  frontier_y,
                  config_.coverage_resolution_m)) !=
              working.coverage.end()) {
            ++already_covered;
          }
        }
        if (visible_frontier <= 0) {
          continue;
        }

        const double route_cost =
            analysis.travel_distance_m[position];
        const double heading = std::atan2(
            y - input.robot_pose.y,
            x - input.robot_pose.x);
        const double momentum =
            (1.0 + std::cos(
                AngleDelta(heading, input.robot_pose.yaw))) * 0.5;
        const double revisit_penalty =
            static_cast<double>(already_covered) /
            static_cast<double>(visible_frontier);
        const double gain =
            static_cast<double>(visible_frontier - already_covered);
        const double score =
            config_.gain_weight * gain -
            config_.travel_weight * route_cost +
            config_.momentum_weight * momentum -
            config_.revisit_weight * revisit_penalty;

        ExploreCandidate candidate;
        candidate.x = x;
        candidate.y = y;
        candidate.score = score;
        candidate.distance_m = std::hypot(
            x - input.robot_pose.x,
            y - input.robot_pose.y);
        candidate.route_cost_m = route_cost;
        candidate.frontier_size =
            static_cast<int>(cluster.cells.size());
        candidate.covered_frontier_cells = visible_frontier;
        candidate.revisit_penalty = revisit_penalty;
        candidate.cluster_id = cluster.id;
        if (!found || candidate.score > best.score) {
          best = candidate;
          found = true;
        }
      }
    }
    if (found) {
      candidates.push_back(best);
    }
    if (cancelled()) {
      return finish_without_commit("cancelled");
    }
  }

  std::sort(
      candidates.begin(),
      candidates.end(),
      [](const ExploreCandidate& lhs, const ExploreCandidate& rhs) {
        if (lhs.score != rhs.score) {
          return lhs.score > rhs.score;
        }
        if (lhs.route_cost_m != rhs.route_cost_m) {
          return lhs.route_cost_m < rhs.route_cost_m;
        }
        return lhs.cluster_id < rhs.cluster_id;
      });
  if (candidates.size() >
      static_cast<std::size_t>(config_.max_candidates)) {
    candidates.resize(static_cast<std::size_t>(config_.max_candidates));
  }
  decision.candidates = candidates;

  if (analysis.clusters.empty()) {
    const Pose2D* home = working.keyposes.Home();
    if (!config_.return_home_when_done || home == nullptr ||
        PoseDistance(input.robot_pose, *home) <=
            config_.return_home_distance_m) {
      decision.done = true;
      decision.reason = "exploration_complete";
      decision.diagnostics.phase = "complete";
    } else {
      decision.route = working.keyposes.RouteHome();
      if (decision.route.size() < 2U) {
        return finish_without_commit("return_route_unavailable");
      }
      decision.has_goal = true;
      decision.goal_x = decision.route[1U].x;
      decision.goal_y = decision.route[1U].y;
      decision.goal_z = 0.0;
      decision.reason = "returning_home";
      decision.diagnostics.phase = "return_home";
    }
  } else if (candidates.empty()) {
    decision.reason = "no_reachable_viewpoint";
    decision.diagnostics.phase = "blocked";
  } else {
    bool local_phase = false;
    const std::vector<std::size_t> order = BuildRouteOrder(
        candidates,
        input.robot_pose,
        config_,
        &local_phase);
    for (const std::size_t index : order) {
      decision.route.push_back({
          candidates[index].x,
          candidates[index].y,
          0.0,
      });
    }
    if (decision.route.empty()) {
      return finish_without_commit("route_construction_failed");
    }
    decision.has_goal = true;
    decision.goal_x = decision.route.front().x;
    decision.goal_y = decision.route.front().y;
    decision.goal_z = 0.0;
    decision.reason =
        local_phase ? "selected_local_coverage" : "selected_global_route";
    decision.diagnostics.phase =
        local_phase ? "local_coverage" : "global_route";
    working.selected_goals.push_back(decision.route.front());
    if (working.selected_goals.size() >
        config_.max_route_targets * 8U) {
      working.selected_goals.erase(
          working.selected_goals.begin(),
          working.selected_goals.begin() +
              static_cast<std::ptrdiff_t>(
                  working.selected_goals.size() -
                  config_.max_route_targets * 8U));
    }
  }

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const std::size_t position =
          static_cast<std::size_t>(grid.index(row, col));
      if (analysis.reachable[position] == 0U) {
        continue;
      }
      const auto [x, y] = detail::CellToWorld(grid, row, col);
      if (std::hypot(
              x - input.robot_pose.x,
              y - input.robot_pose.y) >
          config_.sensor_range_m) {
        continue;
      }
      const std::uint64_t key =
          CoverageKey(x, y, config_.coverage_resolution_m);
      if (working.coverage.find(key) == working.coverage.end() &&
          working.coverage.size() >= config_.max_coverage_cells) {
        return finish_without_commit("resource_limit_coverage_cells");
      }
      working.coverage[key] = input.map.generation;
    }
    if ((row & 31) == 0 && cancelled()) {
      return finish_without_commit("cancelled");
    }
  }

  working.accepted_generation = input.map.generation;
  const detail::KeyposeGraphStats graph_stats =
      working.keyposes.Stats();
  decision.diagnostics.keypose_nodes = graph_stats.nodes;
  decision.diagnostics.keypose_edges = graph_stats.edges;
  decision.diagnostics.covered_cells = working.coverage.size();
  decision.diagnostics.route_targets = decision.route.size();
  decision.diagnostics.route_length_m =
      RouteLength(input.robot_pose, decision.route);
  decision.diagnostics.reset_count = working.reset_count;
  decision.diagnostics.accepted_generation =
      input.map.generation;
  decision.diagnostics.state_committed = true;
  decision.diagnostics.planning_time_ms =
      std::chrono::duration<double, std::milli>(
          Clock::now() - started).count();
  working.last = decision.diagnostics;
  *impl_ = std::move(working);
  return decision;
}

TareDecision TarePolicy::select(
    const Grid2D& grid,
    const Pose2D& robot,
    const std::vector<Pose2D>& visited_goals) {
  ExploreInput input;
  input.exploration_grid = grid;
  input.robot_pose = robot;
  input.visited_goals = visited_goals;
  input.stamp_s = static_cast<double>(++legacy_generation_);
  input.map_frame = "map";
  input.map.frame_id = "map";
  input.map.session_id = "in_process";
  input.map.reset_epoch = 1U;
  input.map.generation = legacy_generation_;
  input.map.live = true;
  return plan(input);
}

void TarePolicy::reset() {
  impl_->ResetState();
  ++impl_->reset_count;
  legacy_generation_ = 0U;
  impl_->last = {};
  impl_->last.reset_count = impl_->reset_count;
}

ExploreDiagnostics TarePolicy::diagnostics() const {
  return impl_->last;
}

}  // namespace lingtu::explore
