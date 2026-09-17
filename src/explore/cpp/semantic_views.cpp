#include "semantic_views.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include "frontier.hpp"

namespace lingtu::explore {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaximumViews = 256U;

SemanticViewProposals Rejected(const std::string& reason) {
  SemanticViewProposals result;
  result.reason = reason;
  result.diagnostics.phase = "rejected";
  return result;
}

std::optional<ExploreCandidate> TurningView(const ExploreInput& input,
    const TarePolicyConfig& config, const ExploreCancelCheck& cancel) {
  const auto& grid = input.exploration_grid;
  detail::Cell robot_cell;
  if (!detail::WorldToCell(grid, input.robot_pose.x, input.robot_pose.y, &robot_cell) ||
      grid.at(robot_cell.row, robot_cell.col) != kFree) return std::nullopt;
  const auto& observed = *input.live_observation_grid;
  const auto bound = [&](double coordinate, double origin, int size) {
    return static_cast<int>(std::clamp(std::floor((coordinate - origin) / grid.resolution),
                                       0.0, static_cast<double>(size - 1)));
  };
  const int min_row = bound(input.robot_pose.y - config.sensor_range_m, grid.origin_y, grid.height);
  const int max_row = bound(input.robot_pose.y + config.sensor_range_m, grid.origin_y, grid.height);
  const int min_col = bound(input.robot_pose.x - config.sensor_range_m, grid.origin_x, grid.width);
  const int max_col = bound(input.robot_pose.x + config.sensor_range_m, grid.origin_x, grid.width);
  const auto columns = static_cast<std::size_t>(max_col - min_col + 1);
  const auto area = static_cast<std::size_t>(max_row - min_row + 1) * columns;
  const auto stride = std::max<std::size_t>(1U,
      (area + config.max_known_map_gain_checks_per_candidate - 1U) /
          config.max_known_map_gain_checks_per_candidate);
  std::vector<double> bearings;
  for (std::size_t flat = 0; flat < area; flat += stride) {
    if (cancel()) return std::nullopt;
    const int row = min_row + static_cast<int>(flat / columns);
    const int col = min_col + static_cast<int>(flat % columns);
    const auto i = static_cast<std::size_t>(grid.index(row, col));
    if (grid.cells[i] != kFree || observed.cells[i] == kFree) continue;
    const auto [x, y] = detail::CellToWorld(grid, row, col);
    const double dx = x - input.robot_pose.x, dy = y - input.robot_pose.y;
    if (std::hypot(dx, dy) > config.sensor_range_m || std::hypot(dx, dy) < 1e-9 ||
        !detail::HasFreeLineOfSight(grid, input.robot_pose, {x, y, 0.0})) continue;
    bearings.push_back(std::atan2(dy, dx));
  }
  if (bearings.empty()) return std::nullopt;
  std::sort(bearings.begin(), bearings.end());
  std::size_t best_count = 0, end = 0;
  double heading = input.robot_pose.yaw, best_turn = 2.0 * kPi;
  // A circular sliding window finds a useful heading, including across +/-pi.
  for (std::size_t first = 0; first < bearings.size(); ++first) {
    while (end < first + bearings.size() &&
        bearings[end % bearings.size()] + (end >= bearings.size() ? 2.0 * kPi : 0.0)
            - bearings[first] <= config.sensor_horizontal_fov_rad) ++end;
    const double last = bearings[(end - 1) % bearings.size()] +
        (end - 1 >= bearings.size() ? 2.0 * kPi : 0.0);
    const double yaw = std::remainder((bearings[first] + last) * 0.5 -
        config.sensor_yaw_offset_rad, 2.0 * kPi);
    const double turn = std::abs(std::remainder(yaw - input.robot_pose.yaw, 2.0 * kPi));
    if (end - first > best_count || (end - first == best_count && turn < best_turn)) {
      best_count = end - first;
      heading = yaw;
      best_turn = turn;
    }
  }
  ExploreCandidate candidate;
  candidate.x = input.robot_pose.x;
  candidate.y = input.robot_pose.y;
  candidate.yaw = heading;
  candidate.frontier_size = static_cast<int>(best_count);
  candidate.score = config.gain_weight * best_count +
      config.momentum_weight * (1.0 + std::cos(best_turn)) * 0.5;
  return candidate;
}
}  // namespace

SemanticViewProposals ProposeSemanticViews(const ExploreInput& input,
                                    const std::vector<double>& reference_heights_m,
                                    const SemanticSearchHistory& history,
                                    TarePolicyConfig config,
                                    const ExploreCancelCheck& cancel) {
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  const auto elapsed_ms = [&] {
    return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  };
  const auto cancelled = [&] {
    return (cancel && cancel()) || elapsed_ms() >= config.max_plan_time_ms;
  };
  if (!input.map.valid() || input.map.live || input.map_frame != input.map.frame_id) {
    return Rejected("semantic_search_requires_saved_map");
  }
  if (!history.map.valid() || !history.map.sameSource(input.map) ||
      history.map.reset_epoch != input.map.reset_epoch) {
    return Rejected("semantic_search_history_map_mismatch");
  }
  const Grid2D& source = input.exploration_grid;
  if (source.width <= 0 || source.height <= 0 || !std::isfinite(source.resolution) ||
      source.resolution <= 0.0 || !std::isfinite(source.origin_x) ||
      !std::isfinite(source.origin_y) ||
      static_cast<std::size_t>(source.width) > config.max_grid_cells /
                                                static_cast<std::size_t>(source.height) ||
      source.cells.size() != static_cast<std::size_t>(source.width) * source.height ||
      reference_heights_m.size() != source.cells.size()) {
    return Rejected("invalid_semantic_search_grid");
  }
  if (history.views.size() > kMaximumViews) {
    return Rejected("semantic_search_view_limit");
  }

  ExploreInput query = input;
  query.visited_goals.clear();
  // Unsupported cells must not connect otherwise disconnected viewpoints.
  for (std::size_t i = 0; i < source.cells.size(); ++i) {
    if (!std::isfinite(reference_heights_m[i]) && query.exploration_grid.cells[i] == kFree) {
      query.exploration_grid.cells[i] = kUnknown;
    }
  }
  Grid2D observation = query.exploration_grid;
  std::fill(observation.cells.begin(), observation.cells.end(), kUnknown);
  for (const CameraSearchView& view : history.views) {
    if (!std::isfinite(view.pose.x) || !std::isfinite(view.pose.y) ||
        !std::isfinite(view.pose.yaw) || !std::isfinite(view.range_m) || view.range_m <= 0.0 ||
        !std::isfinite(view.horizontal_fov_rad) || view.horizontal_fov_rad <= 0.0 ||
        view.horizontal_fov_rad > 2.0 * kPi) {
      return Rejected("invalid_camera_search_view");
    }
    detail::Cell view_cell;
    if (!detail::WorldToCell(query.exploration_grid, view.pose.x, view.pose.y, &view_cell) ||
        query.exploration_grid.at(view_cell.row, view_cell.col) != kFree) {
      continue;
    }
    // Coverage is directional. A camera pose must not blacklist its location.
    for (int row = 0; row < source.height; ++row) {
      if (cancelled()) {
        return Rejected("cancelled");
      }
      const double y = source.origin_y + (row + 0.5) * source.resolution;
      if (std::abs(y - view.pose.y) > view.range_m) {
        continue;
      }
      for (int col = 0; col < source.width; ++col) {
        const auto index = static_cast<std::size_t>(source.index(row, col));
        if (query.exploration_grid.cells[index] != kFree || observation.cells[index] == kFree) {
          continue;
        }
        const auto [x, cell_y] = detail::CellToWorld(source, row, col);
        const double dx = x - view.pose.x;
        const double dy = cell_y - view.pose.y;
        if (std::hypot(dx, dy) > view.range_m ||
            std::abs(std::remainder(std::atan2(dy, dx) - view.pose.yaw, 2.0 * kPi)) >
                view.horizontal_fov_rad * 0.5 ||
            !detail::HasFreeLineOfSight(query.exploration_grid, view.pose, {x, cell_y, 0.0})) {
          continue;
        }
        observation.cells[index] = kFree;
      }
    }
  }
  if (cancelled()) {
    return Rejected("cancelled");
  }
  query.live_observation_grid = std::move(observation);
  config.return_home_when_done = false;
  // Validate policy bounds before using them in the bounded heading search.
  TarePolicy policy(config);
  const auto turning = TurningView(query, config, cancelled);
  if (cancelled()) return Rejected("cancelled");
  auto decision = policy.plan(query, cancelled);
  if (!decision.diagnostics.state_committed) {
    return Rejected(decision.reason);
  }
  SemanticViewProposals result;
  result.map = input.map;
  result.candidates = std::move(decision.candidates);
  result.diagnostics = decision.diagnostics;
  result.reason = decision.reason;
  result.geometry_exhausted = decision.done;
  if (turning && !result.geometry_exhausted) {
    result.candidates.push_back(*turning);
    std::stable_sort(result.candidates.begin(), result.candidates.end(),
        [](const auto& a, const auto& b) { return a.score > b.score; });
    if (result.candidates.size() > static_cast<std::size_t>(config.max_candidates))
      result.candidates.resize(config.max_candidates);
    if (!decision.has_goal) {
      result.reason = "selected_camera_turn";
      result.diagnostics.phase = "camera_turn";
    }
  }
  for (auto& candidate : result.candidates) {
    detail::Cell cell;
    if (detail::WorldToCell(source, candidate.x, candidate.y, &cell)) {
      candidate.z = reference_heights_m[static_cast<std::size_t>(source.index(cell.row, cell.col))];
    }
  }
  // Geometry can be exhausted without finding the requested semantic object.
  if (result.geometry_exhausted) {
    result.reason = "camera_search_geometry_covered";
  }
  result.diagnostics.state_committed = false;
  result.diagnostics.planning_time_ms = elapsed_ms();
  return result;
}
}  // namespace lingtu::explore
