#include "plan/rolling_map_segment_executor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lingtu::nav::endpoint {
namespace {

constexpr double kEpsilon = 1e-9;

struct Cell {
  int row{0};
  int col{0};
};

struct SearchNode {
  double priority{0.0};
  double score{0.0};
  double travel_m{0.0};
  int index{0};
  std::size_t steps{0U};
};

struct HigherPriority {
  bool operator()(const SearchNode &left, const SearchNode &right) const {
    return left.priority > right.priority;
  }
};

bool finiteCoordinate(double value, double limit) {
  return std::isfinite(value) && std::abs(value) <= limit;
}

bool validPose(const lingtu::explore::Pose2D &pose, double coordinate_limit) {
  return finiteCoordinate(pose.x, coordinate_limit) && finiteCoordinate(pose.y, coordinate_limit) &&
         std::isfinite(pose.yaw);
}

bool validTarget(const lingtu::explore::DirectedTarget &target, double coordinate_limit) {
  return finiteCoordinate(target.x, coordinate_limit) &&
         finiteCoordinate(target.y, coordinate_limit);
}

bool gridCellCount(int width, int height, std::size_t maximum, std::size_t *result) {
  if (width <= 0 || height <= 0 || maximum == 0U) {
    return false;
  }
  const auto wide = static_cast<std::size_t>(width);
  const auto high = static_cast<std::size_t>(height);
  if (wide > maximum / high) {
    return false;
  }
  if (result != nullptr) {
    *result = wide * high;
  }
  return true;
}

bool sameGeometry(const lingtu::explore::Grid2D &occupancy, const TerrainCostGrid &terrain) {
  return occupancy.width == terrain.width && occupancy.height == terrain.height &&
         occupancy.resolution == terrain.resolution && occupancy.origin_x == terrain.origin_x &&
         occupancy.origin_y == terrain.origin_y;
}

bool sameEpoch(const lingtu::explore::ExploreMapIdentity &left,
               const lingtu::explore::ExploreMapIdentity &right) {
  return left.sameSource(right) && left.reset_epoch == right.reset_epoch;
}

bool freshStamp(double stamp_s, double now_s, const RollingMapSegmentExecutorConfig &config) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || !std::isfinite(now_s) || now_s <= 0.0) {
    return false;
  }
  const double age_s = now_s - stamp_s;
  return age_s >= -config.future_tolerance_s && age_s <= config.snapshot_max_age_s;
}

bool validOccupancy(const lingtu::explore::Grid2D &grid,
                    const RollingMapSegmentExecutorConfig &config, std::size_t *cell_count) {
  std::size_t count = 0U;
  if (!gridCellCount(grid.width, grid.height, config.max_grid_cells, &count) ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0.0 || !std::isfinite(grid.origin_x) ||
      !std::isfinite(grid.origin_y) || grid.cells.size() != count) {
    return false;
  }
  for (const auto value : grid.cells) {
    if (value != lingtu::explore::kFree && value != lingtu::explore::kOccupied &&
        value != lingtu::explore::kUnknown) {
      return false;
    }
  }
  if (cell_count != nullptr) {
    *cell_count = count;
  }
  return true;
}

bool validTerrain(const TerrainCostGrid &terrain, std::size_t expected_cell_count,
                  const RollingMapSegmentExecutorConfig &config) {
  if (!std::isfinite(terrain.resolution) || terrain.resolution <= 0.0 ||
      !std::isfinite(terrain.origin_x) || !std::isfinite(terrain.origin_y) ||
      terrain.costs.size() != expected_cell_count) {
    return false;
  }
  for (const float cost : terrain.costs) {
    if (!std::isfinite(cost) || cost < 0.0F || cost > config.max_terrain_cost) {
      return false;
    }
  }
  return true;
}

std::string inputError(const RollingMapSegmentInput &input,
                       const RollingMapSegmentExecutorConfig &config) {
  if (!input.input_ready) {
    return "segment_inputs_not_ready";
  }
  if (!std::isfinite(input.now_s) || input.now_s <= 0.0) {
    return "segment_clock_invalid";
  }
  if (!input.snapshot.identity.valid() || !input.snapshot.identity.live ||
      input.snapshot.identity.frame_id != "map") {
    return "rolling_map_identity_invalid";
  }
  if (!freshStamp(input.snapshot.stamp_s, input.now_s, config)) {
    return "rolling_map_snapshot_stale";
  }
  if (!input.snapshot.terrain_risk_ready) {
    return "terrain_risk_not_ready";
  }
  if (!freshStamp(input.snapshot.terrain_risk_stamp_s, input.now_s, config)) {
    return "terrain_risk_stale";
  }
  std::size_t cell_count = 0U;
  if (!validOccupancy(input.snapshot.occupancy, config, &cell_count)) {
    return "rolling_map_occupancy_invalid";
  }
  if (!sameGeometry(input.snapshot.occupancy, input.snapshot.terrain_cost) ||
      !validTerrain(input.snapshot.terrain_cost, cell_count, config)) {
    return "terrain_cost_grid_invalid";
  }
  if (!validPose(input.robot_pose, config.max_coordinate_m)) {
    return "robot_pose_invalid";
  }
  return {};
}

std::optional<Cell> cellFor(const lingtu::explore::Grid2D &grid, double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }
  const double col_value = std::floor((x - grid.origin_x) / grid.resolution);
  const double row_value = std::floor((y - grid.origin_y) / grid.resolution);
  if (!std::isfinite(row_value) || !std::isfinite(col_value) || row_value < 0.0 ||
      row_value >= static_cast<double>(grid.height) || col_value < 0.0 ||
      col_value >= static_cast<double>(grid.width)) {
    return std::nullopt;
  }
  const int row = static_cast<int>(row_value);
  const int col = static_cast<int>(col_value);
  return Cell{row, col};
}

int indexFor(const lingtu::explore::Grid2D &grid, Cell cell) {
  return grid.index(cell.row, cell.col);
}

Cell cellAtIndex(const lingtu::explore::Grid2D &grid, int index) {
  return {index / grid.width, index % grid.width};
}

bool safeObservedFree(const RollingMapSegmentSnapshot &snapshot,
                      const RollingMapSegmentExecutorConfig &config, Cell cell) {
  if (cell.row < 0 || cell.row >= snapshot.occupancy.height || cell.col < 0 ||
      cell.col >= snapshot.occupancy.width) {
    return false;
  }
  const auto index = static_cast<std::size_t>(indexFor(snapshot.occupancy, cell));
  return snapshot.occupancy.cells[index] == lingtu::explore::kFree &&
         snapshot.terrain_cost.costs[index] <= config.terrain_risk_threshold;
}

bool safeGridTransition(const RollingMapSegmentSnapshot &snapshot,
                        const RollingMapSegmentExecutorConfig &config, Cell from, Cell to) {
  const int row_delta = to.row - from.row;
  const int col_delta = to.col - from.col;
  if (std::abs(row_delta) > 1 || std::abs(col_delta) > 1 ||
      !safeObservedFree(snapshot, config, from) || !safeObservedFree(snapshot, config, to)) {
    return false;
  }
  if (row_delta == 0 || col_delta == 0) {
    return true;
  }
  return safeObservedFree(snapshot, config, {from.row + row_delta, from.col}) &&
         safeObservedFree(snapshot, config, {from.row, from.col + col_delta});
}

lingtu::explore::Pose2D cellCenter(const lingtu::explore::Grid2D &grid, Cell cell) {
  return {
      grid.origin_x + (static_cast<double>(cell.col) + 0.5) * grid.resolution,
      grid.origin_y + (static_cast<double>(cell.row) + 0.5) * grid.resolution,
      0.0,
  };
}

double targetDistance(double x, double y, const lingtu::explore::DirectedTarget &target) {
  return std::hypot(target.x - x, target.y - y);
}

bool betterCandidate(double candidate_target_distance, double candidate_score,
                     double candidate_travel, double best_target_distance, double best_score,
                     double best_travel) {
  if (candidate_target_distance + kEpsilon < best_target_distance) {
    return true;
  }
  if (std::abs(candidate_target_distance - best_target_distance) > kEpsilon) {
    return false;
  }
  if (candidate_score + kEpsilon < best_score) {
    return true;
  }
  if (std::abs(candidate_score - best_score) > kEpsilon) {
    return false;
  }
  return candidate_travel + kEpsilon < best_travel;
}

RollingMapSegmentDecision rejected(std::string reason) {
  RollingMapSegmentDecision decision;
  decision.action = RollingMapSegmentAction::Rejected;
  decision.reason = std::move(reason);
  return decision;
}

RollingMapSegmentDecision noop(std::string reason,
                               const lingtu::explore::ExploreMapIdentity *identity = nullptr,
                               std::uint64_t generation = 0U) {
  RollingMapSegmentDecision decision;
  decision.action = RollingMapSegmentAction::Noop;
  decision.reason = std::move(reason);
  if (identity != nullptr) {
    decision.executed_map = *identity;
    decision.executed_generation = generation;
  }
  return decision;
}

bool pathSafe(const std::vector<lingtu::explore::Pose2D> &path,
              const RollingMapSegmentSnapshot &snapshot,
              const RollingMapSegmentExecutorConfig &config) {
  if (path.empty()) {
    return false;
  }
  const auto safe_cell_at = [&](double x, double y) -> std::optional<Cell> {
    const auto cell = cellFor(snapshot.occupancy, x, y);
    if (!cell.has_value() || !safeObservedFree(snapshot, config, *cell)) {
      return std::nullopt;
    }
    return cell;
  };
  for (const auto &waypoint : path) {
    if (!safe_cell_at(waypoint.x, waypoint.y).has_value()) {
      return false;
    }
  }

  const double step_m = std::max(1e-6, snapshot.occupancy.resolution * 0.5);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto &first = path[index - 1U];
    const auto &second = path[index];
    const auto first_cell = safe_cell_at(first.x, first.y);
    if (!first_cell.has_value()) {
      return false;
    }
    Cell previous_cell = *first_cell;
    const double length = std::hypot(second.x - first.x, second.y - first.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const auto current_cell = safe_cell_at(first.x + (second.x - first.x) * ratio,
                                             first.y + (second.y - first.y) * ratio);
      if (!current_cell.has_value() ||
          !safeGridTransition(snapshot, config, previous_cell, *current_cell)) {
        return false;
      }
      previous_cell = *current_cell;
    }
  }
  return true;
}

// Drop only path points which the current robot pose has already passed along
// the ordered segment. This keeps a rolling map from invalidating a safe
// forward suffix merely because old cells have fallen behind the robot.
bool trimConsumedPrefix(std::vector<lingtu::explore::Pose2D> &path,
                        const lingtu::explore::Pose2D &robot) {
  if (path.empty()) {
    return false;
  }
  while (path.size() > 1U) {
    const auto &from = path[0U];
    const auto &next = path[1U];
    const double dx = next.x - from.x;
    const double dy = next.y - from.y;
    const double span_squared = dx * dx + dy * dy;
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(span_squared)) {
      return false;
    }
    if (span_squared <= kEpsilon) {
      path.erase(path.begin());
      continue;
    }
    const double rx = robot.x - from.x;
    const double ry = robot.y - from.y;
    const double progress = rx * dx + ry * dy;
    if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(progress)) {
      return false;
    }
    if (progress + kEpsilon < span_squared) {
      break;
    }
    path.erase(path.begin());
  }
  path.front() = robot;
  return true;
}

}  // namespace

RollingMapSegmentExecutor::RollingMapSegmentExecutor(RollingMapSegmentExecutorConfig config)
    : config_(std::move(config)) {
  if (config_.max_grid_cells == 0U || !std::isfinite(config_.max_segment_length_m) ||
      config_.max_segment_length_m <= 0.0 || config_.max_waypoints < 2U ||
      !std::isfinite(config_.terrain_risk_threshold) || config_.terrain_risk_threshold < 0.0F ||
      !std::isfinite(config_.max_terrain_cost) ||
      config_.max_terrain_cost < config_.terrain_risk_threshold ||
      !std::isfinite(config_.snapshot_max_age_s) || config_.snapshot_max_age_s < 0.0 ||
      !std::isfinite(config_.future_tolerance_s) || config_.future_tolerance_s < 0.0 ||
      !std::isfinite(config_.max_coordinate_m) || config_.max_coordinate_m <= 0.0) {
    throw std::invalid_argument("rolling map segment executor configuration is invalid");
  }
}

RollingMapSegmentDecision RollingMapSegmentExecutor::plan(const RollingMapSegmentInput &input,
                                                          const RollingMapSegmentRequest &request) {
  const std::string invalid = inputError(input, config_);
  if (!invalid.empty()) {
    return active_ ? cancelActive(invalid) : rejected(invalid);
  }
  if (last_seen_.has_value() && last_seen_->sameSource(input.snapshot.identity)) {
    if (input.snapshot.identity.reset_epoch < last_seen_->reset_epoch) {
      return active_ ? cancelActive("stale_reset_epoch") : rejected("stale_reset_epoch");
    }
    if (input.snapshot.identity.reset_epoch == last_seen_->reset_epoch &&
        input.snapshot.identity.generation < last_seen_->generation) {
      return active_ ? cancelActive("stale_map_generation") : rejected("stale_map_generation");
    }
  }

  if (!last_seen_.has_value() || !last_seen_->sameSource(input.snapshot.identity) ||
      input.snapshot.identity.reset_epoch > last_seen_->reset_epoch ||
      (input.snapshot.identity.reset_epoch == last_seen_->reset_epoch &&
       input.snapshot.identity.generation > last_seen_->generation)) {
    last_seen_ = input.snapshot.identity;
  }
  if (active_.has_value()) {
    if (!sameEpoch(active_->identity, input.snapshot.identity)) {
      last_seen_ = input.snapshot.identity;
      return cancelActive("segment_map_epoch_changed");
    }
    if (input.snapshot.identity.generation < active_->generation) {
      return cancelActive("segment_map_generation_regressed");
    }
  }
  if (!validTarget(request.target, config_.max_coordinate_m)) {
    return rejected("segment_target_invalid");
  }

  if (input.snapshot.identity.generation < request.requested_generation) {
    last_seen_ = input.snapshot.identity;
    return active_ ? noop("awaiting_requested_generation", &active_->identity, active_->generation)
                   : noop("awaiting_requested_generation");
  }

  const auto start = cellFor(input.snapshot.occupancy, input.robot_pose.x, input.robot_pose.y);
  if (!start.has_value()) {
    return rejected("robot_outside_rolling_map");
  }
  if (!safeObservedFree(input.snapshot, config_, *start)) {
    return rejected("robot_not_observed_free_or_safe");
  }

  const double initial_distance =
      targetDistance(input.robot_pose.x, input.robot_pose.y, request.target);
  if (initial_distance <= kEpsilon) {
    last_seen_ = input.snapshot.identity;
    return active_ ? noop("target_already_reached", &active_->identity, active_->generation)
                   : noop("target_already_reached");
  }

  const auto start_center = cellCenter(input.snapshot.occupancy, *start);
  const double start_offset_m =
      std::hypot(input.robot_pose.x - start_center.x, input.robot_pose.y - start_center.y);
  const auto cell_count = input.snapshot.occupancy.cells.size();
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> scores(cell_count, infinity);
  std::vector<double> travels(cell_count, infinity);
  std::vector<int> previous(cell_count, -1);
  std::vector<std::size_t> steps(cell_count, std::numeric_limits<std::size_t>::max());
  std::priority_queue<SearchNode, std::vector<SearchNode>, HigherPriority> open;

  const int start_index = indexFor(input.snapshot.occupancy, *start);
  scores[static_cast<std::size_t>(start_index)] = 0.0;
  travels[static_cast<std::size_t>(start_index)] = 0.0;
  steps[static_cast<std::size_t>(start_index)] = 0U;
  open.push({initial_distance, 0.0, 0.0, start_index, 0U});

  int best_index = -1;
  double best_target_distance = infinity;
  double best_score = infinity;
  double best_travel = infinity;
  constexpr int neighbors[8][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1},
  };

  while (!open.empty()) {
    const SearchNode current = open.top();
    open.pop();
    const auto current_index = static_cast<std::size_t>(current.index);
    if (current.score > scores[current_index] + kEpsilon ||
        current.travel_m > travels[current_index] + kEpsilon ||
        current.steps != steps[current_index]) {
      continue;
    }

    const Cell current_cell = cellAtIndex(input.snapshot.occupancy, current.index);
    if (current.index != start_index) {
      const auto center = cellCenter(input.snapshot.occupancy, current_cell);
      const double distance = targetDistance(center.x, center.y, request.target);
      if (distance + kEpsilon < initial_distance &&
          betterCandidate(distance, current.score, current.travel_m, best_target_distance,
                          best_score, best_travel)) {
        best_index = current.index;
        best_target_distance = distance;
        best_score = current.score;
        best_travel = current.travel_m;
      }
    }

    if (current.steps + 1U >= config_.max_waypoints) {
      continue;
    }
    for (const auto &offset : neighbors) {
      const Cell next{
          current_cell.row + offset[0],
          current_cell.col + offset[1],
      };
      if (!safeGridTransition(input.snapshot, config_, current_cell, next)) {
        continue;
      }

      const double step_m = input.snapshot.occupancy.resolution *
                            (offset[0] == 0 || offset[1] == 0 ? 1.0 : std::sqrt(2.0));
      const double next_travel = current.travel_m + step_m;
      if (start_offset_m + next_travel > config_.max_segment_length_m + kEpsilon) {
        continue;
      }
      const int next_index = indexFor(input.snapshot.occupancy, next);
      const auto next_unsigned = static_cast<std::size_t>(next_index);
      const float terrain_cost = input.snapshot.terrain_cost.costs[next_unsigned];
      const double next_score =
          current.score +
          step_m * (1.0 + static_cast<double>(terrain_cost) /
                              std::max(1.0, static_cast<double>(config_.max_terrain_cost)));
      const std::size_t next_steps = current.steps + 1U;
      const bool lower_score = next_score + kEpsilon < scores[next_unsigned];
      const bool equal_score_shorter = std::abs(next_score - scores[next_unsigned]) <= kEpsilon &&
                                       next_travel + kEpsilon < travels[next_unsigned];
      if (!lower_score && !equal_score_shorter) {
        continue;
      }

      scores[next_unsigned] = next_score;
      travels[next_unsigned] = next_travel;
      steps[next_unsigned] = next_steps;
      previous[next_unsigned] = current.index;
      const auto center = cellCenter(input.snapshot.occupancy, next);
      open.push({
          next_score + targetDistance(center.x, center.y, request.target),
          next_score,
          next_travel,
          next_index,
          next_steps,
      });
    }
  }

  if (best_index < 0) {
    return rejected("no_safe_observed_free_prefix");
  }

  std::vector<int> reverse_path;
  for (int index = best_index; index >= 0; index = previous[static_cast<std::size_t>(index)]) {
    reverse_path.push_back(index);
    if (index == start_index) {
      break;
    }
  }
  if (reverse_path.empty() || reverse_path.back() != start_index) {
    return rejected("segment_path_reconstruction_failed");
  }
  std::reverse(reverse_path.begin(), reverse_path.end());

  std::vector<lingtu::explore::Pose2D> path;
  path.reserve(reverse_path.size());
  path.push_back(input.robot_pose);
  for (std::size_t index = 1U; index < reverse_path.size(); ++index) {
    path.push_back(cellCenter(input.snapshot.occupancy,
                              cellAtIndex(input.snapshot.occupancy, reverse_path[index])));
  }
  if (path.size() > config_.max_waypoints || !pathSafe(path, input.snapshot, config_)) {
    return rejected("segment_path_safety_validation_failed");
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    path[index - 1U].yaw =
        std::atan2(path[index].y - path[index - 1U].y, path[index].x - path[index - 1U].x);
  }
  if (path.size() > 1U) {
    path.back().yaw = path[path.size() - 2U].yaw;
  }

  active_ = ActiveSegment{
      input.snapshot.identity,
      input.snapshot.identity.generation,
      path,
  };
  last_seen_ = input.snapshot.identity;

  RollingMapSegmentDecision decision;
  decision.action = RollingMapSegmentAction::Accepted;
  decision.reason = "safe_observed_free_prefix";
  decision.executed_map = input.snapshot.identity;
  decision.executed_generation = input.snapshot.identity.generation;
  decision.path = std::move(path);
  return decision;
}

RollingMapSegmentDecision
RollingMapSegmentExecutor::revalidate(const RollingMapSegmentInput &input) {
  if (!active_.has_value()) {
    return noop("no_active_segment");
  }
  const std::string invalid = inputError(input, config_);
  if (!invalid.empty()) {
    return cancelActive(invalid);
  }
  if (!sameEpoch(active_->identity, input.snapshot.identity)) {
    last_seen_ = input.snapshot.identity;
    return cancelActive("segment_map_epoch_changed");
  }
  if (input.snapshot.identity.generation < active_->generation ||
      (last_seen_.has_value() && last_seen_->sameSource(input.snapshot.identity) &&
       input.snapshot.identity.reset_epoch == last_seen_->reset_epoch &&
       input.snapshot.identity.generation < last_seen_->generation)) {
    return cancelActive("segment_map_generation_regressed");
  }

  if (!trimConsumedPrefix(active_->path, input.robot_pose) ||
      !pathSafe(active_->path, input.snapshot, config_)) {
    return cancelActive("segment_path_no_longer_safe");
  }
  if (!last_seen_.has_value() || !last_seen_->sameSource(input.snapshot.identity) ||
      input.snapshot.identity.reset_epoch > last_seen_->reset_epoch ||
      (input.snapshot.identity.reset_epoch == last_seen_->reset_epoch &&
       input.snapshot.identity.generation > last_seen_->generation)) {
    last_seen_ = input.snapshot.identity;
  }
  return noop("segment_revalidated", &active_->identity, active_->generation);
}

void RollingMapSegmentExecutor::reset() noexcept {
  active_.reset();
  last_seen_.reset();
}

bool RollingMapSegmentExecutor::active() const noexcept {
  return active_.has_value();
}

RollingMapSegmentDecision RollingMapSegmentExecutor::cancelActive(std::string reason) {
  RollingMapSegmentDecision decision;
  decision.action = RollingMapSegmentAction::Cancel;
  decision.reason = std::move(reason);
  if (active_.has_value()) {
    decision.executed_map = active_->identity;
    decision.executed_generation = active_->generation;
  }
  active_.reset();
  return decision;
}

}  // namespace lingtu::nav::endpoint
