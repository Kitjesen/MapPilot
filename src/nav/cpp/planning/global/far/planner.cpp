#include "planning/global/far/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

namespace lingtu::nav::plan::far_planner {
namespace {

constexpr double kEpsilon = 1e-9;

bool Finite(double value) {
  return std::isfinite(value);
}

double Distance2D(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

double Distance3D(const GlobalPlanPoint& lhs, const GlobalPlanPoint& rhs) {
  return std::hypot(std::hypot(lhs.x - rhs.x, lhs.y - rhs.y), lhs.z - rhs.z);
}

bool SameIdentity(const MapIdentity& lhs, const MapIdentity& rhs) {
  return lhs.map_id == rhs.map_id && lhs.content_epoch == rhs.content_epoch &&
      lhs.frame_id == rhs.frame_id;
}

void CopyOverlayIdentity(const GlobalPlanRequest& request, GlobalPlanResult* result) {
  result->overlay_revision = request.temporary_overlay.revision;
  result->overlay_frame_epoch = request.temporary_overlay.frame_epoch;
  result->overlay_obstacle_generation = request.temporary_overlay.obstacle_generation;
  result->overlay_traversability_generation =
      request.temporary_overlay.traversability_generation;
}

}  // namespace

std::size_t FarGridMap::CellCount() const noexcept {
  if (width <= 0 || height <= 0) {
    return 0U;
  }
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

std::size_t FarGridMap::Index(std::int32_t x, std::int32_t y) const {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    throw std::out_of_range("FAR grid cell is outside map bounds");
  }
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(x);
}

void FarGridMap::Validate() const {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("FAR map width and height must be positive");
  }
  const auto count64 = static_cast<std::uint64_t>(width) *
      static_cast<std::uint64_t>(height);
  if (count64 > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("FAR map exceeds the 32-bit graph cell index contract");
  }
  if (cells.size() != CellCount()) {
    throw std::invalid_argument("FAR map cell count does not match width * height");
  }
  if (!Finite(resolution_m) || resolution_m <= 0.0 ||
      !Finite(origin_x_m) || !Finite(origin_y_m)) {
    throw std::invalid_argument("FAR map geometry must be finite and resolution must be positive");
  }
  if (frame_id.empty()) {
    throw std::invalid_argument("FAR map frame_id is required");
  }
  if (generation == 0U) {
    throw std::invalid_argument("FAR map generation must be positive");
  }
  for (const auto cell : cells) {
    if (cell != kUnknownCell && cell != kFreeCell && cell != kOccupiedCell) {
      throw std::invalid_argument(
          "FAR map accepts only trinary cells: -1 unknown, 0 free, 100 occupied");
    }
  }
  if (identity.valid() && identity.frame_id != frame_id) {
    throw std::invalid_argument("FAR map identity frame does not match map frame");
  }
}

void FarPlanner::ValidateConfig(const FarPlannerConfig& config) {
  if (!Finite(config.robot_radius_m) || config.robot_radius_m < 0.0 ||
      !Finite(config.obstacle_clearance_m) || config.obstacle_clearance_m < 0.0 ||
      !Finite(config.max_visibility_distance_m) || config.max_visibility_distance_m <= 0.0 ||
      !Finite(config.unknown_cost_multiplier) || config.unknown_cost_multiplier < 1.0) {
    throw std::invalid_argument("FAR distance and cost settings are invalid");
  }
  if (config.corner_separation_cells < 1 || config.snap_search_radius_cells < 0) {
    throw std::invalid_argument("FAR cell-radius settings are invalid");
  }
  if (config.max_graph_nodes < 2U || config.max_visibility_pairs == 0U ||
      config.max_search_expansions == 0U) {
    throw std::invalid_argument("FAR graph and search limits must be positive");
  }
}

FarPlanner::FarPlanner(FarPlannerConfig config) : config_(std::move(config)) {
  ValidateConfig(config_);
}

std::uint64_t FarPlanner::EdgeKey(std::uint32_t lhs, std::uint32_t rhs) {
  const auto low = std::min(lhs, rhs);
  const auto high = std::max(lhs, rhs);
  return (static_cast<std::uint64_t>(low) << 32U) | static_cast<std::uint64_t>(high);
}

bool FarPlanner::SameGeometry(const FarGridMap& lhs, const FarGridMap& rhs) const {
  return lhs.width == rhs.width && lhs.height == rhs.height &&
      std::abs(lhs.resolution_m - rhs.resolution_m) <= kEpsilon &&
      std::abs(lhs.origin_x_m - rhs.origin_x_m) <= kEpsilon &&
      std::abs(lhs.origin_y_m - rhs.origin_y_m) <= kEpsilon &&
      lhs.frame_id == rhs.frame_id;
}

void FarPlanner::UpdateMap(FarGridMap map) {
  map.Validate();
  if (has_map_ && map.generation < map_.generation) {
    throw std::invalid_argument("FAR rejected stale map generation");
  }
  if (has_map_ && map.generation == map_.generation) {
    const bool identical = SameGeometry(map_, map) && map_.cells == map.cells &&
        SameIdentity(map_.identity, map.identity);
    if (!identical) {
      throw std::invalid_argument("FAR rejected different map content with the same generation");
    }
    diagnostics_.map_update_mode = "noop_same_generation";
    diagnostics_.changed_cells = 0U;
    return;
  }

  const bool full_rebuild = !has_map_ || !SameGeometry(map_, map);
  FarPlanner staged(config_);
  staged.map_ = std::move(map);
  staged.has_map_ = true;
  staged.inflated_blocked_ = staged.BuildInflatedMask(staged.map_);

  std::vector<std::uint8_t> changed(staged.map_.CellCount(), 1U);
  if (!full_rebuild) {
    changed.assign(staged.map_.CellCount(), 0U);
    for (std::size_t index = 0U; index < changed.size(); ++index) {
      if (map_.cells[index] != staged.map_.cells[index] ||
          inflated_blocked_[index] != staged.inflated_blocked_[index]) {
        changed[index] = 1U;
      }
    }
    staged.edge_cache_ = edge_cache_;
  }

  staged.diagnostics_.map_generation = staged.map_.generation;
  staged.diagnostics_.map_frame_id = staged.map_.frame_id;
  staged.diagnostics_.map_update_mode = full_rebuild ? "full" : "incremental";
  staged.diagnostics_.changed_cells = static_cast<std::size_t>(
      std::count(changed.begin(), changed.end(), static_cast<std::uint8_t>(1U)));
  staged.RebuildVisibilityGraph(changed, full_rebuild);

  map_ = std::move(staged.map_);
  has_map_ = true;
  inflated_blocked_ = std::move(staged.inflated_blocked_);
  nodes_ = std::move(staged.nodes_);
  edges_ = std::move(staged.edges_);
  adjacency_ = std::move(staged.adjacency_);
  edge_cache_ = std::move(staged.edge_cache_);
  diagnostics_ = std::move(staged.diagnostics_);
}

bool FarPlanner::WorldToCell(double x_m, double y_m, CellCoord* cell) const {
  if (cell == nullptr || !Finite(x_m) || !Finite(y_m) || !has_map_) {
    return false;
  }
  const auto x = static_cast<std::int64_t>(
      std::floor((x_m - map_.origin_x_m) / map_.resolution_m));
  const auto y = static_cast<std::int64_t>(
      std::floor((y_m - map_.origin_y_m) / map_.resolution_m));
  if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) {
    return false;
  }
  cell->x = static_cast<std::int32_t>(x);
  cell->y = static_cast<std::int32_t>(y);
  return true;
}

GlobalPlanPoint FarPlanner::CellCenter(const CellCoord& cell, double z_m) const {
  return {
      map_.origin_x_m + (static_cast<double>(cell.x) + 0.5) * map_.resolution_m,
      map_.origin_y_m + (static_cast<double>(cell.y) + 0.5) * map_.resolution_m,
      z_m,
  };
}

bool FarPlanner::IsCellBlocked(const CellCoord& cell) const {
  return inflated_blocked_[map_.Index(cell.x, cell.y)] != 0U;
}

bool FarPlanner::IsCellFree(const CellCoord& cell) const {
  const auto index = map_.Index(cell.x, cell.y);
  return inflated_blocked_[index] == 0U && map_.cells[index] == kFreeCell;
}

bool FarPlanner::IsCellUnknown(const CellCoord& cell) const {
  const auto index = map_.Index(cell.x, cell.y);
  return inflated_blocked_[index] == 0U && map_.cells[index] == kUnknownCell;
}

FarPlanner::EndpointCell FarPlanner::SnapEndpoint(
    double x_m,
    double y_m,
    bool allow_unknown) const {
  CellCoord origin{};
  if (!WorldToCell(x_m, y_m, &origin)) {
    throw std::out_of_range("FAR endpoint is outside the planning map");
  }
  if (IsCellFree(origin) || (allow_unknown && IsCellUnknown(origin))) {
    return {origin, false};
  }

  bool have_free = false;
  bool have_unknown = false;
  CellCoord best_free{};
  CellCoord best_unknown{};
  double best_free_score = std::numeric_limits<double>::infinity();
  double best_unknown_score = std::numeric_limits<double>::infinity();
  for (std::int32_t dy = -config_.snap_search_radius_cells;
       dy <= config_.snap_search_radius_cells;
       ++dy) {
    for (std::int32_t dx = -config_.snap_search_radius_cells;
         dx <= config_.snap_search_radius_cells;
         ++dx) {
      CellCoord candidate{origin.x + dx, origin.y + dy};
      if (candidate.x < 0 || candidate.y < 0 ||
          candidate.x >= map_.width || candidate.y >= map_.height) {
        continue;
      }
      const double score = static_cast<double>(dx) * dx + static_cast<double>(dy) * dy;
      if (IsCellFree(candidate) && score < best_free_score) {
        best_free = candidate;
        best_free_score = score;
        have_free = true;
      } else if (allow_unknown && IsCellUnknown(candidate) && score < best_unknown_score) {
        best_unknown = candidate;
        best_unknown_score = score;
        have_unknown = true;
      }
    }
  }
  if (have_free) {
    return {best_free, true};
  }
  if (have_unknown) {
    return {best_unknown, true};
  }
  throw std::runtime_error("FAR found no traversable endpoint within snap radius");
}

std::vector<std::uint8_t> FarPlanner::BuildInflatedMask(const FarGridMap& map) const {
  std::vector<std::uint8_t> blocked(map.CellCount(), 0U);
  const double clearance = config_.robot_radius_m + config_.obstacle_clearance_m;
  const auto radius = static_cast<std::int32_t>(std::ceil(clearance / map.resolution_m));
  const double radius_sq = clearance * clearance + kEpsilon;
  for (std::int32_t y = 0; y < map.height; ++y) {
    for (std::int32_t x = 0; x < map.width; ++x) {
      if (map.cells[map.Index(x, y)] != kOccupiedCell) {
        continue;
      }
      for (std::int32_t dy = -radius; dy <= radius; ++dy) {
        for (std::int32_t dx = -radius; dx <= radius; ++dx) {
          const auto nx = x + dx;
          const auto ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
            continue;
          }
          const double metric_dx = static_cast<double>(dx) * map.resolution_m;
          const double metric_dy = static_cast<double>(dy) * map.resolution_m;
          if (metric_dx * metric_dx + metric_dy * metric_dy <= radius_sq) {
            blocked[map.Index(nx, ny)] = 1U;
          }
        }
      }
    }
  }
  return blocked;
}

std::vector<FarPlanner::GraphNode> FarPlanner::ExtractCornerNodes() const {
  std::vector<GraphNode> nodes;
  const auto blocked = [&](std::int32_t x, std::int32_t y) {
    if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) {
      return true;
    }
    return inflated_blocked_[map_.Index(x, y)] != 0U;
  };
  const auto known_free = [&](std::int32_t x, std::int32_t y) {
    if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) {
      return false;
    }
    const auto index = map_.Index(x, y);
    return inflated_blocked_[index] == 0U && map_.cells[index] == kFreeCell;
  };

  const auto min_dist_sq = config_.corner_separation_cells * config_.corner_separation_cells;
  for (std::int32_t y = 1; y + 1 < map_.height; ++y) {
    for (std::int32_t x = 1; x + 1 < map_.width; ++x) {
      if (!known_free(x, y)) {
        continue;
      }
      const bool corner =
          (blocked(x - 1, y - 1) && known_free(x - 1, y) && known_free(x, y - 1)) ||
          (blocked(x + 1, y - 1) && known_free(x + 1, y) && known_free(x, y - 1)) ||
          (blocked(x - 1, y + 1) && known_free(x - 1, y) && known_free(x, y + 1)) ||
          (blocked(x + 1, y + 1) && known_free(x + 1, y) && known_free(x, y + 1));
      if (!corner) {
        continue;
      }
      bool separated = true;
      for (const auto& node : nodes) {
        const auto dx = node.cell.x - x;
        const auto dy = node.cell.y - y;
        if (dx * dx + dy * dy < min_dist_sq) {
          separated = false;
          break;
        }
      }
      if (!separated) {
        continue;
      }
      if (nodes.size() >= config_.max_graph_nodes) {
        throw std::length_error("FAR visibility node limit exceeded");
      }
      const CellCoord cell{x, y};
      const auto point = CellCenter(cell, 0.0);
      nodes.push_back({
          static_cast<std::uint32_t>(map_.Index(x, y)),
          cell,
          point.x,
          point.y,
      });
    }
  }
  return nodes;
}

std::vector<std::uint32_t> FarPlanner::RasterLine(
    const CellCoord& from,
    const CellCoord& to) const {
  std::vector<std::uint32_t> cells;
  auto add = [&](std::int32_t x, std::int32_t y) {
    if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) {
      return;
    }
    cells.push_back(static_cast<std::uint32_t>(map_.Index(x, y)));
  };

  std::int32_t x = from.x;
  std::int32_t y = from.y;
  const auto dx = std::abs(to.x - from.x);
  const auto dy = std::abs(to.y - from.y);
  const auto sx = to.x > from.x ? 1 : (to.x < from.x ? -1 : 0);
  const auto sy = to.y > from.y ? 1 : (to.y < from.y ? -1 : 0);
  std::int32_t ix = 0;
  std::int32_t iy = 0;
  add(x, y);
  while (ix < dx || iy < dy) {
    const auto decision = (1 + 2 * ix) * dy - (1 + 2 * iy) * dx;
    if (decision == 0) {
      add(x + sx, y);
      add(x, y + sy);
      x += sx;
      y += sy;
      ++ix;
      ++iy;
    } else if (decision < 0) {
      x += sx;
      ++ix;
    } else {
      y += sy;
      ++iy;
    }
    add(x, y);
  }
  std::sort(cells.begin(), cells.end());
  cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
  return cells;
}

FarPlanner::VisibilityEdge FarPlanner::EvaluateEdge(
    std::uint32_t from,
    std::uint32_t to,
    const GraphNode& lhs,
    const GraphNode& rhs) const {
  VisibilityEdge edge;
  edge.from = from;
  edge.to = to;
  edge.length_m = Distance2D(lhs.x_m, lhs.y_m, rhs.x_m, rhs.y_m);
  edge.raster_cells = RasterLine(lhs.cell, rhs.cell);
  for (const auto cell_index : edge.raster_cells) {
    if (inflated_blocked_[cell_index] != 0U) {
      edge.occupied_blocked = true;
    }
    if (map_.cells[cell_index] == kUnknownCell) {
      ++edge.unknown_cells;
    }
  }
  return edge;
}

bool FarPlanner::EdgeTouchesChanged(
    const VisibilityEdge& edge,
    const std::vector<std::uint8_t>& changed) const {
  for (const auto index : edge.raster_cells) {
    if (index < changed.size() && changed[index] != 0U) {
      return true;
    }
  }
  return false;
}

void FarPlanner::RebuildVisibilityGraph(
    const std::vector<std::uint8_t>& changed,
    bool full_rebuild) {
  nodes_ = ExtractCornerNodes();
  edges_.clear();
  adjacency_.assign(nodes_.size(), {});
  std::unordered_map<std::uint64_t, VisibilityEdge> next_cache;
  if (!full_rebuild) {
    next_cache.reserve(edge_cache_.size());
  }

  std::size_t considered = 0U;
  std::size_t reused = 0U;
  std::size_t recomputed = 0U;
  for (std::uint32_t lhs = 0U; lhs < nodes_.size(); ++lhs) {
    for (std::uint32_t rhs = lhs + 1U; rhs < nodes_.size(); ++rhs) {
      if (Distance2D(
              nodes_[lhs].x_m,
              nodes_[lhs].y_m,
              nodes_[rhs].x_m,
              nodes_[rhs].y_m) > config_.max_visibility_distance_m) {
        continue;
      }
      ++considered;
      if (considered > config_.max_visibility_pairs) {
        throw std::length_error("FAR visibility pair limit exceeded");
      }
      const auto key = EdgeKey(nodes_[lhs].cell_index, nodes_[rhs].cell_index);
      VisibilityEdge edge;
      const auto previous = edge_cache_.find(key);
      if (!full_rebuild && previous != edge_cache_.end() &&
          !EdgeTouchesChanged(previous->second, changed)) {
        edge = previous->second;
        edge.from = lhs;
        edge.to = rhs;
        ++reused;
      } else {
        edge = EvaluateEdge(lhs, rhs, nodes_[lhs], nodes_[rhs]);
        ++recomputed;
      }
      next_cache.emplace(key, edge);
      const auto edge_index = static_cast<std::uint32_t>(edges_.size());
      edges_.push_back(std::move(edge));
      if (!edges_.back().occupied_blocked) {
        adjacency_[lhs].push_back({rhs, edge_index});
        adjacency_[rhs].push_back({lhs, edge_index});
      }
    }
  }
  edge_cache_ = std::move(next_cache);
  diagnostics_.graph_nodes = nodes_.size();
  diagnostics_.visibility_pairs = considered;
  diagnostics_.reusable_edges = reused;
  diagnostics_.recomputed_edges = recomputed;
}

bool FarPlanner::EdgeUsable(const VisibilityEdge& edge, bool allow_unknown) const {
  return !edge.occupied_blocked && (allow_unknown || edge.unknown_cells == 0U);
}

double FarPlanner::EdgeCost(const VisibilityEdge& edge, bool allow_unknown) const {
  if (!allow_unknown || edge.unknown_cells == 0U || edge.raster_cells.empty()) {
    return edge.length_m;
  }
  const double ratio = static_cast<double>(edge.unknown_cells) /
      static_cast<double>(edge.raster_cells.size());
  return edge.length_m *
      (1.0 + ratio * (config_.unknown_cost_multiplier - 1.0));
}

FarPlanner::SearchResult FarPlanner::Search(
    const std::vector<GraphNode>& nodes,
    const std::vector<VisibilityEdge>& edges,
    const std::vector<std::vector<Arc>>& adjacency,
    std::uint32_t start_index,
    std::uint32_t goal_index,
    bool allow_unknown,
    std::size_t max_expansions,
    const GlobalPlanCancelCheck& cancel) const {
  struct QueueEntry {
    double score{0.0};
    std::uint32_t node{0U};
    bool operator>(const QueueEntry& other) const {
      if (std::abs(score - other.score) > kEpsilon) {
        return score > other.score;
      }
      return node > other.node;
    }
  };

  SearchResult result;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> gscore(nodes.size(), inf);
  std::vector<std::uint32_t> parent(nodes.size(), std::numeric_limits<std::uint32_t>::max());
  std::vector<std::uint8_t> closed(nodes.size(), 0U);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  gscore[start_index] = 0.0;
  open.push({
      Distance2D(
          nodes[start_index].x_m,
          nodes[start_index].y_m,
          nodes[goal_index].x_m,
          nodes[goal_index].y_m),
      start_index,
  });

  while (!open.empty()) {
    if (cancel && cancel()) {
      result.cancelled = true;
      return result;
    }
    const auto current = open.top().node;
    open.pop();
    if (closed[current] != 0U) {
      continue;
    }
    closed[current] = 1U;
    ++result.expansions;
    if (result.expansions > max_expansions) {
      return result;
    }
    if (current == goal_index) {
      result.found = true;
      break;
    }
    for (const auto& arc : adjacency[current]) {
      const auto& edge = edges[arc.edge_index];
      if (!EdgeUsable(edge, allow_unknown) || closed[arc.to] != 0U) {
        continue;
      }
      const double tentative = gscore[current] + EdgeCost(edge, allow_unknown);
      if (tentative + kEpsilon >= gscore[arc.to]) {
        continue;
      }
      gscore[arc.to] = tentative;
      parent[arc.to] = current;
      const double heuristic = Distance2D(
          nodes[arc.to].x_m,
          nodes[arc.to].y_m,
          nodes[goal_index].x_m,
          nodes[goal_index].y_m);
      open.push({tentative + heuristic, arc.to});
    }
  }
  if (!result.found) {
    return result;
  }
  for (std::uint32_t node = goal_index;; node = parent[node]) {
    result.node_indices.push_back(node);
    if (node == start_index) {
      break;
    }
    if (parent[node] == std::numeric_limits<std::uint32_t>::max()) {
      result.found = false;
      result.node_indices.clear();
      return result;
    }
  }
  std::reverse(result.node_indices.begin(), result.node_indices.end());
  return result;
}

std::vector<std::uint32_t> FarPlanner::SimplifyNodePath(
    const std::vector<std::uint32_t>& path,
    const std::vector<GraphNode>& nodes,
    bool allow_unknown) const {
  if (!config_.simplify_path || path.size() <= 2U) {
    return path;
  }
  std::vector<std::uint32_t> simplified;
  simplified.push_back(path.front());
  std::size_t anchor = 0U;
  while (anchor + 1U < path.size()) {
    std::size_t selected = anchor + 1U;
    for (std::size_t candidate = path.size() - 1U; candidate > anchor; --candidate) {
      const auto edge = EvaluateEdge(
          path[anchor], path[candidate], nodes[path[anchor]], nodes[path[candidate]]);
      if (EdgeUsable(edge, allow_unknown)) {
        selected = candidate;
        break;
      }
    }
    simplified.push_back(path[selected]);
    anchor = selected;
  }
  return simplified;
}

std::size_t FarPlanner::CountUnknownOnPath(
    const std::vector<std::uint32_t>& path,
    const std::vector<GraphNode>& nodes) const {
  std::unordered_set<std::uint32_t> unknown;
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto cells = RasterLine(nodes[path[index - 1U]].cell, nodes[path[index]].cell);
    for (const auto cell : cells) {
      if (map_.cells[cell] == kUnknownCell) {
        unknown.insert(cell);
      }
    }
  }
  return unknown.size();
}

GlobalPlanResult FarPlanner::FailureResult(
    const GlobalPlanRequest& request,
    std::string reason,
    bool cancelled,
    double elapsed_ms) {
  diagnostics_.failure_reason = reason;
  diagnostics_.cancelled = cancelled;
  diagnostics_.elapsed_ms = elapsed_ms;
  GlobalPlanResult result;
  result.cancelled = cancelled;
  result.failure_reason = std::move(reason);
  result.elapsed_ms = elapsed_ms;
  result.options = request.options;
  result.map_identity = map_.identity;
  result.map_generation = map_.generation;
  CopyOverlayIdentity(request, &result);
  return result;
}

GlobalPlanResult FarPlanner::Plan(
    const GlobalPlanRequest& request,
    const GlobalPlanCancelCheck& cancel) {
  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  };
  const auto update_mode = diagnostics_.map_update_mode;
  const auto changed_cells = diagnostics_.changed_cells;
  const auto reusable_edges = diagnostics_.reusable_edges;
  const auto recomputed_edges = diagnostics_.recomputed_edges;
  const auto visibility_pairs = diagnostics_.visibility_pairs;
  diagnostics_ = {};
  diagnostics_.map_generation = map_.generation;
  diagnostics_.map_frame_id = map_.frame_id;
  diagnostics_.map_update_mode = update_mode;
  diagnostics_.changed_cells = changed_cells;
  diagnostics_.graph_nodes = nodes_.size();
  diagnostics_.visibility_pairs = visibility_pairs;
  diagnostics_.reusable_edges = reusable_edges;
  diagnostics_.recomputed_edges = recomputed_edges;

  if (!has_map_) {
    return FailureResult(request, "far_map_unavailable", false, elapsed_ms());
  }
  if (!Finite(request.start.x) || !Finite(request.start.y) || !Finite(request.start.z) ||
      !Finite(request.goal.x) || !Finite(request.goal.y) || !Finite(request.goal.z)) {
    return FailureResult(request, "far_non_finite_request", false, elapsed_ms());
  }
  if (request.map_generation != 0U && request.map_generation != map_.generation) {
    return FailureResult(request, "far_stale_map_generation", false, elapsed_ms());
  }
  if (request.map_identity.valid() &&
      (!map_.identity.valid() || !sameMapIdentity(request.map_identity, map_.identity))) {
    return FailureResult(request, "far_map_identity_mismatch", false, elapsed_ms());
  }
  if (!request.temporary_overlay.empty()) {
    return FailureResult(
        request, "far_temporary_overlay_unsupported", false, elapsed_ms());
  }

  EndpointCell start{};
  try {
    start = SnapEndpoint(request.start.x, request.start.y, false);
  } catch (const std::exception& exc) {
    return FailureResult(
        request, std::string("far_invalid_start: ") + exc.what(), false, elapsed_ms());
  }
  diagnostics_.start_snapped = start.snapped;

  struct Attempt {
    bool found{false};
    bool cancelled{false};
    bool allow_unknown{false};
    EndpointCell goal{};
    std::size_t expansions{0U};
    std::vector<GraphNode> nodes;
    std::vector<std::uint32_t> path;
  };

  const auto run_attempt = [&](bool allow_unknown) {
    Attempt attempt;
    attempt.allow_unknown = allow_unknown;
    try {
      attempt.goal = SnapEndpoint(request.goal.x, request.goal.y, allow_unknown);
    } catch (const std::exception&) {
      return attempt;
    }
    attempt.nodes = nodes_;
    const auto start_point = CellCenter(start.cell, request.start.z);
    const auto goal_point = CellCenter(attempt.goal.cell, request.goal.z);
    const auto start_index = static_cast<std::uint32_t>(attempt.nodes.size());
    attempt.nodes.push_back({
        static_cast<std::uint32_t>(map_.Index(start.cell.x, start.cell.y)),
        start.cell,
        start_point.x,
        start_point.y,
    });
    const auto goal_index = static_cast<std::uint32_t>(attempt.nodes.size());
    attempt.nodes.push_back({
        static_cast<std::uint32_t>(map_.Index(attempt.goal.cell.x, attempt.goal.cell.y)),
        attempt.goal.cell,
        goal_point.x,
        goal_point.y,
    });

    auto edges = edges_;
    auto adjacency = adjacency_;
    adjacency.resize(attempt.nodes.size());
    const auto connect_dynamic = [&](std::uint32_t dynamic_index) {
      for (std::uint32_t other = 0U; other < dynamic_index; ++other) {
        const bool start_goal_pair =
            dynamic_index == goal_index && other == start_index;
        if (!start_goal_pair && Distance2D(
                attempt.nodes[dynamic_index].x_m,
                attempt.nodes[dynamic_index].y_m,
                attempt.nodes[other].x_m,
                attempt.nodes[other].y_m) > config_.max_visibility_distance_m) {
          continue;
        }
        auto edge = EvaluateEdge(
            other, dynamic_index, attempt.nodes[other], attempt.nodes[dynamic_index]);
        const auto edge_index = static_cast<std::uint32_t>(edges.size());
        edges.push_back(std::move(edge));
        if (!edges.back().occupied_blocked) {
          adjacency[other].push_back({dynamic_index, edge_index});
          adjacency[dynamic_index].push_back({other, edge_index});
        }
      }
    };
    connect_dynamic(start_index);
    connect_dynamic(goal_index);

    const std::size_t request_limit = request.options.max_iterations > 0
        ? static_cast<std::size_t>(request.options.max_iterations)
        : config_.max_search_expansions;
    const auto search = Search(
        attempt.nodes,
        edges,
        adjacency,
        start_index,
        goal_index,
        allow_unknown,
        std::min(request_limit, config_.max_search_expansions),
        cancel);
    attempt.cancelled = search.cancelled;
    attempt.expansions = search.expansions;
    if (search.found) {
      attempt.found = true;
      attempt.path = SimplifyNodePath(search.node_indices, attempt.nodes, allow_unknown);
    }
    return attempt;
  };

  Attempt selected = run_attempt(false);
  diagnostics_.search_expansions += selected.expansions;
  if (selected.cancelled) {
    return FailureResult(request, "far_cancelled", true, elapsed_ms());
  }

  const auto known_goal_point = selected.found
      ? CellCenter(selected.goal.cell, request.goal.z)
      : GlobalPlanPoint{};
  const bool known_reaches_requested = selected.found &&
      Distance3D(known_goal_point, request.goal) <= request.options.terminal_goal_tolerance_m;
  if ((!selected.found || !known_reaches_requested) && config_.allow_unknown_fallback) {
    auto fallback = run_attempt(true);
    diagnostics_.search_expansions += fallback.expansions;
    if (fallback.cancelled) {
      return FailureResult(request, "far_cancelled", true, elapsed_ms());
    }
    if (fallback.found) {
      selected = std::move(fallback);
    }
  }
  if (!selected.found) {
    diagnostics_.planning_phase = config_.allow_unknown_fallback
        ? "unknown_fallback_failed"
        : "known_free_failed";
    return FailureResult(request, "far_no_path", false, elapsed_ms());
  }

  diagnostics_.planning_phase = selected.allow_unknown
      ? "unknown_fallback"
      : "known_free";
  diagnostics_.used_unknown_space = selected.allow_unknown;
  diagnostics_.goal_snapped = selected.goal.snapped;
  diagnostics_.raw_path_points = selected.path.size();
  diagnostics_.unknown_cells_traversed =
      CountUnknownOnPath(selected.path, selected.nodes);

  GlobalPlanResult result;
  result.ok = true;
  result.cancelled = false;
  result.options = request.options;
  result.map_identity = map_.identity;
  result.map_generation = map_.generation;
  CopyOverlayIdentity(request, &result);
  result.elapsed_ms = elapsed_ms();
  const auto snapped_goal = CellCenter(selected.goal.cell, request.goal.z);
  result.goal_error_m = Distance3D(snapped_goal, request.goal);
  result.goal_xy_error_m = Distance2D(
      snapped_goal.x, snapped_goal.y, request.goal.x, request.goal.y);
  result.goal_z_error_m = std::abs(snapped_goal.z - request.goal.z);
  result.reached_goal =
      result.goal_error_m <= request.options.terminal_goal_tolerance_m &&
      result.goal_xy_error_m <= request.options.terminal_goal_xy_tolerance_m &&
      result.goal_z_error_m <= request.options.terminal_goal_z_tolerance_m;

  result.path.reserve(selected.path.size());
  for (std::size_t index = 0U; index < selected.path.size(); ++index) {
    const auto& node = selected.nodes[selected.path[index]];
    GlobalPlanPoint point{node.x_m, node.y_m, request.start.z};
    if (index == 0U && !start.snapped) {
      point = request.start;
    }
    if (index + 1U == selected.path.size()) {
      point.z = request.goal.z;
      if (!selected.goal.snapped) {
        point = request.goal;
      }
    }
    result.path.push_back(point);
  }
  diagnostics_.output_path_points = result.path.size();
  diagnostics_.elapsed_ms = result.elapsed_ms;
  return result;
}

}  // namespace lingtu::nav::plan::far_planner
