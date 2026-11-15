#include "global_planner.h"
#include "mls_backend.hpp"
#include "surface_graph.hpp"

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <memory>
#include <set>
#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct Options {
  std::string map;
  // Match the production surface preparation: support height plus one map cell.
  double headroom{0.45};
  double closing{0.20};
  double max_hole_area{0.16};
  double max_slope{0.70};
  bool has_start{false};
  bool has_goal{false};
  global_planner::PointPose start{};
  global_planner::PointPose goal{};
};

constexpr double kSupportHeight = 0.35;

struct OcTreeRegistry : octomap::OcTree {
  static void ensure() { ocTreeMemberInit.ensureLinking(); }
};

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::runtime_error("missing value for " + arg);
      return argv[i];
    };
    if (arg == "--map") options.map = value();
    else if (arg == "--headroom") options.headroom = std::stod(value());
    else if (arg == "--closing") options.closing = std::stod(value());
    else if (arg == "--max-hole-area") options.max_hole_area = std::stod(value());
    else if (arg == "--max-slope") options.max_slope = std::stod(value());
    else if (arg == "--start") {
      options.start = {std::stod(value()), 0.0, 0.0, std::nullopt};
      options.start.y = std::stod(value());
      options.start.z = std::stod(value());
      options.has_start = true;
    } else if (arg == "--goal") {
      options.goal = {std::stod(value()), 0.0, 0.0, std::nullopt};
      options.goal.y = std::stod(value());
      options.goal.z = std::stod(value());
      options.has_goal = true;
    }
    else throw std::runtime_error("unknown argument: " + arg);
  }
  if (options.map.empty()) throw std::runtime_error("--map is required");
  if (options.has_start != options.has_goal) {
    throw std::runtime_error("--start and --goal must be provided together");
  }
  return options;
}

std::shared_ptr<octomap::OcTree> loadMap(const std::string& path) {
  OcTreeRegistry::ensure();
  std::ifstream header_stream(path, std::ios::binary);
  std::string header;
  const bool binary_header = static_cast<bool>(std::getline(header_stream, header)) &&
      header == "# Octomap OcTree binary file";
  auto binary = std::make_shared<octomap::OcTree>(0.2);
  if (binary_header && binary->readBinary(path)) return binary;
  std::unique_ptr<octomap::AbstractOcTree> abstract(octomap::AbstractOcTree::read(path));
  if (!abstract) throw std::runtime_error("cannot read OctoMap: " + path);
  auto* raw = dynamic_cast<octomap::OcTree*>(abstract.release());
  if (!raw) throw std::runtime_error("map is not an OcTree: " + path);
  return std::shared_ptr<octomap::OcTree>(raw);
}

std::vector<global_planner::mls::SurfaceCell> occupiedCells(const octomap::OcTree& tree) {
  const double resolution = tree.getResolution();
  std::vector<global_planner::mls::SurfaceCell> cells;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it)) continue;
    const int span = static_cast<int>(std::lround(it.getSize() / resolution));
    const int x0 = static_cast<int>(std::lround((it.getX() - .5 * it.getSize()) / resolution));
    const int y0 = static_cast<int>(std::lround((it.getY() - .5 * it.getSize()) / resolution));
    const int z0 = static_cast<int>(std::lround((it.getZ() - .5 * it.getSize()) / resolution));
    for (int x = x0; x < x0 + span; ++x)
      for (int y = y0; y < y0 + span; ++y)
        for (int z = z0; z < z0 + span; ++z)
          cells.push_back({x, y, z});
  }
  return cells;
}

std::vector<global_planner::mls::SurfaceCell> freeCells(const octomap::OcTree& tree) {
  const double resolution = tree.getResolution();
  std::vector<global_planner::mls::SurfaceCell> cells;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (tree.isNodeOccupied(*it)) continue;
    const int span = static_cast<int>(std::lround(it.getSize() / resolution));
    const int x0 = static_cast<int>(std::lround((it.getX() - .5 * it.getSize()) / resolution));
    const int y0 = static_cast<int>(std::lround((it.getY() - .5 * it.getSize()) / resolution));
    const int z0 = static_cast<int>(std::lround((it.getZ() - .5 * it.getSize()) / resolution));
    for (int x = x0; x < x0 + span; ++x)
      for (int y = y0; y < y0 + span; ++y)
        for (int z = z0; z < z0 + span; ++z)
          cells.push_back({x, y, z});
  }
  return cells;
}

std::vector<global_planner::PointPose> worldSurface(
    const std::vector<global_planner::mls::SurfaceCell>& cells,
    double resolution, double headroom) {
  std::vector<global_planner::PointPose> points;
  points.reserve(cells.size());
  for (const auto& cell : cells) {
    points.push_back({(cell.x + .5) * resolution, (cell.y + .5) * resolution,
                      (cell.z + .5) * resolution + headroom, std::nullopt});
  }
  return points;
}

struct GraphResult {
  std::size_t nodes{0};
  std::size_t edges{0};
  std::size_t components{0};
  bool start_attached{false};
  bool goal_attached{false};
  double route_length{0.0};
  std::size_t route_points{0};
  std::vector<global_planner::PointPose> route;
};

struct LayerStats {
  std::size_t xy_columns{0};
  std::size_t multi_level_columns{0};
  std::size_t max_z_levels{0};
};

LayerStats layerStats(const std::vector<global_planner::mls::SurfaceCell>& cells) {
  std::map<std::pair<int, int>, std::set<int>> levels;
  for (const auto& cell : cells) levels[{cell.x, cell.y}].insert(cell.z);
  LayerStats result;
  result.xy_columns = levels.size();
  for (const auto& [xy, zs] : levels) {
    (void)xy;
    result.max_z_levels = std::max(result.max_z_levels, zs.size());
    if (zs.size() > 1) ++result.multi_level_columns;
  }
  return result;
}

using CellSet = std::set<std::array<int, 3>>;

CellSet cellSet(const std::vector<global_planner::mls::SurfaceCell>& cells) {
  CellSet result;
  for (const auto& cell : cells) result.insert({cell.x, cell.y, cell.z});
  return result;
}

void printCellSet(const char* name, const CellSet& cells) {
  std::cout << "\"" << name << "\":[";
  std::size_t index = 0;
  for (const auto& cell : cells) {
    if (index++) std::cout << ",";
    std::cout << "[" << cell[0] << "," << cell[1] << "," << cell[2] << "]";
  }
  std::cout << "]";
}

void printRejectionCounts(const std::vector<lingtu::maps::layers::SurfaceRejection>& rejections) {
  std::map<std::string, std::size_t> counts;
  for (const auto& rejection : rejections) ++counts[rejection.reason];
  std::cout << "\"rejection_reason_counts\":{";
  std::size_t index = 0;
  for (const auto& [reason, count] : counts) {
    if (index++) std::cout << ",";
    std::cout << "\"" << reason << "\":" << count;
  }
  std::cout << "}";
}

GraphResult makeGraph(const std::vector<global_planner::PointPose>& points,
                      double resolution,
                      const global_planner::PointPose& start,
                      const global_planner::PointPose& goal) {
  global_planner::PlannerConfig config;
  config.search_resolution_m = .10;
  config.max_slope = .70;
  config.max_step_height = .45;
  config.max_iterations = 200000;
  global_planner::SurfaceGraph graph(
      points, resolution, config, [](const auto&) { return true; },
      [](const auto&, const auto&) { return true; });
  GraphResult result;
  result.nodes = graph.nodeCount();
  result.edges = graph.edgeCount();
  result.components = graph.componentCount();
  if (points.size() < 2) return result;
  const auto route = graph.plan(start, goal,
      [](const auto&, const auto&) { return true; });
  result.route = route;
  result.route_points = route.size();
  result.start_attached = !route.empty();
  result.goal_attached = !route.empty();
  for (std::size_t i = 1; i < route.size(); ++i) {
    result.route_length += std::hypot(
        route[i].x - route[i - 1].x,
        std::hypot(route[i].y - route[i - 1].y, route[i].z - route[i - 1].z));
  }
  return result;
}

const char* failureName(global_planner::OctoPlanner3D::TraversabilityFailure failure) {
  using Failure = global_planner::OctoPlanner3D::TraversabilityFailure;
  switch (failure) {
    case Failure::None: return "none";
    case Failure::OutsideBounds: return "outside_bounds";
    case Failure::UnknownSpace: return "unknown_space";
    case Failure::GroundSupport: return "ground_support";
    case Failure::ExternalPreblockedBelow: return "preblocked_below";
    case Failure::ExternalPreblockedBody: return "preblocked_body";
    case Failure::OccupiedBody: return "occupied_body";
  }
  return "unknown";
}

global_planner::OctoPlanner3D::RouteValidation validateRustRoute(
    const std::shared_ptr<octomap::OcTree>& tree,
    const global_planner::PointPose& start,
    const global_planner::PointPose& goal,
    const std::vector<global_planner::PointPose>& route) {
  global_planner::PlannerConfig config;
  config.surface_closing_radius_m = .20;
  config.surface_max_hole_area_m2 = .16;
  config.surface_max_slope = .70;
  config.ground_support = true;
  config.support_height_m = .35;
  config.support_height_tolerance_m = .05;
  config.max_step_height = .45;
  config.max_slope = .70;
  config.max_iterations = 200000;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(tree);
  planner.makePlan(start, goal);
  return planner.validateRoute(route);
}

void printRouteValidation(const char* name,
                          const global_planner::OctoPlanner3D::RouteValidation& result) {
  std::cout << "\"" << name << "\":{";
  std::cout << "\"valid\":" << (result.valid ? "true" : "false")
            << ",\"metric_limits\":" << (result.metric_limits ? "true" : "false")
            << ",\"support\":" << (result.support ? "true" : "false")
            << ",\"body\":" << (result.body ? "true" : "false")
            << ",\"sweep\":" << (result.sweep ? "true" : "false")
            << ",\"segments\":" << result.segments
            << ",\"failed_segment\":";
  if (result.failed_segment == std::numeric_limits<std::size_t>::max())
    std::cout << "null";
  else
    std::cout << result.failed_segment;
  std::cout << ",\"failure\":\"" << failureName(result.failure) << "\"}";
}

void printGraph(const char* name, const GraphResult& graph) {
  std::cout << "\"" << name << "\":{";
  std::cout << "\"nodes\":" << graph.nodes << ",\"edges\":" << graph.edges
            << ",\"components\":" << graph.components
            << ",\"start_attached\":" << (graph.start_attached ? "true" : "false")
            << ",\"goal_attached\":" << (graph.goal_attached ? "true" : "false")
            << ",\"route_points\":" << graph.route_points
            << ",\"route_length_m\":" << graph.route_length << "}";
}

void printRustGraph(const global_planner::mls::RustGraphSummary& graph) {
  std::cout << "\"rust_native\":{";
  std::cout << "\"nodes\":" << graph.nodes << ",\"edges\":" << graph.edges
            << ",\"components\":" << graph.components
            << ",\"corridor_cells\":" << graph.corridor_cells
            << ",\"max_corridor_cells\":" << graph.max_corridor_cells << "}";
}

std::vector<global_planner::PointPose> rustEdgeRoute(
    const global_planner::mls::Edge& edge, double resolution) {
  std::vector<global_planner::PointPose> route;
  route.reserve(edge.corridor.size());
  for (const auto& cell : edge.corridor) {
    route.push_back({(cell.x + .5) * resolution,
                     (cell.y + .5) * resolution,
                     (cell.z + .5) * resolution + kSupportHeight,
                     std::nullopt});
  }
  return route;
}

global_planner::OctoPlanner3D::RouteValidation validateRustEdge(
    global_planner::OctoPlanner3D& planner,
    const global_planner::mls::Edge& edge,
    double resolution) {
  return planner.validateRoute(rustEdgeRoute(edge, resolution));
}

std::optional<global_planner::mls::Edge> repairRustEdge(
    global_planner::OctoPlanner3D& planner,
    const global_planner::mls::Edge& edge,
    double resolution) {
  if (edge.corridor.size() < 2) return std::nullopt;
  const auto route = rustEdgeRoute(edge, resolution);
  const std::size_t count = route.size();
  std::vector<std::size_t> next(count, count);
  next.back() = count - 1;
  for (std::size_t cursor = count - 1; cursor > 0; --cursor) {
    const std::size_t from = cursor - 1;
    for (std::size_t to = count - 1; to > from; --to) {
      if (next[to] == count) continue;
      if (!planner.validateRoute({route[from], route[to]}).valid) continue;
      next[from] = to;
      break;
    }
  }
  if (next.front() == count) return std::nullopt;
  global_planner::mls::Edge repaired = edge;
  repaired.corridor.clear();
  for (std::size_t index = 0; ; index = next[index]) {
    repaired.corridor.push_back(edge.corridor[index]);
    if (index == count - 1) break;
  }
  return repaired;
}

global_planner::mls::RustGraph filterRustGraph(
    const std::shared_ptr<octomap::OcTree>& tree,
    const global_planner::PointPose& start,
    const global_planner::PointPose& goal,
    double resolution,
    const global_planner::mls::RustGraph& source,
    std::size_t* repaired_edges,
    std::size_t* rejected_edges) {
  global_planner::PlannerConfig config;
  config.surface_closing_radius_m = .20;
  config.surface_max_hole_area_m2 = .16;
  config.surface_max_slope = .70;
  config.ground_support = true;
  config.support_height_m = kSupportHeight;
  config.support_height_tolerance_m = .05;
  config.max_step_height = .45;
  config.max_slope = .70;
  config.max_iterations = 200000;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(tree);
  planner.makePlan(start, goal);
  global_planner::mls::RustGraph filtered;
  filtered.nodes = source.nodes;
  if (repaired_edges) *repaired_edges = 0;
  if (rejected_edges) *rejected_edges = 0;
  for (const auto& edge : source.edges) {
    const auto validation = validateRustEdge(planner, edge, resolution);
    if (validation.valid) {
      filtered.edges.push_back(edge);
      continue;
    }
    if (auto repaired = repairRustEdge(planner, edge, resolution)) {
      if (repaired_edges) ++*repaired_edges;
      filtered.edges.push_back(std::move(*repaired));
    } else if (rejected_edges) {
      ++*rejected_edges;
    }
  }
  return filtered;
}

std::vector<global_planner::PointPose> planRustGraph(
    const global_planner::mls::RustGraph& graph,
    double resolution,
    const global_planner::PointPose& start,
    const global_planner::PointPose& goal) {
  if (graph.nodes.empty()) return {};
  const auto world = [resolution](const global_planner::mls::SurfaceCell& cell) {
    return global_planner::PointPose{
        (cell.x + .5) * resolution,
        (cell.y + .5) * resolution,
        (cell.z + .5) * resolution + kSupportHeight,
        std::nullopt};
  };
  std::size_t start_node = 0;
  std::size_t goal_node = 0;
  double start_distance = std::numeric_limits<double>::infinity();
  double goal_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
    const auto point = world(graph.nodes[i].cell);
    const auto distance_to = [&](const global_planner::PointPose& target) {
      return std::hypot(point.x - target.x,
                        std::hypot(point.y - target.y, point.z - target.z));
    };
    if (distance_to(start) < start_distance) {
      start_distance = distance_to(start);
      start_node = i;
    }
    if (distance_to(goal) < goal_distance) {
      goal_distance = distance_to(goal);
      goal_node = i;
    }
  }
  using QueueItem = std::pair<double, std::size_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  std::vector<double> distances(graph.nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(graph.nodes.size(), std::numeric_limits<std::size_t>::max());
  std::vector<std::size_t> predecessor_edge(graph.nodes.size(), std::numeric_limits<std::size_t>::max());
  std::vector<std::vector<std::pair<std::size_t, std::size_t>>> adjacency(graph.nodes.size());
  for (std::size_t edge_index = 0; edge_index < graph.edges.size(); ++edge_index) {
    const auto& edge = graph.edges[edge_index];
    if (edge.from >= graph.nodes.size() || edge.to >= graph.nodes.size()) continue;
    adjacency[edge.from].push_back({edge.to, edge_index});
    adjacency[edge.to].push_back({edge.from, edge_index});
  }
  distances[start_node] = 0.0;
  queue.push({0.0, start_node});
  while (!queue.empty()) {
    const auto [cost, current] = queue.top();
    queue.pop();
    if (cost > distances[current] + 1e-7) continue;
    if (current == goal_node) break;
    for (const auto [next, edge_index] : adjacency[current]) {
      const double next_cost = cost + graph.edges[edge_index].cost;
      if (next_cost + 1e-7 >= distances[next]) continue;
      distances[next] = next_cost;
      predecessor[next] = current;
      predecessor_edge[next] = edge_index;
      queue.push({next_cost, next});
    }
  }
  if (!std::isfinite(distances[goal_node])) return {};
  std::vector<std::size_t> nodes;
  for (std::size_t node = goal_node;; node = predecessor[node]) {
    nodes.push_back(node);
    if (node == start_node) break;
    if (predecessor[node] == std::numeric_limits<std::size_t>::max()) return {};
  }
  std::reverse(nodes.begin(), nodes.end());
  std::vector<global_planner::PointPose> route{start};
  const auto append_segment = [&](const global_planner::PointPose& from,
                                  const global_planner::PointPose& to) {
    const double length = std::hypot(to.x - from.x,
                                    std::hypot(to.y - from.y, to.z - from.z));
    const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(
        std::ceil(length / std::max(resolution, 1e-6))));
    for (std::size_t step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      route.push_back({from.x + t * (to.x - from.x),
                       from.y + t * (to.y - from.y),
                       from.z + t * (to.z - from.z),
                       std::nullopt});
    }
  };
  append_segment(start, world(graph.nodes[nodes.front()].cell));
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const auto& edge = graph.edges[predecessor_edge[nodes[i]]];
    const bool forward = edge.from == graph.nodes[nodes[i - 1]].id;
    if (forward) {
      for (const auto& cell : edge.corridor) {
        const auto point = world(cell);
        if (route.empty() || std::hypot(point.x - route.back().x,
                                        std::hypot(point.y - route.back().y,
                                                   point.z - route.back().z)) > 1e-7)
          route.push_back(point);
      }
    } else {
      for (auto it = edge.corridor.rbegin(); it != edge.corridor.rend(); ++it) {
        const auto point = world(*it);
        if (route.empty() || std::hypot(point.x - route.back().x,
                                        std::hypot(point.y - route.back().y,
                                                   point.z - route.back().z)) > 1e-7)
          route.push_back(point);
      }
    }
  }
  append_segment(route.back(), goal);
  return route;
}

struct PlannerRoute {
  bool returned{false};
  bool used_surface_graph{false};
  bool used_rust_graph{false};
  std::size_t points{0};
  double length_m{0.0};
  std::vector<global_planner::PointPose> route;
  global_planner::OctoPlanner3D::RouteValidation geometry{};
};

PlannerRoute runCppPlanner(const std::shared_ptr<octomap::OcTree>& tree,
                           const global_planner::PointPose& start,
                           const global_planner::PointPose& goal) {
  global_planner::PlannerConfig config;
  config.surface_closing_radius_m = .20;
  config.surface_max_hole_area_m2 = .16;
  config.surface_max_slope = .70;
  config.ground_support = true;
  config.support_height_m = .35;
  config.support_height_tolerance_m = .05;
  config.max_step_height = .45;
  config.max_slope = .70;
  config.max_iterations = 200000;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(tree);
  planner.makePlan(start, goal);
  std::vector<global_planner::PointPose> path;
  planner.getPlannerResults(path);
  PlannerRoute result;
  result.returned = !path.empty();
  result.used_surface_graph = planner.searchInfo().used_surface_graph;
  result.used_rust_graph = planner.searchInfo().used_rust_graph;
  result.points = path.size();
  result.route = path;
  result.geometry = planner.validateRoute(path);
  for (std::size_t i = 1; i < path.size(); ++i) {
    result.length_m += std::hypot(path[i].x - path[i - 1].x,
                                  std::hypot(path[i].y - path[i - 1].y,
                                             path[i].z - path[i - 1].z));
  }
  return result;
}

}  // namespace

int main(int argc, char** argv) try {
  const auto options = parse(argc, argv);
  const auto tree = loadMap(options.map);
  const double resolution = tree->getResolution();
  const auto occupied = occupiedCells(*tree);
  const auto known_free = freeCells(*tree);
  const auto rust_surface_started = std::chrono::steady_clock::now();
  const auto rust_surface = global_planner::mls::extractSurfacesRust(
      occupied, known_free,
      static_cast<std::int32_t>(std::ceil(options.headroom / resolution)),
      static_cast<std::uint32_t>(std::ceil(options.closing / resolution)),
      static_cast<std::uint64_t>(std::floor(options.max_hole_area / (resolution * resolution))),
      options.max_slope);
  const double rust_surface_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rust_surface_started).count();
  const auto max_link_cells = static_cast<std::int32_t>(
      std::floor(0.45 / resolution + 1e-9));
  const auto cpp_surface_started = std::chrono::steady_clock::now();
  global_planner::SurfaceIndex cpp_index(
      *tree, options.headroom, options.closing, options.max_hole_area, options.max_slope, true);
  const auto cpp_cells = cpp_index.surfaces(options.headroom);
  const double cpp_surface_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cpp_surface_started).count();
  std::vector<global_planner::mls::SurfaceCell> cpp_surface;
  cpp_surface.reserve(cpp_cells.size());
  for (const auto& cell : cpp_cells) cpp_surface.push_back({cell.x, cell.y, cell.z});

  auto rust_cells = rust_surface.observed;
  rust_cells.insert(rust_cells.end(), rust_surface.inferred.begin(), rust_surface.inferred.end());
  const auto rust_native_graph = global_planner::mls::summarizeGraphRust(
      rust_cells,
      static_cast<std::int32_t>(std::ceil(1.0 / resolution)),
      max_link_cells,
      static_cast<std::int32_t>(std::ceil(0.05 / resolution)),
      static_cast<std::int32_t>(std::ceil(0.20 / resolution)),
      20.0f,
      4.0f);
  const auto rust_graph_started = std::chrono::steady_clock::now();
  const auto rust_native = global_planner::mls::buildGraphRust(
      rust_cells,
      static_cast<std::int32_t>(std::ceil(1.0 / resolution)),
      max_link_cells,
      static_cast<std::int32_t>(std::ceil(0.05 / resolution)),
      static_cast<std::int32_t>(std::ceil(0.20 / resolution)),
      20.0f,
      4.0f);
  const double rust_graph_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rust_graph_started).count();
  const auto rust_points = worldSurface(rust_cells, resolution, kSupportHeight);
  const auto cpp_points = worldSurface(cpp_surface, resolution, kSupportHeight);
  if (cpp_points.size() < 2) throw std::runtime_error("surface map has fewer than two cells");
  const auto start = options.has_start ? options.start : cpp_points.front();
  const auto goal = options.has_goal ? options.goal : cpp_points.back();
  std::size_t repaired_edges = 0;
  std::size_t rejected_edges = 0;
  const auto rust_native_filtered = filterRustGraph(
      tree, start, goal, resolution, rust_native, &repaired_edges, &rejected_edges);
  const auto cpp_graph_started = std::chrono::steady_clock::now();
  const auto rust_graph = makeGraph(rust_points, resolution, start, goal);
  const double rust_surface_graph_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cpp_graph_started).count();
  const auto cpp_surface_graph_started = std::chrono::steady_clock::now();
  const auto cpp_graph = makeGraph(cpp_points, resolution, start, goal);
  const double cpp_graph_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cpp_surface_graph_started).count();
  const auto cpp_route = runCppPlanner(tree, start, goal);
  const auto rust_geometry = validateRustRoute(tree, start, goal, rust_graph.route);
  const auto rust_native_route = planRustGraph(rust_native, resolution, start, goal);
  const auto rust_native_geometry = validateRustRoute(tree, start, goal, rust_native_route);
  const auto rust_native_filtered_route = planRustGraph(
      rust_native_filtered, resolution, start, goal);
  const auto rust_native_filtered_geometry = validateRustRoute(
      tree, start, goal, rust_native_filtered_route);
  const auto cpp_layers = layerStats(cpp_surface);
  const auto rust_layers = layerStats(rust_cells);
  auto zRange = [](const std::vector<global_planner::mls::SurfaceCell>& cells) {
    std::pair<int, int> result{0, 0};
    if (cells.empty()) return result;
    result = {cells.front().z, cells.front().z};
    for (const auto& cell : cells) {
      result.first = std::min(result.first, cell.z);
      result.second = std::max(result.second, cell.z);
    }
    return result;
  };
  const auto cpp_z = zRange(cpp_surface);
  const auto rust_z = zRange(rust_surface.observed);
  const auto cpp_observed_set = cellSet(cpp_surface);
  const auto rust_observed_set = cellSet(rust_surface.observed);
  CellSet cpp_only_observed;
  CellSet rust_only_observed;
  std::set_difference(cpp_observed_set.begin(), cpp_observed_set.end(),
                      rust_observed_set.begin(), rust_observed_set.end(),
                      std::inserter(cpp_only_observed, cpp_only_observed.end()));
  std::set_difference(rust_observed_set.begin(), rust_observed_set.end(),
                      cpp_observed_set.begin(), cpp_observed_set.end(),
                      std::inserter(rust_only_observed, rust_only_observed.end()));

  std::cout << "{\"map\":\"" << options.map << "\",\"resolution_m\":" << resolution
            << ",\"occupied_cells\":" << occupied.size() << ",\"cpp\":{"
            << "\"observed_surface_count\":" << cpp_index.observedSurfaces()
            << ",\"inferred_surface_count\":" << cpp_index.inferredSurfaces()
            << ",\"surface_count\":" << cpp_surface.size()
            << ",\"xy_columns\":" << cpp_layers.xy_columns
            << ",\"multi_level_columns\":" << cpp_layers.multi_level_columns
            << ",\"max_z_levels\":" << cpp_layers.max_z_levels
            << ",\"z_min\":" << cpp_z.first << ",\"z_max\":" << cpp_z.second
            << ",\"bounded_holes\":" << cpp_index.stats().bounded_holes
            << ",\"filled_holes\":" << cpp_index.stats().filled_holes
            << ",\"free_rejected\":" << cpp_index.stats().free_rejected
            << ",\"rejection_count\":" << cpp_index.rejectedInferred().size()
            << "},\"rust\":{"
            << "\"observed_surface_count\":" << rust_surface.observed.size()
            << ",\"inferred_surface_count\":" << rust_surface.inferred.size()
            << ",\"surface_count\":" << rust_surface.observed.size() + rust_surface.inferred.size()
            << ",\"xy_columns\":" << rust_layers.xy_columns
            << ",\"multi_level_columns\":" << rust_layers.multi_level_columns
            << ",\"max_z_levels\":" << rust_layers.max_z_levels
            << ",\"z_min\":" << rust_z.first << ",\"z_max\":" << rust_z.second
            << ",\"inferred_cells\":[";
  for (std::size_t i = 0; i < rust_surface.inferred.size(); ++i) {
    if (i) std::cout << ",";
    const auto& cell = rust_surface.inferred[i];
    std::cout << "[" << cell.x << "," << cell.y << "," << cell.z << "]";
  }
  std::cout << "],\"inferred_rejections\":[";
  bool first_rejection = true;
  for (const auto& cell : rust_surface.inferred) {
    bool matched = false;
    for (const auto& rejection : cpp_index.rejectedInferred()) {
      if (rejection.cell.x != cell.x || rejection.cell.y != cell.y ||
          rejection.cell.z != cell.z) continue;
      matched = true;
      if (!first_rejection) std::cout << ",";
      first_rejection = false;
      std::cout << "{\"cell\":[" << cell.x << "," << cell.y << "," << cell.z
                << "],\"reason\":\"" << rejection.reason << "\"}";
    }
    if (!matched) {
      if (!first_rejection) std::cout << ",";
      first_rejection = false;
      std::cout << "{\"cell\":[" << cell.x << "," << cell.y << "," << cell.z
                << "],\"reason\":\"not_generated_by_cpp\"}";
    }
  }
  std::cout << "],";
  printCellSet("observed_only_cpp", cpp_only_observed);
  std::cout << ",";
  printCellSet("observed_only_rust", rust_only_observed);
  std::cout << ",";
  printRejectionCounts(cpp_index.rejectedInferred());
  std::cout << "},\"graphs\":{";
  printGraph("cpp_surface", cpp_graph);
  std::cout << ",";
  printGraph("rust_surface", rust_graph);
  std::cout << ",";
  printRustGraph(rust_native_graph);
  std::cout << "},\"timing_ms\":{";
  std::cout << "\"rust_surface\":" << rust_surface_ms
            << ",\"cpp_surface\":" << cpp_surface_ms
            << ",\"rust_graph\":" << rust_graph_ms
            << ",\"cpp_graph\":" << cpp_graph_ms
            << ",\"rust_surface_graph\":" << rust_surface_graph_ms << "},";
  std::cout << "\"cpp_planner_geometry\":{\"returned\":"
            << (cpp_route.returned ? "true" : "false")
            << ",\"used_surface_graph\":" << (cpp_route.used_surface_graph ? "true" : "false")
            << ",\"used_rust_graph\":" << (cpp_route.used_rust_graph ? "true" : "false")
            << ",\"route_points\":" << cpp_route.points
            << ",\"route_length_m\":" << cpp_route.length_m
            << ",\"rust_geometry\":\"same_cpp_route_verifier\"},\"cpp_route_geometry\":{";
  printRouteValidation("cpp_route", cpp_route.geometry);
  std::cout << ",";
  printRouteValidation("rust_route", rust_geometry);
  std::cout << ",";
  printRouteValidation("rust_native_route", rust_native_geometry);
  std::cout << ",\"rust_native_filtered\":{";
  std::cout << "\"nodes\":" << rust_native_filtered.nodes.size()
            << ",\"edges\":" << rust_native_filtered.edges.size()
            << ",\"repaired_edges\":" << repaired_edges
            << ",\"rejected_edges\":" << rejected_edges << "},";
  printRouteValidation("route", rust_native_filtered_geometry);
  std::cout << "}";
  std::cout << "}\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
}
