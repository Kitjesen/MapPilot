#include "mls_backend.hpp"
#include "lingtu_mls_core.h"

#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace global_planner::mls {
namespace {

constexpr const char* kMagic = "lingtu.mls.artifact";

struct CellKey {
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const CellKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& key) const {
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

void fail(const char* message, std::string* error) {
  if (error != nullptr) *error = message;
}

std::uint64_t edgeKey(std::uint32_t from, std::uint32_t to) {
  const auto a = std::min(from, to);
  const auto b = std::max(from, to);
  return (static_cast<std::uint64_t>(a) << 32) | b;
}

struct RustBuffers {
  LtMlsBuffer observed{};
  LtMlsBuffer inferred{};

  ~RustBuffers() {
    lt_mls_free_buffer(&observed);
    lt_mls_free_buffer(&inferred);
  }
};

const LtMlsCell kDummyCell{0, 0, 0};

LtMlsPose metricPose(const PointPose& point) {
  return {{point.x, point.y, point.z}, point.yaw.value_or(0.0),
          static_cast<std::uint8_t>(point.yaw.has_value())};
}

struct MetricInput {
  const MetricGeometry& source;
  std::vector<LtMlsCell> preblocked;
  std::exception_ptr error;
  LtMlsGeometry raw{};

  explicit MetricInput(const MetricGeometry& geometry) : source(geometry) {
    preblocked.reserve(source.preblocked.size());
    for (const auto& cell : source.preblocked) preblocked.push_back({cell.x, cell.y, cell.z});
    raw.config = source.config;
    raw.regions = source.regions.data();
    raw.regions_len = source.regions.size();
    raw.preblocked = preblocked.data();
    raw.preblocked_len = preblocked.size();
    raw.context = this;
    raw.cancel = [](void* context) -> std::int32_t {
      auto& input = *static_cast<MetricInput*>(context);
      try {
        return input.source.cancel && input.source.cancel() ? 1 : 0;
      } catch (...) {
        input.error = std::current_exception();
        return 1;
      }
    };
  }
};

std::vector<SurfaceCell> copyCells(const LtMlsBuffer& buffer) {
  std::vector<SurfaceCell> cells;
  cells.reserve(static_cast<std::size_t>(buffer.len));
  for (std::uint64_t i = 0; i < buffer.len; ++i) {
    const auto& cell = buffer.ptr[i];
    cells.push_back({cell.x, cell.y, cell.z});
  }
  return cells;
}

}  // namespace

LtMlsGraphConfig rustGraphConfig(const PlannerConfig& config, double resolution) {
  const int stride = std::max(1, static_cast<int>(std::ceil(
      config.search_resolution_m / resolution)));
  const int height_stride = config.max_slope > 0.0
      ? std::max(stride, static_cast<int>(std::ceil(1.0 / config.max_slope)))
      : stride;
  const int step_cells = std::max(0, static_cast<int>(std::ceil(
      config.max_step_height / resolution)));
  return {
      stride,
      std::max(height_stride, step_cells),
      0,
      std::max(1, config.obstacle_clearance_radius_cells),
      static_cast<float>(std::max(0.0, config.obstacle_clearance_weight)),
      static_cast<float>(std::max(0.0, config.floor_change_penalty)),
      static_cast<float>(std::max(0.0, config.max_slope)),
      step_cells,
      height_stride};
}

PreparedArtifactHandle::~PreparedArtifactHandle() {
  lt_mls_artifact_free(handle_);
}

PreparedArtifactHandle::PreparedArtifactHandle(PreparedArtifactHandle&& other) noexcept
    : handle_(other.handle_) {
  other.handle_ = nullptr;
}

PreparedArtifactHandle& PreparedArtifactHandle::operator=(PreparedArtifactHandle&& other) noexcept {
  if (this == &other) return *this;
  lt_mls_artifact_free(handle_);
  handle_ = other.handle_;
  other.handle_ = nullptr;
  return *this;
}

PreparedArtifactHandle PreparedArtifactHandle::create(
    const std::vector<SurfaceCell>& occupied,
    const std::vector<SurfaceCell>& known_free,
    const LtMlsConfig& surface_config,
    const LtMlsGraphConfig& graph_config,
    std::uint64_t revision) {
  std::vector<LtMlsCell> occupied_input;
  occupied_input.reserve(occupied.size());
  for (const auto& cell : occupied) occupied_input.push_back({cell.x, cell.y, cell.z});
  std::vector<LtMlsCell> free_input;
  free_input.reserve(known_free.size());
  for (const auto& cell : known_free) free_input.push_back({cell.x, cell.y, cell.z});
  LtMlsArtifact* handle = nullptr;
  const auto status = lt_mls_artifact_create(
      occupied_input.empty() ? &kDummyCell : occupied_input.data(), occupied_input.size(),
      free_input.empty() ? &kDummyCell : free_input.data(), free_input.size(),
      &surface_config, &graph_config, revision, &handle);
  if (status != 0 || handle == nullptr) {
    throw std::runtime_error("Rust MLS artifact creation failed: " + std::to_string(status));
  }
  return PreparedArtifactHandle(handle);
}

PreparedArtifactHandle PreparedArtifactHandle::createFromSurface(
    const std::vector<SurfaceCell>& observed,
    const std::vector<SurfaceCell>& inferred,
    const LtMlsGraphConfig& graph_config,
    std::uint64_t revision) {
  std::vector<LtMlsCell> observed_input;
  observed_input.reserve(observed.size());
  for (const auto& cell : observed) observed_input.push_back({cell.x, cell.y, cell.z});
  std::vector<LtMlsCell> inferred_input;
  inferred_input.reserve(inferred.size());
  for (const auto& cell : inferred) inferred_input.push_back({cell.x, cell.y, cell.z});
  LtMlsArtifact* handle = nullptr;
  const auto status = lt_mls_artifact_create_from_surface(
      observed_input.empty() ? &kDummyCell : observed_input.data(), observed_input.size(),
      inferred_input.empty() ? &kDummyCell : inferred_input.data(), inferred_input.size(),
      &graph_config, revision, &handle);
  if (status != 0 || handle == nullptr) {
    throw std::runtime_error("Rust MLS surface artifact creation failed: " +
                             std::to_string(status));
  }
  return PreparedArtifactHandle(handle);
}

PreparedArtifactHandle PreparedArtifactHandle::createFromGraph(
    const std::vector<SurfaceCell>& observed,
    const std::vector<SurfaceCell>& inferred,
    const RustGraph& graph,
    const LtMlsGraphConfig& graph_config,
    std::uint64_t revision) {
  std::vector<LtMlsCell> observed_input;
  observed_input.reserve(observed.size());
  for (const auto& cell : observed) observed_input.push_back({cell.x, cell.y, cell.z});
  std::vector<LtMlsCell> inferred_input;
  inferred_input.reserve(inferred.size());
  for (const auto& cell : inferred) inferred_input.push_back({cell.x, cell.y, cell.z});
  std::vector<LtMlsGraphNode> nodes;
  nodes.reserve(graph.nodes.size());
  std::vector<LtMlsGraphEdge> edges;
  edges.reserve(graph.edges.size());
  std::vector<LtMlsCell> corridor;
  std::size_t corridor_count = 0;
  for (const auto& edge : graph.edges) corridor_count += edge.corridor.size();
  corridor.reserve(corridor_count);
  for (const auto& node : graph.nodes) {
    nodes.push_back({node.id, node.cell.x, node.cell.y, node.cell.z});
  }
  for (const auto& edge : graph.edges) {
    const auto offset = static_cast<std::uint64_t>(corridor.size());
    for (const auto& cell : edge.corridor) corridor.push_back({cell.x, cell.y, cell.z});
    edges.push_back({edge.id != 0 ? edge.id : edgeKey(edge.from, edge.to),
                     edge.from, edge.to, static_cast<float>(edge.cost), offset,
                     static_cast<std::uint64_t>(edge.corridor.size())});
  }
  LtMlsArtifact* handle = nullptr;
  const auto status = lt_mls_artifact_create_from_graph(
      observed_input.empty() ? &kDummyCell : observed_input.data(), observed_input.size(),
      inferred_input.empty() ? &kDummyCell : inferred_input.data(), inferred_input.size(),
      nodes.empty() ? nullptr : nodes.data(), nodes.size(),
      edges.empty() ? nullptr : edges.data(), edges.size(),
      corridor.empty() ? nullptr : corridor.data(), corridor.size(),
      &graph_config, revision, &handle);
  if (status != 0 || handle == nullptr) {
    throw std::runtime_error("Rust MLS graph artifact restore failed: " +
                             std::to_string(status));
  }
  return PreparedArtifactHandle(handle);
}

ArtifactSummary PreparedArtifactHandle::summary() const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  LtMlsArtifactSummary summary{};
  if (lt_mls_artifact_summary(handle_, &summary) != 0) {
    throw std::runtime_error("Rust MLS artifact summary failed");
  }
  return {summary.revision, summary.observed_surface_count, summary.inferred_surface_count,
          summary.node_count, summary.edge_count, summary.component_count};
}

SurfaceMap PreparedArtifactHandle::surfaces() const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  RustBuffers buffers;
  if (lt_mls_artifact_surfaces(handle_, &buffers.observed, &buffers.inferred) != 0) {
    throw std::runtime_error("Rust MLS artifact surface export failed");
  }
  return {copyCells(buffers.observed), copyCells(buffers.inferred)};
}

std::optional<std::int32_t> PreparedArtifactHandle::below(
    std::int32_t x, std::int32_t y, std::int32_t z) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  std::int32_t value = 0;
  const auto status = lt_mls_artifact_below(handle_, x, y, z, &value);
  if (status < 0) throw std::runtime_error("Rust MLS below query failed");
  return status == 0 ? std::nullopt : std::optional<std::int32_t>(value);
}

bool PreparedArtifactHandle::inferred(
    std::int32_t x, std::int32_t y, std::int32_t min_z,
    std::int32_t max_z) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  const auto status = lt_mls_artifact_inferred(handle_, x, y, min_z, max_z);
  if (status < 0) throw std::runtime_error("Rust MLS inferred query failed");
  return status != 0;
}

bool PreparedArtifactHandle::occupied(
    std::int32_t x, std::int32_t y, std::int32_t min_z,
    std::int32_t max_z) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  const auto status = lt_mls_artifact_occupied(handle_, x, y, min_z, max_z);
  if (status < 0) throw std::runtime_error("Rust MLS occupied query unavailable");
  return status != 0;
}

std::optional<std::int32_t> PreparedArtifactHandle::occupiedBelow(
    std::int32_t x, std::int32_t y, std::int32_t z) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  std::int32_t value = 0;
  const auto status = lt_mls_artifact_occupied_below(handle_, x, y, z, &value);
  if (status < 0) throw std::runtime_error("Rust MLS occupied-below query unavailable");
  return status == 0 ? std::nullopt : std::optional<std::int32_t>(value);
}

bool PreparedArtifactHandle::regionOccupied(const LtMlsRegion& region) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  const auto status = lt_mls_artifact_region_occupied(handle_, region);
  if (status < 0) throw std::runtime_error("Rust MLS region query unavailable");
  return status != 0;
}

std::optional<double> PreparedArtifactHandle::height(
    double x, double y, double expected_z, double tolerance,
    double resolution) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(expected_z) ||
      !std::isfinite(tolerance) || tolerance < 0.0 || !std::isfinite(resolution) ||
      resolution <= 0.0) {
    throw std::invalid_argument("invalid Rust MLS height query");
  }
  const auto cell_x = static_cast<std::int32_t>(std::floor(x / resolution));
  const auto cell_y = static_cast<std::int32_t>(std::floor(y / resolution));
  double value = 0.0;
  const auto status = lt_mls_artifact_height(
      handle_, cell_x, cell_y, expected_z, tolerance, resolution, &value);
  if (status < 0) throw std::runtime_error("Rust MLS height query failed");
  return status == 0 ? std::nullopt : std::optional<double>(value);
}

RustGraph PreparedArtifactHandle::graph() const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  LtMlsGraphOutput output{};
  if (lt_mls_artifact_graph(handle_, &output) != 0) {
    throw std::runtime_error("Rust MLS artifact graph export failed");
  }
  RustGraph result;
  result.nodes.reserve(static_cast<std::size_t>(output.nodes_len));
  result.edges.reserve(static_cast<std::size_t>(output.edges_len));
  for (std::uint64_t i = 0; i < output.nodes_len; ++i) {
    const auto& node = output.nodes_ptr[i];
    result.nodes.push_back({node.id, {node.x, node.y, node.z}});
  }
  for (std::uint64_t i = 0; i < output.edges_len; ++i) {
    const auto& edge = output.edges_ptr[i];
    const auto end = edge.corridor_offset + edge.corridor_len;
    if (end > output.corridor_len) {
      lt_mls_free_graph(&output);
      throw std::runtime_error("Rust MLS artifact corridor is out of bounds");
    }
    Edge converted{edge.from, edge.to, edge.cost, {},
                   edge.id != 0 ? edge.id : edgeKey(edge.from, edge.to)};
    converted.corridor.reserve(static_cast<std::size_t>(edge.corridor_len));
    for (std::uint64_t j = edge.corridor_offset; j < end; ++j) {
      const auto& cell = output.corridor_ptr[j];
      converted.corridor.push_back({cell.x, cell.y, cell.z});
    }
    result.edges.push_back(std::move(converted));
  }
  lt_mls_free_graph(&output);
  return result;
}

std::vector<SurfaceCell> PreparedArtifactHandle::query(
    SurfaceCell start, SurfaceCell goal) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  LtMlsBuffer output{};
  if (lt_mls_artifact_query(handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
                            &output) != 0) {
    throw std::runtime_error("Rust MLS artifact query failed");
  }
  std::vector<SurfaceCell> result = copyCells(output);
  lt_mls_free_buffer(&output);
  return result;
}

RustRouteCheckResult PreparedArtifactHandle::checkRoute(
    const std::vector<SurfaceCell>& route,
    const LtMlsRouteCheckConfig& config) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  std::vector<LtMlsCell> input;
  input.reserve(route.size());
  for (const auto& cell : route) input.push_back({cell.x, cell.y, cell.z});
  LtMlsRouteCheckResult output{};
  const auto status = lt_mls_artifact_check_route(
      handle_, input.empty() ? &kDummyCell : input.data(), input.size(), &config, &output);
  if (status != 0) throw std::runtime_error("Rust MLS route check failed");
  return {output.valid != 0, output.failure_kind, output.failed_segment,
          {output.failed_cell.x, output.failed_cell.y, output.failed_cell.z}};
}

std::vector<SurfaceCell> PreparedArtifactHandle::query(
    SurfaceCell start, SurfaceCell goal, std::int32_t max_snap_cells) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  LtMlsBuffer output{};
  if (lt_mls_artifact_query_limited(
          handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
          max_snap_cells, &output) != 0) {
    throw std::runtime_error("Rust MLS limited graph query failed");
  }
  std::vector<SurfaceCell> result = copyCells(output);
  lt_mls_free_buffer(&output);
  return result;
}

std::vector<SurfaceCell> PreparedArtifactHandle::query(
    SurfaceCell start, SurfaceCell goal, std::int32_t max_snap_cells,
    std::int32_t max_iterations) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  LtMlsBuffer output{};
  if (lt_mls_artifact_query_budgeted(
          handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
          max_snap_cells, max_iterations, &output) != 0) {
    throw std::runtime_error("Rust MLS budgeted graph query failed");
  }
  std::vector<SurfaceCell> result = copyCells(output);
  lt_mls_free_buffer(&output);
  return result;
}

std::vector<SurfaceCell> PreparedArtifactHandle::queryAvoiding(
    SurfaceCell start, SurfaceCell goal, std::int32_t max_snap_cells,
    std::int32_t max_iterations,
    const std::vector<SurfaceCell>& blocked_cells) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  if (max_snap_cells < 0 || max_iterations < 1) {
    throw std::invalid_argument("invalid Rust MLS detour budget");
  }
  std::vector<LtMlsCell> blocked;
  blocked.reserve(blocked_cells.size());
  for (const auto& cell : blocked_cells) blocked.push_back({cell.x, cell.y, cell.z});
  LtMlsBuffer output{};
  const LtMlsCell dummy{0, 0, 0};
  if (lt_mls_artifact_query_avoiding(
          handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
          max_snap_cells, max_iterations,
          blocked.empty() ? &dummy : blocked.data(), blocked.size(), &output) != 0) {
    throw std::runtime_error("Rust MLS detour query failed");
  }
  std::vector<SurfaceCell> result = copyCells(output);
  lt_mls_free_buffer(&output);
  return result;
}

PreparedArtifactHandle::QueryResult PreparedArtifactHandle::checkedQuery(
    SurfaceCell start, SurfaceCell goal, std::int32_t max_snap_cells,
    std::int32_t max_iterations, const SegmentCheck& check) const {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  struct Context {
    const SegmentCheck& check;
    std::exception_ptr error;
  } context{check, {}};
  const auto callback = [](LtMlsCell from, LtMlsCell to, std::uint32_t kind,
                           void* user) -> std::int32_t {
    auto& context = *static_cast<Context*>(user);
    try {
      return context.check({from.x, from.y, from.z}, {to.x, to.y, to.z}, kind);
    } catch (...) {
      context.error = std::current_exception();
      return -1;
    }
  };
  LtMlsBuffer output{};
  const auto status = lt_mls_artifact_query_checked(
      handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
      max_snap_cells, max_iterations, callback, &context, &output);
  auto cells = copyCells(output);
  lt_mls_free_buffer(&output);
  if (context.error) std::rethrow_exception(context.error);
  switch (status) {
    case 0: return {QueryStatus::Found, std::move(cells)};
    case 1: return {QueryStatus::Exhausted, {}};
    case 2: return {QueryStatus::Budget, {}};
    case 3: return {QueryStatus::Cancelled, {}};
    default: throw std::runtime_error("Rust MLS checked query failed");
  }
}


PreparedArtifactHandle::MetricQueryResult PreparedArtifactHandle::metricQuery(
    const PointPose& start, const PointPose& goal, std::int32_t max_snap_cells,
    std::int32_t max_iterations, const MetricGeometry& geometry) const {
  MetricInput input(geometry);
  LtMlsPoseBuffer output{};
  const auto status = lt_mls_artifact_query_metric(
      handle_, metricPose(start), metricPose(goal), max_snap_cells, max_iterations,
      &input.raw, &output);
  std::vector<PointPose> poses;
  poses.reserve(output.len);
  for (std::uint64_t i = 0; i < output.len; ++i) {
    const auto& pose = output.ptr[i];
    poses.push_back({pose.point.x,pose.point.y,pose.point.z,
        pose.has_yaw ? std::optional<double>(pose.yaw) : std::nullopt});
  }
  lt_mls_free_pose_buffer(&output);
  if (input.error) std::rethrow_exception(input.error);
  switch (status) {
    case 0: return {QueryStatus::Found, std::move(poses)};
    case 1: return {QueryStatus::Exhausted, {}};
    case 2: return {QueryStatus::Budget, {}};
    case 3: return {QueryStatus::Cancelled, {}};
    default: throw std::runtime_error("Rust MLS metric query failed: " + std::to_string(status));
  }
}

RustRouteCheckResult PreparedArtifactHandle::checkMetricRoute(
    const std::vector<PointPose>& route, const MetricGeometry& geometry) const {
  MetricInput input(geometry);
  std::vector<LtMlsPose> poses;
  poses.reserve(route.size());
  for (const auto& point : route) poses.push_back(metricPose(point));
  LtMlsRouteCheckResult result{};
  const auto status = lt_mls_artifact_check_metric_poses(
      handle_, poses.data(), poses.size(), &input.raw, &result);
  if (input.error) std::rethrow_exception(input.error);
  if (status != 0) throw std::runtime_error("Rust MLS metric route check failed");
  return {result.valid != 0, result.failure_kind, result.failed_segment,
          {result.failed_cell.x, result.failed_cell.y, result.failed_cell.z}};
}

void PreparedArtifactHandle::setGeometry(
    const std::vector<SurfaceCell>& occupied,
    const std::vector<SurfaceCell>& known_free,
    const LtMlsConfig& config) {
  std::vector<LtMlsCell> source, free;
  source.reserve(occupied.size());
  free.reserve(known_free.size());
  for (const auto& cell : occupied) source.push_back({cell.x,cell.y,cell.z});
  for (const auto& cell : known_free) free.push_back({cell.x,cell.y,cell.z});
  if (lt_mls_artifact_set_geometry(handle_, source.data(), source.size(),
                                  free.data(), free.size(), &config) != 0)
    throw std::runtime_error("Rust MLS geometry restore failed");
}

ArtifactUpdateReport PreparedArtifactHandle::update(
    LtMlsRegion region,
    const std::vector<SurfaceCell>& occupied,
    const std::vector<SurfaceCell>& known_free) {
  if (handle_ == nullptr) throw std::runtime_error("empty Rust MLS artifact handle");
  std::vector<LtMlsCell> occupied_input;
  occupied_input.reserve(occupied.size());
  for (const auto& cell : occupied) occupied_input.push_back({cell.x, cell.y, cell.z});
  std::vector<LtMlsCell> free_input;
  free_input.reserve(known_free.size());
  for (const auto& cell : known_free) free_input.push_back({cell.x, cell.y, cell.z});
  LtMlsUpdateReport report{};
  const auto status = lt_mls_artifact_update(
      handle_, region, occupied_input.empty() ? &kDummyCell : occupied_input.data(),
      occupied_input.size(), free_input.empty() ? &kDummyCell : free_input.data(),
      free_input.size(), &report);
  if (status != 0) throw std::runtime_error("Rust MLS artifact update failed");
  return {report.revision, report.surface_cells_rebuilt, report.affected_nodes,
          report.affected_edges, report.preserved_nodes, report.preserved_edges,
          report.reused_nodes, report.new_nodes, report.removed_nodes,
          report.reused_edges, report.rebuilt_edges,
          report.route_replan_required != 0, report.graph_rebuilt_globally != 0};
}

SurfaceMap extractSurfacesRust(const std::vector<SurfaceCell>& occupied,
                               const std::vector<SurfaceCell>& known_free,
                               const LtMlsConfig& config) {
  const LtMlsCell dummy{0, 0, 0};
  std::vector<LtMlsCell> input;
  input.reserve(occupied.size());
  for (const auto& cell : occupied) input.push_back({cell.x, cell.y, cell.z});
  const auto* input_ptr = input.empty() ? &dummy : input.data();
  std::vector<LtMlsCell> free_input;
  free_input.reserve(known_free.size());
  for (const auto& cell : known_free) free_input.push_back({cell.x, cell.y, cell.z});
  const auto* free_ptr = free_input.empty() ? &dummy : free_input.data();
  RustBuffers buffers;
  const auto status = lt_mls_extract_surfaces(input_ptr, input.size(), free_ptr, free_input.size(), &config,
                                               &buffers.observed, &buffers.inferred);
  if (status != 0) {
    throw std::runtime_error("Rust MLS surface extraction failed: " + std::to_string(status));
  }
  return {copyCells(buffers.observed), copyCells(buffers.inferred)};
}

RustGraphSummary summarizeGraphRust(const std::vector<SurfaceCell>& surfaces,
                                    std::int32_t node_spacing_cells,
                                    std::int32_t max_link_cells,
                                    std::int32_t wall_clearance_cells,
                                    std::int32_t wall_buffer_cells,
                                    float wall_buffer_weight,
                                    float step_penalty_weight) {
  const LtMlsCell dummy{0, 0, 0};
  std::vector<LtMlsCell> input;
  input.reserve(surfaces.size());
  for (const auto& cell : surfaces) input.push_back({cell.x, cell.y, cell.z});
  const auto* input_ptr = input.empty() ? &dummy : input.data();
  const LtMlsGraphConfig config{node_spacing_cells, max_link_cells,
                                wall_clearance_cells, wall_buffer_cells,
                                wall_buffer_weight, step_penalty_weight};
  LtMlsGraphSummary output{};
  if (lt_mls_build_graph_summary(input_ptr, input.size(), &config, &output) != 0) {
    throw std::runtime_error("Rust MLS graph summary failed");
  }
  return {output.node_count, output.edge_count, output.component_count,
          output.corridor_cells, output.max_corridor_cells};
}

RustGraph buildGraphRust(const std::vector<SurfaceCell>& surfaces,
                         std::int32_t node_spacing_cells,
                         std::int32_t max_link_cells,
                         std::int32_t wall_clearance_cells,
                         std::int32_t wall_buffer_cells,
                         float wall_buffer_weight,
                         float step_penalty_weight) {
  const LtMlsCell dummy{0, 0, 0};
  std::vector<LtMlsCell> input;
  input.reserve(surfaces.size());
  for (const auto& cell : surfaces) input.push_back({cell.x, cell.y, cell.z});
  const auto* input_ptr = input.empty() ? &dummy : input.data();
  const LtMlsGraphConfig config{node_spacing_cells, max_link_cells,
                                wall_clearance_cells, wall_buffer_cells,
                                wall_buffer_weight, step_penalty_weight};
  LtMlsGraphOutput output{};
  if (lt_mls_build_graph(input_ptr, input.size(), &config, &output) != 0) {
    throw std::runtime_error("Rust MLS graph export failed");
  }
  RustGraph result;
  result.nodes.reserve(static_cast<std::size_t>(output.nodes_len));
  result.edges.reserve(static_cast<std::size_t>(output.edges_len));
  for (std::uint64_t i = 0; i < output.nodes_len; ++i) {
    const auto& node = output.nodes_ptr[i];
    result.nodes.push_back({node.id, {node.x, node.y, node.z}});
  }
  for (std::uint64_t i = 0; i < output.edges_len; ++i) {
    const auto& edge = output.edges_ptr[i];
    Edge converted;
    converted.id = edge.id != 0 ? edge.id : edgeKey(edge.from, edge.to);
    converted.from = edge.from;
    converted.to = edge.to;
    converted.cost = edge.cost;
    const auto end = edge.corridor_offset + edge.corridor_len;
    if (end > output.corridor_len) {
      lt_mls_free_graph(&output);
      throw std::runtime_error("Rust MLS graph corridor is out of bounds");
    }
    converted.corridor.reserve(static_cast<std::size_t>(edge.corridor_len));
    for (std::uint64_t j = edge.corridor_offset; j < end; ++j) {
      const auto& cell = output.corridor_ptr[j];
      converted.corridor.push_back({cell.x, cell.y, cell.z});
    }
    result.edges.push_back(std::move(converted));
  }
  lt_mls_free_graph(&output);
  return result;
}

SurfaceCell cellFromWorld(const PointPose& point, double resolution_m) {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0 ||
      !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    throw std::invalid_argument("invalid MLS world point");
  }
  const auto toCell = [](double value) {
    const double cell = std::floor(value);
    if (cell < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
        cell > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
      throw std::out_of_range("MLS world point is outside cell range");
    }
    return static_cast<std::int32_t>(cell);
  };
  return {toCell(point.x / resolution_m), toCell(point.y / resolution_m),
          toCell(point.z / resolution_m)};
}

PointPose worldFromCell(const SurfaceCell& cell, double resolution_m) {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
    throw std::invalid_argument("invalid MLS resolution");
  }
  return {
      (static_cast<double>(cell.x) + 0.5) * resolution_m,
      (static_cast<double>(cell.y) + 0.5) * resolution_m,
      (static_cast<double>(cell.z) + 0.5) * resolution_m,
      std::nullopt,
  };
}

bool validate(const Artifact& artifact, std::string* error) {
  if (!std::isfinite(artifact.resolution_m) || artifact.resolution_m <= 0.0) {
    fail("invalid MLS artifact resolution", error);
    return false;
  }

  const auto check_cells = [&](const std::vector<SurfaceCell>& cells,
                               std::unordered_set<CellKey, CellKeyHash>& seen,
                               const char* label) {
    for (const auto& cell : cells) {
      if (!seen.insert({cell.x, cell.y, cell.z}).second) {
        if (error != nullptr) *error = std::string("duplicate MLS ") + label + " cell";
        return false;
      }
    }
    return true;
  };
  std::unordered_set<CellKey, CellKeyHash> surfaces;
  surfaces.reserve(artifact.observed.size() + artifact.inferred.size());
  if (!check_cells(artifact.observed, surfaces, "observed") ||
      !check_cells(artifact.inferred, surfaces, "inferred")) return false;

  std::unordered_set<std::uint32_t> nodes;
  nodes.reserve(artifact.nodes.size());
  for (const auto& node : artifact.nodes) {
    if (!nodes.insert(node.id).second) {
      fail("duplicate MLS node id", error);
      return false;
    }
  }

  std::unordered_set<std::uint64_t> edges;
  edges.reserve(artifact.edges.size());
  for (const auto& edge : artifact.edges) {
    const auto expected_id = edgeKey(edge.from, edge.to);
    if (nodes.find(edge.from) == nodes.end() || nodes.find(edge.to) == nodes.end() ||
        edge.from == edge.to || !std::isfinite(edge.cost) || edge.cost <= 0.0 ||
        edge.corridor.size() < 2 ||
        (edge.id != 0 && edge.id != expected_id) ||
        !edges.insert(expected_id).second) {
      fail("invalid MLS edge", error);
      return false;
    }
  }
  return true;
}

void write(lingtu::maps::BinaryWriter& out, const Artifact& artifact) {
  std::string error;
  if (!validate(artifact, &error)) throw std::runtime_error(error);

  out.text(kMagic);
  out.value<std::uint32_t>(kFormatVersion);
  out.value(artifact.resolution_m);
  out.value(artifact.revision);

  out.value<std::uint64_t>(artifact.observed.size());
  for (const auto& cell : artifact.observed) {
    out.value(cell.x);
    out.value(cell.y);
    out.value(cell.z);
  }
  out.value<std::uint64_t>(artifact.inferred.size());
  for (const auto& cell : artifact.inferred) {
    out.value(cell.x);
    out.value(cell.y);
    out.value(cell.z);
  }

  out.value<std::uint64_t>(artifact.nodes.size());
  for (const auto& node : artifact.nodes) {
    out.value(node.id);
    out.value(node.cell.x);
    out.value(node.cell.y);
    out.value(node.cell.z);
  }

  out.value<std::uint64_t>(artifact.edges.size());
  for (const auto& edge : artifact.edges) {
    const auto id = edge.id != 0 ? edge.id : edgeKey(edge.from, edge.to);
    out.value<std::uint64_t>(id);
    out.value(edge.from);
    out.value(edge.to);
    out.value(edge.cost);
    out.vector(edge.corridor);
  }
}

Artifact read(lingtu::maps::BinaryReader& in, bool require_complete) {
  static_assert(sizeof(SurfaceCell) == 12 && offsetof(SurfaceCell, z) == 8);
  static_assert(sizeof(Node) == 16 && offsetof(Node, cell) == 4);
  if (in.text() != kMagic) {
    throw std::runtime_error("unsupported MLS artifact format");
  }
  const auto format = in.value<std::uint32_t>();
  if (format != kFormatVersion) {
    throw std::runtime_error("unsupported MLS artifact format");
  }

  Artifact artifact;
  artifact.resolution_m = in.value<double>();
  artifact.revision = in.value<std::uint64_t>();

  artifact.observed = in.vector<SurfaceCell>();
  artifact.inferred = in.vector<SurfaceCell>();
  artifact.nodes = in.vector<Node>();

  const auto edge_count = in.count(32);
  artifact.edges.reserve(edge_count);
  for (std::size_t i = 0; i < edge_count; ++i) {
    Edge edge;
    edge.id = in.value<std::uint64_t>();
    edge.from = in.value<std::uint32_t>();
    edge.to = in.value<std::uint32_t>();
    edge.cost = in.value<double>();
    edge.corridor = in.vector<SurfaceCell>();
    if (edge.id == 0) edge.id = edgeKey(edge.from, edge.to);
    artifact.edges.push_back(std::move(edge));
  }

  std::string error;
  if ((require_complete && !in.complete()) || !validate(artifact, &error)) {
    throw std::runtime_error(error.empty() ? "invalid MLS artifact" : error);
  }
  return artifact;
}

}  // namespace global_planner::mls
