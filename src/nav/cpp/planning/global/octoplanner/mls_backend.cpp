#include "mls_backend.hpp"
#include "lingtu_mls_core.h"

#include <cmath>
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
  return (static_cast<std::uint64_t>(from) << 32) | to;
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

void writeSurfaceMap(lingtu::maps::BinaryWriter& out, const SurfaceMap& surface) {
  for (const auto* cells : {&surface.observed, &surface.inferred}) {
    out.value<std::uint64_t>(cells->size());
    for (const auto& cell : *cells) {
      out.value<std::int32_t>(cell.x);
      out.value<std::int32_t>(cell.y);
      out.value<std::int32_t>(cell.z);
    }
  }
}

SurfaceMap readSurfaceMap(lingtu::maps::BinaryReader& in) {
  SurfaceMap surface;
  auto read = [&](std::vector<SurfaceCell>& cells) {
    const auto count = in.count(sizeof(std::int32_t) * 3);
    cells.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      cells.push_back({in.value<std::int32_t>(), in.value<std::int32_t>(),
                       in.value<std::int32_t>()});
    }
  };
  read(surface.observed);
  read(surface.inferred);
  return surface;
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
    Edge converted{edge.from, edge.to, edge.cost, {}};
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
  const auto status = lt_mls_artifact_query_budgeted(
      handle_, {start.x, start.y, start.z}, {goal.x, goal.y, goal.z},
      max_snap_cells, max_iterations, &output);
  if (status == -3) {
    throw std::runtime_error("Rust MLS graph query exceeded iteration budget");
  }
  if (status != 0) {
    throw std::runtime_error("Rust MLS budgeted graph query failed");
  }
  std::vector<SurfaceCell> result = copyCells(output);
  lt_mls_free_buffer(&output);
  return result;
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
                               std::int32_t clearance_cells,
                               std::uint32_t closing_passes,
                               std::uint64_t max_hole_cells,
                               double max_slope) {
  if (clearance_cells < 0) {
    throw std::invalid_argument("invalid MLS clearance");
  }
  const LtMlsCell dummy{0, 0, 0};
  std::vector<LtMlsCell> input;
  input.reserve(occupied.size());
  for (const auto& cell : occupied) input.push_back({cell.x, cell.y, cell.z});
  const auto* input_ptr = input.empty() ? &dummy : input.data();
  std::vector<LtMlsCell> free_input;
  free_input.reserve(known_free.size());
  for (const auto& cell : known_free) free_input.push_back({cell.x, cell.y, cell.z});
  const auto* free_ptr = free_input.empty() ? &dummy : free_input.data();
  const LtMlsConfig config{clearance_cells, closing_passes, max_hole_cells,
                           static_cast<float>(max_slope)};
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

  std::unordered_set<CellKey, CellKeyHash> surfaces;
  surfaces.reserve(artifact.surfaces.size());
  for (const auto& cell : artifact.surfaces) {
    if (!surfaces.insert({cell.x, cell.y, cell.z}).second) {
      fail("duplicate MLS surface cell", error);
      return false;
    }
  }

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
    if (nodes.find(edge.from) == nodes.end() || nodes.find(edge.to) == nodes.end() ||
        edge.from == edge.to || !std::isfinite(edge.cost) || edge.cost <= 0.0 ||
        edge.corridor.size() < 2 || !edges.insert(edgeKey(edge.from, edge.to)).second) {
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

  out.value<std::uint64_t>(artifact.surfaces.size());
  for (const auto& cell : artifact.surfaces) {
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
    out.value(edge.from);
    out.value(edge.to);
    out.value(edge.cost);
    out.value<std::uint64_t>(edge.corridor.size());
    for (const auto& cell : edge.corridor) {
      out.value(cell.x);
      out.value(cell.y);
      out.value(cell.z);
    }
  }
}

Artifact read(lingtu::maps::BinaryReader& in) {
  if (in.text() != kMagic || in.value<std::uint32_t>() != kFormatVersion) {
    throw std::runtime_error("unsupported MLS artifact format");
  }

  Artifact artifact;
  artifact.resolution_m = in.value<double>();
  artifact.revision = in.value<std::uint64_t>();

  const auto surface_count = in.count(12);
  artifact.surfaces.reserve(surface_count);
  for (std::size_t i = 0; i < surface_count; ++i) {
    artifact.surfaces.push_back({in.value<std::int32_t>(), in.value<std::int32_t>(),
                                 in.value<std::int32_t>()});
  }

  const auto node_count = in.count(16);
  artifact.nodes.reserve(node_count);
  for (std::size_t i = 0; i < node_count; ++i) {
    artifact.nodes.push_back({in.value<std::uint32_t>(),
                              {in.value<std::int32_t>(), in.value<std::int32_t>(),
                               in.value<std::int32_t>()}});
  }

  const auto edge_count = in.count(24);
  artifact.edges.reserve(edge_count);
  for (std::size_t i = 0; i < edge_count; ++i) {
    Edge edge;
    edge.from = in.value<std::uint32_t>();
    edge.to = in.value<std::uint32_t>();
    edge.cost = in.value<double>();
    const auto corridor_count = in.count(12);
    edge.corridor.reserve(corridor_count);
    for (std::size_t j = 0; j < corridor_count; ++j) {
      edge.corridor.push_back({in.value<std::int32_t>(), in.value<std::int32_t>(),
                               in.value<std::int32_t>()});
    }
    artifact.edges.push_back(std::move(edge));
  }

  std::string error;
  if (!in.complete() || !validate(artifact, &error)) {
    throw std::runtime_error(error.empty() ? "invalid MLS artifact" : error);
  }
  return artifact;
}

}  // namespace global_planner::mls
