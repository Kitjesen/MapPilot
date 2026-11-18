#pragma once

#include "global_planner.h"
#include "lingtu_mls_core.h"
#include "lingtu/maps/io/binary.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace global_planner::mls {

// Stable interchange types for the Rust MLS backend. They do not expose the
// native planner's collision API.
struct SurfaceCell {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const SurfaceCell& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct Node {
  std::uint32_t id{0};
  SurfaceCell cell{};

  bool operator==(const Node& other) const {
    return id == other.id && cell == other.cell;
  }
};

struct Edge {
  std::uint32_t from{0};
  std::uint32_t to{0};
  double cost{0.0};
  std::vector<SurfaceCell> corridor;
  // Stable for the lifetime of the endpoint node IDs. Legacy artifacts derive
  // this value from from/to while they are read.
  std::uint64_t id{0};

  bool operator==(const Edge& other) const {
    return id == other.id && from == other.from && to == other.to && cost == other.cost &&
           corridor == other.corridor;
  }
};

struct Artifact {
  double resolution_m{0.0};
  std::uint64_t revision{0};
  std::vector<SurfaceCell> observed;
  std::vector<SurfaceCell> inferred;
  std::vector<Node> nodes;
  std::vector<Edge> edges;

  bool operator==(const Artifact& other) const {
    const auto same_edge = [](const Edge& lhs, const Edge& rhs) {
      const auto id = [](const Edge& edge) {
        if (edge.id != 0) return edge.id;
        const auto a = std::min(edge.from, edge.to);
        const auto b = std::max(edge.from, edge.to);
        return (std::uint64_t{a} << 32) | std::uint64_t{b};
      };
      return id(lhs) == id(rhs) && lhs.from == rhs.from && lhs.to == rhs.to &&
             lhs.cost == rhs.cost && lhs.corridor == rhs.corridor;
    };
    return resolution_m == other.resolution_m && revision == other.revision &&
           observed == other.observed && inferred == other.inferred &&
           nodes == other.nodes && edges.size() == other.edges.size() &&
           std::equal(edges.begin(), edges.end(), other.edges.begin(), same_edge);
  }
};

struct SurfaceMap {
  std::vector<SurfaceCell> observed;
  std::vector<SurfaceCell> inferred;
};

struct RustGraphSummary {
  std::uint64_t nodes{0};
  std::uint64_t edges{0};
  std::uint64_t components{0};
  std::uint64_t corridor_cells{0};
  std::uint64_t max_corridor_cells{0};
};

struct RustGraph {
  std::vector<Node> nodes;
  std::vector<Edge> edges;
};

struct RustRouteCheckResult {
  static constexpr std::uint32_t kCancelled = LT_MLS_ROUTE_CANCELLED;
  bool valid{false};
  std::uint32_t failure_kind{0};
  std::int32_t failed_segment{-1};
  SurfaceCell failed_cell{};
};

struct MetricGeometry {
  LtMlsMetricConfig config{};
  std::vector<LtMlsMetricRegion> regions;
  std::vector<SurfaceCell> preblocked;
  std::function<bool()> cancel;
};

struct ArtifactSummary {
  std::uint64_t revision{0};
  std::uint64_t observed_surfaces{0};
  std::uint64_t inferred_surfaces{0};
  std::uint64_t nodes{0};
  std::uint64_t edges{0};
  std::uint64_t components{0};
};

struct ArtifactUpdateReport {
  std::uint64_t revision{0};
  std::uint64_t surface_cells_rebuilt{0};
  std::uint64_t affected_nodes{0};
  std::uint64_t affected_edges{0};
  std::uint64_t preserved_nodes{0};
  std::uint64_t preserved_edges{0};
  std::uint64_t reused_nodes{0};
  std::uint64_t new_nodes{0};
  std::uint64_t removed_nodes{0};
  std::uint64_t reused_edges{0};
  std::uint64_t rebuilt_edges{0};
  bool route_replan_required{false};
  bool graph_rebuilt_globally{false};
};

class PreparedArtifactHandle {
public:
  enum class QueryStatus { Found, Exhausted, Budget, Cancelled, StartUnreachable, GoalUnreachable };
  struct QueryResult {
    QueryStatus status;
    std::vector<SurfaceCell> cells;
  };
  struct MetricQueryResult {
    QueryStatus status;
    std::vector<PointPose> poses;
    std::uint32_t failure_stage{0};
    std::int32_t failure_segment{-1};
    SurfaceCell failure_cell{};
  };
  using SegmentCheck = std::function<int(SurfaceCell, SurfaceCell, std::uint32_t)>;
  PreparedArtifactHandle() = default;
  ~PreparedArtifactHandle();
  PreparedArtifactHandle(const PreparedArtifactHandle&) = delete;
  PreparedArtifactHandle& operator=(const PreparedArtifactHandle&) = delete;
  PreparedArtifactHandle(PreparedArtifactHandle&& other) noexcept;
  PreparedArtifactHandle& operator=(PreparedArtifactHandle&& other) noexcept;

  static PreparedArtifactHandle create(
      const std::vector<SurfaceCell>& occupied,
      const std::vector<SurfaceCell>& known_free,
      const LtMlsConfig& surface_config,
      const LtMlsGraphConfig& graph_config,
      std::uint64_t revision = 1);

  static PreparedArtifactHandle createFromSurface(
      const std::vector<SurfaceCell>& observed,
      const std::vector<SurfaceCell>& inferred,
      const LtMlsGraphConfig& graph_config,
      std::uint64_t revision = 1);

  static PreparedArtifactHandle createFromGraph(
      const std::vector<SurfaceCell>& observed,
      const std::vector<SurfaceCell>& inferred,
      const RustGraph& graph,
      const LtMlsGraphConfig& graph_config,
      std::uint64_t revision = 1);

  ArtifactSummary summary() const;
  SurfaceMap surfaces() const;
  std::optional<std::int32_t> below(std::int32_t x, std::int32_t y,
                                    std::int32_t z) const;
  bool inferred(std::int32_t x, std::int32_t y, std::int32_t min_z,
                std::int32_t max_z) const;
  bool occupied(std::int32_t x, std::int32_t y, std::int32_t min_z,
                std::int32_t max_z) const;
  std::optional<std::int32_t> occupiedBelow(
      std::int32_t x, std::int32_t y, std::int32_t z) const;
  bool regionOccupied(const LtMlsRegion& region) const;
  std::optional<double> height(double x, double y, double expected_z,
                               double tolerance, double resolution) const;
  RustGraph graph() const;
  std::vector<SurfaceCell> query(SurfaceCell start, SurfaceCell goal) const;
  std::vector<SurfaceCell> query(SurfaceCell start, SurfaceCell goal,
                                 std::int32_t max_snap_cells) const;
  std::vector<SurfaceCell> query(SurfaceCell start, SurfaceCell goal,
                                 std::int32_t max_snap_cells,
                                 std::int32_t max_iterations) const;
  std::vector<SurfaceCell> queryAvoiding(
      SurfaceCell start, SurfaceCell goal, std::int32_t max_snap_cells,
      std::int32_t max_iterations,
      const std::vector<SurfaceCell>& blocked_cells) const;
  QueryResult checkedQuery(SurfaceCell start, SurfaceCell goal,
                           std::int32_t max_snap_cells,
                           std::int32_t max_iterations,
                           const SegmentCheck& check) const;
  MetricQueryResult metricQuery(const PointPose& start, const PointPose& goal,
                          std::int32_t max_snap_cells, std::int32_t max_iterations,
                          const MetricGeometry& geometry) const;
  RustRouteCheckResult checkMetricRoute(const std::vector<PointPose>& route,
                                        const MetricGeometry& geometry) const;
  RustRouteCheckResult checkRoute(
      const std::vector<SurfaceCell>& route,
      const LtMlsRouteCheckConfig& config) const;
  void setGeometry(const std::vector<SurfaceCell>& occupied,
                   const std::vector<SurfaceCell>& known_free,
                   const LtMlsConfig& config);
  ArtifactUpdateReport update(
      LtMlsRegion region,
      const std::vector<SurfaceCell>& occupied,
      const std::vector<SurfaceCell>& known_free);

private:
  explicit PreparedArtifactHandle(LtMlsArtifact* handle) : handle_(handle) {}
  LtMlsArtifact* handle_{nullptr};
};

constexpr std::uint32_t kFormatVersion = 3;

LtMlsGraphConfig rustGraphConfig(const PlannerConfig& config, double resolution);

SurfaceMap extractSurfacesRust(const std::vector<SurfaceCell>& occupied,
                               const std::vector<SurfaceCell>& known_free,
                               const LtMlsConfig& config);

RustGraphSummary summarizeGraphRust(const std::vector<SurfaceCell>& surfaces,
                                    std::int32_t node_spacing_cells = 1,
                                    std::int32_t max_link_cells = 4,
                                    std::int32_t wall_clearance_cells = 0,
                                    std::int32_t wall_buffer_cells = 0,
                                    float wall_buffer_weight = 0.0f,
                                    float step_penalty_weight = 0.0f);

RustGraph buildGraphRust(const std::vector<SurfaceCell>& surfaces,
                         std::int32_t node_spacing_cells,
                         std::int32_t max_link_cells,
                         std::int32_t wall_clearance_cells = 0,
                         std::int32_t wall_buffer_cells = 0,
                         float wall_buffer_weight = 0.0f,
                         float step_penalty_weight = 0.0f);

SurfaceCell cellFromWorld(const PointPose& point, double resolution_m);
PointPose worldFromCell(const SurfaceCell& cell, double resolution_m);

bool validate(const Artifact& artifact, std::string* error = nullptr);
void write(lingtu::maps::BinaryWriter& out, const Artifact& artifact);
Artifact read(lingtu::maps::BinaryReader& in, bool require_complete = true);

}  // namespace global_planner::mls
