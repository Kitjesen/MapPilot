#pragma once

#include "planning/global/contract.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lingtu::nav::plan::far_planner {

constexpr std::int8_t kUnknownCell = -1;
constexpr std::int8_t kFreeCell = 0;
constexpr std::int8_t kOccupiedCell = 100;

struct FarGridMap {
  std::int32_t width{0};
  std::int32_t height{0};
  double resolution_m{0.20};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  std::string frame_id{"map"};
  std::uint64_t generation{0U};
  MapIdentity identity{};
  std::vector<std::int8_t> cells;

  std::size_t CellCount() const noexcept;
  std::size_t Index(std::int32_t x, std::int32_t y) const;
  void Validate() const;
};

struct FarPlannerConfig {
  double robot_radius_m{0.35};
  double obstacle_clearance_m{0.10};
  double max_visibility_distance_m{30.0};
  double unknown_cost_multiplier{6.0};
  std::int32_t corner_separation_cells{1};
  std::int32_t snap_search_radius_cells{12};
  std::size_t max_graph_nodes{4096U};
  std::size_t max_visibility_pairs{500000U};
  std::size_t max_search_expansions{500000U};
  bool allow_unknown_fallback{false};
  bool simplify_path{true};
};

struct FarDiagnostics {
  std::uint64_t map_generation{0U};
  std::string map_frame_id;
  std::string map_update_mode{"none"};
  std::string planning_phase{"none"};
  std::string failure_reason;
  std::size_t changed_cells{0U};
  std::size_t graph_nodes{0U};
  std::size_t visibility_pairs{0U};
  std::size_t reusable_edges{0U};
  std::size_t recomputed_edges{0U};
  std::size_t search_expansions{0U};
  std::size_t raw_path_points{0U};
  std::size_t output_path_points{0U};
  std::size_t unknown_cells_traversed{0U};
  bool used_unknown_space{false};
  bool start_snapped{false};
  bool goal_snapped{false};
  bool cancelled{false};
  double elapsed_ms{0.0};
};

class FarPlanner final {
 public:
  explicit FarPlanner(FarPlannerConfig config = {});

  FarPlanner(const FarPlanner&) = delete;
  FarPlanner& operator=(const FarPlanner&) = delete;

  void UpdateMap(FarGridMap map);
  GlobalPlanResult Plan(
      const GlobalPlanRequest& request,
      const GlobalPlanCancelCheck& cancel = {});

  const FarGridMap& Map() const noexcept { return map_; }
  const FarPlannerConfig& Config() const noexcept { return config_; }
  const FarDiagnostics& LastDiagnostics() const noexcept { return diagnostics_; }
  bool HasMap() const noexcept { return has_map_; }

 private:
  struct CellCoord {
    std::int32_t x{0};
    std::int32_t y{0};
  };

  struct GraphNode {
    std::uint32_t cell_index{0U};
    CellCoord cell{};
    double x_m{0.0};
    double y_m{0.0};
  };

  struct VisibilityEdge {
    std::uint32_t from{0U};
    std::uint32_t to{0U};
    double length_m{0.0};
    std::size_t unknown_cells{0U};
    bool occupied_blocked{false};
    std::vector<std::uint32_t> raster_cells;
  };

  struct Arc {
    std::uint32_t to{0U};
    std::uint32_t edge_index{0U};
  };

  struct SearchResult {
    bool found{false};
    bool cancelled{false};
    std::size_t expansions{0U};
    std::vector<std::uint32_t> node_indices;
  };

  struct EndpointCell {
    CellCoord cell{};
    bool snapped{false};
  };

  static void ValidateConfig(const FarPlannerConfig& config);
  static std::uint64_t EdgeKey(std::uint32_t lhs, std::uint32_t rhs);

  bool SameGeometry(const FarGridMap& lhs, const FarGridMap& rhs) const;
  bool WorldToCell(double x_m, double y_m, CellCoord* cell) const;
  GlobalPlanPoint CellCenter(const CellCoord& cell, double z_m) const;
  bool IsCellFree(const CellCoord& cell) const;
  bool IsCellUnknown(const CellCoord& cell) const;
  bool IsCellBlocked(const CellCoord& cell) const;
  EndpointCell SnapEndpoint(
      double x_m,
      double y_m,
      bool allow_unknown) const;

  std::vector<std::uint8_t> BuildInflatedMask(const FarGridMap& map) const;
  std::vector<GraphNode> ExtractCornerNodes() const;
  std::vector<std::uint32_t> RasterLine(
      const CellCoord& from,
      const CellCoord& to) const;
  VisibilityEdge EvaluateEdge(
      std::uint32_t from,
      std::uint32_t to,
      const GraphNode& lhs,
      const GraphNode& rhs) const;
  bool EdgeTouchesChanged(
      const VisibilityEdge& edge,
      const std::vector<std::uint8_t>& changed) const;
  void RebuildVisibilityGraph(const std::vector<std::uint8_t>& changed, bool full_rebuild);

  bool EdgeUsable(const VisibilityEdge& edge, bool allow_unknown) const;
  double EdgeCost(const VisibilityEdge& edge, bool allow_unknown) const;
  SearchResult Search(
      const std::vector<GraphNode>& nodes,
      const std::vector<VisibilityEdge>& edges,
      const std::vector<std::vector<Arc>>& adjacency,
      std::uint32_t start_index,
      std::uint32_t goal_index,
      bool allow_unknown,
      std::size_t max_expansions,
      const GlobalPlanCancelCheck& cancel) const;
  std::vector<std::uint32_t> SimplifyNodePath(
      const std::vector<std::uint32_t>& path,
      const std::vector<GraphNode>& nodes,
      bool allow_unknown) const;
  std::size_t CountUnknownOnPath(
      const std::vector<std::uint32_t>& path,
      const std::vector<GraphNode>& nodes) const;
  GlobalPlanResult FailureResult(
      const GlobalPlanRequest& request,
      std::string reason,
      bool cancelled,
      double elapsed_ms);

  FarPlannerConfig config_;
  FarGridMap map_;
  bool has_map_{false};
  std::vector<std::uint8_t> inflated_blocked_;
  std::vector<GraphNode> nodes_;
  std::vector<VisibilityEdge> edges_;
  std::vector<std::vector<Arc>> adjacency_;
  std::unordered_map<std::uint64_t, VisibilityEdge> edge_cache_;
  FarDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::plan::far_planner
