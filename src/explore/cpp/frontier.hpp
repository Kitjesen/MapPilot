#pragma once

#include "explore_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::explore::detail {

struct Cell {
  int row{0};
  int col{0};
};

struct FrontierCluster {
  std::uint64_t id{0U};
  Cell centroid;
  std::vector<Cell> cells;
};

struct FrontierAnalysis {
  bool valid{false};
  bool resource_limited{false};
  std::string reason;
  Cell robot_cell;
  std::vector<std::uint8_t> reachable;
  std::vector<double> travel_distance_m;
  std::vector<FrontierCluster> clusters;
  std::size_t reachable_free_cells{0U};
  std::size_t frontier_cells{0U};
};

[[nodiscard]] bool WorldToCell(
    const Grid2D& grid,
    double x,
    double y,
    Cell* out);

[[nodiscard]] std::pair<double, double> CellToWorld(
    const Grid2D& grid,
    int row,
    int col);

[[nodiscard]] bool HasFreeLineOfSight(
    const Grid2D& grid,
    const Pose2D& from,
    const Pose2D& to);

[[nodiscard]] FrontierAnalysis AnalyzeFrontiers(
    const Grid2D& grid,
    const Pose2D& robot,
    int min_frontier_size,
    std::size_t max_frontier_cells,
    std::size_t max_frontier_clusters,
    const ExploreCancelCheck& cancel);

}  // namespace lingtu::explore::detail
