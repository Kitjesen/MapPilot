#pragma once

// Ported from SCAN-Planner path_searching/dyn_a_star.h at commit 348e8a5.
// ROS logging/time were replaced at the module seam; search behavior is kept.
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Eigen>
#include <memory>
#include <queue>
#include <vector>

#include "planning/local/scan/upstream/plan_env/grid_map.h"

namespace nav_kernel::local::scan::upstream {

constexpr double kInfinity = 1 << 20;

struct GridNode;
using GridNodePtr = GridNode *;

enum class AStarResult {
  Success,
  InitError,
  SearchError,
};

struct GridNode {
  enum class State {
    OpenSet = 1,
    ClosedSet = 2,
    Undefined = 3,
  };

  int rounds{0};
  State state{State::Undefined};
  Eigen::Vector3i index{};
  double gScore{kInfinity};
  double fScore{kInfinity};
  GridNodePtr cameFrom{nullptr};
};

class NodeComparator {
 public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) const {
    return node1->fScore > node2->fScore;
  }
};

class AStar {
 public:
  AStar() = default;
  ~AStar();

  AStar(const AStar &) = delete;
  AStar &operator=(const AStar &) = delete;

  void initGridMap(GridMap::Ptr occupancyMap, const Eigen::Vector3i &poolSize);
  void setGridMap(GridMap::Ptr occupancyMap) noexcept;
  AStarResult search(double stepSize, Eigen::Vector3d start,
                     Eigen::Vector3d end);

  [[nodiscard]] std::vector<Eigen::Vector3d> path() const;
  [[nodiscard]] int expandedNodes() const noexcept;

 private:
  void releaseGridNodes() noexcept;
  double getDiagHeuristic(GridNodePtr node1, GridNodePtr node2) const;
  double getHeuristic(GridNodePtr node1, GridNodePtr node2) const;
  bool convertAndAdjust(Eigen::Vector3d start, Eigen::Vector3d end,
                        Eigen::Vector3i &startIndex,
                        Eigen::Vector3i &endIndex) const;
  Eigen::Vector3d indexToCoordinate(const Eigen::Vector3i &index) const;
  bool coordinateToIndex(const Eigen::Vector3d &point,
                         Eigen::Vector3i &index) const;
  int occupancy(const Eigen::Vector3d &position, double yaw) const;
  std::vector<GridNodePtr> retrievePath(GridNodePtr current) const;

  GridMap::Ptr gridMap_{};
  double stepSize_{0.0};
  double inverseStepSize_{0.0};
  Eigen::Vector3d center_{};
  Eigen::Vector3i centerIndex_{};
  Eigen::Vector3i poolSize_{};
  static constexpr double kTieBreaker = 1.0 + 1.0 / 10000.0;
  std::vector<GridNodePtr> gridPath_{};
  GridNodePtr ***gridNodeMap_{nullptr};
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator>
      openSet_{};
  int rounds_{0};
  int expandedNodes_{0};
};

}  // namespace nav_kernel::local::scan::upstream
