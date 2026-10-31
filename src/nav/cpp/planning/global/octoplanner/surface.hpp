#pragma once

#include "global_planner.h"
#include <optional>

namespace global_planner {

// Immutable occupied columns and inferred support surfaces; never writes the OcTree.
class SurfaceIndex {
 public:
  SurfaceIndex(const octomap::OcTree &tree, double headroom, double closing_radius);
  std::optional<int> below(int x, int y, int z) const;
  bool occupied(int x, int y, int minimum_z, int maximum_z) const;
  bool inferred(int x, int y, int minimum_z, int maximum_z) const;
  bool regionOccupied(int min_x, int min_y, int max_x, int max_y, int min_z, int max_z) const;
  std::optional<double> height(const PointPose &expected, double tolerance, int neighbor_radius = 0) const;
  std::vector<GridIndex> surfaces(double headroom) const;
  std::size_t observedSurfaces() const { return observed_surfaces_; }
  std::size_t inferredSurfaces() const { return inferred_surfaces_; }

 private:
  using Columns = std::unordered_map<GridIndex, std::vector<int>, GridIndexHash>;
  bool headroomFree(int x, int y, int z, int height) const;
  bool nearbyEvidence(int x, int y, int z) const;
  Columns occupied_;
  Columns inferred_;
  struct OccupiedLevel { int z; std::uint64_t xy; };
  std::unordered_map<GridIndex,std::vector<OccupiedLevel>,GridIndexHash> blocks_;
  std::size_t observed_surfaces_{0};
  std::size_t inferred_surfaces_{0};
  double resolution_{0.0};
};

}  // namespace global_planner
