#pragma once

#include <lingtu/settings.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace octomap { class OcTree; }

namespace lingtu::maps { class BinaryReader; class BinaryWriter; }
namespace lingtu::maps::layers {

struct SurfaceCell {
  int x, y, z;
  bool operator==(const SurfaceCell& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct SurfaceCellHash {
  std::size_t operator()(const SurfaceCell& cell) const {
    std::size_t seed = std::hash<int>{}(cell.x);
    for (int value : {cell.y, cell.z})
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct SurfaceStats {
  // A sloped hole can contribute one slice at several expected heights.
  std::size_t bounded_holes{0};
  std::size_t filled_holes{0};
  std::size_t free_rejected{0};
};

// Immutable observed occupancy and inferred support; the source OcTree is never changed.
class SurfaceIndex {
 public:
  SurfaceIndex(const octomap::OcTree& tree, double headroom, double radius,
               double max_hole_area_m2 = ::lingtu::settings::global::native_nav::global::surface_max_hole_area_m2,
               double max_slope = ::lingtu::settings::global::native_nav::global::surface_max_slope);
  std::optional<int> below(int x, int y, int z) const;
  bool occupied(int x, int y, int minimum_z, int maximum_z) const;
  bool inferred(int x, int y, int minimum_z, int maximum_z) const;
  bool regionOccupied(int min_x, int min_y, int max_x, int max_y, int min_z, int max_z) const;
  std::optional<double> height(const std::array<double, 3>& expected, double tolerance) const;
  std::vector<SurfaceCell> surfaces(double headroom) const;
  std::vector<SurfaceCell> inferredPoints() const;
  std::size_t observedSurfaces() const { return observed_surfaces_; }
  std::size_t inferredSurfaces() const { return inferred_surfaces_; }
  double resolution() const { return resolution_; }
  const SurfaceStats& stats() const { return stats_; }
  void write(BinaryWriter& out) const;
  static std::shared_ptr<SurfaceIndex> read(BinaryReader& in);

 private:
  SurfaceIndex() = default;
  using Columns = std::unordered_map<SurfaceCell, std::vector<int>, SurfaceCellHash>;
  bool headroomFree(int x, int y, int z, int height) const;
  Columns occupied_;
  Columns inferred_;
  struct OccupiedLevel { int z; std::uint64_t xy; };
  std::unordered_map<SurfaceCell, std::vector<OccupiedLevel>, SurfaceCellHash> blocks_;
  std::size_t observed_surfaces_{0};
  std::size_t inferred_surfaces_{0};
  double resolution_{0.0};
  SurfaceStats stats_;
};

}  // namespace lingtu::maps::layers
