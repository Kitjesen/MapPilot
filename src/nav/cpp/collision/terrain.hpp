#pragma once

#include <cstddef>
#include <functional>
#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

#include "collision/body.hpp"

namespace lingtu::nav::collision {

// Map-frame ground height near the requested layer; distinct from measured collision hits.
using SupportQuery = std::function<std::optional<double>(const Vec3 &, double)>;

struct TerrainSurface {
  Vec3 normal{0.0, 0.0, 1.0};
  double offset_m{0.0};
  Aabb observed_bounds{};
  double rms_m{0.0};
  double max_positive_residual_m{0.0};

  [[nodiscard]] double heightAt(double x, double y) const;
};

// Immutable hit positions or conservative per-cell hit bounds, never cell centres.
// Surfaces describe local geometry; the caller owns robot slope/contact limits.
class Terrain {
 public:
  Terrain(std::vector<Vec3> points, double sample_resolution_m);
  // Each bound must enclose every retained hit in its occupancy cell.
  Terrain(std::vector<Aabb> hit_bounds, double sample_resolution_m);

  [[nodiscard]] double resolution() const noexcept { return resolution_m_; }
  [[nodiscard]] const std::vector<Aabb> *hitBounds() const noexcept {
    return bounded_hits_ ? &samples_ : nullptr;
  }
  [[nodiscard]] std::optional<TerrainSurface> surface(
      const Vec3 &expected, double height_tolerance_m, double radius_m) const;
  // Height estimate from measured hit-bound midpoints, not a claim that all
  // enclosed hits lie on this plane. Classification only, never contact exemption.
  [[nodiscard]] std::optional<TerrainSurface> classificationSurface(
      const Vec3 &expected, double height_tolerance_m, double radius_m) const;
  // A fitted surface can replace a cell's cube only when that cell contains
  // original hits and every one belongs to this same finite surface patch.
  [[nodiscard]] bool explains(const Aabb &occupied, const TerrainSurface &surface,
                              double *max_positive_residual_m = nullptr) const;
  // Missing hit bounds leave the original voxel collision unchanged.
  [[nodiscard]] bool hitBoundsClear(const PreparedPart &part, const Aabb &occupied) const;

 private:
  struct Cell {
    int x, y, z;
    bool operator==(const Cell &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CellHash {
    std::size_t operator()(const Cell &cell) const;
  };
  [[nodiscard]] Cell cell(const Vec3 &point) const;
  [[nodiscard]] Cell hitCell(const Vec3 &point) const;
  [[nodiscard]] std::optional<TerrainSurface> fitSurface(
      const Vec3 &expected, double height_tolerance_m, double radius_m,
      double residual_tolerance_m, bool classification = false) const;

  double resolution_m_;
  bool bounded_hits_{false};
  std::vector<Aabb> samples_;
  std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells_;
  std::unordered_multimap<Cell, std::size_t, CellHash> hit_cells_;
};

// Exact source-primitive support against an upward unit plane. Padding matches
// Body: local box axes, cylinder radius/half-length, sphere/capsule radius.
[[nodiscard]] double primitivePlaneClearance(
    const Part &part, const Pose &pose, const TerrainSurface &surface, double margin = 0.0);

inline double nominalGroundHeight(const Body &body, const Pose &pose) {
  double height = pose.position.z;
  bool found = false;
  for (const auto &part : body.parts) if (part.support_contact) {
    const double bottom = primitiveLowestZ(part, pose);
    if (!found || bottom < height) height = bottom;
    found = true;
  }
  return height;
}

inline std::optional<TerrainSurface> partTerrainSurface(
    const Terrain *terrain, const Part &part, const Pose &pose,
    double ground_height, double resolution, double margin, double horizontal_margin) {
  if (!terrain) return {};
  const auto primitive = primitiveBounds(part, pose);
  const auto collision_bounds = partBounds(part, pose, margin, horizontal_margin);
  const double expected = part.support_contact ? primitiveLowestZ(part, pose) : ground_height;
  return terrain->surface({primitive.center.x, primitive.center.y, expected},
      resolution + .03, std::max(2 * resolution,
          std::hypot(collision_bounds.half_size.x, collision_bounds.half_size.y) + resolution));
}

inline bool terrainCellClear(const Terrain &terrain, const Part &part,
    const Pose &pose, double margin, const Aabb &cell, const TerrainSurface &surface,
    double horizontal_margin = 0.0) {
  double rise = 0;
  if (!terrain.explains(cell, surface, &rise)) return false;
  if (part.support_contact && primitivePlaneClearance(part, pose, surface) - rise >= -1e-12)
    return true;
  return primitivePlaneClearance(part, pose, surface, margin) - rise -
      horizontal_margin * (std::abs(surface.normal.x) + std::abs(surface.normal.y)) > 1e-12;
}

}  // namespace lingtu::nav::collision
