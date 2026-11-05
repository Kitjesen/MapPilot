#pragma once

#include "collision/terrain.hpp"

namespace lingtu::nav::collision {

// Occupied-cell geometry shared by saved-map and live-map adapters.
// Terrain evidence always belongs to the source of the tested occupied cell.
class PartQuery {
 public:
  PartQuery(const Part &part, const Pose &pose, double margin,
            double horizontal_margin, double ground_height)
      : geometry(preparePart(part, pose, margin, horizontal_margin)),
        part_(part), pose_(pose), margin_(margin),
        horizontal_margin_(horizontal_margin), ground_height_(ground_height) {}

  bool collides(const Aabb &cell, const Terrain *terrain = nullptr) {
    if (!intersects(geometry, cell) || allowsGroundContact(geometry, cell)) return false;
    if (!terrain) return true;
    if (terrain->hitBoundsClear(geometry, cell)) return false;
    const double resolution = 2 * cell.half_size.x;
    if (surface_terrain_ != terrain || surface_resolution_ != resolution) {
      surface_ = partTerrainSurface(terrain, part_, pose_, ground_height_, resolution,
                                    margin_, horizontal_margin_);
      surface_terrain_ = terrain;
      surface_resolution_ = resolution;
    }
    return !surface_ || !terrainCellClear(*terrain, part_, pose_, margin_, cell,
                                         *surface_, horizontal_margin_);
  }

  const PreparedPart geometry;

 private:
  const Part &part_;
  const Pose pose_;
  const double margin_;
  const double horizontal_margin_;
  const double ground_height_;
  const Terrain *surface_terrain_{nullptr};
  double surface_resolution_{0.0};
  std::optional<TerrainSurface> surface_;
};

}  // namespace lingtu::nav::collision
