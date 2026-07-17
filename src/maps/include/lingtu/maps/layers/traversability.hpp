#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/layers/elevation.hpp"
#include "lingtu/maps/layers/esdf.hpp"
#include "lingtu/maps/layers/occupancy.hpp"

namespace lingtu::maps::layers {

struct TraversabilityGridView {
  std::string frame_id{"map"};
  float resolution_m{0.2F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  std::vector<std::uint8_t> cost;
};

class TraversabilityLayer {
 public:
  virtual ~TraversabilityLayer() = default;
  virtual void Reset() = 0;
  virtual void Update(
      const OccupancyGridView& occupancy,
      const ElevationGridView* elevation,
      const EsdfGridView* esdf) = 0;
  virtual TraversabilityGridView SnapshotGrid() const = 0;
};

}  // namespace lingtu::maps::layers
