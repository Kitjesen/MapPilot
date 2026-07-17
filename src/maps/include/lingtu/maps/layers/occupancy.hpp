#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/cloud.hpp"

namespace lingtu::maps::layers {

struct OccupancyGridView {
  std::string frame_id{"map"};
  float resolution_m{0.2F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  std::vector<std::int8_t> cells;
};

class OccupancyLayer {
 public:
  virtual ~OccupancyLayer() = default;
  virtual void Reset() = 0;
  virtual void Update(const MapCloudFrame& frame) = 0;
  virtual OccupancyGridView SnapshotGrid() const = 0;
};

}  // namespace lingtu::maps::layers
