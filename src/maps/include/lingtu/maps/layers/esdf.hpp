#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/layers/occupancy.hpp"

namespace lingtu::maps::layers {

struct EsdfGridView {
  std::string frame_id{"map"};
  float resolution_m{0.2F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  std::vector<float> distance_m;
  std::vector<float> grad_x;
  std::vector<float> grad_y;
};

class EsdfLayer {
 public:
  virtual ~EsdfLayer() = default;
  virtual void Reset() = 0;
  virtual void Update(const OccupancyGridView& occupancy) = 0;
  virtual EsdfGridView SnapshotGrid() const = 0;
};

}  // namespace lingtu::maps::layers
