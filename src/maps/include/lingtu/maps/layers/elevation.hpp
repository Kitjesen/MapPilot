#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/cloud.hpp"

namespace lingtu::maps::layers {

struct ElevationGridView {
  std::string frame_id{"map"};
  float resolution_m{0.2F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  std::vector<float> min_z_m;
  std::vector<float> max_z_m;
  std::vector<std::uint8_t> valid;
};

class ElevationLayer {
 public:
  virtual ~ElevationLayer() = default;
  virtual void Reset() = 0;
  virtual void Update(const MapCloudFrame& frame) = 0;
  virtual ElevationGridView SnapshotGrid() const = 0;
};

}  // namespace lingtu::maps::layers
