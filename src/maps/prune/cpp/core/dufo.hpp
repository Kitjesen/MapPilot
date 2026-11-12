#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/types.hpp"

namespace lingtu::map_cleaning {

class DufoMap {
 public:
  explicit DufoMap(double resolution_m);
  ~DufoMap();

  void observe(const std::vector<PointXYZI>& scan, const Pose& pose,
               const std::array<float, 3>& sensor_origin);
  std::vector<std::uint8_t> removeMask(const std::vector<PointXYZI>& source);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::map_cleaning
