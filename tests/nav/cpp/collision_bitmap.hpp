#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "planning/local/planner.hpp"

namespace lingtu::nav::tests {

class CollisionBitmap {
 public:
  CollisionBitmap(nav_kernel::Vec3 minimum = {-5.0, -5.0, -2.0},
                  nav_kernel::Vec3 maximum = {5.0, 5.0, 2.0}, double resolution = 0.1)
      : minimum_(minimum), maximum_(maximum), resolution_(resolution),
        size_x_(dimension(maximum.x - minimum.x)),
        size_y_(dimension(maximum.y - minimum.y)),
        size_z_(dimension(maximum.z - minimum.z)),
        bits_((cellCount() + 7U) / 8U, 0U) {}

  void clear() { std::fill(bits_.begin(), bits_.end(), 0U); }

  void occupy(const nav_kernel::Vec3 &point) {
    const int x = index(point.x, minimum_.x, size_x_);
    const int y = index(point.y, minimum_.y, size_y_);
    const int z = index(point.z, minimum_.z, size_z_);
    if (x < 0 || y < 0 || z < 0) return;
    const std::size_t linear =
        (static_cast<std::size_t>(z) * static_cast<std::size_t>(size_y_) +
         static_cast<std::size_t>(y)) *
            static_cast<std::size_t>(size_x_) +
        static_cast<std::size_t>(x);
    bits_[linear / 8U] |= static_cast<std::uint8_t>(1U << (linear % 8U));
  }

  void occupyPoints(const std::vector<float> &xyz) {
    for (std::size_t offset = 0U; offset + 2U < xyz.size(); offset += 3U) {
      occupy(nav_kernel::Vec3{xyz[offset], xyz[offset + 1U], xyz[offset + 2U]});
    }
  }

  void occupyInflated(const std::vector<float> &xyz, double radius,
                      double below, double above) {
    const int xy_cells = static_cast<int>(std::ceil(radius / resolution_));
    const int down_cells = static_cast<int>(std::ceil(below / resolution_));
    const int up_cells = static_cast<int>(std::ceil(above / resolution_));
    for (std::size_t offset = 0U; offset + 2U < xyz.size(); offset += 3U) {
      const nav_kernel::Vec3 center{xyz[offset], xyz[offset + 1U], xyz[offset + 2U]};
      for (int z = -down_cells; z <= up_cells; ++z) {
        for (int y = -xy_cells; y <= xy_cells; ++y) {
          for (int x = -xy_cells; x <= xy_cells; ++x) {
            const double dx = static_cast<double>(x) * resolution_;
            const double dy = static_cast<double>(y) * resolution_;
            if (dx * dx + dy * dy <= radius * radius + 1e-9) {
              occupy(nav_kernel::Vec3{
                  center.x + dx, center.y + dy,
                  center.z + static_cast<double>(z) * resolution_});
            }
          }
        }
      }
    }
  }

  [[nodiscard]] nav_kernel::LocalCollisionMapView view(
      double stamp_s = 1.0, std::uint64_t generation = 1U) const {
    nav_kernel::LocalCollisionMapView result;
    result.inflatedBits = bits_.data();
    result.inflatedBytes = bits_.size();
    result.sizeX = size_x_;
    result.sizeY = size_y_;
    result.sizeZ = size_z_;
    result.resolution = resolution_;
    result.aabbMin = minimum_;
    result.aabbMax = maximum_;
    result.resetEpoch = 1U;
    result.observationSequence = generation;
    result.generation = generation;
    result.stampS = stamp_s;
    result.receiveStampS = stamp_s;
    result.complete = true;
    result.live = true;
    return result;
  }

 private:
  [[nodiscard]] int dimension(double extent) const {
    return static_cast<int>(std::llround(extent / resolution_));
  }

  [[nodiscard]] int index(double value, double minimum, int size) const {
    const int result = static_cast<int>(std::floor((value - minimum) / resolution_));
    return result >= 0 && result < size ? result : -1;
  }

  [[nodiscard]] std::size_t cellCount() const {
    return static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_) *
           static_cast<std::size_t>(size_z_);
  }

  nav_kernel::Vec3 minimum_{};
  nav_kernel::Vec3 maximum_{};
  double resolution_{0.1};
  int size_x_{0};
  int size_y_{0};
  int size_z_{0};
  std::vector<std::uint8_t> bits_;
};

}  // namespace lingtu::nav::tests
