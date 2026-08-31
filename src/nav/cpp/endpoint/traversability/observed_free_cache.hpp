#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include "traversability/observed_safety_grid.hpp"

namespace lingtu::nav::endpoint {

class ObservedFreeCache {
 public:
  explicit ObservedFreeCache(double resolution_m) : resolution_m_(std::max(0.01, resolution_m)) {}

  void observeRay(double origin_x, double origin_y, double hit_x, double hit_y, double stamp_s,
                  double max_range_m) {
    if (!std::isfinite(origin_x) || !std::isfinite(origin_y) || !std::isfinite(hit_x) ||
        !std::isfinite(hit_y) || !std::isfinite(stamp_s) || stamp_s <= 0.0) {
      return;
    }
    const double dx = hit_x - origin_x;
    const double dy = hit_y - origin_y;
    const double range = std::hypot(dx, dy);
    if (!std::isfinite(range)) {
      return;
    }
    const double traversed = max_range_m > 0.0 ? std::min(range, max_range_m) : range;
    const double step_m = std::max(0.01, resolution_m_ * 0.5);
    const int steps = std::max(1, static_cast<int>(std::ceil(traversed / step_m)));
    for (int step = 0; step <= steps; ++step) {
      const double distance = traversed * static_cast<double>(step) / static_cast<double>(steps);
      const double ratio = range > 1e-12 ? distance / range : 0.0;
      const CellKey key = keyFor(origin_x + ratio * dx, origin_y + ratio * dy);
      auto [it, inserted] = observed_at_s_.emplace(key, stamp_s);
      if (!inserted) {
        it->second = std::max(it->second, stamp_s);
      }
    }
  }

  void prune(double now_s, double ttl_s) {
    if (!std::isfinite(now_s) || now_s <= 0.0) {
      return;
    }
    const double cutoff = now_s - std::max(0.0, ttl_s);
    for (auto it = observed_at_s_.begin(); it != observed_at_s_.end();) {
      if (!std::isfinite(it->second) || it->second < cutoff) {
        it = observed_at_s_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::size_t apply(lingtu::maps::layers::Grid2D &grid, double now_s, double ttl_s,
                    float free_cost = 0.0F, double inflation_radius_m = 0.0) {
    prune(now_s, ttl_s);
    const double inflation_radius = std::max(0.0, inflation_radius_m);
    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution_m_));
    const double inflation_squared = inflation_radius * inflation_radius;
    std::size_t applied = 0;
    for (const auto &[key, stamp_s] : observed_at_s_) {
      (void)stamp_s;
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          const double offset_x = static_cast<double>(dx) * resolution_m_;
          const double offset_y = static_cast<double>(dy) * resolution_m_;
          if (inflation_radius > 0.0 &&
              offset_x * offset_x + offset_y * offset_y > inflation_squared + 1e-12) {
            continue;
          }
          const double x = (static_cast<double>(key.x + dx) + 0.5) * resolution_m_;
          const double y = (static_cast<double>(key.y + dy) + 0.5) * resolution_m_;
          int row = -1;
          int col = -1;
          if (!safetyGridCell(grid, x, y, row, col)) {
            continue;
          }
          const auto index = static_cast<std::size_t>(grid.index(row, col));
          if (grid.data[index] == free_cost) {
            continue;
          }
          grid.data[index] = free_cost;
          ++applied;
        }
      }
    }
    return applied;
  }

  void clear() { observed_at_s_.clear(); }
  std::size_t size() const { return observed_at_s_.size(); }

 private:
  struct CellKey {
    std::int64_t x{0};
    std::int64_t y{0};

    bool operator==(const CellKey &other) const { return x == other.x && y == other.y; }
  };

  struct CellKeyHash {
    std::size_t operator()(const CellKey &key) const {
      const auto x = static_cast<std::uint64_t>(key.x);
      const auto y = static_cast<std::uint64_t>(key.y);
      return static_cast<std::size_t>((x * 0x9e3779b97f4a7c15ULL) ^ (y + 0x517cc1b727220a95ULL));
    }
  };

  CellKey keyFor(double x, double y) const {
    return {
        observedLatticeIndex(x),
        observedLatticeIndex(y),
    };
  }

  std::int64_t observedLatticeIndex(double coordinate) const {
    // Registered-cloud coordinates are float32, so snap their representation noise.
    const double normalized = coordinate / resolution_m_;
    const double boundary = std::nearbyint(normalized);
    const double tolerance = 8.0 * static_cast<double>(std::numeric_limits<float>::epsilon()) *
                             std::max(1.0, std::fabs(normalized));
    return static_cast<std::int64_t>(
        std::floor(std::fabs(normalized - boundary) <= tolerance ? boundary : normalized));
  }

  double resolution_m_{0.2};
  std::unordered_map<CellKey, double, CellKeyHash> observed_at_s_;
};

}  // namespace lingtu::nav::endpoint
