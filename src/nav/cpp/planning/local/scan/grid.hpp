#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

struct GridIndex {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex &other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

class Grid {
 public:
  Grid(const LocalPlannerParams &params, const LocalPlanRequest &input);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const std::string &reason() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] int sizeX() const noexcept;
  [[nodiscard]] int sizeY() const noexcept;
  [[nodiscard]] int sizeZ() const noexcept;
  [[nodiscard]] int cellCount() const noexcept;
  [[nodiscard]] int occupiedCellCount() const noexcept;
  [[nodiscard]] int collisionPointCount() const noexcept;
  [[nodiscard]] int linear(const GridIndex &index) const noexcept;
  [[nodiscard]] GridIndex indexFromLinear(int linear) const noexcept;
  [[nodiscard]] bool contains(const GridIndex &index) const noexcept;
  [[nodiscard]] GridIndex index(const Vec3 &point) const noexcept;
  [[nodiscard]] Vec3 point(const GridIndex &index) const noexcept;

  [[nodiscard]] bool obstacleFree(const GridIndex &index, double yaw) const;
  [[nodiscard]] bool obstacleFree(const Vec3 &center, double yaw) const;
  [[nodiscard]] bool hypothesisFree(const Vec3 &center, double yaw) const;
  [[nodiscard]] bool free(const GridIndex &index, double yaw) const;
  [[nodiscard]] bool free(const Vec3 &center, double yaw) const;
  [[nodiscard]] bool segmentFree(const Vec3 &from, const Vec3 &to) const;
  [[nodiscard]] bool routeHeightAllowed(const Vec3 &point) const;
  [[nodiscard]] double routeDistance(const Vec3 &point) const;
  [[nodiscard]] Vec3 nearestRoutePoint(const Vec3 &point) const;
  [[nodiscard]] const std::vector<Vec3> &route() const noexcept;

 private:
  [[nodiscard]] bool markOccupied(int linearIndex) noexcept;
  [[nodiscard]] bool occupiedLinear(int linearIndex) const noexcept;
  [[nodiscard]] bool occupiedCell(const GridIndex &index) const noexcept;
  [[nodiscard]] bool traversabilityFootprintBlocked(const Vec3 &center, double yaw) const;
  [[nodiscard]] bool traversabilityCellInsideInitialFootprint(double cellX, double cellY,
                                                              double cellHalf) const;

  const LocalPlannerParams &params_;
  Pose vehicle_{};
  std::vector<Vec3> route_;
  LocalCollisionMapView collision_{};
  std::vector<std::uint64_t> occupied_;
  LocalTraversabilityView traversability_{};
  Vec3 origin_{};
  int nx_{0};
  int ny_{0};
  int nz_{0};
  double resolution_{0.0};
  int collision_point_count_{0};
  int occupied_cell_count_{0};
  std::string reason_{"grid_invalid"};
};

}  // namespace nav_kernel::local::scan
