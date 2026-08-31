// SCAN rolling-map and collision semantics, ported from upstream 348e8a5.
// Modified for LingTu's Mapd collision snapshot and ROS-free query API.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_kernel::local::scan {
namespace {

double squared(double value) {
  return value * value;
}

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool footprintIntersectsCell(double center_x, double center_y, double c, double s,
                             double half_length, double half_width, double cell_x, double cell_y,
                             double cell_half) {
  constexpr double kOverlapEpsilon = 1e-9;
  const double dx = cell_x - center_x;
  const double dy = cell_y - center_y;
  const double abs_c = std::abs(c);
  const double abs_s = std::abs(s);
  if (std::abs(dx) >= cell_half + half_length * abs_c + half_width * abs_s - kOverlapEpsilon) {
    return false;
  }
  if (std::abs(dy) >= cell_half + half_length * abs_s + half_width * abs_c - kOverlapEpsilon) {
    return false;
  }
  if (std::abs(dx * c + dy * s) >= half_length + cell_half * (abs_c + abs_s) - kOverlapEpsilon) {
    return false;
  }
  if (std::abs(-dx * s + dy * c) >= half_width + cell_half * (abs_s + abs_c) - kOverlapEpsilon) {
    return false;
  }
  return true;
}

Vec3 projectSegment(const Vec3 &point, const Vec3 &a, const Vec3 &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double length_sq = dx * dx + dy * dy + dz * dz;
  if (length_sq <= 1e-12)
    return a;
  const double t = std::clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz) / length_sq, 0.0, 1.0);
  return {a.x + t * dx, a.y + t * dy, a.z + t * dz};
}

double squaredDistance(const Vec3 &a, const Vec3 &b) {
  return squared(a.x - b.x) + squared(a.y - b.y) + squared(a.z - b.z);
}

}  // namespace

Grid::Grid(const LocalPlannerParams &params, const LocalPlanRequest &input)
    : params_(params),
      vehicle_(input.robot.pose),
      route_(),
      collision_(input.environment.collision),
      traversability_(input.environment.traversability) {
  const LocalRouteView *route = input.route();
  const auto &collision = input.environment.collision;
  if (route == nullptr || !route->valid() || !finitePoint(input.robot.pose.position)) {
    reason_ = "route_invalid";
    return;
  }
  route_.assign(route->points, route->points + route->count);
  if (std::any_of(route_.begin(), route_.end(),
                  [](const Vec3 &point) { return !finitePoint(point); })) {
    route_.clear();
    reason_ = "route_nonfinite";
    return;
  }
  if (params.checkObstacle && !collision.present()) {
    route_.clear();
    reason_ = "collision_map_missing";
    return;
  }
  resolution_ = std::clamp(params.scan.voxelResolution, 0.05, 0.50);
  const double horizontal_range = std::clamp(params.scan.horizontalRange, 1.0, 12.0);
  double min_z = input.robot.pose.position.z;
  double max_z = input.robot.pose.position.z;
  for (const auto &point : route_) {
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  const double vertical_margin = std::clamp(params.scan.verticalMargin, 0.20, 3.0);
  const double requested_min_z =
      min_z - vertical_margin - std::max(0.0, params.scan.bodyClearanceBelow);
  const double requested_max_z =
      max_z + vertical_margin + std::max(0.0, params.scan.bodyClearanceAbove);
  const double vertical_range = std::max(resolution_, requested_max_z - requested_min_z);
  const double vertical_center = 0.5 * (requested_min_z + requested_max_z);
  const Vec3 &local_target = route_.back();
  const double horizontal_center_x =
      0.5 * (input.robot.pose.position.x + local_target.x);
  const double horizontal_center_y =
      0.5 * (input.robot.pose.position.y + local_target.y);
  origin_ = {
      horizontal_center_x - horizontal_range,
      horizontal_center_y - horizontal_range,
      vertical_center - 0.5 * vertical_range,
  };
  const double collision_resolution = collision.resolution;
  const bool collision_resolution_valid =
      std::isfinite(collision_resolution) && collision_resolution > 0.0;
  const double lattice_ratio =
      collision_resolution_valid
          ? std::max(collision_resolution, resolution_) /
                std::min(collision_resolution, resolution_)
          : std::numeric_limits<double>::infinity();
  const double source_yaw = collision.hasBox ? collision.boxYaw : 0.0;
  const double quarter_turn_error =
      std::abs(std::remainder(source_yaw, 0.5 * M_PI));
  const bool source_axis_aligned = quarter_turn_error <= 1e-9;
  const bool commensurate_lattices =
      std::isfinite(lattice_ratio) &&
      std::abs(lattice_ratio - std::round(lattice_ratio)) <= 1e-9 &&
      source_axis_aligned;
  if (collision.present() &&
      collision_resolution_valid &&
      finitePoint(collision.aabbMin) && commensurate_lattices) {
    // Mapd publishes voxel centres on a lattice rooted at aabbMin. Aligning
    // this temporary query window to that lattice keeps one 5 cm source voxel
    // in one planner cell instead of conservatively rasterizing it into as many
    // as 2 x 2 x 2 cells before footprint inflation.
    const auto align_down = [this](double value, double lattice_origin) {
      return lattice_origin +
             std::floor((value - lattice_origin) / resolution_ + 1e-9) *
                 resolution_;
    };
    origin_.x = align_down(origin_.x, collision.aabbMin.x);
    origin_.y = align_down(origin_.y, collision.aabbMin.y);
    origin_.z = align_down(origin_.z, collision.aabbMin.z);
  }
  nx_ = static_cast<int>(std::ceil(2.0 * horizontal_range / resolution_)) + 1;
  ny_ = nx_;
  nz_ = static_cast<int>(std::ceil(vertical_range / resolution_)) +
        1;
  if (nx_ <= 0 || ny_ <= 0 || nz_ <= 0) {
    reason_ = "grid_dimensions_invalid";
    return;
  }
  // Projected A* changes X/Y only, but every collision query remains three
  // dimensional. This mirrors upstream SCAN's inflated 3D occupancy buffer.
  const std::size_t cells =
      static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_);
  if (cells == 0 || cells > 8'000'000U) {
    nx_ = ny_ = nz_ = 0;
    reason_ = "grid_capacity_exceeded";
    return;
  }
  occupied_.assign((cells + 63U) / 64U, 0U);
  const double inflation_radius =
      std::max(0.05, params.scan.cylinderRadius) + std::max(0.0, params.footprintPadding);
  const double inflation_z_up = std::max(0.0, params.scan.inflationZUp);
  const double inflation_z_down = std::max(0.0, params.scan.inflationZDown);
  std::vector<int> occupied_voxels;
  occupied_voxels.reserve(static_cast<std::size_t>(std::max(0, collision.occupiedCount)));
  const auto mark_obstacle_voxel = [&](const Vec3 &center, const Vec3 &source_half) {
    const double boundary_epsilon =
        (commensurate_lattices ? 1e-5 : 1e-9) *
        std::max(resolution_, collision_resolution);
    GridIndex lower =
        index({center.x - source_half.x + boundary_epsilon,
               center.y - source_half.y + boundary_epsilon,
               center.z - source_half.z + boundary_epsilon});
    GridIndex upper =
        index({center.x + source_half.x - boundary_epsilon,
               center.y + source_half.y - boundary_epsilon,
               center.z + source_half.z - boundary_epsilon});
    lower.x = std::max(0, lower.x);
    lower.y = std::max(0, lower.y);
    lower.z = std::max(0, lower.z);
    upper.x = std::min(nx_ - 1, upper.x);
    upper.y = std::min(ny_ - 1, upper.y);
    upper.z = std::min(nz_ - 1, upper.z);
    if (lower.x > upper.x || lower.y > upper.y || lower.z > upper.z) {
      return false;
    }
    for (int z = lower.z; z <= upper.z; ++z) {
      for (int y = lower.y; y <= upper.y; ++y) {
        for (int x = lower.x; x <= upper.x; ++x) {
          const int voxel = linear({x, y, z});
          if (markOccupied(voxel)) {
            ++occupied_cell_count_;
            occupied_voxels.push_back(voxel);
          }
        }
      }
    }
    return true;
  };

  if (params.checkObstacle && collision.present()) {
    const bool structural_valid =
        collision.occupiedCount >= 0 &&
        (collision.occupiedCount == 0 || collision.occupiedXyz != nullptr) &&
        std::isfinite(collision.resolution) && collision.resolution > 0.0 &&
        finitePoint(collision.aabbMin) && finitePoint(collision.aabbMax) &&
        collision.aabbMin.x < collision.aabbMax.x && collision.aabbMin.y < collision.aabbMax.y &&
        collision.aabbMin.z < collision.aabbMax.z && collision.resetEpoch > 0U &&
        collision.observationSequence > 0U && collision.generation > 0U &&
        std::isfinite(collision.stampS) && collision.stampS > 0.0 &&
        std::isfinite(collision.receiveStampS) && collision.receiveStampS > 0.0;
    if (!structural_valid) {
      occupied_.clear();
      reason_ = "collision_map_invalid";
      return;
    }
    if (!collision.complete) {
      occupied_.clear();
      reason_ = "collision_map_incomplete";
      return;
    }
    if (!collision.live) {
      occupied_.clear();
      reason_ = "collision_map_not_live";
      return;
    }
    const double age = input.clock.timestampS - collision.receiveStampS;
    if (!std::isfinite(age) || age < -0.10 || age > std::max(0.10, params.scan.collisionMaxAge)) {
      occupied_.clear();
      reason_ = "collision_map_stale";
      return;
    }
    for (int i = 0; i < collision.occupiedCount; ++i) {
      const float *raw = collision.occupiedXyz + i * 3;
      const Vec3 obstacle{raw[0], raw[1], raw[2]};
      if (!finitePoint(obstacle)) {
        occupied_.clear();
        reason_ = "collision_map_nonfinite";
        return;
      }
      const double route_z = nearestRoutePoint(obstacle).z;
      const double support_limit =
          route_z - std::max(0.0, params.scan.bodyClearanceBelow) + 0.5 * collision.resolution;
      if (obstacle.z <= support_limit + 1e-9) {
        continue;
      }
      const double half = 0.5 * collision.resolution;
      const double rotated_half_xy =
          half * (std::abs(std::cos(source_yaw)) + std::abs(std::sin(source_yaw)));
      if (mark_obstacle_voxel(obstacle,
                              {rotated_half_xy, rotated_half_xy, half})) {
        ++collision_point_count_;
      }
    }
  }

  const int xy_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
  const int z_cells_up = static_cast<int>(std::ceil(inflation_z_up / resolution_));
  const int z_cells_down = static_cast<int>(std::ceil(inflation_z_down / resolution_));
  std::vector<int> x_reach(static_cast<std::size_t>(2 * xy_cells + 1), 0);
  const double inflation_radius_squared = squared(inflation_radius);
  for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
    const double y_squared = squared(static_cast<double>(dy) * resolution_);
    if (y_squared > inflation_radius_squared + 1e-9) {
      x_reach[static_cast<std::size_t>(dy + xy_cells)] = -1;
      continue;
    }
    const double remaining = std::max(0.0, inflation_radius_squared - y_squared);
    x_reach[static_cast<std::size_t>(dy + xy_cells)] =
        static_cast<int>(std::floor(std::sqrt(remaining) / resolution_ + 1e-9));
  }

  // The structuring element is an XY disk extruded through a Z interval. For
  // each affected row, record only the inclusive X interval endpoints, then
  // materialize the union with one prefix scan. This preserves the exact
  // lattice footprint of the former offset loop without random writes for
  // every disk cell around every source voxel.
  std::vector<std::vector<GridIndex>> occupied_by_z(static_cast<std::size_t>(nz_));
  for (const int linear_index : occupied_voxels) {
    const GridIndex obstacle = indexFromLinear(linear_index);
    occupied_by_z[static_cast<std::size_t>(obstacle.z)].push_back(obstacle);
  }
  const std::size_t row_stride = static_cast<std::size_t>(nx_ + 1);
  std::vector<int> row_differences(
      row_stride * static_cast<std::size_t>(ny_), 0);
  for (int z = 0; z < nz_; ++z) {
    std::fill(row_differences.begin(), row_differences.end(), 0);
    const int first_source_z = std::max(0, z - z_cells_up);
    const int last_source_z = std::min(nz_ - 1, z + z_cells_down);
    for (int source_z = first_source_z; source_z <= last_source_z;
         ++source_z) {
      for (const GridIndex &obstacle :
           occupied_by_z[static_cast<std::size_t>(source_z)]) {
        for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
          const int y = obstacle.y + dy;
          if (y < 0 || y >= ny_)
            continue;
          const int reach = x_reach[static_cast<std::size_t>(dy + xy_cells)];
          if (reach < 0)
            continue;
          const int first_x = std::max(0, obstacle.x - reach);
          const int last_x = std::min(nx_ - 1, obstacle.x + reach);
          const std::size_t row = static_cast<std::size_t>(y) * row_stride;
          ++row_differences[row + static_cast<std::size_t>(first_x)];
          --row_differences[row + static_cast<std::size_t>(last_x + 1)];
        }
      }
    }
    for (int y = 0; y < ny_; ++y) {
      const std::size_t row = static_cast<std::size_t>(y) * row_stride;
      int coverage = 0;
      for (int x = 0; x < nx_; ++x) {
        coverage += row_differences[row + static_cast<std::size_t>(x)];
        if (coverage > 0 && markOccupied(linear({x, y, z})))
          ++occupied_cell_count_;
      }
    }
  }
  reason_ = "ready";
}

bool Grid::valid() const noexcept {
  return !route_.empty() && nx_ > 0 && ny_ > 0 && nz_ > 0 &&
         occupied_.size() == (static_cast<std::size_t>(cellCount()) + 63U) / 64U;
}

const std::string &Grid::reason() const noexcept {
  return reason_;
}

double Grid::resolution() const noexcept {
  return resolution_;
}

int Grid::sizeX() const noexcept {
  return nx_;
}

int Grid::sizeY() const noexcept {
  return ny_;
}

int Grid::sizeZ() const noexcept {
  return nz_;
}

int Grid::cellCount() const noexcept {
  return nx_ * ny_ * nz_;
}

int Grid::occupiedCellCount() const noexcept {
  return occupied_cell_count_;
}

int Grid::collisionPointCount() const noexcept {
  return collision_point_count_;
}

int Grid::linear(const GridIndex &value) const noexcept {
  return (value.z * ny_ + value.y) * nx_ + value.x;
}

GridIndex Grid::indexFromLinear(int value) const noexcept {
  GridIndex out;
  out.x = value % nx_;
  value /= nx_;
  out.y = value % ny_;
  value /= ny_;
  out.z = value;
  return out;
}

bool Grid::contains(const GridIndex &value) const noexcept {
  return value.x >= 0 && value.x < nx_ && value.y >= 0 && value.y < ny_ && value.z >= 0 &&
         value.z < nz_;
}

bool Grid::markOccupied(int linear_index) noexcept {
  if (linear_index < 0 || linear_index >= cellCount()) {
    return false;
  }
  const std::size_t value = static_cast<std::size_t>(linear_index);
  std::uint64_t &word = occupied_[value >> 6U];
  const std::uint64_t mask = std::uint64_t{1} << (value & 63U);
  if ((word & mask) != 0U) {
    return false;
  }
  word |= mask;
  return true;
}

bool Grid::occupiedLinear(int linear_index) const noexcept {
  if (linear_index < 0 || linear_index >= cellCount()) {
    return true;
  }
  const std::size_t value = static_cast<std::size_t>(linear_index);
  return (occupied_[value >> 6U] & (std::uint64_t{1} << (value & 63U))) != 0U;
}

GridIndex Grid::index(const Vec3 &value) const noexcept {
  return {
      static_cast<int>(std::floor((value.x - origin_.x) / resolution_)),
      static_cast<int>(std::floor((value.y - origin_.y) / resolution_)),
      static_cast<int>(std::floor((value.z - origin_.z) / resolution_)),
  };
}

Vec3 Grid::point(const GridIndex &value) const noexcept {
  return {
      origin_.x + (static_cast<double>(value.x) + 0.5) * resolution_,
      origin_.y + (static_cast<double>(value.y) + 0.5) * resolution_,
      origin_.z + (static_cast<double>(value.z) + 0.5) * resolution_,
  };
}

bool Grid::occupiedCell(const GridIndex &value) const noexcept {
  return !contains(value) || occupiedLinear(linear(value));
}

bool Grid::traversabilityCellInsideInitialFootprint(double cellX, double cellY,
                                                    double cellHalf) const {
  const double half_length = std::max(0.1, params_.vehicleLength * 0.5 + params_.footprintPadding);
  const double half_width = std::max(0.1, params_.vehicleWidth * 0.5 + params_.footprintPadding);
  const double c = std::cos(vehicle_.yaw);
  const double s = std::sin(vehicle_.yaw);
  const double dx = cellX - vehicle_.position.x;
  const double dy = cellY - vehicle_.position.y;
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const double projected_cell_half = cellHalf * (std::abs(c) + std::abs(s));
  constexpr double kOverlapEpsilon = 1e-9;
  return std::abs(local_x) + projected_cell_half <= half_length + kOverlapEpsilon &&
         std::abs(local_y) + projected_cell_half <= half_width + kOverlapEpsilon;
}

bool Grid::traversabilityFootprintBlocked(const Vec3 &center, double yaw) const {
  if (!params_.useTraversabilityCost || !traversability_.valid())
    return false;
  if (!finitePoint(center) || !std::isfinite(yaw) || !std::isfinite(traversability_.originX) ||
      !std::isfinite(traversability_.originY) || !std::isfinite(params_.traversabilityHardCost)) {
    return true;
  }

  constexpr double kOverlapEpsilon = 1e-9;
  const double half_length = std::max(0.1, params_.vehicleLength * 0.5 + params_.footprintPadding);
  const double half_width = std::max(0.1, params_.vehicleWidth * 0.5 + params_.footprintPadding);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double extent_x = half_length * std::abs(c) + half_width * std::abs(s);
  const double extent_y = half_length * std::abs(s) + half_width * std::abs(c);
  const double min_x = center.x - extent_x;
  const double max_x = center.x + extent_x;
  const double min_y = center.y - extent_y;
  const double max_y = center.y + extent_y;
  const double grid_max_x = traversability_.originX +
                            static_cast<double>(traversability_.cols) * traversability_.resolution;
  const double grid_max_y = traversability_.originY +
                            static_cast<double>(traversability_.rows) * traversability_.resolution;
  if (min_x < traversability_.originX - kOverlapEpsilon ||
      min_y < traversability_.originY - kOverlapEpsilon || max_x > grid_max_x + kOverlapEpsilon ||
      max_y > grid_max_y + kOverlapEpsilon) {
    return true;
  }

  const int min_col = std::max(0, static_cast<int>(std::floor((min_x - traversability_.originX) /
                                                              traversability_.resolution)));
  const int max_col =
      std::min(traversability_.cols - 1,
               static_cast<int>(std::floor((max_x - traversability_.originX - kOverlapEpsilon) /
                                           traversability_.resolution)));
  const int min_row = std::max(0, static_cast<int>(std::floor((min_y - traversability_.originY) /
                                                              traversability_.resolution)));
  const int max_row =
      std::min(traversability_.rows - 1,
               static_cast<int>(std::floor((max_y - traversability_.originY - kOverlapEpsilon) /
                                           traversability_.resolution)));
  const double cell_half = traversability_.resolution * 0.5;
  for (int row = min_row; row <= max_row; ++row) {
    const double cell_y =
        traversability_.originY + (static_cast<double>(row) + 0.5) * traversability_.resolution;
    for (int col = min_col; col <= max_col; ++col) {
      const double cell_x =
          traversability_.originX + (static_cast<double>(col) + 0.5) * traversability_.resolution;
      if (!footprintIntersectsCell(center.x, center.y, c, s, half_length, half_width, cell_x,
                                   cell_y, cell_half) ||
          traversabilityCellInsideInitialFootprint(cell_x, cell_y, cell_half)) {
        continue;
      }
      const float risk = traversability_.values[row * traversability_.cols + col];
      if (!std::isfinite(risk) || risk < 0.0F || risk > 100.0F ||
          risk >= static_cast<float>(params_.traversabilityHardCost)) {
        return true;
      }
    }
  }
  return false;
}

bool Grid::obstacleFree(const GridIndex &value, double yaw) const {
  if (!contains(value))
    return false;
  return obstacleFree(point(value), yaw);
}

bool Grid::obstacleFree(const Vec3 &center, double yaw) const {
  if (!contains(index(center)))
    return false;
  const double separation = std::max(0.0, params_.scan.cylinderOffset);
  const double radius = std::max(0.05, params_.scan.cylinderRadius) +
                        std::max(0.0, params_.footprintPadding);
  const double below = std::max(0.0, params_.scan.bodyClearanceBelow);
  const double above = std::max(0.0, params_.scan.bodyClearanceAbove);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  for (double sign : {-1.0, 1.0}) {
    const Vec3 cylinder{
        center.x + sign * separation * c,
        center.y + sign * separation * s,
        center.z,
    };
    if ((params_.checkObstacle &&
         !collision_.coversCylinder(cylinder, radius, below, above)) ||
        occupiedCell(index(cylinder)))
      return false;
  }
  return true;
}

bool Grid::hypothesisFree(const Vec3 &center, double yaw) const {
  if (!contains(index(center)) || !routeHeightAllowed(center))
    return false;
  const double separation = std::max(0.0, params_.scan.cylinderOffset);
  const double radius = std::max(0.05, params_.scan.cylinderRadius) +
                        std::max(0.0, params_.footprintPadding);
  const double below = std::max(0.0, params_.scan.bodyClearanceBelow);
  const double above = std::max(0.0, params_.scan.bodyClearanceAbove);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (double sign : {-1.0, 1.0}) {
    const GridIndex cylinder = index({
        center.x + sign * separation * c,
        center.y + sign * separation * s,
        center.z,
    });
    const Vec3 cylinder_point{
        center.x + sign * separation * c,
        center.y + sign * separation * s,
        center.z,
    };
    const bool in_virtual_xy_layer = cylinder.x >= -1 && cylinder.x <= nx_ && cylinder.y >= -1 &&
                                     cylinder.y <= ny_ && cylinder.z >= 0 && cylinder.z < nz_;
    if ((params_.checkObstacle &&
         !collision_.coversCylinder(cylinder_point, radius, below, above)) ||
        !in_virtual_xy_layer ||
        (contains(cylinder) && occupiedCell(cylinder)))
      return false;
  }
  return true;
}

bool Grid::free(const GridIndex &value, double yaw) const {
  if (!contains(value))
    return false;
  return free(point(value), yaw);
}

bool Grid::free(const Vec3 &center, double yaw) const {
  if (!contains(index(center)))
    return false;
  if (!routeHeightAllowed(center) || traversabilityFootprintBlocked(center, yaw)) {
    return false;
  }
  return obstacleFree(center, yaw);
}

bool Grid::segmentFree(const Vec3 &from, const Vec3 &to) const {
  const double distance = distance3D(from, to);
  if (!std::isfinite(distance))
    return false;
  const double horizontal = std::hypot(to.x - from.x, to.y - from.y);
  const double vertical = std::abs(to.z - from.z);
  if (vertical > 1e-6 &&
      (horizontal <= 1e-6 || vertical / horizontal > std::max(0.0, params_.scan.maxSlope))) {
    return false;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / (resolution_ * 0.5))));
  const double yaw = std::atan2(to.y - from.y, to.x - from.x);
  // The caller has already admitted the segment start (the robot's current
  // pose for the first edge, or the previous edge's checked endpoint).
  for (int step = 1; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Vec3 sample{
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t,
        from.z + (to.z - from.z) * t,
    };
    if (!free(sample, yaw))
      return false;
  }
  return true;
}

Vec3 Grid::nearestRoutePoint(const Vec3 &value) const {
  if (route_.empty())
    return {};
  Vec3 best = route_.front();
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < route_.size(); ++i) {
    const Vec3 projected = projectSegment(value, route_[i], route_[i + 1]);
    const double distance_squared = squaredDistance(value, projected);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best = projected;
    }
  }
  return best;
}

bool Grid::routeHeightAllowed(const Vec3 &value) const {
  const Vec3 guide = nearestRoutePoint(value);
  return std::abs(value.z - guide.z) <= std::max(resolution_, params_.scan.routeZTolerance);
}

double Grid::routeDistance(const Vec3 &value) const {
  return distance3D(value, nearestRoutePoint(value));
}

const std::vector<Vec3> &Grid::route() const noexcept {
  return route_;
}

}  // namespace nav_kernel::local::scan
