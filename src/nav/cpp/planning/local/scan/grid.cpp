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

}  // namespace

Grid::Grid(const LocalPlannerParams &params, const LocalPlanInput &input)
    : params_(params), vehicle_(input.vehicle), traversability_(input.traversability) {
  if (!input.route.valid() || !finitePoint(input.vehicle.position)) {
    reason_ = "route_invalid";
    return;
  }
  route_.assign(input.route.points, input.route.points + input.route.count);
  if (std::any_of(route_.begin(), route_.end(),
                  [](const Vec3 &point) { return !finitePoint(point); })) {
    route_.clear();
    reason_ = "route_nonfinite";
    return;
  }
  resolution_ = std::clamp(params.scan.voxelResolution, 0.05, 0.50);
  const double horizontal_range = std::clamp(params.scan.horizontalRange, 1.0, 12.0);
  double min_z = input.vehicle.position.z;
  double max_z = input.vehicle.position.z;
  for (const auto &point : route_) {
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  const double vertical_margin = std::clamp(params.scan.verticalMargin, 0.20, 3.0);
  origin_ = {
      input.vehicle.position.x - horizontal_range,
      input.vehicle.position.y - horizontal_range,
      min_z - vertical_margin - std::max(0.0, params.scan.bodyClearanceBelow),
  };
  nx_ = static_cast<int>(std::ceil(2.0 * horizontal_range / resolution_)) + 1;
  ny_ = nx_;
  nz_ = static_cast<int>(std::ceil((max_z - min_z + 2.0 * vertical_margin +
                                    std::max(0.0, params.scan.bodyClearanceBelow) +
                                    std::max(0.0, params.scan.bodyClearanceAbove)) /
                                   resolution_)) +
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
  occupied_.assign(cells, 0U);
  const double inflation_radius =
      std::max(0.05, params.scan.cylinderRadius) + std::max(0.0, params.footprintPadding);
  const double inflation_z_up = std::max(0.0, params.scan.inflationZUp);
  const double inflation_z_down = std::max(0.0, params.scan.inflationZDown);
  const Vec3 grid_max{
      origin_.x + static_cast<double>(nx_) * resolution_,
      origin_.y + static_cast<double>(ny_) * resolution_,
      origin_.z + static_cast<double>(nz_) * resolution_,
  };
  std::vector<int> occupied_voxels;
  occupied_voxels.reserve(static_cast<std::size_t>(std::max(0, input.collision.occupiedCount)));
  const auto mark_obstacle_voxel = [&](const Vec3 &center, double source_resolution) {
    const double half = 0.5 * std::max(1e-6, source_resolution);
    const double upper_epsilon = 1e-9 * std::max(1.0, resolution_);
    GridIndex lower = index({center.x - half, center.y - half, center.z - half});
    GridIndex upper = index({center.x + half - upper_epsilon, center.y + half - upper_epsilon,
                             center.z + half - upper_epsilon});
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
          auto &occupied = occupied_[static_cast<std::size_t>(voxel)];
          if (occupied == 0U) {
            occupied = 1U;
            occupied_voxels.push_back(voxel);
          }
        }
      }
    }
    return true;
  };

  if (params.checkObstacle && input.collision.present()) {
    const auto &collision = input.collision;
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
    const double age = input.timestampS - collision.receiveStampS;
    if (!std::isfinite(age) || age < -0.10 || age > std::max(0.10, params.scan.collisionMaxAge)) {
      occupied_.clear();
      reason_ = "collision_map_stale";
      return;
    }
    const double tolerance = std::max(resolution_, collision.resolution);
    if (origin_.x < collision.aabbMin.x - tolerance ||
        origin_.y < collision.aabbMin.y - tolerance ||
        origin_.z < collision.aabbMin.z - tolerance ||
        grid_max.x > collision.aabbMax.x + tolerance ||
        grid_max.y > collision.aabbMax.y + tolerance ||
        grid_max.z > collision.aabbMax.z + tolerance) {
      occupied_.clear();
      reason_ = "collision_map_roi_uncovered";
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
      if (mark_obstacle_voxel(obstacle, collision.resolution)) {
        ++collision_point_count_;
      }
    }
  }

  // A complete Mapd collision snapshot is the authoritative obstacle layer.
  // Re-fusing the registered-cloud fallback here double-inflates the same
  // geometry and makes narrow passages appear/disappear with LiDAR sampling.
  // Keep the legacy points only for development inputs that do not provide
  // Mapd collision data at all.
  if (params.checkObstacle && !input.collision.present() && input.obstacles.xyzh != nullptr &&
      input.obstacles.count > 0) {
    for (int i = 0; i < input.obstacles.count; ++i) {
      const float *raw = input.obstacles.xyzh + i * 4;
      const Vec3 obstacle{raw[0], raw[1], raw[2]};
      const double height = raw[3];
      if (!finitePoint(obstacle) || !std::isfinite(height))
        continue;
      if (params.useTerrainAnalysis &&
          (height < params.obstacleHeightThre || height > params.obstacleHeightMax)) {
        continue;
      }
      const double route_z = nearestRoutePoint(obstacle).z;
      const double support_limit =
          route_z - std::max(0.0, params.scan.bodyClearanceBelow) + 0.5 * resolution_;
      if (obstacle.z <= support_limit + 1e-9) {
        continue;
      }
      if (mark_obstacle_voxel(obstacle, resolution_)) {
        ++legacy_obstacle_point_count_;
      }
    }
  }

  const int xy_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
  const int z_cells_up = static_cast<int>(std::ceil(inflation_z_up / resolution_));
  const int z_cells_down = static_cast<int>(std::ceil(inflation_z_down / resolution_));
  std::vector<unsigned char> inflated = occupied_;
  for (const int linear_index : occupied_voxels) {
    const GridIndex obstacle = indexFromLinear(linear_index);
    for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
      for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
        if (squared(static_cast<double>(dx) * resolution_) +
                squared(static_cast<double>(dy) * resolution_) >
            squared(inflation_radius) + 1e-9) {
          continue;
        }
        for (int dz = -z_cells_down; dz <= z_cells_up; ++dz) {
          const GridIndex candidate{obstacle.x + dx, obstacle.y + dy, obstacle.z + dz};
          if (contains(candidate)) {
            inflated[static_cast<std::size_t>(linear(candidate))] = 1U;
          }
        }
      }
    }
  }
  occupied_.swap(inflated);
  occupied_cell_count_ = static_cast<int>(std::count(occupied_.begin(), occupied_.end(), 1U));
  reason_ = "ready";
}

bool Grid::valid() const noexcept {
  return !route_.empty() && nx_ > 0 && ny_ > 0 && nz_ > 0 &&
         occupied_.size() == static_cast<std::size_t>(cellCount());
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

int Grid::legacyObstaclePointCount() const noexcept {
  return legacy_obstacle_point_count_;
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
  return !contains(value) || occupied_[static_cast<std::size_t>(linear(value))] != 0U;
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
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  for (double sign : {-1.0, 1.0}) {
    const Vec3 cylinder{
        center.x + sign * separation * c,
        center.y + sign * separation * s,
        center.z,
    };
    if (occupiedCell(index(cylinder)))
      return false;
  }
  return true;
}

bool Grid::hypothesisFree(const Vec3 &center, double yaw) const {
  if (!contains(index(center)) || !routeHeightAllowed(center))
    return false;
  const double separation = std::max(0.0, params_.scan.cylinderOffset);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (double sign : {-1.0, 1.0}) {
    const GridIndex cylinder = index({
        center.x + sign * separation * c,
        center.y + sign * separation * s,
        center.z,
    });
    const bool in_virtual_xy_layer = cylinder.x >= -1 && cylinder.x <= nx_ && cylinder.y >= -1 &&
                                     cylinder.y <= ny_ && cylinder.z >= 0 && cylinder.z < nz_;
    if (!in_virtual_xy_layer || (contains(cylinder) && occupiedCell(cylinder)))
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
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < route_.size(); ++i) {
    const Vec3 projected = projectSegment(value, route_[i], route_[i + 1]);
    const double distance = distance3D(value, projected);
    if (distance < best_distance) {
      best_distance = distance;
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
