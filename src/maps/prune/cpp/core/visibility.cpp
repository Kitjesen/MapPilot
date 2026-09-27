#include "core/visibility.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/evidence.hpp"

namespace lingtu::map_cleaning {
namespace {
double distanceSquared(const PointXYZI& a, const PointXYZI& b) {
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

PointXYZI inverseTransformPoint(const PointXYZI& point, const Pose& pose) {
  const double norm =
      std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz);
  const double qw = norm > 1e-12 ? pose.qw / norm : 1.0;
  const double qx = norm > 1e-12 ? -pose.qx / norm : 0.0;
  const double qy = norm > 1e-12 ? -pose.qy / norm : 0.0;
  const double qz = norm > 1e-12 ? -pose.qz / norm : 0.0;
  const PointXYZI translated{
      static_cast<float>(point.x - pose.tx),
      static_cast<float>(point.y - pose.ty),
      static_cast<float>(point.z - pose.tz),
      point.intensity,
  };
  return transformPoint(translated, {0, 0, 0, qw, qx, qy, qz});
}
}  // namespace

VisibilityEvidence::VisibilityEvidence(const std::vector<PointXYZI>& points,
    const StaticCleanerOptions& options)
    : points_(points), options_(options), free_frames_(points.size()), seen_(points.size()),
      hit_frames_(points.size()), ground_protected_(points.size()),
      occupied_stamp_(points.size()), free_stamp_(points.size()) {
  for (std::size_t i = 0; i < points.size(); ++i)
    cells_[voxelKey(points[i], options_.voxel_size_m)].push_back(i);
}

void VisibilityEvidence::observe(const std::vector<PointXYZI>& scan, const Pose& pose) {
  const auto& xyz = *options_.sensor_origin;
  const auto origin = transformPoint({xyz[0], xyz[1], xyz[2]}, pose);
  ++frame_;
  occupied_.clear();
  free_.clear();
  const double voxel = options_.voxel_size_m;
  const double margin = options_.endpoint_margin_m;
  const double confirm_radius = options_.endpoint_confirm_radius_m;
  const double tube = options_.ray_tolerance_m;
  const int neighbors = static_cast<int>(std::ceil(confirm_radius / voxel));
  for (const auto& local : scan) {
    const auto endpoint = transformPoint(local, pose);
    const auto key = voxelKey(endpoint, options_.voxel_size_m);
    // Protect measured surfaces even when quantization puts them in a neighbor cell.
    for (int x = -neighbors; x <= neighbors; ++x)
      for (int y = -neighbors; y <= neighbors; ++y)
        for (int z = -neighbors; z <= neighbors; ++z) {
          const auto found = cells_.find({key.x + x, key.y + y, key.z + z});
          if (found == cells_.end()) continue;
          for (const auto index : found->second)
            if (occupied_stamp_[index] != frame_ &&
                distanceSquared(points_[index], endpoint) <= confirm_radius * confirm_radius) {
              occupied_stamp_[index] = frame_;
              occupied_.push_back(index);
              if (!ground_protected_[index] &&
                  inverseTransformPoint(points_[index], pose).z <= options_.ground_z_threshold)
                ground_protected_[index] = true;
            }
        }

    const double length = std::sqrt(distanceSquared(origin, endpoint));
    if (length <= 2 * margin) continue;
    const double direction[3] = {(endpoint.x - origin.x) / length,
        (endpoint.y - origin.y) / length, (endpoint.z - origin.z) / length};
    const double start[3] = {origin.x, origin.y, origin.z};
    const double end = std::min(length - margin, static_cast<double>(options_.max_ray_length_m));
    auto cell = voxelKey(origin, options_.voxel_size_m);
    int coordinate[3] = {cell.x, cell.y, cell.z};
    int step[3];
    double next[3], delta[3];
    for (int axis = 0; axis < 3; ++axis) {
      step[axis] = direction[axis] > 0 ? 1 : -1;
      if (std::abs(direction[axis]) < 1e-12) {
        next[axis] = delta[axis] = std::numeric_limits<double>::infinity();
      } else {
        const double boundary = (coordinate[axis] + (step[axis] > 0 ? 1 : 0)) * voxel;
        next[axis] = (boundary - start[axis]) / direction[axis];
        delta[axis] = voxel / std::abs(direction[axis]);
      }
    }
    // Traverse only cells hit by a measured ray. No return means no free evidence.
    double travelled = 0;
    while (travelled < end) {
      const auto axis = static_cast<int>(std::min_element(next, next + 3) - next);
      const double segment_end = std::min(next[axis], end);
      int lower[3], upper[3];
      for (int a = 0; a < 3; ++a) {
        const double from = start[a] + travelled * direction[a];
        const double to = start[a] + segment_end * direction[a];
        lower[a] = static_cast<int>(std::floor((std::min(from, to) - tube) / voxel));
        upper[a] = static_cast<int>(std::floor((std::max(from, to) + tube) / voxel));
      }
      // Include the tube on either side of a voxel boundary, then check exact distance.
      for (int x = lower[0]; x <= upper[0]; ++x)
        for (int y = lower[1]; y <= upper[1]; ++y)
          for (int z = lower[2]; z <= upper[2]; ++z) {
            const auto found = cells_.find({x, y, z});
            if (found == cells_.end()) continue;
            for (const auto index : found->second) {
              if (free_stamp_[index] == frame_) continue;
              const auto& point = points_[index];
              const double offset[3] = {point.x - origin.x, point.y - origin.y, point.z - origin.z};
              const double along = offset[0] * direction[0] + offset[1] * direction[1] + offset[2] * direction[2];
              const double cross2 = std::max(0.0, distanceSquared(point, origin) - along * along);
              if (along > margin && along < end && cross2 <= tube * tube) {
                free_stamp_[index] = frame_;
                free_.push_back(index);
              }
            }
          }
      travelled = next[axis];
      coordinate[axis] += step[axis];
      next[axis] += delta[axis];
    }
  }
  // Occupied endpoints win over free rays in the same frame; one vote per frame.
  for (const auto index : occupied_) {
    seen_[index] = true;
    free_frames_[index] = 0;
    if (hit_frames_[index] < 2) ++hit_frames_[index];
  }
  for (const auto index : free_)
    if (seen_[index] && occupied_stamp_[index] != frame_ &&
        free_frames_[index] < options_.min_free_frames)
      ++free_frames_[index];
}

bool VisibilityEvidence::contradicted(std::size_t index) const {
  return seen_[index] && free_frames_[index] >= options_.min_free_frames;
}

bool VisibilityEvidence::groundProtected(std::size_t index) const {
  return ground_protected_[index];
}

const std::vector<bool>& VisibilityEvidence::groundEvidence() const {
  return ground_protected_;
}

bool VisibilityEvidence::onSupportedSurface(std::size_t index) const {
  // Only persistent neighbors without later free evidence may protect a surface.
  // A supported plane is a preservation rule, never a new deletion condition.
  constexpr double radius = 0.50, plane_tolerance = 0.04;
  const auto& point = points_[index];
  const auto key = voxelKey(point, options_.voxel_size_m);
  const int count = static_cast<int>(std::ceil(radius / options_.voxel_size_m));
  std::vector<std::size_t> neighbors;
  for (int x = -count; x <= count; ++x)
    for (int y = -count; y <= count; ++y)
      for (int z = -count; z <= count; ++z) {
        const auto found = cells_.find({key.x + x, key.y + y, key.z + z});
        if (found == cells_.end()) continue;
        for (const auto other : found->second)
          if (other != index && hit_frames_[other] >= 2 && free_frames_[other] == 0 &&
              distanceSquared(point, points_[other]) <= radius * radius)
            neighbors.push_back(other);
      }
  if (neighbors.size() < 6) return false;
  const auto anchor_index = *std::min_element(neighbors.begin(), neighbors.end(),
      [&](auto a, auto b) { return distanceSquared(point, points_[a]) < distanceSquared(point, points_[b]); });
  const auto& anchor = points_[anchor_index];
  const auto farthest = *std::max_element(neighbors.begin(), neighbors.end(),
      [&](auto a, auto b) { return distanceSquared(anchor, points_[a]) < distanceSquared(anchor, points_[b]); });
  const auto& edge_end = points_[farthest];
  const double edge[3] = {edge_end.x - anchor.x, edge_end.y - anchor.y, edge_end.z - anchor.z};
  const double edge2 = distanceSquared(anchor, edge_end);
  if (edge2 < 0.04) return false;
  double normal[3]{}, area2 = 0;
  for (const auto other : neighbors) {
    const auto& p = points_[other];
    const double offset[3] = {p.x - anchor.x, p.y - anchor.y, p.z - anchor.z};
    const double cross[3] = {edge[1] * offset[2] - edge[2] * offset[1],
        edge[2] * offset[0] - edge[0] * offset[2], edge[0] * offset[1] - edge[1] * offset[0]};
    const double norm2 = cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2];
    if (norm2 > area2) { area2 = norm2; std::copy(cross, cross + 3, normal); }
  }
  // Reject line-like support: require at least 10 cm of spread off the first edge.
  if (area2 < edge2 * 0.01) return false;
  for (auto& component : normal) component /= std::sqrt(area2);
  const auto plane_distance = [&](const PointXYZI& p) {
    return std::abs((p.x - anchor.x) * normal[0] + (p.y - anchor.y) * normal[1] +
        (p.z - anchor.z) * normal[2]);
  };
  if (plane_distance(point) > plane_tolerance) return false;
  const auto supported = std::count_if(neighbors.begin(), neighbors.end(),
      [&](auto other) { return plane_distance(points_[other]) <= plane_tolerance; });
  return static_cast<std::size_t>(supported) * 5 >= neighbors.size() * 4;
}
}  // namespace lingtu::map_cleaning
