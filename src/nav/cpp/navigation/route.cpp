#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace lingtu::nav::navigation {
namespace {

nav_kernel::Vec3 projectSegment(const nav_kernel::Vec3& point,
                                const nav_kernel::Vec3& from,
                                const nav_kernel::Vec3& to) {
  const nav_kernel::Vec3 delta{
      to.x - from.x,
      to.y - from.y,
      to.z - from.z,
  };
  const double length_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  if (length_squared <= 1e-12) return from;
  const double ratio = std::clamp(
      ((point.x - from.x) * delta.x +
       (point.y - from.y) * delta.y +
       (point.z - from.z) * delta.z) /
          length_squared,
      0.0,
      1.0);
  return {
      from.x + ratio * delta.x,
      from.y + ratio * delta.y,
      from.z + ratio * delta.z,
  };
}

}  // namespace

bool MapFromOdomTransform::valid() const {
  return std::isfinite(translation.x) && std::isfinite(translation.y) &&
         std::isfinite(translation.z) && std::isfinite(yaw);
}

nav_kernel::Vec3 MapFromOdomTransform::mapPointFromOdom(
    const nav_kernel::Vec3& point_odom) const {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {
      translation.x + c * point_odom.x - s * point_odom.y,
      translation.y + s * point_odom.x + c * point_odom.y,
      translation.z + point_odom.z,
  };
}

nav_kernel::Vec3 MapFromOdomTransform::odomPointFromMap(
    const nav_kernel::Vec3& point_map) const {
  const double dx = point_map.x - translation.x;
  const double dy = point_map.y - translation.y;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      point_map.z - translation.z,
  };
}

Executor::SegmentTarget Executor::buildSegment(
    const nav_kernel::Pose& map_body,
    const nav_kernel::Pose& planning_body,
    const MapFromOdomTransform& map_from_odom) {
  segment.clear();
  segment.push_back(planning_body.position);
  SegmentTarget target;
  if (route.empty()) return target;
  if (route.size() == 1) {
    if (!height_offset_) {
      height_offset_ = map_body.position.z - route.front().z;
    }
    nav_kernel::Vec3 body_target = route.front();
    body_target.z += *height_offset_;
    target.point = body_target;
    target.reachesGoal = true;
    const nav_kernel::Vec3 planning_target =
        map_from_odom.odomPointFromMap(body_target);
    if (nav_kernel::distance3D(segment.back(), planning_target) > 1e-4) {
      segment.push_back(planning_target);
    }
    return target;
  }

  const std::size_t first_segment =
      std::min(progress, route.size() - 2);
  const auto route_point = [&](std::size_t index, double offset) {
    nav_kernel::Vec3 point = route[index];
    point.z += offset;
    return point;
  };
  const auto nearest_projection = [&](double offset,
                                      std::size_t* nearest_segment,
                                      nav_kernel::Vec3* projection) {
    *nearest_segment = first_segment;
    *projection = projectSegment(
        map_body.position,
        route_point(first_segment, offset),
        route_point(first_segment + 1, offset));
    double nearest_distance =
        nav_kernel::distance3D(map_body.position, *projection);
    for (std::size_t index = first_segment + 1;
         index + 1 < route.size(); ++index) {
      const nav_kernel::Vec3 vertex = route_point(index, offset);
      const nav_kernel::Vec3 candidate = projectSegment(
          map_body.position, vertex, route_point(index + 1, offset));
      const double distance =
          nav_kernel::distance3D(map_body.position, candidate);
      const bool closer = distance < nearest_distance - 1e-9;
      const bool forward_vertex_tie =
          std::abs(distance - nearest_distance) <= 1e-9 &&
          nav_kernel::distance3D(map_body.position, vertex) <=
              std::max(0.01, config_.waypoint_reached_m);
      if (closer || forward_vertex_tie) {
        *nearest_segment = index;
        *projection = candidate;
        nearest_distance = distance;
      }
    }
  };

  std::size_t nearest_segment = first_segment;
  nav_kernel::Vec3 projection{};
  nearest_projection(height_offset_.value_or(0.0),
                     &nearest_segment, &projection);
  if (!height_offset_) {
    height_offset_ = map_body.position.z - projection.z;
    nearest_projection(*height_offset_, &nearest_segment, &projection);
  }
  progress = nearest_segment;

  // The first edge only connects the measured body pose to its projection on
  // the global route. Its vertical delta is body heave/local map quantization,
  // not a terrain slope. Anchor that connector at the measured body height;
  // all forward route points keep their planned elevation.
  projection.z = map_body.position.z;

  // Calibrate the planner route to the body once. Recomputing this offset on
  // every frame would erase ramps and floor changes whenever the robot lags
  // behind the requested elevation.
  const auto planning_route_point = [&](const nav_kernel::Vec3& point) {
    return map_from_odom.odomPointFromMap(point);
  };

  const nav_kernel::Vec3 projection_planning =
      planning_route_point(projection);
  if (nav_kernel::distance3D(
          segment.back(), projection_planning) > 1e-4) {
    segment.push_back(projection_planning);
  }

  double remaining = std::max(0.05, config_.corridor_lookahead_m);
  nav_kernel::Vec3 cursor = projection;
  for (std::size_t index = nearest_segment + 1;
       index < route.size(); ++index) {
    const nav_kernel::Vec3 endpoint = route_point(index, *height_offset_);
    // Lookahead is travel distance over the ground. Counting elevation in the
    // budget shortens the route segment exactly where a ramp or stair begins.
    const double length = std::hypot(endpoint.x - cursor.x,
                                     endpoint.y - cursor.y);
    if (length > 1e-9 && remaining < length - 1e-9) {
      const double ratio = remaining / length;
      target.index = index;
      target.point = {
          cursor.x + ratio * (endpoint.x - cursor.x),
          cursor.y + ratio * (endpoint.y - cursor.y),
          cursor.z + ratio * (endpoint.z - cursor.z),
      };
      const nav_kernel::Vec3 planning_target =
          planning_route_point(target.point);
      if (nav_kernel::distance3D(
              segment.back(), planning_target) > 1e-4) {
        segment.push_back(planning_target);
      }
      return target;
    }
    if (length > 1e-9) {
      const nav_kernel::Vec3 point =
          planning_route_point(endpoint);
      if (nav_kernel::distance3D(
              segment.back(), point) > 1e-4) {
        segment.push_back(point);
      }
      remaining -= length;
    }
    cursor = endpoint;
    target.index = index;
    target.point = endpoint;
    if (remaining <= 1e-9) {
      target.reachesGoal = index + 1 == route.size();
      return target;
    }
  }
  target.index = route.size() - 1;
  target.point = route_point(route.size() - 1, *height_offset_);
  target.reachesGoal = true;
  return target;
}

void Executor::applyCommittedLocalGuide(
    const nav_kernel::Pose& map_body,
    const nav_kernel::Pose& planning_body,
    const MapFromOdomTransform& map_from_odom,
    double timestamp_s) {
  const double commitment_timeout_s =
      std::max(0.5, config_.recovery.blocked_interval_s);
  const bool commitment_expired =
      committed_local_path_time_s_ < 0.0 || !std::isfinite(timestamp_s) ||
      timestamp_s < committed_local_path_time_s_ ||
      timestamp_s - committed_local_path_time_s_ > commitment_timeout_s;
  if (commitment_expired) {
    committed_local_path_map_.clear();
    committed_route_generation_ = 0;
    committed_local_path_time_s_ = -1.0;
  }
  if (committed_route_generation_ != generation ||
      committed_local_path_map_.size() < 2 || segment.size() < 2) {
    return;
  }

  const double minimum_guide_distance = std::clamp(
      1.5 * std::max(0.0, config_.follower.baseLookAheadDis),
      0.30,
      std::max(0.30, 0.5 * config_.corridor_lookahead_m));
  const auto closest = std::min_element(
      committed_local_path_map_.begin(), committed_local_path_map_.end(),
      [&](const nav_kernel::Vec3& left, const nav_kernel::Vec3& right) {
        return nav_kernel::distance3D(map_body.position, left) <
               nav_kernel::distance3D(map_body.position, right);
      });
  if (closest == committed_local_path_map_.end() ||
      std::next(closest) == committed_local_path_map_.end()) {
    return;
  }
  const nav_kernel::Vec3* guide_map = nullptr;
  for (auto point = std::next(closest);
       point != committed_local_path_map_.end(); ++point) {
    if (nav_kernel::distance3D(map_body.position, *point) >=
        minimum_guide_distance) {
      guide_map = &*point;
      break;
    }
  }
  if (guide_map == nullptr) {
    return;
  }

  const nav_kernel::Vec3 guide_planning =
      map_from_odom.odomPointFromMap(*guide_map);
  const nav_kernel::Vec3 route_direction{
      segment.back().x - planning_body.position.x,
      segment.back().y - planning_body.position.y,
      0.0,
  };
  const nav_kernel::Vec3 guide_direction{
      guide_planning.x - planning_body.position.x,
      guide_planning.y - planning_body.position.y,
      0.0,
  };
  if (route_direction.x * guide_direction.x +
          route_direction.y * guide_direction.y <=
      0.0) {
    return;
  }
  std::vector<nav_kernel::Vec3> guided;
  guided.reserve(segment.size() + 1);
  guided.push_back(planning_body.position);
  if (nav_kernel::distance3D(guided.back(), guide_planning) > 1e-4) {
    guided.push_back(guide_planning);
  }
  for (std::size_t index = 1; index < segment.size(); ++index) {
    if (nav_kernel::distance3D(guided.back(), segment[index]) > 1e-4) {
      guided.push_back(segment[index]);
    }
  }
  segment.swap(guided);
}

std::vector<nav_kernel::Vec3> Executor::bodyPathToMap(
    const nav_kernel::Pose& odom_map_body,
    const std::vector<nav_kernel::Vec3>& body_path) const {
  std::vector<nav_kernel::Vec3> out;
  out.reserve(body_path.size());
  const double c = std::cos(odom_map_body.yaw);
  const double s = std::sin(odom_map_body.yaw);
  for (const auto& point : body_path) {
    out.push_back({
        odom_map_body.position.x + c * point.x - s * point.y,
        odom_map_body.position.y + s * point.x + c * point.y,
        odom_map_body.position.z + point.z,
    });
  }
  return out;
}

nav_kernel::LocalPlannerDebugSnapshot Executor::debugSnapshotToMap(
    const nav_kernel::Pose& odom_map_body,
    nav_kernel::LocalPlannerDebugSnapshot snapshot) const {
  for (auto& candidate : snapshot.candidates) {
    candidate.path = bodyPathToMap(odom_map_body, candidate.path);
  }
  return snapshot;
}

bool Executor::atGoal(const nav_kernel::Pose& odom_map_body) const {
  if (route.empty()) return false;
  nav_kernel::Vec3 goal = route.back();
  goal.z += height_offset_.value_or(0.0);
  const double planar_distance =
      std::hypot(goal.x - odom_map_body.position.x, goal.y - odom_map_body.position.y);
  return planar_distance <= active_goal_reached_m_ &&
         std::abs(goal.z - odom_map_body.position.z) <= active_goal_height_tolerance_m_;
}

}  // namespace lingtu::nav::navigation
