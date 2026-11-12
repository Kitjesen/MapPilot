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
    nav_kernel::Vec3 body_target = route.front();
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
  std::size_t nearest_segment = first_segment;
  nav_kernel::Vec3 projection = projectSegment(
      map_body.position,
      route[first_segment],
      route[first_segment + 1]);
  double nearest_distance =
      nav_kernel::distance3D(map_body.position, projection);
  for (std::size_t index = first_segment + 1;
       index + 1 < route.size(); ++index) {
    const nav_kernel::Vec3 vertex = route[index];
    const nav_kernel::Vec3 candidate = projectSegment(
        map_body.position, vertex, route[index + 1]);
    const double distance =
        nav_kernel::distance3D(map_body.position, candidate);
    const bool closer = distance < nearest_distance - 1e-9;
    const bool forward_vertex_tie =
        std::abs(distance - nearest_distance) <= 1e-9 &&
        nav_kernel::distance3D(map_body.position, vertex) <=
            std::max(0.01, config_.waypoint_reached_m);
    if (closer || forward_vertex_tie) {
      nearest_segment = index;
      projection = candidate;
      nearest_distance = distance;
    }
  }
  progress = nearest_segment;
  // The measured start joins the route horizontally; a height discrepancy
  // must not insert an in-place vertical leg. Forward route heights stay intact.
  projection.z = map_body.position.z;
  const nav_kernel::Vec3 projection_planning =
      map_from_odom.odomPointFromMap(projection);
  const double projection_distance=nav_kernel::distance3D(segment.back(),projection_planning);
  // A pose-stage boundary already reached within its declared tolerance does
  // not need a millimetre-scale sideways return before the forward reference.
  // Keep the measured first point and check the resulting candidate in 3D.
  const bool reached_pose_projection=!pose_stages_.empty() &&
      projection_distance<=active_goal_reached_m_;
  if (projection_distance>1e-4 && !reached_pose_projection) {
    segment.push_back(projection_planning);
  }

  double remaining = std::max(0.05, config_.corridor_lookahead_m);
  nav_kernel::Vec3 cursor = projection;
  for (std::size_t index = nearest_segment + 1;
       index < route.size(); ++index) {
    const nav_kernel::Vec3 endpoint = route[index];
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
          map_from_odom.odomPointFromMap(target.point);
      if (nav_kernel::distance3D(
              segment.back(), planning_target) > 1e-4) {
        segment.push_back(planning_target);
      }
      return target;
    }
    if (length > 1e-9) {
      const nav_kernel::Vec3 point =
          map_from_odom.odomPointFromMap(endpoint);
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
  target.point = route.back();
  target.reachesGoal = true;
  return target;
}

void Executor::buildReference(
    const SegmentTarget& target,
    const MapFromOdomTransform& map_from_odom) {
  // Route activation, suspension and frame-epoch reset clear this snapshot.
  // Progress changes only the short segment, not the full map-frame reference.
  if (config_.planning_frame == PlanningFrame::Map && !reference.empty()) return;
  reference = segment;
  // A moving terminal spline still needs a two-point reference at the endpoint.
  if (target.reachesGoal && reference.size() < 2 && route.size() > 1) {
    reference.clear();
    for (std::size_t index = route.size() - 2; index < route.size(); ++index) {
      auto point = route[index];
      reference.push_back(map_from_odom.odomPointFromMap(point));
    }
  }
  if (target.reachesGoal || route.empty()) return;

  const std::size_t first = std::min(target.index, route.size() - 1);
  for (std::size_t index = first; index < route.size(); ++index) {
    nav_kernel::Vec3 point = route[index];
    point = map_from_odom.odomPointFromMap(point);
    if (reference.empty() ||
        nav_kernel::distance3D(reference.back(), point) > 1e-4) {
      reference.push_back(point);
    }
  }
}

void Executor::applyLocalGuide(
    const nav_kernel::Pose& map_body,
    const nav_kernel::Pose& planning_body,
    const MapFromOdomTransform& map_from_odom,
    double timestamp_s) {
  const double commitment_timeout_s =
      std::max(0.5, config_.recovery.blocked_interval_s);
  const bool commitment_expired =
      guide_stamp_s_ < 0.0 || !std::isfinite(timestamp_s) ||
      timestamp_s < guide_stamp_s_ ||
      timestamp_s - guide_stamp_s_ > commitment_timeout_s;
  if (commitment_expired) {
    guide_map_.clear();
  }
  if (guide_generation_ != generation ||
      guide_map_.size() < 2 || segment.size() < 2) {
    return;
  }

  const double minimum_guide_distance = std::clamp(
      1.5 * std::max(0.0, config_.follower.baseLookAheadDis),
      0.30,
      std::max(0.30, 0.5 * config_.corridor_lookahead_m));
  const auto closest = std::min_element(
      guide_map_.begin(), guide_map_.end(),
      [&](const nav_kernel::Vec3& left, const nav_kernel::Vec3& right) {
        return nav_kernel::distance3D(map_body.position, left) <
               nav_kernel::distance3D(map_body.position, right);
      });
  if (closest == guide_map_.end() ||
      std::next(closest) == guide_map_.end()) {
    return;
  }
  const nav_kernel::Vec3* guide_map = nullptr;
  for (auto point = std::next(closest);
       point != guide_map_.end(); ++point) {
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

bool Executor::atGoal(const nav_kernel::Pose& body) const {
  if (route.empty()) return false;
  const bool task_goal = requested_goal_ &&
      (pose_stages_.empty() || pose_stage_ + 1 == pose_stages_.size());
  const auto& goal = task_goal ? *requested_goal_ : route.back();
  return std::hypot(goal.x - body.position.x, goal.y - body.position.y) <= active_goal_reached_m_ &&
         std::abs(goal.z - body.position.z) <= config_.goal_height_tolerance_m;
}

bool TraversabilityGridView::valid() const {
  return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
}


std::vector<nav_kernel::Vec3> Executor::planningPathToBody(
    const nav_kernel::Pose &body,
    const std::vector<nav_kernel::Vec3> &planningPath) {
  std::vector<nav_kernel::Vec3> result;
  result.reserve(planningPath.size());
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  for (const auto &point : planningPath) {
    const double dx = point.x - body.position.x;
    const double dy = point.y - body.position.y;
    result.push_back({c * dx + s * dy, -s * dx + c * dy,
                      point.z - body.position.z});
  }
  return result;
}


std::vector<nav_kernel::Vec3> Executor::planningPathToMap(
    const lingtu::nav::navigation::MapFromOdomTransform &mapFromOdom,
    const std::vector<nav_kernel::Vec3> &planningPath) {
  std::vector<nav_kernel::Vec3> result;
  result.reserve(planningPath.size());
  for (const auto &point : planningPath)
    result.push_back(mapFromOdom.mapPointFromOdom(point));
  return result;
}


nav_kernel::LocalPlanRequest Executor::makeLocalPlanRequest(
    const nav_kernel::Pose &vehicle, const std::vector<nav_kernel::Vec3> &route,
    const std::vector<nav_kernel::Vec3> *reference_route,
    std::uint64_t route_generation,
    nav_kernel::LocalKinematicState kinematics, const ExecutionObservation &observation,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
    bool execution_frozen, const TraversabilityGridView &traversability,
    bool include_traversability,
    const nav_kernel::LocalMotionIntent *intent) {
  nav_kernel::LocalRouteView route_view{
      route.empty() ? nullptr : route.data(),
      static_cast<int>(route.size()),
      route_generation,
  };
  nav_kernel::LocalPlanRequest request;
  request.robot = {vehicle, kinematics};
  request.objective = intent != nullptr
                          ? nav_kernel::LocalObjective{
                                nav_kernel::MotionIntentTarget{*intent, route_view}}
                          : nav_kernel::LocalObjective{nav_kernel::RouteTarget{route_view}};
  if (reference_route != nullptr && reference_route->size() >= 2U) {
    request.reference = {
        reference_route->data(),
        static_cast<int>(reference_route->size()),
        route_generation,
    };
  }
  request.identity = {
      observation.frame_epoch,
      observation.collision.present() ? observation.collision.generation
                                      : observation.cloud_generation,
      include_traversability ? observation.traversability_generation : 0,
  };
  request.environment.obstacles = {obstacle_xyzh, obstacle_count};
  request.environment.collision = observation.collision;
  request.environment.predictions = observation.predictions;
  request.environment.no_go_regions = observation.no_go_regions;
  request.clock = {timestamp_s, execution_frozen, observation.clock_mode};
  if (include_traversability && traversability.valid()) {
    request.environment.traversability = {
        traversability.values, traversability.rows, traversability.cols,
        traversability.resolution, traversability.origin_x, traversability.origin_y,
    };
  }
  return request;
}


nav_kernel::LocalPlan Executor::planLocal(
    const nav_kernel::LocalPlanRequest &request,
    const MapFromOdomTransform &map_from_odom,
    nav_kernel::LocalPlannerDebugSnapshot *debug) {
  nav_kernel::LocalPlan plan = local_planner_.plan(request);
  nav_kernel::Pose map_body;
  map_body.position = map_from_odom.mapPointFromOdom(request.robot.pose.position);
  map_body.yaw = nav_kernel::normalizeAngle(map_from_odom.yaw + request.robot.pose.yaw);
  *debug = debugSnapshotToMap(map_body, local_planner_.debugSnapshot());
  if (debug->collisionEvidence && config_.planning_frame == PlanningFrame::Odom) {
    auto &evidence = *debug->collisionEvidence;
    evidence.rejectedPosition = map_from_odom.mapPointFromOdom(evidence.rejectedPosition);
    if (evidence.measuredPoints) {
      auto points = std::make_shared<std::vector<nav_kernel::Vec3>>(*evidence.measuredPoints);
      for (auto &point : *points) point = map_from_odom.mapPointFromOdom(point);
      evidence.measuredPoints = std::move(points);
    }
  }
  return plan;
}


ExecutionOutput Executor::tickRoute(const nav_kernel::Pose &map_body,
                                    const nav_kernel::Pose &odom_body,
                                    const MapFromOdomTransform &map_from_odom,
                                    const float *obstacle_xyzh_map, int obstacle_count,
                                    double timestamp_s, TraversabilityGridView odom_traversability,
                                    ExecutionObservation observation) {
  if (config_.planning_frame == PlanningFrame::Odom && !map_from_odom.valid()) {
    ExecutionOutput output;
    output.active = !route.empty();
    output.near_field_stop = output.active;
    output.reason = "invalid_map_from_odom";
    recovery_action_ = 0;
    recovery_attempt_ = -1;
    recovery_follower_.reset();
    velocity_stamp_s_ = -1.0;
    resetLocalPlanning(true);
    recovery_.reset();
    resetAutonomyProgress();
    return output;
  }

  const int safe_obstacle_count =
      obstacle_xyzh_map == nullptr ? 0 : std::max(0, obstacle_count);
  if (config_.planning_frame == PlanningFrame::Map) {
    if (reference_epoch && *reference_epoch != observation.frame_epoch) {
      resetMotion();
    }
    reference_epoch = observation.frame_epoch;
    // The occupancy layer and complete reference are already map-frame data.
    // The optional 2D traversability view is odom-only; do not relabel it.
    observation.collision.gridFromPlanningTranslation = {};
    observation.collision.gridFromPlanningYaw = 0.0;
    return tickInPlanningFrame(map_body, map_body, {}, obstacle_xyzh_map,
                               safe_obstacle_count, timestamp_s, {}, observation);
  }
  obstacle_xyzh_odom_scratch_.clear();
  predictions_odom_scratch_.clear();
  if (observation.predictions.fresh(timestamp_s)) {
    for (std::size_t i = 0; i < observation.predictions.count; ++i) {
      auto prediction = observation.predictions.obstacles[i];
      prediction.start = map_from_odom.odomPointFromMap(prediction.start);
      prediction.end = map_from_odom.odomPointFromMap(prediction.end);
      prediction.minZ -= map_from_odom.translation.z;
      prediction.maxZ -= map_from_odom.translation.z;
      predictions_odom_scratch_.push_back(prediction);
    }
  }
  observation.predictions.obstacles = predictions_odom_scratch_.data();
  observation.predictions.count = predictions_odom_scratch_.size();
  obstacle_xyzh_odom_scratch_.reserve(static_cast<std::size_t>(safe_obstacle_count) * 4U);
  for (int index = 0; index < safe_obstacle_count; ++index) {
    const nav_kernel::Vec3 point_map{
        obstacle_xyzh_map[index * 4],
        obstacle_xyzh_map[index * 4 + 1],
        obstacle_xyzh_map[index * 4 + 2],
    };
    const nav_kernel::Vec3 point_odom = map_from_odom.odomPointFromMap(point_map);
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.x));
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.y));
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.z));
    obstacle_xyzh_odom_scratch_.push_back(obstacle_xyzh_map[index * 4 + 3]);
  }

  observation.collision.gridFromPlanningTranslation = map_from_odom.translation;
  observation.collision.gridFromPlanningYaw = map_from_odom.yaw;

  return tickInPlanningFrame(
      map_body, odom_body, map_from_odom,
      obstacle_xyzh_odom_scratch_.empty() ? nullptr : obstacle_xyzh_odom_scratch_.data(),
      safe_obstacle_count, timestamp_s, odom_traversability, observation);
}


}  // namespace lingtu::nav::navigation
