#include "navigation/stairs.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

double along(const StairFlight& flight, const nav_kernel::Vec3& point) {
  return (point.x - flight.origin.x) * flight.axis.x +
         (point.y - flight.origin.y) * flight.axis.y;
}

double lateral(const StairFlight& flight, const nav_kernel::Vec3& point) {
  return -(point.x - flight.origin.x) * flight.axis.y +
         (point.y - flight.origin.y) * flight.axis.x;
}

nav_kernel::Vec3 interpolate(const nav_kernel::Vec3& a,
                            const nav_kernel::Vec3& b, double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
          a.z + (b.z - a.z) * t};
}

bool usable(const StairFlight& flight) {
  return flight.treads.size() >= 3 && flight.lateralMax > flight.lateralMin &&
         std::abs(std::hypot(flight.axis.x, flight.axis.y) - 1.0) < 1e-3 &&
         flight.treads.back().height > flight.treads.front().height &&
         flight.treads.back().begin > flight.treads.front().end;
}

bool onSurface(const StairFlight& flight, const nav_kernel::Vec3& point,
               double body_height, double tolerance) {
  const double u = lateral(flight, point);
  if (u < flight.lateralMin || u > flight.lateralMax) return false;
  const double s = along(flight, point);
  for (const auto& tread : flight.treads) {
    if (s >= tread.begin - 0.05 && s <= tread.end + 0.05 &&
        std::abs(point.z - tread.height - body_height) <= tolerance) return true;
  }
  // Map recognition already established adjacent surface connectivity. Match
  // the connector inside those measured bounds; a second fixed gap limit here
  // would silently discard flights accepted at the map's sampling resolution.
  for (std::size_t i = 1; i < flight.treads.size(); ++i) {
    const auto& previous = flight.treads[i - 1];
    const auto& next = flight.treads[i];
    if (s >= previous.end - 0.05 && s <= next.begin + 0.05 &&
        point.z >= previous.height + body_height - tolerance &&
        point.z <= next.height + body_height + tolerance) return true;
  }
  return false;
}

struct Crossing {
  double distance;
  bool upper;
  nav_kernel::Vec3 point;
};

bool matchesSection(const std::vector<nav_kernel::Vec3>& path,
                    const std::vector<double>& distances,
                    const Crossing& from, const Crossing& to,
                    const StairFlight& flight, double body_height,
                    double tolerance) {
  double last_s = along(flight, from.point);
  const double direction = from.upper ? -1.0 : 1.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double a = std::max(distances[i - 1], from.distance);
    const double b = std::min(distances[i], to.distance);
    if (b < a || distances[i] <= distances[i - 1]) continue;
    const int samples = std::max(1, static_cast<int>(std::ceil((b - a) / 0.025)));
    for (int j = 0; j <= samples; ++j) {
      const double d = a + (b - a) * static_cast<double>(j) / samples;
      const auto point = interpolate(path[i - 1], path[i],
          (d - distances[i - 1]) / (distances[i] - distances[i - 1]));
      const double s = along(flight, point);
      if (!onSurface(flight, point, body_height, tolerance) ||
          direction * (s - last_s) < -0.05) return false;
      last_s = s;
    }
  }
  return true;
}

StairProfile required(const StairSegment& segment) {
  return segment.ascending ? StairProfile::Ascent : StairProfile::Descent;
}

}  // namespace

std::vector<StairSegment> resolveStairSegments(
    const std::vector<nav_kernel::Vec3>& path,
    const std::vector<StairFlight>& flights, double body_height_m,
    double height_tolerance_m) {
  std::vector<StairSegment> result;
  if (path.size() < 2 || !std::isfinite(body_height_m)) return result;
  std::vector<double> distances(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i)
    distances[i] = distances[i - 1] + nav_kernel::distance3D(path[i - 1], path[i]);

  for (const auto& flight : flights) {
    if (!usable(flight)) continue;
    std::vector<Crossing> crossings;
    const auto inside_flight = [&](const nav_kernel::Vec3& point) {
      const double s = along(flight, point);
      return s > flight.treads.front().end + 1e-6 &&
             s < flight.treads.back().begin - 1e-6 &&
             onSurface(flight, point, body_height_m, height_tolerance_m);
    };
    const bool ascending = along(flight, path.back()) >= along(flight, path.front());
    if (inside_flight(path.front())) crossings.push_back({0.0, !ascending, path.front()});
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double from = along(flight, path[i - 1]);
      const double to = along(flight, path[i]);
      if (std::abs(to - from) < 1e-9) continue;
      for (bool upper : {false, true}) {
        const double boundary = upper ? flight.treads.back().begin
                                      : flight.treads.front().end;
        const double ratio = (boundary - from) / (to - from);
        if (ratio < 0.0 || ratio > 1.0) continue;
        const auto point = interpolate(path[i - 1], path[i], ratio);
        if (!onSurface(flight, point, body_height_m, height_tolerance_m)) continue;
        const double distance = distances[i - 1] +
            ratio * (distances[i] - distances[i - 1]);
        crossings.push_back({distance, upper, point});
      }
    }
    if (inside_flight(path.back()))
      crossings.push_back({distances.back(), ascending, path.back()});
    std::sort(crossings.begin(), crossings.end(), [](const auto& a, const auto& b) {
      return a.distance < b.distance;
    });
    crossings.erase(std::unique(crossings.begin(), crossings.end(),
        [](const auto& a, const auto& b) {
          return a.upper == b.upper && std::abs(a.distance - b.distance) < 1e-6;
        }), crossings.end());
    for (std::size_t i = 1; i < crossings.size(); ++i) {
      const auto& from = crossings[i - 1];
      const auto& to = crossings[i];
      if (from.upper == to.upper ||
          !matchesSection(path, distances, from, to, flight,
                          body_height_m, height_tolerance_m)) continue;
      result.push_back({flight, !from.upper, from.distance, to.distance,
                        body_height_m, from.point, to.point});
    }
  }
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.pathBegin < b.pathBegin;
  });
  return result;
}

StairTraversal::StairTraversal(StairTraversalConfig config) : config_(config) {}

void StairTraversal::setSegments(std::vector<StairSegment> segments) {
  pending_ = std::move(segments);
  cancelled_ = false;
  if (active_ && profile_requested_) {
    const auto same = std::find_if(pending_.begin(), pending_.end(), [&](const auto& segment) {
      return segment.flight.id == active_->flight.id && segment.ascending == active_->ascending;
    });
    if (same != pending_.end()) pending_.erase(same);
    else cancelled_ = true;
  } else {
    active_.reset();
    phase_ = StairPhase::Ordinary;
  }
}

void StairTraversal::cancel() {
  pending_.clear();
  cancelled_ = true;
  if (!profile_requested_) {
    active_.reset();
    phase_ = StairPhase::Ordinary;
  }
}

bool StairTraversal::pending() const { return active_.has_value() || !pending_.empty(); }

double StairTraversal::extentAlong(double yaw, const StairFlight& flight) const {
  const double angle = yaw - std::atan2(flight.axis.y, flight.axis.x);
  return 0.5 * (config_.bodyLength * std::abs(std::cos(angle)) +
                config_.bodyWidth * std::abs(std::sin(angle)));
}

bool StairTraversal::confirmed(StairProfile request, double now,
                               const StairProfileFeedback& feedback) const {
  return feedback.requirement == request && feedback.ready &&
         !feedback.activeProfile.empty() && std::isfinite(feedback.stampS) &&
         now >= feedback.stampS && now - feedback.stampS <= config_.feedbackMaxAge;
}

StairTraversalOutput StairTraversal::step(const nav_kernel::Pose& body,
                                         const nav_kernel::Twist& velocity,
                                         bool velocity_valid, double now,
                                         const StairProfileFeedback& feedback) {
  StairTraversalOutput out;
  if (!active_ && !pending_.empty()) {
    active_ = std::move(pending_.front());
    pending_.erase(pending_.begin());
    phase_ = StairPhase::Approach;
  }
  if (!active_) return out;
  const auto& segment = *active_;
  const auto& flight = segment.flight;
  const double sign = segment.ascending ? 1.0 : -1.0;
  const double entry_s = segment.ascending ? flight.treads.front().end : flight.treads.back().begin;
  const double exit_s = segment.ascending ? flight.treads.back().begin : flight.treads.front().end;
  const auto& entry_landing = segment.ascending ? flight.treads.front() : flight.treads.back();
  const auto& exit_landing = segment.ascending ? flight.treads.back() : flight.treads.front();
  const double s = along(flight, body.position);
  const double extent = extentAlong(body.yaw, flight);
  const double lateral_extent = extentAlong(body.yaw + 1.5707963267948966, flight);
  const double u = lateral(flight, body.position);
  const bool within_width = u - lateral_extent >= flight.lateralMin &&
                            u + lateral_extent <= flight.lateralMax;
  const bool entry_clear = sign * (entry_s - s) >= extent &&
      body.position.z >= entry_landing.height - config_.heightTolerance &&
      body.position.z <= entry_landing.height + segment.bodyHeight + config_.heightTolerance;
  const bool on_entry_landing = entry_clear && within_width &&
      s - extent >= entry_landing.begin && s + extent <= entry_landing.end;
  const bool exit_clear = sign * (s - exit_s) >= extent + config_.entryMargin &&
      s - extent >= exit_landing.begin && s + extent <= exit_landing.end && within_width &&
      body.position.z >= exit_landing.height - config_.heightTolerance &&
      body.position.z <= exit_landing.height + segment.bodyHeight + config_.heightTolerance;
  const bool overlaps_flight =
      s + extent > flight.treads.front().end && s - extent < flight.treads.back().begin &&
      u + lateral_extent > flight.lateralMin && u - lateral_extent < flight.lateralMax &&
      onSurface(flight, body.position, segment.bodyHeight, config_.heightTolerance);
  out.flightId = flight.id;
  out.onFlight = overlaps_flight && !entry_clear && !exit_clear;
  out.constrainRecovery = out.onFlight || profile_requested_;
  if (out.onFlight || profile_requested_) out.speedLimit = config_.maxSpeed;

  if (exit_clear && profile_requested_ && !cancelled_ && !pending_.empty() &&
      required(pending_.front()) == required(segment)) {
    const auto& next = pending_.front();
    const auto& landing = next.ascending ? next.flight.treads.front() : next.flight.treads.back();
    const double next_s = along(next.flight, body.position);
    const double next_u = lateral(next.flight, body.position);
    const double next_extent = extentAlong(body.yaw, next.flight);
    const double next_lateral_extent = extentAlong(body.yaw + 1.5707963267948966, next.flight);
    if (next_s - next_extent >= landing.begin && next_s + next_extent <= landing.end &&
        next_u - next_lateral_extent >= next.flight.lateralMin &&
        next_u + next_lateral_extent <= next.flight.lateralMax &&
        std::abs(landing.height - exit_landing.height) <= config_.heightTolerance) {
      active_ = std::move(pending_.front());
      pending_.erase(pending_.begin());
      phase_ = StairPhase::Approach;
      return step(body, velocity, velocity_valid, now, feedback);
    }
  }
  const bool quiet = velocity_valid && std::hypot(velocity.vx, velocity.vy) <= 0.03 &&
                      std::abs(velocity.wz) <= 0.08;
  if (phase_ != StairPhase::Restoring && profile_requested_ &&
      (exit_clear || (cancelled_ && on_entry_landing))) {
    if (!quiet) {
      out.phase = StairPhase::Restoring;
      out.request = required(segment);
      out.hold = true;
      out.reason = "stair_settling";
      return out;
    }
    // Enter once measured motion is quiet. Renewing this already-issued
    // request must survive temporary odometry loss during Grounded/Stand.
    phase_ = StairPhase::Restoring;
  }
  if (phase_ == StairPhase::Restoring) {
    out.phase = phase_;
    out.request = StairProfile::Restore;
    out.hold = true;
    out.reason = "stair_restoring";
    if (confirmed(out.request, now, feedback)) {
      active_.reset();
      profile_requested_ = false;
      cancelled_ = false;
      phase_ = StairPhase::Ordinary;
      return step(body, velocity, velocity_valid, now, feedback);
    }
    return out;
  }
  if (cancelled_) {
    out.phase = StairPhase::Waiting;
    out.hold = true;
    out.reason = "stair_hold_on_flight";
    return out;
  }

  const auto profile = required(segment);
  if (profile_requested_ && phase_ != StairPhase::Approach && confirmed(profile, now, feedback))
    phase_ = StairPhase::Traversing;
  if (phase_ == StairPhase::Traversing) {
    out.phase = phase_;
    out.request = profile;
    out.bodyYaw = std::atan2(sign * flight.axis.y, sign * flight.axis.x);
    out.hold = !confirmed(profile, now, feedback);
    out.reason = out.hold ? "stair_profile_wait" :
        (segment.ascending ? "stair_ascending" : "stair_descending");
    return out;
  }
  if (phase_ == StairPhase::Switching) {
    out.phase = feedback.unsupported ? StairPhase::Waiting : phase_;
    out.request = profile;
    out.hold = true;
    out.reason = feedback.unsupported ? "stair_profile_unsupported" : "stair_switching";
    return out;
  }

  // The stopping point leaves the complete footprint before the first riser.
  // Local planning receives this shortened reference before any switch request.
  const double stop_s = entry_s - sign * (0.5 * config_.bodyLength + config_.entryMargin);
  auto target = segment.entry;
  const double shift = stop_s - along(flight, target);
  target.x += flight.axis.x * shift;
  target.y += flight.axis.y * shift;
  target.z = entry_landing.height + segment.bodyHeight;
  out.approachTarget = target;
  out.bodyYaw = std::atan2(sign * flight.axis.y, sign * flight.axis.x);
  const double distance = nav_kernel::distance3D(body.position, target);
  const bool near_entrance = distance <= std::max(1.0, 2.0 * config_.bodyLength);
  if (near_entrance) out.speedLimit = config_.maxSpeed;
  const double target_u = lateral(flight, target);
  const bool landing_fits = stop_s - 0.5 * config_.bodyLength >= entry_landing.begin &&
                           stop_s + 0.5 * config_.bodyLength <= entry_landing.end &&
                           target_u - 0.5 * config_.bodyWidth >= flight.lateralMin &&
                           target_u + 0.5 * config_.bodyWidth <= flight.lateralMax;
  if (!landing_fits && near_entrance) {
    out.phase = StairPhase::Waiting;
    out.hold = true;
    out.reason = "stair_landing_space";
    return out;
  }
  if (!entry_clear && out.onFlight) {
    out.phase = StairPhase::Waiting;
    out.hold = true;
    out.reason = "stair_entry_required";
    return out;
  }
  const bool aligned = std::abs(nav_kernel::normalizeAngle(body.yaw - *out.bodyYaw)) <= 0.12;
  if (distance > 0.08 || !aligned || !within_width) {
    out.phase = StairPhase::Approach;
    out.reason = "stair_approach";
    return out;
  }
  if (!quiet) {
    out.phase = StairPhase::Approach;
    out.hold = true;
    out.reason = "stair_settling";
    return out;
  }
  profile_requested_ = true;
  phase_ = StairPhase::Switching;
  out.phase = feedback.unsupported ? StairPhase::Waiting : phase_;
  out.request = profile;
  out.hold = true;
  out.constrainRecovery = true;
  out.reason = feedback.unsupported ? "stair_profile_unsupported" : "stair_switching";
  return out;
}

StairGoalGuide StairTraversal::guideGoal(const nav_kernel::Vec3& goal,
                                        double tolerance, double yaw) const {
  StairGoalGuide guide;
  if (!active_) return guide;
  const auto& segment = *active_;
  const auto& flight = segment.flight;
  const double goal_s = along(flight, goal);
  const bool goal_on_flight = goal_s > flight.treads.front().end &&
      goal_s < flight.treads.back().begin &&
      onSurface(flight, goal, segment.bodyHeight, config_.heightTolerance);
  guide.arrivalAllowed = goal_on_flight && phase_ == StairPhase::Traversing;
  if (guide.arrivalAllowed || phase_ != StairPhase::Traversing) return guide;

  const double sign = segment.ascending ? 1.0 : -1.0;
  const auto& landing = segment.ascending ? flight.treads.back() : flight.treads.front();
  const double boundary = segment.ascending ? landing.begin : landing.end;
  // Five centimetres beyond the clearance boundary lets the existing 4 cm
  // local follower finish tolerance settle with the complete footprint clear.
  const double extent = std::max(0.5 * config_.bodyLength, extentAlong(yaw, flight));
  const double clear_s = boundary + sign * (extent + config_.entryMargin + 0.05);
  if (sign * (goal_s - boundary) < 0.0 || sign * (goal_s - clear_s) >= 0.0)
    return guide;

  auto target = goal;
  target.x += flight.axis.x * (clear_s - goal_s);
  target.y += flight.axis.y * (clear_s - goal_s);
  target.z = landing.height + segment.bodyHeight;
  const double u = lateral(flight, target);
  const double lateral_extent = std::max(0.5 * config_.bodyWidth,
      extentAlong(yaw + 1.5707963267948966, flight));
  const bool fits = clear_s - extent >= landing.begin &&
      clear_s + extent <= landing.end && u - lateral_extent >= flight.lateralMin &&
      u + lateral_extent <= flight.lateralMax;
  const bool within_goal = std::hypot(target.x - goal.x, target.y - goal.y) <= tolerance &&
      std::abs(target.z - goal.z) <= config_.heightTolerance;
  if (!fits || !within_goal) guide.hold = true;
  else guide.target = target;
  return guide;
}

bool StairTraversal::localPathAllowed(const std::vector<nav_kernel::Vec3>& path,
                                      double yaw) const {
  if (!active_ || phase_ != StairPhase::Traversing) return true;
  const auto& flight = active_->flight;
  if (path.size() < 2) {
    const double width = flight.lateralMax - flight.lateralMin;
    return width > config_.bodyWidth + 2.0 * config_.entryMargin;
  }
  const double axis_yaw = std::atan2(flight.axis.y, flight.axis.x);
  const double target = active_->ascending ? 0.0 : 3.14159265358979323846;
  const double from = nav_kernel::normalizeAngle(yaw - axis_yaw);
  const double to = from + nav_kernel::normalizeAngle(target - from);
  const double low = std::min(from, to), high = std::max(from, to);
  const auto lateral_extent = [&](double angle) {
    return 0.5 * (config_.bodyLength * std::abs(std::sin(angle)) +
                  config_.bodyWidth * std::abs(std::cos(angle)));
  };
  double extent = std::max(lateral_extent(from), lateral_extent(to));
  // The controller holds the flight heading during side steps. Cover its
  // entire shortest turn from measured yaw, including interior maxima.
  const double peak = std::atan2(config_.bodyLength, config_.bodyWidth);
  const double pi = 3.14159265358979323846;
  for (double sign : {-1.0, 1.0}) {
    const double angle = sign * peak + std::ceil((low - sign * peak) / pi) * pi;
    if (angle <= high) extent = std::max(extent, lateral_extent(angle));
  }
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto& point = path[i];
    const double s = along(flight, point);
    if (s < flight.treads.front().end || s > flight.treads.back().begin) continue;
    const double u = lateral(flight, point);
    if (u - extent < flight.lateralMin || u + extent > flight.lateralMax) return false;
  }
  return true;
}

const char* stairPhaseName(StairPhase phase) {
  switch (phase) {
    case StairPhase::Approach: return "approach";
    case StairPhase::Switching: return "switching";
    case StairPhase::Traversing: return "traversing";
    case StairPhase::Restoring: return "restoring";
    case StairPhase::Waiting: return "waiting";
    default: return "ordinary";
  }
}

const char* stairProfileName(StairProfile profile) {
  switch (profile) {
    case StairProfile::Ascent: return "stairs_up";
    case StairProfile::Descent: return "stairs_down";
    case StairProfile::Restore: return "restore";
    default: return "";
  }
}

}  // namespace lingtu::nav::navigation
