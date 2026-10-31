// Mapd adapter for SCAN-Planner's GridMap query seam.
// Upstream algorithm commit: 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "planning/local/grid_axis.hpp"

namespace nav_kernel::local::scan {
namespace {
constexpr double bodySweepSpacingM = .01;

double pointSegmentSquared(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
  const double dx = b.x-a.x, dy = b.y-a.y;
  const double length = dx*dx+dy*dy;
  const double t = length > 1e-12
      ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length, 0.0, 1.0) : 0.0;
  return std::pow(p.x-a.x-t*dx, 2) + std::pow(p.y-a.y-t*dy, 2);
}

double segmentSquared(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d) {
  const double ax = b.x-a.x, ay = b.y-a.y;
  const double bx = d.x-c.x, by = d.y-c.y;
  const double cross = ax*by-ay*bx;
  if (std::abs(cross) > 1e-12) {
    const double t = ((c.x-a.x)*by-(c.y-a.y)*bx)/cross;
    const double u = ((c.x-a.x)*ay-(c.y-a.y)*ax)/cross;
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return 0.0;
  }
  return std::min({pointSegmentSquared(a,c,d), pointSegmentSquared(b,c,d),
                   pointSegmentSquared(c,a,b), pointSegmentSquared(d,a,b)});
}
}  // namespace

Grid::Grid(const LocalPlannerParams &params, const LocalPlanRequest &input)
    : checkObstacle_(params.checkObstacle || input.body != nullptr || input.observedBody != nullptr),
      configuredResolution_(params.scan.voxelResolution),
      cylinderOffset_(std::max(0.0, params.scan.cylinderOffset)),
      robotYaw_(input.robot.pose.yaw),
      robotPosition_(input.robot.pose.position),
      collision_(input.environment.collision), body_(input.observedBody ? input.observedBody : input.body) {
  if (input.supportSurface && input.supportHeightM && input.supportHeightToleranceM) {
    supportSurface_ = input.supportSurface;
    supportHeightM_ = *input.supportHeightM;
    supportToleranceM_ = *input.supportHeightToleranceM;
    const auto *reference = input.referenceRoute();
    if (reference && reference->points && reference->count > 0)
      supportReference_.assign(reference->points, reference->points + reference->count);
  }
  if (input.observedBody && !input.body) {
    reason_ = "body_model_missing";
    return;
  }
  if (body_ && (body_->parts.empty() || !std::isfinite(robotYaw_))) {
    reason_ = "body_model_invalid";
    return;
  }
  // Assisted translation preserves body heading instead of following the
  // spline tangent. Collision queries must use the same footprint.
  if (input.intent() != nullptr) bodyHeadings_.push_back(robotYaw_);
  else if (input.bodyHeading) {
    if (!std::isfinite(*input.bodyHeading) || !std::isfinite(robotYaw_)) {
      reason_ = "body_heading_invalid";
      return;
    }
    // Only the measured start needs alignment; future segments hold the target yaw.
    const double angle = normalizeAngle(*input.bodyHeading - robotYaw_);
    const double resolution = checkObstacle_ ? collision_.resolution : configuredResolution_;
    if (!std::isfinite(resolution) || resolution <= 0.0) return;
    const int steps = std::max(1, static_cast<int>(std::ceil(
        std::abs(angle) * cylinderOffset_ / (0.5 * resolution))));
    if (std::abs(angle) < 1e-9) bodyHeadings_.push_back(robotYaw_);
    else {
      for (int i = 0; i <= steps; ++i)
        bodyHeadings_.push_back(robotYaw_ + angle * i / steps);
      headingSweepPadding_ = std::abs(angle) * cylinderOffset_ / (2.0 * steps);
    }
  }
  if (!checkObstacle_) {
    reason_ = std::isfinite(configuredResolution_) && configuredResolution_ > 0.0
                  ? "ready"
                  : "grid_resolution_invalid";
    return;
  }
  if (!collision_.valid()) {
    return;
  }
  if (body_) {
    const auto bytes = (collision_.cellCount() + 7U) / 8U;
    if (!collision_.measuredOccupiedStorage ||
        collision_.measuredOccupiedStorage->size() != bytes) {
      reason_ = "body_raw_evidence_missing";
      return;
    }
  }
  // `complete` and the epoch/sequence counters describe transport history.
  // The bitmap itself has already passed the DDS decoder's exact-size and
  // geometry checks; they are not additional collision-query prerequisites.
  if (std::abs(collision_.resolution - params.scan.voxelResolution) >
      std::max(1e-9, 1e-6 * params.scan.voxelResolution)) {
    reason_ = "collision_map_resolution_mismatch";
    return;
  }
  if (input.environment.predictions.fresh(input.clock.timestampS)) {
    const auto &view = input.environment.predictions;
    if (!std::isfinite(view.observedAtS) || !std::isfinite(view.horizonS) ||
        view.horizonS <= 0.0) {
      // Dynamic predictions are an optional overlay.  Keep the measured 3D
      // collision bitmap usable when that overlay has a malformed timestamp.
      reason_ = "ready";
      return;
    }
    predictionAgeS_ = std::max(0.0, input.clock.timestampS - view.observedAtS);
    predictionHorizonS_ = view.horizonS;
    predictions_.reserve(view.count);
    for (std::size_t i = 0; i < view.count; ++i) {
      auto prediction = view.obstacles[i];
      if (!std::isfinite(prediction.start.x) || !std::isfinite(prediction.start.y) ||
          !std::isfinite(prediction.end.x) || !std::isfinite(prediction.end.y) ||
          !std::isfinite(prediction.radius) || prediction.radius < 0.0 ||
          !std::isfinite(prediction.minZ) || !std::isfinite(prediction.maxZ) ||
          prediction.minZ > prediction.maxZ) {
        continue;
      }
      if (!body_) {
        prediction.radius += params.scan.cylinderRadius;
        prediction.minZ -= std::max(0.0, params.scan.bodyClearanceAbove);
        prediction.maxZ += std::max(0.0, params.scan.bodyClearanceBelow);
      }
      predictions_.push_back(prediction);
    }
  }
  reason_ = "ready";
}

bool Grid::predictionIntersects(const Vec3 &start, double startYaw,
                                const Vec3 &end, double endYaw) const noexcept {
  if (predictions_.empty()) return false;
  if (body_) return bodyPredictionIntersects(start, startYaw, end, endYaw,
                                            0.0, 0.0, true, true);
  const auto intersects = [&](const Vec3 &fromPoint, double fromYaw,
                              const Vec3 &toPoint, double toYaw, double padding) {
    for (const auto &prediction : predictions_) {
      const double radius = prediction.radius + padding;
      if (std::max(fromPoint.z, toPoint.z) < prediction.minZ ||
          std::min(fromPoint.z, toPoint.z) > prediction.maxZ) continue;
      const double ageFraction = std::clamp(predictionAgeS_ / predictionHorizonS_, 0.0, 1.0);
      // The prediction begins at its observation time. Keep only the remaining
      // sweep; retaining its past path invents obstacles the object has left.
      const Vec3 current{prediction.start.x + ageFraction * (prediction.end.x - prediction.start.x),
                         prediction.start.y + ageFraction * (prediction.end.y - prediction.start.y),
                         0.0};
      for (const double sign : {-1.0, 1.0}) {
        const Vec3 from{fromPoint.x + sign*cylinderOffset_*std::cos(fromYaw),
                         fromPoint.y + sign*cylinderOffset_*std::sin(fromYaw), fromPoint.z};
        const Vec3 to{toPoint.x + sign*cylinderOffset_*std::cos(toYaw),
                       toPoint.y + sign*cylinderOffset_*std::sin(toYaw), toPoint.z};
        if (segmentSquared(from, to, current, prediction.end) <=
            radius * radius) return true;
      }
    }
    return false;
  };
  if (bodyHeadings_.empty()) return intersects(start, startYaw, end, endYaw, 0.0);
  if (atRobotPosition(start))
    for (double yaw : bodyHeadings_)
      if (intersects(start, yaw, start, yaw, headingSweepPadding_)) return true;
  return intersects(start, bodyHeadings_.back(), end, bodyHeadings_.back(), 0.0);
}

bool Grid::atRobotPosition(const Vec3 &point) const noexcept {
  return std::abs(point.x - robotPosition_.x) <= 1e-6 &&
         std::abs(point.y - robotPosition_.y) <= 1e-6 &&
         std::abs(point.z - robotPosition_.z) <= 1e-6;
}

bool Grid::valid() const noexcept {
  return reason_ == "ready";
}

const std::string &Grid::reason() const noexcept {
  return reason_;
}

double Grid::resolution() const noexcept {
  return checkObstacle_ ? collision_.resolution : configuredResolution_;
}

int Grid::occupiedCellCount() const noexcept {
  return static_cast<int>(std::min<std::size_t>(
      collision_.occupiedCount(), static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

int Grid::collisionPointCount() const noexcept {
  return occupiedCellCount();
}

bool Grid::obstacleFree(const Vec3 &center, double yaw) const noexcept {
  return inflatedOccupancy(center, yaw) == 0;
}

int Grid::inflatedOccupancy(const Vec3 &center, double yaw) const noexcept {
  if (!valid()) return -1;
  if (!checkObstacle_) return 0;
  if (timedPredictionIntersects(center, yaw, center, yaw, 0.0, 0.0)) return 1;
  if (body_) return measuredSegmentOccupancy(center, yaw, center, yaw);
  if (!bodyHeadings_.empty())
    return measuredSegmentOccupancy(center, yaw, center, yaw);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const int front = occupiedState({center.x + cylinderOffset_ * c,
                                   center.y + cylinderOffset_ * s, center.z});
  if (front != 0) return front;
  return occupiedState({center.x - cylinderOffset_ * c,
                        center.y - cylinderOffset_ * s, center.z});
}

int Grid::segmentInflatedOccupancy(const Vec3 &start, double startYaw,
                                    const Vec3 &end, double endYaw) const noexcept {
  if (!valid()) return -1;
  if (!checkObstacle_) return 0;
  if (timedPredictionIntersects(start, startYaw, end, endYaw, 0.0, 0.0)) return 1;
  return measuredSegmentOccupancy(start, startYaw, end, endYaw);
}

bool Grid::timedPredictionIntersects(const Vec3 &start, double startYaw,
                                     const Vec3 &end, double endYaw,
                                     double startAheadS, double endAheadS,
                                     bool useBodyHeading) const noexcept {
  if (predictions_.empty()) return false;
  if (body_) return bodyPredictionIntersects(start, startYaw, end, endYaw,
                                            startAheadS, endAheadS, useBodyHeading);
  if (useBodyHeading && !bodyHeadings_.empty()) {
    if (startAheadS <= 1e-9 && atRobotPosition(start)) {
      for (double yaw : bodyHeadings_) {
        for (const auto &prediction : predictions_) {
          if (start.z < prediction.minZ || start.z > prediction.maxZ) continue;
          const double fraction = std::clamp(predictionAgeS_ / predictionHorizonS_, 0.0, 1.0);
          const Vec3 center{prediction.start.x + fraction*(prediction.end.x-prediction.start.x),
                              prediction.start.y + fraction*(prediction.end.y-prediction.start.y), 0};
          const double radius = prediction.radius + headingSweepPadding_;
          for (double sign : {-1.0, 1.0}) {
            const Vec3 point{start.x + sign*cylinderOffset_*std::cos(yaw),
                              start.y + sign*cylinderOffset_*std::sin(yaw), 0};
            if (pointSegmentSquared(center, point, point) <= radius*radius) return true;
          }
        }
      }
    }
    return timedPredictionIntersects(start, bodyHeadings_.back(), end, bodyHeadings_.back(),
                                      startAheadS, endAheadS, false);
  }
  {
    const double fromYaw = startYaw;
    const double toYaw = endYaw;
    for (const auto &prediction : predictions_) {
      const double radius = prediction.radius;
      if (std::max(start.z, end.z) < prediction.minZ ||
          std::min(start.z, end.z) > prediction.maxZ) continue;
      const auto centerAt = [&](double ahead) {
        const double f = std::clamp((predictionAgeS_ + ahead) / predictionHorizonS_, 0.0, 1.0);
        return Vec3{prediction.start.x + f*(prediction.end.x-prediction.start.x),
                    prediction.start.y + f*(prediction.end.y-prediction.start.y), 0.0};
      };
      // Split at the prediction horizon: afterward the last predicted position
      // is retained. Each piece has linear relative motion, not two independent sweeps.
      const double split = std::clamp(predictionHorizonS_ - predictionAgeS_, startAheadS, endAheadS);
      const std::array<double, 3> times{startAheadS, split, endAheadS};
      for (const double sign : {-1.0, 1.0}) {
        const Vec3 from{start.x + sign*cylinderOffset_*std::cos(fromYaw),
                       start.y + sign*cylinderOffset_*std::sin(fromYaw), 0.0};
        const Vec3 to{end.x + sign*cylinderOffset_*std::cos(toYaw),
                     end.y + sign*cylinderOffset_*std::sin(toYaw), 0.0};
        // Geometry queries have no arrival time: compare the entire segment
        // against the object's current position, not its future swept volume.
        if (endAheadS == startAheadS) {
          if (pointSegmentSquared(centerAt(startAheadS), from, to) <=
              radius * radius) return true;
          continue;
        }
        const auto relativeAt = [&](double ahead) {
          const double f = endAheadS > startAheadS ? (ahead-startAheadS)/(endAheadS-startAheadS) : 0.0;
          const auto center = centerAt(ahead);
          return Vec3{from.x + f*(to.x-from.x)-center.x,
                      from.y + f*(to.y-from.y)-center.y, 0.0};
        };
        for (int i = 0; i < 2; ++i) {
          if (pointSegmentSquared({}, relativeAt(times[i]), relativeAt(times[i+1])) <=
              radius * radius) return true;
        }
      }
    }
  }
  return false;
}

bool Grid::bodyPredictionIntersects(const Vec3 &start, double startYaw,
                                    const Vec3 &end, double endYaw,
                                    double startAheadS, double endAheadS,
                                    bool useBodyHeading, bool futureSweep) const noexcept {
  namespace c = lingtu::nav::collision;
  if (useBodyHeading && !bodyHeadings_.empty()) {
    const double targetYaw = bodyHeadings_.back();
    if (startAheadS <= 1e-9 && atRobotPosition(start) &&
        bodyPredictionIntersects(start, robotYaw_, start, targetYaw,
                                  0.0, 0.0, false, futureSweep)) return true;
    return bodyPredictionIntersects(start, targetYaw, end, targetYaw,
                                     startAheadS, endAheadS, false, futureSweep);
  }
  const double yawDelta = c::shortestYawDelta(startYaw, endYaw);
  const auto poseAt = [&](double fraction) {
    return c::Pose{{start.x + fraction*(end.x-start.x),
                     start.y + fraction*(end.y-start.y),
                     start.z + fraction*(end.z-start.z)}, startYaw + fraction*yawDelta};
  };
  for (const auto &prediction : predictions_) {
    const auto centerAt = [&](double ahead) {
      const double fraction = std::clamp(
          (predictionAgeS_ + ahead) / predictionHorizonS_, 0.0, 1.0);
      return c::Vec3{prediction.start.x + fraction*(prediction.end.x-prediction.start.x),
                      prediction.start.y + fraction*(prediction.end.y-prediction.start.y), 0.0};
    };
    const auto intersects = [&](c::Pose from, c::Pose to, const c::Vec3 &first,
                                const c::Vec3 &last, c::Aabb obstacle) {
      from.position.x -= first.x;
      from.position.y -= first.y;
      to.position.x -= last.x;
      to.position.y -= last.y;
      const double distance = c::sweepBound(*body_, from, to);
      const double spacing = std::min(.01, .25 * resolution());
      const int samples = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
      const double padding = distance / (2.0 * samples);
      const double turn = c::shortestYawDelta(from.yaw, to.yaw);
      for (int sample = 0; sample < samples; ++sample) {
        const double fraction = (sample + .5) / samples;
        const c::Pose pose{{from.position.x + fraction*(to.position.x-from.position.x),
                            from.position.y + fraction*(to.position.y-from.position.y),
                            from.position.z + fraction*(to.position.z-from.position.z)},
                           from.yaw + fraction*turn};
        for (const auto &part : body_->parts)
          if (c::intersects(part, pose, obstacle, body_->margin_m + padding,
                              body_->horizontal_margin_m)) return true;
      }
      return false;
    };
    // Prediction cylinders are conservatively enclosed by a box. Body shape
    // and margin are applied once, directly against this raw obstacle volume.
    c::Aabb obstacle{{0.0, 0.0, .5*(prediction.minZ+prediction.maxZ)},
                      {prediction.radius, prediction.radius, .5*(prediction.maxZ-prediction.minZ)}};
    if (futureSweep) {
      const auto current = centerAt(0.0);
      const c::Vec3 center{.5*(current.x+prediction.end.x),
                            .5*(current.y+prediction.end.y), 0.0};
      obstacle.half_size.x += .5*std::abs(prediction.end.x-current.x);
      obstacle.half_size.y += .5*std::abs(prediction.end.y-current.y);
      if (intersects(poseAt(0.0), poseAt(1.0), center, center, obstacle)) return true;
    } else if (endAheadS == startAheadS) {
      const auto center = centerAt(startAheadS);
      if (intersects(poseAt(0.0), poseAt(1.0), center, center, obstacle)) return true;
    } else {
      // At the horizon the prediction stops. Each side has linear relative
      // translation, so the same point-travel bound also covers between samples.
      const double split = std::clamp(predictionHorizonS_ - predictionAgeS_, startAheadS, endAheadS);
      const std::array<double, 3> times{startAheadS, split, endAheadS};
      for (int piece = 0; piece < 2; ++piece) {
        if (times[piece+1] <= times[piece]) continue;
        const double first = (times[piece]-startAheadS)/(endAheadS-startAheadS);
        const double last = (times[piece+1]-startAheadS)/(endAheadS-startAheadS);
        if (intersects(poseAt(first), poseAt(last), centerAt(times[piece]),
                       centerAt(times[piece+1]), obstacle)) return true;
      }
    }
  }
  return false;
}

int Grid::trajectoryOccupancy(const Vec3 &start, const Vec3 &velocity,
                               const Vec3 &end, const Vec3 &endVelocity,
                               double startAheadS, double endAheadS) const noexcept {
  if (!valid() || !std::isfinite(startAheadS) || !std::isfinite(endAheadS) ||
      startAheadS < 0.0 || endAheadS < startAheadS) return -1;
  if (!supportedSegment(start, end)) return 1;
  if (!checkObstacle_) return 0;
  const auto heading = [&](const Vec3 &v, double previousYaw) {
    // Quiet motion preserves the known heading rather than replaying the request yaw.
    return std::hypot(v.x, v.y) <= 0.03 ? previousYaw : std::atan2(v.y, v.x);
  };
  double startYaw = startAheadS <= 1e-9 ? robotYaw_ : heading(velocity, robotYaw_);
  double endYaw = heading(endVelocity, startYaw);
  if (!bodyHeadings_.empty()) {
    if (startAheadS <= 1e-9 && atRobotPosition(start)) {
      const int alignment = measuredSegmentOccupancy(start, robotYaw_, start, bodyHeadings_.back());
      if (alignment != 0) return alignment;
    }
    startYaw = endYaw = bodyHeadings_.back();
  }
  if (timedPredictionIntersects(start, startYaw, end, endYaw, startAheadS, endAheadS)) return 1;
  return measuredSegmentOccupancy(start, startYaw, end, endYaw, false);
}

bool Grid::supportedSegment(const Vec3 &start, const Vec3 &end) const noexcept {
  if (!supportSurface_ || supportReference_.empty()) return true;
  const int steps = std::max(1, static_cast<int>(std::ceil(distance3D(start, end) / (0.5 * resolution()))));
  const double cosine = std::cos(collision_.gridFromPlanningYaw);
  const double sine = std::sin(collision_.gridFromPlanningYaw);
  for (int sample = 0; sample <= steps; ++sample) {
    const double fraction = static_cast<double>(sample) / steps;
    const Vec3 point{start.x + fraction * (end.x-start.x),
                     start.y + fraction * (end.y-start.y),
                     start.z + fraction * (end.z-start.z)};
    double closest = std::numeric_limits<double>::infinity();
    double low = supportReference_.front().z, high = low;
    for (std::size_t i = 1; i < supportReference_.size(); ++i) {
      const auto &from = supportReference_[i-1], &to = supportReference_[i];
      const Vec3 delta{to.x-from.x,to.y-from.y,to.z-from.z};
      const double length = delta.x*delta.x+delta.y*delta.y+delta.z*delta.z;
      const double t = length > 0.0 ? std::clamp(((point.x-from.x)*delta.x+
          (point.y-from.y)*delta.y+(point.z-from.z)*delta.z)/length, 0.0, 1.0) : 0.0;
      const double dx = point.x-from.x-t*delta.x, dy = point.y-from.y-t*delta.y,
                   dz = point.z-from.z-t*delta.z;
      const double distance = dx*dx+dy*dy+dz*dz;
      if (distance < closest) {
        closest = distance;
        low = std::min(from.z,to.z);
        high = std::max(from.z,to.z);
      }
    }
    // The reference selects the terrain layer. Base heave does not move that layer.
    const lingtu::nav::collision::Vec3 ground{
        collision_.gridFromPlanningTranslation.x + cosine*point.x - sine*point.y,
        collision_.gridFromPlanningTranslation.y + sine*point.x + cosine*point.y,
        collision_.gridFromPlanningTranslation.z + .5*(low+high) - supportHeightM_};
    if (!supportSurface_(ground, .5*(high-low) + supportToleranceM_)) return false;
  }
  return true;
}

int Grid::measuredSegmentOccupancy(const Vec3 &start, double startYaw,
                                   const Vec3 &end, double endYaw,
                                   bool useBodyHeading) const noexcept {
  if (body_) {
    if (useBodyHeading && !bodyHeadings_.empty()) {
      const double targetYaw = bodyHeadings_.back();
      if (atRobotPosition(start) && std::abs(normalizeAngle(targetYaw - robotYaw_)) > 1e-9) {
        const int turn = bodySweepOccupancy(start, robotYaw_, start, targetYaw);
        if (turn != 0) return turn;
      }
      return bodySweepOccupancy(start, targetYaw, end, targetYaw);
    }
    return bodySweepOccupancy(start, startYaw, end, endYaw);
  }
  const double c = std::cos(collision_.gridFromPlanningYaw);
  const double s = std::sin(collision_.gridFromPlanningYaw);
  const auto toGrid = [&](const Vec3 &point) {
    return Vec3{
        collision_.gridFromPlanningTranslation.x + c * point.x - s * point.y,
        collision_.gridFromPlanningTranslation.y + s * point.x + c * point.y,
        collision_.gridFromPlanningTranslation.z + point.z};
  };
  const Vec3 startGrid = toGrid(start);
  const Vec3 endGrid = toGrid(end);
  const auto offset = [&](double yaw) {
    return Vec3{cylinderOffset_ * std::cos(yaw + collision_.gridFromPlanningYaw),
                cylinderOffset_ * std::sin(yaw + collision_.gridFromPlanningYaw), 0.0};
  };
  const auto check = [&](const Vec3& from, const Vec3& fromOffset,
                         const Vec3& to, const Vec3& toOffset) {
    for (double sign : {1.0, -1.0}) {
      const int state = occupiedGridSegment(
          {from.x + sign * fromOffset.x, from.y + sign * fromOffset.y, from.z},
          {to.x + sign * toOffset.x, to.y + sign * toOffset.y, to.z});
      if (state != 0) return state;
    }
    return 0;
  };
  if (!useBodyHeading || bodyHeadings_.empty())
    return check(startGrid, offset(startYaw), endGrid, offset(endYaw));
  if (atRobotPosition(start)) {
    auto previous = offset(bodyHeadings_.front());
    for (std::size_t h = 0; h < bodyHeadings_.size(); ++h) {
      const auto current = offset(bodyHeadings_[h]);
      const int state = check(startGrid, previous, startGrid, current);
      if (state != 0) return state;
      previous = current;
    }
  }
  const auto target = offset(bodyHeadings_.back());
  return check(startGrid, target, endGrid, target);
}

int Grid::bodyPoseOccupancy(const Vec3 &center, double yaw,
                             double extraMargin,
                             std::vector<std::size_t> *occupiedCells) const noexcept {
  namespace c = lingtu::nav::collision;
  if (!valid() || !body_ || !std::isfinite(center.x) || !std::isfinite(center.y) ||
      !std::isfinite(center.z) || !std::isfinite(yaw)) return -1;
  const double cosine = std::cos(collision_.gridFromPlanningYaw);
  const double sine = std::sin(collision_.gridFromPlanningYaw);
  const c::Pose pose{{collision_.gridFromPlanningTranslation.x + cosine * center.x - sine * center.y,
                      collision_.gridFromPlanningTranslation.y + sine * center.x + cosine * center.y,
                      collision_.gridFromPlanningTranslation.z + center.z},
                     yaw + collision_.gridFromPlanningYaw};
  const double margin = body_->margin_m + extraMargin;
  const double resolution = collision_.resolution;
  const auto bit = [](const std::vector<std::uint8_t> &bits, std::size_t index) {
    return (bits[index / 8U] & static_cast<std::uint8_t>(1U << (index % 8U))) != 0U;
  };
  const double groundHeight = c::nominalGroundHeight(*body_, pose);
  for (const auto &part : body_->parts) {
    const auto geometry = c::preparePart(part, pose, margin, body_->horizontal_margin_m);
    std::optional<c::TerrainSurface> surface;
    bool surfaceQueried = false;
    const auto prepareSurface = [&] {
      if (!surfaceQueried) {
        surface = c::partTerrainSurface(collision_.terrain.get(), part, pose,
                                        groundHeight, resolution, margin, body_->horizontal_margin_m);
        surfaceQueried = true;
      }
    };
    const auto &bounds = geometry.bounds;
    const c::Vec3 minimum{bounds.center.x - bounds.half_size.x,
                           bounds.center.y - bounds.half_size.y,
                           bounds.center.z - bounds.half_size.z};
    const c::Vec3 maximum{bounds.center.x + bounds.half_size.x,
                           bounds.center.y + bounds.half_size.y,
                           bounds.center.z + bounds.half_size.z};
    // The complete expanded body must remain inside observed grid coverage.
    if (minimum.x <= collision_.aabbMin.x || minimum.y <= collision_.aabbMin.y ||
        minimum.z <= collision_.aabbMin.z || maximum.x >= collision_.aabbMax.x ||
        maximum.y >= collision_.aabbMax.y || maximum.z >= collision_.aabbMax.z) return -1;
    constexpr double contact = 1e-12;
    const auto index = [resolution](double value, double origin) {
      return static_cast<int>(std::floor((value - origin) / resolution));
    };
    const int x0 = index(minimum.x - contact, collision_.aabbMin.x);
    const int y0 = index(minimum.y - contact, collision_.aabbMin.y);
    const int z0 = index(minimum.z - contact, collision_.aabbMin.z);
    const int x1 = index(maximum.x + contact, collision_.aabbMin.x);
    const int y1 = index(maximum.y + contact, collision_.aabbMin.y);
    const int z1 = index(maximum.z + contact, collision_.aabbMin.z);
    for (int z = z0; z <= z1; ++z) for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        if (x < 0 || x >= collision_.sizeX || y < 0 || y >= collision_.sizeY ||
            z < 0 || z >= collision_.sizeZ) return -1;
        const std::size_t linear = (static_cast<std::size_t>(z) * collision_.sizeY + y) *
                                      collision_.sizeX + x;
        const bool occupied = bit(*collision_.measuredOccupiedStorage, linear);
        // Live local queries test measured obstacles; unknown evidence stays unchanged.
        if (!occupied) continue;
        const c::Aabb cell{{collision_.aabbMin.x + (x + .5) * resolution,
                            collision_.aabbMin.y + (y + .5) * resolution,
                            collision_.aabbMin.z + (z + .5) * resolution},
                           {resolution * .5, resolution * .5, resolution * .5}};
        if (!c::intersects(geometry, cell)) continue;
        if (c::allowsGroundContact(geometry, cell)) continue;
        if (collision_.terrain && collision_.terrain->hitBoundsClear(geometry, cell)) continue;
        prepareSurface();
        if (surface && c::terrainCellClear(*collision_.terrain, part, pose, margin,
                                           cell, *surface, body_->horizontal_margin_m)) continue;
        if (!occupiedCells) return 1;
        occupiedCells->push_back(linear);
      }
  }
  return occupiedCells && !occupiedCells->empty() ? 1 : 0;
}

int Grid::bodySweepOccupancy(const Vec3 &start, double startYaw,
                              const Vec3 &end, double endYaw) const noexcept {
  namespace c = lingtu::nav::collision;
  if (!valid() || !body_ || !std::isfinite(startYaw) || !std::isfinite(endYaw) ||
      !std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.z) ||
      !std::isfinite(end.x) || !std::isfinite(end.y) || !std::isfinite(end.z)) return -1;
  const c::Pose from{{start.x, start.y, start.z}, startYaw};
  const c::Pose to{{end.x, end.y, end.z}, endYaw};
  const double distance = c::sweepBound(*body_, from, to);
  const int startState = bodyPoseOccupancy(start, startYaw);
  if (startState != 0 || distance <= 1e-12) return startState;
  const int endState = bodyPoseOccupancy(end, endYaw);
  if (endState != 0) return endState;
  const double spacing = std::min(bodySweepSpacingM, .25 * collision_.resolution);
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  const double padding = distance / (2.0 * samples);
  const double yawDelta = c::shortestYawDelta(startYaw, endYaw);
  // A midpoint plus half the segment travel covers the sweep without extending
  // the sampling allowance beyond a clear departure or arrival.
  for (int sample = 0; sample < samples; ++sample) {
    const double t = (sample + .5) / samples;
    const int state = bodyPoseOccupancy({start.x + t * (end.x - start.x),
                                        start.y + t * (end.y - start.y),
                                        start.z + t * (end.z - start.z)},
                                       startYaw + t * yawDelta, padding);
    if (state != 0) return state;
  }
  return 0;
}

int Grid::brakingOccupancy(const Pose &body, const Twist &command,
                           double reactionS, double linearDeceleration,
                           double yawDeceleration) const noexcept {
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.wz) || !std::isfinite(body.yaw) ||
      !std::isfinite(reactionS) || reactionS < 0.0 ||
      !std::isfinite(linearDeceleration) || linearDeceleration <= 0.0 ||
      !std::isfinite(yawDeceleration) || yawDeceleration <= 0.0 || !valid()) return -1;
  const double speed = std::hypot(command.vx, command.vy);
  if (body_ && speed == 0.0 && command.wz == 0.0 && predictions_.empty())
    return bodyPoseOccupancy(body.position, body.yaw);
  const double stopTime = std::max(speed / linearDeceleration,
                                    std::abs(command.wz) / yawDeceleration);
  // A linearly decreasing twist follows the same arc as a constant twist
  // over this effective duration, including command latency before braking.
  const double duration = reactionS + stopTime;
  const double effectiveDuration = reactionS + 0.5 * stopTime;
  const double sweptLength = (speed + cylinderOffset_ * std::abs(command.wz)) * effectiveDuration;
  std::optional<lingtu::nav::collision::TerrainSurface> brakingSurface;
  Vec3 bodyInGrid{};
  if (body_ && collision_.terrain) {
    namespace c = lingtu::nav::collision;
    const double cosine=std::cos(collision_.gridFromPlanningYaw);
    const double sine=std::sin(collision_.gridFromPlanningYaw);
    bodyInGrid={collision_.gridFromPlanningTranslation.x+cosine*body.position.x-sine*body.position.y,
                collision_.gridFromPlanningTranslation.y+sine*body.position.x+cosine*body.position.y,
                collision_.gridFromPlanningTranslation.z+body.position.z};
    const c::Pose current{{bodyInGrid.x,bodyInGrid.y,bodyInGrid.z},
                           body.yaw+collision_.gridFromPlanningYaw};
    brakingSurface=collision_.terrain->surface(
        {bodyInGrid.x,bodyInGrid.y,c::nominalGroundHeight(*body_,current)},
        resolution()+.03,2*resolution());
  }
  double bodyPointSpeed = 0.0;
  if (body_) {
    namespace c = lingtu::nav::collision;
    const double radius = c::sweepBound(*body_, {{},0.0}, {{},1.0});
    const double slope = brakingSurface ?
        std::hypot(brakingSurface->normal.x,brakingSurface->normal.y)/brakingSurface->normal.z : 0;
    bodyPointSpeed = speed*std::hypot(1.,slope) + radius * std::abs(command.wz);
  }
  // Without timed obstacles, effective time parameterizes
  // the same complete braking arc. Sample its point-travel bound, rather than
  // repeatedly querying nearly stationary poses at the end of braking.
  const bool spatialSampling = body_ && predictions_.empty();
  const double samplingDistance = body_
      ? bodyPointSpeed * (spatialSampling ? effectiveDuration : duration) : sweptLength;
  const double samplingSpacing = body_
      ? std::min(bodySweepSpacingM, .25 * resolution()) : .5 * resolution();
  const int samples = std::max(1, static_cast<int>(std::ceil(
      std::max({samplingDistance / samplingSpacing, std::abs(command.wz) * effectiveDuration / 0.05,
                spatialSampling ? 0.0 : duration / 0.01}))));
  const auto poseAt = [&](double time) {
    const double angle = command.wz * time;
    double x = command.vx * time;
    double y = command.vy * time;
    if (std::abs(command.wz) > 1e-6) {
      x = (command.vx * std::sin(angle) + command.vy * (std::cos(angle) - 1.0)) / command.wz;
      y = (command.vx * (1.0 - std::cos(angle)) + command.vy * std::sin(angle)) / command.wz;
    }
    Pose predicted{{body.position.x + std::cos(body.yaw) * x - std::sin(body.yaw) * y,
                     body.position.y + std::sin(body.yaw) * x + std::cos(body.yaw) * y,
                     body.position.z}, body.yaw + angle};
    if (brakingSurface) {
      const double cosine=std::cos(collision_.gridFromPlanningYaw);
      const double sine=std::sin(collision_.gridFromPlanningYaw);
      const double gridX=collision_.gridFromPlanningTranslation.x+
          cosine*predicted.position.x-sine*predicted.position.y;
      const double gridY=collision_.gridFromPlanningTranslation.y+
          sine*predicted.position.x+cosine*predicted.position.y;
      // shortcut: predict a locally planar base response; field use still needs
      // locomotion validation. Every part/contact retains its 3-D map query.
      predicted.position.z += brakingSurface->heightAt(gridX,gridY)-
          brakingSurface->heightAt(bodyInGrid.x,bodyInGrid.y);
    }
    return predicted;
  };
  if (body_) {
    const int startState = bodyPoseOccupancy(body.position, body.yaw);
    if (startState != 0) return startState;
    const auto end = poseAt(effectiveDuration);
    const int endState = bodyPoseOccupancy(end.position, end.yaw);
    if (endState != 0) return endState;
  }
  Pose previous = body;
  double previousTime = 0.0;
  double previousEffectiveTime = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double elapsed = duration * i / samples;
    const double braking = std::max(0.0, elapsed - reactionS);
    const double time = spatialSampling ? effectiveDuration * i / samples
        : (stopTime > 0.0 ? elapsed - 0.5 * braking * braking / stopTime : elapsed);
    const Pose next = poseAt(time);
    int occupied = 0;
    if (!supportedSegment(previous.position, next.position)) return 1;
    if (timedPredictionIntersects(previous.position, previous.yaw,
                                  next.position, next.yaw, previousTime, elapsed, false)) return 1;
    if (body_) {
      // The exact arc midpoint plus half the point-travel bound covers this
      // interval without inventing extra travel before braking starts.
      const double padding = spatialSampling ? .5 * samplingDistance / samples
          : .5 * bodyPointSpeed * (time - previousEffectiveTime);
      const auto midpoint = poseAt(.5 * (time + previousEffectiveTime));
      occupied = bodyPoseOccupancy(midpoint.position, midpoint.yaw, padding);
    } else {
      occupied = measuredSegmentOccupancy(previous.position, previous.yaw,
                                           next.position, next.yaw, false);
    }
    if (occupied != 0) return occupied;
    previous = next;
    previousTime = elapsed;
    previousEffectiveTime = time;
  }
  return 0;
}

double Grid::brakingScale(const Pose &body, const Twist &command,
                           double reactionS, double linearDeceleration,
                           double yawDeceleration) const noexcept {
  if (brakingOccupancy(body, command, reactionS, linearDeceleration, yawDeceleration) == 0)
    return 1.0;
  if (brakingOccupancy(body, {}, reactionS, linearDeceleration, yawDeceleration) != 0)
    return 0.0;

  // Scaling shortens the same braking arc. Height starts at the measured base;
  // only observed terrain, never a planned ascent, may change its prediction.
  double safe = 0.0;
  double blocked = 1.0;
  for (int iteration = 0; iteration < 8; ++iteration) {
    const double scale = (safe + blocked) * 0.5;
    const Twist candidate{command.vx * scale, command.vy * scale, command.wz * scale};
    if (brakingOccupancy(body, candidate, reactionS, linearDeceleration, yawDeceleration) == 0)
      safe = scale;
    else
      blocked = scale;
  }
  return safe;
}

int Grid::occupiedState(const Vec3 &planningPoint) const noexcept {
  if (!valid() || !std::isfinite(planningPoint.x) || !std::isfinite(planningPoint.y) ||
      !std::isfinite(planningPoint.z)) {
    return -1;
  }

  const double c = std::cos(collision_.gridFromPlanningYaw);
  const double s = std::sin(collision_.gridFromPlanningYaw);
  const Vec3 point{
      collision_.gridFromPlanningTranslation.x + c * planningPoint.x - s * planningPoint.y,
      collision_.gridFromPlanningTranslation.y + s * planningPoint.x + c * planningPoint.y,
      collision_.gridFromPlanningTranslation.z + planningPoint.z,
  };
  return occupiedGridState(point);
}

int Grid::occupiedGridState(const Vec3 &point) const noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    return -1;
  constexpr double boundaryEpsilon = 1e-4;
  if (point.x < collision_.aabbMin.x + boundaryEpsilon ||
      point.y < collision_.aabbMin.y + boundaryEpsilon ||
      point.z < collision_.aabbMin.z + boundaryEpsilon ||
      point.x > collision_.aabbMax.x - boundaryEpsilon ||
      point.y > collision_.aabbMax.y - boundaryEpsilon ||
      point.z > collision_.aabbMax.z - boundaryEpsilon) {
    return -1;
  }

  const auto logicalIndex = [this](double value, double minimum) {
    return GridAxis(minimum, collision_.resolution).index(value);
  };
  const std::int64_t x = logicalIndex(point.x, collision_.aabbMin.x);
  const std::int64_t y = logicalIndex(point.y, collision_.aabbMin.y);
  const std::int64_t z = logicalIndex(point.z, collision_.aabbMin.z);
  if (x < 0 || x >= collision_.sizeX || y < 0 || y >= collision_.sizeY ||
      z < 0 || z >= collision_.sizeZ) {
    return -1;
  }
  const std::size_t linear =
      (static_cast<std::size_t>(z) * static_cast<std::size_t>(collision_.sizeY) +
       static_cast<std::size_t>(y)) *
          static_cast<std::size_t>(collision_.sizeX) +
      static_cast<std::size_t>(x);
  return collision_.occupiedLinear(linear) ? 1 : 0;
}

bool Grid::boundaryDepartureMotionFree(const Pose &body, const Twist &command,
                                        const Twist &measured, double maxSpeed,
                                        double reactionS, double deceleration,
                                        double yawDeceleration) const noexcept {
  const double speed = std::hypot(command.vx, command.vy);
  const double measuredSpeed = std::hypot(measured.vx, measured.vy);
  if (!std::isfinite(speed) || !std::isfinite(command.wz) ||
      !std::isfinite(measuredSpeed) || !std::isfinite(measured.wz) ||
      !std::isfinite(maxSpeed) || maxSpeed <= 0.0 ||
      !std::isfinite(reactionS) || reactionS < 0.0 ||
      !std::isfinite(deceleration) || deceleration <= 0.0 ||
      !std::isfinite(yawDeceleration) || yawDeceleration <= 0.0 ||
      speed <= 1e-6 || speed > maxSpeed + 1e-6 || std::abs(command.wz) >= 1e-6 ||
      measuredSpeed > maxSpeed || std::abs(measured.wz) > 0.08) return false;
  constexpr double exitDistance = 0.35;
  // The entire reaction and braking distance must fit the checked exit.
  for (const auto &twist : {command,measured}) {
    const double linear = std::hypot(twist.vx,twist.vy);
    const double stop = std::max(linear/deceleration,std::abs(twist.wz)/yawDeceleration);
    if (linear * (reactionS + .5*stop) > exitDistance) return false;
  }
  const auto clearDirection = [&](const Twist &twist) {
    const double angle = body.yaw + std::atan2(twist.vy, twist.vx);
    return boundaryDepartureFree(body, {body.position.x + exitDistance * std::cos(angle),
                                        body.position.y + exitDistance * std::sin(angle),
                                        body.position.z});
  };
  // A valid commanded exit cannot excuse a controller moving into another cell.
  // Below the stop-evidence quiet threshold the commanded direction starts motion.
  if (!clearDirection(command) || (measuredSpeed > 0.03 && !clearDirection(measured)))
    return false;
  if (!body_) return true;
  const Grid source = sourceBodyGrid();
  std::vector<std::size_t> initialCells;
  const int startState = bodyPoseOccupancy(body.position,body.yaw);
  if (startState < 0) return false;
  if (startState > 0) {
    const double padding = .5 * std::min(bodySweepSpacingM, .25 * resolution());
    if (bodyPoseOccupancy(body.position,body.yaw,padding,&initialCells) < 0) return false;
  }
  const Grid departure = excludingCells(initialCells);
  for (const auto &twist : {command,measured}) {
    if (source.brakingOccupancy(body,twist,reactionS,deceleration,yawDeceleration) != 0 ||
        departure.brakingOccupancy(body,twist,reactionS,deceleration,yawDeceleration) != 0)
      return false;
  }
  return true;
}

Grid Grid::sourceBodyGrid() const {
  Grid result = *this;
  auto body = std::make_shared<lingtu::nav::collision::Body>(*body_);
  body->margin_m = 0;
  body->horizontal_margin_m = 0;
  result.body_ = std::move(body);
  return result;
}

Grid Grid::excludingCells(const std::vector<std::size_t> &cells) const {
  Grid result = *this;
  auto remaining = std::make_shared<std::vector<std::uint8_t>>(
      *collision_.measuredOccupiedStorage);
  for (const auto cell : cells)
    (*remaining)[cell / 8U] &= static_cast<std::uint8_t>(~(1U << (cell % 8U)));
  result.collision_.measuredOccupiedStorage = std::move(remaining);
  return result;
}

bool Grid::boundaryDepartureFree(const Pose &start, const Vec3 &end) const noexcept {
  if (body_) {
    if (!valid() || std::abs(start.position.z - end.z) > 1e-6 ||
        distance2D(start.position, end) > 0.5 ||
        bodyPredictionIntersects(start.position, start.yaw, end, start.yaw,
                                  0, 0, false, true) ||
        !supportedSegment(start.position, end) ||
        bodyPoseOccupancy(end, start.yaw) != 0) return false;
    const int startState = bodyPoseOccupancy(start.position, start.yaw);
    if (startState < 0) return false;
    if (startState == 0)
      return bodySweepOccupancy(start.position, start.yaw, end, start.yaw) == 0;

    // A recovery may leave its clearance buffer, but the source primitives
    // must remain clear along the entire exit, including continuous coverage.
    const Grid source = sourceBodyGrid();
    if (source.bodySweepOccupancy(start.position, start.yaw, end, start.yaw) != 0)
      return false;

    // Match midpoint sweep coverage so its sampling allowance cannot create a new start cell.
    std::vector<std::size_t> initialCells;
    const double padding = .5 * std::min(bodySweepSpacingM, .25 * resolution());
    if (bodyPoseOccupancy(start.position, start.yaw, padding, &initialCells) < 0) return false;
    // Only this candidate view excludes the already-overlapping buffer cells.
    // New obstacles retain full clearance; measured evidence is never changed.
    const Grid departure = excludingCells(initialCells);
    return departure.bodySweepOccupancy(start.position, start.yaw, end, start.yaw) == 0;
  }
  // The static padding exception never permits entering a moving object's
  // swept volume, even when that volume also covers the initial footprint.
  if (predictionIntersects(start.position, start.yaw, end, start.yaw)) return false;
  if (!valid() || !std::isfinite(start.yaw) ||
      std::abs(start.position.z - end.z) > 1e-6 ||
      distance2D(start.position, end) > 0.5 ||
      inflatedOccupancy(end, start.yaw) != 0) return false;
  const double c = std::cos(collision_.gridFromPlanningYaw);
  const double s = std::sin(collision_.gridFromPlanningYaw);
  const auto toGrid = [&](const Vec3 &p, double sign) {
    const double x = p.x + sign * cylinderOffset_ * std::cos(start.yaw);
    const double y = p.y + sign * cylinderOffset_ * std::sin(start.yaw);
    return Vec3{collision_.gridFromPlanningTranslation.x + c * x - s * y,
                collision_.gridFromPlanningTranslation.y + s * x + c * y,
                collision_.gridFromPlanningTranslation.z + p.z};
  };
  for (double sign : {-1.0, 1.0})
    if (occupiedGridSegment(toGrid(start.position, sign), toGrid(end, sign), true) != 0)
      return false;
  return true;
}

int Grid::occupiedGridSegment(const Vec3 &start, const Vec3 &end,
                              bool leaveInitialCell) const noexcept {
  const int startState = occupiedGridState(start);
  const int endState = occupiedGridState(end);
  if (startState < 0 || endState < 0) return -1;
  if ((!leaveInitialCell && startState != 0) || endState != 0) return 1;

  const double resolution = collision_.resolution;
  const std::array<GridAxis, 3> axes{{{collision_.aabbMin.x, resolution},
                                     {collision_.aabbMin.y, resolution},
                                     {collision_.aabbMin.z, resolution}}};
  const std::array<double, 3> from{axes[0].coordinate(start.x), axes[1].coordinate(start.y),
                                  axes[2].coordinate(start.z)};
  const std::array<double, 3> to{axes[0].coordinate(end.x), axes[1].coordinate(end.y),
                                axes[2].coordinate(end.z)};
  std::array<std::int64_t, 3> cell{}, target{};
  std::array<int, 3> step{};
  std::array<double, 3> next{}, delta{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    cell[axis] = static_cast<std::int64_t>(std::floor(from[axis])) - axes[axis].base;
    target[axis] = static_cast<std::int64_t>(std::floor(to[axis])) - axes[axis].base;
    next[axis] = std::numeric_limits<double>::infinity();
    if (cell[axis] == target[axis]) continue;
    const double direction = to[axis] - from[axis];
    step[axis] = direction > 0.0 ? 1 : -1;
    delta[axis] = 1.0 / std::abs(direction);
    const double boundary = std::floor(from[axis]) + (step[axis] > 0 ? 1.0 : 0.0);
    next[axis] = (boundary - from[axis]) / direction;
  }

  while (cell != target) {
    const double crossing = std::min({next[0], next[1], next[2]});
    const double tolerance = 4.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(crossing));
    const std::array<bool, 3> crosses{next[0] - crossing <= tolerance,
                                     next[1] - crossing <= tolerance,
                                     next[2] - crossing <= tolerance};
    // Floor assigns the boundary point to positive-axis cells before negative-axis exits.
    for (const int direction : {1, -1}) {
      bool moved = false;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        if (!crosses[axis] || step[axis] != direction) continue;
        cell[axis] += step[axis];
        next[axis] = cell[axis] == target[axis]
                         ? std::numeric_limits<double>::infinity()
                         : next[axis] + delta[axis];
        moved = true;
      }
      if (!moved) continue;
      const std::size_t linear =
          (static_cast<std::size_t>(cell[2]) * static_cast<std::size_t>(collision_.sizeY) +
           static_cast<std::size_t>(cell[1])) * static_cast<std::size_t>(collision_.sizeX) +
          static_cast<std::size_t>(cell[0]);
      if (collision_.occupiedLinear(linear)) return 1;
    }
  }
  return 0;
}

}  // namespace nav_kernel::local::scan
