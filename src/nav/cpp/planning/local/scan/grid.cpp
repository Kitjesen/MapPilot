#include "planning/surface_support.hpp"
// Mapd adapter for SCAN-Planner's GridMap query seam.
// Upstream algorithm commit: 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// SPDX-License-Identifier: Apache-2.0
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>

namespace nav_kernel::local::scan {
namespace {
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
    : checkObstacle_(params.checkObstacle),
      configuredResolution_(params.scan.voxelResolution),
      cylinderOffset_(std::max(0.0, params.scan.cylinderOffset)),
      support_(params.scan),
      robotYaw_(input.robot.pose.yaw),
      collision_(input.environment.collision) {
  // Assisted translation preserves body heading instead of following the
  // spline tangent. Collision queries must use the same footprint.
  if (input.intent() != nullptr) bodyHeading_ = input.robot.pose.yaw;
  if (!checkObstacle_) {
    reason_ = std::isfinite(configuredResolution_) && configuredResolution_ > 0.0
                  ? "ready"
                  : "grid_resolution_invalid";
    return;
  }
  if (!collision_.valid()) {
    return;
  }
  if (!collision_.complete) {
    reason_ = "collision_map_incomplete";
    return;
  }
  if (collision_.resetEpoch == 0U || collision_.observationSequence == 0U ||
      collision_.generation == 0U) {
    return;
  }
  if (std::abs(collision_.resolution - params.scan.voxelResolution) >
      std::max(1e-9, 1e-6 * params.scan.voxelResolution)) {
    reason_ = "collision_map_resolution_mismatch";
    return;
  }
  if (support_.supportHeight > 0.0 &&
      (!collision_.measuredOccupiedStorage || !collision_.knownFreeStorage ||
       collision_.measuredOccupiedStorage->size() != collision_.inflatedBytes ||
       collision_.knownFreeStorage->size() != collision_.inflatedBytes)) {
    reason_ = "ground_support_evidence_missing";
    return;
  }
  if (input.environment.predictions.fresh(input.clock.timestampS)) {
    const auto &view = input.environment.predictions;
    if (!std::isfinite(view.observedAtS) || !std::isfinite(view.horizonS) ||
        view.horizonS <= 0.0) {
      reason_ = "prediction_time_invalid";
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
        reason_ = "prediction_invalid";
        return;
      }
      prediction.radius += params.scan.cylinderRadius;
      prediction.minZ -= std::max(0.0, params.scan.bodyClearanceAbove);
      prediction.maxZ += std::max(0.0, params.scan.bodyClearanceBelow);
      predictions_.push_back(prediction);
    }
  }
  reason_ = "ready";
  const char *supportFailure = "robot_ground_support_unconfirmed";
  if (support_.supportHeight > 0.0 &&
      !supported(input.robot.pose.position, input.robot.pose.yaw, &supportFailure)) {
    reason_ = supportFailure;
  }
}

bool Grid::predictionIntersects(const Vec3 &start, double startYaw,
                                const Vec3 &end, double endYaw) const noexcept {
  startYaw = bodyHeading_.value_or(startYaw);
  endYaw = bodyHeading_.value_or(endYaw);
  for (const auto &prediction : predictions_) {
    if (std::max(start.z, end.z) < prediction.minZ ||
        std::min(start.z, end.z) > prediction.maxZ) continue;
    for (const double sign : {-1.0, 1.0}) {
      const Vec3 from{start.x + sign*cylinderOffset_*std::cos(startYaw),
                       start.y + sign*cylinderOffset_*std::sin(startYaw), start.z};
      const Vec3 to{end.x + sign*cylinderOffset_*std::cos(endYaw),
                     end.y + sign*cylinderOffset_*std::sin(endYaw), end.z};
      if (segmentSquared(from, to, prediction.start, prediction.end) <=
          prediction.radius * prediction.radius) return true;
    }
  }
  return false;
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
  if (!supported(center, yaw)) return 1;
  if (predictionIntersects(center, yaw, center, yaw)) return 1;
  yaw = bodyHeading_.value_or(yaw);
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
  if (predictionIntersects(start, startYaw, end, endYaw)) return 1;
  return measuredSegmentOccupancy(start, startYaw, end, endYaw);
}

bool Grid::timedPredictionIntersects(const Vec3 &start, double startYaw,
                                     const Vec3 &end, double endYaw,
                                     double startAheadS, double endAheadS) const noexcept {
  startYaw = bodyHeading_.value_or(startYaw);
  endYaw = bodyHeading_.value_or(endYaw);
  for (const auto &prediction : predictions_) {
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
      const Vec3 from{start.x + sign*cylinderOffset_*std::cos(startYaw),
                     start.y + sign*cylinderOffset_*std::sin(startYaw), 0.0};
      const Vec3 to{end.x + sign*cylinderOffset_*std::cos(endYaw),
                   end.y + sign*cylinderOffset_*std::sin(endYaw), 0.0};
      const auto relativeAt = [&](double ahead) {
        const double f = endAheadS > startAheadS ? (ahead-startAheadS)/(endAheadS-startAheadS) : 0.0;
        const auto center = centerAt(ahead);
        return Vec3{from.x + f*(to.x-from.x)-center.x,
                    from.y + f*(to.y-from.y)-center.y, 0.0};
      };
      for (int i = 0; i < 2; ++i) {
        if (pointSegmentSquared({}, relativeAt(times[i]), relativeAt(times[i+1])) <=
            prediction.radius * prediction.radius) return true;
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
  if (!checkObstacle_) return 0;
  const auto heading = [&](const Vec3 &v) {
    // Odometry below the stop-evidence quiet speed cannot define body heading.
    return bodyHeading_.value_or(std::hypot(v.x, v.y) <= 0.03
                                     ? robotYaw_ : std::atan2(v.y, v.x));
  };
  const double startYaw = startAheadS <= 1e-9 ? robotYaw_ : heading(velocity);
  const double endYaw = heading(endVelocity);
  if (timedPredictionIntersects(start, startYaw, end, endYaw, startAheadS, endAheadS)) return 1;
  return measuredSegmentOccupancy(start, startYaw, end, endYaw);
}

int Grid::measuredSegmentOccupancy(const Vec3 &start, double startYaw,
                                   const Vec3 &end, double endYaw) const noexcept {
  if (!supportedSegment(start, startYaw, end, endYaw)) return 1;
  startYaw = bodyHeading_.value_or(startYaw);
  endYaw = bodyHeading_.value_or(endYaw);
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
  const double startGridYaw = startYaw + collision_.gridFromPlanningYaw;
  const double endGridYaw = endYaw + collision_.gridFromPlanningYaw;
  const Vec3 startOffset{cylinderOffset_ * std::cos(startGridYaw),
                         cylinderOffset_ * std::sin(startGridYaw), 0.0};
  const Vec3 endOffset{cylinderOffset_ * std::cos(endGridYaw),
                       cylinderOffset_ * std::sin(endGridYaw), 0.0};
  for (const double sign : {1.0, -1.0}) {
    const int state = occupiedGridSegment(
        {startGrid.x + sign * startOffset.x, startGrid.y + sign * startOffset.y,
         startGrid.z},
        {endGrid.x + sign * endOffset.x, endGrid.y + sign * endOffset.y, endGrid.z});
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
  const double stopTime = std::max(speed / linearDeceleration,
                                    std::abs(command.wz) / yawDeceleration);
  // A linearly decreasing twist follows the same arc as a constant twist
  // over this effective duration, including command latency before braking.
  const double duration = reactionS + stopTime;
  const double effectiveDuration = reactionS + 0.5 * stopTime;
  const double sweptLength = (speed + cylinderOffset_ * std::abs(command.wz)) * effectiveDuration;
  const int samples = std::max(1, static_cast<int>(std::ceil(
      std::max({sweptLength / (0.5 * resolution()), std::abs(command.wz) * effectiveDuration / 0.05,
                duration / 0.01}))));
  Pose previous = body;
  double previousTime = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double elapsed = duration * i / samples;
    const double braking = std::max(0.0, elapsed - reactionS);
    const double time = stopTime > 0.0 ? elapsed - 0.5 * braking * braking / stopTime : elapsed;
    const double angle = command.wz * time;
    double x = command.vx * time;
    double y = command.vy * time;
    if (std::abs(command.wz) > 1e-6) {
      x = (command.vx * std::sin(angle) + command.vy * (std::cos(angle) - 1.0)) / command.wz;
      y = (command.vx * (1.0 - std::cos(angle)) + command.vy * std::sin(angle)) / command.wz;
    }
    const Pose next{{body.position.x + std::cos(body.yaw) * x - std::sin(body.yaw) * y,
                     body.position.y + std::sin(body.yaw) * x + std::cos(body.yaw) * y,
                     body.position.z}, body.yaw + angle};
    const int occupied = timedPredictionIntersects(previous.position, previous.yaw,
                                                    next.position, next.yaw, previousTime, elapsed)
                             ? 1 : measuredSegmentOccupancy(previous.position, previous.yaw,
                                                             next.position, next.yaw);
    if (occupied != 0) return occupied;
    previous = next;
    previousTime = elapsed;
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

  // Scaling the complete twist shortens the same braking arc. Keep the actual
  // body height; a planned ascent is not evidence that the robot has climbed.
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
    const auto global = static_cast<std::int64_t>(
        std::floor(value / collision_.resolution));
    const auto minimumIndex = static_cast<std::int64_t>(
        std::llround(minimum / collision_.resolution));
    return global - minimumIndex;
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
                                        double reactionS, double deceleration) const noexcept {
  const double speed = std::hypot(command.vx, command.vy);
  const double measuredSpeed = std::hypot(measured.vx, measured.vy);
  if (!std::isfinite(speed) || !std::isfinite(command.wz) ||
      !std::isfinite(measuredSpeed) || !std::isfinite(measured.wz) ||
      !std::isfinite(maxSpeed) || maxSpeed <= 0.0 ||
      !std::isfinite(reactionS) || reactionS < 0.0 ||
      !std::isfinite(deceleration) || deceleration <= 0.0 ||
      speed <= 1e-6 || speed > maxSpeed + 1e-6 || std::abs(command.wz) >= 1e-6 ||
      measuredSpeed > maxSpeed || std::abs(measured.wz) > 0.08) return false;
  constexpr double exitDistance = 0.35;
  const double worstSpeed = std::max(speed, measuredSpeed);
  // The entire reaction and braking distance must fit the checked exit.
  if (worstSpeed * reactionS + worstSpeed * worstSpeed / (2.0 * deceleration) >
      exitDistance) return false;
  const auto clearDirection = [&](const Twist &twist) {
    const double angle = body.yaw + std::atan2(twist.vy, twist.vx);
    return boundaryDepartureFree(body, {body.position.x + exitDistance * std::cos(angle),
                                        body.position.y + exitDistance * std::sin(angle),
                                        body.position.z});
  };
  // A valid commanded exit cannot excuse a controller moving into another cell.
  // Below the stop-evidence quiet threshold the commanded direction starts motion.
  return clearDirection(command) && (measuredSpeed <= 0.03 || clearDirection(measured));
}

bool Grid::boundaryDepartureFree(const Pose &start, const Vec3 &end) const noexcept {
  // The static padding exception never permits entering a moving object's
  // swept volume, even when that volume also covers the initial footprint.
  if (predictionIntersects(start.position, start.yaw, end, start.yaw)) return false;
  if (!valid() || !std::isfinite(start.yaw) ||
      std::abs(start.position.z - end.z) > 1e-6 ||
      distance2D(start.position, end) > 0.5 ||
      inflatedOccupancy(end, start.yaw) != 0) return false;
  if (!supportedSegment(start.position, start.yaw, end, start.yaw)) return false;
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

bool Grid::evidenceBit(const std::vector<std::uint8_t> &bits,
                       int x, int y, int z) const noexcept {
  if (x < 0 || x >= collision_.sizeX || y < 0 || y >= collision_.sizeY ||
      z < 0 || z >= collision_.sizeZ) return false;
  const auto index = (static_cast<std::size_t>(z) * collision_.sizeY + y) *
                     collision_.sizeX + x;
  return (bits[index / 8U] & (1U << (index % 8U))) != 0U;
}

bool Grid::supportPatch(double x, double y, double bodyZ, double &height,
                        const char **failure) const noexcept {
  const double r=collision_.resolution;
  const int ox=int(std::llround(collision_.aabbMin.x/r));
  const int oy=int(std::llround(collision_.aabbMin.y/r));
  const int oz=int(std::llround(collision_.aabbMin.z/r));
  const support::SurfaceQuery query{r,support_.supportHeight,support_.supportHeightTolerance,
                                     support_.maxSupportSlope,oz,oz+collision_.sizeZ-1};
  return support::surfacePatch(query,x,y,bodyZ,
      [&](int ix,int iy,int iz) { return evidenceBit(*collision_.measuredOccupiedStorage,ix-ox,iy-oy,iz-oz); },
      [&](int ix,int iy,int iz) { return evidenceBit(*collision_.knownFreeStorage,ix-ox,iy-oy,iz-oz); },
      height,failure);
}

bool Grid::supported(const Vec3 &center, double yaw,
                     const char **failure) const noexcept {
  if (support_.supportHeight <= 0.0) return true;
  if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
      !std::isfinite(yaw) || !collision_.covers(center)) return false;
  yaw = bodyHeading_.value_or(yaw) + collision_.gridFromPlanningYaw;
  const double c = std::cos(collision_.gridFromPlanningYaw);
  const double s = std::sin(collision_.gridFromPlanningYaw);
  const Vec3 p{collision_.gridFromPlanningTranslation.x + c * center.x - s * center.y,
               collision_.gridFromPlanningTranslation.y + s * center.x + c * center.y,
               collision_.gridFromPlanningTranslation.z + center.z};
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double width = support_.cylinderRadius;
  const std::array<Vec3, 7> samples{{{0, 0, 0},
      {cylinderOffset_, width, 0}, {cylinderOffset_, -width, 0},
      {-cylinderOffset_, width, 0}, {-cylinderOffset_, -width, 0},
      {cylinderOffset_ + width, 0, 0}, {-cylinderOffset_ - width, 0, 0}}};
  double baseHeight = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto &o = samples[i];
    double height = 0.0;
    if (!supportPatch(p.x + cy * o.x - sy * o.y, p.y + sy * o.x + cy * o.y,
                      p.z, height, failure)) return false;
    if (i == 0) baseHeight = height;
    if (std::abs(height - baseHeight) > support_.maxStepHeight + 1e-9) return false;
  }
  return true;
}

bool Grid::supportedSegment(const Vec3 &start, double startYaw,
                             const Vec3 &end, double endYaw) const noexcept {
  if (support_.supportHeight <= 0.0) return true;
  if (!collision_.covers(start) || !collision_.covers(end) ||
      !std::isfinite(startYaw) || !std::isfinite(endYaw)) return false;
  const double yawDelta = std::remainder(endYaw - startYaw, 2.0 * std::acos(-1.0));
  const double distance = std::hypot(std::hypot(end.x - start.x, end.y - start.y), end.z - start.z);
  const double swept = distance + (cylinderOffset_ + support_.cylinderRadius) * std::abs(yawDelta);
  const int count = std::max(1, static_cast<int>(std::ceil(swept / (0.5 * collision_.resolution))));
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    if (!supported({start.x + t * (end.x - start.x), start.y + t * (end.y - start.y),
                     start.z + t * (end.z - start.z)}, startYaw + t * yawDelta)) return false;
  }
  return true;
}

int Grid::occupiedGridSegment(const Vec3 &start, const Vec3 &end,
                              bool leaveInitialCell) const noexcept {
  const int startState = occupiedGridState(start);
  const int endState = occupiedGridState(end);
  if (startState < 0 || endState < 0) return -1;
  if ((!leaveInitialCell && startState != 0) || endState != 0) return 1;

  const double resolution = collision_.resolution;
  const std::array<double, 3> from{start.x / resolution, start.y / resolution,
                                  start.z / resolution};
  const std::array<double, 3> to{end.x / resolution, end.y / resolution,
                                end.z / resolution};
  const std::array<double, 3> minimum{collision_.aabbMin.x, collision_.aabbMin.y,
                                     collision_.aabbMin.z};
  std::array<std::int64_t, 3> cell{}, target{};
  std::array<int, 3> step{};
  std::array<double, 3> next{}, delta{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto base = static_cast<std::int64_t>(std::llround(minimum[axis] / resolution));
    cell[axis] = static_cast<std::int64_t>(std::floor(from[axis])) - base;
    target[axis] = static_cast<std::int64_t>(std::floor(to[axis])) - base;
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
