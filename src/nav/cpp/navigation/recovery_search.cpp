#include "navigation/recovery_search.hpp"
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace lingtu::nav::navigation::detail {
namespace {

class RecoverySearchImplementation {
 public:
  explicit RecoverySearchImplementation(
      const RecoverySearchParams& params = RecoverySearchParams())
      : p_(params) {}

  const RecoverySearchParams& params() const { return p_; }

  RecoverySearchResult plan(const RecoverySearchInput& input) const {
    RecoverySearchResult out;
    RecoverySafetyFailure invalid = RecoverySafetyFailure::None;
    if (!validateInput(input, &invalid)) {
      out.status = RecoverySearchStatus::InvalidInput;
      out.safetyFailure = invalid;
      out.diagnostics.lastFailure = invalid;
      return out;
    }
    if (input.collisionGrid != nullptr &&
        !input.collisionGrid->obstacleFree(input.vehiclePose.position, input.vehiclePose.yaw)) {
      return boundaryDeparture(input);
    }
    if (searchTranslation(input, &out)) return out;
    if (out.status == RecoverySearchStatus::InvalidInput) return out;

    RecoverySafetyFailure rotationFailure = RecoverySafetyFailure::None;
    double bestDelta = 0.0;
    double bestReach = -1.0;
    double bestScore = -std::numeric_limits<double>::infinity();
    const bool preferPositive = input.goalDirectionBodyRad >= 0.0;
    for (double magnitude = p_.minRotationRad;;) {
      const std::array<double, 2> deltas =
          preferPositive ? std::array<double, 2>{{magnitude, -magnitude}}
                         : std::array<double, 2>{{-magnitude, magnitude}};
      for (double delta : deltas) {
        const int bit = delta > 0.0 ? 0x1 : 0x2;
        if ((input.rejectedRotationDirectionMask & bit) != 0) continue;
        ++out.diagnostics.rotationCandidateCount;
        RecoverySafetyFailure candidateFailure = RecoverySafetyFailure::None;
        if (!validateRotation(input, delta, &candidateFailure)) {
          rotationFailure = candidateFailure;
          continue;
        }
        const double reach = forwardReachAfterRotation(input, delta);
        const double alignment =
            std::cos(nav_kernel::normalizeAngle(input.goalDirectionBodyRad - delta));
        const double score = reach + p_.goalDirectionWeight * alignment;
        const bool better =
            score > bestScore + kScoreEpsilon ||
            (std::fabs(score - bestScore) <= kScoreEpsilon &&
             std::fabs(delta) < std::fabs(bestDelta) - kEpsilon);
        if (better) {
          bestDelta = delta;
          bestReach = reach;
          bestScore = score;
        }
      }
      if (magnitude >= p_.maxRotationRad - kEpsilon) break;
      magnitude = std::min(
          p_.maxRotationRad, magnitude + p_.rotationCandidateStepRad);
    }
    if (std::fabs(bestDelta) > kEpsilon) {
      out.status = RecoverySearchStatus::RotationReady;
      out.action = nav_kernel::RecoveryAction::Rotate;
      out.rotationDeltaRad = bestDelta;
      out.safetyFailure = RecoverySafetyFailure::None;
      out.diagnostics.selectedRotationRad = bestDelta;
      out.diagnostics.selectedForwardReachM = bestReach;
      out.diagnostics.selectedScore = bestScore;
      out.diagnostics.lastFailure = RecoverySafetyFailure::None;
      out.verified = true;
      return out;
    }

    out.status = RecoverySearchStatus::NoSafeCandidate;
    out.safetyFailure =
        rotationFailure != RecoverySafetyFailure::None
            ? rotationFailure
            : out.diagnostics.lastFailure;
    out.diagnostics.lastFailure = out.safetyFailure;
    return out;
  }

  bool validateBodyPath(
      const RecoverySearchInput& input,
      const std::vector<nav_kernel::Vec3>& bodyPath,
      RecoverySafetyFailure* failure = nullptr) const {
    setFailure(failure, RecoverySafetyFailure::None);
    RecoverySafetyFailure invalid = RecoverySafetyFailure::None;
    if (!validateInput(input, &invalid) || bodyPath.empty()) {
      setFailure(failure, invalid == RecoverySafetyFailure::None
                              ? RecoverySafetyFailure::InvalidInput
                              : invalid);
      return false;
    }
    for (const nav_kernel::Vec3& p : bodyPath) {
      if (!finite(p.x) || !finite(p.y) || !finite(p.z)) {
        setFailure(failure, RecoverySafetyFailure::InvalidInput);
        return false;
      }
    }
    RecoverySafetyFailure why = RecoverySafetyFailure::None;
    if (input.collisionGrid != nullptr && bodyPath.size() == 2 &&
        distance2D(bodyPath.front(), {}) < 1e-6 &&
        !input.collisionGrid->obstacleFree(input.vehiclePose.position, input.vehiclePose.yaw)) {
      auto end = bodyPathToWorld(bodyPath, input.vehiclePose).back();
      // Recovery is planar; body bobbing must not turn the retained waypoint
      // height into a requested ascent or invalidate an otherwise safe exit.
      end.z = input.vehiclePose.position.z;
      const bool free = input.collisionGrid->boundaryDepartureFree(input.vehiclePose, end);
      setFailure(failure, free ? RecoverySafetyFailure::None : RecoverySafetyFailure::ObstacleCollision);
      return free;
    }
    if (!safePose(input, bodyPath.front().x, bodyPath.front().y, 0.0, &why)) {
      setFailure(failure, why);
      return false;
    }
    for (std::size_t i = 1; i < bodyPath.size(); ++i) {
      if (!safeEdge(input, bodyPath[i - 1].x, bodyPath[i - 1].y,
                    bodyPath[i].x, bodyPath[i].y, &why)) {
        setFailure(failure, why);
        return false;
      }
    }
    return true;
  }
  bool validateRotation(
      const RecoverySearchInput& input,
      double rotationDeltaRad,
      RecoverySafetyFailure* failure = nullptr) const {
    setFailure(failure, RecoverySafetyFailure::None);
    RecoverySafetyFailure invalid = RecoverySafetyFailure::None;
    if (!validateInput(input, &invalid) ||
        !finite(rotationDeltaRad) ||
        std::fabs(rotationDeltaRad) <= kEpsilon) {
      setFailure(failure, invalid == RecoverySafetyFailure::None
                              ? RecoverySafetyFailure::InvalidInput
                              : invalid);
      return false;
    }

    const int samples = std::max(
        1, static_cast<int>(std::ceil(
               std::fabs(rotationDeltaRad) / p_.rotationSampleStepRad)));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      RecoverySafetyFailure why = RecoverySafetyFailure::None;
      if (!safePose(input, 0.0, 0.0, rotationDeltaRad * t, &why)) {
        setFailure(failure, why);
        return false;
      }
    }
    return true;
  }

  static std::vector<nav_kernel::Vec3> bodyPathToWorld(
      const std::vector<nav_kernel::Vec3>& bodyPath,
      const nav_kernel::Pose& vehiclePose) {
    std::vector<nav_kernel::Vec3> world;
    world.reserve(bodyPath.size());
    const double c = std::cos(vehiclePose.yaw);
    const double s = std::sin(vehiclePose.yaw);
    for (const nav_kernel::Vec3& p : bodyPath) {
      world.push_back({
          vehiclePose.position.x + c * p.x - s * p.y,
          vehiclePose.position.y + s * p.x + c * p.y,
          vehiclePose.position.z + p.z,
      });
    }
    return world;
  }

  static std::vector<nav_kernel::Vec3> worldPathToBody(
      const std::vector<nav_kernel::Vec3>& worldPath,
      const nav_kernel::Pose& vehiclePose) {
    std::vector<nav_kernel::Vec3> body;
    body.reserve(worldPath.size());
    const double c = std::cos(vehiclePose.yaw);
    const double s = std::sin(vehiclePose.yaw);
    for (const nav_kernel::Vec3& p : worldPath) {
      const double dx = p.x - vehiclePose.position.x;
      const double dy = p.y - vehiclePose.position.y;
      body.push_back({
          c * dx + s * dy,
          -s * dx + c * dy,
          p.z - vehiclePose.position.z,
      });
    }
    return body;
  }

 private:
  static constexpr double kEpsilon = 1e-9;
  static constexpr double kScoreEpsilon = 1e-12;
  static constexpr int kDirectionBins = 16;

  RecoverySearchParams p_;

  RecoverySearchResult boundaryDeparture(const RecoverySearchInput& in) const {
    RecoverySearchResult out;
    for (int bin = 0; bin < kDirectionBins; ++bin) {
      if (in.rejectedTranslationDirectionMask & (std::uint32_t{1} << bin)) continue;
      const double angle = 2.0 * M_PI * bin / kDirectionBins;
      const std::vector<nav_kernel::Vec3> body{{}, {p_.minTranslationDistance * std::cos(angle),
                                       p_.minTranslationDistance * std::sin(angle), 0.0}};
      const auto world = bodyPathToWorld(body, in.vehiclePose);
      if (!in.collisionGrid->boundaryDepartureFree(in.vehiclePose, world.back())) continue;
      ++out.diagnostics.candidateCount;
      const double score = std::cos(angle - in.goalDirectionBodyRad);
      if (score <= out.diagnostics.selectedScore) continue;
      out.status = RecoverySearchStatus::TranslationReady;
      out.action = nav_kernel::RecoveryAction::Translate;
      out.pathBody = body;
      out.pathWorld = world;
      out.verified = true;
      out.diagnostics.selectedScore = score;
      out.diagnostics.selectedDirectionBin = bin;
    }
    if (!out.verified) out.safetyFailure = RecoverySafetyFailure::ObstacleCollision;
    return out;
  }

  static bool finite(double value) { return std::isfinite(value); }

  static void setFailure(RecoverySafetyFailure* out, RecoverySafetyFailure value) {
    if (out != nullptr) *out = value;
  }

  bool validateInput(
      const RecoverySearchInput& in,
      RecoverySafetyFailure* failure) const {
    setFailure(failure, RecoverySafetyFailure::None);
    const bool paramsValid =
        finite(p_.vehicleLength) && p_.vehicleLength > 0.0 &&
        finite(p_.vehicleWidth) && p_.vehicleWidth > 0.0 &&
        finite(p_.footprintPadding) && p_.footprintPadding >= 0.0 &&
        finite(p_.obstacleHeightThreshold) &&
        finite(p_.traversabilityHardCost) &&
        p_.traversabilityHardCost >= 0.0 &&
        p_.traversabilityHardCost <= 100.0 &&
        finite(p_.searchRadius) && p_.searchRadius > 0.0 &&
        finite(p_.latticeResolution) && p_.latticeResolution > 0.0 &&
        finite(p_.edgeSampleResolution) && p_.edgeSampleResolution > 0.0 &&
        finite(p_.minTranslationDistance) &&
        p_.minTranslationDistance > 0.0 &&
        p_.minTranslationDistance <= p_.searchRadius &&
        finite(p_.portalMargin) && p_.portalMargin >= 0.0 &&
        finite(p_.minRotationRad) && p_.minRotationRad > 0.0 &&
        finite(p_.maxRotationRad) && p_.maxRotationRad >= p_.minRotationRad &&
        p_.maxRotationRad <= M_PI &&
        finite(p_.rotationCandidateStepRad) && p_.rotationCandidateStepRad > 0.0 &&
        finite(p_.rotationSampleStepRad) &&
        p_.rotationSampleStepRad > 0.0 &&
        p_.rotationSampleStepRad <= p_.minRotationRad &&
        finite(p_.clearanceWeight) && p_.clearanceWeight >= 0.0 &&
        finite(p_.goalDirectionWeight) && p_.goalDirectionWeight >= 0.0 &&
        p_.maxSearchNodes > 0 && p_.maxObstaclePoints >= 0;
    if (!paramsValid ||
        !finite(in.vehiclePose.position.x) ||
        !finite(in.vehiclePose.position.y) ||
        !finite(in.vehiclePose.position.z) ||
        !finite(in.vehiclePose.yaw) ||
        !finite(in.goalDirectionBodyRad) ||
        in.obstacleCount < 0 ||
        in.obstacleCount > p_.maxObstaclePoints) {
      setFailure(failure, RecoverySafetyFailure::InvalidInput);
      return false;
    }

    if (in.obstacleCount > 0 &&
        (in.obstacleX == nullptr || in.obstacleY == nullptr ||
         in.obstacleHeight == nullptr)) {
      setFailure(failure, RecoverySafetyFailure::InvalidInput);
      return false;
    }
    for (int i = 0; i < in.obstacleCount; ++i) {
      if (!std::isfinite(in.obstacleX[i]) ||
          !std::isfinite(in.obstacleY[i]) ||
          !std::isfinite(in.obstacleHeight[i])) {
        setFailure(failure, RecoverySafetyFailure::InvalidInput);
        return false;
      }
    }

    const bool anyGrid =
        in.traversabilityGrid != nullptr ||
        in.traversabilityRows != 0 ||
        in.traversabilityCols != 0 ||
        in.traversabilityResolution != 0.0;
    if (!anyGrid) {
      if (p_.requireTraversability) {
        setFailure(failure, RecoverySafetyFailure::TraversabilityUnavailable);
        return false;
      }
      return true;
    }
    if (in.traversabilityGrid == nullptr ||
        in.traversabilityRows <= 0 ||
        in.traversabilityCols <= 0 ||
        !finite(in.traversabilityResolution) ||
        in.traversabilityResolution <= 0.0 ||
        !finite(in.traversabilityOriginX) ||
        !finite(in.traversabilityOriginY) ||
        in.traversabilityRows >
            std::numeric_limits<int>::max() / in.traversabilityCols) {
      setFailure(failure, RecoverySafetyFailure::InvalidInput);
      return false;
    }
    return true;
  }
  static int directionBin(double x, double y) {
    constexpr double twoPi = 2.0 * M_PI;
    constexpr double width = twoPi / static_cast<double>(kDirectionBins);
    double angle = std::atan2(y, x);
    if (angle < 0.0) angle += twoPi;
    const int bin =
        static_cast<int>(std::floor((angle + 0.5 * width) / width));
    return bin % kDirectionBins;
  }

  bool admitted(float height) const {
    return !p_.useTerrainAnalysis ||
           static_cast<double>(height) >= p_.obstacleHeightThreshold;
  }

  bool safePose(
      const RecoverySearchInput& in,
      double bodyX,
      double bodyY,
      double relativeYaw,
      RecoverySafetyFailure* failure) const {
    setFailure(failure, RecoverySafetyFailure::None);
    if (!finite(bodyX) || !finite(bodyY) || !finite(relativeYaw)) {
      setFailure(failure, RecoverySafetyFailure::InvalidInput);
      return false;
    }

    const double halfLength =
        p_.vehicleLength * 0.5 + p_.footprintPadding;
    const double halfWidth =
        p_.vehicleWidth * 0.5 + p_.footprintPadding;
    const double c = std::cos(relativeYaw);
    const double s = std::sin(relativeYaw);

    // Collision is checked against the full padded rectangle, not a centre
    // strip. The same check is used for forward, lateral, and reverse motion.
    if (in.collisionGrid != nullptr) {
      const double vc = std::cos(in.vehiclePose.yaw);
      const double vs = std::sin(in.vehiclePose.yaw);
      const nav_kernel::Vec3 center{in.vehiclePose.position.x + vc * bodyX - vs * bodyY,
                        in.vehiclePose.position.y + vs * bodyX + vc * bodyY,
                        in.vehiclePose.position.z};
      if (!in.collisionGrid->obstacleFree(center, in.vehiclePose.yaw + relativeYaw)) {
        setFailure(failure, RecoverySafetyFailure::ObstacleCollision);
        return false;
      }
    } else if (p_.checkObstacles) {
      for (int i = 0; i < in.obstacleCount; ++i) {
        if (!admitted(in.obstacleHeight[i])) continue;
        const double dx = static_cast<double>(in.obstacleX[i]) - bodyX;
        const double dy = static_cast<double>(in.obstacleY[i]) - bodyY;
        const double localX = c * dx + s * dy;
        const double localY = -s * dx + c * dy;
        if (std::fabs(localX) <= halfLength + kEpsilon &&
            std::fabs(localY) <= halfWidth + kEpsilon) {
          setFailure(failure, RecoverySafetyFailure::ObstacleCollision);
          return false;
        }
      }
    }

    if (in.traversabilityGrid == nullptr) {
      if (p_.requireTraversability) {
        setFailure(failure, RecoverySafetyFailure::TraversabilityUnavailable);
        return false;
      }
      return true;
    }

    const double vc = std::cos(in.vehiclePose.yaw);
    const double vs = std::sin(in.vehiclePose.yaw);
    const double worldX =
        in.vehiclePose.position.x + vc * bodyX - vs * bodyY;
    const double worldY =
        in.vehiclePose.position.y + vs * bodyX + vc * bodyY;
    return safeGridFootprint(
        in, worldX, worldY,
        nav_kernel::normalizeAngle(in.vehiclePose.yaw + relativeYaw),
        halfLength, halfWidth, failure);
  }

  bool safeEdge(
      const RecoverySearchInput& in,
      double fromX,
      double fromY,
      double toX,
      double toY,
      RecoverySafetyFailure* failure) const {
    if (in.collisionGrid != nullptr) {
      const double c = std::cos(in.vehiclePose.yaw);
      const double s = std::sin(in.vehiclePose.yaw);
      const auto world = [&](double x, double y) {
        return nav_kernel::Vec3{in.vehiclePose.position.x + c * x - s * y,
                    in.vehiclePose.position.y + s * x + c * y,
                    in.vehiclePose.position.z};
      };
      if (in.collisionGrid->segmentInflatedOccupancy(
              world(fromX, fromY), in.vehiclePose.yaw,
              world(toX, toY), in.vehiclePose.yaw) != 0) {
        setFailure(failure, RecoverySafetyFailure::ObstacleCollision);
        return false;
      }
    }
    const double length = std::hypot(toX - fromX, toY - fromY);
    const int samples = std::max(
        1, static_cast<int>(
               std::ceil(length / p_.edgeSampleResolution)));
    for (int i = 1; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      RecoverySafetyFailure why = RecoverySafetyFailure::None;
      if (!safePose(in,
                    fromX + (toX - fromX) * t,
                    fromY + (toY - fromY) * t,
                    0.0, &why)) {
        setFailure(failure, why);
        return false;
      }
    }
    setFailure(failure, RecoverySafetyFailure::None);
    return true;
  }

  bool safeGridFootprint(
      const RecoverySearchInput& in,
      double centerX,
      double centerY,
      double yaw,
      double halfLength,
      double halfWidth,
      RecoverySafetyFailure* failure) const {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double absC = std::fabs(c);
    const double absS = std::fabs(s);
    const double extentX = halfLength * absC + halfWidth * absS;
    const double extentY = halfLength * absS + halfWidth * absC;
    const double minX = centerX - extentX;
    const double maxX = centerX + extentX;
    const double minY = centerY - extentY;
    const double maxY = centerY + extentY;
    const double res = in.traversabilityResolution;
    const double gridMaxX =
        in.traversabilityOriginX +
        static_cast<double>(in.traversabilityCols) * res;
    const double gridMaxY =
        in.traversabilityOriginY +
        static_cast<double>(in.traversabilityRows) * res;

    // Recovery is fail-closed outside known traversability coverage.
    if (minX < in.traversabilityOriginX - kEpsilon ||
        minY < in.traversabilityOriginY - kEpsilon ||
        maxX > gridMaxX + kEpsilon ||
        maxY > gridMaxY + kEpsilon) {
      setFailure(failure, RecoverySafetyFailure::TraversabilityOutOfBounds);
      return false;
    }

    const int minCol = std::max(
        0, static_cast<int>(
               std::floor((minX - in.traversabilityOriginX) / res)));
    const int maxCol = std::min(
        in.traversabilityCols - 1,
        static_cast<int>(std::floor(
            (maxX - in.traversabilityOriginX - kEpsilon) / res)));
    const int minRow = std::max(
        0, static_cast<int>(
               std::floor((minY - in.traversabilityOriginY) / res)));
    const int maxRow = std::min(
        in.traversabilityRows - 1,
        static_cast<int>(std::floor(
            (maxY - in.traversabilityOriginY - kEpsilon) / res)));

    const double cellHalf = res * 0.5;
    for (int row = minRow; row <= maxRow; ++row) {
      const double cellY =
          in.traversabilityOriginY + (row + 0.5) * res;
      for (int col = minCol; col <= maxCol; ++col) {
        const double cellX =
            in.traversabilityOriginX + (col + 0.5) * res;
        if (!obbIntersectsCell(centerX, centerY, c, s,
                               halfLength, halfWidth,
                               cellX, cellY, cellHalf)) {
          continue;
        }
        if (gridCellFullyInsideInitialFootprint(
                in, halfLength, halfWidth, cellX, cellY, cellHalf)) {
          continue;
        }
        const float raw =
            in.traversabilityGrid[row * in.traversabilityCols + col];
        if (!std::isfinite(raw)) {
          setFailure(failure, RecoverySafetyFailure::TraversabilityNonFinite);
          return false;
        }
        const double risk = static_cast<double>(raw);
        if (risk < 0.0 || risk > 100.0) {
          setFailure(failure, RecoverySafetyFailure::TraversabilityInvalidValue);
          return false;
        }
        if (risk >= p_.traversabilityHardCost) {
          setFailure(failure, RecoverySafetyFailure::TraversabilityBlocked);
          return false;
        }
      }
    }
    setFailure(failure, RecoverySafetyFailure::None);
    return true;
  }

  static bool obbIntersectsCell(
      double rectX,
      double rectY,
      double c,
      double s,
      double halfLength,
      double halfWidth,
      double cellX,
      double cellY,
      double cellHalf) {
    const double dx = cellX - rectX;
    const double dy = cellY - rectY;
    const double absC = std::fabs(c);
    const double absS = std::fabs(s);

    // Four separating axes: the two world axes and the two robot axes.
    // Boundary-only contact is not an overlap with the blocked cell.
    if (std::fabs(dx) >=
        cellHalf + halfLength * absC + halfWidth * absS - kEpsilon) {
      return false;
    }
    if (std::fabs(dy) >=
        cellHalf + halfLength * absS + halfWidth * absC - kEpsilon) {
      return false;
    }
    if (std::fabs(dx * c + dy * s) >=
        halfLength + cellHalf * (absC + absS) - kEpsilon) {
      return false;
    }
    if (std::fabs(-dx * s + dy * c) >=
        halfWidth + cellHalf * (absS + absC) - kEpsilon) {
      return false;
    }
    return true;
  }

  static bool gridCellFullyInsideInitialFootprint(
      const RecoverySearchInput& in,
      double halfLength,
      double halfWidth,
      double cellX,
      double cellY,
      double cellHalf) {
    const double c = std::cos(in.vehiclePose.yaw);
    const double s = std::sin(in.vehiclePose.yaw);
    const double dx = cellX - in.vehiclePose.position.x;
    const double dy = cellY - in.vehiclePose.position.y;
    const double localX = c * dx + s * dy;
    const double localY = -s * dx + c * dy;
    const double projectedCellHalf =
        cellHalf * (std::fabs(c) + std::fabs(s));
    return std::fabs(localX) + projectedCellHalf <=
               halfLength + kEpsilon &&
           std::fabs(localY) + projectedCellHalf <=
               halfWidth + kEpsilon;
  }

  double obstacleClearance(
      const RecoverySearchInput& in,
      double bodyX,
      double bodyY) const {
    if (!p_.checkObstacles || in.obstacleCount == 0) {
      return p_.searchRadius;
    }
    const double halfLength =
        p_.vehicleLength * 0.5 + p_.footprintPadding;
    const double halfWidth =
        p_.vehicleWidth * 0.5 + p_.footprintPadding;
    double best = p_.searchRadius;
    for (int i = 0; i < in.obstacleCount; ++i) {
      if (!admitted(in.obstacleHeight[i])) continue;
      const double dx = std::max(
          std::fabs(static_cast<double>(in.obstacleX[i]) - bodyX) -
              halfLength,
          0.0);
      const double dy = std::max(
          std::fabs(static_cast<double>(in.obstacleY[i]) - bodyY) -
              halfWidth,
          0.0);
      best = std::min(best, std::hypot(dx, dy));
    }
    return best;
  }

  double forwardReachAfterRotation(const RecoverySearchInput& in,
                                   double rotationDeltaRad) const {
    const double step = std::min(p_.edgeSampleResolution, p_.latticeResolution);
    const double c = std::cos(rotationDeltaRad);
    const double s = std::sin(rotationDeltaRad);
    double reach = 0.0;
    for (double distance = step; distance <= p_.searchRadius + kEpsilon;
         distance += step) {
      RecoverySafetyFailure failure = RecoverySafetyFailure::None;
      if (!safePose(in, c * distance, s * distance, rotationDeltaRad, &failure)) {
        break;
      }
      reach = std::min(distance, p_.searchRadius);
    }
    return reach;
  }

  bool searchTranslation(
      const RecoverySearchInput& in,
      RecoverySearchResult* out) const {
    const double halfValue =
        std::floor(p_.searchRadius / p_.latticeResolution);
    if (!finite(halfValue) || halfValue < 1.0 ||
        halfValue >
            static_cast<double>(std::numeric_limits<int>::max() / 2 - 1)) {
      out->status = RecoverySearchStatus::InvalidInput;
      out->safetyFailure = RecoverySafetyFailure::InvalidInput;
      out->diagnostics.lastFailure = RecoverySafetyFailure::InvalidInput;
      return false;
    }
    const int half = static_cast<int>(halfValue);
    const int side = half * 2 + 1;
    if (side > p_.maxSearchNodes ||
        side > p_.maxSearchNodes / side) {
      out->status = RecoverySearchStatus::InvalidInput;
      out->safetyFailure = RecoverySafetyFailure::InvalidInput;
      out->diagnostics.lastFailure = RecoverySafetyFailure::InvalidInput;
      return false;
    }

    struct Node {
      int ix{0};
      int iy{0};
      int parent{-1};
      int firstBin{-1};
      bool visited{false};
    };
    const int capacity = side * side;
    std::vector<Node> nodes(static_cast<std::size_t>(capacity));
    std::vector<int> queue;
    queue.reserve(static_cast<std::size_t>(capacity));
    const auto indexOf = [half, side](int ix, int iy) {
      return (iy + half) * side + ix + half;
    };

    RecoverySafetyFailure why = RecoverySafetyFailure::None;
    if (!safePose(in, 0.0, 0.0, 0.0, &why)) {
      out->diagnostics.lastFailure = why;
      return false;
    }

    const int root = indexOf(0, 0);
    nodes[static_cast<std::size_t>(root)] = {0, 0, -1, -1, true};
    queue.push_back(root);

    static constexpr std::array<std::array<int, 2>, 8> neighbors{{
        {{1, 0}}, {{1, 1}}, {{0, 1}}, {{-1, 1}},
        {{-1, 0}}, {{-1, -1}}, {{0, -1}}, {{1, -1}},
    }};

    int bestIndex = -1;
    bool bestPortal = false;
    double bestScore = -std::numeric_limits<double>::infinity();
    std::size_t head = 0;
    while (head < queue.size()) {
      const int currentIndex = queue[head++];
      const Node current = nodes[static_cast<std::size_t>(currentIndex)];
      ++out->diagnostics.expandedNodeCount;

      for (const auto& d : neighbors) {
        const int nx = current.ix + d[0];
        const int ny = current.iy + d[1];
        if (nx < -half || nx > half || ny < -half || ny > half) continue;

        const double x = nx * p_.latticeResolution;
        const double y = ny * p_.latticeResolution;
        const double distance = std::hypot(x, y);
        if (distance > p_.searchRadius + kEpsilon) continue;

        const int nextIndex = indexOf(nx, ny);
        Node& next = nodes[static_cast<std::size_t>(nextIndex)];
        if (next.visited) continue;

        int firstBin = current.firstBin;
        if (current.parent < 0) {
          firstBin = directionBin(d[0], d[1]);
          const std::uint32_t bit = std::uint32_t{1} << firstBin;
          if ((in.rejectedTranslationDirectionMask & bit) != 0U) continue;
        }

        const double fromX = current.ix * p_.latticeResolution;
        const double fromY = current.iy * p_.latticeResolution;
        if (!safeEdge(in, fromX, fromY, x, y, &why)) {
          out->diagnostics.lastFailure = why;
          continue;
        }

        next = {nx, ny, currentIndex, firstBin, true};
        queue.push_back(nextIndex);
        out->diagnostics.maxReachDistance =
            std::max(out->diagnostics.maxReachDistance, distance);
        if (distance + kEpsilon < p_.minTranslationDistance) continue;

        ++out->diagnostics.candidateCount;
        const bool portal =
            distance + std::max(p_.portalMargin,
                                p_.latticeResolution * 1.5) >=
            p_.searchRadius;
        const double heading = std::atan2(y, x);
        const double alignment =
            std::cos(nav_kernel::normalizeAngle(heading - in.goalDirectionBodyRad));
        const double clearance =
            std::min(p_.searchRadius, obstacleClearance(in, x, y));
        const double score =
            distance + p_.clearanceWeight * clearance +
            p_.goalDirectionWeight * alignment;

        const bool betterClass = portal && !bestPortal;
        const bool betterWithinClass =
            portal == bestPortal &&
            (score > bestScore + kScoreEpsilon ||
             (std::fabs(score - bestScore) <= kScoreEpsilon &&
              (bestIndex < 0 ||
               firstBin <
                   nodes[static_cast<std::size_t>(bestIndex)].firstBin)));
        if (bestIndex < 0 || betterClass || betterWithinClass) {
          bestIndex = nextIndex;
          bestPortal = portal;
          bestScore = score;
        }
      }
    }

    if (bestIndex < 0) return false;

    std::vector<nav_kernel::Vec3> reverse;
    for (int index = bestIndex; index >= 0;
         index = nodes[static_cast<std::size_t>(index)].parent) {
      const Node& node = nodes[static_cast<std::size_t>(index)];
      reverse.push_back({
          node.ix * p_.latticeResolution,
          node.iy * p_.latticeResolution,
          0.0,
      });
    }
    out->pathBody.assign(reverse.rbegin(), reverse.rend());

    RecoverySafetyFailure finalFailure = RecoverySafetyFailure::None;
    if (!validateBodyPath(in, out->pathBody, &finalFailure)) {
      out->pathBody.clear();
      out->diagnostics.lastFailure = finalFailure;
      return false;
    }

    out->pathWorld = bodyPathToWorld(out->pathBody, in.vehiclePose);
    out->status = RecoverySearchStatus::TranslationReady;
    out->action = nav_kernel::RecoveryAction::Translate;
    out->safetyFailure = RecoverySafetyFailure::None;
    out->verified = true;
    out->diagnostics.selectedDirectionBin =
        nodes[static_cast<std::size_t>(bestIndex)].firstBin;
    out->diagnostics.selectedScore = bestScore;
    out->diagnostics.portalSelected = bestPortal;
    out->diagnostics.lastFailure = RecoverySafetyFailure::None;
    return true;
  }
};

}  // namespace

bool RecoverySearchResult::found() const {
  return verified && action != nav_kernel::RecoveryAction::None &&
         (status == RecoverySearchStatus::TranslationReady ||
          status == RecoverySearchStatus::RotationReady);
}

RecoverySearch::RecoverySearch(const RecoverySearchParams& params)
    : params_(params) {}

const RecoverySearchParams& RecoverySearch::params() const {
  return params_;
}

RecoverySearchResult RecoverySearch::plan(
    const RecoverySearchInput& input) const {
  return RecoverySearchImplementation(params_).plan(input);
}

bool RecoverySearch::validateBodyPath(
    const RecoverySearchInput& input,
    const std::vector<nav_kernel::Vec3>& bodyPath,
    RecoverySafetyFailure* failure) const {
  return RecoverySearchImplementation(params_).validateBodyPath(
      input, bodyPath, failure);
}

bool RecoverySearch::validateRotation(
    const RecoverySearchInput& input,
    double rotationDeltaRad,
    RecoverySafetyFailure* failure) const {
  return RecoverySearchImplementation(params_).validateRotation(
      input, rotationDeltaRad, failure);
}

std::vector<nav_kernel::Vec3> RecoverySearch::bodyPathToWorld(
    const std::vector<nav_kernel::Vec3>& bodyPath,
    const nav_kernel::Pose& vehiclePose) {
  return RecoverySearchImplementation::bodyPathToWorld(bodyPath, vehiclePose);
}

std::vector<nav_kernel::Vec3> RecoverySearch::worldPathToBody(
    const std::vector<nav_kernel::Vec3>& worldPath,
    const nav_kernel::Pose& vehiclePose) {
  return RecoverySearchImplementation::worldPathToBody(worldPath, vehiclePose);
}

}  // namespace lingtu::nav::navigation::detail
