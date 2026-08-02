#pragma once

#include "nav_kernel/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace nav_kernel {

// Candidate search is stateless. The caller owns timers, progress monitoring,
// retry budgets, and the rejected-direction masks.
enum class RecoveryAction { None, Translate, Rotate };

enum class PlanStatus {
  InvalidInput,
  NoSafeCandidate,
  TranslationReady,
  RotationReady,
};

enum class SafetyFailure {
  None,
  InvalidInput,
  ObstacleCollision,
  TraversabilityUnavailable,
  TraversabilityOutOfBounds,
  TraversabilityNonFinite,
  TraversabilityInvalidValue,
  TraversabilityBlocked,
};

struct RecoveryPlannerParams {
  double vehicleLength{0.6};
  double vehicleWidth{0.6};
  double footprintPadding{0.1};
  double obstacleHeightThreshold{0.2};
  bool useTerrainAnalysis{true};
  bool checkObstacles{true};
  bool requireTraversability{true};
  double traversabilityHardCost{90.0};
  double searchRadius{1.2};
  double latticeResolution{0.1};
  double edgeSampleResolution{0.05};
  double minTranslationDistance{0.35};
  double portalMargin{0.15};
  double rotationStepRad{0.35};
  double rotationSampleStepRad{0.05};
  double rotationRate{0.25};
  double clearanceWeight{0.10};
  double goalDirectionWeight{0.03};
  int maxSearchNodes{20000};
  int maxObstaclePoints{1000000};
};

struct RecoveryPlannerInput {
  Pose vehiclePose{};
  const float* obstacleX{nullptr};
  const float* obstacleY{nullptr};
  const float* obstacleHeight{nullptr};
  int obstacleCount{0};

  // Row-major 0..100 traversability grid in world coordinates.
  const float* traversabilityGrid{nullptr};
  int traversabilityRows{0};
  int traversabilityCols{0};
  double traversabilityResolution{0.0};
  double traversabilityOriginX{0.0};
  double traversabilityOriginY{0.0};

  // Goal direction is only a weak tie-break.
  double goalDirectionBodyRad{0.0};

  // Translation: 16 direction bins. Rotation: bit 0 positive, bit 1 negative.
  std::uint32_t rejectedTranslationDirectionMask{0};
  int rejectedRotationDirectionMask{0};
};

struct RecoveryPlannerDiagnostics {
  int expandedNodeCount{0};
  int candidateCount{0};
  int rotationCandidateCount{0};
  int selectedDirectionBin{-1};
  double selectedScore{-std::numeric_limits<double>::infinity()};
  double maxReachDistance{0.0};
  bool portalSelected{false};
  SafetyFailure lastFailure{SafetyFailure::None};
};

struct RecoveryPlanResult {
  PlanStatus status{PlanStatus::NoSafeCandidate};
  RecoveryAction action{RecoveryAction::None};
  std::vector<Vec3> pathBody;
  std::vector<Vec3> pathWorld;
  Twist directCommand{};
  double rotationDeltaRad{0.0};
  SafetyFailure safetyFailure{SafetyFailure::None};
  RecoveryPlannerDiagnostics diagnostics{};
  bool verified{false};

  bool found() const {
    return verified && action != RecoveryAction::None &&
           (status == PlanStatus::TranslationReady ||
            status == PlanStatus::RotationReady);
  }
};

class RecoveryPlanner {
 public:
  explicit RecoveryPlanner(
      const RecoveryPlannerParams& params = RecoveryPlannerParams())
      : p_(params) {}

  const RecoveryPlannerParams& params() const { return p_; }

  RecoveryPlanResult plan(const RecoveryPlannerInput& input) const {
    RecoveryPlanResult out;
    SafetyFailure invalid = SafetyFailure::None;
    if (!validateInput(input, &invalid)) {
      out.status = PlanStatus::InvalidInput;
      out.safetyFailure = invalid;
      out.diagnostics.lastFailure = invalid;
      return out;
    }
    if (searchTranslation(input, &out)) return out;
    if (out.status == PlanStatus::InvalidInput) return out;

    const bool preferPositive = input.goalDirectionBodyRad >= 0.0;
    const std::array<double, 2> deltas =
        preferPositive
            ? std::array<double, 2>{{p_.rotationStepRad, -p_.rotationStepRad}}
            : std::array<double, 2>{{-p_.rotationStepRad, p_.rotationStepRad}};
    SafetyFailure rotationFailure = SafetyFailure::None;
    for (double delta : deltas) {
      const int bit = delta > 0.0 ? 0x1 : 0x2;
      if ((input.rejectedRotationDirectionMask & bit) != 0) continue;
      ++out.diagnostics.rotationCandidateCount;
      if (!validateRotation(input, delta, &rotationFailure)) continue;
      out.status = PlanStatus::RotationReady;
      out.action = RecoveryAction::Rotate;
      out.rotationDeltaRad = delta;
      out.directCommand.wz = std::copysign(p_.rotationRate, delta);
      out.safetyFailure = SafetyFailure::None;
      out.diagnostics.lastFailure = SafetyFailure::None;
      out.verified = true;
      return out;
    }

    out.status = PlanStatus::NoSafeCandidate;
    out.safetyFailure =
        rotationFailure != SafetyFailure::None
            ? rotationFailure
            : out.diagnostics.lastFailure;
    out.diagnostics.lastFailure = out.safetyFailure;
    return out;
  }

  bool validateBodyPath(
      const RecoveryPlannerInput& input,
      const std::vector<Vec3>& bodyPath,
      SafetyFailure* failure = nullptr) const {
    setFailure(failure, SafetyFailure::None);
    SafetyFailure invalid = SafetyFailure::None;
    if (!validateInput(input, &invalid) || bodyPath.empty()) {
      setFailure(failure, invalid == SafetyFailure::None
                              ? SafetyFailure::InvalidInput
                              : invalid);
      return false;
    }
    for (const Vec3& p : bodyPath) {
      if (!finite(p.x) || !finite(p.y) || !finite(p.z)) {
        setFailure(failure, SafetyFailure::InvalidInput);
        return false;
      }
    }
    SafetyFailure why = SafetyFailure::None;
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
      const RecoveryPlannerInput& input,
      double rotationDeltaRad,
      SafetyFailure* failure = nullptr) const {
    setFailure(failure, SafetyFailure::None);
    SafetyFailure invalid = SafetyFailure::None;
    if (!validateInput(input, &invalid) ||
        !finite(rotationDeltaRad) ||
        std::fabs(rotationDeltaRad) <= kEpsilon) {
      setFailure(failure, invalid == SafetyFailure::None
                              ? SafetyFailure::InvalidInput
                              : invalid);
      return false;
    }

    const int samples = std::max(
        1, static_cast<int>(std::ceil(
               std::fabs(rotationDeltaRad) / p_.rotationSampleStepRad)));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      SafetyFailure why = SafetyFailure::None;
      if (!safePose(input, 0.0, 0.0, rotationDeltaRad * t, &why)) {
        setFailure(failure, why);
        return false;
      }
    }
    return true;
  }

  static std::vector<Vec3> bodyPathToWorld(
      const std::vector<Vec3>& bodyPath,
      const Pose& vehiclePose) {
    std::vector<Vec3> world;
    world.reserve(bodyPath.size());
    const double c = std::cos(vehiclePose.yaw);
    const double s = std::sin(vehiclePose.yaw);
    for (const Vec3& p : bodyPath) {
      world.push_back({
          vehiclePose.position.x + c * p.x - s * p.y,
          vehiclePose.position.y + s * p.x + c * p.y,
          vehiclePose.position.z + p.z,
      });
    }
    return world;
  }

  static std::vector<Vec3> worldPathToBody(
      const std::vector<Vec3>& worldPath,
      const Pose& vehiclePose) {
    std::vector<Vec3> body;
    body.reserve(worldPath.size());
    const double c = std::cos(vehiclePose.yaw);
    const double s = std::sin(vehiclePose.yaw);
    for (const Vec3& p : worldPath) {
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

  RecoveryPlannerParams p_;

  static bool finite(double value) { return std::isfinite(value); }

  static void setFailure(SafetyFailure* out, SafetyFailure value) {
    if (out != nullptr) *out = value;
  }

  bool validateInput(
      const RecoveryPlannerInput& in,
      SafetyFailure* failure) const {
    setFailure(failure, SafetyFailure::None);
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
        finite(p_.rotationStepRad) && p_.rotationStepRad > 0.0 &&
        p_.rotationStepRad <= M_PI &&
        finite(p_.rotationSampleStepRad) &&
        p_.rotationSampleStepRad > 0.0 &&
        p_.rotationSampleStepRad <= p_.rotationStepRad &&
        finite(p_.rotationRate) && p_.rotationRate > 0.0 &&
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
      setFailure(failure, SafetyFailure::InvalidInput);
      return false;
    }

    if (in.obstacleCount > 0 &&
        (in.obstacleX == nullptr || in.obstacleY == nullptr ||
         in.obstacleHeight == nullptr)) {
      setFailure(failure, SafetyFailure::InvalidInput);
      return false;
    }
    for (int i = 0; i < in.obstacleCount; ++i) {
      if (!std::isfinite(in.obstacleX[i]) ||
          !std::isfinite(in.obstacleY[i]) ||
          !std::isfinite(in.obstacleHeight[i])) {
        setFailure(failure, SafetyFailure::InvalidInput);
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
        setFailure(failure, SafetyFailure::TraversabilityUnavailable);
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
      setFailure(failure, SafetyFailure::InvalidInput);
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
           static_cast<double>(height) > p_.obstacleHeightThreshold;
  }

  bool safePose(
      const RecoveryPlannerInput& in,
      double bodyX,
      double bodyY,
      double relativeYaw,
      SafetyFailure* failure) const {
    setFailure(failure, SafetyFailure::None);
    if (!finite(bodyX) || !finite(bodyY) || !finite(relativeYaw)) {
      setFailure(failure, SafetyFailure::InvalidInput);
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
    if (p_.checkObstacles) {
      for (int i = 0; i < in.obstacleCount; ++i) {
        if (!admitted(in.obstacleHeight[i])) continue;
        const double dx = static_cast<double>(in.obstacleX[i]) - bodyX;
        const double dy = static_cast<double>(in.obstacleY[i]) - bodyY;
        const double localX = c * dx + s * dy;
        const double localY = -s * dx + c * dy;
        if (std::fabs(localX) <= halfLength + kEpsilon &&
            std::fabs(localY) <= halfWidth + kEpsilon) {
          setFailure(failure, SafetyFailure::ObstacleCollision);
          return false;
        }
      }
    }

    if (in.traversabilityGrid == nullptr) {
      if (p_.requireTraversability) {
        setFailure(failure, SafetyFailure::TraversabilityUnavailable);
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
        normalizeAngle(in.vehiclePose.yaw + relativeYaw),
        halfLength, halfWidth, failure);
  }

  bool safeEdge(
      const RecoveryPlannerInput& in,
      double fromX,
      double fromY,
      double toX,
      double toY,
      SafetyFailure* failure) const {
    const double length = std::hypot(toX - fromX, toY - fromY);
    const int samples = std::max(
        1, static_cast<int>(
               std::ceil(length / p_.edgeSampleResolution)));
    for (int i = 1; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      SafetyFailure why = SafetyFailure::None;
      if (!safePose(in,
                    fromX + (toX - fromX) * t,
                    fromY + (toY - fromY) * t,
                    0.0, &why)) {
        setFailure(failure, why);
        return false;
      }
    }
    setFailure(failure, SafetyFailure::None);
    return true;
  }

  bool safeGridFootprint(
      const RecoveryPlannerInput& in,
      double centerX,
      double centerY,
      double yaw,
      double halfLength,
      double halfWidth,
      SafetyFailure* failure) const {
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
      setFailure(failure, SafetyFailure::TraversabilityOutOfBounds);
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
        const float raw =
            in.traversabilityGrid[row * in.traversabilityCols + col];
        if (!std::isfinite(raw)) {
          setFailure(failure, SafetyFailure::TraversabilityNonFinite);
          return false;
        }
        const double risk = static_cast<double>(raw);
        if (risk < 0.0 || risk > 100.0) {
          setFailure(failure, SafetyFailure::TraversabilityInvalidValue);
          return false;
        }
        if (risk >= p_.traversabilityHardCost) {
          setFailure(failure, SafetyFailure::TraversabilityBlocked);
          return false;
        }
      }
    }
    setFailure(failure, SafetyFailure::None);
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

  double obstacleClearance(
      const RecoveryPlannerInput& in,
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
  bool searchTranslation(
      const RecoveryPlannerInput& in,
      RecoveryPlanResult* out) const {
    const double halfValue =
        std::floor(p_.searchRadius / p_.latticeResolution);
    if (!finite(halfValue) || halfValue < 1.0 ||
        halfValue >
            static_cast<double>(std::numeric_limits<int>::max() / 2 - 1)) {
      out->status = PlanStatus::InvalidInput;
      out->safetyFailure = SafetyFailure::InvalidInput;
      out->diagnostics.lastFailure = SafetyFailure::InvalidInput;
      return false;
    }
    const int half = static_cast<int>(halfValue);
    const int side = half * 2 + 1;
    if (side > p_.maxSearchNodes ||
        side > p_.maxSearchNodes / side) {
      out->status = PlanStatus::InvalidInput;
      out->safetyFailure = SafetyFailure::InvalidInput;
      out->diagnostics.lastFailure = SafetyFailure::InvalidInput;
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

    SafetyFailure why = SafetyFailure::None;
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
            std::cos(normalizeAngle(heading - in.goalDirectionBodyRad));
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

    std::vector<Vec3> reverse;
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

    SafetyFailure finalFailure = SafetyFailure::None;
    if (!validateBodyPath(in, out->pathBody, &finalFailure)) {
      out->pathBody.clear();
      out->diagnostics.lastFailure = finalFailure;
      return false;
    }

    out->pathWorld = bodyPathToWorld(out->pathBody, in.vehiclePose);
    out->status = PlanStatus::TranslationReady;
    out->action = RecoveryAction::Translate;
    out->safetyFailure = SafetyFailure::None;
    out->verified = true;
    out->diagnostics.selectedDirectionBin =
        nodes[static_cast<std::size_t>(bestIndex)].firstBin;
    out->diagnostics.selectedScore = bestScore;
    out->diagnostics.portalSelected = bestPortal;
    out->diagnostics.lastFailure = SafetyFailure::None;
    return true;
  }
};

}  // namespace nav_kernel
