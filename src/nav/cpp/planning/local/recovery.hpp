/** Stateless recovery-candidate search used by navigation/recovery.cpp. */
#pragma once

#include "nav_kernel/types.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace nav_kernel {

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
  double minRotationRad{0.20};
  double maxRotationRad{1.20};
  double rotationCandidateStepRad{0.20};
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
  const float* traversabilityGrid{nullptr};
  int traversabilityRows{0};
  int traversabilityCols{0};
  double traversabilityResolution{0.0};
  double traversabilityOriginX{0.0};
  double traversabilityOriginY{0.0};
  double goalDirectionBodyRad{0.0};
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
  double selectedRotationRad{0.0};
  double selectedForwardReachM{0.0};
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

  bool found() const;
};

class RecoveryPlanner {
 public:
  explicit RecoveryPlanner(
      const RecoveryPlannerParams& params = RecoveryPlannerParams());

  const RecoveryPlannerParams& params() const;
  RecoveryPlanResult plan(const RecoveryPlannerInput& input) const;
  bool validateBodyPath(const RecoveryPlannerInput& input,
                        const std::vector<Vec3>& bodyPath,
                        SafetyFailure* failure = nullptr) const;
  bool validateRotation(const RecoveryPlannerInput& input,
                        double rotationDeltaRad,
                        SafetyFailure* failure = nullptr) const;

  static std::vector<Vec3> bodyPathToWorld(
      const std::vector<Vec3>& bodyPath, const Pose& vehiclePose);
  static std::vector<Vec3> worldPathToBody(
      const std::vector<Vec3>& worldPath, const Pose& vehiclePose);

 private:
  RecoveryPlannerParams params_;
};

}  // namespace nav_kernel
