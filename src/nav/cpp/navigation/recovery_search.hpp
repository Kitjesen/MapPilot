/**
 * Internal candidate search owned by navigation::Recovery.
 *
 * This header is intentionally under navigation (rather than local planning)
 * and is included only by Recovery and its focused tests.  It is not a local
 * planner backend or a second runtime owner.
 */
#pragma once

#include <lingtu/settings.hpp>

#include "nav_kernel/types.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace nav_kernel::local::scan {
class Grid;
}

namespace lingtu::nav::navigation::detail {

enum class RecoverySearchStatus {
  InvalidInput,
  NoSafeCandidate,
  TranslationReady,
  RotationReady,
};

enum class RecoverySafetyFailure {
  None,
  InvalidInput,
  ObstacleCollision,
  TraversabilityUnavailable,
  TraversabilityOutOfBounds,
  TraversabilityNonFinite,
  TraversabilityInvalidValue,
  TraversabilityBlocked,
};

struct RecoverySearchParams {
  double vehicleLength{0.6};
  double vehicleWidth{0.6};
  double footprintPadding{0.1};
  double obstacleHeightThreshold{
      ::lingtu::settings::local::native_nav::local_planner_obstacle_height_min_m};
  bool useTerrainAnalysis{true};
  // Controls point-cloud fallback; a supplied collision Grid remains authoritative.
  bool checkObstacles{true};
  bool requireTraversability{true};
  double traversabilityHardCost{90.0};
  double searchRadius{1.2};
  double latticeResolution{0.1};
  double edgeSampleResolution{0.05};
  double minTranslationDistance{0.35};
  double portalMargin{0.15};
  double minRotationRad{::lingtu::settings::recovery::native_nav::recovery::min_rotation_rad};
  double maxRotationRad{::lingtu::settings::recovery::native_nav::recovery::max_rotation_rad};
  double rotationCandidateStepRad{::lingtu::settings::recovery::native_nav::recovery::rotation_candidate_step_rad};
  double rotationSampleStepRad{::lingtu::settings::recovery::native_nav::recovery::rotation_sample_step_rad};
  double clearanceWeight{0.10};
  double goalDirectionWeight{0.03};
  int maxSearchNodes{20000};
  int maxObstaclePoints{1000000};
};

struct RecoverySearchInput {
  nav_kernel::Pose vehiclePose{};
  // Query the same configured collision geometry as normal SCAN planning.
  const nav_kernel::local::scan::Grid* collisionGrid{nullptr};
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

struct RecoverySearchDiagnostics {
  int expandedNodeCount{0};
  int candidateCount{0};
  int rotationCandidateCount{0};
  int selectedDirectionBin{-1};
  double selectedScore{-std::numeric_limits<double>::infinity()};
  double maxReachDistance{0.0};
  bool portalSelected{false};
  double selectedRotationRad{0.0};
  double selectedForwardReachM{0.0};
  RecoverySafetyFailure lastFailure{RecoverySafetyFailure::None};
};

struct RecoverySearchResult {
  RecoverySearchStatus status{RecoverySearchStatus::NoSafeCandidate};
  nav_kernel::RecoveryAction action{nav_kernel::RecoveryAction::None};
  std::vector<nav_kernel::Vec3> pathBody;
  std::vector<nav_kernel::Vec3> pathWorld;
  double rotationDeltaRad{0.0};
  RecoverySafetyFailure safetyFailure{RecoverySafetyFailure::None};
  RecoverySearchDiagnostics diagnostics{};
  bool verified{false};

  bool found() const;
};

class RecoverySearch {
 public:
  explicit RecoverySearch(
      const RecoverySearchParams& params = RecoverySearchParams());

  const RecoverySearchParams& params() const;
  RecoverySearchResult plan(const RecoverySearchInput& input) const;
  bool validateBodyPath(const RecoverySearchInput& input,
                        const std::vector<nav_kernel::Vec3>& bodyPath,
                        RecoverySafetyFailure* failure = nullptr) const;
  bool validateRotation(const RecoverySearchInput& input,
                        double rotationDeltaRad,
                        RecoverySafetyFailure* failure = nullptr) const;

  static std::vector<nav_kernel::Vec3> bodyPathToWorld(
      const std::vector<nav_kernel::Vec3>& bodyPath,
      const nav_kernel::Pose& vehiclePose);
  static std::vector<nav_kernel::Vec3> worldPathToBody(
      const std::vector<nav_kernel::Vec3>& worldPath,
      const nav_kernel::Pose& vehiclePose);

 private:
  RecoverySearchParams params_;
};

}  // namespace lingtu::nav::navigation::detail
