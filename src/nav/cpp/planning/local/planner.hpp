/**
 * Public interface for the ROS-free local path planner.
 *
 * The facade lives in planner.cpp; backend state and algorithms stay under
 * planning/local/<backend>/. Callers only depend on this contract.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav_kernel/types.hpp"

namespace nav_kernel {

enum class LocalPlannerBackend { Cmu, Scan };

const char *localPlannerBackendName(LocalPlannerBackend backend);

struct ScanPlannerParams {
  double voxelResolution = 0.15;
  double horizontalRange = 4.0;
  double verticalMargin = 0.60;
  double routeZTolerance = 0.35;
  double maxSlope = 0.80;
  double bodyClearanceBelow = 0.20;
  double bodyClearanceAbove = 0.35;
  double obstacleClearance = 0.0;
  double cylinderRadius = 0.25;
  double cylinderOffset = 0.18;
  double inflationZUp = 0.10;
  double inflationZDown = 0.10;
  double guideWeight = 1.5;
  double verticalWeight = 2.0;
  double controlPointSpacing = 0.20;
  double sampleSpacing = 0.08;
  double continuityHorizon = 0.50;
  double collisionMaxAge = 0.50;
  double maxVerticalSpeed = 0.25;
  double maxAcceleration = 1.0;
  double smoothWeight = 1.0;
  double collisionWeight = 1.0;
  double feasibilityWeight = 0.1;
  double feasibilityTolerance = 0.5;
  double collisionDistance = 0.20;
  int maxSearchNodes = 12000;
  int smoothingIterations = 200;
};

struct LocalPlannerParams {
  LocalPlannerBackend backend = LocalPlannerBackend::Cmu;
  ScanPlannerParams scan{};
  double vehicleLength = 0.6;
  double vehicleWidth = 0.6;
  bool twoWayDrive = true;
  double adjacentRange = 3.5;
  double obstacleHeightThre = 0.2;
  double obstacleHeightMax = 1.2;
  double groundHeightThre = 0.1;
  double costHeightThre1 = 0.15;
  double costHeightThre2 = 0.1;
  bool useCost = false;
  bool checkObstacle = true;
  bool checkRotObstacle = false;
  bool useTerrainAnalysis = true;
  int pointPerPathThre = 2;
  double minRelZ = -0.5;
  double maxRelZ = 0.25;
  double dirWeight = 0.02;
  double dirThre = 90.0;
  bool dirToVehicle = false;
  double pathScale = 1.0;
  double minPathScale = 0.75;
  double pathScaleStep = 0.25;
  bool pathScaleBySpeed = true;
  double minPathRange = 1.0;
  double pathRangeStep = 0.5;
  bool pathRangeBySpeed = true;
  bool pathCropByGoal = true;
  double maxSpeed = 1.0;
  double autonomySpeed = 1.0;
  double slopeWeight = 0.0;
  double goalClearRange = 0.5;
  double goalBehindRange = 0.8;
  double nearFieldStopDis = 0.5;
  double footprintPadding = 0.1;
  double freezeAng = 90.0;
  double freezeTime = 2.0;
  double omniDirGoalThre = 1.0;
  int slowPathNumThre = 5;
  int slowGroupNumThre = 1;
  bool useTraversabilityCost = true;
  bool traversabilityNearFieldStop = false;
  double traversabilityHardCost = 90.0;
  double traversabilitySoftCost = 40.0;
  double traversabilityWeight = 0.01;
  int scoringThreads = 2;
  int debugCandidateLimit = 0;
};

// Non-owning views used only for the duration of one synchronous plan call.
// All geometry in a LocalPlanInput must use the same planning frame.
struct LocalObstacleView {
  const float *xyzh{nullptr};
  int count{0};
};

struct LocalCollisionMapView {
  const float *occupiedXyz{nullptr};
  int occupiedCount{0};
  double resolution{0.0};
  Vec3 aabbMin{};
  Vec3 aabbMax{};
  std::uint64_t resetEpoch{0};
  std::uint64_t observationSequence{0};
  std::uint64_t generation{0};
  double stampS{0.0};
  double receiveStampS{0.0};
  bool complete{false};
  bool live{false};

  [[nodiscard]] bool present() const noexcept {
    return occupiedXyz != nullptr || occupiedCount != 0 || resolution > 0.0 || resetEpoch != 0 ||
           observationSequence != 0 || generation != 0;
  }
};

struct LocalTraversabilityView {
  const float *values{nullptr};
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double originX{0.0};
  double originY{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
  }
};

// Non-owning route segment in the planning frame. Navigator includes the current
// body position and every global-route bend through the selected local horizon.
struct LocalRouteView {
  const Vec3 *points{nullptr};
  int count{0};
  std::uint64_t generation{0};
  bool reachesGoal{false};

  [[nodiscard]] bool valid() const noexcept { return points != nullptr && count >= 2; }

  [[nodiscard]] Vec3 target() const noexcept { return valid() ? points[count - 1] : Vec3{}; }
};

// Velocity and acceleration are expressed in the planning frame.
struct LocalKinematicState {
  Vec3 linearVelocity{};
  Vec3 linearAcceleration{};
  double yawRate{0.0};
  double yawAcceleration{0.0};
  bool valid{false};
};

struct LocalPlanIdentity {
  std::uint64_t frameEpoch{0};
  std::uint64_t obstacleGeneration{0};
  std::uint64_t traversabilityGeneration{0};
};

// One complete local-planning frame. Route, robot state, obstacles, terrain,
// identities, and timestamp are submitted atomically.
struct LocalPlanInput {
  Pose vehicle{};
  LocalRouteView route{};
  LocalKinematicState kinematics{};
  LocalPlanIdentity identity{};
  LocalObstacleView obstacles{};
  LocalCollisionMapView collision{};
  LocalTraversabilityView traversability{};
  double timestampS{0.0};
};

struct LocalMotionIntent {
  double directionBodyDeg{0.0};
  double speedNormalized{0.0};
  double horizonM{0.0};
  double maxDirectionDeviationDeg{0.0};
};

enum class LocalPlanStatus {
  Ready,
  InvalidInput,
  NotConfigured,
  NoPath,
  Blocked,
};

const char *localPlanStatusName(LocalPlanStatus status);

struct LocalPlanResult {
  LocalPlannerBackend backend{LocalPlannerBackend::Cmu};
  LocalPlanStatus status{LocalPlanStatus::NoPath};
  std::string reason{"no_path"};
  // Body-frame geometric result. CMU executes this directly; SCAN also
  // provides it for telemetry and inspection.
  std::vector<Vec3> path;
  // Optional body-frame timed result. When present, Navigator executes this
  // instead of selecting behavior from the backend enum.
  std::vector<TrajectoryPoint> trajectory;
  int slowDown = 0;
  bool pathFound = false;
  bool nearFieldStop = false;
};

enum class LocalCandidateState {
  Feasible,
  CollisionBlocked,
  RotationBlocked,
  TerrainCost,
  TerrainBlocked,
  DirectionRejected,
};

const char *localCandidateStateName(LocalCandidateState state);

struct LocalPlanCandidate {
  int rotationIndex{-1};
  int groupId{-1};
  double rotationDeg{0.0};
  double aggregateScore{0.0};
  double terrainRisk{-1.0};
  int totalPathCount{0};
  int collisionFreePathCount{0};
  int directionAllowedPathCount{0};
  int terrainAllowedPathCount{0};
  int directionScoredPathCount{0};
  int heightCostAllowedPathCount{0};
  int terrainSoftPenalizedPathCount{0};
  int contributingPathCount{0};
  bool rotationAllowed{true};
  bool selected{false};
  LocalCandidateState state{LocalCandidateState::DirectionRejected};
  std::vector<Vec3> path;
};

struct LocalPlannerDebugSnapshot {
  bool valid{false};
  LocalPlannerBackend backend{LocalPlannerBackend::Cmu};
  double timestampS{0.0};
  double planningTimeMs{0.0};
  double reuseTimeMs{0.0};
  double gridTimeMs{0.0};
  double searchTimeMs{0.0};
  double splineTimeMs{0.0};
  std::string searchReason;
  int expandedNodes{0};
  int occupiedCellCount{0};
  int collisionPointCount{0};
  int legacyObstaclePointCount{0};
  int trajectoryPointCount{0};
  bool continuityReused{false};
  bool splineFallback{false};
  double pathScale{0.0};
  double pathRange{0.0};
  double relativeGoalDistanceM{0.0};
  double traversabilitySoftCost{0.0};
  double traversabilityHardCost{0.0};
  int validRotationCount{0};
  int selectedGroupId{-1};
  std::vector<LocalPlanCandidate> candidates;
};

inline constexpr int kPathNum = 343;
inline constexpr int kGroupNum = 7;
inline constexpr int kRotDirs = 36;

namespace local {

class Planner {
 public:
  explicit Planner(const LocalPlannerParams &params = LocalPlannerParams());
  ~Planner();

  Planner(Planner &&) noexcept;
  Planner &operator=(Planner &&) noexcept;
  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;

  // CMU loads its fixed path library; SCAN needs no external resource.
  bool configure(const std::string &pathLibraryDir = {});
  bool configured() const;
  LocalPlannerDebugSnapshot debugSnapshot() const;

  LocalPlanResult plan(const LocalPlanInput &input);
  void reset();
  LocalPlanResult planIntent(const LocalPlanInput &input, const LocalMotionIntent &intent);

  const LocalPlannerParams &params() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace local

}  // namespace nav_kernel
