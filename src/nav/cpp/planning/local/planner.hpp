/**
 * Public interface for the ROS-free local path planner.
 *
 * The facade lives in planner.cpp; backend state and algorithms stay under
 * planning/local/<backend>/. Callers only depend on this contract.
 */
#pragma once

#include <lingtu/settings.hpp>

#include <cstddef>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "nav_kernel/types.hpp"
#include "collision/terrain.hpp"

namespace lingtu::nav::collision { struct Body; class Terrain; }

namespace nav_kernel {

enum class LocalPlannerBackend { Cmu, Scan };

using LocalPlanCancel = std::function<bool()>;

const char *localPlannerBackendName(LocalPlannerBackend backend);

struct ScanPlannerParams {
  static constexpr double kMinReferenceWaypointDistanceM = 0.5;
  double voxelResolution{::lingtu::settings::maps::maps::occupancy::resolution_m};
  double bodyClearanceBelow = 0.25;
  double bodyClearanceAbove = 0.35;
  double cylinderOffset = 0.18;
  double cylinderRadius = 0.40;
  double controlPointSpacing{::lingtu::settings::local::parameters::scan_planner::control_point_spacing_m};
  double replanDistance{::lingtu::settings::local::parameters::scan_planner::replan_distance_m};
  double noReplanDistance{::lingtu::settings::local::parameters::scan_planner::no_replan_distance_m};
  double maxVelocity{::lingtu::settings::limits::native_nav::follower::max_speed_mps};
  double maxAcceleration{::lingtu::settings::limits::native_nav::follower::max_accel_mps2};
  double planningHorizon{::lingtu::settings::local::parameters::scan_planner::planning_horizon_m};
  double smoothWeight{::lingtu::settings::local::parameters::scan_planner::smooth_weight};
  double collisionWeight{::lingtu::settings::local::parameters::scan_planner::collision_weight};
  double feasibilityWeight{::lingtu::settings::local::parameters::scan_planner::feasibility_weight};
  double fitnessWeight{::lingtu::settings::local::parameters::scan_planner::fitness_weight};
  double feasibilityTolerance{::lingtu::settings::local::parameters::scan_planner::feasibility_tolerance};
  double velocityTolerance{::lingtu::settings::local::parameters::scan_planner::velocity_tolerance};
  double accelerationTolerance{::lingtu::settings::local::parameters::scan_planner::acceleration_tolerance};
  double collisionDistance{::lingtu::settings::local::parameters::scan_planner::collision_distance_m};
  // Existing executor body-axis capabilities; zero leaves legacy callers
  // governed by the vector speed limit alone.
  double maxBodyVx = 0.0;
  double maxBodyVy = 0.0;
};

struct LocalPlannerParams {
  LocalPlannerBackend backend = LocalPlannerBackend::Cmu;
  ScanPlannerParams scan{};
  // Endpoint input policy shared with recovery collision queries.
  double localCollisionMaxAge{::lingtu::settings::local::parameters::local_collision_max_age_s};
  double vehicleLength = 0.6;
  double vehicleWidth = 0.6;
  bool twoWayDrive = true;
  double adjacentRange{::lingtu::settings::follower::native_nav::corridor_lookahead_m};
  double obstacleHeightThre{::lingtu::settings::local::native_nav::local_planner_obstacle_height_min_m};
  double obstacleHeightMax{::lingtu::settings::local::native_nav::local_planner_obstacle_height_max_m};
  double groundHeightThre{::lingtu::settings::local::native_nav::cmu::ground_height_m};
  double costHeightThre1{::lingtu::settings::local::native_nav::cmu::cost_height_high_m};
  double costHeightThre2{::lingtu::settings::local::native_nav::cmu::cost_height_low_m};
  bool useCost = false;
  bool checkObstacle = true;
  bool checkRotObstacle = false;
  bool useTerrainAnalysis = false;
  int pointPerPathThre{::lingtu::settings::local::native_nav::cmu::points_per_path};
  double dirWeight{::lingtu::settings::local::native_nav::cmu::direction_weight};
  double dirThre{::lingtu::settings::local::native_nav::cmu::direction_threshold_deg};
  bool dirToVehicle{::lingtu::settings::local::native_nav::cmu::direction_to_vehicle};
  double pathScale{::lingtu::settings::local::native_nav::cmu::path_scale};
  double minPathScale{::lingtu::settings::local::native_nav::cmu::min_path_scale};
  double pathScaleStep{::lingtu::settings::local::native_nav::cmu::path_scale_step};
  bool pathScaleBySpeed{::lingtu::settings::local::native_nav::cmu::scale_by_speed};
  double minPathRange{::lingtu::settings::local::native_nav::cmu::min_path_range_m};
  double pathRangeStep{::lingtu::settings::local::native_nav::cmu::path_range_step_m};
  bool pathRangeBySpeed{::lingtu::settings::local::native_nav::cmu::range_by_speed};
  bool pathCropByGoal{::lingtu::settings::local::native_nav::cmu::crop_by_goal};
  double maxSpeed = kDefaultLinearSpeedMps;
  double autonomySpeed = kDefaultLinearSpeedMps;
  double slopeWeight = 0.0;
  double goalClearRange{::lingtu::settings::local::native_nav::cmu::goal_clear_range_m};
  double nearFieldStopDis = 0.5;
  double footprintPadding = 0.1;
  double selfFilterPadding = 0.03;
  double freezeAng = 90.0;
  double freezeTime = 2.0;
  double omniDirGoalThre = 1.0;
  int slowPathNumThre{::lingtu::settings::local::native_nav::cmu::slow_path_count};
  int slowGroupNumThre{::lingtu::settings::local::native_nav::cmu::slow_group_count};
  bool useTraversabilityCost = false;
  bool traversabilityNearFieldStop = false;
  double traversabilityHardCost{::lingtu::settings::local::native_nav::traversability_hard_cost};
  double traversabilitySoftCost{::lingtu::settings::local::native_nav::traversability_soft_cost};
  double traversabilityWeight{::lingtu::settings::local::native_nav::traversability_weight};
  int scoringThreads{::lingtu::settings::local::native_nav::local_planner_threads};
  int debugCandidateLimit{::lingtu::settings::local::native_nav::local_planner_debug_candidates};
};

// Non-owning views used only for the duration of one synchronous plan call.
// All geometry in a LocalPlanRequest must use the same planning frame.
struct LocalObstacleView {
  const float *xyzh{nullptr};
  int count{0};
};

// Conservative swept volume of one confirmed moving cluster over a short
// prediction horizon. Geometry is in the request's planning frame, not a map
// mutation or a promise of time-optimal dynamic obstacle avoidance.
struct PredictedObstacle {
  Vec3 start{};
  Vec3 end{};
  double radius{0.0};
  double minZ{0.0};
  double maxZ{0.0};
};

struct PredictionView {
  const PredictedObstacle *obstacles{nullptr};
  std::size_t count{0};
  double expiresAtS{0.0};
  double observedAtS{0.0};
  double horizonS{1.0};

  [[nodiscard]] bool fresh(double nowS) const noexcept {
    return (count == 0 || obstacles != nullptr) && std::isfinite(nowS) &&
           std::isfinite(expiresAtS) && expiresAtS > 0.0 && nowS < expiresAtS;
  }
};

struct LocalCollisionMapView {
  const std::uint8_t *inflatedBits{nullptr};
  std::size_t inflatedBytes{0U};
  int sizeX{0};
  int sizeY{0};
  int sizeZ{0};
  double resolution{0.0};
  Vec3 aabbMin{};
  Vec3 aabbMax{};
  std::uint64_t resetEpoch{0};
  std::uint64_t observationSequence{0};
  std::uint64_t generation{0};
  double stampS{0.0};
  // Receive time in the consuming request's clock; endpoint rebases for sim.
  double receiveStampS{0.0};
  bool complete{false};
  bool live{false};
  // Rigid transform grid <- planning. Mapd remains in map while the local
  // planner normally runs in odom, so the packed grid never needs resampling.
  Vec3 gridFromPlanningTranslation{};
  double gridFromPlanningYaw{0.0};
  // Optional lifetime owner supplied by the endpoint. In-process consumers
  // share the decoded bitmap instead of copying it for every map generation.
  std::shared_ptr<const std::vector<std::uint8_t>> inflatedStorage{};
  // Endpoint-owned views cache this once while decoding the DDS bitmap.
  std::size_t occupiedCells{0U};
  bool occupiedCellsKnown{false};
  // Immutable measured 3D evidence from the same DDS snapshot as inflation.
  std::shared_ptr<const std::vector<std::uint8_t>> measuredOccupiedStorage{};
  std::shared_ptr<const std::vector<std::uint8_t>> knownFreeStorage{};
  // Original hits in the collision grid frame from the same map observations.
  std::shared_ptr<const lingtu::nav::collision::Terrain> terrain{};

  [[nodiscard]] bool present() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::size_t cellCount() const noexcept;
  [[nodiscard]] std::size_t occupiedCount() const noexcept;
  [[nodiscard]] bool covers(const Vec3 &planningPoint, double tolerance = 0.0) const noexcept;
  [[nodiscard]] bool coversCylinder(const Vec3 &planningCenter, double radius,
                                    double below, double above) const noexcept;
  // Bounds use the stored grid frame; touching a cell face adds no volume.
  [[nodiscard]] bool observedFreeBox(const Vec3 &minimum, const Vec3 &maximum) const noexcept;
  // Partial observations can confirm the current stance without clearing unknown cells.
  [[nodiscard]] bool observedClearBox(const Vec3 &minimum, const Vec3 &maximum) const noexcept;
  [[nodiscard]] bool occupied(const Vec3 &planningPoint) const noexcept;
  [[nodiscard]] bool occupiedLinear(std::size_t linear) const noexcept;
  [[nodiscard]] Vec3 planningCellCenter(std::size_t linear) const noexcept;

 private:
  [[nodiscard]] bool observedBox(const Vec3 &minimum, const Vec3 &maximum,
                                 bool requireAllFree) const noexcept;
};

struct LocalTraversabilityView {
  // Optional 2D risk input for the CMU backend. SCAN consumes the 3D
  // LocalCollisionMapView instead and must never receive this view.
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

// Non-owning route geometry in the planning frame. Generation is the reference
// event identity and must change whenever the geometry is intentionally
// replaced. Robot-pose updates alone do not create a new reference event.
struct LocalRouteView {
  const Vec3 *points{nullptr};
  int count{0};
  std::uint64_t generation{0};

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
  // Used only by a backend that consumes LocalTraversabilityView (currently
  // CMU). SCAN keeps this at zero and keys replanning from its 3D collision
  // generation instead.
  std::uint64_t traversabilityGeneration{0};
};

struct LocalMotionIntent {
  double directionBodyDeg{0.0};
  double speedNormalized{0.0};
  double horizonM{0.0};
  double maxDirectionDeviationDeg{0.0};
};

enum class LocalPlanStatus {
  Ready,
  Pending,
  InvalidInput,
  NotConfigured,
  NoPath,
  Blocked,
  NearFieldStop,
  Cancelled,
  Expired,
};

// Stable diagnostics vocabulary for the local planner boundary. The C++ enum
// names stay compact for the native API; callers should use this function for
// serialized status text.
const char *localPlanStatusName(LocalPlanStatus status);

struct RobotState {
  Pose pose{};
  LocalKinematicState kinematics{};
};

struct RouteTarget {
  LocalRouteView route{};
};

struct MotionIntentTarget {
  LocalMotionIntent intent{};
  LocalRouteView guide{};
};

using LocalObjective = std::variant<RouteTarget, MotionIntentTarget>;

struct EnvironmentView {
  LocalObstacleView obstacles{};
  LocalCollisionMapView collision{};
  LocalTraversabilityView traversability{};
  PredictionView predictions{};
};

using PlanIdentity = LocalPlanIdentity;

enum class PlanClockMode { Steady, External };

struct PlanClock {
  double timestampS{0.0};
  bool executionFrozen{false};
  PlanClockMode mode{PlanClockMode::Steady};

  [[nodiscard]] double afterElapsed(double wallElapsedS) const {
    return timestampS + (mode == PlanClockMode::Steady ? wallElapsedS : 0.0);
  }
};

struct LocalPlanRequest {
  RobotState robot{};
  LocalObjective objective{RouteTarget{}};
  // Optional complete reference route for planners that own their local
  // target progression (SCAN REFERENCE_PATH). Short-horizon planners consume
  // objective instead. Motion intent uses its guide for both roles.
  LocalRouteView reference{};
  EnvironmentView environment{};
  PlanIdentity identity{};
  PlanClock clock{};
  // Resolved runtime linear-speed ceiling; zero uses configured planner limits.
  double maxLinearSpeedMps{0.0};
  // Optional body heading in the planning frame, independent of route tangent.
  // Explicit native pose stages keep this heading independent of route tangent.
  std::optional<double> bodyHeading{};
  // Platform geometry for native motion checks; SCAN candidates use map inflation.
  std::shared_ptr<const lingtu::nav::collision::Body> body{};
  // Measured body attitude applied to nominal joint geometry in the yaw-only
  // frame. This does not reconstruct geometry from live joint angles.
  std::shared_ptr<const lingtu::nav::collision::Body> observedBody{};
  // Global base-to-support height metadata for trajectory initialization.
  // Local collision queries do not require whole-body floor observation.
  std::optional<double> supportHeightM;
  std::optional<double> supportHeightToleranceM;

  [[nodiscard]] const LocalRouteView *route() const noexcept;
  [[nodiscard]] const LocalRouteView *referenceRoute() const noexcept;
  [[nodiscard]] const LocalMotionIntent *intent() const noexcept;
};

struct PathTarget {
  std::vector<Vec3> points;
};

using FollowTarget = std::variant<PathTarget, SplineTarget>;

struct ControlHints {
  int slowdownLevel{0};
  // Planner-approved reuse of the visible path as a short route guide.
  // Exact spline continuity remains entirely inside the SCAN task/backend.
  bool retainRouteGuide{false};
};

class LocalPlan {
 public:
  LocalPlan();

  static LocalPlan stopped(LocalPlanStatus status, ControlHints hints = {});
  static LocalPlan path(std::vector<Vec3> points, ControlHints hints = {});
  static LocalPlan path(std::vector<Vec3> points, LocalPlanStatus status,
                        ControlHints hints = {});
  static LocalPlan spline(SplineTarget target, ControlHints hints = {});

  [[nodiscard]] LocalPlanStatus status() const noexcept;
  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] const FollowTarget &target() const noexcept;
  // The preview is materialized once with the plan payload. Repeated control
  // ticks borrow the same samples instead of resampling and reallocating the
  // spline for visualization.
  [[nodiscard]] const std::vector<Vec3> &previewPath() const noexcept;
  [[nodiscard]] const ControlHints &hints() const noexcept;

 private:
  struct Payload;

  LocalPlanStatus status_{LocalPlanStatus::NoPath};
  std::shared_ptr<const Payload> payload_;
  ControlHints hints_{};
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

struct ScanAttemptDiagnostics {
  std::uint64_t attemptId{0};
  double timestampS{0.0};  // Attempt time; timer/status refreshes do not renew it.
  bool attempted{false};
  bool success{false};
  std::string stage;
  std::string reason;
  bool optimizerReturnCodeValid{false};
  int optimizerReturnCode{0};
  bool collisionValid{false};
  // Rejected segment start, or a control point when collisionTimeS is -1.
  // This is not the obstacle voxel or an exact contact point.
  Vec3 collisionPosition{};
  double collisionTimeS{-1.0};
  int collisionState{0};
  bool dynamicViolationValid{false};
  std::string dynamicQuantity;
  double dynamicValue{0.0};
  double dynamicLimit{0.0};
  double dynamicTimeS{-1.0};
};

// Backend-neutral evidence for one rejected local body pose. The points are
// measured occupied voxel centres from the exact collision snapshot used by
// that attempt; inflated-only cells and the rejected pose itself are not
// obstacle evidence.
struct LocalCollisionEvidence {
  double timestampS{0.0};
  PlanIdentity identity{};
  std::uint64_t collisionResetEpoch{0};
  std::uint64_t collisionObservationSequence{0};
  std::uint64_t collisionGeneration{0};
  Vec3 rejectedPosition{};
  double voxelResolution{0.0};
  std::shared_ptr<const std::vector<Vec3>> measuredPoints{};
};

// Final attempted result of the first failed FSM tick in the latest episode.
// A tick that recovers through an internal retry is not a failure episode.
// Large map bytes retain
// immutable ownership and are persisted separately from periodic status.
struct ScanFailureSnapshot {
  std::uint64_t sequence{0};
  ScanAttemptDiagnostics attempt;
  PlanClock clock;
  PlanIdentity identity;
  RobotState robot;
  ScanPlannerParams params;
  bool checkObstacle{true};
  double maxLinearSpeedMps{0.0};
  std::optional<double> bodyHeading{};
  std::shared_ptr<const lingtu::nav::collision::Body> body{};
  std::shared_ptr<const lingtu::nav::collision::Body> observedBody{};
  std::optional<double> supportHeightM;
  std::optional<double> supportHeightToleranceM;
  std::optional<LocalMotionIntent> intent;
  std::vector<Vec3> reference;
  std::uint64_t referenceGeneration{0};
  LocalCollisionMapView collision;
  std::vector<PredictedObstacle> predictions;
  double predictionsObservedAtS{0.0}, predictionsHorizonS{1.0};
  double predictionsExpiresAtS{0.0};
  Vec3 startPosition{}, startVelocity{}, startAcceleration{};
  Vec3 targetPosition{}, targetVelocity{};
  bool polyInit{false}, randomPolyInit{false};
  double candidateIntervalS{0.0};
  std::vector<Vec3> candidateControlPoints;
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
  std::size_t predictedObstacleCount{0};
  int trajectoryPointCount{0};
  int reboundRestarts{0};
  int optimizerEvaluations{0};
  int collisionSegments{0};
  int anchorSearches{0};
  bool continuityReused{false};
  bool splineFallback{false};
  bool localTargetValid{false};
  Vec3 localTarget{};
  ScanAttemptDiagnostics scanAttempt;
  std::optional<LocalCollisionEvidence> collisionEvidence;
  std::shared_ptr<const ScanFailureSnapshot> lastScanFailure;
  double pathScale{0.0};
  double pathRange{0.0};
  double relativeGoalDistanceM{0.0};
  double traversabilitySoftCost{0.0};
  double traversabilityHardCost{0.0};
  int validRotationCount{0};
  int selectedGroupId{-1};
  std::vector<LocalPlanCandidate> candidates;
};

// Capabilities exposed by the selected local planner backend.  Navigation
// execution depends on this contract instead of inspecting the backend enum.
// SCAN owns the complete reference and consumes the 3D collision/prediction
// inputs; CMU is the only backend that consumes the optional 2D risk grid.
struct LocalPlannerCapabilities {
  bool ownsReferencePath{false};
  bool consumesTraversability{false};
  bool consumesPredictions{false};
  bool supportsBoundaryDeparture{false};
  bool consumesCollisionMap{false};
};

// Keep backend-specific capability derivation in the local-planner boundary.
// Navigation and recovery consume this contract instead of branching on the
// selected backend enum themselves.
[[nodiscard]] LocalPlannerCapabilities plannerCapabilities(
    const LocalPlannerParams &params) noexcept;

inline constexpr int kPathNum = 343;
inline constexpr int kGroupNum = 7;
inline constexpr int kRotDirs = 36;

// Public local-planning module, independent of the selected algorithm.
class LocalPlanner {
 public:
  explicit LocalPlanner(const LocalPlannerParams &params = LocalPlannerParams());
  ~LocalPlanner();

  LocalPlanner(LocalPlanner &&) noexcept;
  LocalPlanner &operator=(LocalPlanner &&) noexcept;
  LocalPlanner(const LocalPlanner &) = delete;
  LocalPlanner &operator=(const LocalPlanner &) = delete;

  // CMU loads its fixed path library; SCAN needs no external resource.
  bool configure(const std::string &pathLibraryDir = {});
  bool configured() const;
  [[nodiscard]] LocalPlannerCapabilities capabilities() const noexcept;
  LocalPlannerDebugSnapshot debugSnapshot() const;

  LocalPlan plan(const LocalPlanRequest &request);
  LocalPlan plan(const LocalPlanRequest &request, const LocalPlanCancel &cancel);
  [[nodiscard]] bool turnClear(const LocalPlanRequest &request, double targetYaw) const;
  [[nodiscard]] bool sweepClear(const LocalPlanRequest &request,
                                const std::vector<Pose> &poses) const;
  [[nodiscard]] bool boundaryDepartureClear(const LocalPlanRequest &request,
                                            const Vec3 &target) const;
  void pause();
  void reset();

  const LocalPlannerParams &params() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel
