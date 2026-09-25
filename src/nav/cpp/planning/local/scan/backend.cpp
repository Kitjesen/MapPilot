#include "planning/local/scan/backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/plan_manage/planner_manager.h"
#include "planning/local/scan/upstream/plan_manage/scan_replan_fsm.h"

namespace nav_kernel::local::scan {
namespace {

using upstream::BsplineOptimizerParams;
using upstream::BsplineTrajectory;
using upstream::FsmInput;
using upstream::FsmOdometry;
using upstream::FsmOutput;
using upstream::GridMap;
using upstream::PlanParameters;
using upstream::SCANPlannerManager;
using upstream::SCANReplanFSM;
using upstream::ScanNavigationMode;
using upstream::ScanReplanParams;
using upstream::ScanReplanState;

constexpr double kOfficialBodyHeightM = 0.4;

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Eigen::Vector3d eigenPoint(const Vec3 &point) {
  return {point.x, point.y, point.z};
}

Vec3 planningPoint(const Eigen::Vector3d &point) {
  return {point.x(), point.y(), point.z()};
}

ScanAttemptDiagnostics attemptDiagnostics(const upstream::ReboundPlanDebug &attempt) {
  ScanAttemptDiagnostics result;
  result.attemptId = attempt.attemptId;
  result.attempted = attempt.attempted;
  result.success = attempt.success;
  result.stage = attempt.stage;
  result.reason = attempt.reason;
  result.optimizerReturnCodeValid = attempt.optimizerReturnCodeValid;
  result.optimizerReturnCode = attempt.optimizerReturnCode;
  result.collisionValid = attempt.collisionValid;
  result.collisionPosition = planningPoint(attempt.collisionPosition);
  result.collisionTimeS = attempt.collisionTimeS;
  result.collisionState = attempt.collisionState;
  result.dynamicViolationValid = attempt.dynamicViolationValid;
  result.dynamicQuantity = attempt.dynamicQuantity;
  result.dynamicValue = attempt.dynamicValue;
  result.dynamicLimit = attempt.dynamicLimit;
  result.dynamicTimeS = attempt.dynamicTimeS;
  return result;
}

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const LocalMotionIntent *right) {
  if (left.has_value() != (right != nullptr))
    return false;
  if (!left)
    return true;
  const bool leftMoving = left->speedNormalized > 1e-6;
  const bool rightMoving = right->speedNormalized > 1e-6;
  return leftMoving == rightMoving &&
         std::abs(left->directionBodyDeg - right->directionBodyDeg) <= 1e-6 &&
         std::abs(left->horizonM - right->horizonM) <= 1e-6 &&
         std::abs(left->maxDirectionDeviationDeg -
                   right->maxDirectionDeviationDeg) <= 1e-6;
}

const char *stateName(ScanReplanState state) {
  switch (state) {
    case ScanReplanState::INIT:
      return "scan_init";
    case ScanReplanState::WAIT_TARGET:
      return "scan_wait_target";
    case ScanReplanState::GEN_NEW_TRAJ:
      return "scan_generate_trajectory";
    case ScanReplanState::REPLAN_TRAJ:
      return "scan_replan_trajectory";
    case ScanReplanState::EXEC_TRAJ:
      return "scan_execute_trajectory";
    case ScanReplanState::EMERGENCY_STOP:
      return "scan_emergency_stop";
  }
  return "scan_invalid_state";
}

PlanParameters planParameters(const LocalPlannerParams &params) {
  PlanParameters output;
  output.max_vel_ = std::max(0.05, params.scan.maxVelocity);
  output.max_acc_ = std::max(0.05, params.scan.maxAcceleration);
  output.max_jerk_ = 4.0;
  output.vel_tolerance_ = std::max(0.0, params.scan.velocityTolerance);
  output.acc_tolerance_ = std::max(0.0, params.scan.accelerationTolerance);
  output.ctrl_pt_dist = std::max(0.05, params.scan.controlPointSpacing);
  output.feasibility_tolerance_ =
      std::max(0.0, params.scan.feasibilityTolerance);
  output.planning_horizon_ = std::max(0.5, params.scan.planningHorizon);
  return output;
}

BsplineOptimizerParams optimizerParameters(const LocalPlannerParams &params,
                                            const PlanParameters &plan) {
  BsplineOptimizerParams output;
  output.lambda_smooth = std::max(0.0, params.scan.smoothWeight);
  output.lambda_collision = std::max(0.0, params.scan.collisionWeight);
  output.lambda_feasibility = std::max(0.0, params.scan.feasibilityWeight);
  output.lambda_fitness = std::max(0.0, params.scan.fitnessWeight);
  output.dist0 = std::max(0.01, params.scan.collisionDistance);
  output.max_vel = plan.max_vel_;
  output.max_acc = plan.max_acc_;
  output.order = 3;
  return output;
}

ScanReplanParams fsmParameters(const LocalPlannerParams &params,
                               const PlanParameters &plan) {
  ScanReplanParams output;
  output.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  output.noReplanThreshold = std::max(0.01, params.scan.noReplanDistance);
  output.replanThreshold = std::max(output.noReplanThreshold,
                                    params.scan.replanDistance);
  output.planningHorizon = plan.planning_horizon_;
  output.emergencyTimeS = 1.0;
  output.enableFailSafe = true;
  output.maxReplanFailCount = 1000;
  output.bodyHeight = kOfficialBodyHeightM;
  return output;
}

SplineTarget splineTarget(const BsplineTrajectory &trajectory, double maxLinearSpeedMps) {
  SplineTarget target;
  target.controls.reserve(static_cast<std::size_t>(trajectory.positionPoints.cols()));
  for (Eigen::Index column = 0; column < trajectory.positionPoints.cols(); ++column) {
    target.controls.push_back({trajectory.positionPoints(0, column),
                               trajectory.positionPoints(1, column),
                               trajectory.positionPoints(2, column)});
  }
  target.order = trajectory.order;
  target.knots.assign(trajectory.knots.data(),
                      trajectory.knots.data() + trajectory.knots.size());
  target.startTimeS = trajectory.startTimeS;
  target.trajectoryId = trajectory.trajectoryId;
  target.maxLinearSpeedMps = maxLinearSpeedMps;
  return target;
}

class GridBinding {
 public:
  GridBinding(GridMap &adapter, const Grid &grid) : adapter_(adapter) {
    adapter_.setGrid(&grid);
  }
  ~GridBinding() { adapter_.setGrid(nullptr); }

 private:
  GridMap &adapter_;
};

}  // namespace

class Backend::Impl {
 public:
  explicit Impl(LocalPlannerParams params)
      : params_(std::move(params)), gridMap_(std::make_shared<GridMap>()) {
    initializeOfficialCore();
  }

  LocalPlan tick(const LocalPlanRequest &input,
                 const LocalPlanCancel &cancel) {
    return run(input, false, cancel);
  }

  LocalPlan checkCollision(const LocalPlanRequest &input,
                           const LocalPlanCancel &cancel) {
    return run(input, true, cancel);
  }

  LocalPlan run(const LocalPlanRequest &input, bool collisionTick,
                const LocalPlanCancel &cancel) {
    const auto started = std::chrono::steady_clock::now();
    manager_->setTimeSource([clock = input.clock, started] {
      return clock.afterElapsed(std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count());
    });
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    debug_.timestampS = input.clock.timestampS;
    debug_.scanAttempt = lastAttempt_;
    debug_.lastScanFailure = lastFailure_;

    const auto stop = [this, started](LocalPlanStatus status,
                                      std::string reason) {
      debug_.searchReason = std::move(reason);
      finishDebug(started);
      return LocalPlan::stopped(status);
    };
    if (cancel && cancel())
      return stop(LocalPlanStatus::Cancelled, "planning_cancelled");

    if (!std::isfinite(input.maxLinearSpeedMps) || input.maxLinearSpeedMps < 0.0)
      return stop(LocalPlanStatus::InvalidInput, "linear_speed_limit_invalid");

    const LocalRouteView *route = input.referenceRoute();
    if (route == nullptr || !route->valid() ||
        !finitePoint(input.robot.pose.position) ||
        !std::isfinite(input.robot.pose.yaw) ||
        !std::isfinite(input.clock.timestampS)) {
      return stop(LocalPlanStatus::InvalidInput, "route_invalid");
    }
    // Direct backend callers also obey reference-generation ownership. Task
    // validates each immutable snapshot on receipt; timer ticks reuse it.
    if (referenceIdentityChanged(input, *route)) {
      for (int index = 0; index < route->count; ++index) {
        if (!finitePoint(route->points[index]))
          return stop(LocalPlanStatus::InvalidInput, "route_invalid");
      }
    }
    if (const LocalMotionIntent *intent = input.intent(); intent != nullptr &&
        (!std::isfinite(intent->directionBodyDeg) ||
         !std::isfinite(intent->speedNormalized) ||
         !std::isfinite(intent->horizonM) ||
         !std::isfinite(intent->maxDirectionDeviationDeg) ||
         intent->horizonM <= 0.0)) {
      return stop(LocalPlanStatus::InvalidInput, "intent_invalid");
    }
    if (const LocalMotionIntent *intent = input.intent();
        intent != nullptr && intent->speedNormalized <= 1e-6) {
      return stop(LocalPlanStatus::NoPath, "intent_idle");
    }

    const auto gridStarted = std::chrono::steady_clock::now();
    Grid grid(params_, input);
    debug_.gridTimeMs = elapsedMs(gridStarted);
    debug_.occupiedCellCount = grid.occupiedCellCount();
    debug_.collisionPointCount = grid.collisionPointCount();
    debug_.predictedObstacleCount = grid.predictionCount();
    if (!grid.valid())
      return stop(LocalPlanStatus::InvalidInput, grid.reason());

    PlanParameters plan = planParameters(params_);
    plan.motion_intent_ = input.intent() != nullptr;
    if (input.maxLinearSpeedMps > 0.0)
      plan.max_vel_ = std::min(plan.max_vel_, input.maxLinearSpeedMps);
    if (plan.max_vel_ != manager_->pp_.max_vel_ ||
        plan.motion_intent_ != manager_->pp_.motion_intent_)
      manager_->updatePlanParameters(plan, optimizerParameters(params_, plan));

    const auto *activeSpline =
        active_ ? std::get_if<SplineTarget>(&active_->target()) : nullptr;
    if (activeSpline != nullptr && !referenceRejected_ && !speedReductionPending_ &&
        plan.max_vel_ < activeSpline->maxLinearSpeedMps) {
      // Executor holds the old trajectory during a speed reduction. Its
      // derivatives are no longer the robot's motion state, so start a fresh
      // FSM from odometry while retaining the manager's allocation and IDs.
      const int trajectoryId = manager_->local_data_.traj_id_;
      manager_->local_data_ = {};
      manager_->local_data_.traj_id_ = trajectoryId;
      fsm_ = std::make_unique<SCANReplanFSM>(*manager_, fsmParameters(params_, plan));
      referenceInitialized_ = false;
      speedReductionPending_ = true;
      active_.reset();
      activeSpline = nullptr;
    }

    GridBinding binding(*gridMap_, grid);
    FsmInput fsmInput;
    fsmInput.nowS = input.clock.timestampS;
    FsmOdometry odometry;
    odometry.position = eigenPoint(input.robot.pose.position);
    odometry.velocity = input.robot.kinematics.valid
                            ? eigenPoint(input.robot.kinematics.linearVelocity)
                            : Eigen::Vector3d::Zero();
    odometry.orientation =
        Eigen::AngleAxisd(input.robot.pose.yaw, Eigen::Vector3d::UnitZ());
    fsmInput.odometry = odometry;
    fsmInput.executionFrozen = input.clock.executionFrozen;
    if (const auto *intent = input.intent()) {
      fsmInput.motionIntentMaxDeviationRad =
          std::clamp(intent->maxDirectionDeviationDeg, 0.0, 90.0) * M_PI / 180.0;
    }

    FsmOutput output;
    if (collisionTick) {
      output = fsm_->checkFutureCollision(fsmInput);
    } else {
      // A new speed is a planning input, not a replacement guide identity.
      // Do not repeatedly inject subscriber events while a replan is pending:
      // they return before the official FSM timer can generate a trajectory.
      const bool speedReplan = activeSpline != nullptr &&
                               !referenceRejected_ &&
                               activeSpline->maxLinearSpeedMps != plan.max_vel_ &&
                               fsm_->state() == ScanReplanState::EXEC_TRAJ;
      if (referenceIdentityChanged(input, *route) || speedReplan) {
        std::vector<Eigen::Vector3d> reference;
        reference.reserve(static_cast<std::size_t>(route->count));
        for (int index = 0; index < route->count; ++index) {
          Eigen::Vector3d point = eigenPoint(route->points[index]);
          // LingTu routes use body-centre Z; upstream REFERENCE_PATH uses the
          // ground-following surface and adds grid_map/body_height internally.
          point.z() -= kOfficialBodyHeightM;
          reference.push_back(point);
        }
        fsmInput.referencePath = std::move(reference);
      }
      output = fsm_->tick(fsmInput);
    }
    if (!collisionTick && (output.targetAccepted || output.targetRejected)) {
      rememberReference(input, *route);
      referenceRejected_ = output.targetRejected;
    }
    debug_.searchReason = stateName(output.state);

    if (cancel && cancel()) {
      // Consume this discarded tick's attempt so a later tick without a new
      // attempt cannot capture it against a different accepted request.
      lastObservedAttemptId_ = manager_->reboundDebug().attemptId;
      return stop(LocalPlanStatus::Cancelled, "planning_cancelled");
    }
    recordAttempt(input, *route);
    if (output.localTargetBlocked) {
      active_.reset();
      return stop(LocalPlanStatus::Blocked, "scan_local_target_blocked");
    }
    if (output.state == ScanReplanState::EMERGENCY_STOP || output.emergencyStopIssued) {
      // Keep the upstream stop trajectory inside the FSM. The native executor
      // owns stopping; publishing this stationary spline as Ready hides failure.
      active_.reset();
      emergencyStopped_ = true;
      return stop(LocalPlanStatus::NearFieldStop, "scan_emergency_stop");
    }
    if (output.trajectory) {
      LocalPlan next = LocalPlan::spline(splineTarget(*output.trajectory, plan.max_vel_));
      if (next.ready()) {
        active_ = next;
        emergencyStopped_ = false;
        speedReductionPending_ = false;
        debug_.valid = true;
        debug_.trajectoryPointCount =
            static_cast<int>(output.trajectory->positionPoints.cols());
      }
    }
    if (referenceRejected_ && !active_)
      return stop(LocalPlanStatus::NoPath, "scan_target_rejected");
    if (emergencyStopped_)
      return stop(fsm_->hasTarget() ? LocalPlanStatus::NearFieldStop : LocalPlanStatus::NoPath,
                  fsm_->hasTarget() ? "scan_emergency_stop" : "scan_replan_failed");
    if (output.initializationFailed && !active_)
      return stop(LocalPlanStatus::Blocked, "scan_initialization_failed");
    if (active_) {
      debug_.valid = true;
      debug_.continuityReused = !output.trajectory.has_value();
      if (const auto *spline = std::get_if<SplineTarget>(&active_->target())) {
        debug_.trajectoryPointCount =
            static_cast<int>(spline->controls.size());
      }
      finishDebug(started);
      return *active_;
    }

    finishDebug(started);
    return LocalPlan::stopped(LocalPlanStatus::Pending);
  }

  void reset() {
    gridMap_->setGrid(nullptr);
    active_.reset();
    lastIntent_.reset();
    referenceInitialized_ = false;
    referenceRejected_ = false;
    speedReductionPending_ = false;
    emergencyStopped_ = false;
    lastFrameEpoch_ = 0;
    lastRouteGeneration_ = 0;
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    debug_.scanAttempt = lastAttempt_;
    debug_.lastScanFailure = lastFailure_;
    lastObservedAttemptId_ = 0;
    failureEpisodeActive_ = false;
    initializeOfficialCore();
  }

  LocalPlannerDebugSnapshot debugSnapshot() const { return debug_; }

 private:

  void recordAttempt(const LocalPlanRequest &input, const LocalRouteView &route) {
    const auto &attempt = manager_->reboundDebug();
    if (!attempt.attempted || attempt.attemptId == lastObservedAttemptId_)
      return;
    lastObservedAttemptId_ = attempt.attemptId;
    lastAttempt_ = attemptDiagnostics(attempt);
    debug_.scanAttempt = lastAttempt_;

    // The final attempted result at the tick boundary defines a failure episode.
    // Failed internal retries followed by success in this tick do not start one.
    if (attempt.success) {
      failureEpisodeActive_ = false;
      return;
    }
    if (failureEpisodeActive_)
      return;
    failureEpisodeActive_ = true;

    auto failure = std::make_shared<ScanFailureSnapshot>();
    failure->sequence = ++failureSequence_;
    failure->attempt = lastAttempt_;
    failure->clock = input.clock;
    failure->identity = input.identity;
    failure->robot = input.robot;
    failure->params = params_.scan;
    failure->checkObstacle = params_.checkObstacle;
    failure->maxLinearSpeedMps = input.maxLinearSpeedMps;
    if (const auto *intent = input.intent())
      failure->intent = *intent;
    failure->reference.assign(route.points, route.points + route.count);
    failure->referenceGeneration = route.generation;
    failure->referenceReachesGoal = route.reachesGoal;
    failure->collision = input.environment.collision;
    const auto &predictions = input.environment.predictions;
    if (predictions.count > 0 && predictions.fresh(input.clock.timestampS))
      failure->predictions.assign(predictions.obstacles, predictions.obstacles + predictions.count);
    failure->predictionsObservedAtS = predictions.observedAtS;
    failure->predictionsHorizonS = predictions.horizonS;
    auto &collision = failure->collision;
    if (collision.inflatedBits != nullptr && collision.inflatedBytes > 0U &&
        (!collision.inflatedStorage ||
         collision.inflatedStorage->data() != collision.inflatedBits ||
         collision.inflatedStorage->size() != collision.inflatedBytes)) {
      collision.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(
          collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
      collision.inflatedBits = collision.inflatedStorage->data();
    }
    failure->startPosition = planningPoint(attempt.startPosition);
    failure->startVelocity = planningPoint(attempt.startVelocity);
    failure->startAcceleration = planningPoint(attempt.startAcceleration);
    failure->targetPosition = planningPoint(attempt.targetPosition);
    failure->targetVelocity = planningPoint(attempt.targetVelocity);
    failure->polyInit = attempt.polyInit;
    failure->randomPolyInit = attempt.randomPolyInit;
    failure->candidateIntervalS = attempt.candidateIntervalS;
    failure->candidateControlPoints.reserve(
        static_cast<std::size_t>(attempt.candidateControlPoints.cols()));
    for (Eigen::Index column = 0; column < attempt.candidateControlPoints.cols(); ++column)
      failure->candidateControlPoints.push_back(
          planningPoint(attempt.candidateControlPoints.col(column)));
    lastFailure_ = std::move(failure);
    debug_.lastScanFailure = lastFailure_;
  }

  void initializeOfficialCore() {
    const PlanParameters plan = planParameters(params_);
    manager_ = std::make_unique<SCANPlannerManager>();
    manager_->initPlanModules(plan, optimizerParameters(params_, plan), gridMap_);
    fsm_ = std::make_unique<SCANReplanFSM>(*manager_,
                                           fsmParameters(params_, plan));
  }

  bool referenceIdentityChanged(const LocalPlanRequest &input,
                                const LocalRouteView &route) const {
    return !referenceInitialized_ ||
           lastFrameEpoch_ != input.identity.frameEpoch ||
           lastRouteGeneration_ != route.generation ||
           !sameIntent(lastIntent_, input.intent());
  }

  void rememberReference(const LocalPlanRequest &input,
                         const LocalRouteView &route) {
    referenceInitialized_ = true;
    lastFrameEpoch_ = input.identity.frameEpoch;
    lastRouteGeneration_ = route.generation;
    lastIntent_ = input.intent() == nullptr
                      ? std::nullopt
                      : std::optional<LocalMotionIntent>{*input.intent()};
  }

  void finishDebug(std::chrono::steady_clock::time_point started) {
    debug_.planningTimeMs = elapsedMs(started);
  }

  static double elapsedMs(std::chrono::steady_clock::time_point started) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - started)
        .count();
  }

  LocalPlannerParams params_;
  GridMap::Ptr gridMap_;
  std::unique_ptr<SCANPlannerManager> manager_;
  std::unique_ptr<SCANReplanFSM> fsm_;
  std::optional<LocalPlan> active_;
  std::optional<LocalMotionIntent> lastIntent_;
  bool referenceInitialized_{false};
  bool referenceRejected_{false};
  bool speedReductionPending_{false};
  bool emergencyStopped_{false};
  std::uint64_t lastFrameEpoch_{0};
  std::uint64_t lastRouteGeneration_{0};
  std::uint64_t lastObservedAttemptId_{0};
  std::uint64_t failureSequence_{0};
  bool failureEpisodeActive_{false};
  ScanAttemptDiagnostics lastAttempt_{};
  std::shared_ptr<const ScanFailureSnapshot> lastFailure_;
  LocalPlannerDebugSnapshot debug_{};
};

Backend::Backend(const LocalPlannerParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

Backend::~Backend() = default;
Backend::Backend(Backend &&) noexcept = default;
Backend &Backend::operator=(Backend &&) noexcept = default;

double Backend::fsmPeriodS() noexcept {
  return SCANReplanFSM::kTickPeriodS;
}

double Backend::collisionPeriodS() noexcept {
  return SCANReplanFSM::kFutureCollisionPeriodS;
}

LocalPlan Backend::tick(const LocalPlanRequest &input,
                        const LocalPlanCancel &cancel) {
  return impl_->tick(input, cancel);
}

LocalPlan Backend::checkCollision(const LocalPlanRequest &input,
                                  const LocalPlanCancel &cancel) {
  return impl_->checkCollision(input, cancel);
}

void Backend::reset() {
  impl_->reset();
}

LocalPlannerDebugSnapshot Backend::debugSnapshot() const {
  return impl_->debugSnapshot();
}

}  // namespace nav_kernel::local::scan
