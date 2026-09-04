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

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Eigen::Vector3d eigenPoint(const Vec3 &point) {
  return {point.x, point.y, point.z};
}

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const LocalMotionIntent *right) {
  if (left.has_value() != (right != nullptr))
    return false;
  if (!left)
    return true;
  return std::abs(left->speedNormalized - right->speedNormalized) <= 0.05 &&
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
  output.max_vel_ = std::max(0.05, params.autonomySpeed);
  output.max_acc_ = std::max(0.05, params.scan.maxAcceleration);
  output.max_jerk_ = 4.0;
  output.vel_tolerance_ = std::max(0.0, params.scan.velocityTolerance);
  output.acc_tolerance_ = std::max(0.0, params.scan.accelerationTolerance);
  output.ctrl_pt_dist = std::max(0.05, params.scan.controlPointSpacing);
  output.feasibility_tolerance_ =
      std::max(0.0, params.scan.feasibilityTolerance);
  output.planning_horizon_ = std::max(0.5, params.adjacentRange);
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
  // LingTu routes already carry body height in the planning frame.
  output.bodyHeight = 0.0;
  return output;
}

SplineTarget splineTarget(const BsplineTrajectory &trajectory) {
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
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    debug_.timestampS = input.clock.timestampS;

    const auto stop = [this, started](LocalPlanStatus status,
                                      std::string reason) {
      debug_.searchReason = std::move(reason);
      finishDebug(started);
      return LocalPlan::stopped(status);
    };
    if (cancel && cancel())
      return stop(LocalPlanStatus::Cancelled, "planning_cancelled");

    const LocalRouteView *route = input.route();
    if (route == nullptr || !route->valid() ||
        !finitePoint(input.robot.pose.position) ||
        !std::isfinite(input.robot.pose.yaw) ||
        !std::isfinite(input.clock.timestampS)) {
      return stop(LocalPlanStatus::InvalidInput, "route_invalid");
    }
    for (int index = 0; index < route->count; ++index) {
      if (!finitePoint(route->points[index]))
        return stop(LocalPlanStatus::InvalidInput, "route_invalid");
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
    if (!grid.valid())
      return stop(LocalPlanStatus::InvalidInput, grid.reason());

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

    FsmOutput output;
    if (collisionTick) {
      output = fsm_->checkFutureCollision(fsmInput);
    } else {
      const bool hardReferenceChange = referenceIdentityChanged(input, *route);
      if (hardReferenceChange)
        active_.reset();
      if (hardReferenceChange || referenceChanged(*route)) {
        std::vector<Eigen::Vector3d> reference;
        reference.reserve(static_cast<std::size_t>(route->count));
        for (int index = 0; index < route->count; ++index)
          reference.push_back(eigenPoint(route->points[index]));
        fsmInput.referencePath = std::move(reference);
      }
      output = fsm_->tick(fsmInput);
    }
    const auto retimeTrajectory = [&](FsmOutput &candidate) {
      if (!candidate.trajectory)
        return;
      const double completedAtS =
          input.clock.timestampS +
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      manager_->local_data_.start_time_ = completedAtS;
      candidate.trajectory->startTimeS = completedAtS;
      fsmInput.nowS = completedAtS;
    };
    retimeTrajectory(output);
    if (!collisionTick && output.targetAccepted)
      rememberReference(input, *route);
    debug_.searchTimeMs = manager_->pp_.time_search_ * 1000.0;
    debug_.splineTimeMs =
        (manager_->pp_.time_optimize_ + manager_->pp_.time_adjust_) * 1000.0;
    debug_.searchReason = stateName(output.state);

    if (cancel && cancel())
      return stop(LocalPlanStatus::Cancelled, "planning_cancelled");
    if (output.trajectory) {
      LocalPlan next = LocalPlan::spline(splineTarget(*output.trajectory));
      if (next.ready()) {
        active_ = next;
        debug_.valid = true;
        debug_.trajectoryPointCount =
            static_cast<int>(output.trajectory->positionPoints.cols());
      }
    }
    if (output.targetRejected && !active_)
      return stop(LocalPlanStatus::NoPath, "scan_target_rejected");
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
    lastReference_.clear();
    lastFrameEpoch_ = 0;
    lastRouteGeneration_ = 0;
    debug_ = {};
    debug_.backend = LocalPlannerBackend::Scan;
    initializeOfficialCore();
  }

  LocalPlannerDebugSnapshot debugSnapshot() const { return debug_; }

 private:

  void initializeOfficialCore() {
    const PlanParameters plan = planParameters(params_);
    manager_ = std::make_unique<SCANPlannerManager>();
    manager_->initPlanModules(plan, optimizerParameters(params_, plan), gridMap_);
    fsm_ = std::make_unique<SCANReplanFSM>(*manager_,
                                           fsmParameters(params_, plan));
  }

  bool referenceChanged(const LocalRouteView &route) const {
    if (lastReference_.empty() || !fsm_->hasTarget()) {
      return true;
    }
    return distance3D(lastReference_.back(), route.target()) >=
           std::max(0.5, params_.scan.replanDistance);
  }

  bool referenceIdentityChanged(const LocalPlanRequest &input,
                                const LocalRouteView &route) const {
    return lastReference_.empty() ||
           lastFrameEpoch_ != input.identity.frameEpoch ||
           lastRouteGeneration_ != route.generation ||
           !sameIntent(lastIntent_, input.intent());
  }

  void rememberReference(const LocalPlanRequest &input,
                         const LocalRouteView &route) {
    lastReference_.assign(route.points, route.points + route.count);
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
  std::vector<Vec3> lastReference_;
  std::uint64_t lastFrameEpoch_{0};
  std::uint64_t lastRouteGeneration_{0};
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
