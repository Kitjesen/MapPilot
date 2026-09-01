#include "planning/local/scan/upstream/plan_manage/scan_replan_fsm.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav_kernel::local::scan::upstream {

SCANReplanFSM::SCANReplanFSM(SCANPlannerManager &plannerManager, ScanReplanParams params)
    : plannerManager_(plannerManager), params_(std::move(params)) {}

FsmOutput SCANReplanFSM::tick(const FsmInput &input) {
  const ScanReplanState initialState = state_;
  FsmOutput output;
  updateRuntimeInput(input);
  acceptTargetInput(input, output);

  if (params_.navigationMode == ScanNavigationMode::PRESET_TARGET && haveOdom_ &&
      !presetTriggered_) {
    presetTriggered_ = true;
    activeWaypoints_ = params_.presetWaypoints;
    currentWaypoint_ = 0;
    trigger_ = true;
    if (!planNextWaypoint(input.nowS)) {
      output.targetRejected = true;
    } else {
      output.targetAccepted = true;
      changeState(ScanReplanState::GEN_NEW_TRAJ);
    }
  }

  switch (state_) {
    case ScanReplanState::INIT:
      if (!haveOdom_ || !trigger_) {
        return finalizeOutput(std::move(output), initialState);
      }
      changeState(ScanReplanState::WAIT_TARGET);
      break;

    case ScanReplanState::WAIT_TARGET:
      if (!haveTarget_) {
        return finalizeOutput(std::move(output), initialState);
      }
      changeState(ScanReplanState::GEN_NEW_TRAJ);
      break;

    case ScanReplanState::GEN_NEW_TRAJ: {
      setStartStateFromOdomOrCurrentTraj(input.nowS);
      const bool randomPolyInit = continuouslyCalledTimes_ != 1;
      if (callReboundReplan(true, randomPolyInit, input.nowS, output)) {
        replanFailCount_ = 0;
        changeState(ScanReplanState::EXEC_TRAJ);
        escapeEmergency_ = true;
      } else {
        ++replanFailCount_;
        changeState(ScanReplanState::GEN_NEW_TRAJ);
      }
      break;
    }

    case ScanReplanState::REPLAN_TRAJ:
      if (planFromCurrentTraj(input.nowS, output)) {
        replanFailCount_ = 0;
        changeState(ScanReplanState::EXEC_TRAJ);
      } else {
        ++replanFailCount_;
        changeState(ScanReplanState::REPLAN_TRAJ);
      }
      break;

    case ScanReplanState::EXEC_TRAJ: {
      LocalTrajData &info = plannerManager_.local_data_;
      const double currentTime = std::min(info.duration_, input.nowS - info.start_time_);
      const Eigen::Vector3d position = info.position_traj_.evaluateDeBoorT(currentTime);

      if (isWaypointSequenceMode() &&
          currentWaypoint_ + 1 < static_cast<int>(activeWaypoints_.size()) &&
          (endPt_ - odomPos_).norm() < 0.5) {
        ++currentWaypoint_;
        if (!planNextWaypoint(input.nowS)) {
          ++replanFailCount_;
        }
        changeState(ScanReplanState::GEN_NEW_TRAJ);
        return finalizeOutput(std::move(output), initialState);
      }

      if (currentTime > info.duration_ - 1e-2) {
        if (isWaypointSequenceMode() &&
            currentWaypoint_ + 1 < static_cast<int>(activeWaypoints_.size())) {
          ++currentWaypoint_;
          if (!planNextWaypoint(input.nowS)) {
            ++replanFailCount_;
          }
          changeState(ScanReplanState::GEN_NEW_TRAJ);
          return finalizeOutput(std::move(output), initialState);
        }

        if (isWaypointSequenceMode()) {
          activeWaypoints_.clear();
          currentWaypoint_ = 0;
        }
        haveTarget_ = false;
        output.targetFinished = true;
        changeState(ScanReplanState::WAIT_TARGET);
        return finalizeOutput(std::move(output), initialState);
      }
      if ((endPt_ - position).norm() < params_.noReplanThreshold ||
          (info.start_pos_ - position).norm() < params_.replanThreshold) {
        return finalizeOutput(std::move(output), initialState);
      }
      changeState(ScanReplanState::REPLAN_TRAJ);
      break;
    }

    case ScanReplanState::EMERGENCY_STOP:
      if (escapeEmergency_) {
        callEmergencyStop(odomPos_, input.nowS, output);
      } else if (params_.enableFailSafe && !needHoverStop_ && odomVel_.norm() < 0.1) {
        changeState(ScanReplanState::GEN_NEW_TRAJ);
      } else if (params_.enableFailSafe && needHoverStop_ && odomVel_.norm() < 0.1) {
        needHoverStop_ = false;
        haveTarget_ = false;
        trigger_ = false;
        changeState(ScanReplanState::WAIT_TARGET);
      }
      escapeEmergency_ = false;
      break;
  }

  finishProcess();
  return finalizeOutput(std::move(output), initialState);
}

FsmOutput SCANReplanFSM::checkFutureCollision(const FsmInput &input) {
  const ScanReplanState initialState = state_;
  FsmOutput output;
  updateRuntimeInput(input);

  LocalTrajData &info = plannerManager_.local_data_;
  if (state_ == ScanReplanState::WAIT_TARGET || info.start_time_ < 1e-5 ||
      !plannerManager_.grid_map_) {
    return finalizeOutput(std::move(output), initialState);
  }

  constexpr double timeStep = 0.01;
  const double currentTime = input.nowS - info.start_time_;
  const double twoThirdsTime = info.duration_ * 2.0 / 3.0;
  for (double time = currentTime; time < info.duration_; time += timeStep) {
    if (currentTime < twoThirdsTime && time >= twoThirdsTime) {
      break;
    }

    const Eigen::Vector3d position = info.position_traj_.evaluateDeBoorT(time);
    const Eigen::Vector3d nextPosition =
        info.position_traj_.evaluateDeBoorT(std::min(time + timeStep, info.duration_));
    if (plannerManager_.grid_map_->getInflateOccupancy(
            position, estimateYawFromSegment(position, nextPosition)) == 0) {
      continue;
    }

    output.collisionDetected = true;
    output.collisionTimeAheadS = time - currentTime;
    if (planFromCurrentTraj(input.nowS, output)) {
      changeState(ScanReplanState::EXEC_TRAJ);
    } else if (time - currentTime < params_.emergencyTimeS) {
      changeState(ScanReplanState::EMERGENCY_STOP);
    } else {
      changeState(ScanReplanState::REPLAN_TRAJ);
    }
    return finalizeOutput(std::move(output), initialState);
  }

  return finalizeOutput(std::move(output), initialState);
}

ScanReplanState SCANReplanFSM::state() const noexcept {
  return state_;
}

bool SCANReplanFSM::hasTarget() const noexcept {
  return haveTarget_;
}

void SCANReplanFSM::updateRuntimeInput(const FsmInput &input) {
  if (input.executionFrozen.has_value()) {
    executionFrozen_ = *input.executionFrozen;
  }
  updateLocalTrajTimeFreeze(input.nowS);
  if (!input.odometry.has_value()) {
    return;
  }

  odomPos_ = input.odometry->position;
  odomVel_ = input.odometry->velocity;
  odomOrientation_ = input.odometry->orientation;
  haveOdom_ = true;
  if (params_.navigationMode == ScanNavigationMode::MANUAL_TARGET && !manualGoalHeightReady_) {
    manualGoalHeight_ = odomPos_(2);
    manualGoalHeightReady_ = true;
  }
}

void SCANReplanFSM::acceptTargetInput(const FsmInput &input, FsmOutput &output) {
  if (input.goal.has_value() && params_.navigationMode == ScanNavigationMode::MANUAL_TARGET) {
    if (!manualGoalHeightReady_ || !planManualTarget(*input.goal, input.nowS, output)) {
      output.targetRejected = true;
    } else {
      output.targetAccepted = true;
    }
  }

  if (input.referencePath.has_value() &&
      params_.navigationMode == ScanNavigationMode::REFERENCE_PATH) {
    if (!haveOdom_ || !planReferencePath(*input.referencePath, input.nowS, output)) {
      output.targetRejected = true;
    } else {
      output.targetAccepted = true;
    }
  }
}

void SCANReplanFSM::updateLocalTrajTimeFreeze(double nowS) {
  const double delta = nowS - lastFreezeUpdateTimeS_;
  lastFreezeUpdateTimeS_ = nowS;
  if (delta <= 0.0 || delta > 0.2) {
    return;
  }

  LocalTrajData &info = plannerManager_.local_data_;
  if (executionFrozen_ && info.start_time_ > 1e-5) {
    info.start_time_ += delta;
  }
}

void SCANReplanFSM::changeState(ScanReplanState newState) {
  continuouslyCalledTimes_ = newState == state_ ? continuouslyCalledTimes_ + 1 : 1;
  state_ = newState;
}

void SCANReplanFSM::finishProcess() {
  if (replanFailCount_ < params_.maxReplanFailCount) {
    return;
  }
  replanFailCount_ = 0;
  needHoverStop_ = true;
  escapeEmergency_ = true;
  changeState(ScanReplanState::EMERGENCY_STOP);
}

bool SCANReplanFSM::planManualTarget(const Eigen::Vector3d &goal, double nowS, FsmOutput &) {
  if (goal(2) < -0.1) {
    return false;
  }
  trigger_ = true;
  endPt_ = {goal(0), goal(1), manualGoalHeight_};
  bool success =
      plannerManager_.planGlobalTraj(odomPos_, odomVel_, Eigen::Vector3d::Zero(), endPt_,
                                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), nowS);
  if (success) {
    success = adjustGlobalTargetIfOccupied();
  }
  if (!success) {
    return false;
  }

  endVel_.setZero();
  haveTarget_ = true;
  haveNewTarget_ = true;
  if (state_ == ScanReplanState::WAIT_TARGET) {
    changeState(ScanReplanState::GEN_NEW_TRAJ);
  } else if (state_ == ScanReplanState::EXEC_TRAJ) {
    changeState(ScanReplanState::REPLAN_TRAJ);
  }
  return true;
}

bool SCANReplanFSM::planReferencePath(const std::vector<Eigen::Vector3d> &path, double nowS,
                                      FsmOutput &) {
  if (path.empty()) {
    return false;
  }

  trigger_ = true;
  endPt_ = path.back();
  endPt_(2) += params_.bodyHeight;

  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(path.size());
  constexpr double minDistance = 0.5;
  for (const Eigen::Vector3d &pathPoint : path) {
    Eigen::Vector3d waypoint = pathPoint;
    waypoint(2) += params_.bodyHeight;
    if (waypoints.empty() || (waypoint - waypoints.back()).norm() >= minDistance) {
      waypoints.push_back(waypoint);
    }
  }
  if ((waypoints.back() - endPt_).norm() > 1e-6) {
    waypoints.push_back(endPt_);
  }
  if (!planGlobalTrajByWaypoints(waypoints, nowS)) {
    return false;
  }

  if (state_ == ScanReplanState::WAIT_TARGET) {
    changeState(ScanReplanState::GEN_NEW_TRAJ);
  } else if (state_ == ScanReplanState::EXEC_TRAJ) {
    changeState(ScanReplanState::REPLAN_TRAJ);
  }
  return true;
}

bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                                              double nowS) {
  if (waypoints.size() < 2) {
    return false;
  }

  endPt_ = waypoints.back();
  const std::vector<Eigen::Vector3d> referenceWaypoints(waypoints.begin() + 1, waypoints.end());
  if (!plannerManager_.planGlobalTrajWaypoints(
          waypoints.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), referenceWaypoints,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), nowS) ||
      !adjustGlobalTargetIfOccupied()) {
    return false;
  }

  endVel_.setZero();
  haveTarget_ = true;
  haveNewTarget_ = true;
  return true;
}

bool SCANReplanFSM::planNextWaypoint(double nowS) {
  if (currentWaypoint_ < 0 || currentWaypoint_ >= static_cast<int>(activeWaypoints_.size())) {
    return false;
  }

  endPt_ = activeWaypoints_[currentWaypoint_];
  setStartStateFromOdomOrCurrentTraj(nowS);
  if (!plannerManager_.planGlobalTraj(startPt_, startVel_, startAcc_, endPt_,
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), nowS) ||
      !adjustGlobalTargetIfOccupied()) {
    return false;
  }

  endVel_.setZero();
  haveTarget_ = true;
  haveNewTarget_ = true;
  return true;
}

bool SCANReplanFSM::adjustGlobalTargetIfOccupied() {
  GridMap::Ptr map = plannerManager_.grid_map_;
  GlobalTrajData &globalData = plannerManager_.global_data_;
  const double duration = globalData.global_duration_;
  if (!map || duration < 1e-3) {
    return true;
  }

  constexpr double sampleStep = 0.05;
  const int sampleCount = std::max(1, static_cast<int>(std::ceil(duration / sampleStep)));
  const Eigen::Vector3d finalPoint = globalData.global_traj_.evaluate(duration);
  const Eigen::Vector3d previousFinalPoint =
      globalData.global_traj_.evaluate(duration * (sampleCount - 1) / sampleCount);
  if (map->getInflateOccupancy(finalPoint,
                               estimateYawFromSegment(previousFinalPoint, finalPoint)) <= 0) {
    return true;
  }

  for (int index = sampleCount; index >= 0; --index) {
    const double time = duration * index / sampleCount;
    const double previousTime = duration * std::max(0, index - 1) / sampleCount;
    const Eigen::Vector3d point = globalData.global_traj_.evaluate(time);
    const Eigen::Vector3d previousPoint = globalData.global_traj_.evaluate(previousTime);
    if (map->getInflateOccupancy(point, estimateYawFromSegment(previousPoint, point)) == 0) {
      endPt_ = point;
      globalData.global_duration_ = time;
      globalData.last_progress_time_ = std::min(globalData.last_progress_time_, time);
      return true;
    }
  }
  return false;
}

bool SCANReplanFSM::isWaypointSequenceMode() const noexcept {
  return params_.navigationMode == ScanNavigationMode::PRESET_TARGET;
}

bool SCANReplanFSM::planFromCurrentTraj(double nowS, FsmOutput &output) {
  LocalTrajData &info = plannerManager_.local_data_;
  const double currentTime = std::min(std::max(nowS - info.start_time_, 0.0), info.duration_);

  if (params_.navigationMode == ScanNavigationMode::REFERENCE_PATH) {
    startPt_ = info.position_traj_.evaluateDeBoorT(currentTime);
    startVel_ = info.velocity_traj_.evaluateDeBoorT(currentTime);
    startAcc_ = info.acceleration_traj_.evaluateDeBoorT(currentTime);
    if (callReboundReplan(false, false, nowS, output)) {
      return true;
    }
    if (callReboundReplan(true, false, nowS, output)) {
      return true;
    }
    return callReboundReplan(true, true, nowS, output);
  }

  startPt_ = odomPos_;
  startVel_ = info.velocity_traj_.evaluateDeBoorT(currentTime);
  startAcc_ = info.acceleration_traj_.evaluateDeBoorT(currentTime);
  const Eigen::Vector2d toGoal = endPt_.head<2>() - odomPos_.head<2>();
  if (toGoal.norm() > 1e-3 && startVel_.head<2>().dot(toGoal) < 0.0) {
    startVel_.setZero();
    startAcc_.setZero();
  }

  if (!plannerManager_.planGlobalTraj(startPt_, startVel_, startAcc_, endPt_,
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), nowS) ||
      !adjustGlobalTargetIfOccupied()) {
    return false;
  }
  if (callReboundReplan(true, false, nowS, output)) {
    return true;
  }
  return callReboundReplan(true, true, nowS, output);
}

void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj(double nowS) {
  startPt_ = odomPos_;
  startVel_ = odomVel_;
  startAcc_.setZero();

  LocalTrajData &info = plannerManager_.local_data_;
  if (info.start_time_ < 1e-5 || info.duration_ <= 1e-5) {
    return;
  }
  const double rawCurrentTime = nowS - info.start_time_;
  if (rawCurrentTime < -1e-3 || rawCurrentTime > info.duration_ + 0.2) {
    return;
  }

  const double currentTime = std::min(std::max(rawCurrentTime, 0.0), info.duration_);
  startVel_ = info.velocity_traj_.evaluateDeBoorT(currentTime);
  startAcc_ = info.acceleration_traj_.evaluateDeBoorT(currentTime);
  const Eigen::Vector2d toGoal = endPt_.head<2>() - odomPos_.head<2>();
  if (toGoal.norm() > 1e-3 && startVel_.head<2>().dot(toGoal) < 0.0) {
    startVel_.setZero();
    startAcc_.setZero();
  }
}

bool SCANReplanFSM::callReboundReplan(bool usePolyInit, bool randomPolyTraj, double nowS,
                                      FsmOutput &output) {
  getLocalTarget();
  const bool success =
      plannerManager_.reboundReplan(startPt_, startVel_, startAcc_, localTargetPt_, localTargetVel_,
                                    haveNewTarget_ || usePolyInit, randomPolyTraj, nowS);
  haveNewTarget_ = false;
  if (success) {
    setTrajectoryOutput(output);
  }
  return success;
}

bool SCANReplanFSM::callEmergencyStop(const Eigen::Vector3d &stopPos, double nowS,
                                      FsmOutput &output) {
  const bool success = plannerManager_.EmergencyStop(stopPos, nowS);
  if (success) {
    output.emergencyStopIssued = true;
    setTrajectoryOutput(output);
  }
  return success;
}

void SCANReplanFSM::getLocalTarget() {
  const double maxVelocity = plannerManager_.pp_.max_vel_;
  const double maxAcceleration = plannerManager_.pp_.max_acc_;
  const double duration = plannerManager_.global_data_.global_duration_;
  double timeStep = maxVelocity > 1e-6 ? params_.planningHorizon / 20.0 / maxVelocity : 0.01;
  timeStep = std::max(timeStep, 0.01);

  double projectionTime = 0.0;
  double minimumDistanceToStart = 9999.0;
  for (double time = 0.0; time < duration; time += timeStep) {
    const Eigen::Vector3d position = plannerManager_.global_data_.getPosition(time);
    const double distanceToStart = (position - startPt_).norm();
    if (distanceToStart < minimumDistanceToStart) {
      minimumDistanceToStart = distanceToStart;
      projectionTime = time;
    }
  }

  double targetTime = duration;
  double totalDistance = 0.0;
  bool targetFound = false;
  Eigen::Vector3d previousPosition = plannerManager_.global_data_.getPosition(projectionTime);
  localTargetPt_ = endPt_;
  for (double time = projectionTime; time < duration; time += timeStep) {
    const Eigen::Vector3d position = plannerManager_.global_data_.getPosition(time);
    totalDistance += (position - previousPosition).norm();
    if (totalDistance >= params_.planningHorizon) {
      localTargetPt_ = position;
      targetTime = time;
      targetFound = true;
      break;
    }
    previousPosition = position;
  }
  plannerManager_.global_data_.last_progress_time_ = targetFound ? targetTime : duration;

  const auto targetOccupancy = [this](const Eigen::Vector3d &point) {
    return plannerManager_.grid_map_->getInflateOccupancy(point,
                                                          estimateYawFromSegment(odomPos_, point));
  };
  if (targetOccupancy(localTargetPt_) != 0) {
    bool foundFreeTarget = false;
    double adjustedTime = targetTime;
    for (double delta = 0.0; delta <= plannerManager_.global_data_.global_duration_;
         delta += timeStep) {
      const double forwardTime = targetTime + delta;
      if (forwardTime <= plannerManager_.global_data_.global_duration_) {
        const Eigen::Vector3d point = plannerManager_.global_data_.getPosition(forwardTime);
        if (targetOccupancy(point) == 0) {
          localTargetPt_ = point;
          adjustedTime = forwardTime;
          foundFreeTarget = true;
          break;
        }
      }

      const double backwardTime = targetTime - delta;
      if (backwardTime >= std::max(0.0, projectionTime)) {
        const Eigen::Vector3d point = plannerManager_.global_data_.getPosition(backwardTime);
        if (targetOccupancy(point) == 0) {
          localTargetPt_ = point;
          adjustedTime = backwardTime;
          foundFreeTarget = true;
          break;
        }
      }
    }
    if (foundFreeTarget) {
      targetTime = adjustedTime;
    }
  }

  if ((endPt_ - localTargetPt_).norm() < maxVelocity * maxVelocity / (2.0 * maxAcceleration)) {
    localTargetVel_.setZero();
  } else {
    localTargetVel_ = plannerManager_.global_data_.getVelocity(targetTime);
    if (localTargetVel_.norm() > maxVelocity) {
      localTargetVel_ = localTargetVel_.normalized() * maxVelocity;
    }
  }
}

void SCANReplanFSM::setTrajectoryOutput(FsmOutput &output) const {
  const LocalTrajData &info = plannerManager_.local_data_;
  BsplineTrajectory trajectory;
  trajectory.order = 3;
  trajectory.startTimeS = info.start_time_;
  trajectory.trajectoryId = info.traj_id_;
  trajectory.positionPoints = info.position_traj_.getControlPoint();
  trajectory.knots = info.position_traj_.getKnot();
  output.trajectory = std::move(trajectory);
}

double SCANReplanFSM::getOdomYaw() const {
  const Eigen::Vector3d heading = odomOrientation_.toRotationMatrix().col(0);
  if (heading.head<2>().squaredNorm() < 1e-8) {
    return 0.0;
  }
  return std::atan2(heading(1), heading(0));
}

double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from,
                                             const Eigen::Vector3d &to) const {
  const Eigen::Vector2d difference(to(0) - from(0), to(1) - from(1));
  if (difference.squaredNorm() < 1e-8) {
    return getOdomYaw();
  }
  return std::atan2(difference(1), difference(0));
}

FsmOutput SCANReplanFSM::finalizeOutput(FsmOutput output, ScanReplanState initialState) const {
  output.state = state_;
  output.stateChanged = state_ != initialState;
  return output;
}

}  // namespace nav_kernel::local::scan::upstream
