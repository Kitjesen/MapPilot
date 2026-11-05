/*
 * PlannerFsm：管理 SCAN 局部轨迹从接收目标到执行、续接和停止的完整流程。
 *
 * 输入：FsmInput 中的时间、里程计、目标/参考路线、执行反馈和接管提前量。
 * 输出：FsmOutput 中的状态、规划结果、待接管样条及目标/碰撞/停止标志。
 * 协作：SCANPlannerManager 组织轨迹求解，GridMap 提供三维占据查询；
 *       跟踪控制器计算速度命令，执行层负责最终运动仲裁。
 * 调用：外部工作线程按 100 Hz / 20 Hz 目标频率分别调用两个入口，
 *       这里不创建线程或定时器；求解耗时可能延迟下一次调用。
 * 阅读顺序：tick -> getLocalTarget -> callReboundReplan -> planFromCurrentTraj。
 *
 * 函数导读（完整声明见 planner_fsm.h）：
 *
 * 入口与状态读取：
 *   PlannerFsm()                 绑定 Manager 并保存状态机参数。
 *   tick()                       推进六状态机，触发首次求解、续接和完成处理。
 *   checkFutureCollision()       检查未来轨迹，优先重规划，需要时进入紧停态。
 *   state() / hasTarget()        读取当前状态及目标是否存在。
 *   localTarget()                读取局部目标位置（在 .h 内联）。
 *   localTargetReady()           判断局部目标是否已准备好（在 .h 内联）。
 *
 * 输入、时钟与状态维护：
 *   adoptSpeedTransition()      Adopt a checked prefix before normal replanning.
 *   abandonTrajectory()         Withdraw old offers and restart from odometry.
 *   updateRuntimeInput()         合并里程计、执行确认、冻结及 MotionIntent 信息。
 *   acceptTargetInput()          按当前模式接收单目标或参考路线。
 *   updateLocalTrajTimeFreeze()   冻结执行时平移轨迹起始时间，保持参考进度。
 *   changeState()                切换状态，维护同一状态的连续调用次数。
 *   finishProcess()              处理规划失败次数上限，达到上限时进入紧停态。
 *                                此名称不表示任务正常完成。
 *
 * 参考路线与局部目标：
 *   planManualTarget()           为单目标构建参考，按需调整占用终点。
 *   planReferencePath()          接收有序三维参考点，保留弯点/高度并加密参考。
 *   planGlobalTrajByWaypoints()   从给定航路点构建 SCAN 内部完整参考曲线；
 *                                此处不执行 OctoPlanner 的地图全局寻路。
 *   planNextWaypoint()           为当前预设航点准备参考与规划边界。
 *   adjustGlobalTargetIfOccupied() 单目标/预设终点占用时，沿原参考回退选点。
 *   isWaypointSequenceMode()     判断是否使用预设航点模式。
 *   getLocalTarget()             沿参考选择局部目标、参考区间和末速度。
 *
 * 求解边界与轨迹续接：
 *   setStartStateFromOdomOrCurrentTraj() 从实测状态/有效旧轨迹准备求解边界。
 *   trackingReferenceSeparated() 判断实测位置是否已明显偏离计划参考。
 *   planFromCurrentTraj()        选择未来接管边界或实测边界，组织替换轨迹求解。
 *   callReboundReplan()          选择局部目标，调用 Manager 的局部优化并收集结果。
 *   trajectoryClear()            Check cylinder occupancy over the requested spline interval.
 *   preferExecutingTail()        比较候选与旧段的目标/时长，判断是否继续有效旧尾段。
 *
 * 异步接管与停止：
 *   offerHandoff()               输出等待接管的新轨迹及交接状态。
 *   reconcileHandoff()           处理接收/撤回结果，确认后更新活动轨迹。
 *   withdrawHandoff()            撤回待接管请求，并处理已发生的接收结果。
 *   callEmergencyStop()          请求静止样条并设置停止标志；底盘命令由执行层处理。
 *
 * MotionIntent（操作方向/速度意图）的临时绕障目标：
 *   usableMotionIntentDetour()   检查候选的距离、方向偏差和连接占据。
 *   selectMotionIntentDetour()   依次尝试前向可用段与允许方向内的侧向目标。
 *   resetMotionIntentDetourSearch() 清除临时目标及候选尝试进度。
 *
 * 输出与朝向辅助：
 *   setTrajectoryOutput()        两个重载：输出活动轨迹或指定轨迹的样条数据。
 *   finalizeOutput()             补齐最终状态、状态变化及规划诊断标志。
 *   getOdomYaw()                 从实测姿态提取水平朝向。
 *   estimateYawFromSegment()     从线段方向估计朝向，重合时使用实测朝向。
 */

#include "planning/local/scan/upstream/plan_manage/planner_fsm.h"
#include "planning/local/planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav_kernel::local::scan::upstream {

PlannerFsm::PlannerFsm(SCANPlannerManager &plannerManager, ScanReplanParams params)
    : plannerManager_(plannerManager), params_(std::move(params)) {}

FsmOutput PlannerFsm::tick(const FsmInput &input) {
  const ScanReplanState initialState = state_;
  FsmOutput output;
  updateRuntimeInput(input);
  const bool handoffResolved = reconcileHandoff(input, output);
  acceptTargetInput(input, output);
  if (output.targetAccepted) {
    initializationFailed_ = false;
    referenceReplacementPending_ = true;
    retainedReplanPosition_.reset();
  }

  // Upstream target subscribers only mutate the FSM state. Planning starts on
  // the next 100 Hz timer callback, never in the subscriber callback itself.
  if (input.goal.has_value() || input.referencePath.has_value()) {
    withdrawHandoff(input, output);
    return finalizeOutput(std::move(output), initialState);
  }
  if (handoffResolved) return finalizeOutput(std::move(output), initialState);

  if (pendingTrajectory_) {
    if (executionFrozen_) {
      withdrawHandoff(input, output);
      if (output.trajectory) return finalizeOutput(std::move(output), initialState);
      changeState(ScanReplanState::REPLAN_TRAJ);
    } else {
      offerHandoff(output);
      return finalizeOutput(std::move(output), initialState);
    }
  }

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
        referenceReplacementPending_ = false;
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
        // A speed bridge awaiting physical reception still needs its continuation.
        if (speedTransitionEndS_ <= 0.0 || (handoffLeadTimeS_ && !executionFrozen_))
          changeState(ScanReplanState::EXEC_TRAJ);
      } else {
        ++replanFailCount_;
        changeState(localTargetBlocked_ ? ScanReplanState::GEN_NEW_TRAJ
                                       : ScanReplanState::REPLAN_TRAJ);
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
        // A reference path can require several local horizons, including short
        // collision-free fallback segments. Their end time is not arrival at
        // the reference endpoint; continue from measured odometry.
        if (params_.navigationMode == ScanNavigationMode::REFERENCE_PATH &&
            (endPt_ - odomPos_).norm() > std::max(0.01, params_.noReplanThreshold)) {
          changeState(ScanReplanState::GEN_NEW_TRAJ);
          return finalizeOutput(std::move(output), initialState);
        }
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
        resetMotionIntentDetourSearch();
        output.targetFinished = true;
        changeState(ScanReplanState::WAIT_TARGET);
        return finalizeOutput(std::move(output), initialState);
      }
      const Eigen::Vector3d trajectoryEnd =
          info.position_traj_.evaluateDeBoorT(info.duration_);
      const bool temporarySegment =
          (endPt_ - trajectoryEnd).norm() > params_.noReplanThreshold;
      double replanDistance = params_.replanThreshold;
      if (plannerManager_.pp_.motion_intent_) {
        // Nominal progress can lead the robot near a short detour's end.
        // Keep tracking that tail until observed arrival or spline completion;
        // the collision timer still replans when its remaining path is blocked.
        if (motionIntentDetour_ &&
            (*motionIntentDetour_ - position).head<2>().norm() < 0.2 &&
            (*motionIntentDetour_ - odomPos_).head<2>().norm() >
                std::max(0.01, params_.noReplanThreshold)) {
          return finalizeOutput(std::move(output), initialState);
        }
        const double segmentLength = (trajectoryEnd - info.start_pos_).norm();
        if (temporarySegment) {
          // A short detour can be entirely inside the normal replan distance.
          // Continue it after half its progress so the next spline is ready
          // before this one reaches its zero-velocity endpoint.
          replanDistance = std::min(
              params_.replanThreshold,
              std::max(params_.noReplanThreshold, 0.5 * segmentLength));
        }
      }
      const double lead = handoffLeadTimeS_.value_or(kFutureCollisionPeriodS);
      // Leave one solve/transfer budget beyond the future handoff boundary.
      // A retained safe tail must not be optimized again on every timer tick.
      const bool prepareContinuation = !plannerManager_.pp_.motion_intent_ &&
          !executionFrozen_ && temporarySegment && !retainedReplanPosition_ &&
          info.duration_ - currentTime <= 2.0 * lead + 3.0 * kTickPeriodS;
      const bool trackingLag = params_.navigationMode == ScanNavigationMode::REFERENCE_PATH &&
          trackingReferenceSeparated(info, currentTime) &&
          (endPt_ - odomPos_).norm() > params_.noReplanThreshold;
      if (!prepareContinuation && !trackingLag &&
          ((endPt_ - position).norm() < params_.noReplanThreshold ||
           (retainedReplanPosition_.value_or(info.start_pos_) - position).norm() < replanDistance)) {
        return finalizeOutput(std::move(output), initialState);
      }
      changeState(ScanReplanState::REPLAN_TRAJ);
      break;
    }

    case ScanReplanState::EMERGENCY_STOP: {
      const bool stopped = odomVel_.allFinite() && odomVel_.norm() < 0.1;
      if (escapeEmergency_) {
        callEmergencyStop(odomPos_, input.nowS, output);
      } else if (params_.enableFailSafe && !needHoverStop_ && stopped) {
        changeState(ScanReplanState::GEN_NEW_TRAJ);
      } else if (params_.enableFailSafe && needHoverStop_ && stopped) {
        needHoverStop_ = false;
        if (plannerManager_.pp_.motion_intent_ && haveTarget_) {
          // Held input keeps the same reference after a stop. Retry it once
          // stopped; the owning executor resets this FSM when intent ends.
          changeState(ScanReplanState::GEN_NEW_TRAJ);
        } else {
          haveTarget_ = false;
          trigger_ = false;
          resetMotionIntentDetourSearch();
          changeState(ScanReplanState::WAIT_TARGET);
        }
      }
      escapeEmergency_ = false;
      break;
    }
  }

  finishProcess();
  return finalizeOutput(std::move(output), initialState);
}

FsmOutput PlannerFsm::checkFutureCollision(const FsmInput &input) {
  const ScanReplanState initialState = state_;
  FsmOutput output;
  updateRuntimeInput(input);
  if (reconcileHandoff(input, output))
    return finalizeOutput(std::move(output), initialState);

  LocalTrajData &info = plannerManager_.local_data_;
  if (localTargetBlocked_ || state_ == ScanReplanState::WAIT_TARGET || info.start_time_ < 1e-5 ||
      !plannerManager_.grid_map_) {
    return finalizeOutput(std::move(output), initialState);
  }

  constexpr double timeStep = 0.01;
  const double currentTime = input.nowS - info.start_time_;
  const double twoThirdsTime = info.duration_ * 2.0 / 3.0;
  const double oldEnd = pendingTrajectory_
                           ? std::min(info.duration_, pendingTrajectory_->start_time_ - info.start_time_)
                           : info.duration_;
  for (double time = std::max(0.0, currentTime); time < oldEnd; time += timeStep) {
    if (currentTime < twoThirdsTime && time >= twoThirdsTime) {
      break;
    }

    const Eigen::Vector3d position = info.position_traj_.evaluateDeBoorT(time);
    const double nextTime = std::min(time + timeStep, oldEnd);
    const Eigen::Vector3d nextPosition =
        info.position_traj_.evaluateDeBoorT(nextTime);
    if (plannerManager_.grid_map_->getInflateOccupancy(
            position, estimateYawFromSegment(position, nextPosition)) == 0) {
      continue;
    }

    output.collisionDetected = true;
    output.collisionTimeAheadS = time - currentTime;
    withdrawHandoff(input, output);
    if (output.trajectory) return finalizeOutput(std::move(output), initialState);
    if (planFromCurrentTraj(input.nowS, output,
                            input.nowS + 0.5 * output.collisionTimeAheadS)) {
      changeState(ScanReplanState::EXEC_TRAJ);
    } else if (time - currentTime < params_.emergencyTimeS) {
      changeState(ScanReplanState::EMERGENCY_STOP);
    } else {
      changeState(ScanReplanState::REPLAN_TRAJ);
    }
    return finalizeOutput(std::move(output), initialState);
  }

  if (pendingTrajectory_) {
    const bool clear = trajectoryClear(*pendingTrajectory_,
        std::max(0.0, input.nowS - pendingTrajectory_->start_time_),
        pendingTrajectory_->duration_);
    // Reception can happen while the geometry query runs. Never discard an
    // already received curve or offer a query result after its deadline.
    reconcileHandoff(input, output);
    if (!clear) {
      withdrawHandoff(input, output);
      changeState(output.trajectory ? ScanReplanState::EMERGENCY_STOP
                                   : ScanReplanState::REPLAN_TRAJ);
    } else if (pendingTrajectory_) {
      offerHandoff(output);
    }
  }

  return finalizeOutput(std::move(output), initialState);
}

ScanReplanState PlannerFsm::state() const noexcept {
  return state_;
}

bool PlannerFsm::hasTarget() const noexcept {
  return haveTarget_;
}

void PlannerFsm::adoptSpeedTransition(double transitionEndS) {
  if (pendingHandoff_) pendingHandoff_->withdraw();
  pendingHandoff_.reset();
  pendingTrajectory_.reset();
  speedTransitionEndS_ = transitionEndS;
  acknowledgedTrajectoryId_ = 0;
  retainedReplanPosition_.reset();
  referenceReplacementPending_ = true;
  changeState(ScanReplanState::REPLAN_TRAJ);
}

void PlannerFsm::abandonTrajectory() {
  const int allocatedId = std::max(plannerManager_.local_data_.traj_id_,
      pendingTrajectory_ ? pendingTrajectory_->traj_id_ : 0);
  if (pendingHandoff_) pendingHandoff_->withdraw();
  pendingHandoff_.reset();
  pendingTrajectory_.reset();
  plannerManager_.local_data_ = {};
  plannerManager_.local_data_.traj_id_ = allocatedId;
  speedTransitionEndS_ = 0;
  acknowledgedTrajectoryId_ = 0;
  retainedReplanPosition_.reset();
  referenceReplacementPending_ = true;
  changeState(ScanReplanState::GEN_NEW_TRAJ);
}

void PlannerFsm::updateRuntimeInput(const FsmInput &input) {
  handoffLeadTimeS_ = input.handoffLeadTimeS;
  LocalTrajData &trajectory = plannerManager_.local_data_;
  if (input.executionStartTimeS && input.executingTrajectoryId == trajectory.traj_id_ &&
      input.executingTrajectoryId != acknowledgedTrajectoryId_) {
    // The first spline starts when the executor receives it, after planning.
    trajectory.start_time_ = *input.executionStartTimeS;
    acknowledgedTrajectoryId_ = input.executingTrajectoryId;
    lastFreezeUpdateTimeS_ = input.nowS;
    freezeClockReady_ = true;
  }
  motionIntentMaxDeviationRad_ = input.motionIntentMaxDeviationRad;
  if (!motionIntentMaxDeviationRad_) {
    resetMotionIntentDetourSearch();
  } else if (motionIntentAdvancePending_) {
    if (motionIntentDetour_) {
      motionIntentDetour_.reset();
      ++motionIntentDetourCandidate_;
    } else if (motionIntentReferenceFallback_) {
      // The original reference has now failed both initializers. Retain that
      // result while sampling the shorter detours on later timer callbacks.
      motionIntentDetourCandidate_ = 0;
      motionIntentShortDetours_ = true;
    }
    motionIntentAdvancePending_ = false;
    motionIntentReferenceFallback_ = false;
  }
  if (input.executionFrozen.has_value()) {
    executionFrozen_ = *input.executionFrozen;
  }
  // In upstream, target subscribers update the target only. Trajectory time is
  // shifted exclusively by the 100 Hz FSM and 20 Hz collision timer callbacks.
  if (!input.goal.has_value() && !input.referencePath.has_value()) {
    updateLocalTrajTimeFreeze(input.nowS);
  }
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

void PlannerFsm::acceptTargetInput(const FsmInput &input, FsmOutput &output) {
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

void PlannerFsm::updateLocalTrajTimeFreeze(double nowS) {
  if (!freezeClockReady_) {
    lastFreezeUpdateTimeS_ = nowS;
    freezeClockReady_ = true;
    return;
  }
  const double delta = nowS - lastFreezeUpdateTimeS_;
  lastFreezeUpdateTimeS_ = nowS;
  if (delta <= 0.0) {
    return;
  }

  LocalTrajData &info = plannerManager_.local_data_;
  if (executionFrozen_ && info.traj_id_ > 0) {
    info.start_time_ += delta;
  }
}

void PlannerFsm::changeState(ScanReplanState newState) {
  continuouslyCalledTimes_ = newState == state_ ? continuouslyCalledTimes_ + 1 : 1;
  state_ = newState;
}

void PlannerFsm::finishProcess() {
  if (replanFailCount_ < params_.maxReplanFailCount) {
    return;
  }
  replanFailCount_ = 0;
  needHoverStop_ = true;
  escapeEmergency_ = true;
  changeState(ScanReplanState::EMERGENCY_STOP);
}

bool PlannerFsm::planManualTarget(const Eigen::Vector3d &goal, double nowS, FsmOutput &) {
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

bool PlannerFsm::planReferencePath(const std::vector<Eigen::Vector3d> &path, double nowS,
                                      FsmOutput &) {
  if (path.empty()) {
    return false;
  }

  trigger_ = true;
  endPt_ = path.back();
  endPt_(2) += params_.bodyHeight;

  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(path.size());
  constexpr double referenceSpacing = ScanPlannerParams::kMinReferenceWaypointDistanceM;
  for (const Eigen::Vector3d &pathPoint : path) {
    Eigen::Vector3d waypoint = pathPoint;
    waypoint(2) += params_.bodyHeight;
    if (waypoints.empty()) {
      waypoints.push_back(waypoint);
      continue;
    }
    const Eigen::Vector3d previous = waypoints.back();
    const double distance = (waypoint - previous).norm();
    if (distance <= 1e-6) {
      continue;
    }
    // Keep every supplied bend and height change. A few widely spaced knots
    // let the global polynomial anticipate distant turns across a straight
    // approach; sample that same 3-D guide at the existing reference spacing.
    const int intervals = std::max(1, static_cast<int>(std::ceil(distance / referenceSpacing)));
    for (int index = 1; index < intervals; ++index) {
      waypoints.push_back(previous + (waypoint - previous) * index / intervals);
    }
    waypoints.push_back(waypoint);
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

bool PlannerFsm::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                                              double nowS) {
  if (waypoints.size() < 2) {
    return false;
  }

  endPt_ = waypoints.back();
  const std::vector<Eigen::Vector3d> referenceWaypoints(waypoints.begin() + 1, waypoints.end());
  if (!plannerManager_.planGlobalTrajWaypoints(
          waypoints.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), referenceWaypoints,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), nowS)) {
    return false;
  }

  // Keep the immutable reference endpoint. getLocalTarget handles temporary
  // occupancy without shortening the reference permanently after a fallback.
  resetMotionIntentDetourSearch();

  endVel_.setZero();
  if (plannerManager_.pp_.motion_intent_) {
    // A rolling operator guide describes continued travel, not an arrival.
    // Keep its geometric parameterization separate from the local boundary speed.
    endVel_ = (waypoints.back() - waypoints[waypoints.size() - 2]).normalized() *
              plannerManager_.pp_.max_vel_;
  }
  haveTarget_ = true;
  haveNewTarget_ = true;
  return true;
}

bool PlannerFsm::planNextWaypoint(double nowS) {
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

bool PlannerFsm::adjustGlobalTargetIfOccupied() {
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

bool PlannerFsm::isWaypointSequenceMode() const noexcept {
  return params_.navigationMode == ScanNavigationMode::PRESET_TARGET;
}

bool PlannerFsm::trajectoryClear(const LocalTrajData &trajectory, double beginS,
                                    double endS) const {
  if (!plannerManager_.grid_map_) return false;
  for (double time = std::max(0.0, beginS); time < endS; time += kTickPeriodS) {
    const double next = std::min(time + kTickPeriodS, endS);
    const Eigen::Vector3d position = trajectory.position_traj_.evaluateDeBoorT(time);
    const Eigen::Vector3d nextPosition = trajectory.position_traj_.evaluateDeBoorT(next);
    if (plannerManager_.grid_map_->getInflateOccupancy(
            position, estimateYawFromSegment(position, nextPosition)) != 0)
      return false;
  }
  return true;
}

bool PlannerFsm::preferExecutingTail(const LocalTrajData &previous,
                                       const LocalTrajData *candidate,
                                       double boundaryTime, double nowS) const {
  if (referenceReplacementPending_ || executionFrozen_ || localTargetBlocked_ ||
      !plannerManager_.grid_map_) return false;
  const double currentTime = std::max(0.0, nowS - previous.start_time_);
  if (currentTime >= previous.duration_ - kTickPeriodS) return false;
  const Eigen::Vector3d endpoint = previous.position_traj_.evaluateDeBoorT(previous.duration_);
  const Eigen::Vector3d terminalVelocity = previous.velocity_traj_.evaluateDeBoorT(previous.duration_);
  const Eigen::Vector3d nextEndpoint = candidate
      ? Eigen::Vector3d(candidate->position_traj_.evaluateDeBoorT(candidate->duration_)) : localTargetPt_;
  const Eigen::Vector3d nextTerminalVelocity = candidate
      ? Eigen::Vector3d(candidate->velocity_traj_.evaluateDeBoorT(candidate->duration_)) : localTargetVel_;
  // Compare the same horizon and endpoint. A shorter old horizon must not beat
  // a candidate that extends the route merely because it takes less time.
  if ((endpoint - nextEndpoint).norm() > 1e-2 ||
      (terminalVelocity - nextTerminalVelocity).norm() > 1e-2 ||
      (candidate && candidate->duration_ < .9 * (previous.duration_ - boundaryTime)))
    return false;
  const Eigen::Vector3d position = previous.position_traj_.evaluateDeBoorT(currentTime);
  if ((position - odomPos_).norm() > std::max(0.0, params_.replanThreshold) ||
      plannerManager_.grid_map_->getInflateOccupancySegment(
          odomPos_, getOdomYaw(), position, getOdomYaw()) != 0) return false;
  return trajectoryClear(previous, currentTime, previous.duration_);
}

bool PlannerFsm::reconcileHandoff(const FsmInput &input, FsmOutput &output) {
  if (!pendingTrajectory_) return false;
  const double nowS = plannerManager_.currentTimeS(input.nowS);
  if (nowS > pendingHandoff_->deadlineS) pendingHandoff_->withdraw();
  const auto state = pendingHandoff_->state.load();
  if (state == TrajectoryHandoff::State::Received) {
    plannerManager_.local_data_ = std::move(*pendingTrajectory_);
    speedTransitionEndS_ = 0.0;
    referenceReplacementPending_ = false;
    retainedReplanPosition_.reset();
    pendingTrajectory_.reset();
    pendingHandoff_.reset();
    setTrajectoryOutput(output);
    changeState(ScanReplanState::EXEC_TRAJ);
    return true;
  }
  if (state == TrajectoryHandoff::State::Withdrawn) {
    output.handoffMissed = nowS > pendingHandoff_->deadlineS;
    pendingTrajectory_.reset();
    pendingHandoff_.reset();
    changeState(ScanReplanState::REPLAN_TRAJ);
    return true;
  }
  return false;
}

void PlannerFsm::withdrawHandoff(const FsmInput &input, FsmOutput &output) {
  if (!pendingHandoff_) return;
  pendingHandoff_->withdraw();
  reconcileHandoff(input, output);
}

void PlannerFsm::offerHandoff(FsmOutput &output) {
  output.handoffPending = true;
  output.handoff = pendingHandoff_;
  setTrajectoryOutput(output, *pendingTrajectory_);
}

bool PlannerFsm::trackingReferenceSeparated(const LocalTrajData &trajectory,
                                               double elapsedS) const {
  // Beyond one control-point spacing, the nominal boundary no longer describes
  // the observed motion; this requests a new reference, not a motion veto.
  return !executionFrozen_ &&
      (trajectory.position_traj_.evaluateDeBoorT(elapsedS) - odomPos_).norm() >
          std::max(plannerManager_.pp_.ctrl_pt_dist, plannerManager_.grid_map_->getResolution());
}

bool PlannerFsm::planFromCurrentTraj(double nowS, FsmOutput &output,
                                        std::optional<double> latestHandoffS) {
  LocalTrajData &info = plannerManager_.local_data_;
  const double currentTime = std::min(std::max(nowS - info.start_time_, 0.0), info.duration_);

  // A normal SCAN solve must start after the explicit deceleration, where its
  // unchanged velocity limit applies. Wait for physical reception before
  // offering a future replacement of this moving transition.
  if (speedTransitionEndS_ > 0.0 && currentTime < speedTransitionEndS_ &&
      (!handoffLeadTimeS_ || executionFrozen_))
    return !latestHandoffS.has_value();

  if (params_.navigationMode == ScanNavigationMode::REFERENCE_PATH) {
    if (handoffLeadTimeS_ && !executionFrozen_ && info.traj_id_ > 0 &&
        currentTime < info.duration_ - 1e-2 && !trackingReferenceSeparated(info, currentTime)) {
      const LocalTrajData previous = info;
      double handoff = std::min(nowS + *handoffLeadTimeS_,
                                 previous.start_time_ + previous.duration_);
      if (speedTransitionEndS_ > 0.0)
        handoff = std::max(handoff, previous.start_time_ + speedTransitionEndS_);
      if (latestHandoffS) handoff = std::min(handoff, *latestHandoffS);
      if (speedTransitionEndS_ > 0.0 &&
          handoff < previous.start_time_ + speedTransitionEndS_)
        return false;
      if (handoff <= nowS ||
          !trajectoryClear(previous, currentTime, handoff - previous.start_time_))
        return false;
      const double boundaryTime = handoff - previous.start_time_;
      startPt_ = previous.position_traj_.evaluateDeBoorT(boundaryTime);
      startVel_ = previous.velocity_traj_.evaluateDeBoorT(boundaryTime);
      startAcc_ = previous.acceleration_traj_.evaluateDeBoorT(boundaryTime);
      // Optimize and validate the actual future boundary, including prediction
      // arrival times. No control points are moved after collision validation.
      const bool preserveBoundary = plannerManager_.pp_.preserve_boundary_;
      plannerManager_.pp_.preserve_boundary_ = true;
      bool success = callReboundReplan(true, false, nowS, output);
      if (!success && plannerManager_.currentTimeS(nowS) < handoff)
        success = callReboundReplan(true, true, nowS, output);
      plannerManager_.pp_.preserve_boundary_ = preserveBoundary;
      if (!latestHandoffS && preferExecutingTail(previous, success ? &info : nullptr,
                                                 boundaryTime, plannerManager_.currentTimeS(nowS))) {
        info = previous;
        output.trajectory.reset();
        retainedReplanPosition_ = previous.position_traj_.evaluateDeBoorT(currentTime);
        return true;
      }
      if (success && plannerManager_.currentTimeS(nowS) + 2.0 * kTickPeriodS < handoff) {
        pendingTrajectory_ = info;
        pendingTrajectory_->start_time_ = handoff;
        pendingHandoff_ = std::make_shared<TrajectoryHandoff>(handoff + kHandoffLatenessS);
        output.handoffPending = true;
      } else if (success) {
        // A missed deadline must not replace the still executing trajectory.
        output.handoffMissed = true;
      }
      info = previous;
      output.trajectory.reset();
      if (pendingTrajectory_) offerHandoff(output);
      return success && (!output.handoffMissed || !latestHandoffS);
    }
    // Route following and held operator intent can both lag the timed spline.
    // Rebuild from observed motion rather than its nominal start and speed.
    startPt_ = odomPos_;
    startVel_ = odomVel_;
    startAcc_.setZero();
    if (callReboundReplan(true, false, nowS, output)) {
      referenceReplacementPending_ = false;
      speedTransitionEndS_ = 0.0;
      return true;
    }
    const bool success = callReboundReplan(true, true, nowS, output);
    if (success) {
      referenceReplacementPending_ = false;
      speedTransitionEndS_ = 0.0;
    }
    return success;
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

void PlannerFsm::setStartStateFromOdomOrCurrentTraj(double nowS) {
  startPt_ = odomPos_;
  startVel_ = odomVel_;
  startAcc_.setZero();

  // A blocked target has already stopped execution at the adapter boundary.
  // Resume from observed motion, not the derivatives of the abandoned spline.
  if (localTargetBlocked_) return;

  LocalTrajData &info = plannerManager_.local_data_;
  if (info.start_time_ < 1e-5 || info.duration_ <= 1e-5) {
    return;
  }
  const double rawCurrentTime = nowS - info.start_time_;
  // Once a held-intent segment has finished, its terminal derivatives no
  // longer describe a lagging robot. Continue from measured position/velocity.
  if (plannerManager_.pp_.motion_intent_ && rawCurrentTime > info.duration_ - 1e-2) {
    return;
  }
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

bool PlannerFsm::callReboundReplan(bool usePolyInit, bool randomPolyTraj, double nowS,
                                      FsmOutput &output) {
  const Eigen::Vector3d previousLocalTarget = localTargetPt_;
  if (!getLocalTarget()) {
    resetMotionIntentDetourSearch();
    return false;
  }
  const bool motionIntentTargetChanged = plannerManager_.pp_.motion_intent_ &&
      (localTargetPt_ - previousLocalTarget).norm() > 1e-6;
  const bool freshMotionIntentTarget = plannerManager_.pp_.motion_intent_ &&
      (haveNewTarget_ || motionIntentTargetChanged);
  const bool reboundRandomPolyTraj =
      freshMotionIntentTarget ? false : randomPolyTraj;
  const bool success =
      plannerManager_.reboundReplan(startPt_, startVel_, startAcc_, localTargetPt_, localTargetVel_,
                                    haveNewTarget_ || usePolyInit || freshMotionIntentTarget,
                                    reboundRandomPolyTraj, nowS, localReferenceInterval_);
  haveNewTarget_ = false;
  if (success) {
    initializationFailed_ = false;
    retainedReplanPosition_.reset();
    setTrajectoryOutput(output);
    motionIntentAdvancePending_ = false;
    if (motionIntentReferenceFallback_) resetMotionIntentDetourSearch();
  } else if (state_ == ScanReplanState::GEN_NEW_TRAJ) {
    initializationFailed_ = true;
  }
  if (!success && reboundRandomPolyTraj && plannerManager_.pp_.motion_intent_ &&
      (motionIntentDetour_ || motionIntentReferenceFallback_)) {
    // Advance only after this target has received its deterministic and random
    // initializers. GEN_NEW_TRAJ spreads them across timer callbacks.
    motionIntentAdvancePending_ = true;
  }
  return success;
}

bool PlannerFsm::callEmergencyStop(const Eigen::Vector3d &stopPos, double nowS,
                                      FsmOutput &output) {
  const bool success = plannerManager_.EmergencyStop(stopPos, nowS);
  if (success) {
    output.emergencyStopIssued = true;
    setTrajectoryOutput(output);
  }
  return success;
}

bool PlannerFsm::getLocalTarget() {
  localReferenceInterval_.reset();
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
  Eigen::Vector3d previousPosition = plannerManager_.global_data_.getPosition(projectionTime);
  localTargetPt_ = endPt_;
  for (double time = projectionTime; time < duration; time += timeStep) {
    const Eigen::Vector3d position = plannerManager_.global_data_.getPosition(time);
    totalDistance += (position - previousPosition).norm();
    if (totalDistance >= params_.planningHorizon) {
      localTargetPt_ = position;
      targetTime = time;
      break;
    }
    previousPosition = position;
  }
  const auto targetOccupancy = [&](const Eigen::Vector3d &point) {
    return plannerManager_.grid_map_->getInflateOccupancy(
        point, estimateYawFromSegment(odomPos_, point));
  };
  const auto usableTarget = [&](const Eigen::Vector3d &point, double time) {
    // A short clear route prefix is useful progress; velocity intents retain
    // their detour spacing and final-approach behavior.
    const double minimumProgress = plannerManager_.pp_.motion_intent_ ? 0.2 : 1e-3;
    const bool finalApproach = plannerManager_.pp_.motion_intent_ &&
                               (point - endPt_).norm() < 1e-6;
    return time >= projectionTime &&
           ((point - startPt_).norm() >= minimumProgress || finalApproach) &&
           targetOccupancy(point) == 0;
  };
  const bool referenceClear = usableTarget(localTargetPt_, targetTime) &&
      (!plannerManager_.pp_.motion_intent_ ||
        plannerManager_.grid_map_->getInflateOccupancySegment(
            startPt_, getOdomYaw(), localTargetPt_, getOdomYaw()) == 0);
  if (referenceClear) resetMotionIntentDetourSearch();
  if (motionIntentDetour_) {
    // The 0.2 m floor admits new candidates; it is not arrival at a selected
    // target. A lagging robot can still need the last 0.1-0.2 m of this exit.
    if (!referenceClear &&
        (*motionIntentDetour_ - odomPos_).head<2>().norm() >
            std::max(0.01, params_.noReplanThreshold) &&
        usableMotionIntentDetour(*motionIntentDetour_, 0.0)) {
      localTargetPt_ = *motionIntentDetour_;
      localTargetVel_.setZero();
      localTargetBlocked_ = false;
      motionIntentReferenceFallback_ = false;
      return true;
    }
    resetMotionIntentDetourSearch();
  }
  // A free endpoint can still lie behind a shelf. A velocity intent may use
  // a clear prefix or side target before retrying that straight guide.
  // RouteTarget callers keep the reference-only behavior below.
  if (!referenceClear && selectMotionIntentDetour()) {
    localTargetPt_ = *motionIntentDetour_;
    localTargetVel_.setZero();
    localTargetBlocked_ = false;
    motionIntentReferenceFallback_ = false;
    return true;
  }
  motionIntentReferenceFallback_ =
      !referenceClear && plannerManager_.pp_.motion_intent_;
  const bool truncatedAtObstacle = !usableTarget(localTargetPt_, targetTime);
  if (truncatedAtObstacle) {
    bool foundFreeTarget = false;
    double adjustedTime = targetTime;
    for (double delta = 0.0; delta <= plannerManager_.global_data_.global_duration_;
         delta += timeStep) {
      const double forwardTime = targetTime + delta;
      if (forwardTime <= plannerManager_.global_data_.global_duration_) {
        const Eigen::Vector3d point = plannerManager_.global_data_.getPosition(forwardTime);
        if (usableTarget(point, forwardTime)) {
          localTargetPt_ = point;
          adjustedTime = forwardTime;
          foundFreeTarget = true;
          break;
        }
      }

      const double backwardTime = targetTime - delta;
      if (backwardTime >= std::max(0.0, projectionTime)) {
        const Eigen::Vector3d point = plannerManager_.global_data_.getPosition(backwardTime);
        if (usableTarget(point, backwardTime)) {
          localTargetPt_ = point;
          adjustedTime = backwardTime;
          foundFreeTarget = true;
          break;
        }
      }
    }
    if (foundFreeTarget) {
      targetTime = adjustedTime;
    } else {
      // No usable reference endpoint exists, so there is no reference
      // trajectory to optimize before trying the short side candidates.
      motionIntentShortDetours_ = true;
      if (selectMotionIntentDetour()) {
        localTargetPt_ = *motionIntentDetour_;
        localTargetVel_.setZero();
        localTargetBlocked_ = false;
        motionIntentReferenceFallback_ = false;
        return true;
      }
      localTargetBlocked_ = true;
      resetMotionIntentDetourSearch();
      return false;
    }
  }

  localTargetBlocked_ = false;
  plannerManager_.global_data_.last_progress_time_ = targetTime;
  if (params_.navigationMode == ScanNavigationMode::REFERENCE_PATH &&
      !plannerManager_.pp_.motion_intent_)
    localReferenceInterval_ = std::make_pair(projectionTime,targetTime);

  if (truncatedAtObstacle) {
    // The reference beyond this endpoint is blocked, so its nominal velocity
    // cannot be carried through the end of the admitted clear prefix.
    localTargetVel_.setZero();
  } else if (plannerManager_.pp_.motion_intent_ && referenceClear) {
    const Eigen::Vector3d tangent = plannerManager_.global_data_.getVelocity(targetTime);
    localTargetVel_ = tangent.squaredNorm() > 1e-12
                          ? Eigen::Vector3d(tangent.normalized() * maxVelocity)
                          : endVel_;
  } else if ((endPt_ - localTargetPt_).norm() <
             maxVelocity * maxVelocity / (2.0 * maxAcceleration)) {
    localTargetVel_.setZero();
  } else {
    localTargetVel_ = plannerManager_.global_data_.getVelocity(targetTime);
    if (localTargetVel_.norm() > maxVelocity) {
      localTargetVel_ = localTargetVel_.normalized() * maxVelocity;
    }
  }
  localTargetVel_ /= std::max(1.0, plannerManager_.pp_.speedRatio(localTargetVel_));
  return true;
}

bool PlannerFsm::usableMotionIntentDetour(const Eigen::Vector3d &point,
                                           double minimumDistance) const {
  if (!plannerManager_.pp_.motion_intent_ || !motionIntentMaxDeviationRad_ ||
      *motionIntentMaxDeviationRad_ <= 0.0) return false;
  const Eigen::Vector2d direction =
      (endPt_ - plannerManager_.global_data_.getPosition(0.0)).head<2>();
  const Eigen::Vector2d offset = (point - startPt_).head<2>();
  if (direction.squaredNorm() < 1e-8 || offset.norm() + 1e-9 < minimumDistance ||
      offset.norm() > params_.planningHorizon + 1e-6) return false;
  const double angle = std::abs(std::atan2(
      direction.x() * offset.y() - direction.y() * offset.x(), direction.dot(offset)));
  if (angle > *motionIntentMaxDeviationRad_ + 1e-6) return false;
  return plannerManager_.grid_map_->getInflateOccupancySegment(
             startPt_, getOdomYaw(), point, getOdomYaw()) == 0;
}

bool PlannerFsm::selectMotionIntentDetour() {
  if (!plannerManager_.pp_.motion_intent_ || !motionIntentMaxDeviationRad_ ||
      *motionIntentMaxDeviationRad_ <= 0.0) return false;
  const Eigen::Vector2d direction =
      (endPt_ - plannerManager_.global_data_.getPosition(0.0)).head<2>();
  if (direction.squaredNorm() < 1e-8) return false;
  const double heading = std::atan2(direction.y(), direction.x());
  const double maxAngle = *motionIntentMaxDeviationRad_;
  const int angleSteps = static_cast<int>(std::ceil(maxAngle / (M_PI / 6.0)));
  constexpr double minimumDistance = nav_kernel::ScanPlannerParams::kMinReferenceWaypointDistanceM;
  std::size_t candidate = 0;
  const auto trySideCandidates = [&](double distance) {
    for (int step = 1; step <= angleSteps; ++step) {
      for (const double side : {1.0, -1.0}) {
        const double yaw = heading + side * maxAngle * step / angleSteps;
        const Eigen::Vector3d point =
            startPt_ + Eigen::Vector3d(distance * std::cos(yaw), distance * std::sin(yaw), 0.0);
        const std::size_t slot = candidate++;
        if (slot < motionIntentDetourCandidate_) continue;
        if (usableMotionIntentDetour(point)) {
          motionIntentDetourCandidate_ = slot;
          motionIntentDetour_ = point;
          return true;
        }
      }
    }
    return false;
  };
  // Use available progress along the operator's direction before selecting
  // a longer side detour. A blocked far guide does not block its near prefix.
  for (double distance = params_.planningHorizon; distance >= minimumDistance;
       distance = std::max(minimumDistance, distance * 0.5)) {
    const Eigen::Vector3d point =
        startPt_ + Eigen::Vector3d(distance * std::cos(heading), distance * std::sin(heading), 0.0);
    const std::size_t slot = candidate++;
    if (slot >= motionIntentDetourCandidate_ && usableMotionIntentDetour(point)) {
      motionIntentDetourCandidate_ = slot;
      motionIntentDetour_ = point;
      return true;
    }
    if (distance == minimumDistance) break;
  }
  // Keep the selected target across replans until reached or invalidated.
  for (double distance = params_.planningHorizon; distance >= minimumDistance;
       distance = std::max(minimumDistance, distance * 0.5)) {
    if (trySideCandidates(distance)) return true;
    if (distance == minimumDistance) break;
  }
  // Give the original reference optimizer its deterministic/random attempts
  // before adding shorter candidates that could change that existing result.
  if (!motionIntentShortDetours_) return false;
  // In tight spaces, cover the remaining distances at 0.1 m intervals down
  // to usableMotionIntentDetour's 0.2 m lower limit. Halving alone skips them.
  for (const double distance : {0.4, 0.3, 0.2}) {
    if (trySideCandidates(distance)) return true;
  }
  return false;
}

void PlannerFsm::resetMotionIntentDetourSearch() {
  motionIntentDetour_.reset();
  motionIntentDetourCandidate_ = 0;
  motionIntentAdvancePending_ = false;
  motionIntentReferenceFallback_ = false;
  motionIntentShortDetours_ = false;
}

void PlannerFsm::setTrajectoryOutput(FsmOutput &output) const {
  setTrajectoryOutput(output, plannerManager_.local_data_);
}

void PlannerFsm::setTrajectoryOutput(FsmOutput &output, const LocalTrajData &info) const {
  BsplineTrajectory trajectory;
  trajectory.order = 3;
  trajectory.startTimeS = info.start_time_;
  trajectory.trajectoryId = info.traj_id_;
  trajectory.positionPoints = info.position_traj_.getControlPoint();
  trajectory.knots = info.position_traj_.getKnot();
  output.trajectory = std::move(trajectory);
}

double PlannerFsm::getOdomYaw() const {
  const Eigen::Vector3d heading = odomOrientation_.toRotationMatrix().col(0);
  if (heading.head<2>().squaredNorm() < 1e-8) {
    return 0.0;
  }
  return std::atan2(heading(1), heading(0));
}

double PlannerFsm::estimateYawFromSegment(const Eigen::Vector3d &from,
                                             const Eigen::Vector3d &to) const {
  const Eigen::Vector2d difference(to(0) - from(0), to(1) - from(1));
  if (difference.squaredNorm() < 1e-8) {
    return getOdomYaw();
  }
  return std::atan2(difference(1), difference(0));
}

FsmOutput PlannerFsm::finalizeOutput(FsmOutput output, ScanReplanState initialState) const {
  output.localTargetBlocked = localTargetBlocked_;
  output.initializationFailed = initializationFailed_;
  output.state = state_;
  output.stateChanged = state_ != initialState;
  return output;
}

}  // namespace nav_kernel::local::scan::upstream
