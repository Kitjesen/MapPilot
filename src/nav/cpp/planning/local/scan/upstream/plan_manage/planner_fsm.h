#pragma once

// Ported from SCAN-Planner plan_manage/scan_replan_fsm.{h,cpp} at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS callbacks, timers and publishers are replaced by value inputs/outputs.
// SPDX-License-Identifier: Apache-2.0

// PlannerFsm 接口：局部轨迹生命周期的状态、输入输出与调度入口。
// 外部向 tick() 提交目标/参考和执行反馈，向 checkFutureCollision() 提交实时状态；
// 两者返回 FsmOutput，包含状态、可选样条及接管/停止标志。
// 六状态：初始化、等待目标、生成轨迹、重规划、执行轨迹、紧急停止。
// 轨迹求解由 SCANPlannerManager 组织，速度控制与最终运动仲裁由后续组件负责。
// 完整函数分组与推荐阅读顺序见 planner_fsm.cpp 文件开头。

#include <Eigen/Eigen>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"
#include "planning/local/scan/upstream/plan_manage/planner_manager.h"

namespace nav_kernel::local::scan::upstream {

enum class ScanReplanState {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  REPLAN_TRAJ,
  EXEC_TRAJ,
  EMERGENCY_STOP,
};

enum class ScanNavigationMode {
  MANUAL_TARGET = 1,
  PRESET_TARGET = 2,
  REFERENCE_PATH = 3,
};

struct ScanReplanParams {
  ScanNavigationMode navigationMode{ScanNavigationMode::MANUAL_TARGET};
  double noReplanThreshold{-1.0};
  double replanThreshold{-1.0};
  double planningHorizon{-1.0};
  double emergencyTimeS{1.0};
  bool enableFailSafe{true};
  int maxReplanFailCount{1000};
  double bodyHeight{0.0};
  std::vector<Eigen::Vector3d> presetWaypoints{};
};

struct FsmOdometry {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct FsmInput {
  double nowS{0.0};
  std::int64_t executingTrajectoryId{0};
  std::optional<double> executionStartTimeS{};
  std::optional<double> handoffLeadTimeS{};
  std::optional<FsmOdometry> odometry{};
  std::optional<bool> executionFrozen{};
  std::optional<double> motionIntentMaxDeviationRad{};
  std::optional<Eigen::Vector3d> goal{};
  std::optional<std::vector<Eigen::Vector3d>> referencePath{};
};

struct FsmOutput {
  ScanReplanState state{ScanReplanState::INIT};
  bool stateChanged{false};
  bool targetAccepted{false};
  bool targetRejected{false};
  bool targetFinished{false};
  bool collisionDetected{false};
  double collisionTimeAheadS{0.0};
  bool emergencyStopIssued{false};
  bool localTargetBlocked{false};
  bool initializationFailed{false};
  bool handoffPending{false};
  bool handoffMissed{false};
  std::shared_ptr<TrajectoryHandoff> handoff{};
  std::optional<BsplineTrajectory> trajectory{};
};

class PlannerFsm {
 public:
  static constexpr double kTickPeriodS{0.01};
  static constexpr double kFutureCollisionPeriodS{0.05};
  static constexpr double kHandoffLatenessS{2.0 * kTickPeriodS};

  PlannerFsm(SCANPlannerManager &plannerManager, ScanReplanParams params = {});

  FsmOutput tick(const FsmInput &input);
  FsmOutput checkFutureCollision(const FsmInput &input);
  void adoptSpeedTransition(double transitionEndS);
  void abandonTrajectory();

  [[nodiscard]] ScanReplanState state() const noexcept;
  [[nodiscard]] bool hasTarget() const noexcept;
  [[nodiscard]] const Eigen::Vector3d &localTarget() const noexcept {
    return localTargetPt_;
  }
  [[nodiscard]] bool localTargetReady() const noexcept {
    return haveTarget_ && !haveNewTarget_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void updateRuntimeInput(const FsmInput &input);
  void acceptTargetInput(const FsmInput &input, FsmOutput &output);
  void updateLocalTrajTimeFreeze(double nowS);
  void changeState(ScanReplanState newState);
  void finishProcess();

  bool planManualTarget(const Eigen::Vector3d &goal, double nowS, FsmOutput &output);
  bool planReferencePath(const std::vector<Eigen::Vector3d> &path, double nowS, FsmOutput &output);
  bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints, double nowS);
  bool planNextWaypoint(double nowS);
  bool adjustGlobalTargetIfOccupied();
  bool isWaypointSequenceMode() const noexcept;

  bool planFromCurrentTraj(double nowS, FsmOutput &output,
                           std::optional<double> latestHandoffS = std::nullopt);
  bool trackingReferenceSeparated(const LocalTrajData &trajectory, double elapsedS) const;
  bool trajectoryClear(const LocalTrajData &trajectory, double beginS,
                        double endS) const;
  bool preferExecutingTail(const LocalTrajData &previous, const LocalTrajData *candidate,
                           double boundaryTime, double nowS) const;
  bool reconcileHandoff(const FsmInput &input, FsmOutput &output);
  void withdrawHandoff(const FsmInput &input, FsmOutput &output);
  void offerHandoff(FsmOutput &output);
  void setStartStateFromOdomOrCurrentTraj(double nowS);
  bool callReboundReplan(bool usePolyInit, bool randomPolyTraj, double nowS, FsmOutput &output);
  bool callEmergencyStop(const Eigen::Vector3d &stopPos, double nowS, FsmOutput &output);
  bool getLocalTarget();
  bool usableMotionIntentDetour(const Eigen::Vector3d &point, double minimumDistance = 0.2) const;
  bool selectMotionIntentDetour();
  void resetMotionIntentDetourSearch();
  void setTrajectoryOutput(FsmOutput &output) const;
  void setTrajectoryOutput(FsmOutput &output, const LocalTrajData &info) const;

  [[nodiscard]] double getOdomYaw() const;
  [[nodiscard]] double estimateYawFromSegment(const Eigen::Vector3d &from,
                                              const Eigen::Vector3d &to) const;
  [[nodiscard]] FsmOutput finalizeOutput(FsmOutput output, ScanReplanState initialState) const;

  SCANPlannerManager &plannerManager_;
  bool initializationFailed_{false};
  ScanReplanParams params_;

  bool trigger_{false};
  bool haveTarget_{false};
  bool haveOdom_{false};
  bool haveNewTarget_{false};
  bool referenceReplacementPending_{false};
  bool manualGoalHeightReady_{false};
  bool executionFrozen_{false};
  bool escapeEmergency_{true};
  bool needHoverStop_{false};
  bool presetTriggered_{false};
  bool localTargetBlocked_{false};
  ScanReplanState state_{ScanReplanState::INIT};
  int continuouslyCalledTimes_{0};
  int replanFailCount_{0};
  double lastFreezeUpdateTimeS_{0.0};
  bool freezeClockReady_{false};
  std::int64_t acknowledgedTrajectoryId_{0};
  std::optional<double> handoffLeadTimeS_{};
  double speedTransitionEndS_{0.0};
  std::optional<LocalTrajData> pendingTrajectory_{};
  std::shared_ptr<TrajectoryHandoff> pendingHandoff_{};
  std::optional<Eigen::Vector3d> retainedReplanPosition_{};

  Eigen::Vector3d odomPos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odomVel_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond odomOrientation_{Eigen::Quaterniond::Identity()};

  Eigen::Vector3d startPt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startVel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startAcc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d endPt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d endVel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d localTargetPt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d localTargetVel_{Eigen::Vector3d::Zero()};
  std::optional<std::pair<double,double>> localReferenceInterval_{};
  std::optional<double> motionIntentMaxDeviationRad_{};
  std::optional<Eigen::Vector3d> motionIntentDetour_{};
  std::size_t motionIntentDetourCandidate_{0};
  bool motionIntentAdvancePending_{false};
  bool motionIntentReferenceFallback_{false};
  bool motionIntentShortDetours_{false};
  double manualGoalHeight_{0.0};

  std::vector<Eigen::Vector3d> activeWaypoints_{};
  int currentWaypoint_{0};
};

}  // namespace nav_kernel::local::scan::upstream
