#pragma once

// Ported from SCAN-Planner plan_manage/scan_replan_fsm.{h,cpp} at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS callbacks, timers and publishers are replaced by value inputs/outputs.
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Eigen>
#include <optional>
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
  std::optional<FsmOdometry> odometry{};
  std::optional<bool> executionFrozen{};
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
  std::optional<BsplineTrajectory> trajectory{};
};

class SCANReplanFSM {
 public:
  static constexpr double kTickPeriodS{0.01};
  static constexpr double kFutureCollisionPeriodS{0.05};

  SCANReplanFSM(SCANPlannerManager &plannerManager, ScanReplanParams params = {});

  FsmOutput tick(const FsmInput &input);
  FsmOutput checkFutureCollision(const FsmInput &input);

  [[nodiscard]] ScanReplanState state() const noexcept;
  [[nodiscard]] bool hasTarget() const noexcept;

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

  bool planFromCurrentTraj(double nowS, FsmOutput &output);
  void setStartStateFromOdomOrCurrentTraj(double nowS);
  bool callReboundReplan(bool usePolyInit, bool randomPolyTraj, double nowS, FsmOutput &output);
  bool callEmergencyStop(const Eigen::Vector3d &stopPos, double nowS, FsmOutput &output);
  void getLocalTarget();
  void setTrajectoryOutput(FsmOutput &output) const;

  [[nodiscard]] double getOdomYaw() const;
  [[nodiscard]] double estimateYawFromSegment(const Eigen::Vector3d &from,
                                              const Eigen::Vector3d &to) const;
  [[nodiscard]] FsmOutput finalizeOutput(FsmOutput output, ScanReplanState initialState) const;

  SCANPlannerManager &plannerManager_;
  ScanReplanParams params_;

  bool trigger_{false};
  bool haveTarget_{false};
  bool haveOdom_{false};
  bool haveNewTarget_{false};
  bool manualGoalHeightReady_{false};
  bool executionFrozen_{false};
  bool escapeEmergency_{true};
  bool needHoverStop_{false};
  bool presetTriggered_{false};
  ScanReplanState state_{ScanReplanState::INIT};
  int continuouslyCalledTimes_{0};
  int replanFailCount_{0};
  double lastFreezeUpdateTimeS_{0.0};

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
  double manualGoalHeight_{0.0};

  std::vector<Eigen::Vector3d> activeWaypoints_{};
  int currentWaypoint_{0};
};

}  // namespace nav_kernel::local::scan::upstream
