#pragma once

// Ported from SCAN-Planner plan_manage/planner_manager.h at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS setup, time and visualization are replaced by explicit values.
// SPDX-License-Identifier: Apache-2.0

#include <functional>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "planning/local/scan/upstream/bspline_opt/bspline_optimizer.h"
#include "planning/local/scan/upstream/plan_env/grid_map.h"
#include "planning/local/scan/upstream/plan_manage/plan_container.hpp"

namespace nav_kernel::local::scan::upstream {

struct ReboundPlanDebug : OptimizationDebug {
  std::uint64_t attemptId{0};
  bool attempted{false};
  bool success{false};
  std::string stage{"not_run"};
  Eigen::Vector3d startPosition{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startVelocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startAcceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d targetPosition{Eigen::Vector3d::Zero()};
  Eigen::Vector3d targetVelocity{Eigen::Vector3d::Zero()};
  bool polyInit{false};
  bool randomPolyInit{false};
  // Failure-only candidate; empty if initialization produced no B-spline.
  Eigen::MatrixXd candidateControlPoints;
  double candidateIntervalS{0.0};
  bool dynamicViolationValid{false};
  std::string dynamicQuantity;
  double dynamicValue{0.0};
  double dynamicLimit{0.0};
  double dynamicTimeS{0.0};
};

class SCANPlannerManager {
 public:
  SCANPlannerManager() = default;
  ~SCANPlannerManager() = default;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void initPlanModules(const PlanParameters &planParams,
                       const BsplineOptimizerParams &optimizerParams,
                       GridMap::Ptr gridMap);
  void updatePlanParameters(const PlanParameters &planParams,
                            const BsplineOptimizerParams &optimizerParams);
  void setTimeSource(std::function<double()> timeSource);

  bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                     Eigen::Vector3d start_acc,
                     Eigen::Vector3d local_target_pt,
                     Eigen::Vector3d local_target_vel, bool flag_polyInit,
                     bool flag_randomPolyTraj, double nowS);
  bool EmergencyStop(Eigen::Vector3d stop_pos, double nowS);
  // One reboundReplan call, retained until the next call, including success.
  const ReboundPlanDebug &reboundDebug() const noexcept { return rebound_debug_; }
  bool planGlobalTraj(const Eigen::Vector3d &start_pos,
                      const Eigen::Vector3d &start_vel,
                      const Eigen::Vector3d &start_acc,
                      const Eigen::Vector3d &end_pos,
                      const Eigen::Vector3d &end_vel,
                      const Eigen::Vector3d &end_acc, double nowS);
  bool planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc,
      const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
      double nowS);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  GridMap::Ptr grid_map_;

  using Ptr = std::unique_ptr<SCANPlannerManager>;

 private:
  BsplineOptimizer::Ptr bspline_optimizer_rebound_;
  std::function<double()> timeSource_;
  int continuous_failures_count_{0};
  ReboundPlanDebug rebound_debug_;

  double currentTimeS(double fallback) const;
  void updateTrajInfo(const UniformBspline &position_traj, double nowS);
  bool checkDynamicFeasibility(UniformBspline position_traj);
  void reparamBspline(UniformBspline &bspline,
                      std::vector<Eigen::Vector3d> &start_end_derivative,
                      double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                      double &time_inc);
  bool refineTrajAlgo(UniformBspline &traj,
                      std::vector<Eigen::Vector3d> &start_end_derivative,
                      double ratio, double &ts,
                      Eigen::MatrixXd &optimal_control_points);
};

}  // namespace nav_kernel::local::scan::upstream
