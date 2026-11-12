// Trajectory timing and feasibility extracted from LingTu's SCAN manager.
// Upstream base: SCAN-Planner 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// Exact physical boundaries and constrained refinement are local adaptations.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/planner_manager.h"

#include <algorithm>
#include <cmath>

namespace nav_kernel::local::scan::upstream {

void SCANPlannerManager::setSplineBoundary(
    Eigen::MatrixXd &controls, const double interval,
    const Eigen::Vector3d &start, const Eigen::Vector3d &target,
    const std::vector<Eigen::Vector3d> &derivatives) {
  // For a uniform cubic spline with interval h seconds, the three endpoint
  // controls are P - h*V + h^2*A/3, P - h^2*A/6, P + h*V + h^2*A/3.
  // Keep physical P [m], V [m/s], A [m/s^2] when the interval changes.
  const double interval_squared = interval * interval;
  const auto set = [&](const Eigen::Index first, const Eigen::Vector3d &p,
                       const Eigen::Vector3d &v, const Eigen::Vector3d &a) {
    controls.col(first) = p - interval * v + interval_squared * a / 3.0;
    controls.col(first + 1) = p - interval_squared * a / 6.0;
    controls.col(first + 2) = p + interval * v + interval_squared * a / 3.0;
  };
  set(0, start, derivatives[0], derivatives[2]);
  set(controls.cols() - 3, target, derivatives[1], derivatives[3]);
}

double SCANPlannerManager::dynamicTimeRatio(
    const UniformBspline &position, const PlanParameters &params) {
  const UniformBspline velocity = position.getDerivative();
  const UniformBspline acceleration = velocity.getDerivative();
  // Each uniform cubic position span has quadratic velocity Bezier controls
  // (D[i] + D[i+1])/2, D[i+1], (D[i+1] + D[i+2])/2. Interior averages are
  // bounded by the inner derivative controls. Include the actual endpoints;
  // the extrapolated first/last controls can exceed the curve's speed limit.
  const Eigen::MatrixXd velocity_controls = velocity.getControlPoint();
  double velocity_ratio = std::max(
      params.speedRatio(velocity.evaluateDeBoorT(0.0)),
      params.speedRatio(velocity.evaluateDeBoorT(position.getTimeSum())));
  for (Eigen::Index i = 1; i + 1 < velocity_controls.cols(); ++i)
    velocity_ratio = std::max(velocity_ratio, params.speedRatio(velocity_controls.col(i)));
  const double acceleration_bound =
      acceleration.getControlPoint().colwise().norm().maxCoeff();
  return std::max({1.0, velocity_ratio,
                   std::sqrt(acceleration_bound / params.max_acc_)});
}

bool SCANPlannerManager::retime(
    UniformBspline &pos, const Eigen::Vector3d &start_pt,
    const Eigen::Vector3d &local_target_pt,
    std::vector<Eigen::Vector3d> &start_end_derivatives, double &ts,
    bool preserveBoundary) {
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                        pp_.feasibility_tolerance_);

  const auto refine = [&](const double ratio) {
    Eigen::MatrixXd optimal_control_points;
    const bool success = refineTrajAlgo(
        pos, start_end_derivatives, ratio, ts, optimal_control_points);
    static_cast<OptimizationDebug &>(rebound_debug_) =
        bspline_optimizer_rebound_->optimizationDebug();
    rebound_debug_.stage = "refine_optimization";
    if (success) {
      pos = UniformBspline(optimal_control_points, 3, ts);
    } else {
      rebound_debug_.candidateControlPoints = optimal_control_points;
      rebound_debug_.candidateIntervalS = ts;
    }
    return success;
  };

  bool feasible = true;
  if (preserveBoundary) {
    if (start_end_derivatives[2].squaredNorm() == 0.0 &&
        start_end_derivatives[3].squaredNorm() == 0.0 &&
        start_end_derivatives[0].squaredNorm() == 0.0 &&
        start_end_derivatives[1].squaredNorm() == 0.0) {
      // With zero endpoint velocity and acceleration, dilation preserves all
      // boundary controls and scales velocity by 1/h and acceleration by 1/h^2.
      ts *= dynamicTimeRatio(pos, pp_);
      pos = UniformBspline(pos.getControlPoint(), 3, ts);
    } else {
      feasible =
          pp_.speedRatio(start_end_derivatives[0]) <= 1.0 + 1e-9 &&
          pp_.speedRatio(start_end_derivatives[1]) <= 1.0 + 1e-9 &&
          start_end_derivatives[2].norm() <= pp_.max_acc_ + 1e-9 &&
          start_end_derivatives[3].norm() <= pp_.max_acc_ + 1e-9;
      double timeRatio = dynamicTimeRatio(pos, pp_);
      if (feasible && timeRatio > 1.0 + 1e-9) {
        const UniformBspline original = pos;
        const double originalDuration = original.getTimeSum();
        const double originalInterval = ts;
        feasible = false;
        for (int pass = 0; pass < 16; ++pass) {
          if (!std::isfinite(timeRatio * originalDuration))
            break;
          // A failed candidate contributes only its time deficit, never its
          // geometry. Refit the original curve before restoring physical PVA.
          UniformBspline reference = original;
          // lengthenTime stretches only the middle spans, excluding 4*h.
          const double interiorRatio = 1.0 + (timeRatio - 1.0) * originalDuration /
                                               (originalDuration - 4.0 * originalInterval);
          Eigen::MatrixXd controls;
          reparamBspline(reference, start_end_derivatives, interiorRatio, controls, ts);
          setSplineBoundary(controls, ts, start_pt, local_target_pt, start_end_derivatives);
          if (!controls.allFinite())
            break;
          pos = UniformBspline(controls, 3, ts);
          if (dynamicTimeRatio(pos, pp_) <= 1.0 + 1e-9) {
            feasible = true;
            break;
          }
          // XY refinement cannot repair a fixed vertical derivative violation.
          const auto velocity = pos.getDerivative();
          const auto velocityControls = velocity.getControlPoint();
          const double verticalSpeed = velocityControls.row(2)
              .segment(1, velocityControls.cols() - 2).cwiseAbs().maxCoeff();
          const double verticalAcceleration =
              velocity.getDerivative().getControlPoint().row(2).cwiseAbs().maxCoeff();
          if (verticalSpeed <= pp_.max_vel_ + 1e-9 &&
              verticalAcceleration <= pp_.max_acc_ + 1e-9) {
            const int segments = static_cast<int>(controls.cols()) - 3;
            bspline_optimizer_rebound_->ref_pts_.clear();
            for (int segment = 0; segment <= segments; ++segment)
              bspline_optimizer_rebound_->ref_pts_.push_back(pos.evaluateDeBoorT(ts * segment));
            feasible = bspline_optimizer_rebound_->refineConstrained(
                controls, ts, controls, pp_.max_body_vx_, pp_.max_body_vy_, pp_.body_yaw_);
            static_cast<OptimizationDebug &>(rebound_debug_) =
                bspline_optimizer_rebound_->optimizationDebug();
            rebound_debug_.stage = "refine_optimization";
            pos = UniformBspline(controls, 3, ts);
          }
          const double remainingRatio = dynamicTimeRatio(pos, pp_);
          if (feasible || remainingRatio <= 1.0 + 1e-9)
            break;
          timeRatio *= std::max(remainingRatio, 1.01);
        }
      }
    }
    if (!feasible ||
        dynamicTimeRatio(pos, pp_) > 1.0 + 1e-9) {
      feasible = false;
      rebound_debug_.stage = "dynamic_feasibility";
      rebound_debug_.reason = "dynamic_limits_after_retiming";
      rebound_debug_.candidateControlPoints = pos.getControlPoint();
      rebound_debug_.candidateIntervalS = ts;
      checkDynamicFeasibility(pos);
    }
    if (feasible) {
      feasible =
          bspline_optimizer_rebound_->checkTrajectoryCollisionFree(pos);
      if (!feasible) {
        static_cast<OptimizationDebug &>(rebound_debug_) =
            bspline_optimizer_rebound_->optimizationDebug();
        rebound_debug_.stage = "retimed_collision_validation";
        rebound_debug_.candidateControlPoints = pos.getControlPoint();
        rebound_debug_.candidateIntervalS = ts;
      }
    }
  } else {
    double ratio;
    if (!pos.checkFeasibility(ratio, false))
      feasible = refine(ratio);
  }

  if (!feasible || !checkDynamicFeasibility(pos)) {
    if (feasible) {
      rebound_debug_.candidateControlPoints = pos.getControlPoint();
      rebound_debug_.candidateIntervalS = ts;
    }
    return false;
  }

  return true;
}

bool SCANPlannerManager::refineTrajAlgo(
    UniformBspline &traj,
    std::vector<Eigen::Vector3d> &start_end_derivative, double ratio,
    double &ts, Eigen::MatrixXd &optimal_control_points) {
  Eigen::MatrixXd ctrl_pts;
  reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts);
  traj = UniformBspline(ctrl_pts, 3, ts);

  const double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
  bspline_optimizer_rebound_->ref_pts_.clear();
  for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step) {
    bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
  }

  return bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(
      ctrl_pts, ts, optimal_control_points);
}

bool SCANPlannerManager::checkDynamicFeasibility(
    UniformBspline position_traj) {
  UniformBspline vel_traj = position_traj.getDerivative();
  UniformBspline acc_traj = vel_traj.getDerivative();
  const double duration = position_traj.getTimeSum();
  const double sample_dt =
      std::max(0.01, std::min(0.05, duration / 50.0));
  const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
  const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

  for (double t = 0.0; t < duration + 1e-6; t += sample_dt) {
    const double tc = std::min(t, duration);
    const Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
    if (vel.norm() > vel_limit) {
      rebound_debug_.stage = "dynamic_feasibility";
      rebound_debug_.reason = "speed_limit_exceeded";
      rebound_debug_.dynamicViolationValid = true;
      rebound_debug_.dynamicQuantity = "speed";
      rebound_debug_.dynamicValue = vel.norm();
      rebound_debug_.dynamicLimit = vel_limit;
      rebound_debug_.dynamicTimeS = tc;
      return false;
    }

    const Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
    if (acc.norm() > acc_limit) {
      rebound_debug_.stage = "dynamic_feasibility";
      rebound_debug_.reason = "acceleration_limit_exceeded";
      rebound_debug_.dynamicViolationValid = true;
      rebound_debug_.dynamicQuantity = "acceleration";
      rebound_debug_.dynamicValue = acc.norm();
      rebound_debug_.dynamicLimit = acc_limit;
      rebound_debug_.dynamicTimeS = tc;
      return false;
    }
  }
  return true;
}

void SCANPlannerManager::reparamBspline(
    UniformBspline &bspline,
    std::vector<Eigen::Vector3d> &start_end_derivative, double ratio,
    Eigen::MatrixXd &ctrl_pts, double &dt) {
  const int seg_num = bspline.getControlPoint().cols() - 3;

  bspline.lengthenTime(ratio);
  const double duration = bspline.getTimeSum();
  dt = duration / static_cast<double>(seg_num);

  std::vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  UniformBspline::parameterizeToBspline(dt, point_set,
                                        start_end_derivative, ctrl_pts);
}

}  // namespace nav_kernel::local::scan::upstream
