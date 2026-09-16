// Ported from SCAN-Planner plan_manage/planner_manager.cpp at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS setup, time, logging and visualization are replaced by C++ values.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/planner_manager.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace nav_kernel::local::scan::upstream {
namespace {

void setMotionIntentBoundary(Eigen::MatrixXd &controls, const double interval,
                             const Eigen::Vector3d &start,
                             const Eigen::Vector3d &target,
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

double motionIntentTimeRatio(const UniformBspline &position,
                             const PlanParameters &params) {
  const UniformBspline velocity = position.getDerivative();
  const UniformBspline acceleration = velocity.getDerivative();
  // Each uniform cubic position span has quadratic velocity Bezier controls
  // (D[i] + D[i+1])/2, D[i+1], (D[i+1] + D[i+2])/2. Interior averages are
  // bounded by the inner derivative controls. Include the actual endpoints;
  // the extrapolated first/last controls can exceed the curve's speed limit.
  const Eigen::MatrixXd velocity_controls = velocity.getControlPoint();
  const double velocity_bound = std::max({
      velocity.evaluateDeBoorT(0.0).norm(),
      velocity.evaluateDeBoorT(position.getTimeSum()).norm(),
      velocity_controls.middleCols(1, velocity_controls.cols() - 2)
          .colwise().norm().maxCoeff()});
  const double acceleration_bound =
      acceleration.getControlPoint().colwise().norm().maxCoeff();
  return std::max({1.0, velocity_bound / params.max_vel_,
                   std::sqrt(acceleration_bound / params.max_acc_)});
}

bool retimeZeroAccelerationMotionIntent(
    UniformBspline &position, const PlanParameters &params,
    const Eigen::Vector3d &start, const Eigen::Vector3d &target,
    const std::vector<Eigen::Vector3d> &derivatives, double &interval) {
  constexpr double kRatioEpsilon = 1e-9;
  if (motionIntentTimeRatio(position, params) <= 1.0 + kRatioEpsilon)
    return true;

  Eigen::MatrixXd base = position.getControlPoint();
  Eigen::MatrixXd slope = Eigen::MatrixXd::Zero(base.rows(), base.cols());
  const Eigen::Index last = base.cols() - 1;

  base.col(0) = start;
  base.col(1) = start;
  base.col(2) = start;
  slope.col(0) = -derivatives[0];
  slope.col(2) = derivatives[0];
  base.col(last - 2) = target;
  base.col(last - 1) = target;
  base.col(last) = target;
  slope.col(last - 2) = -derivatives[1];
  slope.col(last) = derivatives[1];

  // With zero endpoint acceleration every control is B + h*L. Therefore the
  // derivative controls are dB/h + dL and d2B/h^2 + d2L/h. These triangle
  // bounds for acceleration and exact velocity roots produce a duration that
  // is strictly feasible for every control.
  double feasible_high = interval;
  for (Eigen::Index index = 0; index < base.cols() - 1; ++index) {
    const Eigen::Vector3d intercept =
        base.col(index + 1) - base.col(index);
    const Eigen::Vector3d asymptotic =
        slope.col(index + 1) - slope.col(index);
    const double intercept_squared = intercept.squaredNorm();
    const double asymptotic_norm = asymptotic.norm();
    if (intercept_squared == 0.0) {
      if (asymptotic_norm > params.max_vel_) return false;
      continue;
    }
    if (asymptotic_norm > params.max_vel_) return false;
    const double margin_squared =
        (params.max_vel_ - asymptotic_norm) *
        (params.max_vel_ + asymptotic_norm);
    const double dot = intercept.dot(asymptotic);
    double required_interval = 0.0;
    if (margin_squared > 0.0) {
      const double root =
          std::sqrt(dot * dot + margin_squared * intercept_squared);
      required_interval = dot < 0.0
                              ? intercept_squared / (root - dot)
                              : (dot + root) / margin_squared;
    } else if (dot < 0.0) {
      required_interval = intercept_squared / (-2.0 * dot);
    } else {
      return false;
    }
    feasible_high = std::max(feasible_high, required_interval);
  }
  for (Eigen::Index index = 0; index < base.cols() - 2; ++index) {
    const double intercept =
        (base.col(index + 2) - 2.0 * base.col(index + 1) +
         base.col(index))
            .norm();
    const double linear =
        (slope.col(index + 2) - 2.0 * slope.col(index + 1) +
         slope.col(index))
            .norm();
    feasible_high = std::max(
        feasible_high,
        (linear + std::sqrt(linear * linear +
                            4.0 * params.max_acc_ * intercept)) /
            (2.0 * params.max_acc_));
  }
  if (!std::isfinite(feasible_high)) return false;

  const auto splineAt = [&](const double candidate_interval) {
    return UniformBspline(base + candidate_interval * slope, 3,
                          candidate_interval);
  };
  UniformBspline feasible = splineAt(feasible_high);
  if (motionIntentTimeRatio(feasible, params) > 1.0 + kRatioEpsilon)
    return false;

  double infeasible_low = interval;
  const double target_width = interval * 1e-6;
  const double initial_width = feasible_high - infeasible_low;
  const int steps = initial_width > target_width
                        ? static_cast<int>(
                              std::ceil(std::log2(initial_width / target_width)))
                        : 0;
  for (int step = 0; step < steps; ++step) {
    const double middle = 0.5 * (infeasible_low + feasible_high);
    UniformBspline candidate = splineAt(middle);
    if (motionIntentTimeRatio(candidate, params) <= 1.0 + kRatioEpsilon) {
      feasible_high = middle;
      feasible = std::move(candidate);
    } else {
      infeasible_low = middle;
    }
  }
  interval = feasible_high;
  position = std::move(feasible);
  return true;
}

bool motionIntentSeed(const PlanParameters &params,
                      const Eigen::Vector3d &start,
                      const Eigen::Vector3d &start_velocity,
                      const Eigen::Vector3d &start_acceleration,
                      const Eigen::Vector3d &target,
                      const Eigen::Vector3d &target_velocity,
                      PolynomialTraj &trajectory, double &duration,
                      double &interval) {
  // This seed joins constant-acceleration-bound velocity ramps to a cruise.
  // Nonzero initial acceleration and short maneuvers retain the existing
  // polynomial initializer, including all measured boundary derivatives.
  const double speed = params.max_vel_;
  if (start_acceleration.squaredNorm() != 0.0 ||
      start_velocity.norm() > speed || target_velocity.norm() > speed)
    return false;

  const auto ramp_time = [&](const Eigen::Vector3d &velocity) {
    // |v_cruise - velocity| <= speed + |velocity|, including lateral motion.
    const double delta = speed + velocity.norm();
    return std::max(1.5 * delta / params.max_acc_,
                    std::sqrt(6.0 * delta / params.max_jerk_));
  };
  const double accelerate_time = ramp_time(start_velocity);
  const double brake_time = ramp_time(target_velocity);
  const Eigen::Vector3d cruise_displacement =
      target - start - 0.5 * accelerate_time * start_velocity -
      0.5 * brake_time * target_velocity;
  const double cruise_equivalent_time = cruise_displacement.norm() / speed;
  const double cruise_time =
      cruise_equivalent_time - 0.5 * (accelerate_time + brake_time);
  if (cruise_time <= 0.0)
    return false;

  const Eigen::Vector3d cruise_velocity =
      cruise_displacement / cruise_equivalent_time;
  const Eigen::Vector3d cruise_start =
      start + 0.5 * accelerate_time * (start_velocity + cruise_velocity);
  const Eigen::Vector3d cruise_end =
      target - 0.5 * brake_time * (cruise_velocity + target_velocity);
  const auto append = [&](const Eigen::Vector3d &p0,
                          const Eigen::Vector3d &v0,
                          const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &v1, const double time) {
    auto segment = PolynomialTraj::one_segment_traj_gen(
        p0, v0, Eigen::Vector3d::Zero(), p1, v1,
        Eigen::Vector3d::Zero(), time);
    trajectory.addSegment(segment.getCoef(0).front(),
                          segment.getCoef(1).front(),
                          segment.getCoef(2).front(), time);
  };
  append(start, start_velocity, cruise_start, cruise_velocity, accelerate_time);
  append(cruise_start, cruise_velocity, cruise_end, cruise_velocity, cruise_time);
  append(cruise_end, cruise_velocity, target, target_velocity, brake_time);
  trajectory.init();
  duration = accelerate_time + cruise_time + brake_time;
  // A cubic spline needs samples within each ramp, even at a low speed cap.
  interval = std::min(interval, std::min(accelerate_time, brake_time) / 3.0);
  return true;
}

void applyLinearZReference(std::vector<Eigen::Vector3d> &points,
                           const double start_z, const double target_z) {
  if (points.empty()) {
    return;
  }
  if (points.size() == 1) {
    points.front()(2) = start_z;
    return;
  }

  std::vector<double> accumulated_xy_length(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    accumulated_xy_length[i] =
        accumulated_xy_length[i - 1] +
        (points[i].head<2>() - points[i - 1].head<2>()).norm();
  }

  const double total_xy_length = accumulated_xy_length.back();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double ratio =
        total_xy_length > 1e-6
            ? accumulated_xy_length[i] / total_xy_length
            : static_cast<double>(i) / static_cast<double>(points.size() - 1);
    points[i](2) = start_z + ratio * (target_z - start_z);
  }

  points.front()(2) = start_z;
  points.back()(2) = target_z;
}

}  // namespace

void SCANPlannerManager::setTimeSource(std::function<double()> timeSource) {
  timeSource_ = std::move(timeSource);
}

double SCANPlannerManager::currentTimeS(const double fallback) const {
  return timeSource_ ? timeSource_() : fallback;
}

void SCANPlannerManager::initPlanModules(
    const PlanParameters &planParams,
    const BsplineOptimizerParams &optimizerParams, GridMap::Ptr gridMap) {
  pp_ = planParams;
  local_data_.traj_id_ = 0;
  grid_map_ = std::move(gridMap);

  bspline_optimizer_rebound_ = std::make_unique<BsplineOptimizer>();
  bspline_optimizer_rebound_->setParam(optimizerParams);
  bspline_optimizer_rebound_->setEnvironment(grid_map_);
  bspline_optimizer_rebound_->a_star_ = std::make_shared<AStar>();
  bspline_optimizer_rebound_->a_star_->initGridMap(
      grid_map_, Eigen::Vector3i(100, 100, 100));
}

void SCANPlannerManager::updatePlanParameters(
    const PlanParameters &planParams,
    const BsplineOptimizerParams &optimizerParams) {
  pp_ = planParams;
  bspline_optimizer_rebound_->setParam(optimizerParams);
}

bool SCANPlannerManager::reboundReplan(
    Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
    Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
    Eigen::Vector3d local_target_vel, bool flag_polyInit,
    bool flag_randomPolyTraj, double nowS) {
  const std::uint64_t attemptId = rebound_debug_.attemptId + 1U;
  rebound_debug_ = {};
  rebound_debug_.attemptId = attemptId;
  rebound_debug_.attempted = true;
  rebound_debug_.stage = "initialization";
  rebound_debug_.startPosition = start_pt;
  rebound_debug_.startVelocity = start_vel;
  rebound_debug_.startAcceleration = start_acc;
  rebound_debug_.targetPosition = local_target_pt;
  rebound_debug_.targetVelocity = local_target_vel;
  rebound_debug_.polyInit = flag_polyInit;
  rebound_debug_.randomPolyInit = flag_randomPolyTraj;
  if ((start_pt - local_target_pt).norm() < 1e-3) {
    rebound_debug_.reason = "target_too_close";
    ++continuous_failures_count_;
    return false;
  }

  double ts = (start_pt - local_target_pt).norm() > 0.1
                  ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2
                  : pp_.ctrl_pt_dist / pp_.max_vel_ * 5.0;
  std::vector<Eigen::Vector3d> point_set, start_end_derivatives;
  static bool flag_first_call = true;
  static bool flag_force_polynomial = false;
  bool flag_regenerate = false;
  do {
    point_set.clear();
    start_end_derivatives.clear();
    flag_regenerate = false;

    if (flag_first_call || flag_polyInit || flag_force_polynomial) {
      flag_first_call = false;
      flag_force_polynomial = false;

      PolynomialTraj gl_traj;
      const double dist = (start_pt - local_target_pt).norm();
      double time =
          std::pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist
              ? std::sqrt(dist / pp_.max_acc_)
              : (dist - std::pow(pp_.max_vel_, 2) / pp_.max_acc_) /
                        pp_.max_vel_ +
                    2 * pp_.max_vel_ / pp_.max_acc_;

      if (!flag_randomPolyTraj) {
        if (!pp_.motion_intent_ ||
            !motionIntentSeed(pp_, start_pt, start_vel, start_acc,
                              local_target_pt, local_target_vel, gl_traj,
                              time, ts)) {
          gl_traj = PolynomialTraj::one_segment_traj_gen(
              start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
              Eigen::Vector3d::Zero(), time);
        }
      } else {
        const Eigen::Vector3d horizon_dir =
            ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1)))
                .normalized();
        const Eigen::Vector3d vertical_dir =
            ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
        const Eigen::Vector3d random_inserted_pt =
            (start_pt + local_target_pt) / 2 +
            (static_cast<double>(std::rand()) / RAND_MAX - 0.5) *
                (start_pt - local_target_pt).norm() * horizon_dir * 0.8 *
                (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
            (static_cast<double>(std::rand()) / RAND_MAX - 0.5) *
                (start_pt - local_target_pt).norm() * vertical_dir * 0.4 *
                (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
        Eigen::MatrixXd pos(3, 3);
        pos.col(0) = start_pt;
        pos.col(1) = random_inserted_pt;
        pos.col(2) = local_target_pt;
        Eigen::VectorXd segment_time(2);
        segment_time(0) = segment_time(1) = time / 2;
        gl_traj = PolynomialTraj::minSnapTraj(
            pos, start_vel, local_target_vel, start_acc,
            Eigen::Vector3d::Zero(), segment_time);
      }

      double t;
      bool flag_too_far;
      ts *= 1.5;
      do {
        ts /= 1.5;
        point_set.clear();
        flag_too_far = false;
        Eigen::Vector3d last_pt = gl_traj.evaluate(0);
        for (t = 0; t < time; t += ts) {
          const Eigen::Vector3d pt = gl_traj.evaluate(t);
          if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) {
            flag_too_far = true;
            break;
          }
          last_pt = pt;
          point_set.push_back(pt);
        }
      } while (flag_too_far || point_set.size() < 7);
      t -= ts;
      start_end_derivatives.push_back(gl_traj.evaluateVel(0));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
      // MotionIntent ends at the actual target with the polynomial's desired
      // zero acceleration, not at the last sample before that endpoint.
      start_end_derivatives.push_back(pp_.motion_intent_
                                         ? Eigen::Vector3d::Zero().eval()
                                         : gl_traj.evaluateAcc(t));
    } else {
      double t;
      const double t_cur = currentTimeS(nowS) - local_data_.start_time_;

      std::vector<double> pseudo_arc_length;
      std::vector<Eigen::Vector3d> segment_point;
      pseudo_arc_length.push_back(0.0);
      for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts) {
        segment_point.push_back(
            local_data_.position_traj_.evaluateDeBoorT(t));
        if (t > t_cur) {
          pseudo_arc_length.push_back(
              (segment_point.back() -
               segment_point[segment_point.size() - 2])
                      .norm() +
                  pseudo_arc_length.back());
        }
      }
      t -= ts;

      const double poly_time =
          (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt)
              .norm() /
          pp_.max_vel_ * 2;
      if (poly_time > ts) {
        PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
            local_data_.position_traj_.evaluateDeBoorT(t),
            local_data_.velocity_traj_.evaluateDeBoorT(t),
            local_data_.acceleration_traj_.evaluateDeBoorT(t),
            local_target_pt, local_target_vel, Eigen::Vector3d::Zero(),
            poly_time);

        for (t = ts; t < poly_time; t += ts) {
          if (pseudo_arc_length.empty()) {
            rebound_debug_.reason = "continuation_empty";
            ++continuous_failures_count_;
            return false;
          }
          segment_point.push_back(gl_traj.evaluate(t));
          pseudo_arc_length.push_back(
              (segment_point.back() -
               segment_point[segment_point.size() - 2])
                      .norm() +
                  pseudo_arc_length.back());
        }
      }

      double sample_length = 0.0;
      double cps_dist = pp_.ctrl_pt_dist * 1.5;
      std::size_t id = 0;
      do {
        cps_dist /= 1.5;
        point_set.clear();
        sample_length = 0.0;
        id = 0;
        while (id <= pseudo_arc_length.size() - 2 &&
               sample_length <= pseudo_arc_length.back()) {
          if (sample_length >= pseudo_arc_length[id] &&
              sample_length < pseudo_arc_length[id + 1]) {
            point_set.push_back(
                (sample_length - pseudo_arc_length[id]) /
                        (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) *
                    segment_point[id + 1] +
                (pseudo_arc_length[id + 1] - sample_length) /
                        (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) *
                    segment_point[id]);
            sample_length += cps_dist;
          } else {
            ++id;
          }
        }
        point_set.push_back(local_target_pt);
      } while (point_set.size() < 7);

      start_end_derivatives.push_back(
          local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(
          local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());

      if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) {
        flag_force_polynomial = true;
        flag_regenerate = true;
      }
    }
  } while (flag_regenerate);

  if (pp_.motion_intent_) {
    // Preserve the caller's physical boundary exactly. Evaluating the fitted
    // seed can leave a nonzero round-off acceleration and select the wrong
    // retiming model even though the measured boundary acceleration is zero.
    start_end_derivatives[0] = start_vel;
    start_end_derivatives[1] = local_target_vel;
    start_end_derivatives[2] = start_acc;
    start_end_derivatives[3].setZero();
  }
  applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set,
                                        start_end_derivatives, ctrl_pts);
  if (pp_.motion_intent_)
    setMotionIntentBoundary(ctrl_pts, ts, start_pt, local_target_pt,
                            start_end_derivatives);

  bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

  const bool flag_step_1_success =
      bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
  static_cast<OptimizationDebug &>(rebound_debug_) =
      bspline_optimizer_rebound_->optimizationDebug();
  rebound_debug_.stage = "rebound_optimization";
  if (!flag_step_1_success) {
    rebound_debug_.candidateControlPoints = ctrl_pts;
    rebound_debug_.candidateIntervalS = ts;
    ++continuous_failures_count_;
    return false;
  }

  UniformBspline pos(ctrl_pts, 3, ts);
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

  bool flag_step_2_success = true;
  if (pp_.motion_intent_) {
    if (start_end_derivatives[2].squaredNorm() == 0.0 &&
        start_end_derivatives[3].squaredNorm() == 0.0) {
      flag_step_2_success = retimeZeroAccelerationMotionIntent(
          pos, pp_, start_pt, local_target_pt, start_end_derivatives, ts);
    } else {
      // Uniform dilation preserves the interior curve. Restore exact physical
      // endpoint P/V/A, which can change the endpoint geometry for nonzero V/A.
      // Those restored controls change the next ratio. Allow convergence;
      // three passes reject feasible moving starts while still near the limit.
      flag_step_2_success =
          start_end_derivatives[0].norm() <= pp_.max_vel_ + 1e-9 &&
          start_end_derivatives[1].norm() <= pp_.max_vel_ + 1e-9 &&
          start_end_derivatives[2].norm() <= pp_.max_acc_ + 1e-9 &&
          start_end_derivatives[3].norm() <= pp_.max_acc_ + 1e-9;
      for (int pass = 0; flag_step_2_success && pass < 64; ++pass) {
        const double ratio = motionIntentTimeRatio(pos, pp_);
        if (ratio <= 1.0 + 1e-9)
          break;
        const double next_interval = ts * ratio;
        if (!std::isfinite(ratio) || !std::isfinite(next_interval)) {
          flag_step_2_success = false;
          break;
        }
        Eigen::MatrixXd adjusted_controls = pos.getControlPoint();
        setMotionIntentBoundary(adjusted_controls, next_interval, start_pt,
                                local_target_pt, start_end_derivatives);
        if (!adjusted_controls.allFinite()) {
          flag_step_2_success = false;
          break;
        }
        ts = next_interval;
        pos = UniformBspline(adjusted_controls, 3, ts);
      }
    }
    if (!flag_step_2_success ||
        motionIntentTimeRatio(pos, pp_) > 1.0 + 1e-9) {
      flag_step_2_success = false;
      rebound_debug_.stage = "dynamic_feasibility";
      rebound_debug_.reason = "dynamic_limits_after_retiming";
      rebound_debug_.candidateControlPoints = pos.getControlPoint();
      rebound_debug_.candidateIntervalS = ts;
    }
    if (flag_step_2_success) {
      flag_step_2_success =
          bspline_optimizer_rebound_->checkTrajectoryCollisionFree(pos);
      if (!flag_step_2_success) {
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
      flag_step_2_success = refine(ratio);
  }

  if (!flag_step_2_success || !checkDynamicFeasibility(pos)) {
    if (flag_step_2_success) {
      rebound_debug_.candidateControlPoints = pos.getControlPoint();
      rebound_debug_.candidateIntervalS = ts;
    }
    ++continuous_failures_count_;
    return false;
  }

  updateTrajInfo(pos, currentTimeS(nowS));
  continuous_failures_count_ = 0;
  rebound_debug_.success = true;
  rebound_debug_.stage = "complete";
  rebound_debug_.reason = "accepted";
  rebound_debug_.collisionValid = false;
  rebound_debug_.dynamicViolationValid = false;
  return true;
}

bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos,
                                       double nowS) {
  Eigen::MatrixXd control_points(3, 6);
  for (int i = 0; i < 6; ++i) {
    control_points.col(i) = stop_pos;
  }
  updateTrajInfo(UniformBspline(control_points, 3, 1.0),
                 currentTimeS(nowS));
  return true;
}

bool SCANPlannerManager::planGlobalTrajWaypoints(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc,
    const std::vector<Eigen::Vector3d> &waypoints,
    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
    double nowS) {
  if (waypoints.empty()) {
    return false;
  }

  std::vector<Eigen::Vector3d> points;
  points.push_back(start_pos);
  points.insert(points.end(), waypoints.begin(), waypoints.end());

  double total_len = 0.0;
  for (std::size_t i = 0; i < points.size() - 1; ++i) {
    total_len += (points[i + 1] - points[i]).norm();
  }

  std::vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = std::max(total_len / 8, 4.0);
  for (std::size_t i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    const double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      const int id_num = std::floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        inter_points.push_back(
            points.at(i) * (1.0 - static_cast<double>(j) / id_num) +
            points.at(i + 1) * static_cast<double>(j) / id_num);
      }
    }
  }
  inter_points.push_back(points.back());

  const int pt_num = static_cast<int>(inter_points.size());
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i) {
    pos.col(i) = inter_points[i];
  }

  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.col(i + 1) - pos.col(i)).norm() / pp_.max_vel_;
  }
  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  PolynomialTraj gl_traj;
  if (pos.cols() >= 3) {
    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel,
                                          start_acc, end_acc, time);
  } else if (pos.cols() == 2) {
    gl_traj = PolynomialTraj::one_segment_traj_gen(
        start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc,
        time(0));
  } else {
    return false;
  }

  global_data_.setGlobalTraj(gl_traj, currentTimeS(nowS));
  return true;
}

bool SCANPlannerManager::planGlobalTraj(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pos,
    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
    double nowS) {
  std::vector<Eigen::Vector3d> points{start_pos, end_pos};
  std::vector<Eigen::Vector3d> inter_points;
  constexpr double dist_thresh = 4.0;

  for (std::size_t i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    const double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      const int id_num = std::floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        inter_points.push_back(
            points.at(i) * (1.0 - static_cast<double>(j) / id_num) +
            points.at(i + 1) * static_cast<double>(j) / id_num);
      }
    }
  }
  inter_points.push_back(points.back());

  const int pt_num = static_cast<int>(inter_points.size());
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i) {
    pos.col(i) = inter_points[i];
  }

  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.col(i + 1) - pos.col(i)).norm() / pp_.max_vel_;
  }
  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  PolynomialTraj gl_traj;
  if (pos.cols() >= 3) {
    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel,
                                          start_acc, end_acc, time);
  } else if (pos.cols() == 2) {
    gl_traj = PolynomialTraj::one_segment_traj_gen(
        start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
  } else {
    return false;
  }

  global_data_.setGlobalTraj(gl_traj, currentTimeS(nowS));
  return true;
}

bool SCANPlannerManager::refineTrajAlgo(
    UniformBspline &traj,
    std::vector<Eigen::Vector3d> &start_end_derivative, double ratio,
    double &ts, Eigen::MatrixXd &optimal_control_points) {
  double time_inc;
  Eigen::MatrixXd ctrl_pts;
  reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, time_inc);
  traj = UniformBspline(ctrl_pts, 3, ts);

  const double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
  bspline_optimizer_rebound_->ref_pts_.clear();
  for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step) {
    bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
  }

  return bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(
      ctrl_pts, ts, optimal_control_points);
}

void SCANPlannerManager::updateTrajInfo(
    const UniformBspline &position_traj, double nowS) {
  local_data_.start_time_ = nowS;
  local_data_.position_traj_ = position_traj;
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ =
      local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ =
      local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  ++local_data_.traj_id_;
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
    Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc) {
  const double time_origin = bspline.getTimeSum();
  const int seg_num = bspline.getControlPoint().cols() - 3;

  bspline.lengthenTime(ratio);
  const double duration = bspline.getTimeSum();
  dt = duration / static_cast<double>(seg_num);
  time_inc = duration - time_origin;

  std::vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  UniformBspline::parameterizeToBspline(dt, point_set,
                                        start_end_derivative, ctrl_pts);
}

}  // namespace nav_kernel::local::scan::upstream
