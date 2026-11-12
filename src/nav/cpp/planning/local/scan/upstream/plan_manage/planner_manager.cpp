// Ported from SCAN-Planner plan_manage/planner_manager.cpp at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS setup, time, logging and visualization are replaced by C++ values.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/planner_manager.h"
#include "planning/local/scan/upstream/plan_manage/seed.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace nav_kernel::local::scan::upstream {

void SCANPlannerManager::setTimeSource(std::function<double()> timeSource) {
  timeSource_ = std::move(timeSource);
}

double SCANPlannerManager::currentTimeS(const double fallback) const {
  return timeSource_ ? timeSource_() : fallback;
}

void SCANPlannerManager::initPlanModules(
    const PlanParameters &planParams,
    const BsplineOptimizerParams &optimizerParams, GridMap::Ptr gridMap) {
  // Retain the search node pool while replacing per-plan optimizer state.
  auto search = bspline_optimizer_rebound_
      ? std::move(bspline_optimizer_rebound_->a_star_) : std::make_shared<AStar>();
  pp_ = planParams;
  local_data_ = {};
  global_data_ = {};
  grid_map_ = std::move(gridMap);
  timeSource_ = {};
  continuous_failures_count_ = 0;
  rebound_debug_ = {};
  reference_waypoints_.clear();
  reference_progress_ = 0.0;

  bspline_optimizer_rebound_ = std::make_unique<BsplineOptimizer>();
  bspline_optimizer_rebound_->setParam(optimizerParams);
  bspline_optimizer_rebound_->setEnvironment(grid_map_);
  bspline_optimizer_rebound_->a_star_ = std::move(search);
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
    bool flag_randomPolyTraj, double nowS,
    std::optional<std::pair<double,double>> referenceInterval) {
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
  // Exact rest boundaries let time dilation preserve a collision-free curve.
  const bool preserveBoundary = pp_.motion_intent_ || pp_.preserve_boundary_ ||
      (local_target_pt - start_pt).norm() <= pp_.ctrl_pt_dist ||
      start_vel.norm() > pp_.max_vel_ ||
      (start_vel.squaredNorm() == 0.0 && start_acc.squaredNorm() == 0.0 &&
       local_target_vel.squaredNorm() == 0.0);
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

      const double stopToStopTime = time;
      if (preserveBoundary && dist > pp_.max_vel_ * pp_.max_vel_ / pp_.max_acc_) {
        // A moving C2 handoff must not turn a lateral or reversing tangent
        // into extra forward travel time. Only the velocity projected along
        // this segment contributes to the ramp; a negative projection means
        // zero forward progress and falls back to the existing baseline.
        const Eigen::Vector3d direction = (local_target_pt - start_pt) / dist;
        const double initial = std::clamp(start_vel.dot(direction), 0.0, pp_.max_vel_);
        const double terminal = std::clamp(local_target_vel.dot(direction), 0.0, pp_.max_vel_);
        const double initial_deficit = pp_.max_vel_ - initial;
        const double terminal_deficit = pp_.max_vel_ - terminal;
        time = std::min(stopToStopTime,
                        dist / pp_.max_vel_ +
                            (initial_deficit * initial_deficit +
                             terminal_deficit * terminal_deficit) /
                                (2.0 * pp_.max_acc_ * pp_.max_vel_));
      }

      if (!flag_randomPolyTraj) {
        // A long low-speed route needs a bounded acceleration ramp as well.
        // One quintic spanning its entire duration restarts an unnecessarily
        // slow acceleration phase on every local replan.
        if (!seed::boundedVelocitySeed(pp_, start_pt, start_vel, start_acc,
                                 local_target_pt, local_target_vel, gl_traj,
                                 time, ts)) {
          if (start_vel.squaredNorm() == 0.0 && start_acc.squaredNorm() == 0.0 &&
              local_target_vel.squaredNorm() == 0.0) {
            // Exact peak bounds of the rest-to-rest quintic; a higher speed
            // cap must not force a short segment into an infeasible seed.
            time = std::max({time, 1.875 * dist / pp_.max_vel_,
                std::sqrt((10.0 * std::sqrt(3.0) / 3.0) * dist / pp_.max_acc_),
                std::cbrt(60.0 * dist / pp_.max_jerk_)});
          }
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

      double t = 0.0;
      bool flag_too_far;
      do {
        const int intervals = std::max(6, static_cast<int>(std::ceil(time / ts)));
        ts = time / intervals;
        point_set.clear();
        flag_too_far = false;
        Eigen::Vector3d last_pt = gl_traj.evaluate(0);
        for (int i = 0; i <= intervals; ++i) {
          t = i == intervals ? time : i * ts;
          const Eigen::Vector3d pt = gl_traj.evaluate(t);
          if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) {
            flag_too_far = true;
            break;
          }
          last_pt = pt;
          point_set.push_back(pt);
        }
        if (flag_too_far)
          ts /= 1.5;
      } while (flag_too_far || point_set.size() < 7);
      start_end_derivatives.push_back(gl_traj.evaluateVel(0));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
      // Exact-boundary segments use the target's desired zero acceleration,
      // rather than the last sample before that endpoint.
      start_end_derivatives.push_back(preserveBoundary
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

  if (preserveBoundary) {
    // Preserve the caller's physical boundary exactly. Evaluating the fitted
    // seed can leave a nonzero round-off acceleration and select the wrong
    // retiming model even though the measured boundary acceleration is zero.
    start_end_derivatives[0] = start_vel;
    start_end_derivatives[1] = local_target_vel;
    start_end_derivatives[2] = start_acc;
    start_end_derivatives[3].setZero();
  }
  double referenceProgress=reference_progress_;
  if (!referenceInterval || !seed::applyReferenceSeed(point_set,ts,global_data_,
      *referenceInterval,start_pt,local_target_pt,pp_.ctrl_pt_dist,
      !pp_.motion_intent_ ? &reference_waypoints_ : nullptr,
      referenceProgress))
    seed::applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set,
                                        start_end_derivatives, ctrl_pts);
  if (preserveBoundary)
    setSplineBoundary(ctrl_pts, ts, start_pt, local_target_pt,
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
  if (!retime(pos, start_pt, local_target_pt, start_end_derivatives, ts, preserveBoundary)) {
    ++continuous_failures_count_;
    return false;
  }

  reference_progress_=referenceProgress;
  updateTrajInfo(pos, currentTimeS(nowS));
  continuous_failures_count_ = 0;
  rebound_debug_.success = true;
  rebound_debug_.stage = "complete";
  rebound_debug_.reason = "accepted";
  rebound_debug_.collisionValid = false;
  rebound_debug_.dynamicViolationValid = false;
  return true;
}

bool SCANPlannerManager::decelerate(const Eigen::Vector3d &position,
                                    const Eigen::Vector3d &velocity,
                                    const Eigen::Vector3d &acceleration,
                                    double cruiseTimeS, double nowS,
                                    double &transitionEndS) {
  const double initialSpeed = velocity.head<2>().norm();
  if (!grid_map_ || !position.allFinite() || !velocity.allFinite() ||
      !acceleration.allFinite() || initialSpeed <= 1e-6 ||
      acceleration.norm() > pp_.max_acc_ + 1e-9)
    return false;
  Eigen::Vector3d target = velocity;
  target.head<2>() *= std::min(1.0, pp_.max_vel_ / initialSpeed);
  // The vertical reference belongs to the mapped surface; body heave is not
  // an instruction to climb. Bring that derivative smoothly to a quiet tail.
  target.z() = 0.0;
  target /= std::max(1.0, pp_.speedRatio(target));
  const Eigen::Vector3d delta = target - velocity;
  const double interval = std::max({0.05,
      delta.norm() / (pp_.max_acc_ - 0.5 * acceleration.norm()),
      (1.5 * acceleration.norm() + std::sqrt(
          2.25 * acceleration.squaredNorm() + 4.0 * pp_.max_jerk_ * delta.norm())) /
          (2.0 * pp_.max_jerk_)});
  const Eigen::Vector3d middleAcceleration = delta / interval - 0.5 * acceleration;
  const int cruiseSpans = std::max(1, static_cast<int>(std::ceil(cruiseTimeS / interval)));
  Eigen::MatrixXd controls(3, 5 + cruiseSpans);
  const double squaredInterval = interval * interval;
  controls.col(0) = position - interval * velocity + squaredInterval * acceleration / 3.0;
  controls.col(1) = position - squaredInterval * acceleration / 6.0;
  controls.col(2) = position + interval * velocity + squaredInterval * acceleration / 3.0;
  for (Eigen::Index index = 1; index + 2 < controls.cols(); ++index) {
    const Eigen::Vector3d a = index == 1 ? middleAcceleration : Eigen::Vector3d::Zero();
    controls.col(index + 2) = squaredInterval * a +
        2.0 * controls.col(index + 1) - controls.col(index);
  }
  UniformBspline curve(controls, 3, interval);
  const auto curveVelocity = curve.getDerivative();
  std::optional<double> heading;
  for (double time = 0.0; time < curve.getTimeSum(); time += 0.01) {
    const double next = std::min(time + 0.01, curve.getTimeSum());
    const Eigen::Vector3d position = curve.evaluateDeBoorT(time);
    if (grid_map_->getTrajectoryOccupancy(position, curveVelocity.evaluateDeBoorT(time),
            curve.evaluateDeBoorT(next), curveVelocity.evaluateDeBoorT(next), time, next, &heading) != 0)
      return false;
  }
  transitionEndS = 2.0 * interval;
  updateTrajInfo(curve, nowS);
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

  // A real start-to-route connector may be submillimetric. Distance/speed
  // timing would force a resting boundary through it in a fraction of a
  // millisecond and inject metre-scale excursions into the reference fit.
  const Eigen::VectorXd time=seed::referenceDurations(
      inter_points,start_vel,end_vel,pp_.max_vel_,pp_.max_acc_);

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
  reference_waypoints_=inter_points;
  reference_progress_=0.0;
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
  reference_waypoints_.clear();
  reference_progress_=0.0;
  return true;
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

}  // namespace nav_kernel::local::scan::upstream
