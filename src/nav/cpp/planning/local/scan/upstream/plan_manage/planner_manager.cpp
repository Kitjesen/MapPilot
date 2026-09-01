// Ported from SCAN-Planner plan_manage/planner_manager.cpp at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS setup, time, logging and visualization are replaced by C++ values.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/planner_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace nav_kernel::local::scan::upstream {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedSeconds(const SteadyClock::time_point start) {
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
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

bool SCANPlannerManager::reboundReplan(
    Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
    Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
    Eigen::Vector3d local_target_vel, bool flag_polyInit,
    bool flag_randomPolyTraj, double nowS) {
  if ((start_pt - local_target_pt).norm() < 0.2) {
    ++continuous_failures_count_;
    return false;
  }

  auto phase_start = SteadyClock::now();

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
      const double time =
          std::pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist
              ? std::sqrt(dist / pp_.max_acc_)
              : (dist - std::pow(pp_.max_vel_, 2) / pp_.max_acc_) /
                        pp_.max_vel_ +
                    2 * pp_.max_vel_ / pp_.max_acc_;

      if (!flag_randomPolyTraj) {
        gl_traj = PolynomialTraj::one_segment_traj_gen(
            start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
            Eigen::Vector3d::Zero(), time);
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
      start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
    } else {
      double t;
      const double t_cur = nowS - local_data_.start_time_;

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

  applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set,
                                        start_end_derivatives, ctrl_pts);

  bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
  pp_.time_search_ = elapsedSeconds(phase_start);
  phase_start = SteadyClock::now();

  const bool flag_step_1_success =
      bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
  if (!flag_step_1_success) {
    ++continuous_failures_count_;
    return false;
  }

  pp_.time_optimize_ = elapsedSeconds(phase_start);
  phase_start = SteadyClock::now();

  UniformBspline pos(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                        pp_.feasibility_tolerance_);

  double ratio;
  bool flag_step_2_success = true;
  if (!pos.checkFeasibility(ratio, false)) {
    Eigen::MatrixXd optimal_control_points;
    flag_step_2_success = refineTrajAlgo(
        pos, start_end_derivatives, ratio, ts, optimal_control_points);
    if (flag_step_2_success) {
      pos = UniformBspline(optimal_control_points, 3, ts);
    }
  }

  if (!flag_step_2_success || !checkDynamicFeasibility(pos)) {
    ++continuous_failures_count_;
    return false;
  }

  pp_.time_adjust_ = elapsedSeconds(phase_start);
  updateTrajInfo(pos, nowS);
  continuous_failures_count_ = 0;
  return true;
}

bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos,
                                       double nowS) {
  Eigen::MatrixXd control_points(3, 6);
  for (int i = 0; i < 6; ++i) {
    control_points.col(i) = stop_pos;
  }
  updateTrajInfo(UniformBspline(control_points, 3, 1.0), nowS);
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

  global_data_.setGlobalTraj(gl_traj, nowS);
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

  global_data_.setGlobalTraj(gl_traj, nowS);
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
    UniformBspline position_traj) const {
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
      return false;
    }

    const Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
    if (acc.norm() > acc_limit) {
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
