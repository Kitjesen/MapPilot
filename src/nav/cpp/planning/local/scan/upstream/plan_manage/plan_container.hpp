#pragma once

// Ported from SCAN-Planner plan_manage/plan_container.hpp at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS time is represented as seconds supplied by the caller.
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"
#include "planning/local/scan/upstream/traj_utils/polynomial_traj.h"

namespace nav_kernel::local::scan::upstream {

class GlobalTrajData {
 public:
  PolynomialTraj global_traj_;
  std::vector<UniformBspline> local_traj_;

  double global_duration_{0.0};
  double global_start_time_{0.0};
  double local_start_time_{-1.0};
  double local_end_time_{-1.0};
  double time_increase_{0.0};
  double last_time_inc_{0.0};
  double last_progress_time_{0.0};

  [[nodiscard]] bool localTrajReachTarget() const {
    return std::fabs(local_end_time_ - global_duration_) < 0.1;
  }

  void setGlobalTraj(const PolynomialTraj &traj, double nowS) {
    global_traj_ = traj;
    global_traj_.init();
    global_duration_ = global_traj_.getTimeSum();
    global_start_time_ = nowS;

    local_traj_.clear();
    local_start_time_ = -1.0;
    local_end_time_ = -1.0;
    time_increase_ = 0.0;
    last_time_inc_ = 0.0;
    last_progress_time_ = 0.0;
  }

  void setLocalTraj(UniformBspline traj, double local_ts, double local_te,
                    double time_inc) {
    local_traj_.resize(3);
    local_traj_[0] = std::move(traj);
    local_traj_[1] = local_traj_[0].getDerivative();
    local_traj_[2] = local_traj_[1].getDerivative();

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_inc;
    time_increase_ += time_inc;
    last_time_inc_ = time_inc;
  }

  Eigen::Vector3d getPosition(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
    }
    if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluate(t - time_increase_);
    }

    double tm, tmp;
    static_cast<void>(local_traj_[0].getTimeSpan(tm, tmp));
    return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
  }

  Eigen::Vector3d getVelocity(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateVel(t);
    }
    if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateVel(t - time_increase_);
    }

    double tm, tmp;
    static_cast<void>(local_traj_[0].getTimeSpan(tm, tmp));
    return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
  }

  Eigen::Vector3d getAcceleration(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateAcc(t);
    }
    if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateAcc(t - time_increase_);
    }

    double tm, tmp;
    static_cast<void>(local_traj_[0].getTimeSpan(tm, tmp));
    return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
  }

  void getTrajByRadius(const double &start_t, const double &des_radius,
                       const double &dist_pt,
                       std::vector<Eigen::Vector3d> &point_set,
                       std::vector<Eigen::Vector3d> &start_end_derivative,
                       double &dt, double &seg_duration) {
    double seg_length = 0.0;
    double seg_time = 0.0;
    double radius = 0.0;

    constexpr double delta = 0.2;
    const Eigen::Vector3d first_pt = getPosition(start_t);
    Eigen::Vector3d prev_pt = first_pt;
    Eigen::Vector3d cur_pt;

    while (radius < des_radius &&
           seg_time < global_duration_ - start_t - 1e-3) {
      seg_time += delta;
      seg_time = std::min(seg_time, global_duration_ - start_t);

      cur_pt = getPosition(start_t + seg_time);
      seg_length += (cur_pt - prev_pt).norm();
      prev_pt = cur_pt;
      radius = (cur_pt - first_pt).norm();
    }

    const int seg_num = std::floor(seg_length / dist_pt);
    seg_duration = seg_time;
    dt = seg_time / seg_num;

    for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt) {
      point_set.push_back(getPosition(start_t + tp));
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + seg_time));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + seg_time));
  }

  void getTrajByDuration(
      double start_t, double duration, int seg_num,
      std::vector<Eigen::Vector3d> &point_set,
      std::vector<Eigen::Vector3d> &start_end_derivative, double &dt) {
    dt = duration / seg_num;
    for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
      point_set.push_back(getPosition(start_t + tp));
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + duration));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + duration));
  }
};

struct PlanParameters {
  double max_vel_{-1.0};
  double max_acc_{-1.0};
  double max_jerk_{-1.0};
  double vel_tolerance_{1.0};
  double acc_tolerance_{1.0};
  double ctrl_pt_dist{-1.0};
  double feasibility_tolerance_{0.0};
  double planning_horizon_{5.0};

  double time_search_{0.0};
  double time_optimize_{0.0};
  double time_adjust_{0.0};
};

struct LocalTrajData {
  int traj_id_{0};
  double duration_{0.0};
  double global_time_offset{0.0};
  double start_time_{0.0};
  Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
  UniformBspline position_traj_;
  UniformBspline velocity_traj_;
  UniformBspline acceleration_traj_;
};

}  // namespace nav_kernel::local::scan::upstream
