
#include "ElevatorProcess.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include "Context.h"
namespace lingtu::ltlio::detail {
/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Elevator motion detection and world-z acceleration implementation.
 */

void ElevatorProcess::processDoorDetector(const PointCloudXYZI& cloud,
                                          double timestamp,
                                          const State& state,
                                          bool lidar_process_active)
{
    if (!ctx_.elevator_enable || !ctx_.elevator_door_detector_enable) return;
    if (!lidar_process_active) return;

    if (!door_first_packet_received_) {
        door_start_bag_time_ = timestamp;
        door_first_packet_received_ = true;

    }

    std::vector<double> distances;
    distances.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) continue;
        distances.push_back(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
    }
    if (distances.empty()) return;

    std::sort(distances.begin(), distances.end());
    int index = static_cast<int>(distances.size() * (1.0 - ctx_.elevator_door_filter_percent));
    if (index < 0) index = 0;
    if (index >= static_cast<int>(distances.size())) index = static_cast<int>(distances.size()) - 1;
    const double filtered_max_d = distances[static_cast<size_t>(index)];
    door_last_filtered_max_distance_ = filtered_max_d;

    // Door entry is a temporal decision.  A duplicated or out-of-order cloud
    // may still update the diagnostic range, but it must not move the hold
    // window or cooldown backwards and then allow a later frame to satisfy an
    // invalid duration.
    if (std::isfinite(last_door_timestamp_) && timestamp <= last_door_timestamp_)
        return;
    last_door_timestamp_ = timestamp;

    const bool door_closed_like = filtered_max_d > 0.1 && filtered_max_d < ctx_.elevator_door_dist_threshold;

    door_low_distance_active_ = door_closed_like;

    if (!state.in_elevator && !ctx_.ELEVATOR_TRIGGER) {
        if ((timestamp - door_last_enter_trigger_time_) < ctx_.elevator_door_cooldown_time) return;

        if (door_closed_like) {
            // Check every scan so moving away and returning cannot reuse an
            // earlier hold. Do not sum frame-to-frame localization jitter.
            if (door_low_dist_start_time_ < 0.0 ||
                (state.p - door_entry_origin_).norm() > ctx_.elevator_entry_motion_m) {
                door_low_dist_start_time_ = timestamp;
                door_entry_origin_ = state.p;
            }

            if (!door_closed_triggered_ &&
                (timestamp - door_low_dist_start_time_) >= ctx_.elevator_door_time_threshold) {
                ctx_.ELEVATOR_TRIGGER = true;
                door_closed_triggered_ = true;
                door_last_enter_trigger_time_ = timestamp;

            }
        } else {
            door_low_dist_start_time_ = -1.0;
            door_closed_triggered_ = false;
        }
    }
}

double ElevatorProcess::doorCooldownRemaining(double timestamp) const
{
    double remaining = 0.0;
    if (std::isfinite(door_last_enter_trigger_time_)) {
        remaining = std::max(remaining, ctx_.elevator_door_cooldown_time - (timestamp - door_last_enter_trigger_time_));
    }
    return std::max(0.0, remaining);
}

/* ============================== 进入/退出模式 ============================== */

void ElevatorProcess::ElevatorModeEnter(State& S, const Eigen::Vector3d& acc_b)
{
    (void)acc_b;
    // Manual and automatic entry consume the same closed-door episode.
    // A fresh open observation after exit must re-arm automatic entry.
    door_closed_triggered_ = true;
    door_low_dist_start_time_ = -1.0;
    S.z = 0.0;
    S.vz = 0.0;
    S.az = 0.0;
    S.in_elevator = true;
    const int idx_elev_start = StateIndex::Z;
    const int dim_elev = 3;
    const int dim_robot = StateIndex::STATE_TOTAL - dim_elev;
    S.P.block(idx_elev_start, 0, dim_elev, dim_robot).setZero();
    S.P.block(0, idx_elev_start, dim_robot, dim_elev).setZero();
    S.P.block(idx_elev_start, idx_elev_start, dim_elev, dim_elev).setZero();
    S.P(StateIndex::Z, StateIndex::Z) = 1e-4;
    S.P(StateIndex::VZ, StateIndex::VZ) = 1e-4;
    S.P(StateIndex::AZ, StateIndex::AZ) = 1e-3;

    S.P = 0.5 * (S.P + S.P.transpose()); // 可选：保证数值对称
}

void ElevatorProcess::ElevatorModeExit(State& S)
{
    // Return to the historical building-map coordinates. Retain upstream's
    // local-map covariance reset policy (paper V-D2), rather than treating the
    // correlated map as an independent world observation after a position fold.
    // Velocity is absorbed explicitly so a manual exit cannot discard motion.
    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> J =
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    J(StateIndex::V + 2, StateIndex::VZ) = 1.0;
    S.P = J * S.P * J.transpose();
    S.p.z() += S.z;
    S.v.z() += S.vz;
    S.z = 0.0;
    S.vz = 0.0;
    S.az = 0.0;
    S.in_elevator = false;

    const int idx_elev_start = StateIndex::Z;
    const int dim_elev = 3;
    const int dim_robot = StateIndex::STATE_TOTAL - dim_elev;
    S.P.block(idx_elev_start, 0, dim_elev, dim_robot).setZero();
    S.P.block(0, idx_elev_start, dim_robot, dim_elev).setZero();
    S.P.block(idx_elev_start, idx_elev_start, dim_elev, dim_elev).setZero();
    S.P(StateIndex::Z, StateIndex::Z) = 1e-4;
    S.P(StateIndex::VZ, StateIndex::VZ) = 1e-3;
    S.P(StateIndex::AZ, StateIndex::AZ) = 1e-2;

    S.P = 0.5 * (S.P + S.P.transpose()); // 可选
}

/* ============================== 1D 传播与绑定 ============================== */
/**
 * @brief 在电梯模式下进行电梯状态的传播，并填充到协方差子块中
 * @param S 状态
 * @param dt 时间步长
 */
void ElevatorProcess::stepProcess(State& S, double dt)
{
    if (!S.in_elevator || dt <= 0) return;
    S.z  += S.vz * dt + 0.5 * S.az * dt * dt;
    S.vz += S.az * dt;
    Eigen::Matrix3d Phi, Qe;
    Phi << 1, dt, 0.5*dt*dt,
           0,  1, dt,
           0,  0, 1;

    const double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, dt5 = dt4*dt;
    Qe << dt5/20.0, dt4/8.0, dt3/6.0,
          dt4/8.0,  dt3/3.0, dt2/2.0,
          dt3/6.0,  dt2/2.0, dt;
    Qe *= kJerkNoise;
    auto P_EE = S.P.block<3,3>(StateIndex::Z, StateIndex::Z);
    P_EE = Phi * P_EE * Phi.transpose() + Qe;
    S.P.block<3,3>(StateIndex::Z, StateIndex::Z) = P_EE;
    constexpr int N_INS = StateIndex::Z;            // 主块维度（Z 之前）
    Eigen::Matrix<double, N_INS, 3> P_IE = S.P.block<N_INS,3>(0, StateIndex::Z);
    S.P.block<N_INS,3>(0, StateIndex::Z) = P_IE * Phi.transpose();
    S.P.block<3, N_INS>(StateIndex::Z, 0) = (P_IE * Phi.transpose()).transpose();
    S.P = 0.5 * (S.P + S.P.transpose());
}

/**
 * @brief 应用零速更新（ZUPT）伪量测.
 * * @param S   需要更新的系统状态 (State&)，函数将直接修改S和S.P。
 * @param prof 包含噪声参数的配置文件，这里主要使用 R_vel_xy。
 */
void ElevatorProcess::applyZeroVelocityUpdateZ(State& S)
{
    auto world_pose = [](const State& state) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = state.q.toRotationMatrix();
        pose.translation() = state.p + Eigen::Vector3d(0.0, 0.0, state.z);
        return pose;
    };
    const Eigen::Isometry3d before = world_pose(S);
    Eigen::Matrix<double, 2, StateIndex::STATE_TOTAL> H;
    H.setZero();
    H(0,StateIndex::VZ) = 1;
    H(1,StateIndex::AZ) = 1;
    Eigen::Matrix<double, 2, 1> r ;
    r(0) = -S.vz;
    r(1) = -S.az;
    Eigen::Matrix<double, 2, 2> R ;
    R.setZero();
    R(0, 0) = 1E-5; // 速度观测噪声
    R(1, 1) = 1E-4; // 加速度观测噪声
    // Retain the complete posterior, including correlated height correction.
    // Carry the same-time pose correction in the world<-odom transform so
    // navigation odometry remains continuous without discarding map information.
    ekfUpdate<2>(S, H, r, R);
    ctx_.world_from_odom = world_pose(S) * before.inverse() * ctx_.world_from_odom;
}

} // namespace lingtu::ltlio::detail
