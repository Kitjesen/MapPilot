// SPDX-License-Identifier: GPL-2.0-or-later
// Numerical body from Elevator-LIO 1d79af77; original source creator: xiaofan.
#include "ESEKF.h"

#include "Context.h"
#include "IkdNearestQuery.hpp"
#include "ElevatorProcess.h"
#include "Types.h"
#include "Localizability.hpp"
#include "omp.h"

#include <cstddef>
#include <limits>
namespace lingtu::ltlio::detail {

static Eigen::Vector3d SO3Log(const Eigen::Matrix3d&SO3 ){
    const Eigen::AngleAxisd rotation(SO3);
    return rotation.angle() * rotation.axis();
}

static Eigen::Matrix3d A_T(const Eigen::Vector3d& v) {
    const double theta_sq = v.squaredNorm();
    const double theta = std::sqrt(theta_sq);
    const Eigen::Matrix3d K = skew3d(v);

    if (theta < 1e-6) {
        return Eigen::Matrix3d::Identity() + 0.5 * K + (1.0/6.0) * K * K;
    }

    const Eigen::Matrix3d K2 = K * K;
    return Eigen::Matrix3d::Identity()
           + ((1.0 - std::cos(theta)) / theta_sq) * K
           + ((theta - std::sin(theta)) / (theta_sq * theta)) * K2;
}

namespace {
    constexpr int kLidarPoseDim = 6;

    /**
     * @brief 辅助函数：将字符串转换为全小写并返回副本
     * 使用 std::string 传值调用自动创建副本，适合需要返回新字符串的场景
     */
    std::string normalizeLowerCopy(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return value;
    }

    /**
     * @brief 判断 Lidar 更新模式
     */
    bool usePose6LidarUpdate(const Context& ctx_) {
        const std::string mode = normalizeLowerCopy(ctx_.lidar_update_mode);
        return mode.empty() || mode == "pose6" || mode == "pose_6";
    }

    /**
     * @brief 判断是否启用 Fast-LIO 风格的评分闸门（Score Gate）
     * @return 若模式匹配则返回 true，用于决定是否采用基于残差评分的匹配过滤
     */


    /**
     * @brief 从完整的状态误差向量中提取位姿相关的误差（旋转+平移）
     * @param error_s 完整的误差状态向量（含旋转、平移、速度、Bias等）
     * @return Eigen::Matrix 包含 6 自由度位姿误差的列向量 [Rotation, Position]^T
     */
    Eigen::Matrix<double, kLidarPoseDim, 1> packPoseError(
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &error_s) {
        Eigen::Matrix<double, kLidarPoseDim, 1> pose_error;
        pose_error.template segment<3>(0) = error_s.template segment<3>(StateIndex::R);
        pose_error.template segment<3>(3) = error_s.template segment<3>(StateIndex::P);
        return pose_error;
    }

    /**
     * @brief 计算动态大小矩阵中所有元素绝对值的最大值
     * @param x 输入矩阵
     * @return 返回最大绝对值，若矩阵为空则返回 0.0
     */
    double maxAbsCoeff(const Eigen::MatrixXd &x) {
        return x.size() > 0 ? x.cwiseAbs().maxCoeff() : 0.0;
    }

    /**
     * @brief 计算固定大小状态向量中所有元素绝对值的最大值
     * @note 常用于检查整个状态向量是否发生数值爆炸或越界
     */
    double maxAbsCoeff(const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &x) {
        return x.cwiseAbs().maxCoeff();
    }

    using FullStateVector = Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>;
    using FullStateMatrix = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>;

    void applyPoseIterationCorrection(const FullStateVector &dx,
                                      FullStateVector &dx_new,
                                      FullStateMatrix &P) {
        const Eigen::Matrix3d so3_correction = A_T(dx.segment<3>(StateIndex::R)).transpose();
        dx_new.segment<3>(StateIndex::R) = so3_correction * dx_new.segment<3>(StateIndex::R);
        for (int c = 0; c < StateIndex::STATE_TOTAL; ++c) {
            P.block<3, 1>(StateIndex::R, c) = so3_correction * P.block<3, 1>(StateIndex::R, c);
        }
        for (int r = 0; r < StateIndex::STATE_TOTAL; ++r) {
            P.block<1, 3>(r, StateIndex::R) = P.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
        }
    }

    void applyPoseFinalCovarianceCorrection(const FullStateVector &dx,
                                            FullStateMatrix &L,
                                            FullStateMatrix &P_base,
                                            FullStateMatrix &K_x) {
        const Eigen::Matrix3d so3_correction = A_T(dx.segment<3>(StateIndex::R)).transpose();
        for (int c = 0; c < StateIndex::STATE_TOTAL; ++c) {
            L.block<3, 1>(StateIndex::R, c) = so3_correction * P_base.block<3, 1>(StateIndex::R, c);
        }
        for (int c = 0; c < kLidarPoseDim; ++c) {
            K_x.block<3, 1>(StateIndex::R, c) = so3_correction * K_x.block<3, 1>(StateIndex::R, c);
        }
        for (int r = 0; r < StateIndex::STATE_TOTAL; ++r) {
            L.block<1, 3>(r, StateIndex::R) = L.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
            P_base.block<1, 3>(r, StateIndex::R) = P_base.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
        }
    }

    /**
     * @brief 判断 IESKF 迭代是否收敛
     * @param update_x 当前迭代步的状态增量
     * @return true 表示旋转和平移的增量均小于设定阈值，可以停止迭代
     */
    bool poseConverged(const Context& ctx_, const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &update_x) {
        const double rot_max = update_x.segment<3>(StateIndex::R).cwiseAbs().maxCoeff();
        const double pos_max = update_x.segment<3>(StateIndex::P).cwiseAbs().maxCoeff();
        return rot_max <= ctx_.lidar_ieskf_converge_rot_rad && pos_max <= ctx_.lidar_ieskf_converge_pos_m;
    }

    /**
     * @brief 判断在迭代过程中是否需要重新寻找最近邻点（Rematch）
     * @param update_x 当前迭代步的状态增量
     * @param iter_idx 当前迭代索引
     * @param iter_num 总迭代次数限制
     * @return 返回 true 表示当前位姿变化已足够小，可以进行最后一次精确的近邻搜索
     */
    bool shouldRematchPose(const Context& ctx_, const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &update_x, int iter_idx, int iter_num) {
        const double rot_max = update_x.segment<3>(StateIndex::R).cwiseAbs().maxCoeff();
        const double pos_max = update_x.segment<3>(StateIndex::P).cwiseAbs().maxCoeff();
        return (rot_max <= ctx_.lidar_ieskf_rematch_rot_rad && pos_max <= ctx_.lidar_ieskf_rematch_pos_m)
               || iter_idx == iter_num - 2;
    }

} // namespace

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, double dt)
{
    double ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();

    if (ang_vel_norm < 0.0000001)
    {

        return Eye3;

    }
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << skew3d(r_axis);
    double r_ang = ang_vel_norm * dt;
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
}

template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t) {
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = q * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}
template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Matrix<T,3,3> &transMatrix,const Eigen::Matrix<T,3,1> &t){
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = transMatrix * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}

EskfEstimator::EskfEstimator(Context& context) : ctx_(context) {
    state_.q = Eigen::Quaterniond(1, 0, 0, 0);
    state_.p = Eigen::Vector3d(0, 0, 0);
    state_.v = Eigen::Vector3d(0, 0, 0);
    state_.bw = Eigen::Vector3d(0, 0, 0);
    state_.ba = Eigen::Vector3d(0, 0, 0);
    state_.g = Eigen::Vector3d(0, 0, -G_m_s2);
    state_.P = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    state_.P.block<3,3>(StateIndex::P, StateIndex::P) = Eigen::Matrix3d::Identity() * 0.01;
    state_.P.block<3,3>(StateIndex::V, StateIndex::V) = Eigen::Matrix3d::Identity() * 0.01;
    state_.time = 0;
    mean_acc_ = Eigen::Vector3d(0, 0, 0);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
}

EskfEstimator::~EskfEstimator() = default;

/**
 * @brief 利用lidar数据进行更新，寻找最近邻点、匹配对应点、迭代 kalman 更新当前状态 state_，迭代过程存储在s_next_v中
 * @param Dedistort_clouds_lidar 去畸变点云，lidar坐标系
 */
void EskfEstimator::UpdateByLidarIESKF(PointCloudXYZI &Dedistort_clouds_lidar) {
    last_geometry_ = {};
    last_iteration_count_ = 0;
    last_converged_ = false;
    last_update_accepted_ = false;
    last_point_match_count_ = last_plane_match_count_ = 0;
    last_rejection_reason_ = "no_effective_points";
    candidate_translation_ = candidate_rotation_ = candidate_velocity_delta_ = 0.0;
    state_vector_post_.clear();
    const State prior = state_;
    State s_next = state_;

    const int iter_num = std::max(1, ctx_.lidar_ieskf_max_iterations);
    int point_num = Dedistort_clouds_lidar.points.size();
    if (point_num < 10) {
        state_vector_post_.push_back(s_next);

        return;
    }
    if(s_next.in_elevator) {

    }
    prepareFrameCache(Dedistort_clouds_lidar);
    bool converge;
    int i;
    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> J_k_inv =
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    bool kd_research_en = true;
    const Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> identity =
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_P_k = s_next.P;
    if (usePose6LidarUpdate(ctx_)) {
        PoseJacobianMatrix H_pose;
        Eigen::VectorXd Z_pose;
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_K_x =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Zero();
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_iter_P =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> final_update_x =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>::Zero();
        bool rematch_next_iter = false;
        int converge_count = 0;

        for (i = 0; i < iter_num; i++) {
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> error_s = getErrorState(s_next, state_);
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> dx_new = error_s;
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_iter = s_next.P;
            applyPoseIterationCorrection(error_s, dx_new, P_iter);
            final_P_k = P_iter;

            const bool search_this_iter = (i == 0) || rematch_next_iter;
            last_iteration_count_ = i + 1;
            const auto previous_geometry = last_geometry_;
            const int previous_points = last_point_match_count_;
            const int previous_planes = last_plane_match_count_;
            if (!calculatePose(s_next, Dedistort_clouds_lidar, Z_pose, H_pose, search_this_iter)) {
                if (ctx_.hybrid_enabled && i > 0 && linearization_all_coincident_) {
                    // The previous valid iterate reached its matched points.
                    // Retain that iterate's H/K/P; zero-length norms have no
                    // new direction and must not turn convergence into rollback.
                    last_converged_ = true;
                    last_geometry_ = previous_geometry;
                    last_point_match_count_ = previous_points;
                    last_plane_match_count_ = previous_planes;
                    vaild_points_num_ = previous_points + previous_planes;
                    break;
                }
                state_ = prior;
                state_vector_post_.push_back(prior);

                return;
            }

            Eigen::Matrix<double, kLidarPoseDim, kLidarPoseDim> HTH =
                    Eigen::Matrix<double, kLidarPoseDim, kLidarPoseDim>::Zero();
            Eigen::Matrix<double, kLidarPoseDim, 1> Ht_h =
                    Eigen::Matrix<double, kLidarPoseDim, 1>::Zero();
            for (int r = 0; r < H_pose.rows(); ++r) {
                const double w = effect_meas_inv_var_.size() == static_cast<size_t>(H_pose.rows())
                                 ? effect_meas_inv_var_[static_cast<size_t>(r)] : 1.0;
                const Eigen::Matrix<double, 1, kLidarPoseDim> h_row = H_pose.row(r);
                const double residual = -Z_pose(r);
                HTH.noalias() += w * (h_row.transpose() * h_row);
                Ht_h.noalias() += w * h_row.transpose() * residual;
            }

            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_temp =
                    (P_iter / ctx_.lidar_prior_scale).inverse();
            P_temp.block<kLidarPoseDim, kLidarPoseDim>(StateIndex::R, StateIndex::R) += HTH;
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_post_like = P_temp.inverse();
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> K_x =
                    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Zero();
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> K_h =
                    Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>::Zero();
            K_h = P_post_like.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) * Ht_h;
            K_x.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) =
                    P_post_like.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) * HTH;
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> update_x =
                    K_h + (K_x - identity) * dx_new;

            converge = poseConverged(ctx_, update_x);

            s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
            s_next.q.normalize();
            s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
            s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
            s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
            s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
            if (ctx_.online_gravity_estimation_enable) {
                s_next.g = s_next.g + update_x.block<3,1>(StateIndex::G,0);
            }
            s_next.z  = s_next.z  + update_x(StateIndex::Z,0);
            s_next.vz = s_next.vz + update_x(StateIndex::VZ,0);
            s_next.az = s_next.az + update_x(StateIndex::AZ,0);

            final_update_x = update_x;
            final_K_x = K_x;
            final_iter_P = P_iter;

            if (converge) {
                ++converge_count;
            } else {
                converge_count = 0;
            }
            last_converged_ = converge_count > 1;
            rematch_next_iter = converge || (converge_count == 0 && i == iter_num - 2);

            if (converge_count > 1 || i == iter_num - 1) {
                break;
            }
        }

        state_ = s_next;
        state_vector_post_.push_back(s_next);
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> L = final_iter_P;
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_base = final_iter_P;
        applyPoseFinalCovarianceCorrection(final_update_x, L, P_base, final_K_x);
        state_.P = L - final_K_x * P_base;
        state_.P = 0.5 * (state_.P + state_.P.transpose());
        finishLidarUpdate(prior);

        return;
    }

    Eigen::MatrixXd K;
    Eigen::MatrixXd H_k;
    Eigen::MatrixXd Z_k;
    for (i = 0; i< iter_num; i++) {
        const Eigen::Matrix<double,static_cast<int>(StateIndex::STATE_TOTAL),1> error_s = getErrorState(s_next,state_);
        J_k_inv.setIdentity();
        J_k_inv.block<3,3>(StateIndex::R,StateIndex::R) = A_T(error_s.block<3,1>(StateIndex::R,0));
        const Eigen::MatrixXd P_k = J_k_inv * s_next.P * J_k_inv.transpose();
        const Eigen::MatrixXd P_k_inv = P_k.inverse();
        last_iteration_count_ = i + 1;
        const auto previous_geometry = last_geometry_;
        const int previous_points = last_point_match_count_;
        const int previous_planes = last_plane_match_count_;
        if (!calculate(s_next,Dedistort_clouds_lidar,Z_k,H_k,kd_research_en)) {
            if (ctx_.hybrid_enabled && i > 0 && linearization_all_coincident_) {
                last_converged_ = true;
                last_geometry_ = previous_geometry;
                last_point_match_count_ = previous_points;
                last_plane_match_count_ = previous_planes;
                vaild_points_num_ = previous_points + previous_planes;
                break;
            }
            state_ = prior;
            state_vector_post_.push_back(prior);

            return;
        }
        final_P_k = P_k;
        Eigen::MatrixXd H_kt = H_k.transpose();
        for (int row = 0; row < H_k.rows(); ++row)
            H_kt.col(row) *= effect_meas_inv_var_[static_cast<std::size_t>(row)];
        K = (H_kt * H_k + P_k_inv * ctx_.lidar_prior_scale ).inverse() * H_kt;
        Eigen::MatrixXd left = -1 * K * Z_k;
        Eigen::MatrixXd right = -1 * (identity - K * H_k) * J_k_inv * error_s;
        Eigen::MatrixXd update_x = left + right;

        converge = poseConverged(ctx_, update_x);
        last_converged_ = converge;
        kd_research_en = false;
        if (shouldRematchPose(ctx_, update_x, i, iter_num)) {
            kd_research_en = true;
        }

        s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
        s_next.q.normalize();
        s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
        s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
        s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
        s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
        if (ctx_.online_gravity_estimation_enable) {
            s_next.g = s_next.g + update_x.block<3,1>(StateIndex::G,0);
        }
        s_next.z  = s_next.z  + update_x(StateIndex::Z,0);
        s_next.vz = s_next.vz + update_x(StateIndex::VZ,0);
        s_next.az  = s_next.az  + update_x(StateIndex::AZ,0);
        if(converge){
            break;
        }
    }
    state_ = s_next;
    state_vector_post_.push_back(s_next);
    state_.P = ( Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() - K * H_k ) * final_P_k;
    finishLidarUpdate(prior);

}

void EskfEstimator::finishLidarUpdate(const State& prior) {
    const auto finite_state = [](const State& value) {
        return value.q.coeffs().allFinite() && value.p.allFinite() &&
            value.v.allFinite() && value.bw.allFinite() && value.ba.allFinite() &&
            value.g.allFinite() && value.P.allFinite() &&
            std::isfinite(value.z) && std::isfinite(value.vz) && std::isfinite(value.az);
    };
    if (!finite_state(state_)) {
        last_rejection_reason_ = "nonfinite_candidate";
    } else {
        Eigen::Vector3d position_delta = state_.p - prior.p;
        position_delta.z() += state_.z - prior.z;
        Eigen::Vector3d velocity_delta = state_.v - prior.v;
        velocity_delta.z() += state_.vz - prior.vz;
        candidate_translation_ = position_delta.norm();
        candidate_rotation_ = state_.q.angularDistance(prior.q);
        candidate_velocity_delta_ = velocity_delta.norm();
        if (ctx_.max_update_translation_m > 0.0 &&
            candidate_translation_ > ctx_.max_update_translation_m)
            last_rejection_reason_ = "update_translation_too_large";
        else if (ctx_.max_update_rotation_rad > 0.0 &&
                 candidate_rotation_ > ctx_.max_update_rotation_rad)
            last_rejection_reason_ = "update_rotation_too_large";
        else if (ctx_.max_update_velocity_delta_mps > 0.0 &&
                 candidate_velocity_delta_ > ctx_.max_update_velocity_delta_mps)
            last_rejection_reason_ = "update_velocity_delta_too_large";
        else {
            last_update_accepted_ = true;
            last_rejection_reason_ = "none";
        }
    }
    if (!last_update_accepted_) {
        state_ = prior;
        state_vector_post_.back() = prior;
    } else if (ctx_.hybrid_enabled) {
        // Last accepted prediction-to-posterior correction drives the next
        // scan's point-specific range/motion gate, as in GenZ-LIO.
        matching_deviation_rotation_ = (prior.q.conjugate() * state_.q).normalized();
        matching_deviation_translation_ = prior.q.conjugate() * (state_.p - prior.p);
    }
}

bool EskfEstimator::applyVelocityObservation(State& state, double sigma_v) const {
    if (state.in_elevator || !std::isfinite(sigma_v) || sigma_v <= 0.0) return false;
    State candidate = state;
    Eigen::Matrix<double, 3, StateIndex::STATE_TOTAL> H =
        Eigen::Matrix<double, 3, StateIndex::STATE_TOTAL>::Zero();
    H.block<3, 3>(0, StateIndex::V).setIdentity();
    const Eigen::Vector3d residual = -state.v;
    const Eigen::Matrix3d noise = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    ekfUpdate<3>(candidate, H, residual, noise);
    if (!candidate.q.coeffs().allFinite() || !candidate.p.allFinite() ||
        !candidate.v.allFinite() || !candidate.bw.allFinite() ||
        !candidate.ba.allFinite() || !candidate.g.allFinite() ||
        !candidate.P.allFinite() || !std::isfinite(candidate.z) ||
        !std::isfinite(candidate.vz) || !std::isfinite(candidate.az)) return false;
    candidate.q.normalize();
    state = std::move(candidate);
    return true;
}

Eigen::Matrix<double,StateIndex::STATE_TOTAL,1> EskfEstimator::getErrorState(const State &s1, const  State &s2){
    Eigen::Matrix<double,StateIndex::STATE_TOTAL,1> es;
    es.setZero();
    es.segment<3>(StateIndex::R)  = SO3Log((s2.q.conjugate() * s1.q).toRotationMatrix()); // δθ
    es.segment<3>(StateIndex::P)  = s1.p  - s2.p;
    es.segment<3>(StateIndex::V)  = s1.v  - s2.v;
    es.segment<3>(StateIndex::BW) = s1.bw - s2.bw;
    es.segment<3>(StateIndex::BA) = s1.ba - s2.ba;
    es.segment<3>(StateIndex::G)  = s1.g  - s2.g;
    es(StateIndex::Z)  = s1.z  - s2.z;
    es(StateIndex::VZ) = s1.vz - s2.vz;
    es(StateIndex::AZ) = s1.az - s2.az;
    return es;
}

/**
 * @brief 利用点进行平面拟合，输出为ax+by+cz+d=0，d=-1/norm，注意法向量的方向
 * @param points 用于平面拟合的点
 * @param pabcd pabcd(0) = a（法向量的 x 分量）pabcd(1) = b（法向量的 y 分量）pabcd(2) = c（法向量的 z 分量），均归一化\n
 *              pabcd(3) = -1/normal，其中 normal 是法向量的模长，用以将法向量单位化。
 * @param threhold 平面拟合的阈值
 * @return bool量，是否是平面
 */
static bool planarFittingAndCheck(const PointVector & points, Eigen::Vector4d &pabcd, float threhold) {
    Eigen::Vector3d normal_vector;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    int point_num = points.size();
    A.resize(point_num,3);
    b.resize(point_num);
    b.setOnes();
    for (int i = 0; i < point_num; i++) {
        A(i,0) = points[i].x;
        A(i,1) = points[i].y;
        A(i,2) = points[i].z;
    }
    /**
     * 解超定方程 Ax = b 【其中A的每一行是一个点，x是待拟合平面的单位法向量，b是全为1的列向量】
     * 注意，此时的 normal_vector 并不是单位向量，还需要归一化
     * 此时解的方程相当于 ax + by + cz = 1
     */
    normal_vector = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++) {
        if (fabs(normal_vector(0) * points[j].x + normal_vector(1) * points[j].y + normal_vector(2) * points[j].z - 1.0f) > threhold)
        {
            return false;
        }
    }
    double normal = normal_vector.norm();
    normal_vector.normalize();
    pabcd(0) = normal_vector(0);
    pabcd(1) = normal_vector(1);
    pabcd(2) = normal_vector(2);
    pabcd(3) = -1/normal;

    return true;
}

static bool planarFittingAndCheckFastest(const PointVector & points, Eigen::Vector4d &pabcd, double threshold_rmse) {
    int n = points.size();
    if (n < 5) return false;
    Eigen::Matrix<double, 3, 5> pts_mat;
    for(int i = 0; i < 5; ++i) {
        pts_mat.col(i) << (double)points[i].x, (double)points[i].y, (double)points[i].z;
    }
    Eigen::Vector3d center = pts_mat.rowwise().mean();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for(int i = 0; i < 5; ++i) {
        Eigen::Vector3d d = pts_mat.col(i) - center;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
    Eigen::Vector3d evals = saes.eigenvalues();
    double rmse = std::sqrt(evals[0] / 5.0);
    if (rmse > threshold_rmse) return false;
    if (evals[1] < 2.0 * evals[0]) return false;
    Eigen::Vector3d normal = saes.eigenvectors().col(0);
    if (normal.dot(center) > 0) normal = -normal;

    pabcd.block<3,1>(0,0) = normal;
    pabcd(3) = -normal.dot(center);

    return true;
}

/**
 * @brief 修正版：基于物理厚度(RMSE)的平面拟合
 * @param points 输入点
 * @param pabcd 输出参数
 * @param threshold_rmse 允许的最大均方根误差 (建议设为 0.03 ~ 0.04)
 */
static bool planarFittingAndCheck1(const PointVector & points, Eigen::Vector4d &pabcd, double threshold_rmse) {
    const int actual_n = static_cast<int>(points.size());
    if (actual_n < 4) return false;
    // Fit the neighborhood selected by near_search_num, without another cap.
    const auto pointAt = [&](int i) -> Eigen::Vector3d {
        return points[i].getVector3fMap().cast<double>();
    };

    Eigen::Vector3d normal, evals, center;
    double rmse = 0.0;
    auto runPCA = [&](int skip_idx) {
        center.setZero();
        int valid_count = 0;
        for(int i = 0; i < actual_n; ++i) {
            if (i == skip_idx) continue; // 虚拟跳过离群点
            center += pointAt(i);
            valid_count++;
        }
        center /= valid_count;

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for(int i = 0; i < actual_n; ++i) {
            if (i == skip_idx) continue;
            Eigen::Vector3d d = pointAt(i) - center;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
        evals = saes.eigenvalues();
        normal = saes.eigenvectors().col(0);
        rmse = std::sqrt(evals[0] / valid_count);
    };
    runPCA(-1); // -1 表示不跳过任何点
    if (rmse > threshold_rmse) {
        if (actual_n <= 4) return false; // 点数不足 5 个，放弃挽救

        int max_idx = -1;
        double max_dist2 = -1.0;
        for(int i = 0; i < actual_n; ++i) {
            double d_ortho = normal.dot(pointAt(i) - center);
            double d2 = d_ortho * d_ortho;
            if(d2 > max_dist2) {
                max_dist2 = d2;
                max_idx = i;
            }
        }
        runPCA(max_idx);
        if (rmse > threshold_rmse) return false;
    }
    // A zero eigengap has no unique smallest-eigenvector plane normal.
    // This tolerance handles numerical rank, not sensor or scene quality.
    const double gap_tolerance = 64.0 * std::numeric_limits<double>::epsilon() * std::max(0.0, evals[2]);
    if (evals[1] - std::max(0.0, evals[0]) <= gap_tolerance) return false;
    if (evals[1] < 2.0 * evals[0]) return false;
    if (normal.dot(center) > 0) normal = -normal;
    double d = -normal.dot(center);

    pabcd(0) = normal(0);
    pabcd(1) = normal(1);
    pabcd(2) = normal(2);
    pabcd(3) = d;

    return true;
}

EskfEstimator::ZIcpResult EskfEstimator::matchZByIcpAgainstIkdTree(
        const PointCloudXYZI &clouds_lidar, const State &initial_state) const {
    ZIcpResult result;
    if (ctx_.ikdtree.Root_Node == nullptr) {
        result.reason = "ikd-tree map is empty";
        return result;
    }
    if (clouds_lidar.size() < static_cast<size_t>(ctx_.elevator_exit_icp_z_min_effective_points)) {
        result.reason = "input cloud has too few points";
        return result;
    }

    IkdNearestQuery map_query(ctx_);
    const int knn = std::max(4, ctx_.ikdtree_match_knn);
    const Eigen::Matrix3d R_wb = initial_state.q.toRotationMatrix();
    const double max_neighbor_dist2 = ctx_.elevator_exit_icp_z_max_neighbor_distance_m *
                                      ctx_.elevator_exit_icp_z_max_neighbor_distance_m;
    double correction_z = 0.0;

    for (int iter = 0; iter < ctx_.elevator_exit_icp_z_max_iterations; ++iter) {
        double z_information = 0.0;
        double solve_hessian_z = 0.0;
        double gradient_z = 0.0;
        double squared_residual_sum = 0.0;
        int geometric_points = 0;
        int effective_points = 0;

        for (const auto &point_lidar : clouds_lidar.points) {
            const Eigen::Vector3d p_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
            const Eigen::Vector3d p_imu = ctx_.imu_R_lidar * p_lidar + ctx_.imu_t_lidar;
            Eigen::Vector3d p_world = R_wb * p_imu + initial_state.p;
            p_world.z() += correction_z;

            PointType query = point_lidar;
            query.x = static_cast<float>(p_world.x());
            query.y = static_cast<float>(p_world.y());
            query.z = static_cast<float>(p_world.z());

            PointVector nearest_points;
            std::vector<float> distance_sq;
            if (!map_query.searchKnn(query, knn, nearest_points, distance_sq) ||
                distance_sq[static_cast<size_t>(knn - 1)] > max_neighbor_dist2) {
                continue;
            }

            Eigen::Vector4d plane;
            if (!planarFittingAndCheck1(nearest_points, plane, ctx_.ikdtree_plane_fit_rmse_threshold)) {
                continue;
            }
            const double residual = plane.head<3>().dot(p_world) + plane(3);
            if (!std::isfinite(residual) || std::abs(residual) > ctx_.elevator_exit_icp_z_max_residual_m) {
                continue;
            }

            const double jacobian_z = plane(2);
            z_information += jacobian_z * jacobian_z;
            ++geometric_points;
            if (std::abs(jacobian_z) < ctx_.elevator_exit_icp_z_min_abs_normal_z) {
                continue;
            }
            const double correspondence_delta_z = -residual / jacobian_z;
            if (!std::isfinite(correspondence_delta_z) ||
                std::abs(correspondence_delta_z) >= ctx_.elevator_exit_icp_z_max_correction_m) {
                continue;
            }

            solve_hessian_z += jacobian_z * jacobian_z;
            gradient_z += jacobian_z * residual;
            squared_residual_sum += residual * residual;
            ++effective_points;
        }

        result.iterations = iter + 1;
        result.effective_points = effective_points;
        if (geometric_points < ctx_.elevator_exit_icp_z_min_effective_points) {
            result.reason = "too few geometrically valid point-plane correspondences";
            return result;
        }

        result.information_ratio = z_information / static_cast<double>(geometric_points);
        if (!std::isfinite(result.information_ratio) ||
            result.information_ratio < ctx_.elevator_exit_icp_z_min_information_ratio) {
            result.degenerate = true;
            result.reason = "z direction is geometrically degenerate";
            return result;
        }
        if (effective_points < ctx_.elevator_exit_icp_z_min_effective_points) {
            result.reason = "too few safe z point-plane correspondences";
            return result;
        }
        result.rmse = std::sqrt(squared_residual_sum / static_cast<double>(effective_points));

        const double delta_z = -gradient_z / solve_hessian_z;
        if (!std::isfinite(delta_z)) {
            result.reason = "non-finite z increment";
            return result;
        }
        const double proposed_correction = correction_z + delta_z;
        if (std::abs(proposed_correction) >= ctx_.elevator_exit_icp_z_max_correction_m) {
            result.correction_z = proposed_correction;
            result.reason = "z correction exceeds acceptance threshold";
            return result;
        }

        correction_z = proposed_correction;
        result.correction_z = correction_z;
        if (std::abs(delta_z) < ctx_.elevator_exit_icp_z_converge_m) {
            result.converged = true;
            result.accepted = true;
            result.reason = "accepted";
            return result;
        }
    }

    result.reason = "ICP did not converge";
    return result;
}

void EskfEstimator::prepareFrameCache(const PointCloudXYZI &clouds_lidar) {
    const size_t point_num = clouds_lidar.size();
    cached_point_num_ = point_num;
    point_lidar_vec_.resize(point_num);
    point_imu_vec_.resize(point_num);
    point_lidar_covariance_.resize(point_num);
    point_world_cache_.resize(point_num);
    norm_vector_.resize(point_num);
    is_effect_point_.resize(point_num, 0);
    Nearest_Points.resize(point_num);
    knn_distance_sq_.resize(point_num);
    for (size_t i = 0; i < point_num; ++i) {
        knn_distance_sq_[i].assign(static_cast<size_t>(std::max(1, ctx_.ikdtree_match_knn)), 0.0f);
    }

    for (size_t i = 0; i < point_num; ++i) {
        const Eigen::Vector3d point_lidar = clouds_lidar.points[i].getVector3fMap().cast<double>();
        point_lidar_vec_[i] = point_lidar;
        point_imu_vec_[i] = ctx_.imu_R_lidar * point_lidar + ctx_.imu_t_lidar;
        point_lidar_covariance_[i] = ctx_.lidar_prior_scale * Eigen::Matrix3d::Identity();
    }
}

void EskfEstimator::evaluatePointMatches(const State &state, PointCloudXYZI &clouds_lidar, bool kd_research_en) {
    IkdNearestQuery map_query(ctx_);

    const size_t point_num = clouds_lidar.size();
    const std::ptrdiff_t point_count = static_cast<std::ptrdiff_t>(point_num);
    const int near_points_num = std::max(1, ctx_.ikdtree_match_knn);
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();
    linearization_all_coincident_ = false;
    // A zero-distance convergence claim needs every previously valid row from
    // this scan, rather than merely one coincident point among lost matches.
    const std::vector<int> previous_effective = ctx_.hybrid_enabled && last_iteration_count_ > 1
        ? effective_indices_ : std::vector<int>{};

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for schedule(guided, 16)
#endif
    for (std::ptrdiff_t i = 0; i < point_count; ++i) {
        point_world_cache_[i] = R_wb * point_imu_vec_[i] + state.p;
    }

    const Eigen::Matrix3d R_wl = R_wb * ctx_.imu_R_lidar;
    if (!kd_research_en) {
        // Keep correspondence identity until rematch, but linearize its residual
        // and physical variance at the current iterate (as with its Jacobian).
        for (const int index : effective_indices_) {
            auto& match = norm_vector_[static_cast<std::size_t>(index)];
            const auto& point = point_world_cache_[static_cast<std::size_t>(index)];
            const Eigen::Matrix3d sensor_covariance = R_wl *
                point_lidar_covariance_[static_cast<std::size_t>(index)] * R_wl.transpose();
            if (match.point_match) {
                if (!linearizeHybridPoint(point, match.point_target, sensor_covariance,
                        match.discretization_variance, ctx_.hybrid_point_weight,
                        match.norm_vec, match.d, match.residual_variance)) {
                    is_effect_point_[static_cast<std::size_t>(index)] =
                        std::isfinite(match.d) && match.d <= 1e-9 ? 2 : 0;
                }
            } else {
                match.d = match.norm_vec.dot(point - match.plane.center);
                match.residual_variance = planeResidualVariance(match.plane, point, sensor_covariance);
            }
        }
        effective_indices_.erase(std::remove_if(effective_indices_.begin(), effective_indices_.end(),
            [&](int index) { return is_effect_point_[static_cast<std::size_t>(index)] != 1; }),
            effective_indices_.end());
        linearization_all_coincident_ = effective_indices_.empty() && !previous_effective.empty() &&
            std::all_of(previous_effective.begin(), previous_effective.end(),
                [&](int index) { return is_effect_point_[static_cast<std::size_t>(index)] == 2; });
        last_point_match_count_ = last_plane_match_count_ = 0;
        for (const auto index : effective_indices_) {
            if (norm_vector_[static_cast<std::size_t>(index)].point_match) ++last_point_match_count_;
            else ++last_plane_match_count_;
        }
        return;
    }

    std::fill(is_effect_point_.begin(), is_effect_point_.end(), static_cast<uint8_t>(0));
    effective_indices_.clear();
    reject_indices_.clear();
    PointVector queries(point_num);
    std::vector<std::uint8_t> eligible(point_num, 0);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for schedule(guided, 16)
#endif
    for (std::ptrdiff_t i = 0; i < point_count; ++i) {
        auto& point_world = queries[i];
        point_world = clouds_lidar.points[i];
        point_world.x = static_cast<float>(point_world_cache_[i].x());
        point_world.y = static_cast<float>(point_world_cache_[i].y());
        point_world.z = static_cast<float>(point_world_cache_[i].z());

        auto &points_near = Nearest_Points[i];
        auto &distance_sq = knn_distance_sq_[i];
        const bool enough_neighbors = map_query.searchKnn(point_world, near_points_num, points_near, distance_sq);
        // Immature cells may have fewer points than a plane needs; allow their
        // bounded candidates into hybrid search without changing plane-only intake.
        const bool hybrid_neighbors = ctx_.hybrid_enabled && !points_near.empty() &&
            !distance_sq.empty() && distance_sq.front() <= ctx_.ikdtree_neighbor_max_dist2;
        if (!hybrid_neighbors &&
            (!enough_neighbors || distance_sq[near_points_num - 1] > ctx_.ikdtree_neighbor_max_dist2)) {
            continue;
        }

        eligible[i] = 1;
    }

    const auto plane_batch = ctx_.ikd_map->planeCandidatesBatch(
        queries, Nearest_Points, eligible, ctx_.ikdtree_plane_fit_rmse_threshold,
        ctx_.hybrid_enabled);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for schedule(guided, 16)
#endif
    for (std::ptrdiff_t i = 0; i < point_count; ++i) {
        if (!eligible[i]) continue;
        const Eigen::Matrix3d sensor_covariance = R_wl * point_lidar_covariance_[i] * R_wl.transpose();
        const Eigen::Matrix3d pose_covariance = matchingPointPoseCovariance(point_imu_vec_[i], R_wb, state.P);
        double best_log_likelihood = -std::numeric_limits<double>::infinity();
        for (const std::size_t plane_index : plane_batch.candidate_indices[i]) {
            const auto& plane = plane_batch.planes[plane_index];
            const Eigen::Vector3d delta = point_world_cache_[i] - plane.center;
            const double pd = plane.normal.dot(delta);
            const double tangent_distance_sq = std::max(0.0, delta.squaredNorm() - pd * pd);
            // A finite-plane bound is valid for planes; point AABB/radius
            // bounds are deliberately used only for the fallback point search.
            if (ctx_.correspondence_pruning_enabled &&
                tangent_distance_sq > 9.0 * plane.radius * plane.radius) continue;
            const double residual_variance = planeResidualVariance(plane, point_world_cache_[i], sensor_covariance);
            // Query pose uncertainty belongs only in association, not in R.
            const double association_variance = residual_variance +
                (plane.normal.transpose() * pose_covariance * plane.normal).value();
            if (!std::isfinite(association_variance) || !(residual_variance > 0.0) ||
                !(association_variance > 0.0) || pd * pd >= 9.0 * association_variance ||
                tangent_distance_sq > 9.0 * plane.radius * plane.radius) continue;
            // Include the Gaussian normalization: choosing only d^2/S would
            // favor an arbitrarily uncertain plane over a precise nearby one.
            const double log_likelihood = -0.5 * (std::log(association_variance) +
                                                 pd * pd / association_variance);
            if (log_likelihood <= best_log_likelihood) continue;
            best_log_likelihood = log_likelihood;
            norm_vector_[i].norm_vec = plane.normal;
            norm_vector_[i].point_imu_xyz = point_imu_vec_[i];
            norm_vector_[i].d = pd;
            norm_vector_[i].plane = plane;
            norm_vector_[i].residual_variance = residual_variance;
            norm_vector_[i].point_match = false;
            is_effect_point_[i] = 1;
        }
        if (!is_effect_point_[i] && ctx_.hybrid_enabled) {
            const double gate = std::min(ctx_.ikdtree_neighbor_max_dist2,
                hybridPointGateSquared(point_lidar_vec_[i], matching_deviation_rotation_,
                    matching_deviation_translation_, ctx_.hybrid_point_initial_threshold_m));
            const auto match = findHybridPointMatch(point_world_cache_[i],
                plane_batch.point_buckets, plane_batch.point_bucket_indices[i], gate,
                ctx_.correspondence_pruning_enabled);
            if (!match.found) continue;
            auto& measurement = norm_vector_[i];
            measurement.discretization_variance = hybridDiscretizationVariance(
                plane_batch.candidate_roots[i], match.candidate_points);
            if (!linearizeHybridPoint(point_world_cache_[i], match.target, sensor_covariance,
                    measurement.discretization_variance, ctx_.hybrid_point_weight,
                    measurement.norm_vec, measurement.d, measurement.residual_variance)) {
                if (std::isfinite(measurement.d) && measurement.d <= 1e-9) is_effect_point_[i] = 2;
                continue;
            }
            measurement.point_match = true;
            measurement.point_target = match.target;
            measurement.point_imu_xyz = point_imu_vec_[i];
            is_effect_point_[i] = 1;
        }
    }

    effective_indices_.reserve(point_num);
    if (ctx_.down_reject_cloud_lidar) {
        reject_indices_.reserve(point_num);
    }
    for (size_t i = 0; i < point_num; ++i) {
        if (is_effect_point_[i] == 1) {
            effective_indices_.push_back(static_cast<int>(i));
        } else if (ctx_.down_reject_cloud_lidar) {
            reject_indices_.push_back(static_cast<int>(i));
        }
    }
    linearization_all_coincident_ = effective_indices_.empty() && !previous_effective.empty() &&
        std::all_of(previous_effective.begin(), previous_effective.end(),
            [&](int index) { return is_effect_point_[static_cast<std::size_t>(index)] == 2; });
    last_point_match_count_ = last_plane_match_count_ = 0;
    for (const auto index : effective_indices_) {
        if (norm_vector_[static_cast<std::size_t>(index)].point_match) ++last_point_match_count_;
        else ++last_plane_match_count_;
    }
}

void EskfEstimator::packMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                            Eigen::MatrixXd &Z, Eigen::MatrixXd &H) {
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();
    const int valid_points_num = static_cast<int>(effective_indices_.size());

    H = Eigen::MatrixXd::Zero(valid_points_num, StateIndex::STATE_TOTAL);
    Z.resize(valid_points_num, 1);
    effect_meas_inv_var_.clear();
    effect_meas_inv_var_.reserve(static_cast<size_t>(valid_points_num));

    ctx_.down_effect_cloud_lidar->clear();
    ctx_.down_effect_cloud_lidar->points.reserve(static_cast<size_t>(valid_points_num));

    for (int effect_i = 0; effect_i < valid_points_num; ++effect_i) {
        const size_t vi = static_cast<size_t>(effective_indices_[static_cast<size_t>(effect_i)]);
        ctx_.down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);

        const Eigen::Vector3d dr = -norm_vector_[vi].norm_vec.transpose() * R_wb * skew3d(norm_vector_[vi].point_imu_xyz);
        H.block<1,3>(effect_i, 0) = dr.transpose();
        H.block<1,3>(effect_i, 3) = norm_vector_[vi].norm_vec.transpose();
        H.block<1,6>(effect_i, 6).setZero();
        Z(effect_i, 0) = norm_vector_[vi].d;
        effect_meas_inv_var_.push_back(scaledPlaneInformation(ctx_.lidar_prior_scale,
                                                             norm_vector_[vi].residual_variance));
    }
    ctx_.down_effect_cloud_lidar->width = ctx_.down_effect_cloud_lidar->points.size();
    ctx_.down_effect_cloud_lidar->height = 1;
    ctx_.down_effect_cloud_lidar->is_dense = true;
}

void EskfEstimator::packPoseMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                                Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose) {
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();
    const int valid_points_num = static_cast<int>(effective_indices_.size());

    H_pose.resize(valid_points_num, kLidarPoseDim);
    Z.resize(valid_points_num);
    effect_meas_inv_var_.clear();
    effect_meas_inv_var_.reserve(static_cast<size_t>(valid_points_num));

    ctx_.down_effect_cloud_lidar->clear();
    ctx_.down_effect_cloud_lidar->points.reserve(static_cast<size_t>(valid_points_num));

    for (int effect_i = 0; effect_i < valid_points_num; ++effect_i) {
        const size_t vi = static_cast<size_t>(effective_indices_[static_cast<size_t>(effect_i)]);
        ctx_.down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);

        const Eigen::Vector3d dr = -norm_vector_[vi].norm_vec.transpose() * R_wb * skew3d(norm_vector_[vi].point_imu_xyz);
        H_pose.block<1,3>(effect_i, 0) = dr.transpose();
        H_pose.block<1,3>(effect_i, 3) = norm_vector_[vi].norm_vec.transpose();
        Z(effect_i) = norm_vector_[vi].d;
        effect_meas_inv_var_.push_back(scaledPlaneInformation(ctx_.lidar_prior_scale,
                                                             norm_vector_[vi].residual_variance));
    }
    ctx_.down_effect_cloud_lidar->width = ctx_.down_effect_cloud_lidar->points.size();
    ctx_.down_effect_cloud_lidar->height = 1;
    ctx_.down_effect_cloud_lidar->is_dense = true;
}

/**
 * @brief 将点从Lidar坐标系转化到世界坐标系
 * @param point Lidar坐标系中的点
 * @return point_world 世界坐标系中的点
 */
std::vector<Eigen::Matrix3d> EskfEstimator::mapPointCovariances(const PointCloudXYZI& points_lidar) const {
    std::vector<Eigen::Matrix3d> covariances;
    covariances.reserve(points_lidar.size());
    const Eigen::Matrix3d rotation = state_.q.toRotationMatrix();
    const Eigen::Matrix3d lidar_rotation = rotation * ctx_.imu_R_lidar;
    for (const auto& point : points_lidar) {
        const Eigen::Vector3d lidar_point(point.x, point.y, point.z);
        const Eigen::Vector3d imu_point = ctx_.imu_R_lidar * lidar_point + ctx_.imu_t_lidar;
        Eigen::Matrix3d covariance = lidar_rotation * (ctx_.lidar_prior_scale * Eigen::Matrix3d::Identity()) *
            lidar_rotation.transpose() + matchingPointPoseCovariance(imu_point, rotation, state_.P);
        covariances.push_back(0.5 * (covariance + covariance.transpose()));
    }
    return covariances;
}

PointType EskfEstimator::transLidar2World(const PointType &point) {
    Eigen::Quaterniond q = state_.q;
    Eigen::Vector3d p = state_.p;
    PointType point_world = transformPoint(transformPoint(point,ctx_.imu_R_lidar,ctx_.imu_t_lidar),q,p);
    return point_world;
}

/**
 * @brief 将点从Lidar坐标系转化到电梯坐标系 【 copy 并修改自 transLidar2World 】
 * @param points_lidar Lidar坐标系下的点云
 * @return points_elevator 电梯坐标系下的点云
 */
PointCloudXYZIPtr EskfEstimator::transLidar2Elevator(const PointCloudXYZI &points_lidar) {
    /********** ！注意是先平移后旋转！ **********/
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = ctx_.imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = ctx_.imu_t_lidar;
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state_.p;
    Eigen::Matrix4d world_T_lidar = world_T_imu * imu_T_lidar;
    PointCloudXYZIPtr points_world (new PointCloudXYZI);
    points_world->points.resize(points_lidar.size());
    for (size_t i = 0; i < points_lidar.size(); ++i) {
        const auto &pt = points_lidar.points[i];
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pw = world_T_lidar * p;
        auto &out_pt = points_world->points[i];
        out_pt.x = pw.x();
        out_pt.y = pw.y();
        out_pt.z = pw.z();
        out_pt.curvature = pt.curvature;
        out_pt.intensity = pt.intensity;
    }
    points_world->width = points_world->points.size();  // 设置 width
    points_world->height = 1;                            // 点云是无组织的（非图像结构）
    points_world->is_dense = true;                       // 没有 NaN 或无效点
    return points_world;
}

/**
 * @brief 将点云从Lidar坐标系转化到世界坐标系 【 注意，这里是手动实现，只附带了 curvature 和 intensity 信息】
 * @param points_lidar Lidar坐标系下的点云
 * @return points_world 世界坐标系下的点云
 */
PointCloudXYZIPtr EskfEstimator::transLidar2World(const PointCloudXYZI &points_lidar) {
    /********** ！注意是先平移后旋转！ **********/
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = ctx_.imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = ctx_.imu_t_lidar;
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state_.p;
    Eigen::Matrix4d world_T_lidar = world_T_imu * imu_T_lidar;
    PointCloudXYZIPtr points_world (new PointCloudXYZI);
    points_world->points.resize(points_lidar.size());
    for (size_t i = 0; i < points_lidar.size(); ++i) {
        const auto &pt = points_lidar.points[i];
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pw = world_T_lidar * p;
        auto &out_pt = points_world->points[i];
        out_pt.x = pw.x();
        out_pt.y = pw.y();
        out_pt.z = pw.z();
        out_pt.curvature = pt.curvature;
        out_pt.intensity = pt.intensity;
    }
    points_world->width = points_world->points.size();  // 设置 width
    points_world->height = 1;                            // 点云是无组织的（非图像结构）
    points_world->is_dense = true;                       // 没有 NaN 或无效点
    return points_world;
}

/**
 * @brief 根据当前状态计算点云与最近点构成平面间残差（忽略异常点），并给出雅可比矩阵用于迭代kalman滤波（类似梯度下降）
 * @param state 当前迭代状态
 * @param clouds_lidar Lidar系下当前帧点云
 * @param Z 残差
 * @param H 雅可比矩阵
 * @return bool量，表示是否有有效点
 */
bool EskfEstimator::calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H, bool kd_research_en) {
    if (cached_point_num_ != clouds_lidar.size()) {
        prepareFrameCache(clouds_lidar);
    }
    evaluatePointMatches(state, clouds_lidar, kd_research_en);

    vaild_points_num_ = static_cast<int>(effective_indices_.size());

    if (kd_research_en && ctx_.down_reject_cloud_lidar) {
        ctx_.down_reject_cloud_lidar->clear();
        ctx_.down_reject_cloud_lidar->points.reserve(reject_indices_.size());
        for (int idx : reject_indices_) {
            ctx_.down_reject_cloud_lidar->points.push_back(clouds_lidar.points[static_cast<size_t>(idx)]);
        }
        ctx_.down_reject_cloud_lidar->width = ctx_.down_reject_cloud_lidar->points.size();
        ctx_.down_reject_cloud_lidar->height = 1;
        ctx_.down_reject_cloud_lidar->is_dense = true;
    }

    if (vaild_points_num_ < 1) {
        last_geometry_ = {};
        return false;
    }

    packMeasurementMatrices(state, clouds_lidar, Z, H);
    const GeometryJacobian pose_jacobian = H.leftCols<6>();
    last_geometry_ = analyzeLocalizability(pose_jacobian, effect_meas_inv_var_,
                                         state.q.toRotationMatrix());
    return true;
}

bool EskfEstimator::calculatePose(const State &state, PointCloudXYZI &clouds_lidar,
                                  Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose, bool kd_research_en) {
    if (cached_point_num_ != clouds_lidar.size()) {
        prepareFrameCache(clouds_lidar);
    }
    evaluatePointMatches(state, clouds_lidar, kd_research_en);

    vaild_points_num_ = static_cast<int>(effective_indices_.size());

    if (kd_research_en && ctx_.down_reject_cloud_lidar) {
        ctx_.down_reject_cloud_lidar->clear();
        ctx_.down_reject_cloud_lidar->points.reserve(reject_indices_.size());
        for (int idx : reject_indices_) {
            ctx_.down_reject_cloud_lidar->points.push_back(clouds_lidar.points[static_cast<size_t>(idx)]);
        }
        ctx_.down_reject_cloud_lidar->width = ctx_.down_reject_cloud_lidar->points.size();
        ctx_.down_reject_cloud_lidar->height = 1;
        ctx_.down_reject_cloud_lidar->is_dense = true;
    }

    if (vaild_points_num_ < 1) {
        last_geometry_ = {};
        return false;
    }

    packPoseMeasurementMatrices(state, clouds_lidar, Z, H_pose);
    last_geometry_ = analyzeLocalizability(H_pose, effect_meas_inv_var_,
                                         state.q.toRotationMatrix());
    return true;
}
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_post_states() {
    return state_vector_post_;
}
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_prior_states() {
    return state_vector_prior_;
}
State EskfEstimator::get_cur_state() {
    return state_;
}
void EskfEstimator::set_cur_state(State state_now) {
    state_ = state_now;
}

} // namespace lingtu::ltlio::detail
