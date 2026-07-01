#include "ieskf.h"

#include <cmath>

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V21D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

V21D State::operator-(const State &other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
    clampCovariance();
}

void IESKF::clampCovariance()
{
    m_P = (0.5 * (m_P + m_P.transpose())).eval();
    for (int i = 0; i < 21; ++i)
    {
        if (!std::isfinite(m_P(i, i)) || m_P(i, i) < P_MIN[i])
            m_P(i, i) = P_MIN[i];
        else if (m_P(i, i) > P_MAX[i])
            m_P(i, i) = P_MAX[i];
    }
}

void IESKF::injectZUPT(double sigma_v, double sigma_pos)
{
    // Observe velocity states [12:15] with z = 0 (robot is stationary)
    Eigen::Matrix<double, 3, 21> H_zupt = Eigen::Matrix<double, 3, 21>::Zero();
    H_zupt.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    V3D innov = -m_x.v;  // z - H*x = 0 - v

    Eigen::Matrix3d S = H_zupt * m_P * H_zupt.transpose() + R_zupt;
    Eigen::Matrix<double, 21, 3> K = m_P * H_zupt.transpose() * S.inverse();

    m_x += K * innov;
    m_P = (M21D::Identity() - K * H_zupt) * m_P;

    // Tighten position covariance when robot is confirmed stationary
    if (sigma_pos > 0.0)
    {
        for (int i = 3; i < 6; ++i)
            m_P(i, i) = std::min(m_P(i, i), sigma_pos * sigma_pos);
    }
    clampCovariance();
}

void IESKF::injectVerticalVelocityConstraint(double sigma_v)
{
    if (!(sigma_v > 0.0) || !std::isfinite(sigma_v))
        return;

    Eigen::Matrix<double, 1, 21> H = Eigen::Matrix<double, 1, 21>::Zero();
    H(0, 14) = 1.0;  // v_z in the world frame
    const double R = sigma_v * sigma_v;
    const double S = (H * m_P * H.transpose())(0, 0) + R;
    if (!(S > 0.0) || !std::isfinite(S))
        return;

    Eigen::Matrix<double, 21, 1> K = m_P * H.transpose() / S;
    const double innov = -m_x.v.z();
    m_x += K * innov;
    m_P = (M21D::Identity() - K * H) * m_P;
    clampCovariance();
}

void IESKF::update()
{
    State predict_x = m_x;
    M21D P_prior = m_P;  // snapshot predict-time covariance for degenerate-DOF retention
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D delta = V21D::Zero();
    M21D H = M21D::Identity();
    V21D b;

    // Observability-Constrained state — populated on first iteration
    bool has_degeneracy = false;
    bool pathological = false;  // condition_number explodes or all 6 DOF degenerate
    Eigen::Matrix<double, 6, 6> saved_evecs = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 1> saved_mask  = Eigen::Matrix<double, 6, 1>::Ones();

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        // P^{-1} 只需解一次, 用 LDLT 代替显式求逆 (数值稳定 + 快 2-3x)
        auto P_ldlt = m_P.ldlt();
        H += J.transpose() * P_ldlt.solve(J);
        b += J.transpose() * P_ldlt.solve(delta);

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        // ── Degeneracy detection (DALI-SLAM / Zhang 2016 approach) ──────
        // Analyze the 6x6 pose block of the LiDAR Hessian (rotation + translation)
        // to detect degenerate directions where LiDAR provides no constraint.
        if (i == 0)  // Only compute on first iteration (H structure is stable)
        {
            Eigen::Matrix<double, 6, 6> H_pose = shared_data.H.block<6, 6>(0, 0);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H_pose);
            auto &evals = solver.eigenvalues();   // sorted ascending
            auto &evecs = solver.eigenvectors();

            shared_data.degeneracy.eigenvalues = evals;
            shared_data.degeneracy.min_eigenvalue = evals(0);
            shared_data.degeneracy.max_eigenvalue = evals(5);
            shared_data.degeneracy.condition_number =
                (evals(0) > 1e-10) ? evals(5) / evals(0) : 1e12;

            // Threshold: eigenvalue < 1% of max → degenerate
            // Based on Zhang 2016 "On Degeneracy of Optimization-based State Estimation"
            const double degen_thresh = evals(5) * 0.01;
            int degen_count = 0;
            Eigen::Matrix<double, 6, 1> mask = Eigen::Matrix<double, 6, 1>::Ones();

            for (int d = 0; d < 6; ++d)
            {
                if (evals(d) < degen_thresh)
                {
                    degen_count++;
                    mask(d) = 0.0;
                }
            }
            shared_data.degeneracy.degenerate_dof_count = degen_count;
            shared_data.degeneracy.dof_mask = mask;
            shared_data.degeneracy.effective_ratio =
                (6.0 - degen_count) / 6.0;
            shared_data.degeneracy.detected = (degen_count > 0);

            // Save eigenbasis for OC delta/P projection outside this block
            saved_evecs = evecs;
            saved_mask  = mask;
            has_degeneracy = (degen_count > 0);

            // ── Pathological degeneracy: eigenvectors themselves are unreliable ─
            // Only when ALL 6 DOFs are degenerate or H is near-singular do we fall
            // back to pure IMU prediction. In all other cases (including 3-5 DOF
            // degenerate, which previously skipped), we keep LiDAR constraints on
            // the observable subspace via OC delta projection below.
            const bool too_many_degenerate_dofs =
                m_degeneracy_max_update_dof >= 0 && degen_count > m_degeneracy_max_update_dof;
            const bool too_ill_conditioned =
                m_degeneracy_max_condition > 0.0
                && shared_data.degeneracy.condition_number > m_degeneracy_max_condition;
            if (degen_count >= 6 || shared_data.degeneracy.condition_number > 1e12
                || too_many_degenerate_dofs || too_ill_conditioned)
            {
                pathological = true;
                break;
            }

            // ── Partial Hessian remapping (Zhang 2016) — still useful for 1-5 DOF
            // to make the information-form solve well-conditioned. OC projection
            // on delta afterward is the safety net that guarantees state does not
            // drift along unobservable directions.
            if (degen_count > 0)
            {
                M21D H_prior = J.transpose() * P_ldlt.solve(J);
                V21D b_prior = J.transpose() * P_ldlt.solve(delta);

                Eigen::Matrix<double, 6, 6> P_good = Eigen::Matrix<double, 6, 6>::Zero();
                for (int d = 0; d < 6; ++d)
                {
                    if (evals(d) >= degen_thresh)
                        P_good += evecs.col(d) * evecs.col(d).transpose();
                }
                Eigen::Matrix<double, 6, 6> P_bad = Eigen::Matrix<double, 6, 6>::Identity() - P_good;
                double regularize = evals(5) * 0.01;
                Eigen::Matrix<double, 6, 6> H_lidar_remapped =
                    P_good * H_pose * P_good + P_bad * regularize;
                Eigen::Matrix<double, 6, 1> b_lidar_remapped =
                    P_good * shared_data.b.head<6>();

                H.block<6, 6>(0, 0) = H_prior.block<6, 6>(0, 0) + H_lidar_remapped;
                b.block<6, 1>(0, 0) = b_prior.block<6, 1>(0, 0) + b_lidar_remapped;
            }
        }

        delta = -H.ldlt().solve(b);

        // ── Observability-Constrained state update (Huang et al. 2019) ──
        // Project delta onto the observable subspace in eigenbasis coordinates.
        // Degenerate DOFs receive zero update — state along those directions is
        // carried by IMU prediction alone, not corrupted by virtual LiDAR signal.
        if (has_degeneracy)
        {
            Eigen::Matrix<double, 6, 1> dp_eig = saved_evecs.transpose() * delta.head<6>();
            for (int d = 0; d < 6; ++d)
            {
                if (saved_mask(d) < 0.5)
                    dp_eig(d) = 0.0;
            }
            delta.head<6>() = saved_evecs * dp_eig;
        }

        m_x += delta;
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    // Store degeneracy info for external access (ROS2 publisher)
    m_degeneracy = shared_data.degeneracy;
    m_degeneracy.iter_num = static_cast<int>(shared_data.iter_num);
    // Loop exited via m_stop_func only if iter_num < m_max_iter; equality means
    // we ran the max iterations without convergence — flag for diagnostics.
    m_degeneracy.converged = (shared_data.iter_num < m_max_iter);
    const V21D update_delta = m_x - predict_x;
    const bool update_translation_too_large =
        m_max_update_translation_m > 0.0
        && update_delta.segment<3>(3).norm() > m_max_update_translation_m;
    const bool update_rotation_too_large =
        m_max_update_rotation_rad > 0.0
        && update_delta.segment<3>(0).norm() > m_max_update_rotation_rad;

    // Pathological degeneracy: revert to IMU prediction entirely (eigenbasis
    // is numerically unreliable so OC projection cannot be trusted either).
    if (pathological
        || update_translation_too_large
        || update_rotation_too_large
        || (m_reject_nonconverged_update && !m_degeneracy.converged)
        || (m_reject_degenerate_nonconverged_update
            && has_degeneracy
            && !m_degeneracy.converged))
    {
        m_x = predict_x;
        clampCovariance();
        m_degeneracy.pos_cov_trace = m_P.diagonal().segment<3>(3).sum();
        return;
    }

    M21D L = M21D::Identity();
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    auto H_ldlt = H.ldlt();
    if (H_ldlt.info() != Eigen::Success || !H_ldlt.isPositive())
    {
        m_x = predict_x;
        m_P = P_prior;
        clampCovariance();
        m_degeneracy.pos_cov_trace = m_P.diagonal().segment<3>(3).sum();
        return;
    }
    M21D P_candidate = L * H_ldlt.solve(L.transpose());  // P = L * H^{-1} * L^T
    P_candidate = (0.5 * (P_candidate + P_candidate.transpose())).eval();
    if (!P_candidate.allFinite()
        || (P_candidate.diagonal().array() <= 0.0).any())
    {
        m_x = predict_x;
        m_P = P_prior;
        clampCovariance();
        m_degeneracy.pos_cov_trace = m_P.diagonal().segment<3>(3).sum();
        return;
    }
    m_P = P_candidate;

    // ── Observability-Constrained covariance update ─────────────────────
    // Posterior P for degenerate DOFs is optimistic (information-form solve
    // sees the regularized H, not the true zero-information Hessian). Retain
    // predict-time P along those directions so downstream drift monitors and
    // fusion weights see the true uncertainty.
    if (has_degeneracy)
    {
        Eigen::Matrix<double, 6, 6> P_post_pose  = m_P.block<6, 6>(0, 0);
        Eigen::Matrix<double, 6, 6> P_prior_pose = P_prior.block<6, 6>(0, 0);
        Eigen::Matrix<double, 6, 6> P_post_eig   = saved_evecs.transpose() * P_post_pose  * saved_evecs;
        Eigen::Matrix<double, 6, 6> P_prior_eig  = saved_evecs.transpose() * P_prior_pose * saved_evecs;
        for (int d = 0; d < 6; ++d)
        {
            if (saved_mask(d) < 0.5)
            {
                P_post_eig.row(d) = P_prior_eig.row(d);
                P_post_eig.col(d) = P_prior_eig.col(d);
            }
        }
        m_P.block<6, 6>(0, 0) = saved_evecs * P_post_eig * saved_evecs.transpose();
    }
    m_P = (0.5 * (m_P + m_P.transpose())).eval();
    if (!m_P.allFinite() || (m_P.diagonal().array() <= 0.0).any())
    {
        m_x = predict_x;
        m_P = P_prior;
        clampCovariance();
        m_degeneracy.pos_cov_trace = m_P.diagonal().segment<3>(3).sum();
        return;
    }

    clampCovariance();
    m_degeneracy.pos_cov_trace = m_P.diagonal().segment<3>(3).sum();
}
