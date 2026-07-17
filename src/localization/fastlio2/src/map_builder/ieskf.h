#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

// Degeneracy detection result — DALI-SLAM inspired Jacobian remapping
struct DegeneracyInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool detected = false;           // true if any DOF is degenerate
    double condition_number = 0.0;   // H condition number (high = bad)
    double min_eigenvalue = 0.0;     // smallest eigenvalue of H_pose (6x6)
    double max_eigenvalue = 0.0;     // largest eigenvalue
    double effective_ratio = 1.0;    // ratio of well-conditioned DOFs (0~1)
    int degenerate_dof_count = 0;    // number of degenerate DOFs (0~6)
    Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
    // Per-DOF degeneracy mask: 1.0 = well-constrained, 0.0 = degenerate
    Eigen::Matrix<double, 6, 1> dof_mask = Eigen::Matrix<double, 6, 1>::Ones();
    // Position covariance trace (m²) — sum of t_wi diagonals after the update.
    // Surfacing this lets external monitors detect IEKF divergence ~30-60s before
    // pose itself blows up, which is when watchdog finally trips on |xy|.
    double pos_cov_trace = 0.0;
    // Iterations actually used in the last update. iter_num == m_max_iter without
    // converging suggests the Jacobian is ill-conditioned for current observations.
    int iter_num = 0;
    bool converged = true;           // false if loop exited at m_max_iter without stop_func
};

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
    DegeneracyInfo degeneracy;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const V21D &delta);

    V21D operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
public:
    IESKF() = default;
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    void setDegeneracyGuard(
        int max_update_dof,
        double max_condition,
        double max_update_translation_m,
        double max_update_rotation_rad,
        double max_update_velocity_mps,
        double max_update_velocity_delta_mps,
        bool reject_nonconverged_update,
        bool reject_degenerate_nonconverged_update)
    {
        m_degeneracy_max_update_dof = max_update_dof;
        m_degeneracy_max_condition = max_condition;
        m_max_update_translation_m = max_update_translation_m;
        m_max_update_rotation_rad = max_update_rotation_rad;
        m_max_update_velocity_mps = max_update_velocity_mps;
        m_max_update_velocity_delta_mps = max_update_velocity_delta_mps;
        m_reject_nonconverged_update = reject_nonconverged_update;
        m_reject_degenerate_nonconverged_update = reject_degenerate_nonconverged_update;
    }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    // Returns true only when a LiDAR correction was accepted. Callers must not
    // integrate the current scan into the local map after a rejected update.
    bool update();

    // Clamp P into a numerically valid covariance envelope.
    void clampCovariance();

    // ZUPT: inject zero-velocity pseudo-observation to constrain drift during static periods
    void injectZUPT(double sigma_v = 0.02, double sigma_pos = 0.1);

    // Ground-robot vertical velocity constraint: observe world-frame v_z = 0
    // without constraining x/y velocity or z position.
    void injectVerticalVelocityConstraint(double sigma_v = 0.05);

    State &x() { return m_x; }

    M21D &P() { return m_P; }

    const DegeneracyInfo &degeneracy() const { return m_degeneracy; }

    // Per-dimension P diagonal upper bounds for 21-state IESKF:
    // [θ_wi(0:3), t_wi(3:6), θ_il(6:9), t_il(9:12), v(12:15), bg(15:18), ba(18:21)]
    static constexpr double P_MAX[21] = {
        9.87,  9.87,  9.87,   // θ_wi  (π² rad²)
        1e4,   1e4,   1e4,    // t_wi  (100 m std²)
        1e-4,  1e-4,  1e-4,   // θ_il  extrinsic rotation
        1e-2,  1e-2,  1e-2,   // t_il  extrinsic translation
        1e2,   1e2,   1e2,    // v     (10 m/s std²)
        1e-2,  1e-2,  1e-2,   // bg    gyro bias
        1.0,   1.0,   1.0,    // ba    acc bias
    };
    static constexpr double P_MIN[21] = {
        1e-12, 1e-12, 1e-12,
        1e-9,  1e-9,  1e-9,
        1e-12, 1e-12, 1e-12,
        1e-12, 1e-12, 1e-12,
        1e-9,  1e-9,  1e-9,
        1e-12, 1e-12, 1e-12,
        1e-12, 1e-12, 1e-12,
    };

private:
    size_t m_max_iter = 10;
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
    DegeneracyInfo m_degeneracy;
    int m_degeneracy_max_update_dof = 2;
    double m_degeneracy_max_condition = 50000.0;
    double m_max_update_translation_m = 0.5;
    double m_max_update_rotation_rad = 0.35;
    double m_max_update_velocity_mps = 3.0;
    double m_max_update_velocity_delta_mps = 1.0;
    bool m_reject_nonconverged_update = false;
    bool m_reject_degenerate_nonconverged_update = true;
};
