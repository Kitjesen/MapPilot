#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace Sophus {

class SO3d {
public:
    SO3d() : r_(Eigen::Matrix3d::Identity()) {}
    explicit SO3d(const Eigen::Matrix3d &r) : r_(r) {}

    const Eigen::Matrix3d &matrix() const { return r_; }

    static Eigen::Matrix3d hat(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d m;
        m << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
        return m;
    }

    static SO3d exp(const Eigen::Vector3d &omega)
    {
        const double theta = omega.norm();
        const Eigen::Matrix3d omega_hat = hat(omega);
        const Eigen::Matrix3d omega_hat2 = omega_hat * omega_hat;
        Eigen::Matrix3d r = Eigen::Matrix3d::Identity();

        if (theta < 1e-10)
        {
            r += omega_hat + 0.5 * omega_hat2;
        }
        else
        {
            r += (std::sin(theta) / theta) * omega_hat
               + ((1.0 - std::cos(theta)) / (theta * theta)) * omega_hat2;
        }
        return SO3d(r);
    }

    Eigen::Vector3d log() const
    {
        Eigen::AngleAxisd aa(r_);
        if (aa.angle() < 1e-10)
            return vee(0.5 * (r_ - r_.transpose()));
        return aa.angle() * aa.axis();
    }

    static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d &omega)
    {
        const double theta = omega.norm();
        const Eigen::Matrix3d omega_hat = hat(omega);
        const Eigen::Matrix3d omega_hat2 = omega_hat * omega_hat;
        if (theta < 1e-10)
            return Eigen::Matrix3d::Identity() + 0.5 * omega_hat + (1.0 / 6.0) * omega_hat2;
        return Eigen::Matrix3d::Identity()
             + ((1.0 - std::cos(theta)) / (theta * theta)) * omega_hat
             + ((theta - std::sin(theta)) / (theta * theta * theta)) * omega_hat2;
    }

    static Eigen::Matrix3d leftJacobianInverse(const Eigen::Vector3d &omega)
    {
        const double theta = omega.norm();
        const Eigen::Matrix3d omega_hat = hat(omega);
        const Eigen::Matrix3d omega_hat2 = omega_hat * omega_hat;
        if (theta < 1e-10)
            return Eigen::Matrix3d::Identity() - 0.5 * omega_hat + (1.0 / 12.0) * omega_hat2;
        const double theta2 = theta * theta;
        const double scale = (1.0 / theta2) - ((1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta)));
        return Eigen::Matrix3d::Identity() - 0.5 * omega_hat + scale * omega_hat2;
    }

private:
    static Eigen::Vector3d vee(const Eigen::Matrix3d &m)
    {
        return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
    }

    Eigen::Matrix3d r_;
};

}  // namespace Sophus
