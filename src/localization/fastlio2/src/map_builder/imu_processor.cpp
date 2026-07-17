#include <cmath>
#include "imu_processor.h"

namespace
{
constexpr int kGyroBiasMinStationaryFrames = 50;
constexpr double kGyroBiasMaxResidualRadPerS = 0.001;
constexpr double kGyroBiasAdaptationTauS = 5.0;
constexpr double kGyroBiasMaxCorrectionRateRadPerS2 = 2e-4;
constexpr double kGyroBiasMaxCumulativeCorrectionRadPerS = 0.01;
constexpr double kGyroBiasMaxStationaryFrameGapS = 0.5;
}

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_imu_cache.clear();
    m_poses_cache.clear();
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
        return false;
    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (const auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());
    const double acc_norm = acc_mean.norm();
    if (!std::isfinite(acc_norm) || acc_norm < 1e-6)
    {
        m_imu_cache.clear();
        return false;
    }
    // FAST-LIO normalizes raw accelerometer magnitude using the stationary
    // initialization mean. Without this, a small scale error integrates into
    // unbounded velocity and position drift even while the robot is static.
    m_acc_scale = State::gravity / acc_norm;
    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;
    m_kf->x().bg = gyro_mean;
    m_initial_gyro_bias = gyro_mean;
    m_bias_static_frame_count = 0;
    m_last_bias_stationary_time_s = -1.0;
    if (m_config.gravity_align)
    {
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
        m_kf->x().initGravityDir(-acc_mean);
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{

    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = m_imu_cache.front().time;
    const double imu_time_end = m_imu_cache.back().time;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    m_poses_cache.clear();
    m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc * m_acc_scale;
    inp.gyro = m_imu_cache.back().gyro;
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);
        if (tail.time < m_last_propagate_end_time)
            continue;
        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val = 0.5 * (head.acc + tail.acc) * m_acc_scale;

        if (head.time < m_last_propagate_end_time)
            dt = tail.time - m_last_propagate_end_time;
        else
            dt = tail.time - head.time;

        inp.acc = acc_val;
        inp.gyro = gyro_val;
        m_kf->predict(inp, dt, m_Q);

        m_last_gyro = gyro_val - m_kf->x().bg;
        m_last_acc = m_kf->x().r_wi * (acc_val - m_kf->x().ba) + m_kf->x().g;
        double offset = tail.time - cloud_time_begin;
        m_poses_cache.emplace_back(offset, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    if (m_config.vertical_velocity_constraint_enabled)
        m_kf->injectVerticalVelocityConstraint(m_config.vertical_velocity_sigma_v);

    checkIMUStationary(m_imu_cache);

    M3D cur_r_wi = m_kf->x().r_wi;
    V3D cur_t_wi = m_kf->x().t_wi;
    M3D cur_r_il = m_kf->x().r_il;
    V3D cur_t_il = m_kf->x().t_il;
    auto it_pcl = package.cloud->points.end() - 1;

    for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == package.cloud->points.begin())
                break;
        }
    }
}

void IMUProcessor::checkIMUStationary(const Vec<IMUData> &batch)
{
    if (batch.size() < 2)
    {
        m_bias_static_frame_count = 0;
        m_last_bias_stationary_time_s = -1.0;
        return;
    }

    // Compute mean of acc and gyro over the batch
    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (const auto &imu : batch)
    {
        acc_mean += imu.acc * m_acc_scale;
        gyro_mean += imu.gyro;
    }
    const double n = static_cast<double>(batch.size());
    acc_mean /= n;
    gyro_mean /= n;
    const V3D gyro_residual_mean = gyro_mean - m_kf->x().bg;

    // Compute variance
    double acc_var = 0.0;
    double gyro_var = 0.0;
    for (const auto &imu : batch)
    {
        acc_var  += (imu.acc * m_acc_scale - acc_mean).squaredNorm();
        gyro_var += (imu.gyro - gyro_mean).squaredNorm();
    }
    acc_var  /= n;
    gyro_var /= n;

    const double acc_thresh2 = m_config.imu_static_acc_thresh * m_config.imu_static_acc_thresh;
    const double gyro_thresh2 = m_config.imu_static_gyro_thresh * m_config.imu_static_gyro_thresh;
    const bool low_variance = acc_var < acc_thresh2 && gyro_var < gyro_thresh2;
    const bool near_static_mean =
        std::abs(acc_mean.norm() - State::gravity) < m_config.imu_static_acc_thresh &&
        gyro_residual_mean.norm() < m_config.imu_static_gyro_thresh;

    if (low_variance && near_static_mean)
    {
        m_static_frame_count++;
        if (m_static_frame_count >= m_config.zupt_min_static_frames)
        {
            m_kf->injectZUPT(m_config.zupt_sigma_v, m_config.zupt_sigma_pos);
            // Keep counter at threshold — continue injecting ZUPT every frame while static
            m_static_frame_count = m_config.zupt_min_static_frames;
        }

        const double gyro_residual_norm = gyro_residual_mean.norm();
        const double stationary_time_s = batch.back().time;
        if (!std::isfinite(gyro_residual_norm) ||
            gyro_residual_norm > kGyroBiasMaxResidualRadPerS ||
            !std::isfinite(stationary_time_s))
        {
            m_bias_static_frame_count = 0;
            m_last_bias_stationary_time_s = -1.0;
            return;
        }

        double elapsed_s = 0.0;
        if (m_last_bias_stationary_time_s >= 0.0)
        {
            elapsed_s = stationary_time_s - m_last_bias_stationary_time_s;
            if (!std::isfinite(elapsed_s) || elapsed_s <= 0.0)
            {
                m_bias_static_frame_count = 0;
                m_last_bias_stationary_time_s = -1.0;
                return;
            }
            if (elapsed_s > kGyroBiasMaxStationaryFrameGapS)
            {
                // A long sensor or callback outage breaks the continuous-static
                // evidence. Treat the current frame as the first frame of a new
                // stationary streak and never integrate the full gap into bg.
                m_bias_static_frame_count = 1;
                m_last_bias_stationary_time_s = stationary_time_s;
                return;
            }
        }
        m_last_bias_stationary_time_s = stationary_time_s;
        m_bias_static_frame_count++;
        if (m_bias_static_frame_count < kGyroBiasMinStationaryFrames || elapsed_s <= 0.0)
            return;

        // Both gyro_mean and bg are expressed in the IMU frame. Apply a slow
        // zero-angular-rate update only after the existing stationary gate has
        // remained true long enough; do not touch pose, velocity, or covariance.
        V3D correction =
            (1.0 - std::exp(-elapsed_s / kGyroBiasAdaptationTauS)) *
            gyro_residual_mean;
        const double max_step = kGyroBiasMaxCorrectionRateRadPerS2 * elapsed_s;
        const double correction_norm = correction.norm();
        if (correction_norm > max_step && correction_norm > 0.0)
            correction *= max_step / correction_norm;

        V3D cumulative_correction =
            m_kf->x().bg + correction - m_initial_gyro_bias;
        const double cumulative_norm = cumulative_correction.norm();
        if (cumulative_norm > kGyroBiasMaxCumulativeCorrectionRadPerS)
        {
            cumulative_correction *=
                kGyroBiasMaxCumulativeCorrectionRadPerS / cumulative_norm;
        }
        m_kf->x().bg = m_initial_gyro_bias + cumulative_correction;
    }
    else
    {
        m_static_frame_count = 0;
        m_bias_static_frame_count = 0;
        m_last_bias_stationary_time_s = -1.0;
    }
}
