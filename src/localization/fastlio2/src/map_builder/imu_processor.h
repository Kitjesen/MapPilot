#pragma once
#include "ieskf.h"
#include "commons.h"

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    // Progress of IMU initialisation as (samples_collected, samples_required).
    // Used by lio_node.cpp to log a stuck-init warning when the cache fills
    // too slowly (no IMU stream, wrong topic, or driver crashed).
    std::pair<int, int> initProgress() const {
        return {static_cast<int>(m_imu_cache.size()), m_config.imu_init_num};
    }

    double accelerationScale() const { return m_acc_scale; }

private:
    void checkIMUStationary(const Vec<IMUData> &batch);

    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;
    double m_acc_scale = 1.0;

    // ZUPT static detection state
    int m_static_frame_count = 0;

    // ZARU gyro-bias adaptation state. Biases stay in the IMU frame.
    V3D m_initial_gyro_bias = V3D::Zero();
    int m_bias_static_frame_count = 0;
    double m_last_bias_stationary_time_s = -1.0;
};
