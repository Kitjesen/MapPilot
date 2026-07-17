#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;


template <typename T>
using Vec = std::vector<T>;


bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    double cube_len = 2000;
    double det_range = 60;
    double move_thresh = 1.5;
    int max_map_points = 5000000;    // ikd-tree 点数硬上限，0=不限制
    double stationary_thresh = 0.05; // 静止检测阈值(米)，0=不启用

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    int imu_init_num = 20;
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    int degeneracy_max_update_dof = 2;
    double degeneracy_max_condition = 50000.0;
    double max_update_translation_m = 0.5;
    double max_update_rotation_rad = 0.35;
    double max_update_velocity_mps = 3.0;
    double max_update_velocity_delta_mps = 1.0;
    // Reaching the iteration limit is advisory when the LiDAR residual is
    // valid, bounded, and observable.  Rejecting every such update reduces
    // FAST-LIO to IMU propagation in scenes that need one more iteration.
    bool reject_nonconverged_update = false;
    bool reject_degenerate_nonconverged_update = true;
    bool gravity_align = true;
    bool esti_il = false;
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();

    double lidar_cov_inv = 1000.0;

    // ZUPT (Zero velocity UPdaTe) parameters — inject pseudo-observation when stationary
    double imu_static_acc_thresh  = 0.04;   // m/s² — acc variance threshold for static detection
    double imu_static_gyro_thresh = 0.001;  // rad/s — gyro variance threshold
    int    zupt_min_static_frames = 5;      // consecutive undistort() calls required to trigger ZUPT
    double zupt_sigma_v           = 0.02;   // ZUPT velocity measurement noise (m/s)
    double zupt_sigma_pos         = 0.1;    // ZUPT position covariance ceiling (m)

    // Optional ground-robot vertical-velocity pseudo-observation. This is
    // disabled by default because real off-road motion can include vertical
    // velocity on slopes; simulation gates may enable it for planar kinematic
    // MuJoCo motion where the IMU model is gravity-only.
    bool   vertical_velocity_constraint_enabled = false;
    double vertical_velocity_sigma_v            = 0.05;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};
