#pragma once
#include "Types.h"
#include "ikd_Tree.h"
#include "IkdMap.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
namespace lingtu::ltlio::detail {
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
inline const Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
inline Eigen::Matrix3d skew3d(const Eigen::Vector3d &a) {
    Eigen::Matrix3d A;
    A << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
    return A;
}
inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
    Eigen::Matrix3d  SO3;
    double so3_norm = so3.norm();
    if (so3_norm<=0.0000001)
    {
        SO3.setIdentity();
        return SO3;
    }

    Eigen::Matrix3d so3_skew_sym = skew3d(so3);
    SO3 = Eigen::Matrix3d::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym/(so3_norm*so3_norm))*(1-cos(so3_norm));
    return SO3;
}


struct Frame {
    PointCloudXYZI::Ptr cloud;
    double base_time = 0.0, end_time = 0.0;
    std::size_t input_points = 0;
};
// All formerly mutable upstream globals belong to exactly one frontend.
struct Context {
double acc_scale = 1.0;
double acc_noise_variance;
double gyro_noise_variance;
double acc_bias_noise_variance;
double gyro_bias_noise_variance;
Eigen::Vector3d imu_t_lidar = Eigen::Vector3d::Zero();
Eigen::Matrix3d imu_R_lidar = Eigen::Matrix3d::Identity();
std::string lidar_update_mode = "pose6";
bool online_gravity_estimation_enable = false;
int lidar_ieskf_max_iterations;
double lidar_prior_scale;
double max_update_translation_m;
double max_update_rotation_rad;
double max_update_velocity_delta_mps;
double imu_static_acc_thresh = 0.0;
double imu_static_gyro_thresh = 0.0;
double lidar_ieskf_converge_rot_rad = 1e-3;
double lidar_ieskf_converge_pos_m = 1e-3;
double lidar_ieskf_rematch_rot_rad = 2e-3;
double lidar_ieskf_rematch_pos_m = 2e-3;
int ikdtree_match_knn;
double ikdtree_plane_fit_rmse_threshold = 0.05;
double ikdtree_neighbor_max_dist2 = 5.0;
std::string ikdtree_gate_mode = "linear_range";
double ikdtree_score_min = 0.95;
double ikdtree_match_thresh_base = 0.03;
double ikdtree_match_thresh_scale = 0.01;
double elevator_exit_icp_z_max_correction_m = 0.3;
int elevator_exit_icp_z_max_iterations = 5;
int elevator_exit_icp_z_min_effective_points = 20;
double elevator_exit_icp_z_min_information_ratio = 0.1;
double elevator_exit_icp_z_max_neighbor_distance_m = 0.5;
double elevator_exit_icp_z_min_abs_normal_z = 0.7;
double elevator_exit_icp_z_max_residual_m = 0.3;
double elevator_exit_icp_z_converge_m = 1e-3;
bool time_log_enable = false;
bool log_save_enable = false;
bool res_wheel_log_enable = false;
bool CovMatrix_vis = false;

bool elevator_enable;
bool ELEVATOR_TRIGGER = false;
bool EXIT_FROM_ELEVATOR = false;
// Coordinate correction only: the same posterior remains authoritative for maps.
Eigen::Isometry3d world_from_odom = Eigen::Isometry3d::Identity();

bool elevator_strong_prior_enable = false;
double elevator_strong_prior_acc_var = 0.001;
bool elevator_zupt_enable = true;
bool elevator_waiting_zupt_enable = true;
bool self_exit_detector = true;
double elevator_waiting_zupt_vz_abs_thresh = 0.1;
double elevator_waiting_zupt_az_abs_thresh = 0.1;
double elevator_waiting_zupt_period_s = 3.0;
bool elevator_door_detector_enable = true;
double elevator_door_filter_percent = 0.06;
double elevator_door_dist_threshold;
double elevator_door_cooldown_time = 5.0;
double elevator_door_time_threshold;
double elevator_entry_motion_m;
int point_filter_num;
double blind = 0.8;
bool use_box_blind = false;

float filter_size = 0.1f;
bool downsample_adaptive_enabled = true;
bool scale_aware_sampling_enabled;
bool hybrid_enabled;
bool correspondence_pruning_enabled;
double hybrid_point_weight;
double hybrid_point_initial_threshold_m;
double downsample_target_points = 20000;
double downsample_alpha = 1.2;
double downsample_min_voxel = 0.05;
double downsample_max_voxel = 0.8;
double local_map_cube_len;
double min_filter_size;
double stationary_thresh;
double map_detect_range;
bool relocation_enable = false;
bool global_map_pub_enable = false;
bool elevator_rebuild_map_on_exit_enable = false;
int elevator_rebuild_map_on_exit_frames = 5;
int elevator_rebuild_map_on_exit_min_points = 2000;
bool elevator_exit_icp_z_enable = false;
bool high_frequency_odom = false;
double lidar_frequency = 10.0;
bool trajectory_log_enable = false;

    KD_TREE<PointType> ikdtree;
    std::shared_ptr<IkdMap<PointType>> ikd_map;
    PointCloudXYZI::Ptr down_effect_cloud_lidar{new PointCloudXYZI};
    PointCloudXYZI::Ptr down_reject_cloud_lidar{new PointCloudXYZI};
    std::deque<Frame> frames;
    double lidar_end_time = 0.0;
    explicit Context(const ::lingtu::ltlio::Config& config) {
        acc_scale = config.acc_scale;
        acc_noise_variance = config.acc_noise_variance;
        gyro_noise_variance = config.gyro_noise_variance;
        acc_bias_noise_variance = config.acc_bias_noise_variance;
        gyro_bias_noise_variance = config.gyro_bias_noise_variance;
        imu_R_lidar = config.imu_from_lidar_rotation;
        imu_t_lidar = config.imu_from_lidar_translation;
        elevator_enable = config.elevator_enabled;
        elevator_door_detector_enable = config.automatic_door;
        elevator_door_dist_threshold = config.elevator_entry_range_m;
        elevator_door_time_threshold = config.elevator_entry_hold_s;
        elevator_entry_motion_m = config.elevator_entry_motion_m;
        point_filter_num = config.point_filter_num;
        blind = config.blind_m;
        scale_aware_sampling_enabled = config.scale_aware_sampling_enabled;
        hybrid_enabled = config.hybrid_enabled;
        correspondence_pruning_enabled = config.correspondence_pruning_enabled;
        hybrid_point_weight = config.hybrid_point_weight;
        hybrid_point_initial_threshold_m = config.hybrid_point_initial_threshold_m;
        if (config.scan_resolution > 0.0) {
            downsample_adaptive_enabled = false;
            filter_size = static_cast<float>(config.scan_resolution);
        }
        min_filter_size = config.map_resolution;
        local_map_cube_len = config.cube_len;
        stationary_thresh = config.stationary_thresh;
        map_detect_range = config.det_range;
        ikdtree_match_knn = config.near_search_num;
        lidar_ieskf_max_iterations = config.ieskf_max_iter;
        lidar_prior_scale = 1.0 / config.lidar_cov_inv;
        max_update_translation_m = config.max_update_translation_m;
        max_update_rotation_rad = config.max_update_rotation_rad;
        max_update_velocity_delta_mps = config.max_update_velocity_delta_mps;
        imu_static_acc_thresh = config.imu_static_acc_thresh;
        imu_static_gyro_thresh = config.imu_static_gyro_thresh;
        IkdMap<PointType>::Config map_config;
        map_config.knn = config.near_search_num;
        map_config.local_map_cube_len = local_map_cube_len;
        map_config.local_map_detect_range = config.det_range;
        map_config.local_map_move_threshold = config.move_thresh;
        map_config.max_map_points = config.max_map_points;
        ikd_map = std::make_shared<IkdMap<PointType>>(map_config, &ikdtree);
        ikd_map->init(min_filter_size);
    }
};
} // namespace lingtu::ltlio::detail
