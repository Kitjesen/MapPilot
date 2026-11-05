// Native interface to Elevator-LIO 1d79af77f3d9747ea57ef52a9b01d326a8ec561a.
// Numerical implementation provenance and licenses remain beside the sources.
#pragma once

#include <lingtu/settings.hpp>

#include "geometry_diagnostics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::ltlio {

namespace slam_defaults = lingtu::settings::slam::slam;

using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;

struct Config {
    Eigen::Matrix3d imu_from_lidar_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu_from_lidar_translation = Eigen::Vector3d::Zero();
    bool elevator_enabled = std::string_view{slam_defaults::elevator_mode} != "off";
    bool automatic_door = std::string_view{slam_defaults::elevator_mode} == "auto";
    double elevator_entry_range_m = slam_defaults::elevator_entry::range_m;
    double elevator_entry_hold_s = slam_defaults::elevator_entry::hold_s;
    double elevator_entry_motion_m = slam_defaults::elevator_entry::motion_m; // Maximum 3D excursion during the entry hold.
    bool estimate_extrinsics = slam_defaults::esti_il; // Unsupported: constructor rejects true.
    std::size_t imu_init_samples = slam_defaults::imu_init_num;
    int point_filter_num = slam_defaults::lidar_filter_num;
    double blind_m = slam_defaults::lidar_min_range;
    double max_range_m = slam_defaults::lidar_max_range;
    double scan_resolution = slam_defaults::scan_resolution; // Zero retains point-count adaptive sampling.
    // Independently selectable for fixed-input ablations before field acceptance.
    // With scale-aware sampling, scan_resolution is the initial voxel size.
    bool scale_aware_sampling_enabled = slam_defaults::scale_aware_sampling_enabled;
    bool hybrid_enabled = slam_defaults::hybrid_enabled;
    bool correspondence_pruning_enabled = slam_defaults::correspondence_pruning_enabled;
    double hybrid_point_weight = slam_defaults::hybrid_point_weight;
    double hybrid_point_initial_threshold_m = slam_defaults::hybrid_point_initial_threshold_m;
    double map_resolution = slam_defaults::map_resolution;
    double cube_len = slam_defaults::cube_len;
    double det_range = slam_defaults::det_range;
    double move_thresh = slam_defaults::move_thresh;
    int max_map_points = slam_defaults::max_map_points;
    double stationary_thresh = slam_defaults::stationary_thresh; // Map insertion distance, not a ZUPT threshold.
    int near_search_num = slam_defaults::near_search_num; // Neighbors nominate cells with retained plane support.
    int ieskf_max_iter = slam_defaults::ieskf_max_iter;
    double lidar_cov_inv = slam_defaults::lidar_cov_inv;
    double max_update_translation_m = slam_defaults::max_update_translation_m;
    double max_update_rotation_rad = slam_defaults::max_update_rotation_rad;
    double max_update_velocity_delta_mps = slam_defaults::max_update_velocity_delta_mps;
    // Zero disables the additional stationary-window checks for exact upstream replay.
    double imu_static_acc_thresh = 0.0;
    double imu_static_gyro_thresh = 0.0;
    int zupt_min_static_frames = slam_defaults::zupt_min_static_frames;
    double zupt_sigma_v = slam_defaults::zupt_sigma_v;
    double acc_scale = 1.0; // Input acceleration times this scale is SI m/s².
    // Continuous-time IMU process-noise coefficients (SI density squared).
    // YAML na/ng/nba/nbg remain densities; the backend squares them once.
    double acc_noise_variance = 1e-4;
    double gyro_noise_variance = 1e-5;
    double acc_bias_noise_variance = 1e-6;
    double gyro_bias_noise_variance = 1e-6;
    std::size_t max_imu_buffer = slam_defaults::max_imu_buffer;
    std::size_t max_lidar_buffer = slam_defaults::max_lidar_buffer;
};

struct ImuSample {
    double stamp_s = 0.0;
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); // Units set by Config::acc_scale.
    Eigen::Vector3d angular_velocity_rps = Eigen::Vector3d::Zero();
};

struct TimedPoint {
    float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
    std::uint64_t offset_ns = 0;
    std::uint8_t line = 0;
    std::uint8_t tag = 0;
};

struct TimedCloud {
    double stamp_s = 0.0;
    // Original frame end, before point selection. Zero derives it from the
    // last raw point's offset, as the upstream Livox intake does.
    double end_stamp_s = 0.0;
    std::vector<TimedPoint> points;
};

// One native intake for estimated and externally supplied motion. Curvature
// stores milliseconds; the caller keeps the unfiltered scan endpoint.
Cloud::Ptr selectPoints(const TimedCloud& cloud, const Config& config);

struct State {
    double stamp_s = 0.0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    // Cabin-relative position/velocity while in_elevator; world otherwise.
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.80665);
    double transport_height = 0.0; // Current ride; absorbed into position on exit.
    double transport_velocity = 0.0;
    double transport_acceleration = 0.0;
    bool in_elevator = false;
    // Official order: R,P,V,BW,BA,G,Z,VZ,AZ = 0,3,6,9,12,15,18,19,20.
    Eigen::Matrix<double, 21, 21> covariance = Eigen::Matrix<double, 21, 21>::Zero();
};

struct ElevatorDiagnostics {
    // Phase immediately before this scan's LiDAR/stop/exit update, matching
    // upstream numerical replay diagnostics. Other fields describe the result.
    int phase = 0; // IDLE, ACCELERATING, CONSTANT_SPEED, DECELERATING, STOP_CHECK.
    double velocity_variance = 0.0;
    double range_quantile_m = -1.0;
    bool stop_confirmed = false;
    int stop_updates = 0;
    std::uint64_t entries = 0;
    std::uint64_t exits = 0;
};

struct Result {
    double scan_start_s = 0.0;
    double scan_end_s = 0.0;
    bool initialized = false;
    bool map_ready = false;
    bool attempted = false;
    bool accepted = false;
    int iterations = 0;
    bool converged = false;
    std::string rejection_reason = "none";
    double candidate_translation_m = 0.0;
    double candidate_rotation_rad = 0.0;
    double candidate_velocity_delta_mps = 0.0;
    std::size_t input_points = 0;
    std::size_t sampled_points = 0;
    std::size_t effective_points = 0;
    std::size_t plane_matches = 0;
    std::size_t point_matches = 0;
    std::size_t mapping_points = 0;
    double sampling_leaf_size_m = 0.0;
    double sampling_scale_m = 0.0;
    GeometryDiagnostics geometry;
    State state;
    Eigen::Vector3d world_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
    // Map/world posterior from continuous odometry. This transform carries
    // transport stop corrections, not another estimate or smoothed state.
    Eigen::Isometry3d world_from_odom = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d world_position_covariance = Eigen::Matrix3d::Zero();
    // Latest consumed real IMU at or before scan_end, in IMU coordinates.
    double causal_gyro_stamp_s = 0.0;
    Eigen::Vector3d causal_gyro_rps = Eigen::Vector3d::Zero();
    // LiDAR coordinates at scan_end; upstream deliberately skips deskew in
    // active elevator mode. An independent snapshot, never mutated later.
    Cloud::ConstPtr deskewed_cloud;
    ElevatorDiagnostics elevator;
};

struct PosePrior {
    Eigen::Quaterniond world_imu_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d world_imu_position = Eigen::Vector3d::Zero();
    std::optional<Eigen::Vector3d> world_imu_velocity;
};

// One caller serializes feed/process/reset. Separate instances share no state.
class Frontend {
public:
    explicit Frontend(const Config& config = Config{});
    ~Frontend();
    Frontend(const Frontend&) = delete;
    Frontend& operator=(const Frontend&) = delete;
    Frontend(Frontend&&) noexcept;
    Frontend& operator=(Frontend&&) noexcept;

    void feedImu(const ImuSample& sample);
    void feedCloud(const TimedCloud& cloud);
    // Consume real IMUs until the oldest cloud is covered. Initializer/map
    // accumulation frames also return status; nullopt means no ready frame.
    // A time-aligned external pose is resolved after matching and before the
    // single state/map commit, so its body/world conversion uses causal IMU.
    // Waiting-ZUPT evidence is requested by IMU samples and applied only to an
    // accepted scan at or after that request, before resolving the external pose.
    std::optional<Result> processNext(
        const std::function<std::optional<PosePrior>(const Result&)>& prior = {});
    // Existing external pose input corrects the same historical filter state.
    // Rotation/position and optional velocity are at the IMU origin in world.
    bool correctState(double stamp_s, const Eigen::Quaterniond& world_imu_rotation,
                      const Eigen::Vector3d& world_imu_position,
                      const std::optional<Eigen::Vector3d>& world_imu_velocity = std::nullopt);
    void requestElevator(bool active);
    void reset(); // Drop all state, queues, detector history and map; keep config.
    bool initialized() const;
    std::optional<State> initialState() const; // Exact init() result; empty after reset.
    std::size_t bufferedImus() const;
    std::size_t bufferedClouds() const;
    std::uint64_t droppedImus() const;
    std::uint64_t droppedClouds() const;
    Cloud::Ptr mapSnapshot() const; // Internal matching map; no output anchoring.

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace lingtu::ltlio
