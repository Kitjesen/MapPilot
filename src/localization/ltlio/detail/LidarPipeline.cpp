
#include "LidarPipeline.h"
#include "Context.h"
#include "IMUProcess.h"
#include "ESEKF.h"
#include "ElevatorSelfExit.h"
#include "IkdMap.hpp"
#include "AdaptiveVoxelPController.hpp"
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <iomanip>
namespace lingtu::ltlio::detail {
/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * LiDAR processing pipeline implementation.
 * Extracted from timer_500HZ_callback in main.cpp.
 */

LidarPipeline::LidarPipeline(Context& context,
                               std::shared_ptr<EskfEstimator> estimator,
                               IMUProcess& imu_process,
                               ElevatorSelfExit& elevator_self_exit,
                               std::shared_ptr<IkdMap<PointType>> owned_map)
    : ctx_(context)
    , estimator_(std::move(estimator))
    , imu_process_(imu_process)
    , elevator_self_exit_(elevator_self_exit)
    , ikd_map_(std::move(owned_map))
    , raw_clouds_lidar_(new PointCloudXYZI)
    , cloud_init_accum_lidar_(new PointCloudXYZI)
    , cloud_init_lidar_(new PointCloudXYZI)
    , cloud_init_world_(new PointCloudXYZI)
    , rebuild_map_accum_lidar_(new PointCloudXYZI)
    , downsampled_cloud_(new PointCloudXYZI)
    , mapping_cloud_(new PointCloudXYZI)
    , blind_filtered_(new PointCloudXYZI)
    , scale_voxelizer_(context.filter_size)
{
}

void LidarPipeline::initAdaptiveVoxel() {
    if (ctx_.scale_aware_sampling_enabled) {
        current_voxel_ = scale_voxelizer_.leafSize();
        return;
    }
    if (!ctx_.downsample_adaptive_enabled) {
        current_voxel_ = ctx_.filter_size;
        return;
    }
    AdaptiveVoxelPController::Config cfg;
    cfg.target_points = ctx_.downsample_target_points;
    cfg.alpha = ctx_.downsample_alpha;
    cfg.min_voxel = ctx_.downsample_min_voxel;
    cfg.max_voxel = ctx_.downsample_max_voxel;
    voxel_ctrl_.setConfig(cfg);
    if (!voxel_ctrl_initialized_) {
        voxel_ctrl_.setVoxel(ctx_.filter_size);
        current_voxel_ = voxel_ctrl_.voxel();
        voxel_ctrl_initialized_ = true;
    }
}

void LidarPipeline::updateAdaptiveConfig(double end_time) {
    if (ctx_.scale_aware_sampling_enabled) return;
    if (!ctx_.downsample_adaptive_enabled) return;

    double lidar_dt = 0.0;
    if (last_lidar_stamp_for_adaptive_ > 0.0 && end_time > last_lidar_stamp_for_adaptive_) {
        lidar_dt = end_time - last_lidar_stamp_for_adaptive_;
        if (adaptive_dt_warmed_up_ && smoothed_lidar_dt_for_adaptive_ > 1e-4) {
            lidar_dt = std::clamp(lidar_dt,
                                  smoothed_lidar_dt_for_adaptive_ * 0.25,
                                  smoothed_lidar_dt_for_adaptive_ * 4.0);
        }
    }
    if (lidar_dt <= 1e-4) {
        lidar_dt = smoothed_lidar_dt_for_adaptive_ > 1e-4 ? smoothed_lidar_dt_for_adaptive_ : 0.1;
    }
    if (smoothed_lidar_dt_for_adaptive_ <= 1e-4) {
        smoothed_lidar_dt_for_adaptive_ = lidar_dt;
    } else {
        constexpr double kAdaptiveDtEma = 0.2;
        smoothed_lidar_dt_for_adaptive_ =
            (1.0 - kAdaptiveDtEma) * smoothed_lidar_dt_for_adaptive_ + kAdaptiveDtEma * lidar_dt;
    }
    adaptive_dt_warmed_up_ = adaptive_dt_warmed_up_ || (last_lidar_stamp_for_adaptive_ > 0.0);
    last_lidar_stamp_for_adaptive_ = end_time;

    current_target_points_per_frame_ =
        std::max(1.0, ctx_.downsample_target_points * smoothed_lidar_dt_for_adaptive_);
    AdaptiveVoxelPController::Config cfg = voxel_ctrl_.config();
    cfg.target_points = current_target_points_per_frame_;
    voxel_ctrl_.setConfig(cfg);
}

void LidarPipeline::applyVoxelFilter(const PointCloudXYZI::Ptr& in,
                                      PointCloudXYZI::Ptr& out,
                                      double blind_dist) {
    out->clear();
    if (!in || in->empty()) return;
    blind_filtered_->clear();
    const double blind_sq = blind_dist * blind_dist;
    blind_filtered_->points.reserve(in->points.size());
    for (const auto& pt : in->points) {
        double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < blind_sq) continue;
        blind_filtered_->points.push_back(pt);
    }
    if (blind_filtered_->empty()) return;

    uniform_voxel_.setLeafSize(current_voxel_, current_voxel_, current_voxel_);
    uniform_voxel_.setInputCloud(blind_filtered_);
    uniform_voxel_.filter(*out);
}

bool LidarPipeline::dequeueFrame(FrameData& out) {
    if (ctx_.frames.empty()) return false;
    const auto& frame = ctx_.frames.front();
    out.raw_cloud = frame.cloud;
    out.base_time = frame.base_time;
    out.end_time = frame.end_time;
    return true;
}

bool LidarPipeline::handleFirstLidarInit(const FrameData& frame) {
    if (!first_lidar_ || ctx_.relocation_enable) return false;

    cloud_init_accum_lidar_->points.insert(
        cloud_init_accum_lidar_->points.end(),
        frame.raw_cloud->points.begin(), frame.raw_cloud->points.end());
    cloud_init_accum_lidar_->width = static_cast<uint32_t>(cloud_init_accum_lidar_->points.size());
    cloud_init_accum_lidar_->height = 1;
    cloud_init_accum_lidar_->is_dense = false;
    init_lidar_accum_count_++;

    if (init_lidar_accum_count_ < kInitLidarAccumFrames) {
        ctx_.frames.pop_front();
        return true; // still accumulating
    }

    ctx_.ikdtree.set_downsample_param(ctx_.min_filter_size);
    if (ctx_.scale_aware_sampling_enabled) {
        auto sampled = scale_voxelizer_.process(*cloud_init_accum_lidar_, frame.end_time, ctx_.blind);
        *cloud_init_lidar_ = std::move(sampled.mapping);
    } else {
        applyVoxelFilter(cloud_init_accum_lidar_, cloud_init_lidar_, ctx_.blind);
    }
    cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
    ikd_map_->add(cloud_init_world_, estimator_->mapPointCovariances(*cloud_init_lidar_));

    first_lidar_ = false;
    init_lidar_accum_count_ = 0;
    cloud_init_accum_lidar_->clear();
    ctx_.frames.pop_front();
    return true; // handled
}

bool LidarPipeline::handlePostElevatorRebuild(const FrameData& frame) {
    if (!rebuild_map_pending_ || ctx_.relocation_enable) return false;

    rebuild_map_accum_lidar_->points.insert(
        rebuild_map_accum_lidar_->points.end(),
        frame.raw_cloud->points.begin(), frame.raw_cloud->points.end());
    rebuild_map_accum_lidar_->width = static_cast<uint32_t>(rebuild_map_accum_lidar_->points.size());
    rebuild_map_accum_lidar_->height = 1;
    rebuild_map_accum_lidar_->is_dense = false;
    ++rebuild_map_accum_count_;

    if (rebuild_map_accum_count_ < ctx_.elevator_rebuild_map_on_exit_frames ||
        static_cast<int>(rebuild_map_accum_lidar_->points.size()) < ctx_.elevator_rebuild_map_on_exit_min_points) {
        ctx_.frames.pop_front();
        return true; // still accumulating
    }

    if (ctx_.scale_aware_sampling_enabled) {
        auto sampled = scale_voxelizer_.process(*rebuild_map_accum_lidar_, frame.end_time, ctx_.blind);
        *cloud_init_lidar_ = std::move(sampled.mapping);
    } else {
        applyVoxelFilter(rebuild_map_accum_lidar_, cloud_init_lidar_, ctx_.blind);
    }
    cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
    if (ikd_map_) {
        ikd_map_->reset();
        ikd_map_->add(cloud_init_world_, estimator_->mapPointCovariances(*cloud_init_lidar_));
    }

    rebuild_map_pending_ = false;
    rebuild_map_accum_count_ = 0;
    rebuild_map_accum_lidar_->clear();

    ctx_.frames.pop_front();
    return true; // handled
}

void LidarPipeline::resolveFrameState(const FrameData& frame, State& out_state, bool& out_in_elevator) {
    imu_process_.get_state_at_t(out_state, frame.end_time);
    imu_process_.elev_process.processDoorDetector(
        *frame.raw_cloud, frame.end_time, out_state,
        true);
    out_in_elevator = out_state.in_elevator;
}

bool LidarPipeline::undistortFrame(const FrameData& frame, State& lidar_end_state,
                                    PointCloudXYZI::Ptr& out_cloud) {
    *out_cloud = *frame.raw_cloud;
    if (exit_from_elevator_) return true;

    std::vector<Pose> imu_pose_vec;
    imu_process_.get_imu_pose_from_t1_to_t2(imu_pose_vec, frame.base_time, frame.end_time);
    if (imu_pose_vec.empty()) return false;

    auto it_pcl = out_cloud->points.begin();
    for (auto it_pose = imu_pose_vec.begin(); it_pose + 1 != imu_pose_vec.end(); ++it_pose) {
        const Pose& head = *it_pose;
        const Pose& tail = *(it_pose + 1);
        const Eigen::Quaterniond q_imu = head.q;
        const Eigen::Vector3d vel_imu = head.v;
        const Eigen::Vector3d pos_imu = head.p;
        const Eigen::Vector3d acc_imu = tail.acc;
        const Eigen::Vector3d gyr_imu = tail.gyr;

        for (; it_pcl != out_cloud->points.end(); ++it_pcl) {
            double point_time = it_pcl->curvature / 1000.0;
            if (point_time > tail.offset_time + 1e-6) break;
            double dt = point_time - head.offset_time;
            Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z);
            Eigen::Vector3d P_I = ctx_.imu_R_lidar * P_L + ctx_.imu_t_lidar;
            Eigen::Matrix3d R_I_W(q_imu * Exp(gyr_imu, dt));
            Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt);
            Eigen::Vector3d P_W = R_I_W * P_I + T_I_W;
            Eigen::Vector3d P_IE = lidar_end_state.q.inverse() * (P_W - lidar_end_state.p);
            Eigen::Vector3d P_LE = ctx_.imu_R_lidar.transpose() * (P_IE - ctx_.imu_t_lidar);
            it_pcl->x = static_cast<float>(P_LE.x());
            it_pcl->y = static_cast<float>(P_LE.y());
            it_pcl->z = static_cast<float>(P_LE.z());
        }
        if (it_pcl == out_cloud->points.end()) break;
    }
    return true;
}

void LidarPipeline::downsampleFrame(PointCloudXYZI::Ptr& cloud, double scan_end_time) {
    if (ctx_.scale_aware_sampling_enabled) {
        if (cloud->size() > 10) {
            auto sampled = scale_voxelizer_.process(*cloud, scan_end_time, ctx_.blind);
            *mapping_cloud_ = std::move(sampled.mapping);
            *downsampled_cloud_ = std::move(sampled.estimation);
            current_voxel_ = sampled.leaf_size;
            last_mapping_points_ = mapping_cloud_->size();
            last_sampling_leaf_size_ = sampled.leaf_size;
            last_scale_indicator_ = sampled.scale_indicator;
        } else {
            mapping_cloud_->clear();
            downsampled_cloud_->clear();
        }
        return;
    }
    last_sampling_leaf_size_ = current_voxel_;
    if (cloud->points.size() > 10)
        applyVoxelFilter(cloud, downsampled_cloud_, ctx_.blind);
    else
        downsampled_cloud_->clear();
    last_mapping_points_ = downsampled_cloud_->size();
    if (ctx_.downsample_adaptive_enabled && !downsampled_cloud_->empty())
        current_voxel_ = voxel_ctrl_.update(downsampled_cloud_->size());
}

void LidarPipeline::runIeskfUpdate(State& post_state) {
    estimator_->state_vector_prior_.clear();
    estimator_->state_vector_prior_.push_back(post_state);
    estimator_->set_cur_state(post_state);
    if (!estimator_->get_prior_states().empty())
        ctx_.ikd_map->removeFar(estimator_->get_prior_states().back().p);
    estimator_->UpdateByLidarIESKF(*downsampled_cloud_);
    post_state = estimator_->get_cur_state();
}

void LidarPipeline::handleElevatorStop(State& post_state) {
    if (!ctx_.elevator_enable) return;
    if (!post_state.in_elevator || exit_from_elevator_) return;
    if (!elevator_self_exit_.stopCheckConfirmed()) return;

    int zupt_count = elevator_self_exit_.stopCheckLidarZuptCount();
    if (ctx_.elevator_zupt_enable && zupt_count < 3) {
        imu_process_.elev_process.applyZeroVelocityUpdateZ(post_state);
        estimator_->set_cur_state(post_state);
        elevator_self_exit_.setStopCheckLidarZuptCount(++zupt_count);

        if (zupt_count >= 3) {
            ctx_.ELEVATOR_TRIGGER = false;
            elevator_self_exit_.reset();

        }
    } else if (!ctx_.elevator_zupt_enable) {
        ctx_.ELEVATOR_TRIGGER = false;
        elevator_self_exit_.reset();

    }
}

bool LidarPipeline::handleElevatorExit(State& post_state) {
    if (!ctx_.elevator_enable) return false;
    if (!exit_from_elevator_) return false;
    exit_from_elevator_ = false;
    ctx_.EXIT_FROM_ELEVATOR = false;
    elevator_self_exit_.reset();

    // Commit position/velocity and their complete covariance once, returning
    // to the historical building map rather than discarding revisitable floors.
    imu_process_.elev_process.ElevatorModeExit(post_state);

    if (ctx_.elevator_exit_icp_z_enable) {
        const EskfEstimator::ZIcpResult icp_result =
                estimator_->matchZByIcpAgainstIkdTree(*downsampled_cloud_, post_state);
        if (icp_result.accepted) {
            post_state.p(2) += icp_result.correction_z;

        } else {

        }
    }

    estimator_->set_cur_state(post_state);

    if (ctx_.elevator_rebuild_map_on_exit_enable && !ctx_.relocation_enable) {
        rebuild_map_pending_ = true;
        rebuild_map_accum_count_ = 0;
        rebuild_map_accum_lidar_->clear();

        return false;
    }
    return true;
}

void LidarPipeline::commitFrame(const FrameData& frame, const State& post_state) {
    ctx_.lidar_end_time = frame.end_time;
    State corrected = post_state;
    imu_process_.set_state_at_t(corrected, frame.end_time);
    ctx_.frames.pop_front();
    const auto& insertion_cloud = ctx_.scale_aware_sampling_enabled ? *mapping_cloud_ : *downsampled_cloud_;
    if (!rebuild_map_pending_ && !ctx_.relocation_enable && !insertion_cloud.empty() &&
        shouldInsertMap(post_state))
        ctx_.ikd_map->add(estimator_->transLidar2World(insertion_cloud),
                          estimator_->mapPointCovariances(insertion_cloud));
}

bool LidarPipeline::shouldInsertMap(const State& state) {
    if (ctx_.stationary_thresh <= 0.0) return true;
    // Rotation changes the observed surfaces even at a stationary IMU origin.
    // Express its effect at the configured detection range in metres.
    const bool moved = !map_insert_pose_valid_ ||
        (state.p - last_map_position_).norm() >= ctx_.stationary_thresh ||
        state.q.angularDistance(last_map_orientation_) * ctx_.map_detect_range >= ctx_.stationary_thresh;
    if (moved) {
        last_map_position_ = state.p;
        last_map_orientation_ = state.q;
        map_insert_pose_valid_ = true;
    }
    return moved;
}

LidarPipeline::Result LidarPipeline::process(const std::function<void(State&)>& before_commit) {
    Result result;
    last_mapping_points_ = 0;
    last_sampling_leaf_size_ = 0.0;
    last_scale_indicator_ = 0.0;
    initAdaptiveVoxel();

    if (!ctx_.elevator_enable) {
        ctx_.ELEVATOR_TRIGGER = false;
        ctx_.EXIT_FROM_ELEVATOR = false;
    }
    exit_from_elevator_ = ctx_.elevator_enable && ctx_.EXIT_FROM_ELEVATOR;
    FrameData frame;
    if (!dequeueFrame(frame)) {
        return result;
    }
    updateAdaptiveConfig(frame.end_time);
    if (handleFirstLidarInit(frame)) {
        return result;
    }
    if (handlePostElevatorRebuild(frame)) {
        return result;
    }
    State lidar_end_state;
    bool in_elevator;
    resolveFrameState(frame, lidar_end_state, in_elevator);
    raw_clouds_lidar_ = frame.raw_cloud;
    if (in_elevator && !exit_from_elevator_) {
    } else {
        if (!undistortFrame(frame, lidar_end_state, raw_clouds_lidar_)) {
            ctx_.frames.pop_front();

            return result;
        }
    }
    downsampleFrame(raw_clouds_lidar_, frame.end_time);
    State post_state = lidar_end_state;
    runIeskfUpdate(post_state);
    if (!estimator_->lastUpdateAccepted()) {
        // A rejected observation must not enter either the IMU history or
        // the matching map under a new timestamp.
        ctx_.frames.pop_front();
        result.processed = true;
        result.post_state = post_state;
        result.dedistorted_cloud = raw_clouds_lidar_;
        return result;
    }
    handleElevatorStop(post_state);
    const bool insert_exit_cloud = handleElevatorExit(post_state);
    if (before_commit) {
        before_commit(post_state);
        estimator_->set_cur_state(post_state);
    }
    if (insert_exit_cloud) {
        if (ctx_.scale_aware_sampling_enabled)
            *cloud_init_lidar_ = *mapping_cloud_;
        else
            applyVoxelFilter(raw_clouds_lidar_, cloud_init_lidar_, ctx_.blind);
        cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
        // Retain historical floor geometry for subsequent revisits.
        ikd_map_->removeFar(post_state.p);
        ikd_map_->add(cloud_init_world_, estimator_->mapPointCovariances(*cloud_init_lidar_));
        map_insert_pose_valid_ = false;
    }
    commitFrame(frame, post_state);
    result.processed = true;
    result.post_state = post_state;
    result.dedistorted_cloud = raw_clouds_lidar_;
    return result;
}

} // namespace lingtu::ltlio::detail
