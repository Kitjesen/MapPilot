// SPDX-License-Identifier: GPL-2.0-or-later
// Native scheduling/ownership adaptation of Elevator-LIO 1d79af77.
#include "frontend.hpp"
#include "detail/Context.h"
#include "detail/ESEKF.h"
#include "detail/IMUProcess.h"
#include "detail/ElevatorSelfExit.h"
#include "detail/LidarPipeline.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace lingtu::ltlio {
namespace {
State exportState(const detail::State& state) {
    State out;
    out.stamp_s = state.time;
    out.orientation = state.q;
    out.position = state.p;
    out.velocity = state.v;
    out.gyro_bias = state.bw;
    out.accel_bias = state.ba;
    out.gravity = state.g;
    out.transport_height = state.z;
    out.transport_velocity = state.vz;
    out.transport_acceleration = state.az;
    out.in_elevator = state.in_elevator;
    out.covariance = state.P;
    return out;
}

Config checkedConfig(const Config& config) {
    if (config.estimate_extrinsics)
        throw std::invalid_argument("Elevator-LIO uses fixed LiDAR/IMU extrinsics");
    if (config.imu_init_samples == 0 || config.point_filter_num <= 0 ||
        !std::isfinite(config.blind_m) || config.blind_m < 0.0 ||
        std::isnan(config.max_range_m) || config.max_range_m < config.blind_m ||
        !std::isfinite(config.acc_scale) || config.acc_scale <= 0.0)
        throw std::invalid_argument("Invalid Elevator-LIO sensor configuration");
    if (!std::isfinite(config.elevator_entry_range_m) || config.elevator_entry_range_m <= 0.1 ||
        !std::isfinite(config.elevator_entry_hold_s) || config.elevator_entry_hold_s <= 0.0 ||
        !std::isfinite(config.elevator_entry_motion_m) || config.elevator_entry_motion_m <= 0.0)
        throw std::invalid_argument("Invalid elevator entry configuration");
    for (const double variance : {config.acc_noise_variance, config.gyro_noise_variance,
                                 config.acc_bias_noise_variance, config.gyro_bias_noise_variance})
        if (!std::isfinite(variance) || variance < 0.0)
            throw std::invalid_argument("Invalid IMU process-noise variance");
    if (!std::isfinite(config.hybrid_point_weight) || config.hybrid_point_weight <= 0.0 ||
        !std::isfinite(config.hybrid_point_initial_threshold_m) || config.hybrid_point_initial_threshold_m <= 0.0)
        throw std::invalid_argument("Invalid hybrid correspondence configuration");
    return config;
}
}

struct Frontend::Impl {
    Config config;
    detail::Context context;
    std::shared_ptr<detail::EskfEstimator> estimator;
    detail::IMUProcess imu;
    detail::ElevatorSelfExit self_exit;
    detail::LidarPipeline lidar;
    std::deque<ImuSample> imu_queue;
    struct RecordedImu {
        ImuSample sample;
        std::optional<double> gap_start_s;
    };
    std::deque<RecordedImu> causal_imus;
    std::optional<double> pending_imu_gap_s;
    std::deque<Result> intake_results;
    std::optional<Eigen::Vector3d> last_accepted_gyro;
    std::optional<Eigen::Vector3d> last_raw_gyro;
    std::optional<State> initial_state;
    std::optional<double> pending_waiting_zupt_s;
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    double latest_imu_time = -1.0;
    double last_detector_imu_time = -1.0;
    struct MotionSample {
        double stamp_s;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };
    std::deque<MotionSample> stationary_poses;
    double last_stationary_scan_s = -1.0;
    std::uint64_t stationary_seen_exits = 0;
    bool prior_active = false;
    std::uint64_t entries = 0, exits = 0;
    std::uint64_t dropped_imus = 0, dropped_clouds = 0;

    explicit Impl(const Config& cfg)
        : config(checkedConfig(cfg)), context(config),
          estimator(std::make_shared<detail::EskfEstimator>(context)),
          imu(context), self_exit(context),
          lidar(context, estimator, imu, self_exit, context.ikd_map) {}

    void clearStationary() {
        stationary_poses.clear();
        last_stationary_scan_s = -1.0;
    }

    void applyWaitingZupt(detail::State& state, double scan_end_s) {
        if (!state.in_elevator || !context.ELEVATOR_TRIGGER || context.EXIT_FROM_ELEVATOR ||
            self_exit.eleState() != 0) {
            pending_waiting_zupt_s.reset();
            return;
        }
        if (!pending_waiting_zupt_s || *pending_waiting_zupt_s > scan_end_s) return;
        pending_waiting_zupt_s.reset();
        if (std::abs(state.vz) >= context.elevator_waiting_zupt_vz_abs_thresh ||
            std::abs(state.az) >= context.elevator_waiting_zupt_az_abs_thresh) return;
        // The posterior and its world<-odom correction belong to this same
        // scan. A covering IMU to its right may only request a later update.
        imu.elev_process.applyZeroVelocityUpdateZ(state);
        state.time = scan_end_s;
    }

    void observeGroundMotion(detail::State& state, const Result& result) {
        const bool exited = stationary_seen_exits != exits;
        stationary_seen_exits = exits;
        if (!result.accepted || state.in_elevator || exited) {
            clearStationary();
            return;
        }
        bool stationary = false;
        if (config.imu_static_acc_thresh > 0.0 && config.imu_static_gyro_thresh > 0.0) {
            const bool continuous = last_stationary_scan_s >= 0.0 &&
                result.scan_end_s > last_stationary_scan_s &&
                result.scan_end_s - last_stationary_scan_s <= 0.5;
            if (!continuous) stationary_poses.clear();
            const double start = continuous ? last_stationary_scan_s : result.scan_start_s;
            last_stationary_scan_s = result.scan_end_s;
            Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
            std::size_t count = 0;
            for (const auto& recorded : causal_imus) {
                const auto& sample = recorded.sample;
                if (sample.stamp_s <= start || sample.stamp_s > result.scan_end_s) continue;
                acc_mean += sample.acceleration * config.acc_scale;
                gyro_mean += sample.angular_velocity_rps;
                ++count;
            }
            bool quiet = count >= 2;
            if (quiet) {
                acc_mean /= static_cast<double>(count);
                gyro_mean /= static_cast<double>(count);
                double acc_variance = 0.0, gyro_variance = 0.0;
                for (const auto& recorded : causal_imus) {
                    const auto& sample = recorded.sample;
                    if (sample.stamp_s <= start || sample.stamp_s > result.scan_end_s) continue;
                    acc_variance += (sample.acceleration * config.acc_scale - acc_mean).squaredNorm();
                    gyro_variance += (sample.angular_velocity_rps - gyro_mean).squaredNorm();
                }
                quiet = acc_variance / count < config.imu_static_acc_thresh * config.imu_static_acc_thresh &&
                    gyro_variance / count < config.imu_static_gyro_thresh * config.imu_static_gyro_thresh &&
                    std::abs((acc_mean - state.ba).norm() - state.g.norm()) < config.imu_static_acc_thresh &&
                    (gyro_mean - state.bw).norm() < config.imu_static_gyro_thresh;
            }
            if (!quiet) {
                stationary_poses.clear();
            } else {
                stationary_poses.push_back({result.scan_end_s, state.p, state.q});
                while (stationary_poses.size() > 1 &&
                       result.scan_end_s - stationary_poses[1].stamp_s >= 1.0)
                    stationary_poses.pop_front();
                const auto& first = stationary_poses.front();
                const double duration = result.scan_end_s - first.stamp_s;
                if (duration >= 0.5 && stationary_poses.size() >=
                    static_cast<std::size_t>(std::max(1, config.zupt_min_static_frames))) {
                    // Quiet IMU alone cannot distinguish rest from steady travel.
                    // Require an accepted LiDAR motion window and a small current
                    // velocity as well, so a newly moving robot is not held at zero.
                    stationary = (state.p - first.position).norm() <= config.zupt_sigma_v * duration &&
                        first.orientation.angularDistance(state.q) <=
                            std::min(0.01, config.imu_static_gyro_thresh) * duration &&
                        state.v.norm() <= 3.0 * config.zupt_sigma_v;
                    for (const auto& pose : stationary_poses) {
                        if ((pose.position - first.position).norm() > 3.0 * config.zupt_sigma_v * duration) {
                            stationary = false;
                            break;
                        }
                    }
                }
            }
        }
        bool corrected = false;
        if (stationary && config.zupt_sigma_v > 0.0)
            corrected = estimator->applyVelocityObservation(state, config.zupt_sigma_v);
        if (corrected) {
            state.time = result.scan_end_s;
        }
    }

    void boundCloudQueue() {
        while (context.frames.size() + intake_results.size() > config.max_lidar_buffer) {
            if (!intake_results.empty() && (context.frames.empty() ||
                intake_results.front().scan_end_s <= context.frames.front().end_time))
                intake_results.pop_front();
            else
                context.frames.pop_front();
            ++dropped_clouds;
        }
    }

    void recordMode(bool active) {
        if (active == prior_active) return;
        if (active) ++entries;
        else ++exits;
        prior_active = active;
    }

    void advanceImu() {
        // This is the original IMU/LiDAR FSM boundary: retain the covering
        // right-hand IMU in history and leave later IMUs queued until commit.
        while (!imu_queue.empty()) {
            if (initial_state && !context.frames.empty() &&
                latest_imu_time >= context.frames.front().end_time)
                break;
            const ImuSample sample = imu_queue.front();
            imu_queue.pop_front();
            causal_imus.push_back({sample, pending_imu_gap_s});
            pending_imu_gap_s.reset();
            if (causal_imus.size() > detail::IMUProcess::MAX_LEN) {
                const double discarded_stamp = causal_imus.front().sample.stamp_s;
                if (config.elevator_enabled && discarded_stamp > last_detector_imu_time) {
                    // Missing detector input breaks continuous stop evidence.
                    // Evicting samples already consumed by scans does not.
                    self_exit.invalidateImuContinuity();
                    pending_waiting_zupt_s.reset();
                    last_detector_imu_time = discarded_stamp;
                }
                causal_imus.pop_front();
            }
            if (!initial_state) {
                detail::State state;
                if (imu.init(sample, static_cast<int>(config.imu_init_samples),
                             mean_acc, mean_gyro, state)) {
                    state.time = sample.stamp_s;
                    estimator->mean_acc_ = mean_acc;
                    estimator->mean_gyr_ = mean_gyro;
                    estimator->set_cur_state(state);
                    imu.set_init_state(state, sample.stamp_s);
                    initial_state = exportState(state);
                }
                continue;
            }
            const Eigen::Vector3d acceleration = sample.acceleration * config.acc_scale;
            imu.integration(acceleration, sample.angular_velocity_rps,
                            sample.stamp_s);
            latest_imu_time = sample.stamp_s;
        }
    }

    void processDetectorThrough(double scan_end_s) {
        if (!config.elevator_enabled) return;
        // IMU integration retains its right bracket and existing history.
        // Transport decisions consume only evidence available at this scan,
        // so later samples can neither confirm nor revoke its stop update.
        for (auto& recorded : causal_imus) {
            const auto& sample = recorded.sample;
            if (sample.stamp_s <= last_detector_imu_time) continue;
            if (recorded.gap_start_s && *recorded.gap_start_s <= scan_end_s) {
                // Intake loss breaks continuous stop evidence at its own time,
                // including scans inside the missing interval. The surviving
                // covering sample can still be in this scan's future.
                self_exit.invalidateImuContinuity();
                pending_waiting_zupt_s.reset();
                recorded.gap_start_s.reset();
            }
            if (sample.stamp_s > scan_end_s) break;
            last_detector_imu_time = sample.stamp_s;
            if (!initial_state || sample.stamp_s <= initial_state->stamp_s ||
                !imu.containsTime(sample.stamp_s)) continue;
            detail::State state;
            imu.get_state_at_t(state, sample.stamp_s);
            // Observe transitions only once their IMU time is reached. This
            // also retains trips whose LiDAR matches are rejected.
            recordMode(state.in_elevator);
            if (context.EXIT_FROM_ELEVATOR) {
                pending_waiting_zupt_s.reset();
                continue;
            }
            const Eigen::Vector3d acceleration = sample.acceleration * config.acc_scale;
            const Eigen::Vector3d acc_world =
                state.q * (acceleration - state.ba) + state.g;
            detail::ElevatorSelfExit::ImuFrame frame;
            frame.timestamp = sample.stamp_s;
            frame.acc_z = acc_world.z();
            frame.vel_z = state.vz;
            frame.in_elevator = state.in_elevator;
            const auto event = self_exit.process(frame, state);
            if (!state.in_elevator || self_exit.eleState() != 0 ||
                std::abs(state.vz) >= context.elevator_waiting_zupt_vz_abs_thresh ||
                std::abs(state.az) >= context.elevator_waiting_zupt_az_abs_thresh)
                pending_waiting_zupt_s.reset();
            else if (event.apply_waiting_zupt && !pending_waiting_zupt_s)
                pending_waiting_zupt_s = sample.stamp_s;
        }
    }

    void populateResult(Result& result, const detail::State& state, int phase_before) const {
        result.state = exportState(state);
        result.state.stamp_s = result.scan_end_s;
        result.world_position = state.p;
        result.world_position.z() += state.z;
        result.world_velocity = state.v;
        result.world_velocity.z() += state.vz;
        result.world_from_odom = context.world_from_odom;
        Eigen::Matrix<double, 3, 21> world_J = Eigen::Matrix<double, 3, 21>::Zero();
        world_J.block<3, 3>(0, detail::P).setIdentity();
        if (state.in_elevator) world_J(2, detail::Z) = 1.0;
        result.world_position_covariance = world_J * state.P * world_J.transpose();
        for (auto it = causal_imus.rbegin(); it != causal_imus.rend(); ++it) {
            if (it->sample.stamp_s <= result.scan_end_s) {
                result.causal_gyro_stamp_s = it->sample.stamp_s;
                result.causal_gyro_rps = it->sample.angular_velocity_rps;
                break;
            }
        }
        result.elevator.phase = phase_before;
        result.elevator.velocity_variance = self_exit.debugInfo().std_dev;
        result.elevator.range_quantile_m = imu.elev_process.doorLastFilteredMaxDistance();
        result.elevator.stop_confirmed = self_exit.stopCheckConfirmed();
        result.elevator.stop_updates = self_exit.stopCheckLidarZuptCount();
        result.elevator.entries = entries;
        result.elevator.exits = exits;
    }
};

Frontend::Frontend(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
Frontend::~Frontend() = default;
Frontend::Frontend(Frontend&&) noexcept = default;
Frontend& Frontend::operator=(Frontend&&) noexcept = default;

Cloud::Ptr selectPoints(const TimedCloud& input, const Config& config) {
    Cloud::Ptr cloud(new Cloud);
    cloud->reserve(input.points.size());
    const unsigned stride = static_cast<unsigned>(std::max(1, config.point_filter_num));
    unsigned valid_num = 0;
    // Preserve original Livox selection: skip point zero, validate line/tag,
    // stride valid returns, then test physical range. End time stays unfiltered.
    for (std::size_t i = 1; i < input.points.size(); ++i) {
        const auto& point = input.points[i];
        if (point.line >= 6 || ((point.tag & 0x30) != 0x10 && (point.tag & 0x30) != 0x00))
            continue;
        if (++valid_num % stride != 0) continue;
        pcl::PointXYZINormal p;
        p.x = point.x; p.y = point.y; p.z = point.z; p.intensity = point.intensity;
        p.curvature = static_cast<float>(point.offset_ns) / 1e6f;
        const float range_sq = p.x * p.x + p.y * p.y + p.z * p.z;
        if (range_sq > config.blind_m * config.blind_m &&
            range_sq <= config.max_range_m * config.max_range_m)
            cloud->push_back(p);
    }
    return cloud;
}

void Frontend::feedImu(const ImuSample& input) {
    ImuSample sample = input;
    // Keep isolated gyro spikes out of propagation without freezing a stale
    // reference forever. Two consistent raw readings establish a new level.
    if (impl_->last_accepted_gyro &&
        (input.angular_velocity_rps - *impl_->last_accepted_gyro).norm() > 10.0) {
        if (!impl_->initial_state) {
            // Calibration requires a complete new raw window, not a series
            // of duplicated readings made artificially quiet by this filter.
            impl_->imu.resetInitialization();
        } else if (!impl_->last_raw_gyro ||
                   (input.angular_velocity_rps - *impl_->last_raw_gyro).norm() > 10.0) {
            sample.angular_velocity_rps = *impl_->last_accepted_gyro;
        }
    }
    impl_->last_raw_gyro = input.angular_velocity_rps;
    impl_->last_accepted_gyro = sample.angular_velocity_rps;
    impl_->imu_queue.push_back(sample);
    while (impl_->imu_queue.size() > impl_->config.max_imu_buffer) {
        if (!impl_->pending_imu_gap_s)
            impl_->pending_imu_gap_s = impl_->imu_queue.front().stamp_s;
        impl_->imu_queue.pop_front();
        ++impl_->dropped_imus;
    }
    impl_->advanceImu();
}

void Frontend::feedCloud(const TimedCloud& input) {
    const double end = input.end_stamp_s != 0.0 ? input.end_stamp_s :
        input.stamp_s + (input.points.empty() ? 0.0 : static_cast<double>(input.points.back().offset_ns) / 1e9);
    if (!impl_->initial_state) {
        // The original callback drops LiDAR during IMU initialization. Expose
        // that lifecycle result without allowing it into the estimator.
        Result status;
        status.scan_start_s = input.stamp_s;
        status.scan_end_s = end;
        status.input_points = input.points.size();
        impl_->intake_results.push_back(std::move(status));
        impl_->boundCloudQueue();
        return;
    }
    detail::Frame frame;
    frame.base_time = input.stamp_s;
    frame.end_time = end;
    frame.input_points = input.points.size();
    frame.cloud = selectPoints(input, impl_->config);
    if (frame.cloud->empty()) {
        Result status;
        status.scan_start_s = frame.base_time;
        status.scan_end_s = frame.end_time;
        status.initialized = true;
        status.map_ready = impl_->lidar.mapReady();
        status.input_points = frame.input_points;
        status.state = *impl_->initial_state;
        impl_->intake_results.push_back(std::move(status));
        impl_->boundCloudQueue();
        return;
    }
    impl_->context.frames.push_back(std::move(frame));
    impl_->boundCloudQueue();
    impl_->advanceImu();
}

std::optional<Result> Frontend::processNext(
    const std::function<std::optional<PosePrior>(const Result&)>& prior) {
    for (;;) {
        if (!impl_->intake_results.empty() && (impl_->context.frames.empty() ||
            impl_->intake_results.front().scan_end_s <= impl_->context.frames.front().end_time)) {
            Result result = std::move(impl_->intake_results.front());
            impl_->intake_results.pop_front();
            impl_->clearStationary();
            return result;
        }
        impl_->advanceImu();
        if (!impl_->initial_state || impl_->context.frames.empty() ||
            impl_->latest_imu_time < impl_->context.frames.front().end_time)
            return std::nullopt;
        // Upstream's five raw initialization frames do not read or commit an
        // IMU posterior; a just-arrived first frame can end before IMU init.
        if (!impl_->lidar.mapReady() ||
            impl_->imu.containsTime(impl_->context.frames.front().end_time)) break;
        // A delayed scan can outlive the fixed IMU history. Active elevator
        // mode skips deskew, so its usual time-coverage check cannot catch it.
        impl_->context.frames.pop_front();
        ++impl_->dropped_clouds;
        impl_->clearStationary();
    }
    const auto frame = impl_->context.frames.front();
    Result result;
    result.scan_start_s = frame.base_time;
    result.scan_end_s = frame.end_time;
    result.input_points = frame.input_points;
    result.initialized = true;
    impl_->processDetectorThrough(frame.end_time);
    const int phase_before = impl_->self_exit.eleState();
    const auto processed = impl_->lidar.process([&](detail::State& state) {
        impl_->recordMode(state.in_elevator);
        impl_->applyWaitingZupt(state, frame.end_time);
        Result observation;
        observation.scan_start_s = frame.base_time;
        observation.scan_end_s = frame.end_time;
        observation.accepted = true;
        impl_->observeGroundMotion(state, observation);
        if (!prior) return;
        impl_->populateResult(observation, state, phase_before);
        if (const auto correction = prior(observation)) {
            if (!correction->world_imu_rotation.coeffs().allFinite() ||
                correction->world_imu_rotation.norm() <= 1e-12 ||
                !correction->world_imu_position.allFinite() ||
                (correction->world_imu_velocity && !correction->world_imu_velocity->allFinite()))
                throw std::invalid_argument("Invalid external LTLIO pose");
            state.q = correction->world_imu_rotation.normalized();
            state.p = correction->world_imu_position;
            state.p.z() -= state.z;
            if (correction->world_imu_velocity) {
                state.v = *correction->world_imu_velocity;
                state.v.z() -= state.vz;
            }
            state.time = frame.end_time;
            impl_->clearStationary();
        }
    });
    result.map_ready = impl_->lidar.mapReady();
    result.attempted = processed.processed;
    result.iterations = processed.processed ? impl_->estimator->lastIterationCount() : 0;
    result.converged = processed.processed && impl_->estimator->lastConverged();
    result.sampled_points = processed.processed ? impl_->lidar.sampledPoints() : 0;
    // Upstream returns before matching for fewer than ten sampled points;
    // its retained matcher count then belongs to the preceding scan.
    result.effective_points = processed.processed && result.sampled_points >= 10
        ? impl_->estimator->validPointCount() : 0;
    result.plane_matches = processed.processed && result.sampled_points >= 10
        ? impl_->estimator->lastPlaneMatchCount() : 0;
    result.point_matches = processed.processed && result.sampled_points >= 10
        ? impl_->estimator->lastPointMatchCount() : 0;
    result.mapping_points = processed.processed ? impl_->lidar.mappingPoints() : 0;
    result.sampling_leaf_size_m = processed.processed ? impl_->lidar.samplingLeafSize() : 0.0;
    result.sampling_scale_m = processed.processed ? impl_->lidar.samplingScaleIndicator() : 0.0;
    if (processed.processed && result.sampled_points >= 10)
        result.geometry = impl_->estimator->lastGeometry();
    detail::State state;
    if (processed.processed) state = processed.post_state;
    else impl_->imu.get_state_at_t(state, frame.end_time);
    result.accepted = result.attempted && impl_->estimator->lastUpdateAccepted() &&
        state.p.allFinite() && state.q.coeffs().allFinite() && state.P.allFinite();
    if (result.attempted) {
        result.rejection_reason = impl_->estimator->lastRejectionReason();
        result.candidate_translation_m = impl_->estimator->candidateTranslation();
        result.candidate_rotation_rad = impl_->estimator->candidateRotation();
        result.candidate_velocity_delta_mps = impl_->estimator->candidateVelocityDelta();
    }
    if (!result.accepted) impl_->clearStationary();
    impl_->populateResult(result, state, phase_before);
    result.deskewed_cloud.reset(new Cloud(processed.dedistorted_cloud ?
        *processed.dedistorted_cloud : *frame.cloud));
    return result;
}

bool Frontend::correctState(double stamp_s, const Eigen::Quaterniond& world_imu_rotation,
                            const Eigen::Vector3d& world_imu_position,
                            const std::optional<Eigen::Vector3d>& world_imu_velocity) {
    if (!impl_->initial_state || !impl_->imu.containsTime(stamp_s) ||
        !world_imu_rotation.coeffs().allFinite() || world_imu_rotation.norm() <= 1e-12 ||
        !world_imu_position.allFinite() ||
        (world_imu_velocity && !world_imu_velocity->allFinite())) return false;
    detail::State state;
    impl_->imu.get_state_at_t(state, stamp_s);
    state.q = world_imu_rotation.normalized();
    state.p = world_imu_position;
    state.p.z() -= state.z;
    if (world_imu_velocity) {
        state.v = *world_imu_velocity;
        state.v.z() -= state.vz;
    }
    state.time = stamp_s;
    impl_->imu.set_state_at_t(state, stamp_s);
    impl_->estimator->set_cur_state(state);
    impl_->clearStationary();
    return true;
}

void Frontend::requestElevator(bool active) {
    // The actual entry consumes the door episode; exit commits with a scan.
    impl_->context.ELEVATOR_TRIGGER = impl_->config.elevator_enabled && active;
    if (!active) impl_->pending_waiting_zupt_s.reset();
    impl_->clearStationary();
}
void Frontend::reset() {
    const Config config = impl_->config;
    impl_.reset();
    impl_ = std::make_unique<Impl>(config);
}
bool Frontend::initialized() const { return impl_->initial_state.has_value(); }
std::optional<State> Frontend::initialState() const { return impl_->initial_state; }
std::size_t Frontend::bufferedImus() const { return impl_->imu_queue.size(); }
std::size_t Frontend::bufferedClouds() const {
    return impl_->context.frames.size() + impl_->intake_results.size();
}
std::uint64_t Frontend::droppedImus() const { return impl_->dropped_imus; }
std::uint64_t Frontend::droppedClouds() const { return impl_->dropped_clouds; }
Cloud::Ptr Frontend::mapSnapshot() const {
    detail::PointVector points;
    impl_->context.ikdtree.Snapshot(points);
    Cloud::Ptr cloud(new Cloud);
    cloud->points = std::move(points);
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}
} // namespace lingtu::ltlio
