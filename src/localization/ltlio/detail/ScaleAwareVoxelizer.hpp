// Scale/PD equations adapted from GenZ-LIO, GNU GPL v2, revision
// bf444f7a747ed3da8548b56b49c14f207e8d49bb, ScaleAwareVoxelizer.cpp.
// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include "../frontend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lingtu::ltlio::detail {

class ScaleAwareVoxelizer {
public:
    struct Config {
        int window_size = 5;
        double scale_threshold = 30.0;
        int setpoint_exponent = 2;
        double min_points = 1000.0;
        double max_points = 4000.0;
        double error_sensitivity = 0.1;
        double error_rate_sensitivity = 0.2;
        double p_gain_min = 5e-6;
        double p_gain_max = 5e-5;
        double d_gain_min = 5e-8;
        double d_gain_max = 5e-7;
    };

    struct Output {
        Cloud mapping;
        Cloud estimation;
        double leaf_size = 0.0;
        double median_range = 0.0;
        double scale_indicator = 0.0;
        double setpoint = 0.0;
        double tracking_error = 0.0;
    };

    static constexpr double kMinLeafSize = 0.02;
    static constexpr double kMaxLeafSize = 1.0;
    static constexpr double kDefaultScanPeriod = 0.1;

    explicit ScaleAwareVoxelizer(double initial_leaf = 0.1)
        : ScaleAwareVoxelizer(initial_leaf, Config{}) {}

    ScaleAwareVoxelizer(double initial_leaf, const Config& config)
        : config_(config), initial_leaf_(std::clamp(initial_leaf, kMinLeafSize, kMaxLeafSize)),
          leaf_size_(initial_leaf_) {}

    void reset() {
        leaf_size_ = initial_leaf_;
        median_window_.clear();
        median_sum_ = 0.0;
        previous_error_ = 0.0;
        previous_scan_time_ = 0.0;
        have_previous_scan_ = false;
    }

    double leafSize() const { return leaf_size_; }

    Output process(const Cloud& undistorted, double scan_end_time, double blind_distance = 0.0) {
        Output output;
        output.leaf_size = leaf_size_;
        Cloud finite;
        finite.header = undistorted.header;
        finite.sensor_origin_ = undistorted.sensor_origin_;
        finite.sensor_orientation_ = undistorted.sensor_orientation_;
        finite.reserve(undistorted.size());
        const double blind_sq = blind_distance * blind_distance;
        for (const auto& point : undistorted) {
            const double range_sq = squaredRange(point);
            if (std::isfinite(range_sq) && range_sq >= blind_sq) finite.push_back(point);
        }
        if (finite.empty()) {
            output.mapping = finite;
            output.estimation = finite;
            return output;
        }

        const Cloud probe = selectVoxels(finite, leaf_size_);
        std::vector<double> ranges;
        ranges.reserve(probe.size());
        for (const auto& point : probe) ranges.push_back(squaredRange(point));
        auto middle = ranges.begin() + ranges.size() / 2;
        std::nth_element(ranges.begin(), middle, ranges.end());
        output.median_range = std::sqrt(*middle);
        median_window_.push_back(output.median_range);
        median_sum_ += output.median_range;
        if (median_window_.size() > static_cast<std::size_t>(config_.window_size)) {
            median_sum_ -= median_window_.front();
            median_window_.pop_front();
        }
        output.scale_indicator = median_sum_ / static_cast<double>(median_window_.size());
        const double scale = std::clamp(output.scale_indicator / config_.scale_threshold, 0.0, 1.0);
        output.setpoint = config_.min_points + (config_.max_points - config_.min_points) *
            (1.0 - std::pow(1.0 - scale, config_.setpoint_exponent));

        double dt = have_previous_scan_ ? scan_end_time - previous_scan_time_ : kDefaultScanPeriod;
        if (!(dt > 1e-9)) dt = kDefaultScanPeriod;
        previous_scan_time_ = scan_end_time;
        have_previous_scan_ = true;
        output.tracking_error = static_cast<double>(probe.size()) - output.setpoint;
        const double error_rate = (output.tracking_error - previous_error_) / dt;
        const auto ratio = [](double value, double limit) {
            return limit > 1e-9 ? std::min(std::abs(value), limit) / limit : 0.0;
        };
        const double p_gain = config_.p_gain_min + (config_.p_gain_max - config_.p_gain_min) *
            std::sqrt(scale * ratio(output.tracking_error, output.setpoint * config_.error_sensitivity));
        const double d_gain = config_.d_gain_min + (config_.d_gain_max - config_.d_gain_min) *
            std::sqrt(scale * ratio(error_rate, output.setpoint * config_.error_rate_sensitivity / dt));
        leaf_size_ = std::clamp(leaf_size_ + p_gain * output.tracking_error + d_gain * error_rate,
                                kMinLeafSize, kMaxLeafSize);
        previous_error_ = output.tracking_error;

        // Keep actual deskewed returns instead of averaging normal/time fields.
        // Coarse representatives are selected from the denser insertion cloud.
        output.mapping = selectVoxels(finite, leaf_size_ * 0.5);
        output.estimation = selectVoxels(output.mapping, leaf_size_);
        output.leaf_size = leaf_size_;
        return output;
    }

private:
    struct Voxel {
        std::int64_t x, y, z;
        bool operator==(const Voxel& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct VoxelHash {
        std::size_t operator()(const Voxel& voxel) const {
            const auto x = std::hash<std::int64_t>{}(voxel.x);
            const auto y = std::hash<std::int64_t>{}(voxel.y);
            const auto z = std::hash<std::int64_t>{}(voxel.z);
            return x ^ (y + 0x9e3779b9 + (x << 6) + (x >> 2)) ^
                (z + 0x9e3779b9 + (y << 6) + (y >> 2));
        }
    };

    static double squaredRange(const pcl::PointXYZINormal& point) {
        return static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y +
            static_cast<double>(point.z) * point.z;
    }

    static Cloud selectVoxels(const Cloud& cloud, double leaf_size) {
        Cloud output;
        output.header = cloud.header;
        output.sensor_origin_ = cloud.sensor_origin_;
        output.sensor_orientation_ = cloud.sensor_orientation_;
        output.reserve(cloud.size());
        std::unordered_set<Voxel, VoxelHash> occupied;
        occupied.reserve(cloud.size());
        for (const auto& point : cloud) {
            const Voxel voxel{static_cast<std::int64_t>(std::floor(point.x / leaf_size)),
                              static_cast<std::int64_t>(std::floor(point.y / leaf_size)),
                              static_cast<std::int64_t>(std::floor(point.z / leaf_size))};
            if (occupied.insert(voxel).second) output.push_back(point);
        }
        std::stable_sort(output.begin(), output.end(), [](const auto& a, const auto& b) {
            return a.curvature < b.curvature;
        });
        output.is_dense = true;
        return output;
    }

    Config config_;
    double initial_leaf_;
    double leaf_size_;
    std::deque<double> median_window_;
    double median_sum_ = 0.0;
    double previous_error_ = 0.0;
    double previous_scan_time_ = 0.0;
    bool have_previous_scan_ = false;
};

} // namespace lingtu::ltlio::detail
