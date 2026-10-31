#include "ltlio/detail/ScaleAwareVoxelizer.hpp"
#include "ltlio/frontend.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace {
namespace lio = lingtu::ltlio;
using lio::detail::ScaleAwareVoxelizer;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

lio::Cloud grid(double offset = 2.0, int side = 35) {
    lio::Cloud cloud;
    cloud.header.stamp = 123456789;
    for (int x = 0; x < side; ++x) {
        for (int y = 0; y < side; ++y) {
            pcl::PointXYZINormal point{};
            point.x = static_cast<float>(offset + x * 0.037);
            point.y = static_cast<float>(0.013 + y * 0.037);
            point.z = 0.021F;
            point.intensity = static_cast<float>(cloud.size() + 1);
            point.curvature = static_cast<float>(100.0 * cloud.size() / (side * side));
            point.normal_x = 0.2F;
            point.normal_y = 0.3F;
            point.normal_z = 0.4F;
            cloud.push_back(point);
        }
    }
    return cloud;
}

bool samePoint(const pcl::PointXYZINormal& a, const pcl::PointXYZINormal& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity &&
        a.curvature == b.curvature && a.normal_x == b.normal_x &&
        a.normal_y == b.normal_y && a.normal_z == b.normal_z;
}

bool sameCloud(const lio::Cloud& a, const lio::Cloud& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), samePoint);
}

void subsetFieldsAndDensity() {
    ScaleAwareVoxelizer sampler(0.25);
    auto input = grid();
    std::reverse(input.begin(), input.end());
    const auto output = sampler.process(input, 42.0);
    require(output.mapping.size() > output.estimation.size() * 2,
            "mapping must retain more geometry than estimation");
    require(output.mapping.header.stamp == input.header.stamp &&
                output.estimation.header.stamp == input.header.stamp,
            "sampling changed the cloud timestamp");
    for (const auto& point : output.mapping)
        require(std::any_of(input.begin(), input.end(), [&](const auto& raw) { return samePoint(raw, point); }),
                "mapping invented a centroid or changed a point field");
    for (const auto& point : output.estimation)
        require(std::any_of(output.mapping.begin(), output.mapping.end(),
                            [&](const auto& mapped) { return samePoint(mapped, point); }),
                "estimation point is not an exact mapping-cloud subset");
    const auto time_order = [](const auto& a, const auto& b) { return a.curvature < b.curvature; };
    require(std::is_sorted(output.mapping.begin(), output.mapping.end(), time_order) &&
                std::is_sorted(output.estimation.begin(), output.estimation.end(), time_order),
            "point timing order was lost");
    ScaleAwareVoxelizer second(0.25);
    const auto duplicate = second.process(input, 42.0);
    require(sameCloud(output.mapping, duplicate.mapping) && sameCloud(output.estimation, duplicate.estimation),
            "the same scan/history did not select deterministic representatives");
}

void scaleWindowAndTargets() {
    ScaleAwareVoxelizer sampler(0.25);
    lio::Cloud near;
    pcl::PointXYZINormal point{};
    point.x = 3.0F;
    near.push_back(point);
    const auto first = sampler.process(near, 1.0);
    require(std::abs(first.scale_indicator - 3.0) < 1e-12 &&
                std::abs(first.setpoint - 1570.0) < 1e-9,
            "near-scene scale did not drive the bounded power-map target");
    lio::Cloud far = near;
    far[0].x = 60.0F;
    const auto transition = sampler.process(far, 1.1);
    require(std::abs(transition.scale_indicator - 31.5) < 1e-12 && transition.setpoint == 4000.0,
            "median sliding average or far-scene saturation is incorrect");
    for (int frame = 0; frame < 5; ++frame) sampler.process(far, 1.2 + frame * 0.1);
    const auto steady = sampler.process(far, 1.8);
    require(steady.scale_indicator == 60.0 && steady.setpoint == 4000.0,
            "the five-frame scale window retained expired near-scene evidence");
    sampler.reset();
    const auto reset = sampler.process(near, 1.0);
    require(reset.leaf_size == first.leaf_size && reset.setpoint == first.setpoint &&
                sameCloud(reset.mapping, first.mapping),
            "reset retained scale, PD error or time history");
}

void pdResponseAndBounds() {
    ScaleAwareVoxelizer dense(0.03);
    const auto input = grid(3.0, 80);
    const auto dense_output = dense.process(input, 10.0);
    require(dense_output.tracking_error > 0.0 && dense_output.leaf_size > 0.03,
            "excess scan points did not increase the voxel leaf");
    ScaleAwareVoxelizer sparse(0.25);
    auto one = grid(3.0, 1);
    const auto sparse_output = sparse.process(one, 10.0);
    require(sparse_output.tracking_error < 0.0 && sparse_output.leaf_size < 0.25,
            "point deficit did not reduce the voxel leaf");
    for (int frame = 0; frame < 30; ++frame) {
        const auto bounded = sparse.process(one, 10.1 + frame * 0.1);
        require(std::isfinite(bounded.leaf_size) && bounded.leaf_size >= 0.02 && bounded.leaf_size <= 1.0 &&
                    bounded.setpoint >= 1000.0 && bounded.setpoint <= 4000.0,
                "PD controller escaped its leaf or target bounds");
    }
    const auto repeated_stamp = dense.process(input, 10.0);
    require(std::isfinite(repeated_stamp.leaf_size), "zero scan interval produced a nonfinite PD step");
    ScaleAwareVoxelizer empty_control(0.25), empty_sampler(0.25);
    lio::Cloud empty;
    require(empty_sampler.process(empty, 1.0).mapping.empty(), "empty input generated geometry");
    auto invalid = one;
    invalid[0].x = std::numeric_limits<float>::quiet_NaN();
    require(empty_sampler.process(invalid, 2.0).estimation.empty(), "nonfinite geometry entered sampling");
    require(empty_sampler.process(one, 3.0, 10.0).mapping.empty(), "blind-range returns entered sampling");
    require(sameCloud(empty_sampler.process(input, 4.0).mapping,
                      empty_control.process(input, 4.0).mapping),
            "an empty or rejected input poisoned scale/controller history");
}

lio::TimedCloud room(double stamp, double shift = 0.0) {
    lio::TimedCloud cloud;
    cloud.stamp_s = stamp;
    cloud.end_stamp_s = stamp + 0.020;
    for (int axis = 0; axis < 3; ++axis) {
        for (int a = -25; a <= 25; ++a) {
            for (int b = -25; b <= 25; ++b) {
                lio::TimedPoint point;
                const double s = 0.04;
                if (axis == 0) { point.x = 3.031F + static_cast<float>(shift); point.y = a * s; point.z = b * s; }
                if (axis == 1) { point.x = a * s + static_cast<float>(shift); point.y = 4.037F; point.z = b * s; }
                if (axis == 2) { point.x = a * s + static_cast<float>(shift); point.y = b * s; point.z = -2.041F; }
                point.intensity = static_cast<float>(cloud.points.size() + 1);
                cloud.points.push_back(point);
            }
        }
    }
    for (std::size_t i = 0; i < cloud.points.size(); ++i)
        cloud.points[i].offset_ns = 20'000'000ULL * i / (cloud.points.size() - 1);
    return cloud;
}

using MapPoints = std::vector<std::tuple<float, float, float, float>>;
MapPoints sortedMap(const lio::Cloud& cloud) {
    MapPoints points;
    for (const auto& point : cloud) points.emplace_back(point.x, point.y, point.z, point.intensity);
    std::sort(points.begin(), points.end());
    return points;
}

void frontendInsertionAndRejection() {
    lio::Config config;
    config.scale_aware_sampling_enabled = true;
    config.elevator_enabled = false;
    config.automatic_door = false;
    config.imu_init_samples = 20;
    config.point_filter_num = 1;
    config.blind_m = 0.1;
    config.scan_resolution = 0.25;
    config.map_resolution = 0.01;
    config.max_update_translation_m = 0.001;
    lio::Frontend frontend(config);
    int imu_index = 0;
    const auto step = [&](int frame, double shift = 0.0) {
        auto scan = room(0.2 + frame * 0.04, shift);
        while (imu_index * 0.005 <= scan.end_stamp_s + 0.006) {
            lio::ImuSample imu;
            imu.stamp_s = imu_index++ * 0.005;
            imu.acceleration = Eigen::Vector3d(0.0, 0.0, 9.80665);
            frontend.feedImu(imu);
        }
        frontend.feedCloud(scan);
        const auto result = frontend.processNext();
        require(result.has_value() && result->scan_end_s == scan.end_stamp_s &&
                    result->scan_start_s == scan.stamp_s,
                "scale sampling changed scan endpoints or lost a covered frame");
        return *result;
    };
    std::vector<lio::Result> first_run;
    lio::Result result;
    for (int frame = 0; frame < 12; ++frame) {
        result = step(frame);
        first_run.push_back(result);
    }
    require(result.accepted && result.sampled_points > 10,
            "scale-aware frontend did not establish real LiDAR matching");
    const auto map = frontend.mapSnapshot();
    require(map->size() > result.sampled_points * 2,
            "real frontend map insertion used the coarse estimation cloud");
    const auto before = sortedMap(*map);
    const auto rejected = step(12, 0.04);
    first_run.push_back(rejected);
    require(rejected.attempted && !rejected.accepted &&
                rejected.rejection_reason == "update_translation_too_large",
            "the shifted scale-aware scan did not exercise the commit rejection seam");
    require(sortedMap(*frontend.mapSnapshot()) == before,
            "rejected scale-aware scan inserted its denser mapping cloud");
    const auto recovered = step(13);
    first_run.push_back(recovered);
    require(recovered.accepted, "a rejected scan prevented scale-aware tracking recovery");
    const auto final_map = sortedMap(*frontend.mapSnapshot());

    frontend.reset();
    imu_index = 0;
    require(!frontend.initialized() && !frontend.initialState().has_value() &&
                frontend.bufferedImus() == 0 && frontend.bufferedClouds() == 0 &&
                frontend.mapSnapshot()->empty(),
            "frontend reset retained initialization, pending input or map geometry");
    for (int frame = 0; frame < static_cast<int>(first_run.size()); ++frame) {
        const auto repeated = step(frame, frame == 12 ? 0.04 : 0.0);
        const auto& original = first_run[static_cast<std::size_t>(frame)];
        require(repeated.initialized == original.initialized && repeated.map_ready == original.map_ready &&
                    repeated.attempted == original.attempted && repeated.accepted == original.accepted &&
                    repeated.rejection_reason == original.rejection_reason &&
                    repeated.sampled_points == original.sampled_points &&
                    repeated.mapping_points == original.mapping_points &&
                    repeated.sampling_leaf_size_m == original.sampling_leaf_size_m &&
                    repeated.sampling_scale_m == original.sampling_scale_m,
                "reset retained sampling, rejection or initialization history during actual reentry");
        require((repeated.state.position - original.state.position).norm() < 1e-12 &&
                    repeated.state.orientation.angularDistance(original.state.orientation) < 1e-12 &&
                    (repeated.state.velocity - original.state.velocity).norm() < 1e-12 &&
                    (repeated.state.covariance - original.state.covariance).norm() < 1e-12,
                "the same recording after reset changed the actual frontend posterior");
    }
    require(sortedMap(*frontend.mapSnapshot()) == final_map,
            "the same recording after reset changed retained mapping geometry");
}
} // namespace

int main() {
    try {
        subsetFieldsAndDensity();
        scaleWindowAndTargets();
        pdResponseAndBounds();
        frontendInsertionAndRejection();
        std::cout << "Scale sampling: subset/fields, scale window, PD bounds, dense map, rejection and reset passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
