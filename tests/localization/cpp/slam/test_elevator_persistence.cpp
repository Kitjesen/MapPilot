#include "slam.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace lingtu::slam;

namespace {
void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool hasMarker(const Cloud& cloud) {
    return std::any_of(cloud.points.begin(), cloud.points.end(), [](const auto& point) {
        return point.x > 7.5F && point.y < -3.5F && point.z > 2.5F;
    });
}

bool hasStaticSurfaces(const Cloud& cloud) {
    int x_plane = 0, y_plane = 0, floor = 0;
    for (const auto& point : cloud.points) {
        x_plane += std::abs(point.x - 1.5F) < 0.05F;
        y_plane += std::abs(point.y - 1.5F) < 0.05F;
        floor += std::abs(point.z + 1.0F) < 0.05F;
    }
    return x_plane >= 50 && y_plane >= 50 && floor >= 50;
}

LidarFrame scan(double stamp, bool marker) {
    LidarFrame frame;
    frame.stamp_s = stamp;
    frame.frame_id = "lidar";
    for (int surface = 0; surface < 3; ++surface) {
        for (int a = -5; a <= 5; ++a) {
            for (int b = -5; b <= 5; ++b) {
                PointXYZIT point;
                if (surface == 0) {
                    point.x = 1.5F; point.y = a * 0.16F; point.z = b * 0.16F;
                } else if (surface == 1) {
                    point.x = a * 0.16F; point.y = 1.5F; point.z = b * 0.16F;
                } else {
                    point.x = a * 0.16F; point.y = b * 0.16F; point.z = -1.0F;
                }
                point.intensity = 20.0F;
                frame.points.push_back(point);
            }
        }
    }
    if (marker) {
        PointXYZIT point;
        point.x = 8.0F; point.y = -4.0F; point.z = 3.0F;
        point.intensity = 233.0F;
        frame.points.push_back(point);
    }
    for (std::size_t i = 0; i < frame.points.size(); ++i) {
        frame.points[i].offset_time_ns = static_cast<std::int64_t>(
            i * 45'000'000ULL / (frame.points.size() - 1));
        frame.points[i].line = static_cast<std::uint8_t>(i % 4);
    }
    return frame;
}

void process(ISlamBackend& backend, double stamp, double& next_imu,
             bool marker, bool use_odom_prior) {
    while (next_imu <= stamp + 0.06) {
        ImuSample sample;
        sample.stamp_s = next_imu;
        sample.az = 9.80665;
        check(backend.feedImu(sample).ok, "IMU feed failed");
        next_imu += 0.005;
    }
    check(backend.feedLidar(scan(stamp, marker)).ok, "LiDAR feed failed");
    if (use_odom_prior) {
        OdomSample prior;
        prior.stamp_s = stamp + 0.045;
        prior.has_velocity = true;
        check(backend.feedVisualOdom(prior).ok, "fresh odometry prior feed failed");
    }
    for (int i = 0; i < 4; ++i) check(backend.tick().ok, "SLAM tick failed");
}

void checkSavedMap(ISlamBackend& backend, const std::filesystem::path& path) {
    const auto saved = backend.saveMap(path.string());
    check(saved.ok, "map save failed: " + saved.message);
    check(std::filesystem::exists(path), "map.pcd was not written");
    const auto output = backend.outputs();
    check(output.saved_map_cloud_map != nullptr, "saved map output missing");
    check(hasStaticSurfaces(*output.saved_map_cloud_map),
          "saving during transport lost the previously recorded static surfaces");
    check(!hasMarker(*output.saved_map_cloud_map),
          "active-only cabin geometry leaked into the saved map");
}

void checkPersistence(const std::filesystem::path& directory, bool use_odom_prior) {
    std::filesystem::create_directories(directory);
    const auto config_path = directory / "ltlio.yaml";
    {
        std::ofstream config(config_path);
        config << "elevator_mode: manual\n"
               << "odom_prior_enabled: " << (use_odom_prior ? "true\n" : "false\n")
               << "odom_prior_bypass_ltlio: false\n"
               << "odom_prior_max_age_s: 0.2\n"
               << "lidar_filter_num: 1\n"
               << "scan_resolution: 0.05\n"
               << "map_resolution: 0.05\n"
               << "imu_init_num: 20\n"
               // The marker must enter the matching tree even though the
               // fixture stays still, otherwise the old fallback also passes.
               << "stationary_thresh: 0.0\n"
               << "patch_min_interval_s: 0.0\n"
               << "patch_min_translation_m: 0.0\n"
               << "patch_min_rotation_rad: 0.0\n"
               << "sam:\n  min_time_s: 1000000000.0\n";
        check(static_cast<bool>(config), "could not write test configuration");
    }
    auto backend = makeLtlioBackend();
    check(backend != nullptr, "LTLIO backend unavailable");
    SlamConfig config;
    config.config_path = config_path.string();
    check(backend->configure(config).ok, "LTLIO configuration failed");
    check(backend->setMode(SlamMode::Mapping, "").ok, "mapping mode failed");
    double next_imu = 0.0;
    for (int i = 0; i < 8; ++i)
        process(*backend, 0.2 + i * 0.1, next_imu, false, use_odom_prior);
    const auto initialized = backend->outputs();
    check(initialized.state == SlamState::Tracking &&
          initialized.ltlio_lidar_update.attempted &&
          initialized.ltlio_lidar_update.accepted,
          "static scene did not produce accepted real LiDAR matching");
    check(initialized.odom_prior_active == use_odom_prior,
          "static fixture did not activate the requested odometry-prior path");
    checkSavedMap(*backend, directory / "before" / "map.pcd");

    check(backend->setElevator(true).ok, "manual transport request failed");
    for (int i = 0; i < 4; ++i)
        process(*backend, 1.0 + i * 0.1, next_imu, true, use_odom_prior);
    const auto active = backend->outputs();
    check(active.elevator.active && active.ltlio_lidar_update.attempted &&
          active.ltlio_lidar_update.accepted,
          "transport marker scan did not pass real LiDAR matching");
    check(active.odom_prior_active == use_odom_prior,
          "fresh odometry prior was not active during transport");
    check(!use_odom_prior || active.odom_prior_age_s < 0.01,
          "transport odometry prior was stale instead of scan-aligned");
    check(active.registered_cloud_body && hasMarker(*active.registered_cloud_body),
          "active-only marker was discarded before reaching LTLIO");
    checkSavedMap(*backend, directory / "during" / "map.pcd");
    check(backend->outputs().elevator.active, "saving unexpectedly exited transport");

    check(backend->setElevator(false).ok, "manual transport exit failed");
    process(*backend, 1.4, next_imu, false, use_odom_prior);
    check(!backend->outputs().elevator.active &&
          backend->outputs().ltlio_lidar_update.accepted,
          "transport exit did not continue accepted matching in the same core");
    checkSavedMap(*backend, directory / "exit" / "map.pcd");
    for (int i = 1; i < 6; ++i)
        process(*backend, 1.4 + i * 0.1, next_imu, false, use_odom_prior);
    const auto after = backend->outputs();
    check(!after.elevator.active && after.state == SlamState::Tracking &&
          after.ltlio_lidar_update.accepted,
          "normal LiDAR processing did not resume after transport");
    checkSavedMap(*backend, directory / "after" / "map.pcd");
    std::cout << "Elevator map persistence passed: "
              << (use_odom_prior ? "fresh odometry prior" : "LiDAR-only pose") << '\n';
}
}

int main() {
    const auto directory = std::filesystem::temp_directory_path() /
        ("lingtu-elevator-persistence-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    try {
        checkPersistence(directory / "without-prior", false);
        checkPersistence(directory / "with-prior", true);
        std::filesystem::remove_all(directory);
        std::cout << "Elevator map persistence checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\nTest files: " << directory << '\n';
        return 1;
    }
}
