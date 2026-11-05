#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>

#include "slam.hpp"

using namespace lingtu::slam;

void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}

int main() {
  try {
    const auto root = std::filesystem::temp_directory_path() /
                      ("lingtu-geometry-test-" + std::to_string(newSourceEpoch()));
    std::filesystem::create_directories(root);
    const auto config_path = root / "config.yaml";
    {
      std::ofstream config(config_path);
      config << "odom_prior_enabled: true\nodom_prior_bypass_ltlio: true\n"
             << "elevator_mode: off\nlidar_filter_num: 1\n"
             << "max_sensor_time_jump_s: 10\npatch_min_interval_s: 0\n"
             << "max_patch_snapshots: 24\n"
             << "patch_min_translation_m: 0\npatch_min_rotation_rad: 0\n"
             << "t_il: [0.02, -0.06, 0.04]\n"
             << "online_mapping:\n  voxel_m: 0.15\n  min_time_s: 30\n  submap_half_window: 2\n";
    }
    auto backend = makeLtlioBackend();
    SlamConfig config;
    config.config_path = config_path.string();
    require(backend->configure(config).ok, "configuration failed");
    require(backend->setMode(SlamMode::Mapping, {}).ok, "mapping mode failed");
    std::optional<MapGeometrySnapshot> first;
    LidarFrame last_frame;
    for (int i = 0; i < 23; ++i) {
      const int position = i <= 10 ? i : std::max(0, 21 - i);
      const double actual_x = position * .3;
      OdomSample prior;
      prior.stamp_s = i * 3. + 1;
      prior.odom_body.x = actual_x + i * (.075 / 21);
      prior.odom_body.z = i * (.045 / 21);
      require(backend->feedVisualOdom(prior).ok, "prior failed");
      ImuSample imu;
      imu.stamp_s = prior.stamp_s;
      imu.az = 9.80665;
      require(backend->feedImu(imu).ok, "imu failed");
      LidarFrame frame;
      frame.stamp_s = prior.stamp_s;
      for (int a = -12; a <= 12; ++a)
        for (int b = -12; b <= 12; ++b) {
          const float u = a * .2F, v = b * .2F, x = static_cast<float>(actual_x);
          const float noise = position * .002F * std::sin(.7F * u + .3F * v);
          frame.points.push_back({u - x, v, noise, 1});
          frame.points.push_back({u - x, -2.2F + noise, v, 1});
          frame.points.push_back({2.4F + noise - x, u, v, 1});
        }
      require(backend->feedLidar(frame).ok && backend->tick().ok, "scan failed");
      last_frame = frame;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(25);
      do {
        require(backend->tick().ok, "mapping poll failed");
        if (!first)
          first = backend->mapGeometrySnapshot();
        const auto out = backend->outputs();
        require(out.odometry_odom_body &&
                    std::abs(out.odometry_odom_body->x - prior.odom_body.x) < 1e-9 &&
                    std::abs(out.odometry_odom_body->z - prior.odom_body.z) < 1e-9,
                "historical correction moved continuous odometry");
        if (!out.global_map_busy && out.global_map_keyframes == static_cast<std::size_t>(i + 1))
          break;
        require(std::chrono::steady_clock::now() < deadline, "graph timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } while (true);
    }
    const auto await_snapshot = [&](std::uint64_t after_revision) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      do {
        if (auto snapshot = backend->mapGeometrySnapshot();
            snapshot && snapshot->graph_revision > after_revision)
          return *snapshot;
        require(std::chrono::steady_clock::now() < deadline, "geometry export timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } while (true);
    };
    if (!first)
      first = await_snapshot(0);
    require(std::filesystem::exists(std::filesystem::path(first->frozen_dir) / "poses.txt"),
            "bundle missing");
    require(!backend->saveMapAsyncInFlight() && !backend->outputs().map_loaded,
            "geometry export changed public save state");
    backend->acknowledgeMapGeometry(first->reset_epoch + 1, first->graph_revision, true);
    require(backend->mapGeometrySnapshot()->graph_revision == first->graph_revision,
            "late ack changed pending export");
    backend->acknowledgeMapGeometry(first->reset_epoch, first->graph_revision, false,
                                    "temporary_read_error");
    require(backend->mapGeometrySnapshot()->graph_revision == first->graph_revision,
            "retry changed frozen revision");
    backend->acknowledgeMapGeometry(first->reset_epoch, first->graph_revision, true);
    require(std::filesystem::exists(first->frozen_dir),
            "confirmed bundle removed before replacement");
    const auto latest = await_snapshot(first->graph_revision);
    require(latest.keyframe_count == 23 && latest.captured_sequence == 23,
            "export did not freeze the graph prefix");
    require(std::filesystem::exists(first->frozen_dir),
            "old confirmed bundle removed before new ack");
    std::ifstream calibration(std::filesystem::path(latest.frozen_dir) / "scan_origin.txt");
    std::string tag;
    double x = 0, y = 0, z = 0;
    calibration >> tag >> x >> y >> z;
    require(tag == "lidar_origin_in_patch" && std::abs(x - .02) < 1e-9 &&
                std::abs(y + .06) < 1e-9 && std::abs(z - .04) < 1e-9,
            "physical scan origin missing");
    calibration.close();
    backend->acknowledgeMapGeometry(latest.reset_epoch, latest.graph_revision, true);
    require(!std::filesystem::exists(first->frozen_dir) &&
                std::filesystem::exists(latest.frozen_dir),
            "confirmed export ownership did not advance");
    const auto optimizations = backend->outputs().global_map_optimizations;
    OdomSample far_prior;
    far_prior.stamp_s = 70;
    far_prior.odom_body.x = 1000;
    ImuSample far_imu;
    far_imu.stamp_s = 70;
    far_imu.az = 9.80665;
    last_frame.stamp_s = 70;
    require(backend->feedVisualOdom(far_prior).ok && backend->feedImu(far_imu).ok &&
                backend->feedLidar(last_frame).ok && backend->tick().ok,
            "ordinary append failed");
    const auto append_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(25);
    while (backend->outputs().global_map_busy || backend->outputs().global_map_keyframes < 24) {
      require(backend->tick().ok, "append poll failed");
      require(std::chrono::steady_clock::now() < append_deadline, "append timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(backend->outputs().global_map_optimizations == optimizations,
            "ordinary append fabricated loop");
    const auto unchanged = backend->mapGeometrySnapshot();
    require(unchanged && unchanged->graph_revision == latest.graph_revision,
            "ordinary keyframe re-exported history without a new loop");
    const auto full = backend->outputs();
    require(full.global_map_state == "mapping_keyframe_capacity_reached" &&
                full.global_map_optimizations > 0,
            "loop-corrected fixture did not reach mapping capacity");
    far_prior.stamp_s = far_imu.stamp_s = last_frame.stamp_s = 71;
    far_prior.odom_body.x = 1000.2;
    require(backend->feedVisualOdom(far_prior).ok && backend->feedImu(far_imu).ok &&
                backend->feedLidar(last_frame).ok && backend->tick().ok,
            "capacity stopped sensor processing");
    const auto continued = backend->outputs();
    require(continued.state == SlamState::Tracking && continued.confidence > 0 &&
                continued.odometry_odom_body &&
                std::abs(continued.odometry_odom_body->x - 1000.2) < 1e-9 &&
                continued.observation_sequence == full.observation_sequence + 1 &&
                continued.global_map_keyframes == full.global_map_keyframes,
            "capacity stopped localization or grew the frozen graph");
    require(backend->saveMap((root / "saved" / "map.pcd").string()).ok,
            "corrected capacity snapshot failed");
    const auto saved = backend->outputs();
    require(saved.saved_map_cloud_map && saved.saved_map_cloud_map->stamp_s == full.stamp_s &&
                saved.saved_map_observation_sequence == full.observation_sequence,
            "corrected capacity snapshot used a later scan identity");
    std::ifstream report_input(root / "saved" / "map_optimization.json");
    const std::string report((std::istreambuf_iterator<char>(report_input)), {});
    report_input.close();
    require(report.find("\"performed\":true") != std::string::npos &&
                report.find("\"pose_count\":24") != std::string::npos &&
                report.find("\"evidence_complete\":true") != std::string::npos,
            "corrected capacity snapshot lost its complete graph prefix");
    require(backend->setMode(SlamMode::Localization, {}).ok && !backend->mapGeometrySnapshot(),
            "localization advertised mapping geometry");
    require(backend->reset().ok, "reset failed");
    require(!std::filesystem::exists(latest.frozen_dir) && !backend->mapGeometrySnapshot(),
            "old epoch export survived reset");
    std::filesystem::remove_all(root);
    std::cout << "map geometry handoff passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
