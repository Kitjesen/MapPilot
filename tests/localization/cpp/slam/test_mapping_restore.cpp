#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include "localization/online_mapping/archive.hpp"
#include "slam.hpp"

using namespace lingtu::slam;
namespace mapping = lingtu::localization::online_mapping;

void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}

void scan(ISlamBackend &backend, double stamp, double odom_x, double world_x) {
  OdomSample prior;
  prior.stamp_s = stamp;
  prior.odom_body.x = odom_x;
  ImuSample imu;
  imu.stamp_s = stamp;
  imu.az = 9.80665;
  LidarFrame lidar;
  lidar.stamp_s = stamp;
  for (int a = -14; a <= 14; ++a)
    for (int b = -14; b <= 14; ++b) {
      const float u = a * .2F, v = b * .2F, x = static_cast<float>(world_x);
      lidar.points.push_back({u - x, v, 0, 1});
      lidar.points.push_back({u - x, -2.2F, v, 1});
      lidar.points.push_back({2.4F - x, u, v, 1});
    }
  require(backend.feedVisualOdom(prior).ok, "prior rejected");
  require(backend.feedImu(imu).ok && backend.feedLidar(lidar).ok, "sensor rejected");
  require(backend.tick().ok, "scan rejected");
}

void drain(ISlamBackend &backend) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  do {
    require(backend.tick().ok, "tick failed");
    if (!backend.outputs().global_map_busy)
      return;
    require(std::chrono::steady_clock::now() < deadline, "graph timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (true);
}

int main(int argc, char **argv) {
  try {
    const bool capture_only = argc == 3 && std::string(argv[1]) == "--capture";
    const bool resume_only = argc == 3 && std::string(argv[1]) == "--resume";
    require(argc == 1 || capture_only || resume_only, "invalid test arguments");
    const auto root = argc == 3
                          ? std::filesystem::path(argv[2])
                          : std::filesystem::temp_directory_path() /
                                ("lingtu-mapping-restore-" + std::to_string(newSourceEpoch()));
    std::filesystem::create_directories(root);
    const auto yaml = root / "config.yaml";
    if (!resume_only) {
      std::ofstream out(yaml);
      out << "odom_prior_enabled: true\nodom_prior_bypass_ltlio: true\n"
          << "elevator_mode: off\nlidar_filter_num: 1\nt_il: [0,0,0]\n"
          << "max_sensor_time_jump_s: 10\nmax_patch_snapshots: 8\n"
          << "patch_min_interval_s: 0\npatch_min_translation_m: 0\npatch_min_rotation_rad: 0\n";
    }
    SlamConfig config;
    config.config_path = yaml.string();
    if (!resume_only) {
      auto backend = makeLtlioBackend();
      require(backend->configure(config).ok, "configure failed");
      require(backend->setMode(SlamMode::Mapping, {}).ok, "mapping failed");
      for (int i = 0; i < 3; ++i) {
        scan(*backend, 1 + i * .1, i * .3, i * .3);
        drain(*backend);
      }
      require(backend->outputs().global_map_keyframes == 3, "initial graph missing");
      const auto before = backend->outputs();
      require(!backend->setMode(SlamMode::Localization, (root / "missing.pcd").string()).ok,
              "missing map accepted");
      require(backend->outputs().global_map_keyframes == 3 &&
                  backend->outputs().odometry_odom_body->x == before.odometry_odom_body->x,
              "failed mode switch changed active graph or estimator");
      require(backend->pauseMapping(true).ok, "pause failed");
      for (int i = 0; i < 12; ++i)
        scan(*backend, 1.3 + i * .1, .6, .6);
      drain(*backend);
      require(backend->outputs().global_map_keyframes == 3, "paused graph grew");
      require(backend->setMode(SlamMode::Localization, {}).ok, "localization failed");
      for (int i = 0; i < 12; ++i)
        scan(*backend, 2.5 + i * .1, .6, .6);
      require(backend->setMode(SlamMode::Mapping, {}).ok, "return to mapping failed");
      scan(*backend, 3.8, .9, .9);
      drain(*backend);
      require(backend->outputs().global_map_keyframes == 4, "navigation evicted mapping history");
      const auto saved = backend->saveMap((root / "a" / "map.pcd").string());
      require(saved.ok, saved.message);
      backend.reset();
      if (capture_only) {
        std::cout << "capture complete; process exiting\n";
        return 0;
      }
    }
    const auto archive = mapping::readArchive(root / "a", 8);
    require(archive.frames.size() == 4 &&
                archive.frames.front().keyframe.patch_name == "scan_000000.pcd",
            "saved graph lost its original prefix");
    auto restarted = makeLtlioBackend();
    require(restarted->configure(config).ok, "restart configure failed");
    const auto restored = restarted->restoreMapping((root / "a" / "map.pcd").string());
    require(restored.ok, restored.message);
    require(!restarted->setMode(SlamMode::Mapping, {}).ok,
            "mapping started before finding the robot in the old map");
    scan(*restarted, 1, 0, .9);
    Pose3d guess;
    guess.x = .9;
    const auto aligned = restarted->relocalize(guess);
    require(aligned.ok, aligned.message);
    require(restarted->setMode(SlamMode::Mapping, {}).ok, "aligned mapping did not resume");
    scan(*restarted, 1.1, .3, 1.2);
    drain(*restarted);
    require(restarted->outputs().global_map_keyframes == 5,
            "restart did not extend original graph");
    require(std::abs(restarted->outputs().odometry_odom_body->x - .3) < 1e-8,
            "restoration moved the continuous odometry origin");
    const auto saved_b = restarted->saveMap((root / "b" / "map.pcd").string());
    require(saved_b.ok, saved_b.message);
    require(std::abs(restarted->outputs().saved_map_cloud_map->stamp_s - 1.1) < 1e-8,
            "archive timestamps leaked into the live save receipt");
    const auto continued = mapping::readArchive(root / "b", 8);
    require(continued.frames.size() == 5 && continued.frames.back().relocalized &&
                continued.frames.back().keyframe.patch_name == "scan_000004.pcd" &&
                continued.frames.back().stamp_s > continued.frames[3].stamp_s,
            "restart bridge, numbering or timestamp continuity missing");
    for (std::size_t i = 0; i < archive.frames.size(); ++i)
      require(archive.frames[i].keyframe.pose.x == continued.frames[i].keyframe.pose.x &&
                  archive.frames[i].body_cloud.size() == continued.frames[i].body_cloud.size(),
              "continuation rewrote original measurements");
    require(archive.info.source_epoch == continued.info.source_epoch,
            "restart changed the retained graph's source identity");
    auto unrelated = makeLtlioBackend();
    require(unrelated->configure(config).ok, "unrelated map configure failed");
    scan(*unrelated, 1, .9, .9);
    drain(*unrelated);
    const auto other_saved = unrelated->saveMap((root / "other" / "map.pcd").string());
    require(other_saved.ok, other_saved.message);
    unrelated.reset();
    require(restarted->setMode(SlamMode::Localization, (root / "other" / "map.pcd").string()).ok,
            "fixed-map localization was unnecessarily restricted");
    const auto incompatible = restarted->setMode(SlamMode::Mapping, {});
    require(!incompatible.ok && incompatible.message == "mapping_reference_mismatch",
            "unrelated localization map was joined to the retained graph");
    restarted.reset();
    auto second = makeLtlioBackend();
    require(second->configure(config).ok, "second restart configure failed");
    const auto restored_b = second->restoreMapping((root / "b" / "map.pcd").string());
    require(restored_b.ok, restored_b.message);
    second.reset();
    std::filesystem::remove_all(root);
    std::cout << "mapping pause, restart, registration and continuation passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
