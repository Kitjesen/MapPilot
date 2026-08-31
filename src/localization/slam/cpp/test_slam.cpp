#include "slam.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace lingtu::slam;

namespace {

std::filesystem::path tempMapDir() {
  auto dir = std::filesystem::temp_directory_path() / "messages_contract_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

bool pathExists(const std::filesystem::path& path) {
  return std::filesystem::exists(path);
}

void check(bool ok, const char* message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  check(modeFromString("mapping") == SlamMode::Mapping, "mapping_mode_parse_failed");
  check(
      modeFromString("localization") == SlamMode::Localization,
      "localization_mode_parse_failed");
  for (const char* rejected : {"localizer", "nav", "localizaton", ""}) {
    bool rejected_mode = false;
    try {
      static_cast<void>(modeFromString(rejected));
    } catch (const std::invalid_argument&) {
      rejected_mode = true;
    }
    check(rejected_mode, "unsupported_mode_was_accepted");
  }

  auto backend = makeContractBackend("fastlio2");
  check(backend != nullptr, "missing_backend");
  check(backend->configure(SlamConfig{}).ok, "configure_failed");
  check(backend->setMode(SlamMode::Mapping, "").ok, "set_mode_failed");

  LidarFrame frame;
  frame.stamp_s = 1.0;
  frame.points.push_back(PointXYZIT{1.0F, 2.0F, 3.0F, 42.0F, 100, 2, 3});
  check(backend->feedLidar(frame).ok, "feed_lidar_failed");

  Pose3d pose;
  pose.x = 1.0;
  pose.y = 2.0;
  check(backend->setInitialPose(pose).ok, "set_initial_pose_failed");
  check(backend->tick().ok, "tick_failed");

  const auto dir = tempMapDir();
  const auto save_status = backend->saveMap((dir / "map.pcd").string());
  check(save_status.ok, "save_map_failed");
  check(pathExists(dir / "map.pcd"), "map_pcd_missing");
  check(pathExists(dir / "poses.txt"), "poses_txt_missing");
  check(pathExists(dir / "trajectory.txt"), "trajectory_txt_missing");
  check(pathExists(dir / "patches"), "patches_dir_missing");
  check(pathExists(dir / "patches" / "latest_scan.pcd"), "latest_scan_patch_missing");

  const auto outputs = backend->outputs();
  check(outputs.alive, "not_alive");
  check(outputs.map_loaded, "map_not_loaded");
  check(outputs.map_cloud_map.has_value(), "map_cloud_missing");
  check(outputs.observation_sequence == 1U, "observation_sequence_missing");
  check(outputs.source_epoch > 0U, "source_epoch_missing");
  check(outputs.map_cloud_map->points[0].offset_time_ns == 100, "point_offset_missing");
  check(!outputs.map_frame_jump, "unexpected_map_frame_jump");
  const auto source_epoch = outputs.source_epoch;
  check(backend->reset().ok, "reset_failed");
  check(backend->outputs().source_epoch > source_epoch, "source_epoch_not_advanced");

  std::cout << "messages_contract ok\n";
  return 0;
}
