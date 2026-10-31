#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::recording {
struct SceneQuery {
  double start_s{0}, end_s{10};
  std::size_t max_frames{16}, max_points{2048}, max_poses{2000};
  std::string cloud_topic, pose_topic;
  std::vector<std::string> path_topics;
  bool info{false};
};
struct SceneTopic { std::string topic, type_name, kind; std::uint64_t message_count{}; };
struct SceneFrame {
  double t{};
  std::uint64_t source_time_ns{}, original_count{};
  std::string frame_id;
  std::vector<double> points;
  std::array<double, 4> orientation{};
  bool gap_before{}, carried_forward{};
};
struct SceneStream {
  std::string topic;
  std::uint64_t original_count{}, read_count{};
  double gap_threshold_s{};
  std::vector<SceneFrame> frames;
};
struct RecordedScene {
  bool info{};
  std::size_t cloud_points_per_frame{0}, path_points_per_frame{512};
  std::uint64_t start_time_ns{};
  double duration_s{}, start_s{}, end_s{};
  std::vector<SceneTopic> topics;
  std::optional<SceneStream> cloud, trajectory;
  std::vector<SceneStream> paths;
  std::string json() const;
};
RecordedScene read_recorded_scene(const std::filesystem::path &file, const SceneQuery &query);
}  // namespace lingtu::recording
