#include "archive.hpp"

#include <cmath>
#include <fstream>
#include <set>

#include "config_yaml.hpp"

namespace lingtu::localization::online_mapping {
namespace {
using map_optimization::Pose;
YAML::Node poseNode(const Pose &pose) {
  YAML::Node node;
  for (double value : {pose.x, pose.y, pose.z, pose.qw, pose.qx, pose.qy, pose.qz})
    node.push_back(value);
  return node;
}

Pose readPose(const YAML::Node &node) {
  if (!node.IsSequence() || node.size() != 7)
    throw std::runtime_error("mapping_archive_invalid_pose");
  std::array<double, 7> v;
  for (std::size_t i = 0; i < v.size(); ++i) {
    v[i] = node[i].as<double>();
    if (!std::isfinite(v[i]))
      throw std::runtime_error("mapping_archive_nonfinite_pose");
  }
  const double norm = v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6];
  if (std::abs(norm - 1.0) > 1e-5)
    throw std::runtime_error("mapping_archive_invalid_rotation");
  return {v[0], v[1], v[2], v[3], v[4], v[5], v[6]};
}
}  // namespace

void writeArchiveInfo(const std::filesystem::path &directory, const ArchiveInfo &info,
                      bool complete) {
  YAML::Node node;
  node["schema"] = "lingtu.mapping_archive.v1";
  node["source_epoch"] = info.source_epoch;
  node["complete"] = complete;
  node["map_frame"] = info.map_frame;
  node["body_frame"] = info.body_frame;
  node["map_from_archive"] = poseNode(info.map_from_archive);
  node["body_from_imu"] = poseNode(info.body_from_imu);
  node["imu_from_lidar"] = poseNode(info.imu_from_lidar);
  YAML::Emitter encoded;
  encoded.SetDoublePrecision(17);
  encoded << node;
  std::ofstream out(directory / "mapping_state.yaml");
  out << encoded.c_str() << '\n';
  out.close();
  if (!out)
    throw std::runtime_error("write_mapping_archive_failed");
}

ArchiveInfo readArchiveInfo(const std::filesystem::path &directory) {
  ArchiveInfo info;
  const auto metadata = YAML::LoadFile((directory / "mapping_state.yaml").string());
  if (metadata["schema"].as<std::string>() != "lingtu.mapping_archive.v1")
    throw std::runtime_error("unsupported_mapping_archive");
  if (!metadata["complete"].as<bool>())
    throw std::runtime_error("mapping_archive_incomplete");
  info.source_epoch = metadata["source_epoch"].as<std::uint64_t>();
  if (info.source_epoch == 0)
    throw std::runtime_error("mapping_archive_source_epoch_missing");
  info.map_frame = metadata["map_frame"].as<std::string>();
  info.body_frame = metadata["body_frame"].as<std::string>();
  info.map_from_archive = readPose(metadata["map_from_archive"]);
  info.body_from_imu = readPose(metadata["body_from_imu"]);
  info.imu_from_lidar = readPose(metadata["imu_from_lidar"]);
  return info;
}

Archive readArchive(const std::filesystem::path &directory, std::size_t capacity) {
  Archive archive;
  archive.info = readArchiveInfo(directory);
  std::ifstream manifest(directory / "patch_bundle.manifest");
  std::string key;
  std::uint64_t value = 0, count = 0;
  bool complete = false, dropped = true;
  if (!(manifest >> key) || key != "LINGTU_PATCH_BUNDLE_V1")
    throw std::runtime_error("mapping_archive_missing_manifest");
  while (manifest >> key >> value) {
    if (key == "complete")
      complete = value == 1;
    else if (key == "dropped_count")
      dropped = value != 0;
    else if (key == "patch_count")
      count = value;
    else if (key == "last_sequence")
      archive.next_sequence = value + 1;
  }
  if (!complete || dropped || count == 0 || count > capacity)
    throw std::runtime_error("mapping_archive_incomplete_or_exceeds_capacity");
  const auto poses = map_optimization::read_poses(directory / "poses.raw.txt");
  if (poses.size() != count)
    throw std::runtime_error("mapping_archive_pose_count_mismatch");
  const auto report = YAML::LoadFile((directory / "map_optimization.json").string());
  if (report["backend"].as<std::string>() != "lio_sam_isam2" || !report["success"].as<bool>() ||
      report["pose_count"].as<std::size_t>() != count)
    throw std::runtime_error("mapping_archive_requires_original_online_graph");
  const auto evidence = YAML::LoadFile((directory / "sam_loops.json").string());
  if (evidence["schema"].as<std::string>() != "lingtu.sam_loops.v1" ||
      !evidence["config"].IsMap() || !evidence["records"].IsSequence() ||
      evidence["records"].size() != count)
    throw std::runtime_error("mapping_archive_incomplete_graph");
  archive.config = readConfig(evidence["config"]);
  std::set<std::size_t> segments;
  if (const auto starts = evidence["segment_starts"])
    for (const auto &start : starts) {
      const auto index = start.as<std::size_t>();
      if (index == 0 || index >= count || !segments.insert(index).second)
        throw std::runtime_error("mapping_archive_invalid_segment");
    }
  std::ifstream stamps(directory / "keyframes.timestamps.txt");
  std::set<std::string> names;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    MappingFrame frame;
    frame.keyframe = poses[i];
    const std::filesystem::path filename(frame.keyframe.patch_name);
    if (filename.filename() != filename || filename.extension() != ".pcd" ||
        !names.insert(filename.string()).second)
      throw std::runtime_error("mapping_archive_invalid_patch_name");
    frame.keyframe.pose = readPose(poseNode(frame.keyframe.pose));
    std::string name;
    if (!(stamps >> name >> frame.stamp_s) || name != filename.string() ||
        !std::isfinite(frame.stamp_s) || (i && frame.stamp_s <= archive.frames.back().stamp_s))
      throw std::runtime_error("mapping_archive_invalid_timestamp");
    frame.body_cloud = map_optimization::read_point_cloud(directory / "patches" / filename);
    if (frame.body_cloud.empty())
      throw std::runtime_error("mapping_archive_empty_patch");
    frame.relocalized = segments.count(i) != 0;
    const auto node = evidence["records"][i];
    LoopResult record;
    record.current = node["current"].as<std::size_t>();
    record.stamp_s = node["stamp_s"].as<double>();
    record.previous = node["previous"].as<int>();
    record.accepted = node["accepted"].as<bool>();
    record.reason = node["reason"].as<std::string>();
    const auto measurement = readPose(node["measurement_xyz_qwxyz"]);
    record.measurement =
        gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(measurement.qw, measurement.qx, measurement.qy,
                                                    measurement.qz)),
                     {measurement.x, measurement.y, measurement.z});
    if (!node["variance"].IsSequence() || node["variance"].size() != 6)
      throw std::runtime_error("mapping_archive_invalid_loop_variance");
    for (std::size_t j = 0; j < 6; ++j)
      record.variance[j] = node["variance"][j].as<double>();
    // Preserve evidence; unaccepted candidates are reverified in the new run.
    for (const auto &target : node["target_frames"])
      record.target_frames.push_back(target.as<std::size_t>());
    record.support_current = node["support_current"].as<int>();
    record.support_previous = node["support_previous"].as<int>();
    record.fitness = node["fitness"].IsNull() ? -1 : node["fitness"].as<double>();
    record.overlap = node["overlap"].as<double>();
    record.inlier_rmse_m = node["inlier_rmse_m"].as<double>();
    record.consistency_translation_m = node["consistency_translation_m"].as<double>();
    record.consistency_rotation_rad = node["consistency_rotation_rad"].as<double>();
    record.correction_translation_m = node["correction_translation_m"].as<double>();
    record.correction_rotation_rad = node["correction_rotation_rad"].as<double>();
    record.source_points = node["source_points"].as<std::size_t>();
    record.target_points = node["target_points"].as<std::size_t>();
    record.candidate_count = node["candidate_count"].as<std::size_t>();
    record.verified_candidates = node["verified_candidates"].as<std::size_t>();
    record.elapsed_ms = node["elapsed_ms"].as<double>();
    archive.frames.push_back(std::move(frame));
    archive.records.push_back(std::move(record));
  }
  if (stamps >> key)
    throw std::runtime_error("mapping_archive_extra_timestamps");
  return archive;
}
}  // namespace lingtu::localization::online_mapping
