#pragma once

#include <filesystem>

#include "worker.hpp"

namespace lingtu::localization::online_mapping {

struct ArchiveInfo {
  std::uint64_t source_epoch = 0;
  map_optimization::Pose map_from_archive;
  map_optimization::Pose body_from_imu;
  map_optimization::Pose imu_from_lidar;
  std::string map_frame;
  std::string body_frame;
};

struct Archive {
  ArchiveInfo info;
  Config config;
  std::vector<MappingFrame> frames;
  std::vector<LoopResult> records;
  std::uint64_t next_sequence = 0;
};

void writeArchiveInfo(const std::filesystem::path &directory, const ArchiveInfo &info,
                      bool complete = true);
ArchiveInfo readArchiveInfo(const std::filesystem::path &directory);
Archive readArchive(const std::filesystem::path &directory, std::size_t capacity);

}  // namespace lingtu::localization::online_mapping
