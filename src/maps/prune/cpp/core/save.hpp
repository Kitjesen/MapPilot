#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "core/types.hpp"

namespace lingtu::map_cleaning {

struct SaveOptions {
  std::filesystem::path map_pcd;
  std::filesystem::path clean_pcd;
  std::filesystem::path removed_pcd;
  std::filesystem::path backup_pcd;
  std::filesystem::path tmp_map_pcd;
  bool overwrite{false};
  bool apply_to_map{false};
};

struct SaveResult {
  bool success{false};
  std::string reason_code;
  std::string message;
  std::filesystem::path backup_pcd;
};

SaveResult writeCleanedMap(const SaveOptions &options, const std::vector<PointXYZI> &kept,
                           const std::vector<PointXYZI> &removed);

}  // namespace lingtu::map_cleaning
