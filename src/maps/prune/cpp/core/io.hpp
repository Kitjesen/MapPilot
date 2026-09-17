#pragma once

#include <filesystem>
#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/types.hpp"

namespace lingtu::map_cleaning {

std::vector<PointXYZI> readPcd(const std::filesystem::path &path);

void writePcd(const std::filesystem::path &path, const std::vector<PointXYZI> &points);

std::unordered_map<std::string, Pose> readLingtuPoses(const std::filesystem::path &path,
    std::vector<std::string>* patch_order = nullptr);

std::optional<std::array<float, 3>> readSensorOrigin(const std::filesystem::path& path);

}  // namespace lingtu::map_cleaning
