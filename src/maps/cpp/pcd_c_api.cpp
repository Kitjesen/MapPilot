#include "lingtu/maps/c_api/pcd.h"

#include "lingtu/maps/build/pcd.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace {

bool IsAccepted(float x, float y, float z, float max_abs_m) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  if (max_abs_m <= 0.0F) {
    return true;
  }
  return std::abs(x) < max_abs_m && std::abs(y) < max_abs_m &&
         std::abs(z) < max_abs_m;
}

}  // namespace

int32_t lingtu_maps_write_xyz_pcd(
    const char* path,
    const float* points,
    uint64_t point_count,
    uint32_t stride_floats,
    float max_abs_m,
    uint64_t* written_points) {
  if (written_points != nullptr) {
    *written_points = 0U;
  }
  if (path == nullptr || path[0] == '\0' || written_points == nullptr ||
      stride_floats < 3U || (point_count > 0U && points == nullptr)) {
    return -1;
  }

  try {
    std::vector<lingtu::maps::PointXyz> accepted;
    accepted.reserve(static_cast<std::size_t>(point_count));
    for (uint64_t index = 0U; index < point_count; ++index) {
      const float* point = points + index * static_cast<uint64_t>(stride_floats);
      if (!IsAccepted(point[0], point[1], point[2], max_abs_m)) {
        continue;
      }
      accepted.push_back({point[0], point[1], point[2]});
    }

    std::string error;
    if (!lingtu::maps::WriteBinaryXyzPcd(
            std::filesystem::path(path), accepted, &error)) {
      return -2;
    }
    *written_points = static_cast<uint64_t>(accepted.size());
    return 0;
  } catch (...) {
    return -3;
  }
}
