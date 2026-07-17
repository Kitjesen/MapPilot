#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace lingtu::map_cleaning {

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct Pose {
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
};

struct VoxelKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t h = 1469598103934665603ULL;
    auto mix = [&](int value) {
      h ^= static_cast<std::uint64_t>(static_cast<std::int64_t>(value));
      h *= 1099511628211ULL;
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return h;
  }
};

struct VoxelEvidence {
  std::uint32_t hits{0};
  std::uint32_t ground_hits{0};
  std::uint32_t frame_count{0};
  std::size_t last_frame{std::numeric_limits<std::size_t>::max()};
};

}  // namespace lingtu::map_cleaning
