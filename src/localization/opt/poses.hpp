#pragma once
#include <filesystem>
#include <string>
#include <vector>
namespace lingtu::localization::opt {
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

struct Keyframe {
  std::string patch_name;
  Pose pose;
};

std::vector<Keyframe> read_poses(const std::filesystem::path& path);
}
