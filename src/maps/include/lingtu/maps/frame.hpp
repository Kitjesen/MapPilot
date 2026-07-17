#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace lingtu::maps {

struct TransformStamped {
  std::string parent_frame_id{"map"};
  std::string child_frame_id;
  std::int64_t stamp_ns{0};
  std::array<double, 3> translation_m{0.0, 0.0, 0.0};
  std::array<double, 4> rotation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct FrameContract {
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
};

}  // namespace lingtu::maps
