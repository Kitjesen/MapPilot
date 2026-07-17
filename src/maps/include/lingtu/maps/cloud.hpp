#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::maps {

enum class CloudLayout {
  kXyzF32Interleaved,
  kXyziF32Interleaved,
  kXyzF32SoA,
  kXyziF32SoA,
};

struct FloatArrayView {
  const float* data{nullptr};
  std::size_t size{0};
};

struct PointCloudView {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  CloudLayout layout{CloudLayout::kXyzF32Interleaved};
  std::size_t point_count{0};

  FloatArrayView interleaved;
  FloatArrayView x;
  FloatArrayView y;
  FloatArrayView z;
  FloatArrayView intensity;
};

struct OwnedPointCloud {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  CloudLayout layout{CloudLayout::kXyzF32Interleaved};
  std::size_t point_count{0};

  std::vector<float> interleaved;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::vector<float> intensity;

  PointCloudView View() const {
    PointCloudView view;
    view.frame_id = frame_id;
    view.stamp_ns = stamp_ns;
    view.layout = layout;
    view.point_count = point_count;
    view.interleaved = {interleaved.data(), interleaved.size()};
    view.x = {x.data(), x.size()};
    view.y = {y.data(), y.size()};
    view.z = {z.data(), z.size()};
    view.intensity = {intensity.data(), intensity.size()};
    return view;
  }
};

struct MapCloudFrame {
  PointCloudView cloud;
  float sensor_origin_x_m{0.0F};
  float sensor_origin_y_m{0.0F};
  float sensor_origin_z_m{0.0F};
  bool full_map{false};
  bool incremental{true};
  bool keyframe{false};
};

}  // namespace lingtu::maps
