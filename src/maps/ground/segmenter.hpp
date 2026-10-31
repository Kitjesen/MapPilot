#pragma once
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace lingtu::maps::ground {
struct Point3f { float x{}, y{}, z{}; };
struct GroundSegmentation {
  std::vector<std::uint8_t> ground_mask;
  // The engine's adaptive height after this frame; zero when segmented is false.
  float sensor_height_m{};
  bool segmented{false};
};
// XYZ must be gravity aligned and relative to the calibrated physical LiDAR origin.
// Labels are preservation evidence only, never navigation ground-support authority.
class PatchworkGroundSegmenter {
 public:
  PatchworkGroundSegmenter();
  ~PatchworkGroundSegmenter();
  GroundSegmentation Segment(std::span<const Point3f> points);
  // Discard learned thresholds and require cold-start height evidence again.
  void Reset();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
