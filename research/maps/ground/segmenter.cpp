#include "ground/segmenter.hpp"
#include "patchwork/patchworkpp.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace lingtu::maps::ground {
struct PatchworkGroundSegmenter::Impl {
  std::unique_ptr<patchwork::PatchWorkpp> engine;
};
PatchworkGroundSegmenter::PatchworkGroundSegmenter() : impl_(std::make_unique<Impl>()) {}
PatchworkGroundSegmenter::~PatchworkGroundSegmenter() = default;
void PatchworkGroundSegmenter::Reset() { impl_ = std::make_unique<Impl>(); }
GroundSegmentation PatchworkGroundSegmenter::Segment(std::span<const Point3f> points) {
  GroundSegmentation result;
  result.ground_mask.resize(points.size());
  if (!impl_->engine) {
    // Estimate only the initial height. Later frames retain the upstream engine's
    // adaptive thresholds even when ground is visible in just a few sectors.
    std::array<std::vector<float>, 16> sectors;
    for (auto p : points) {
      const float radius = std::hypot(p.x, p.y);
      if (!std::isfinite(p.z) || !std::isfinite(radius)) continue;
      if (radius < .5F || radius > 5.F || p.z >= -.15F || p.z < -3.F) continue;
      int sector = static_cast<int>((std::atan2(p.y, p.x) + 3.141592654F) * 16 / 6.283185308F);
      sectors[std::clamp(sector, 0, 15)].push_back(p.z);
    }
    std::vector<float> lows;
    for (auto& sector : sectors) {
      if (sector.size() < 6) continue;
      const auto at = sector.begin() + sector.size() / 5;
      std::nth_element(sector.begin(), at, sector.end());
      lows.push_back(*at);
    }
    if (lows.size() < 6) return result;
    std::sort(lows.begin(), lows.end());
    const float height = -lows[lows.size() / 2];
    // Disjoint levels must not seed a fictitious initial floor.
    if (lows[lows.size() * 3 / 4] - lows[lows.size() / 4] > .35F) return result;
    patchwork::Params params;
    params.enable_RNR = false; // XYZ has no calibrated reflectivity.
    params.verbose = false;
    params.sensor_height = height;
    params.min_range = .3;
    params.max_range = 40.;
    params.th_dist = .04;
    params.th_seeds = .08;
    impl_->engine = std::make_unique<patchwork::PatchWorkpp>(params);
  }
  std::vector<std::size_t> indices;
  Eigen::MatrixXf input(points.size(), 3);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto p = points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    input.row(indices.size()) << p.x, p.y, p.z;
    indices.push_back(i);
  }
  input.conservativeResize(indices.size(), 3);
  if (indices.empty()) return result;
  impl_->engine->estimateGround(std::move(input));
  for (auto index : impl_->engine->getGroundIndices())
    if (index >= 0 && static_cast<std::size_t>(index) < indices.size())
      result.ground_mask[indices[index]] = 1;
  result.sensor_height_m = static_cast<float>(impl_->engine->getHeight());
  result.segmented = true;
  return result;
}
}
