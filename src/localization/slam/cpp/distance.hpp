#pragma once

#include <cmath>
#include <optional>

#include "slam.hpp"

namespace lingtu::slam {

// Planar travel in continuous SLAM odom, before global map/GNSS corrections.
// The total belongs to this localization process; a reset only changes its anchor.
class TravelDistance {
 public:
  void update(const SlamOutputs& out) {
    if (!out.odometry_odom_body || !std::isfinite(out.stamp_s) ||
        !std::isfinite(out.odometry_odom_body->x) ||
        !std::isfinite(out.odometry_odom_body->y)) {
      anchor_.reset();
      return;
    }
    const auto& pose = *out.odometry_odom_body;
    if (anchor_ && epoch_ == out.source_epoch) {
      if (out.stamp_s == stamp_s_) return;
      const double dt = out.stamp_s - stamp_s_;
      if (dt > 0.0 && dt <= 2.0) {
        const double segment = std::hypot(pose.x - anchor_->x, pose.y - anchor_->y);
        // Accumulate from a fixed anchor, not each noisy frame. Slow movement
        // still crosses the threshold; the pending remainder is below 5 cm.
        if (segment >= kMinimumSegmentM) {
          meters_ += segment;
          anchor_ = pose;
        }
      } else {
        anchor_ = pose;
      }
    } else {
      anchor_ = pose;
    }
    stamp_s_ = out.stamp_s;
    epoch_ = out.source_epoch;
  }

  [[nodiscard]] std::optional<double> meters() const {
    return anchor_ ? std::optional<double>{meters_} : std::nullopt;
  }

 private:
  static constexpr double kMinimumSegmentM = 0.05;
  std::optional<Pose3d> anchor_;
  double stamp_s_{0.0};
  std::uint64_t epoch_{0};
  double meters_{0.0};
};

}  // namespace lingtu::slam
