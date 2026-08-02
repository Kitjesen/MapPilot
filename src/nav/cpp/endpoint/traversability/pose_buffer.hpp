#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

class PoseBuffer {
 public:
  explicit PoseBuffer(double window_s = 2.0) : window_s_(std::max(0.1, window_s)) {}

  void push(double stamp_s, const nav_kernel::Pose &pose) {
    if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
      return;
    }
    const auto it =
        std::lower_bound(entries_.begin(), entries_.end(), stamp_s,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    if (it != entries_.end() && std::abs(it->stamp_s - stamp_s) <= 1e-9) {
      it->pose = pose;
    } else {
      entries_.insert(it, Entry{stamp_s, pose});
    }
    const double cutoff = entries_.back().stamp_s - window_s_;
    const auto keep =
        std::lower_bound(entries_.begin(), entries_.end(), cutoff,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    entries_.erase(entries_.begin(), keep);
  }

  std::optional<nav_kernel::Pose> sample(double stamp_s, double max_gap_s) const {
    if (entries_.empty() || !std::isfinite(stamp_s) || stamp_s <= 0.0) {
      return std::nullopt;
    }
    const double gap = std::max(0.0, max_gap_s);
    const auto right =
        std::lower_bound(entries_.begin(), entries_.end(), stamp_s,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    if (right != entries_.end() && std::abs(right->stamp_s - stamp_s) <= 1e-9) {
      return right->pose;
    }
    if (right == entries_.begin()) {
      return std::abs(right->stamp_s - stamp_s) <= gap
                 ? std::optional<nav_kernel::Pose>(right->pose)
                 : std::nullopt;
    }
    if (right == entries_.end()) {
      const Entry &last = entries_.back();
      return std::abs(stamp_s - last.stamp_s) <= gap ? std::optional<nav_kernel::Pose>(last.pose)
                                                     : std::nullopt;
    }
    const Entry &after = *right;
    const Entry &before = *(right - 1);
    if (stamp_s - before.stamp_s > gap || after.stamp_s - stamp_s > gap) {
      return std::nullopt;
    }
    const double dt = after.stamp_s - before.stamp_s;
    if (dt <= 1e-9) {
      return before.pose;
    }
    const double t = (stamp_s - before.stamp_s) / dt;
    nav_kernel::Pose out;
    out.position.x = before.pose.position.x + t * (after.pose.position.x - before.pose.position.x);
    out.position.y = before.pose.position.y + t * (after.pose.position.y - before.pose.position.y);
    out.position.z = before.pose.position.z + t * (after.pose.position.z - before.pose.position.z);
    out.yaw = before.pose.yaw + t * wrap(after.pose.yaw - before.pose.yaw);
    out.yaw = wrap(out.yaw);
    return out;
  }

  double nearestGap(double stamp_s) const {
    if (entries_.empty() || !std::isfinite(stamp_s)) {
      return std::numeric_limits<double>::infinity();
    }
    const auto right =
        std::lower_bound(entries_.begin(), entries_.end(), stamp_s,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    if (right == entries_.begin()) {
      return std::abs(right->stamp_s - stamp_s);
    }
    if (right == entries_.end()) {
      return std::abs(stamp_s - entries_.back().stamp_s);
    }
    return std::min(std::abs(stamp_s - (right - 1)->stamp_s), std::abs(right->stamp_s - stamp_s));
  }

 private:
  struct Entry {
    double stamp_s{0.0};
    nav_kernel::Pose pose{};
  };

  static double wrap(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  double window_s_{2.0};
  std::vector<Entry> entries_;
};

}  // namespace lingtu::nav::endpoint
