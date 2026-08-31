#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "input/frame.hpp"

namespace lingtu::nav::endpoint {

class TransformBuffer {
 public:
  explicit TransformBuffer(double window_s = 2.0) : window_s_(std::max(0.1, window_s)) {}

  void push(double stamp_s, RigidTransform transform) {
    if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || !transform.valid) {
      return;
    }
    transform.stamp_s = stamp_s;
    transform.rotation = normalizeQuaternion(transform.rotation);
    transform.yaw = quaternionYaw(transform.rotation);
    const auto it =
        std::lower_bound(entries_.begin(), entries_.end(), stamp_s,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    if (it != entries_.end() && std::abs(it->stamp_s - stamp_s) <= 1e-9) {
      it->transform = transform;
    } else {
      entries_.insert(it, Entry{stamp_s, transform});
    }
    const double cutoff = entries_.back().stamp_s - window_s_;
    const auto keep =
        std::lower_bound(entries_.begin(), entries_.end(), cutoff,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    entries_.erase(entries_.begin(), keep);
  }

  std::optional<RigidTransform> sample(double stamp_s, double max_gap_s) const {
    if (entries_.empty() || !std::isfinite(stamp_s) || stamp_s <= 0.0) {
      return std::nullopt;
    }
    const double gap = std::max(0.0, max_gap_s);
    const auto right =
        std::lower_bound(entries_.begin(), entries_.end(), stamp_s,
                         [](const Entry &entry, double stamp) { return entry.stamp_s < stamp; });
    if (right != entries_.end() && std::abs(right->stamp_s - stamp_s) <= 1e-9) {
      return right->transform;
    }
    if (right == entries_.begin()) {
      return std::abs(right->stamp_s - stamp_s) <= gap
                 ? std::optional<RigidTransform>(right->transform)
                 : std::nullopt;
    }
    if (right == entries_.end()) {
      const Entry &last = entries_.back();
      return std::abs(stamp_s - last.stamp_s) <= gap ? std::optional<RigidTransform>(last.transform)
                                                     : std::nullopt;
    }
    const Entry &after = *right;
    const Entry &before = *(right - 1);
    if (stamp_s - before.stamp_s > gap || after.stamp_s - stamp_s > gap) {
      return std::nullopt;
    }
    const double duration = after.stamp_s - before.stamp_s;
    if (duration <= 1e-9) {
      return before.transform;
    }
    const double ratio = (stamp_s - before.stamp_s) / duration;
    return interpolate(before.transform, after.transform, ratio, stamp_s);
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

  void clear() { entries_.clear(); }

 private:
  struct Entry {
    double stamp_s{0.0};
    RigidTransform transform{};
  };

  static RigidTransform interpolate(const RigidTransform &before, const RigidTransform &after,
                                    double ratio, double stamp_s) {
    const double t = std::clamp(ratio, 0.0, 1.0);
    RigidTransform out;
    out.translation = {
        before.translation.x + t * (after.translation.x - before.translation.x),
        before.translation.y + t * (after.translation.y - before.translation.y),
        before.translation.z + t * (after.translation.z - before.translation.z),
    };
    Quaternion lhs = normalizeQuaternion(before.rotation);
    Quaternion rhs = normalizeQuaternion(after.rotation);
    const double dot = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
    if (dot < 0.0) {
      rhs.x = -rhs.x;
      rhs.y = -rhs.y;
      rhs.z = -rhs.z;
      rhs.w = -rhs.w;
    }
    out.rotation = normalizeQuaternion({
        lhs.x + t * (rhs.x - lhs.x),
        lhs.y + t * (rhs.y - lhs.y),
        lhs.z + t * (rhs.z - lhs.z),
        lhs.w + t * (rhs.w - lhs.w),
    });
    out.yaw = quaternionYaw(out.rotation);
    out.stamp_s = stamp_s;
    out.valid = before.valid && after.valid;
    return out;
  }

  double window_s_{2.0};
  std::vector<Entry> entries_;
};

}  // namespace lingtu::nav::endpoint
