#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace nav_kernel {

struct DynamicClearParams {
  bool enabled = true;
  double voxelSize = 0.20;
  double weakTtlS = 0.80;
  double staticTtlS = 3.0;
  std::uint32_t staticMinHits = 3;
  std::uint32_t staticMinFrames = 2;
  bool raycastClearing = true;
  std::uint32_t raycastClearMinFrames = 2;
  double raycastMaxRange = 6.0;
  std::size_t maxRayCount = 512;
};

struct DynamicClearStats {
  std::size_t input_points = 0;
  std::size_t kept_points = 0;
  std::size_t dynamic_points = 0;
  std::size_t ray_cleared_points = 0;
  std::size_t current_points = 0;
  std::size_t evidence_voxels = 0;
  std::size_t free_voxels = 0;
  std::size_t raycast_rays = 0;
  std::size_t raycast_voxels = 0;
};

struct DynamicClearResult {
  std::vector<float> kept_xyzi;
  std::vector<float> dynamic_xyzi;
  DynamicClearStats stats;
};

struct DynamicClearOrigin {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  bool valid = false;
};

class DynamicClearCore {
 public:
  explicit DynamicClearCore(const DynamicClearParams& params = DynamicClearParams())
      : p_(params) {}

  void reset() {
    evidence_.clear();
    frame_id_ = 0;
  }

  const DynamicClearParams& params() const { return p_; }

  DynamicClearResult filter(
      const std::vector<float>& rolling_xyzi,
      const std::vector<float>& current_xyz,
      double stamp_s) {
    return filter(rolling_xyzi, current_xyz, DynamicClearOrigin{}, stamp_s);
  }

  DynamicClearResult filter(
      const std::vector<float>& rolling_xyzi,
      const std::vector<float>& current_xyz,
      const DynamicClearOrigin& origin,
      double stamp_s) {
    DynamicClearResult result;
    result.stats.input_points = rolling_xyzi.size() / 4;
    result.stats.current_points = current_xyz.size() / 3;

    if (!p_.enabled) {
      result.kept_xyzi = rolling_xyzi;
      result.stats.kept_points = result.stats.input_points;
      result.stats.evidence_voxels = evidence_.size();
      return result;
    }
    if (!(p_.voxelSize > 0.0) || !std::isfinite(p_.voxelSize)) {
      result.kept_xyzi = rolling_xyzi;
      result.stats.kept_points = result.stats.input_points;
      result.stats.evidence_voxels = evidence_.size();
      return result;
    }

    ++frame_id_;
    const std::uint64_t current_frame = frame_id_;
    ingestCurrent(current_xyz, origin, stamp_s, current_frame, result.stats);

    result.kept_xyzi.reserve(rolling_xyzi.size());
    result.dynamic_xyzi.reserve(rolling_xyzi.size());
    const std::size_t n = rolling_xyzi.size() / 4;
    for (std::size_t i = 0; i < n; ++i) {
      const float x = rolling_xyzi[i * 4 + 0];
      const float y = rolling_xyzi[i * 4 + 1];
      const float z = rolling_xyzi[i * 4 + 2];
      const float intensity = rolling_xyzi[i * 4 + 3];
      const DynamicVoxelKey key = voxelKey(x, y, z);
      const auto found = evidence_.find(key);
      bool ray_cleared = false;
      const bool keep = found != evidence_.end() &&
                        shouldKeep(found->second, stamp_s, current_frame, ray_cleared);
      if (keep) {
        appendPoint(result.kept_xyzi, x, y, z, intensity);
      } else {
        appendPoint(result.dynamic_xyzi, x, y, z, -std::max(0.1f, std::fabs(intensity)));
        if (ray_cleared) {
          ++result.stats.ray_cleared_points;
        }
      }
    }

    pruneEvidence(stamp_s);
    result.stats.kept_points = result.kept_xyzi.size() / 4;
    result.stats.dynamic_points = result.dynamic_xyzi.size() / 4;
    result.stats.evidence_voxels = evidence_.size();
    for (const auto& entry : evidence_) {
      if (entry.second.free_hits > 0) {
        ++result.stats.free_voxels;
      }
    }
    return result;
  }

 private:
  struct DynamicVoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const DynamicVoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct DynamicVoxelHash {
    std::size_t operator()(const DynamicVoxelKey& key) const {
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

  struct Evidence {
    std::uint32_t hits = 0;
    std::uint32_t frames = 0;
    std::uint64_t last_hit_frame = 0;
    std::uint32_t free_hits = 0;
    std::uint32_t free_frames = 0;
    std::uint64_t last_free_frame = 0;
    double last_seen_s = -std::numeric_limits<double>::infinity();
    double last_cleared_s = -std::numeric_limits<double>::infinity();
  };

  struct CurrentPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };

  DynamicClearParams p_;
  std::unordered_map<DynamicVoxelKey, Evidence, DynamicVoxelHash> evidence_;
  std::uint64_t frame_id_ = 0;

  DynamicVoxelKey voxelKey(float x, float y, float z) const {
    const double inv = 1.0 / p_.voxelSize;
    return {
        static_cast<int>(std::floor(static_cast<double>(x) * inv)),
        static_cast<int>(std::floor(static_cast<double>(y) * inv)),
        static_cast<int>(std::floor(static_cast<double>(z) * inv)),
    };
  }

  void ingestCurrent(
      const std::vector<float>& current_xyz,
      const DynamicClearOrigin& origin,
      double stamp_s,
      std::uint64_t current_frame,
      DynamicClearStats& stats) {
    const std::size_t n = current_xyz.size() / 3;
    std::unordered_set<DynamicVoxelKey, DynamicVoxelHash> hit_keys;
    hit_keys.reserve(n);
    std::vector<CurrentPoint> endpoints;
    endpoints.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      const float x = current_xyz[i * 3 + 0];
      const float y = current_xyz[i * 3 + 1];
      const float z = current_xyz[i * 3 + 2];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const DynamicVoxelKey key = voxelKey(x, y, z);
      if (!hit_keys.insert(key).second) {
        continue;
      }
      endpoints.push_back({x, y, z});
      Evidence& ev = evidence_[key];
      ++ev.hits;
      if (ev.last_hit_frame != current_frame) {
        ev.last_hit_frame = current_frame;
        ++ev.frames;
        // A fresh hit invalidates all accumulated raycast-free evidence for
        // this voxel; without this reset, stale free_frames from a previous
        // clearing episode could incorrectly remove a re-observed obstacle.
        ev.free_hits = 0;
        ev.free_frames = 0;
      }
      ev.last_seen_s = stamp_s;
    }

    if (!p_.raycastClearing || !origin.valid || endpoints.empty() ||
        p_.maxRayCount == 0) {
      return;
    }
    const std::size_t stride = endpoints.size() > p_.maxRayCount
        ? static_cast<std::size_t>(std::ceil(
              static_cast<double>(endpoints.size()) /
              static_cast<double>(p_.maxRayCount)))
        : 1;
    std::unordered_set<DynamicVoxelKey, DynamicVoxelHash> free_keys;
    free_keys.reserve(p_.maxRayCount * 64);
    for (std::size_t i = 0; i < endpoints.size(); i += stride) {
      collectRayFree(origin, endpoints[i], hit_keys, free_keys);
      ++stats.raycast_rays;
    }
    stats.raycast_voxels = free_keys.size();
    for (const auto& key : free_keys) {
      if (hit_keys.find(key) != hit_keys.end()) {
        continue;
      }
      Evidence& ev = evidence_[key];
      ++ev.free_hits;
      if (ev.last_free_frame != current_frame) {
        ev.last_free_frame = current_frame;
        ++ev.free_frames;
      }
      ev.last_cleared_s = stamp_s;
    }
  }

  void collectRayFree(
      const DynamicClearOrigin& origin,
      const CurrentPoint& endpoint,
      const std::unordered_set<DynamicVoxelKey, DynamicVoxelHash>& occupied,
      std::unordered_set<DynamicVoxelKey, DynamicVoxelHash>& free_keys) const {
    const double dx = static_cast<double>(endpoint.x) - origin.x;
    const double dy = static_cast<double>(endpoint.y) - origin.y;
    const double dz = static_cast<double>(endpoint.z) - origin.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(range > p_.voxelSize) || !std::isfinite(range)) {
      return;
    }
    const double max_range =
        p_.raycastMaxRange > 0.0 ? std::min(range, p_.raycastMaxRange) : range;
    const double stop = std::max(0.0, std::min(max_range, range - p_.voxelSize));
    const double step = std::max(0.05, p_.voxelSize * 0.5);
    const int steps = static_cast<int>(std::floor(stop / step));
    for (int i = 1; i <= steps; ++i) {
      const double t = (static_cast<double>(i) * step) / range;
      const DynamicVoxelKey key = voxelKey(
          static_cast<float>(origin.x + dx * t),
          static_cast<float>(origin.y + dy * t),
          static_cast<float>(origin.z + dz * t));
      // Stop at the first currently-occupied voxel: the ray is blocked there,
      // so voxels beyond it must not be marked free (prevents clearing
      // legitimate obstacles hidden behind a nearer surface).
      if (occupied.count(key) != 0) {
        break;
      }
      free_keys.insert(key);
    }
  }

  bool shouldKeep(
      const Evidence& ev,
      double stamp_s,
      std::uint64_t current_frame,
      bool& ray_cleared) const {
    ray_cleared = false;
    if (ev.last_hit_frame == current_frame) {
      return true;
    }
    if (ev.last_cleared_s > ev.last_seen_s &&
        ev.free_frames >= p_.raycastClearMinFrames) {
      ray_cleared = true;
      return false;
    }
    const double age = stamp_s - ev.last_seen_s;
    if (age <= p_.weakTtlS) {
      return true;
    }
    const bool static_evidence =
        ev.hits >= p_.staticMinHits && ev.frames >= p_.staticMinFrames;
    return static_evidence && age <= p_.staticTtlS;
  }

  void pruneEvidence(double stamp_s) {
    const double max_age = std::max(p_.weakTtlS, p_.staticTtlS);
    for (auto it = evidence_.begin(); it != evidence_.end();) {
      const double last_activity = std::max(it->second.last_seen_s, it->second.last_cleared_s);
      if (stamp_s - last_activity > max_age) {
        it = evidence_.erase(it);
      } else {
        ++it;
      }
    }
  }

  static void appendPoint(std::vector<float>& out, float x, float y, float z, float intensity) {
    out.push_back(x);
    out.push_back(y);
    out.push_back(z);
    out.push_back(intensity);
  }
};

}  // namespace nav_kernel
