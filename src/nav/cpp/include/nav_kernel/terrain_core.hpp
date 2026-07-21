/**
 * nav_kernel/terrain_core.hpp -- ROS-free terrain analysis core.
 *
 * Owns rolling voxel terrain analysis around the robot:
 *   1. Rolling voxel grid around robot
 *   2. Stack incoming scans into voxels
 *   3. Downsample + time decay
 *   4. Ground estimation (quantile Z or min Z)
 *   5. Dynamic obstacle filtering (optional)
 *   6. Terrain map generation (obstacle height above ground)
 *
 * Runtime modules own transport, frames, and point cloud packing.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav_kernel {



struct TerrainParams {
  // Voxel grid
  double scanVoxelSize = 0.05;
  double terrainVoxelSize = 1.0;
  int terrainVoxelHalfWidth = 10;   // grid is (2*half+1)^2
  std::size_t maxPointsPerVoxel = 512;
  std::size_t maxStoredPoints = 0;

  // Decay
  double decayTime = 2.0;
  double noDecayDis = 4.0;
  double clearingDis = 8.0;

  // Ground estimation
  bool useSorting = true;
  double quantileZ = 0.25;
  bool considerDrop = false;
  bool limitGroundLift = false;
  double maxGroundLift = 0.15;

  // Reachable-ground extraction. A height surface is usable only when it is
  // connected to the support surface below the robot. Disconnected high
  // surfaces are retained as diagnostics but never emitted as traversable
  // terrain when this gate is enabled.
  bool checkTerrainConnectivity = false;
  double terrainUnderVehicle = -0.75;
  double terrainConnectionHeight = 0.50;
  double ceilingFilteringHeight = 2.00;
  int terrainConnectivityRadiusCells = 2;
  int groundSeedSearchRadiusCells = 3;
  double maxGroundSeedError = 1.00;

  // Dynamic obstacle
  bool clearDyObs = false;
  double minDyObsDis = 0.3;
  double minDyObsAngle = 0.0;
  double minDyObsRelZ = -0.5;
  double absDyObsRelZThre = 0.2;
  double minDyObsVFOV = -16.0;
  double maxDyObsVFOV = 16.0;
  int minDyObsPointNum = 1;
  int minOutOfFovPointNum = 2;

  // Obstacle thresholds
  double obstacleHeightThre = 0.2;
  bool noDataObstacle = false;
  int noDataBlockSkipNum = 0;
  int minBlockPointNum = 10;
  double vehicleHeight = 1.5;

  // Update thresholds
  int voxelPointUpdateThre = 100;
  double voxelTimeUpdateThre = 2.0;
  double minRelZ = -1.5;
  double maxRelZ = 0.2;
  double disRatioZ = 0.2;

  // Planar grid (finer resolution for ground estimation)
  double planarVoxelSize = 0.2;
  int planarVoxelHalfWidth = 25;  // grid is (2*half+1)^2
};

// ── Result ──

struct TerrainResult {
  // Terrain cloud: each point is (x, y, z, height_above_ground)
  // Stored flat: [x0, y0, z0, h0, x1, y1, z1, h1, ...]
  std::vector<float> terrain_points;
  int n_points = 0;

  // Ground elevation map (planarVoxelWidth x planarVoxelWidth)
  std::vector<float> elevation_map;
  // 0=unobserved/disconnected, 1=connected support surface, -1=ceiling.
  std::vector<std::int8_t> connectivity_map;
  int connected_cells = 0;
  int ceiling_cells = 0;
  int map_width = 0;

  // Vehicle-centric origin of elevation map
  float map_origin_x = 0;
  float map_origin_y = 0;
  float map_resolution = 0;
};

// ── Core Algorithm (stateful --maintains rolling voxel grid) ──

class TerrainAnalysisCore {
public:
  explicit TerrainAnalysisCore(const TerrainParams& params = TerrainParams())
    : p_(params)
  {
    terrainVoxelWidth_ = 2 * p_.terrainVoxelHalfWidth + 1;
    terrainVoxelNum_ = terrainVoxelWidth_ * terrainVoxelWidth_;
    planarVoxelWidth_ = 2 * p_.planarVoxelHalfWidth + 1;
    planarVoxelNum_ = planarVoxelWidth_ * planarVoxelWidth_;

    // Allocate voxel storage
    terrainVoxelCloud_.resize(terrainVoxelNum_);
    terrainVoxelUpdateNum_.resize(terrainVoxelNum_, 0);
    terrainVoxelUpdateTime_.resize(terrainVoxelNum_, 0.0);

    planarVoxelElev_.resize(planarVoxelNum_, 0.0f);
    planarVoxelEdge_.resize(planarVoxelNum_, 0);
    planarVoxelDyObs_.resize(planarVoxelNum_, 0);
    planarVoxelOutOfFov_.resize(planarVoxelNum_, 0);
    planarVoxelConn_.resize(planarVoxelNum_, 0);
    planarPointElev_.resize(planarVoxelNum_);
  }

  /// Update vehicle pose (call before process)
  void updateVehicle(double x, double y, double z,
                     double roll, double pitch, double yaw) {
    vx_ = x; vy_ = y; vz_ = z;
    vroll_ = roll; vpitch_ = pitch; vyaw_ = yaw;

    // Precompute rotation matrix (ZYX euler)
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);
    R_[0][0] = cy*cp; R_[0][1] = cy*sp*sr - sy*cr; R_[0][2] = cy*sp*cr + sy*sr;
    R_[1][0] = sy*cp; R_[1][1] = sy*sp*sr + cy*cr; R_[1][2] = sy*sp*cr - cy*sr;
    R_[2][0] = -sp;   R_[2][1] = cp*sr;             R_[2][2] = cp*cr;

    if (noDataInited_ == 0) {
      vrecx_ = vx_; vrecy_ = vy_;
      noDataInited_ = 1;
    }
    if (noDataInited_ == 1) {
      double d = std::hypot(vx_ - vrecx_, vy_ - vrecy_);
      if (d >= p_.noDecayDis) noDataInited_ = 2;
    }
  }

  /// Process a point cloud. Points are Nx4 float (x, y, z, intensity).
  /// timestamp: monotonic time in seconds.
  /// Returns terrain result with obstacle cloud + elevation map.
  TerrainResult process(const float* cloud_xyzi, int n_points, double timestamp) {
    if (!systemInited_) {
      systemInitTime_ = timestamp;
      systemInited_ = true;
    }
    double relTime = timestamp - systemInitTime_;

    // 1. Crop points by height + distance
    cropped_.clear();
    float fvx = static_cast<float>(vx_), fvy = static_cast<float>(vy_), fvz = static_cast<float>(vz_);
    float maxDis = static_cast<float>(p_.terrainVoxelSize * (p_.terrainVoxelHalfWidth + 1));
    float maxDisSq = maxDis * maxDis;
    float fRelTime = static_cast<float>(relTime);
    float minRelZ = static_cast<float>(p_.minRelZ);
    float maxRelZ = static_cast<float>(p_.maxRelZ);
    float disRatioZ = static_cast<float>(p_.disRatioZ);

    // Quick reject: if disRatioZ==0, avoid sqrt entirely
    if (disRatioZ == 0.0f) {
      for (int i = 0; i < n_points; i++) {
        float px = cloud_xyzi[i*4 + 0];
        float py = cloud_xyzi[i*4 + 1];
        float pz = cloud_xyzi[i*4 + 2];
        float rx = px - fvx, ry = py - fvy;
        if (rx * rx + ry * ry >= maxDisSq) continue;
        float relz = pz - fvz;
        if (relz > minRelZ && relz < maxRelZ) {
          cropped_.push_back({px, py, pz, fRelTime});
        }
      }
    } else {
      // Use fast inverse-sqrt approximation to avoid per-point sqrt
      for (int i = 0; i < n_points; i++) {
        float px = cloud_xyzi[i*4 + 0];
        float py = cloud_xyzi[i*4 + 1];
        float pz = cloud_xyzi[i*4 + 2];
        float rx = px - fvx, ry = py - fvy;
        float disSq = rx * rx + ry * ry;
        if (disSq >= maxDisSq) continue;
        float dis = std::sqrt(disSq);
        float relz = pz - fvz;
        float margin = disRatioZ * dis;
        if (relz > minRelZ - margin && relz < maxRelZ + margin) {
          cropped_.push_back({px, py, pz, fRelTime});
        }
      }
    }

    // 2. Rolling voxel grid shift
    shiftGrid();

    // 3. Stack cropped points into voxels
    for (auto& pt : cropped_) {
      int ix = (int)((pt.x - vx_ + p_.terrainVoxelSize / 2) / p_.terrainVoxelSize)
               + p_.terrainVoxelHalfWidth;
      int iy = (int)((pt.y - vy_ + p_.terrainVoxelSize / 2) / p_.terrainVoxelSize)
               + p_.terrainVoxelHalfWidth;
      if (pt.x - vx_ + p_.terrainVoxelSize / 2 < 0) ix--;
      if (pt.y - vy_ + p_.terrainVoxelSize / 2 < 0) iy--;
      if (ix >= 0 && ix < terrainVoxelWidth_ && iy >= 0 && iy < terrainVoxelWidth_) {
        int idx = terrainVoxelWidth_ * ix + iy;
        terrainVoxelCloud_[idx].push_back(pt);
        terrainVoxelUpdateNum_[idx]++;
      }
    }

    // 4. Filter voxels (downsample + decay)
    filterVoxels(relTime);

    // 5. Merge nearby voxels
    merged_.clear();
    int hw = p_.terrainVoxelHalfWidth;
    for (int ix = hw - 5; ix <= hw + 5; ix++) {
      for (int iy = hw - 5; iy <= hw + 5; iy++) {
        if (ix >= 0 && ix < terrainVoxelWidth_ && iy >= 0 && iy < terrainVoxelWidth_) {
          auto& vc = terrainVoxelCloud_[terrainVoxelWidth_ * ix + iy];
          merged_.insert(merged_.end(), vc.begin(), vc.end());
        }
      }
    }

    // 6. Ground estimation
    estimateGround();

    // 7. Generate terrain result
    return generateResult(relTime);
  }

  /// Clear all accumulated data
  void clear() {
    for (auto& vc : terrainVoxelCloud_) vc.clear();
    std::fill(terrainVoxelUpdateNum_.begin(), terrainVoxelUpdateNum_.end(), 0);
    std::fill(terrainVoxelUpdateTime_.begin(), terrainVoxelUpdateTime_.end(), 0.0);
    noDataInited_ = 0;
    systemInited_ = false;
    terrainVoxelShiftX_ = 0;
    terrainVoxelShiftY_ = 0;
  }

  const TerrainParams& params() const { return p_; }

  std::size_t storedPointCount() const {
    std::size_t count = 0;
    for (const auto& voxel : terrainVoxelCloud_) {
      count += voxel.size();
    }
    return count;
  }

private:
  struct Point4 { float x, y, z, t; };

  struct ScanVoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const ScanVoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct ScanVoxelHash {
    std::size_t operator()(const ScanVoxelKey& key) const {
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

  TerrainParams p_;

  // Grid dimensions
  int terrainVoxelWidth_ = 0;
  int terrainVoxelNum_ = 0;
  int planarVoxelWidth_ = 0;
  int planarVoxelNum_ = 0;

  // Rolling voxel grid
  std::vector<std::vector<Point4>> terrainVoxelCloud_;
  std::vector<int> terrainVoxelUpdateNum_;
  std::vector<double> terrainVoxelUpdateTime_;
  int terrainVoxelShiftX_ = 0;
  int terrainVoxelShiftY_ = 0;

  // Planar grid for ground estimation
  std::vector<float> planarVoxelElev_;
  std::vector<int> planarVoxelEdge_;
  std::vector<int> planarVoxelDyObs_;
  std::vector<int> planarVoxelOutOfFov_;
  std::vector<std::int8_t> planarVoxelConn_;
  std::vector<std::vector<float>> planarPointElev_;

  // Vehicle state
  double vx_ = 0, vy_ = 0, vz_ = 0;
  double vroll_ = 0, vpitch_ = 0, vyaw_ = 0;
  double R_[3][3] = {};
  double vrecx_ = 0, vrecy_ = 0;
  int noDataInited_ = 0;

  // System time
  bool systemInited_ = false;
  double systemInitTime_ = 0;

  // Scratch buffers
  std::vector<Point4> cropped_;
  std::vector<Point4> merged_;

  void shiftGrid() {
    float cenX = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftX_);
    float cenY = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftY_);
    int w = terrainVoxelWidth_;

    while (vx_ - cenX < -p_.terrainVoxelSize) {
      for (int iy = 0; iy < w; iy++) {
        auto tmp = std::move(terrainVoxelCloud_[w*(w-1)+iy]);
        for (int ix = w-1; ix >= 1; ix--)
          terrainVoxelCloud_[w*ix+iy] = std::move(terrainVoxelCloud_[w*(ix-1)+iy]);
        tmp.clear();
        terrainVoxelCloud_[iy] = std::move(tmp);
      }
      terrainVoxelShiftX_--;
      cenX = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftX_);
    }
    while (vx_ - cenX > p_.terrainVoxelSize) {
      for (int iy = 0; iy < w; iy++) {
        auto tmp = std::move(terrainVoxelCloud_[iy]);
        for (int ix = 0; ix < w-1; ix++)
          terrainVoxelCloud_[w*ix+iy] = std::move(terrainVoxelCloud_[w*(ix+1)+iy]);
        tmp.clear();
        terrainVoxelCloud_[w*(w-1)+iy] = std::move(tmp);
      }
      terrainVoxelShiftX_++;
      cenX = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftX_);
    }
    while (vy_ - cenY < -p_.terrainVoxelSize) {
      for (int ix = 0; ix < w; ix++) {
        auto tmp = std::move(terrainVoxelCloud_[w*ix+(w-1)]);
        for (int iy = w-1; iy >= 1; iy--)
          terrainVoxelCloud_[w*ix+iy] = std::move(terrainVoxelCloud_[w*ix+(iy-1)]);
        tmp.clear();
        terrainVoxelCloud_[w*ix] = std::move(tmp);
      }
      terrainVoxelShiftY_--;
      cenY = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftY_);
    }
    while (vy_ - cenY > p_.terrainVoxelSize) {
      for (int ix = 0; ix < w; ix++) {
        auto tmp = std::move(terrainVoxelCloud_[w*ix]);
        for (int iy = 0; iy < w-1; iy++)
          terrainVoxelCloud_[w*ix+iy] = std::move(terrainVoxelCloud_[w*ix+(iy+1)]);
        tmp.clear();
        terrainVoxelCloud_[w*ix+(w-1)] = std::move(tmp);
      }
      terrainVoxelShiftY_++;
      cenY = static_cast<float>(p_.terrainVoxelSize * terrainVoxelShiftY_);
    }
  }

  void filterVoxels(double relTime) {
    // Hoist constants outside the loop
    float fvx = static_cast<float>(vx_), fvy = static_cast<float>(vy_), fvz = static_cast<float>(vz_);
    float noDecayDisSq = static_cast<float>(p_.noDecayDis * p_.noDecayDis);
    float decayTime = static_cast<float>(p_.decayTime);
    float fRelTime = static_cast<float>(relTime);
    float minRelZ = static_cast<float>(p_.minRelZ);
    float maxRelZ = static_cast<float>(p_.maxRelZ);
    float disRatioZ = static_cast<float>(p_.disRatioZ);
    int pointThre = p_.voxelPointUpdateThre;
    double timeThre = p_.voxelTimeUpdateThre;

    // Each voxel is independent -- parallel-safe (no shared writes)
    #pragma omp parallel for schedule(dynamic, 16) if(terrainVoxelNum_ >= 100)
    for (int idx = 0; idx < terrainVoxelNum_; idx++) {
      auto& vc = terrainVoxelCloud_[idx];
      const bool over_limit =
          p_.maxPointsPerVoxel > 0 && vc.size() > p_.maxPointsPerVoxel;
      if (!over_limit && terrainVoxelUpdateNum_[idx] < pointThre &&
          relTime - terrainVoxelUpdateTime_[idx] < timeThre)
        continue;

      std::vector<Point4> filtered;
      filtered.reserve(vc.size());
      for (auto& pt : vc) {
        float dx = pt.x - fvx, dy = pt.y - fvy;
        float disSq = dx * dx + dy * dy;
        float dis = std::sqrt(disSq);
        float relz = pt.z - fvz;
        if (relz > minRelZ - disRatioZ * dis &&
            relz < maxRelZ + disRatioZ * dis &&
            (fRelTime - pt.t < decayTime || disSq < noDecayDisSq)) {
          filtered.push_back(pt);
        }
      }
      compactPoints(filtered);
      vc = std::move(filtered);
      terrainVoxelUpdateNum_[idx] = 0;
      terrainVoxelUpdateTime_[idx] = relTime;
    }
    enforceGlobalPointLimit();
  }

  void compactPoints(std::vector<Point4>& points) const {
    compactPointsTo(points, p_.maxPointsPerVoxel, p_.maxPointsPerVoxel > 0);
  }

  void compactPointsTo(
      std::vector<Point4>& points,
      std::size_t limit,
      bool bounded) const {
    if (points.empty()) {
      return;
    }
    if (bounded && limit == 0) {
      points.clear();
      return;
    }
    double leaf = std::max(0.01, p_.scanVoxelSize);
    std::unordered_map<ScanVoxelKey, Point4, ScanVoxelHash> compact;
    std::size_t previous_size = std::numeric_limits<std::size_t>::max();
    for (;;) {
      compact.clear();
      compact.reserve(
          bounded ? std::min(points.size(), limit * 2) : points.size());
      const double inv = 1.0 / leaf;
      for (const auto& point : points) {
        const ScanVoxelKey key{
            static_cast<int>(std::floor(static_cast<double>(point.x) * inv)),
            static_cast<int>(std::floor(static_cast<double>(point.y) * inv)),
            static_cast<int>(std::floor(static_cast<double>(point.z) * inv)),
        };
        auto [it, inserted] = compact.emplace(key, point);
        if (!inserted) {
          const bool higher = point.z > it->second.z + 1e-4f;
          const bool same_height = std::abs(point.z - it->second.z) <= 1e-4f;
          if (higher || (same_height && point.t > it->second.t)) {
            it->second = point;
          }
        }
      }
      if (!bounded || compact.size() <= limit) {
        break;
      }
      const double ratio = std::cbrt(
          static_cast<double>(compact.size()) / static_cast<double>(limit));
      leaf *= compact.size() >= previous_size ? 2.0 : std::max(1.15, ratio);
      previous_size = compact.size();
    }

    points.clear();
    points.reserve(compact.size());
    for (const auto& entry : compact) {
      points.push_back(entry.second);
    }
  }

  void enforceGlobalPointLimit() {
    const std::size_t limit = p_.maxStoredPoints;
    if (limit == 0) {
      return;
    }
    const std::size_t total = storedPointCount();
    if (total <= limit) {
      return;
    }

    std::vector<int> active;
    active.reserve(terrainVoxelCloud_.size());
    for (int idx = 0; idx < terrainVoxelNum_; ++idx) {
      if (!terrainVoxelCloud_[idx].empty()) {
        active.push_back(idx);
      }
    }
    std::sort(active.begin(), active.end(), [&](int a, int b) {
      const int ax = a / terrainVoxelWidth_ - p_.terrainVoxelHalfWidth;
      const int ay = a % terrainVoxelWidth_ - p_.terrainVoxelHalfWidth;
      const int bx = b / terrainVoxelWidth_ - p_.terrainVoxelHalfWidth;
      const int by = b % terrainVoxelWidth_ - p_.terrainVoxelHalfWidth;
      const int ad = ax * ax + ay * ay;
      const int bd = bx * bx + by * by;
      return ad == bd ? a < b : ad < bd;
    });

    std::vector<std::size_t> targets(terrainVoxelCloud_.size(), 0);
    if (limit < active.size()) {
      for (std::size_t i = 0; i < limit; ++i) {
        targets[active[i]] = 1;
      }
    } else {
      std::size_t remaining = limit - active.size();
      std::size_t extra_total = total - active.size();
      for (const int idx : active) {
        targets[idx] = 1;
        if (extra_total == 0) {
          continue;
        }
        const std::size_t capacity = terrainVoxelCloud_[idx].size() - 1;
        const std::size_t extra = capacity * remaining / extra_total;
        targets[idx] += std::min(capacity, extra);
      }
      std::size_t assigned = 0;
      for (const int idx : active) {
        assigned += targets[idx];
      }
      remaining = limit - std::min(limit, assigned);
      for (const int idx : active) {
        if (remaining == 0) {
          break;
        }
        if (targets[idx] < terrainVoxelCloud_[idx].size()) {
          ++targets[idx];
          --remaining;
        }
      }
    }

    for (const int idx : active) {
      compactPointsTo(terrainVoxelCloud_[idx], targets[idx], true);
    }
  }

  void estimateGround() {
    int pw = planarVoxelWidth_;
    for (int i = 0; i < planarVoxelNum_; i++) {
      planarVoxelElev_[i] = 0;
      planarVoxelEdge_[i] = 0;
      planarVoxelDyObs_[i] = 0;
      planarVoxelOutOfFov_[i] = 0;
      planarVoxelConn_[i] = 0;
      planarPointElev_[i].clear();
    }

    for (auto& pt : merged_) {
      float relz = pt.z - (float)vz_;
      if (relz <= p_.minRelZ || relz >= p_.maxRelZ) continue;

      int ix = (int)((pt.x - vx_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      int iy = (int)((pt.y - vy_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      if (pt.x - vx_ + p_.planarVoxelSize / 2 < 0) ix--;
      if (pt.y - vy_ + p_.planarVoxelSize / 2 < 0) iy--;

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          int nx = ix + dx, ny = iy + dy;
          if (nx >= 0 && nx < pw && ny >= 0 && ny < pw) {
            planarPointElev_[pw * nx + ny].push_back(pt.z);
          }
        }
      }

      if (p_.clearDyObs && ix >= 0 && ix < pw && iy >= 0 && iy < pw) {
        accumulateDynamicObstacleEvidence(pt, pw * ix + iy);
      }
    }

    if (p_.clearDyObs) {
      for (const auto& pt : cropped_) {
        int ix = (int)((pt.x - vx_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
        int iy = (int)((pt.y - vy_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
        if (pt.x - vx_ + p_.planarVoxelSize / 2 < 0) ix--;
        if (pt.y - vy_ + p_.planarVoxelSize / 2 < 0) iy--;
        if (ix < 0 || ix >= pw || iy < 0 || iy >= pw) continue;

        const float dx = pt.x - static_cast<float>(vx_);
        const float dy = pt.y - static_cast<float>(vy_);
        const float dz = pt.z - static_cast<float>(vz_);
        const float dis = std::sqrt(dx * dx + dy * dy);
        const float angle = std::atan2(
            dz - static_cast<float>(p_.minDyObsRelZ),
            std::max(dis, 1e-6f)) * 180.0f / static_cast<float>(M_PI);
        if (angle > static_cast<float>(p_.minDyObsAngle)) {
          planarVoxelDyObs_[pw * ix + iy] = 0;
        }
      }
    }

    if (p_.useSorting) {
      float quantileZ = static_cast<float>(p_.quantileZ);
      bool limitLift = p_.limitGroundLift;
      float maxLift = static_cast<float>(p_.maxGroundLift);
      // 2601 voxels, each nth_element is independent --embarrassingly parallel
      #pragma omp parallel for schedule(dynamic, 64) if(planarVoxelNum_ >= 256)
      for (int i = 0; i < planarVoxelNum_; i++) {
        auto& elev = planarPointElev_[i];
        if (elev.empty()) continue;
        int qid = std::clamp(static_cast<int>(quantileZ * elev.size()),
                             0, static_cast<int>(elev.size()) - 1);
        std::nth_element(elev.begin(), elev.begin() + qid, elev.end());
        if (limitLift) {
          float minVal = *std::min_element(elev.begin(), elev.begin() + qid + 1);
          if (elev[qid] > minVal + maxLift)
            planarVoxelElev_[i] = minVal + maxLift;
          else
            planarVoxelElev_[i] = elev[qid];
        } else {
          planarVoxelElev_[i] = elev[qid];
        }
      }
    } else {
      #pragma omp parallel for schedule(dynamic, 64) if(planarVoxelNum_ >= 256)
      for (int i = 0; i < planarVoxelNum_; i++) {
        auto& elev = planarPointElev_[i];
        if (!elev.empty())
          planarVoxelElev_[i] = *std::min_element(elev.begin(), elev.end());
      }
    }
    computeTerrainConnectivity();
  }

  void computeTerrainConnectivity() {
    std::fill(planarVoxelConn_.begin(), planarVoxelConn_.end(), 0);
    if (!p_.checkTerrainConnectivity) {
      for (int index = 0; index < planarVoxelNum_; ++index) {
        if (!planarPointElev_[index].empty()) {
          planarVoxelConn_[index] = 1;
        }
      }
      return;
    }

    const int width = planarVoxelWidth_;
    const int half = p_.planarVoxelHalfWidth;
    const int center = width * half + half;
    const float expected_ground =
        static_cast<float>(vz_ + p_.terrainUnderVehicle);
    const int seed_radius = std::max(0, p_.groundSeedSearchRadiusCells);
    int seed = -1;
    float seed_error = std::numeric_limits<float>::infinity();
    for (int dx = -seed_radius; dx <= seed_radius; ++dx) {
      for (int dy = -seed_radius; dy <= seed_radius; ++dy) {
        const int x = half + dx;
        const int y = half + dy;
        if (x < 0 || x >= width || y < 0 || y >= width) {
          continue;
        }
        const int index = width * x + y;
        if (planarPointElev_[index].empty()) {
          continue;
        }
        const float error = std::fabs(planarVoxelElev_[index] - expected_ground);
        if (error < seed_error) {
          seed = index;
          seed_error = error;
        }
      }
    }
    if (seed >= 0 && seed_error > static_cast<float>(p_.maxGroundSeedError)) {
      seed = -1;
    }
    if (seed < 0) {
      seed = center;
      planarVoxelElev_[seed] = expected_ground;
    }

    const int radius = std::max(1, p_.terrainConnectivityRadiusCells);
    const int radius_sq = radius * radius;
    const float max_step = static_cast<float>(
        std::max(0.0, p_.terrainConnectionHeight));
    std::queue<int> pending;
    planarVoxelConn_[seed] = 1;
    pending.push(seed);
    while (!pending.empty()) {
      const int current = pending.front();
      pending.pop();
      const int current_x = current / width;
      const int current_y = current % width;
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if ((dx == 0 && dy == 0) || dx * dx + dy * dy > radius_sq) {
            continue;
          }
          const int x = current_x + dx;
          const int y = current_y + dy;
          if (x < 0 || x >= width || y < 0 || y >= width) {
            continue;
          }
          const int next = width * x + y;
          if (planarVoxelConn_[next] != 0 || planarPointElev_[next].empty()) {
            continue;
          }
          if (std::fabs(planarVoxelElev_[current] - planarVoxelElev_[next]) <=
              max_step) {
            planarVoxelConn_[next] = 1;
            pending.push(next);
          }
        }
      }
    }

    const float ceiling_step = static_cast<float>(
        std::max(p_.terrainConnectionHeight, p_.ceilingFilteringHeight));
    for (int index = 0; index < planarVoxelNum_; ++index) {
      if (planarVoxelConn_[index] != 0 || planarPointElev_[index].empty()) {
        continue;
      }
      const int cell_x = index / width;
      const int cell_y = index % width;
      float nearest_connected_delta = std::numeric_limits<float>::infinity();
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (dx * dx + dy * dy > radius_sq) {
            continue;
          }
          const int x = cell_x + dx;
          const int y = cell_y + dy;
          if (x < 0 || x >= width || y < 0 || y >= width) {
            continue;
          }
          const int neighbor = width * x + y;
          if (planarVoxelConn_[neighbor] == 1) {
            nearest_connected_delta = std::min(
                nearest_connected_delta,
                std::fabs(planarVoxelElev_[index] - planarVoxelElev_[neighbor]));
          }
        }
      }
      if (nearest_connected_delta >= ceiling_step &&
          std::isfinite(nearest_connected_delta)) {
        planarVoxelConn_[index] = -1;
      }
    }
  }

  TerrainResult generateResult(double relTime) {
    (void)relTime;
    TerrainResult res;
    int pw = planarVoxelWidth_;
    res.map_width = pw;
    res.map_origin_x = (float)vx_;
    res.map_origin_y = (float)vy_;
    res.map_resolution = (float)p_.planarVoxelSize;
    res.elevation_map.assign(planarVoxelElev_.begin(), planarVoxelElev_.end());
    res.connectivity_map.assign(planarVoxelConn_.begin(), planarVoxelConn_.end());
    if (p_.checkTerrainConnectivity) {
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (int index = 0; index < planarVoxelNum_; ++index) {
        if (planarVoxelConn_[index] == 1) {
          if (!planarPointElev_[index].empty()) {
            ++res.connected_cells;
          }
        } else {
          res.elevation_map[index] = nan;
          if (planarVoxelConn_[index] < 0) {
            ++res.ceiling_cells;
          }
        }
      }
    }

    for (auto& pt : merged_) {
      float relz = pt.z - (float)vz_;
      if (relz <= p_.minRelZ || relz >= p_.maxRelZ) continue;

      int ix = (int)((pt.x - vx_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      int iy = (int)((pt.y - vy_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      if (pt.x - vx_ + p_.planarVoxelSize / 2 < 0) ix--;
      if (pt.y - vy_ + p_.planarVoxelSize / 2 < 0) iy--;

      if (ix >= 0 && ix < pw && iy >= 0 && iy < pw) {
        int vidx = pw * ix + iy;
        if (p_.checkTerrainConnectivity && planarVoxelConn_[vidx] != 1) {
          continue;
        }
        if (p_.clearDyObs && planarVoxelDyObs_[vidx] >= p_.minDyObsPointNum) {
          continue;
        }
        float disZ = pt.z - planarVoxelElev_[vidx];
        if (p_.considerDrop) disZ = std::fabs(disZ);

        int nPts = (int)planarPointElev_[vidx].size();
        if (disZ >= 0 && disZ < p_.vehicleHeight && nPts >= p_.minBlockPointNum) {
          res.terrain_points.push_back(pt.x);
          res.terrain_points.push_back(pt.y);
          res.terrain_points.push_back(pt.z);
          res.terrain_points.push_back(disZ);  // height above ground
          res.n_points++;
        }
      }
    }
    appendNoDataObstacles(res);
    return res;
  }

  void accumulateDynamicObstacleEvidence(const Point4& pt, int voxel_idx) {
    const float dx = pt.x - static_cast<float>(vx_);
    const float dy = pt.y - static_cast<float>(vy_);
    const float dz = pt.z - static_cast<float>(vz_);
    const float dis = std::sqrt(dx * dx + dy * dy);
    if (dis > static_cast<float>(p_.minDyObsDis)) {
      const float angle = std::atan2(
          dz - static_cast<float>(p_.minDyObsRelZ),
          std::max(dis, 1e-6f)) * 180.0f / static_cast<float>(M_PI);
      if (angle <= static_cast<float>(p_.minDyObsAngle)) {
        return;
      }
      const float cy = std::cos(static_cast<float>(vyaw_));
      const float sy = std::sin(static_cast<float>(vyaw_));
      const float cp = std::cos(static_cast<float>(vpitch_));
      const float sp = std::sin(static_cast<float>(vpitch_));
      const float cr = std::cos(static_cast<float>(vroll_));
      const float sr = std::sin(static_cast<float>(vroll_));
      const float x2 = dx * cy + dy * sy;
      const float y2 = -dx * sy + dy * cy;
      const float z2 = dz;
      const float x3 = x2 * cp - z2 * sp;
      const float y3 = y2;
      const float z3 = x2 * sp + z2 * cp;
      const float x4 = x3;
      const float y4 = y3 * cr + z3 * sr;
      const float z4 = -y3 * sr + z3 * cr;
      const float dis4 = std::sqrt(x4 * x4 + y4 * y4);
      const float angle4 =
          std::atan2(z4, std::max(dis4, 1e-6f)) * 180.0f / static_cast<float>(M_PI);
      if ((angle4 > static_cast<float>(p_.minDyObsVFOV) &&
           angle4 < static_cast<float>(p_.maxDyObsVFOV)) ||
          std::fabs(z4) < static_cast<float>(p_.absDyObsRelZThre)) {
        planarVoxelDyObs_[voxel_idx]++;
      }
    } else {
      planarVoxelDyObs_[voxel_idx] += p_.minDyObsPointNum;
    }
  }

  void appendNoDataObstacles(TerrainResult& res) {
    if (!p_.noDataObstacle || noDataInited_ != 2) {
      return;
    }
    const int pw = planarVoxelWidth_;
    for (int i = 0; i < planarVoxelNum_; ++i) {
      if (static_cast<int>(planarPointElev_[i].size()) < p_.minBlockPointNum) {
        planarVoxelEdge_[i] = 1;
      }
    }
    for (int skip = 0; skip < p_.noDataBlockSkipNum; ++skip) {
      for (int i = 0; i < planarVoxelNum_; ++i) {
        if (planarVoxelEdge_[i] < 1) {
          continue;
        }
        const int ix = i / pw;
        const int iy = i % pw;
        bool edge_voxel = false;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            const int nx = ix + dx;
            const int ny = iy + dy;
            if (nx >= 0 && nx < pw && ny >= 0 && ny < pw &&
                planarVoxelEdge_[pw * nx + ny] < planarVoxelEdge_[i]) {
              edge_voxel = true;
            }
          }
        }
        if (!edge_voxel) {
          planarVoxelEdge_[i]++;
        }
      }
    }
    for (int i = 0; i < planarVoxelNum_; ++i) {
      if (planarVoxelEdge_[i] <= p_.noDataBlockSkipNum) {
        continue;
      }
      const int ix = i / pw;
      const int iy = i % pw;
      const float cx = static_cast<float>(
          p_.planarVoxelSize * (ix - p_.planarVoxelHalfWidth) + vx_);
      const float cy = static_cast<float>(
          p_.planarVoxelSize * (iy - p_.planarVoxelHalfWidth) + vy_);
      const float z = static_cast<float>(vz_);
      const float h = static_cast<float>(p_.vehicleHeight);
      const float d = static_cast<float>(p_.planarVoxelSize * 0.25);
      appendTerrainPoint(res, cx - d, cy - d, z, h);
      appendTerrainPoint(res, cx + d, cy - d, z, h);
      appendTerrainPoint(res, cx + d, cy + d, z, h);
      appendTerrainPoint(res, cx - d, cy + d, z, h);
    }
  }

  static void appendTerrainPoint(
      TerrainResult& res, float x, float y, float z, float height) {
    res.terrain_points.push_back(x);
    res.terrain_points.push_back(y);
    res.terrain_points.push_back(z);
    res.terrain_points.push_back(height);
    res.n_points++;
  }
};

}  // namespace nav_kernel
