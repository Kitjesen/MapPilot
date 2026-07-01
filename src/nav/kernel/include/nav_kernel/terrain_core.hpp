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

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>

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

  // Dynamic obstacle
  bool clearDyObs = false;
  double minDyObsDis = 0.3;
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
  int map_width = 0;

  // Vehicle-centric origin of elevation map
  float map_origin_x = 0;
  float map_origin_y = 0;
  float map_resolution = 0;
};

// ── Core Algorithm (stateful �?maintains rolling voxel grid) ──

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
      if (d > p_.minDyObsDis) noDataInited_ = 2;
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

private:
  struct Point4 { float x, y, z, t; };

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

    // Each voxel is independent �?parallel-safe (no shared writes)
    #pragma omp parallel for schedule(dynamic, 16) if(terrainVoxelNum_ >= 100)
    for (int idx = 0; idx < terrainVoxelNum_; idx++) {
      if (terrainVoxelUpdateNum_[idx] < pointThre &&
          relTime - terrainVoxelUpdateTime_[idx] < timeThre)
        continue;

      auto& vc = terrainVoxelCloud_[idx];
      std::vector<Point4> filtered;
      filtered.reserve(vc.size() / 2);
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
      vc = std::move(filtered);
      terrainVoxelUpdateNum_[idx] = 0;
      terrainVoxelUpdateTime_[idx] = relTime;
    }
  }

  void estimateGround() {
    int pw = planarVoxelWidth_;
    for (int i = 0; i < planarVoxelNum_; i++) {
      planarVoxelElev_[i] = 0;
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
    }

    if (p_.useSorting) {
      float quantileZ = static_cast<float>(p_.quantileZ);
      bool limitLift = p_.limitGroundLift;
      float maxLift = static_cast<float>(p_.maxGroundLift);
      // 2601 voxels, each nth_element is independent �?embarrassingly parallel
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
  }

  TerrainResult generateResult(double relTime) {
    TerrainResult res;
    int pw = planarVoxelWidth_;
    res.map_width = pw;
    res.map_origin_x = (float)vx_;
    res.map_origin_y = (float)vy_;
    res.map_resolution = (float)p_.planarVoxelSize;
    res.elevation_map.assign(planarVoxelElev_.begin(), planarVoxelElev_.end());

    for (auto& pt : merged_) {
      float relz = pt.z - (float)vz_;
      if (relz <= p_.minRelZ || relz >= p_.maxRelZ) continue;

      int ix = (int)((pt.x - vx_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      int iy = (int)((pt.y - vy_ + p_.planarVoxelSize / 2) / p_.planarVoxelSize) + p_.planarVoxelHalfWidth;
      if (pt.x - vx_ + p_.planarVoxelSize / 2 < 0) ix--;
      if (pt.y - vy_ + p_.planarVoxelSize / 2 < 0) iy--;

      if (ix >= 0 && ix < pw && iy >= 0 && iy < pw) {
        int vidx = pw * ix + iy;
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
    return res;
  }
};

}  // namespace nav_kernel
