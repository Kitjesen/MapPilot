/**
 * local_planner/cpp/local_planner.hpp -- LocalPlannerCore, ROS-free CMU local planner.
 *
 * Owns the full per-frame planning loop:
 *   1. Load the precomputed CMU path library.
 *   2. Project obstacle points into path voxels.
 *   3. Score path groups and select the best local path.
 *   4. Degrade path scale/range when blocked.
 *   5. Emit near-field stop and recovery path states.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include "local_planner_scoring.hpp"
#include "nav_kernel/simd_accel.hpp"
#include <cmath>
#include <cstring>
#include <atomic>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace nav_kernel {

// 鈹€鈹€ Parameters 鈹€鈹€

struct LocalPlannerParams {
  double vehicleLength      = 0.6;
  double vehicleWidth       = 0.6;
  double sensorOffsetX      = 0.0;
  double sensorOffsetY      = 0.0;
  bool   twoWayDrive        = true;
  double adjacentRange      = 3.5;
  double obstacleHeightThre = 0.2;
  double groundHeightThre   = 0.1;
  double costHeightThre1    = 0.15;
  double costHeightThre2    = 0.1;
  bool   useCost            = false;
  bool   checkObstacle      = true;
  bool   checkRotObstacle   = false;
  bool   useTerrainAnalysis = true;
  int    pointPerPathThre   = 2;
  double minRelZ            = -0.5;
  double maxRelZ            = 0.25;
  double dirWeight          = 0.02;
  double dirThre            = 90.0;
  bool   dirToVehicle       = false;
  double pathScale          = 1.0;
  double minPathScale       = 0.75;
  double pathScaleStep      = 0.25;
  bool   pathScaleBySpeed   = true;
  double minPathRange       = 1.0;
  double pathRangeStep      = 0.5;
  bool   pathRangeBySpeed   = true;
  bool   pathCropByGoal     = true;
  double maxSpeed           = 1.0;
  double autonomySpeed      = 1.0;
  double slopeWeight        = 0.0;
  double goalClearRange     = 0.5;
  double goalBehindRange    = 0.8;
  double nearFieldStopDis   = 0.5;
  double footprintPadding   = 0.1;
  double freezeAng          = 90.0;
  double freezeTime         = 2.0;
  double omniDirGoalThre    = 1.0;
  int    slowPathNumThre    = 5;
  int    slowGroupNumThre   = 1;
  bool   useTraversabilityCost = true;
  bool   traversabilityNearFieldStop = false;
  double traversabilityHardCost = 90.0;
  double traversabilitySoftCost = 40.0;
  double traversabilityWeight   = 0.01;

  // Recovery
  double recoveryBlockedThre = 2.0;
  double recoveryRotateTime  = 2.5;
  double recoveryBackupTime  = 1.5;
  int    recoveryMaxCycles   = 3;
};

// 鈹€鈹€ Result 鈹€鈹€

struct LocalPlanResult {
  std::vector<Vec3> path;
  int  slowDown    = 0;     // 0-3
  bool pathFound   = false;
  bool nearFieldStop = false;
  int  recoveryState = 0;   // 0=normal, 1=rotating, 2=backing_up
};

// 鈹€鈹€ Core Algorithm 鈹€鈹€

static constexpr int kPathNum  = 343;
static constexpr int kGroupNum = 7;
static constexpr int kRotDirs  = 36;

class LocalPlannerCore {
public:
  explicit LocalPlannerCore(const LocalPlannerParams& params = LocalPlannerParams())
    : p_(params) {}

  /// Load pre-computed path library from directory containing PLY files.
  /// Returns true on success.
  bool loadPaths(const std::string& pathsDir) {
    bool ok = true;
    ok = ok && loadStartPaths(pathsDir + "/startPaths.ply");
    ok = ok && loadPathList(pathsDir + "/pathList.ply");
    ok = ok && loadCorrespondences(pathsDir + "/correspondences.txt");
    pathsLoaded_ = ok;
    return ok;
  }

  bool pathsLoaded() const { return pathsLoaded_; }

  /// Update vehicle pose (odom frame). Call before plan().
  void setVehicle(double x, double y, double z, double yaw) {
    cosYaw_ = std::cos(yaw);
    sinYaw_ = std::sin(yaw);
    vx_ = x - cosYaw_ * p_.sensorOffsetX + sinYaw_ * p_.sensorOffsetY;
    vy_ = y - sinYaw_ * p_.sensorOffsetX - cosYaw_ * p_.sensorOffsetY;
    vz_ = z;
    vyaw_ = yaw;
  }

  /// Update goal position (odom frame). Call before plan().
  void setGoal(double gx, double gy) {
    goalX_ = gx; goalY_ = gy;
  }

  /// Set a 0..100 traversability risk grid in odom/map coordinates.
  void setTraversabilityGrid(const float* grid,
                             int rows,
                             int cols,
                             double resolution,
                             double originX,
                             double originY) {
    traversabilityGrid_.clear();
    traversabilityRows_ = 0;
    traversabilityCols_ = 0;
    traversabilityResolution_ = 0.0;
    if (grid == nullptr || rows <= 0 || cols <= 0 || resolution <= 0.0) return;
    const int n = rows * cols;
    traversabilityGrid_.assign(grid, grid + n);
    traversabilityRows_ = rows;
    traversabilityCols_ = cols;
    traversabilityResolution_ = resolution;
    traversabilityOriginX_ = originX;
    traversabilityOriginY_ = originY;
  }

  void clearTraversabilityGrid() {
    traversabilityGrid_.clear();
    traversabilityRows_ = 0;
    traversabilityCols_ = 0;
    traversabilityResolution_ = 0.0;
  }

  /// Run one planning cycle.
  /// obstacle_pts: Nx4 float array [x,y,z,intensity] in odom frame.
  /// timestamp: monotonic seconds.
  LocalPlanResult plan(const float* obstacle_pts, int n_pts, double timestamp) {
    if (!pathsLoaded_) return {};

    LocalPlanResult result;
    odomTime_ = timestamp;

    // Joy speed for autonomy mode
    double joySpeed = p_.autonomySpeed / p_.maxSpeed;
    joySpeed = std::clamp(joySpeed, 0.0, 1.0);

    // Transform goal to body frame
    double relGoalX = (goalX_ - vx_) * cosYaw_ + (goalY_ - vy_) * sinYaw_;
    double relGoalY = -(goalX_ - vx_) * sinYaw_ + (goalY_ - vy_) * cosYaw_;
    double relGoalDis = std::hypot(relGoalX, relGoalY);
    double joyDir = std::atan2(relGoalY, relGoalX) * 180.0 / M_PI;

    if (!p_.twoWayDrive) {
      // Single-direction drive should not immediately reverse into a close
      // behind-body goal. Two-way drive must keep that target trackable.
      if (std::fabs(joyDir) > p_.freezeAng && relGoalDis < p_.goalBehindRange) {
        relGoalDis = 0; joyDir = 0;
      }
      updateFreezeState(joyDir);
      if (freezeStatus_ == 1) { relGoalDis = 0; joyDir = 0; }
      joyDir = std::clamp(joyDir, -95.0, 95.0);
    } else {
      freezeStatus_ = 0;
    }
    // Near-field stop check covers explicit obstacle points and native
    // traversability risk cells in front of the body.
    result.nearFieldStop =
        checkNearFieldStop(obstacle_pts, n_pts) ||
        (p_.traversabilityNearFieldStop && checkNearFieldTraversability());

    // Transform obstacles to body frame and crop
    buildPlannerCloud(obstacle_pts, n_pts);

    // Path scoring loop with scale degradation
    double pathRange = p_.adjacentRange;
    if (p_.pathRangeBySpeed) pathRange = p_.adjacentRange * joySpeed;
    if (pathRange < p_.minPathRange) pathRange = p_.minPathRange;

    double defPathScale = p_.pathScale;
    double curPathScale = defPathScale;
    if (p_.pathScaleBySpeed) curPathScale = defPathScale * joySpeed;
    if (curPathScale < p_.minPathScale) curPathScale = p_.minPathScale;

    bool pathFound = false;

    while (curPathScale >= p_.minPathScale && pathRange >= p_.minPathRange) {
      int selectedGroupID = scoreAndSelect(
          curPathScale, pathRange, relGoalDis, joyDir, joySpeed, result.slowDown);

      if (selectedGroupID >= 0) {
        buildOutputPath(selectedGroupID, curPathScale, pathRange, relGoalDis, result);
        if (result.pathFound) {
          pathFound = true;
          recoveryState_ = 0;
          blockedStartTime_ = -1.0;
          recoveryCycleCount_ = 0;
        }
        break;
      }
      if (curPathScale >= p_.minPathScale + p_.pathScaleStep) {
        curPathScale -= p_.pathScaleStep;
        pathRange = p_.adjacentRange * curPathScale / defPathScale;
      } else {
        pathRange -= p_.pathRangeStep;
      }
    }

    result.pathFound = pathFound;

    if (!pathFound) {
      buildRecoveryPath(result);
    }

    result.recoveryState = recoveryState_;
    return result;
  }

  LocalPlanResult planFrame(double x,
                            double y,
                            double z,
                            double yaw,
                            double gx,
                            double gy,
                            const float* traversability_grid,
                            int traversability_rows,
                            int traversability_cols,
                            double traversability_resolution,
                            double traversability_origin_x,
                            double traversability_origin_y,
                            const float* obstacle_pts,
                            int n_pts,
                            double timestamp) {
    setVehicle(x, y, z, yaw);
    setGoal(gx, gy);
    if (traversability_grid != nullptr &&
        traversability_rows > 0 &&
        traversability_cols > 0 &&
        traversability_resolution > 0.0) {
      setTraversabilityGrid(
          traversability_grid,
          traversability_rows,
          traversability_cols,
          traversability_resolution,
          traversability_origin_x,
          traversability_origin_y);
    } else {
      clearTraversabilityGrid();
    }
    return plan(obstacle_pts, n_pts, timestamp);
  }

  const LocalPlannerParams& params() const { return p_; }

private:
  LocalPlannerParams p_;
  bool pathsLoaded_ = false;

  // Vehicle state
  double vx_ = 0, vy_ = 0, vz_ = 0, vyaw_ = 0;
  double cosYaw_ = 1.0, sinYaw_ = 0.0;
  double goalX_ = 0, goalY_ = 0;
  double odomTime_ = 0;
  std::vector<float> traversabilityGrid_;
  int traversabilityRows_ = 0;
  int traversabilityCols_ = 0;
  double traversabilityResolution_ = 0.0;
  double traversabilityOriginX_ = 0.0;
  double traversabilityOriginY_ = 0.0;

  // Freeze state
  int freezeStatus_ = 0;
  double freezeStartTime_ = 0;

  // Recovery state
  int    recoveryState_       = 0;
  double blockedStartTime_    = -1.0;
  double recoveryPhaseStart_  = -1.0;
  int    recoveryCycleCount_  = 0;

  // Path library
  struct PathPoint { float x, y, z; };
  std::array<std::vector<PathPoint>, kGroupNum> startPaths_;
  std::array<int, kPathNum> pathList_{};
  std::array<float, kPathNum> endDirPathList_{};

  // Correspondences: voxel_id -> path_ids (CSR format for cache locality)
  int gridVoxelNumX_ = 161;
  int gridVoxelNumY_ = 451;
  int gridVoxelNum_  = gridVoxelNumX_ * gridVoxelNumY_;
  // CSR: corrOffset_[i] .. corrOffset_[i+1] index into corrData_
  std::vector<int> corrOffset_;   // size = gridVoxelNum_ + 1
  std::vector<int> corrData_;     // flat array of all path IDs

  // Per-frame scratch buffers, using SoA layout for cache-friendly scoring.
  struct PlannerCloudSoA {
    std::vector<float> x, y, h;  // body-frame x, y + terrain height
    int size = 0;
    void clear() { x.clear(); y.clear(); h.clear(); size = 0; }
    void reserve(int n) { x.reserve(n); y.reserve(n); h.reserve(n); }
    void push(float bx, float by, float bh) {
      x.push_back(bx); y.push_back(by); h.push_back(bh);
      size++;
    }
  } cloud_;

  // Pre-filtered valid rotation indices (avoids branch in inner loop)
  std::array<int, kRotDirs> validRotDirs_{};
  int nValidRotDirs_ = 0;

  // SIMD scratch buffers (reused across frames, no per-frame alloc)
  std::vector<float> scaledX_, scaledY_, disSqBuf_;
  std::vector<float> rotX_, rotY_;
  std::vector<float> parRotBufs_;  // parallel per-rotation scratch

  // Scoring arrays
  std::vector<int>   clearPathList_;
  std::vector<float> pathPenaltyList_;
  std::vector<double> clearPathPerGroupScore_;
  std::vector<int>   clearPathPerGroupNum_;
  std::vector<float> pathPenaltyPerGroupScore_;

  // File loading

  int readPlyHeader(std::ifstream& f) {
    std::string line;
    int pointNum = 0;
    while (std::getline(f, line)) {
      // Strip trailing \r (Windows CRLF safety)
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line == "end_header") break;
      // Parse "element vertex N"
      if (line.find("element vertex") != std::string::npos) {
        std::istringstream iss(line);
        std::string a, b; int n = 0;
        iss >> a >> b >> n;
        pointNum = n;
      }
    }
    return pointNum;
  }

  bool loadStartPaths(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    int n = readPlyHeader(f);
    for (int i = 0; i < n; i++) {
      float x, y, z; int groupID;
      f >> x >> y >> z >> groupID;
      if (groupID >= 0 && groupID < kGroupNum)
        startPaths_[groupID].push_back({x, y, z});
    }
    return true;
  }

  bool loadPathList(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    int n = readPlyHeader(f);
    if (n != kPathNum) return false;
    for (int i = 0; i < kPathNum; i++) {
      float ex, ey, ez; int pathID, groupID;
      f >> ex >> ey >> ez >> pathID >> groupID;
      if (pathID >= 0 && pathID < kPathNum && groupID >= 0 && groupID < kGroupNum) {
        pathList_[pathID] = groupID;
        endDirPathList_[pathID] = 2.0f * std::atan2(ey, ex) * 180.0f / (float)M_PI;
      }
    }
    return true;
  }

  bool loadCorrespondences(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;

    // First pass: load into temporary vector-of-vectors
    std::vector<std::vector<int>> tmp(gridVoxelNum_);
    for (int i = 0; i < gridVoxelNum_; i++) {
      int gridVoxelID;
      f >> gridVoxelID;
      int pathID;
      while (f >> pathID) {
        if (pathID == -1) break;
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum_ &&
            pathID >= 0 && pathID < kPathNum) {
          tmp[gridVoxelID].push_back(pathID);
        }
      }
    }

    // Convert to CSR (Compressed Sparse Row) in one contiguous array.
    corrOffset_.resize(gridVoxelNum_ + 1);
    corrOffset_[0] = 0;
    int totalEntries = 0;
    for (int i = 0; i < gridVoxelNum_; i++) {
      totalEntries += static_cast<int>(tmp[i].size());
      corrOffset_[i + 1] = totalEntries;
    }
    corrData_.resize(totalEntries);
    for (int i = 0; i < gridVoxelNum_; i++) {
      std::copy(tmp[i].begin(), tmp[i].end(),
                corrData_.begin() + corrOffset_[i]);
    }

    // Allocate scoring arrays
    int total = kRotDirs * kPathNum;
    clearPathList_.resize(total);
    pathPenaltyList_.resize(total);
    clearPathPerGroupScore_.resize(kRotDirs * kGroupNum);
    clearPathPerGroupNum_.resize(kRotDirs * kGroupNum);
    pathPenaltyPerGroupScore_.resize(kRotDirs * kGroupNum);
    return true;
  }

  // 鈹€鈹€ Frame processing 鈹€鈹€

  void updateFreezeState(double joyDir) {
    if (std::fabs(joyDir) > p_.freezeAng && freezeStatus_ == 0) {
      freezeStartTime_ = odomTime_;
      freezeStatus_ = 1;
    } else if (odomTime_ - freezeStartTime_ > p_.freezeTime && freezeStatus_ == 1) {
      freezeStatus_ = 2;
    } else if (std::fabs(joyDir) <= p_.freezeAng && freezeStatus_ == 2) {
      freezeStatus_ = 0;
    }
  }

  bool checkNearFieldStop(const float* pts, int n) {
    if (!p_.checkObstacle) return false;
    const double halfW = footprintHalfWidth();
    const double frontEdge = footprintFrontEdge();
    for (int i = 0; i < n; i++) {
      float px = pts[i*4], py = pts[i*4+1], pz = pts[i*4+2], h = pts[i*4+3];
      float dx = px - (float)vx_, dy = py - (float)vy_;
      // To body frame
      float bx = dx * (float)cosYaw_ + dy * (float)sinYaw_;
      float by = -dx * (float)sinYaw_ + dy * (float)cosYaw_;
      if (insideFootprint(bx, by)) continue;
      if (bx > (float)frontEdge && bx < (float)(frontEdge + p_.nearFieldStopDis) &&
          std::fabs(by) < halfW &&
          (h > (float)p_.obstacleHeightThre || !p_.useTerrainAnalysis)) {
        return true;
      }
    }
    return false;
  }

  float traversabilityRiskAtWorld(double wx, double wy) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_.empty() ||
        traversabilityRows_ <= 0 || traversabilityCols_ <= 0 ||
        traversabilityResolution_ <= 0.0) {
      return 0.0f;
    }
    int col = static_cast<int>((wx - traversabilityOriginX_) / traversabilityResolution_);
    int row = static_cast<int>((wy - traversabilityOriginY_) / traversabilityResolution_);
    if (row < 0 || row >= traversabilityRows_ || col < 0 || col >= traversabilityCols_) {
      return 0.0f;
    }
    float risk = traversabilityGrid_[row * traversabilityCols_ + col];
    if (!std::isfinite(risk)) return 0.0f;
    return std::clamp(risk, 0.0f, 100.0f);
  }

  float traversabilityRiskAtBody(double bx, double by) const {
    double wx = vx_ + bx * cosYaw_ - by * sinYaw_;
    double wy = vy_ + bx * sinYaw_ + by * cosYaw_;
    return traversabilityRiskAtWorld(wx, wy);
  }

  bool checkNearFieldTraversability() const {
    if (!p_.checkObstacle || !p_.useTraversabilityCost ||
        traversabilityGrid_.empty() || traversabilityResolution_ <= 0.0) {
      return false;
    }
    const double halfW = footprintHalfWidth();
    const double frontEdge = footprintFrontEdge();
    double step = std::clamp(
        traversabilityResolution_, 0.05, std::max(0.05, p_.nearFieldStopDis));
    for (double bx = frontEdge + step * 0.5;
         bx < frontEdge + p_.nearFieldStopDis;
         bx += step) {
      if (traversabilityRiskAtBody(bx, 0.0) >=
          static_cast<float>(p_.traversabilityHardCost)) {
        return true;
      }
      for (double by = step; by <= halfW; by += step) {
        if (traversabilityRiskAtBody(bx, by) >=
                static_cast<float>(p_.traversabilityHardCost) ||
            traversabilityRiskAtBody(bx, -by) >=
                static_cast<float>(p_.traversabilityHardCost)) {
          return true;
        }
      }
    }
    return false;
  }

  double footprintHalfLength() const {
    return std::max(0.0, p_.vehicleLength * 0.5) + std::max(0.0, p_.footprintPadding);
  }

  double footprintHalfWidth() const {
    return std::max(0.0, p_.vehicleWidth * 0.5) + std::max(0.0, p_.footprintPadding);
  }

  double footprintFrontEdge() const {
    return footprintHalfLength();
  }

  bool insideFootprint(float bx, float by) const {
    return std::fabs(bx) <= static_cast<float>(footprintHalfLength()) &&
           std::fabs(by) <= static_cast<float>(footprintHalfWidth());
  }

  void buildPlannerCloud(const float* pts, int n) {
    cloud_.clear();
    cloud_.reserve(n);
    float adjRangeSq = static_cast<float>(p_.adjacentRange * p_.adjacentRange);
    float cosY = static_cast<float>(cosYaw_), sinY = static_cast<float>(sinYaw_);
    float fx = static_cast<float>(vx_), fy = static_cast<float>(vy_), fz = static_cast<float>(vz_);
    float minZ = static_cast<float>(p_.minRelZ), maxZ = static_cast<float>(p_.maxRelZ);
    bool useTerrain = p_.useTerrainAnalysis;

    // Two-pass: 1) gather dx/dy into temp SoA, 2) SIMD batch disSq + rotate
    // For small clouds, scalar is fine. For large clouds, avoid AoS stride-4 penalty.
    if (n >= 256 && useTerrain) {
      // Phase 1: gather from AoS �?SoA, apply range filter
      bpcDx_.resize(n); bpcDy_.resize(n); bpcH_.resize(n);
      int kept = 0;
      for (int i = 0; i < n; i++) {
        float dx = pts[i*4] - fx, dy = pts[i*4+1] - fy;
        if (dx*dx + dy*dy >= adjRangeSq) continue;
        float bx = dx * cosY + dy * sinY;
        float by = -dx * sinY + dy * cosY;
        if (insideFootprint(bx, by)) continue;
        bpcDx_[kept] = dx;
        bpcDy_[kept] = dy;
        bpcH_[kept] = pts[i*4+3];
        kept++;
      }
      // Phase 2: SIMD batch rotation on the kept points
      cloud_.x.resize(kept);
      cloud_.y.resize(kept);
      cloud_.h.resize(kept);
      cloud_.size = kept;
      simd::rotateCloud(bpcDx_.data(), bpcDy_.data(), cosY, sinY,
                        cloud_.x.data(), cloud_.y.data(), kept);
      std::memcpy(cloud_.h.data(), bpcH_.data(), kept * sizeof(float));
    } else {
      for (int i = 0; i < n; i++) {
        float px = pts[i*4], py = pts[i*4+1], pz = pts[i*4+2], h = pts[i*4+3];
        float dx = px - fx, dy = py - fy, dz = pz - fz;
        if (dx*dx + dy*dy >= adjRangeSq) continue;
        if (!((dz > minZ && dz < maxZ) || useTerrain)) continue;
        float bx = dx * cosY + dy * sinY;
        float by = -dx * sinY + dy * cosY;
        if (insideFootprint(bx, by)) continue;
        cloud_.push(bx, by, h);
      }
    }
  }

  // Scratch buffers for buildPlannerCloud SIMD path
  std::vector<float> bpcDx_, bpcDy_, bpcH_;

  int scoreAndSelect(double pathScale, double pathRange, double relGoalDis,
                     double joyDir, double joySpeed, int& slowDown) {
    const auto& lut = rotLUT();
    int totalPaths = kRotDirs * kPathNum;
    int totalGroups = kRotDirs * kGroupNum;

    // Single memset pass: clear all scoring arrays at once
    std::memset(clearPathList_.data(), 0, sizeof(int) * totalPaths);
    std::memset(pathPenaltyList_.data(), 0, sizeof(float) * totalPaths);
    std::memset(clearPathPerGroupScore_.data(), 0, sizeof(double) * totalGroups);
    std::memset(clearPathPerGroupNum_.data(), 0, sizeof(int) * totalGroups);
    std::memset(pathPenaltyPerGroupScore_.data(), 0, sizeof(float) * totalGroups);

    // Pre-filter valid rotation directions into compact array (no branch in hot loop)
    nValidRotDirs_ = 0;
    for (int d = 0; d < kRotDirs; d++) {
      float a = std::fabs(static_cast<float>(joyDir) - lut.rotAngDeg[d]);
      if (a > 180.0f) a = 360.0f - a;
      float rotDeg = 10.0f * d;
      bool skip = (a > p_.dirThre && !p_.dirToVehicle) ||
                  (std::fabs(lut.rotAngDeg[d]) > p_.dirThre && std::fabs(joyDir) <= 90.0 && p_.dirToVehicle) ||
                  ((rotDeg > p_.dirThre && 360.0f - rotDeg > p_.dirThre) && std::fabs(joyDir) > 90.0 && p_.dirToVehicle);
      if (!skip) {
        validRotDirs_[nValidRotDirs_++] = d;
      }
    }

    // Voxel grid constants (precomputed as float for inner loop)
    float invPS = 1.0f / static_cast<float>(pathScale);
    float invGS = 1.0f / 0.02f;
    float offXH = 3.2f + 0.02f * 0.5f;
    float offYH = 4.5f + 0.02f * 0.5f;
    float scaleA = 0.45f / 4.5f;
    float scaleB = 1.0f / 3.2f;
    float pathRangeScaleSq = static_cast<float>((pathRange / pathScale) * (pathRange / pathScale));
    float goalClearScaleSq = static_cast<float>(((relGoalDis + p_.goalClearRange) / pathScale)
                             * ((relGoalDis + p_.goalClearRange) / pathScale));
    float obsThre = static_cast<float>(p_.obstacleHeightThre);
    float gndThre = static_cast<float>(p_.groundHeightThre);
    bool useTerrain = p_.useTerrainAnalysis;
    bool useCost = p_.useCost;
    bool cropByGoal = p_.pathCropByGoal;
    int pptThre = p_.pointPerPathThre;
    int cloudSize = cloud_.size;

    double minObsAngCW = -180.0;
    double minObsAngCCW = 180.0;
    if (p_.checkRotObstacle && cloudSize > 0) {
      const float halfLenScale = static_cast<float>(p_.vehicleLength * 0.5) * invPS;
      const float halfWidScale = static_cast<float>(p_.vehicleWidth * 0.5) * invPS;
      const float diameterScaleSq = halfLenScale * halfLenScale + halfWidScale * halfWidScale;
      const float angOffset = std::atan2(static_cast<float>(p_.vehicleWidth),
                                         static_cast<float>(p_.vehicleLength)) *
                              180.0f / static_cast<float>(M_PI);
      for (int i = 0; i < cloudSize; i++) {
        const float x = cloud_.x[i] * invPS;
        const float y = cloud_.y[i] * invPS;
        const float disSq = x * x + y * y;
        const float h = cloud_.h[i];
        if (disSq < diameterScaleSq &&
            (std::fabs(x) > halfLenScale || std::fabs(y) > halfWidScale) &&
            (h > obsThre || !useTerrain)) {
          const float angObs = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
          if (angObs > 0) {
            if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
            if (minObsAngCW < angObs + angOffset - 180.0f) minObsAngCW = angObs + angOffset - 180.0f;
          } else {
            if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
            if (minObsAngCCW > 180.0f + angObs - angOffset) minObsAngCCW = 180.0f + angObs - angOffset;
          }
        }
      }
      if (minObsAngCW > 0) minObsAngCW = 0;
      if (minObsAngCCW < 0) minObsAngCCW = 0;
    }

    // 鈹€鈹€ Obstacle scoring: ROTATION-MAJOR loop with SIMD batch rotation 鈹€鈹€
    if (p_.checkObstacle && cloudSize > 0) {
      const float* ch = cloud_.h.data();

      // Pre-scale cloud by invPS once (reused across all rotations)
      scaledX_.resize(cloudSize);
      scaledY_.resize(cloudSize);
      disSqBuf_.resize(cloudSize);
      {
        const float* cx = cloud_.x.data();
        const float* cy = cloud_.y.data();
        float* sx = scaledX_.data();
        float* sy = scaledY_.data();
        for (int i = 0; i < cloudSize; i++) {
          sx[i] = cx[i] * invPS;
          sy[i] = cy[i] * invPS;
        }
        simd::distSqBatch(sx, sy, disSqBuf_.data(), cloudSize);
      }

      // Per-rotation scoring lambda (each rotation writes to non-overlapping
      // clearPathList_[kPathNum*d..kPathNum*(d+1)], so parallel-safe)
      const float* sxp = scaledX_.data();
      const float* syp = scaledY_.data();
      const float* dsqp = disSqBuf_.data();
      const int* corrOff = corrOffset_.data();
      const int* corrDat = corrData_.data();
      int* clearArr = clearPathList_.data();
      float* penArr = pathPenaltyList_.data();
      int gvnx = gridVoxelNumX_, gvny = gridVoxelNumY_;

      auto scoreRotation = [&](int d, float* rxBuf, float* ryBuf) {
        float cosD = static_cast<float>(lut.c[d]);
        float sinD = static_cast<float>(lut.s[d]);
        int base = kPathNum * d;

        // SIMD batch rotation
        simd::rotateCloud(sxp, syp, cosD, sinD, rxBuf, ryBuf, cloudSize);

        // Voxel lookup (sequential, cache-friendly CSR)
        for (int i = 0; i < cloudSize; i++) {
          float disSq = dsqp[i];
          if (disSq >= pathRangeScaleSq) continue;
          if (cropByGoal && disSq > goalClearScaleSq) continue;

          float x2 = rxBuf[i];
          float y2 = ryBuf[i];
          float sy = x2 * scaleB + scaleA * (3.2f - x2) * scaleB;
          if (std::fabs(sy) < 1e-6f) continue;

          int indX = static_cast<int>((offXH - x2) * invGS);
          int indY = static_cast<int>((offYH - y2 / sy) * invGS);
          if (indX < 0 || indX >= gvnx || indY < 0 || indY >= gvny) continue;

          float h = ch[i];
          int ind = gvny * indX + indY;
          // Prefetch next iteration's CSR offset (reduces cache miss stalls)
          #if defined(__GNUC__) || defined(__clang__)
          if (i + 1 < cloudSize)
            __builtin_prefetch(corrOff + ind + 2, 0, 1);
          #endif
          const int* pb = corrDat + corrOff[ind];
          const int* pe = corrDat + corrOff[ind + 1];
          for (const int* pp = pb; pp != pe; ++pp) {
            int idx = base + *pp;
            if (h > obsThre || !useTerrain) {
              clearArr[idx]++;
            } else if (useCost && h > gndThre) {
              if (penArr[idx] < h) penArr[idx] = h;
            }
          }
        }
      };

      // Parallel execution: each rotation has its own scratch buffers
      if (nValidRotDirs_ >= 4 && cloudSize >= 100) {
        // Allocate per-thread scratch (one pair per rotation)
        parRotBufs_.resize(nValidRotDirs_ * 2 * cloudSize);
        #pragma omp parallel for schedule(dynamic) if(nValidRotDirs_ >= 6)
        for (int vi = 0; vi < nValidRotDirs_; vi++) {
          float* rxBuf = parRotBufs_.data() + vi * 2 * cloudSize;
          float* ryBuf = rxBuf + cloudSize;
          scoreRotation(validRotDirs_[vi], rxBuf, ryBuf);
        }
      } else {
        // Sequential: reuse single buffer pair
        rotX_.resize(cloudSize);
        rotY_.resize(cloudSize);
        for (int vi = 0; vi < nValidRotDirs_; vi++) {
          scoreRotation(validRotDirs_[vi], rotX_.data(), rotY_.data());
        }
      }
    }

    // 鈹€鈹€ Aggregate: nested loops (no division/modulo) + hoisted angDiffDeg 鈹€鈹€
    PathScoreParams sp;
    sp.dirWeight = p_.dirWeight;
    sp.slopeWeight = p_.slopeWeight;
    sp.omniDirGoalThre = p_.omniDirGoalThre;
    std::vector<float> traversabilityRiskCache(totalGroups, -1.0f);

    for (int vi = 0; vi < nValidRotDirs_; vi++) {
      int rotDir = validRotDirs_[vi];
      int pathBase = kPathNum * rotDir;
      int groupBase = kGroupNum * rotDir;
      double rotW4 = lut.rotDirW4[rotDir];
      float rotAng = lut.rotAngDeg[rotDir];

      for (int pathIdx = 0; pathIdx < kPathNum; pathIdx++) {
        int flatIdx = pathBase + pathIdx;
        if (clearPathList_[flatIdx] >= pptThre) continue;

        // angDiffDeg hoisted: only rotAng changes per outer loop
        double dirDiff = angDiffDeg(joyDir,
                                    static_cast<double>(endDirPathList_[pathIdx]) + rotAng);
        int grp = pathList_[pathIdx];
        int riskIdx = groupBase + grp;
        float travRisk = traversabilityRiskCache[riskIdx];
        if (travRisk < 0.0f) {
          travRisk = traversabilityRiskForGroup(
              rotDir, grp, pathScale, pathRange, relGoalDis);
          traversabilityRiskCache[riskIdx] = travRisk;
        }
        if (p_.useTraversabilityCost &&
            travRisk >= static_cast<float>(p_.traversabilityHardCost)) {
          continue;
        }
        double grpW2 = lut.groupDirW2[grp];
        double score = scorePathFast(dirDiff, rotW4, grpW2,
                                     pathPenaltyList_[flatIdx], relGoalDis,
                                     sp, lut);
        if (p_.useTraversabilityCost &&
            travRisk > static_cast<float>(p_.traversabilitySoftCost)) {
          double penalty = 1.0 - p_.traversabilityWeight *
              (static_cast<double>(travRisk) - p_.traversabilitySoftCost);
          score *= std::clamp(penalty, 0.0, 1.0);
        }
        if (score > 0) {
          int groupIdx = groupBase + grp;
          clearPathPerGroupScore_[groupIdx] += score;
          clearPathPerGroupNum_[groupIdx]++;
          pathPenaltyPerGroupScore_[groupIdx] += pathPenaltyList_[flatIdx];
        }
      }
    }

    // Select best group
    int selectedGroupID = selectBestGroup(clearPathPerGroupScore_, kGroupNum,
                                          minObsAngCW, minObsAngCCW,
                                          p_.twoWayDrive,
                                          p_.checkRotObstacle).selectedGroupID;

    // Compute slow-down
    if (selectedGroupID >= 0) {
      int num = clearPathPerGroupNum_[selectedGroupID];
      float penaltyScore = (num > 0) ? pathPenaltyPerGroupScore_[selectedGroupID] / num : 0;
      if (penaltyScore > p_.costHeightThre1) slowDown = 1;
      else if (penaltyScore > p_.costHeightThre2) slowDown = 2;
      else if (num < p_.slowPathNumThre &&
               std::abs(selectedGroupID - 129) > p_.slowGroupNumThre) slowDown = 3;
      else slowDown = 0;
    }

    return selectedGroupID;
  }

  float traversabilityRiskForGroup(int rotDir,
                                   int groupID,
                                   double pathScale,
                                   double pathRange,
                                   double relGoalDis) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_.empty() ||
        traversabilityRows_ <= 0 || traversabilityCols_ <= 0 ||
        traversabilityResolution_ <= 0.0 || groupID < 0 || groupID >= kGroupNum) {
      return 0.0f;
    }
    const auto& seg = startPaths_[groupID];
    if (seg.empty()) return 0.0f;

    const auto& lut = rotLUT();
    double rc = lut.c[rotDir], rs = lut.s[rotDir];
    double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);
    float maxRisk = 0.0f;
    for (const auto& pt : seg) {
      double disSq = pt.x * pt.x + pt.y * pt.y;
      if (disSq > pathRangeScaleSq || disSq > relGoalScaledSq) break;
      double bx = pathScale * (rc * pt.x - rs * pt.y);
      double by = pathScale * (rs * pt.x + rc * pt.y);
      float risk = traversabilityRiskAtBody(bx, by);
      if (risk > maxRisk) maxRisk = risk;
    }
    return std::clamp(maxRisk, 0.0f, 100.0f);
  }

  void buildOutputPath(int selectedGroupID, double pathScale, double pathRange,
                       double relGoalDis, LocalPlanResult& result) {
    const auto& lut = rotLUT();
    int rotDir = selectedGroupID / kGroupNum;
    int groupID = selectedGroupID % kGroupNum;
    double rc = lut.c[rotDir], rs = lut.s[rotDir];

    double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);

    auto& seg = startPaths_[groupID];
    result.path.clear();
    result.path.reserve(seg.size());

    for (auto& pt : seg) {
      double disSq = pt.x * pt.x + pt.y * pt.y;
      if (disSq > pathRangeScaleSq || disSq > relGoalScaledSq) break;
      // Rotate back to body frame, then scale
      double bx = pathScale * (rc * pt.x - rs * pt.y);
      double by = pathScale * (rs * pt.x + rc * pt.y);
      double bz = pathScale * pt.z;
      result.path.push_back({bx, by, bz});
    }
    result.pathFound = !result.path.empty();
  }

  void buildRecoveryPath(LocalPlanResult& result) {
    if (blockedStartTime_ < 0) blockedStartTime_ = odomTime_;
    double blockedDur = odomTime_ - blockedStartTime_;

    if (recoveryState_ == 0 && blockedDur >= p_.recoveryBlockedThre) {
      if (recoveryCycleCount_ >= p_.recoveryMaxCycles) {
        // Exhausted recovery cycles; stop and wait.
      } else {
        recoveryState_ = 1;
        recoveryPhaseStart_ = odomTime_;
      }
    } else if (recoveryState_ == 1 &&
               odomTime_ - recoveryPhaseStart_ >= p_.recoveryRotateTime) {
      recoveryState_ = 2;
      recoveryPhaseStart_ = odomTime_;
    } else if (recoveryState_ == 2 &&
               odomTime_ - recoveryPhaseStart_ >= p_.recoveryBackupTime) {
      recoveryState_ = 0;
      blockedStartTime_ = odomTime_;
      recoveryCycleCount_++;
    }

    result.path.clear();

    if (recoveryState_ == 1) {
      // Rotate toward goal
      double goalAngle = std::atan2(goalY_ - vy_, goalX_ - vx_);
      double relAngle = goalAngle - vyaw_;
      while (relAngle > M_PI) relAngle -= 2.0 * M_PI;
      while (relAngle < -M_PI) relAngle += 2.0 * M_PI;
      double turnDir = (relAngle >= 0) ? 1.0 : -1.0;

      // Check rotation safety (using SoA cloud)
      bool safe = true;
      for (int ci = 0; ci < cloud_.size; ci++) {
        float px = cloud_.x[ci], py = cloud_.y[ci];
        double dist = std::hypot(px, py);
        if (dist < 0.9 && dist > 0.1) {
          double ptAngle = std::atan2(py, px);
          bool inSweep = turnDir > 0 ? (ptAngle > 0 && ptAngle < 1.5)
                                     : (ptAngle < 0 && ptAngle > -1.5);
          if (inSweep && (cloud_.h[ci] > p_.obstacleHeightThre * 0.5 || !p_.useTerrainAnalysis)) {
            safe = false;
            break;
          }
        }
      }
      double dir = safe ? turnDir : -turnDir;
      for (int i = 1; i <= 6; i++) {
        double arc = dir * i * 0.25;
        result.path.push_back({0.15 * i * std::cos(arc), 0.15 * i * std::sin(arc), 0.0});
      }
    } else if (recoveryState_ == 2) {
      // Backup: check rear safety
      bool safe = true;
      for (int ci = 0; ci < cloud_.size; ci++) {
        float px = cloud_.x[ci];
        if (px < -0.1f && px > -1.2f && std::fabs(cloud_.y[ci]) < 0.35f) {
          if (cloud_.h[ci] > p_.obstacleHeightThre * 0.5 || !p_.useTerrainAnalysis) {
            safe = false; break;
          }
        }
      }
      if (safe) {
        for (int i = 1; i <= 5; i++)
          result.path.push_back({-0.2 * i, 0.0, 0.0});
      } else {
        recoveryState_ = 0;
        blockedStartTime_ = odomTime_;
        recoveryCycleCount_++;
        result.path.push_back({0, 0, 0});
      }
    } else {
      result.path.push_back({0, 0, 0});
    }
  }
};

}  // namespace nav_kernel
