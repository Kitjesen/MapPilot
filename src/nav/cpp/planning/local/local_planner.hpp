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
#include "recovery_planner.hpp"
#include "local_planner_scoring.hpp"
#include "nav_kernel/simd_accel.hpp"
#include <cmath>
#include <cstring>
#include <atomic>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <limits>
#include <sstream>
#include <algorithm>
#include <cstdint>

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
  double obstacleHeightMax  = 1.2;
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
  int    scoringThreads         = 2;
  int    debugCandidateLimit    = 0;

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
  int  recoveryState = 0;   // ABI: 0=idle, 1=true rotation, 2=validated translation
  bool recoveryExhausted = false;
  bool recoveryActive = false;
  bool recoveryVerified = false;
  bool recoveryDirectCommand = false;
  bool recoveryObservationRefreshRequired = false;
  RecoveryAction recoveryAction = RecoveryAction::None;
  std::string recoveryReason;
  double recoveryProgress = 0.0;
  int recoveryAttempt = 0;
  int recoveryCandidateCount = 0;
  int recoveryRotationDirection = 0;
};

enum class LocalCandidateState {
  Feasible,
  CollisionBlocked,
  RotationBlocked,
  TerrainCost,
  TerrainBlocked,
  DirectionRejected,
};

inline const char* localCandidateStateName(LocalCandidateState state) {
  switch (state) {
    case LocalCandidateState::Feasible:
      return "feasible";
    case LocalCandidateState::CollisionBlocked:
      return "collision_blocked";
    case LocalCandidateState::RotationBlocked:
      return "rotation_blocked";
    case LocalCandidateState::TerrainCost:
      return "terrain_cost";
    case LocalCandidateState::TerrainBlocked:
      return "terrain_blocked";
    case LocalCandidateState::DirectionRejected:
      return "direction_rejected";
  }
  return "direction_rejected";
}

struct LocalPlanCandidate {
  int rotationIndex{-1};
  int groupId{-1};
  double rotationDeg{0.0};
  double aggregateScore{0.0};
  double terrainRisk{-1.0};
  int totalPathCount{0};
  int collisionFreePathCount{0};
  int directionAllowedPathCount{0};
  int terrainAllowedPathCount{0};
  int directionScoredPathCount{0};
  int heightCostAllowedPathCount{0};
  int terrainSoftPenalizedPathCount{0};
  int contributingPathCount{0};
  bool rotationAllowed{true};
  bool selected{false};
  LocalCandidateState state{LocalCandidateState::DirectionRejected};
  std::vector<Vec3> path;
};

struct LocalPlannerDebugSnapshot {
  bool valid{false};
  double timestampS{0.0};
  double pathScale{0.0};
  double pathRange{0.0};
  double relativeGoalDistanceM{0.0};
  double traversabilitySoftCost{0.0};
  double traversabilityHardCost{0.0};
  int validRotationCount{0};
  int selectedGroupId{-1};
  std::vector<LocalPlanCandidate> candidates;
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

  LocalPlannerDebugSnapshot debugSnapshot() const { return debugSnapshot_; }

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
  /// obstacle_pts: Nx4 float array [x,y,z,height] in odom frame.
  /// timestamp: monotonic seconds.
  LocalPlanResult plan(const float* obstacle_pts, int n_pts, double timestamp) {
    if (!pathsLoaded_) return {};

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
    return planForIntent(
        obstacle_pts,
        n_pts,
        timestamp,
        relGoalDis,
        joyDir,
        joySpeed,
        p_.dirThre,
        false,
        true);
  }

  /// Force recovery planning after the execution layer has detected a stall
  /// even when the ordinary local planner can still produce a geometric path.
  LocalPlanResult planRecovery(const float* obstacle_pts,
                               int n_pts,
                               double timestamp) {
    LocalPlanResult result;
    odomTime_ = timestamp;
    debugSnapshot_ = {};

    const double relGoalX =
        (goalX_ - vx_) * cosYaw_ + (goalY_ - vy_) * sinYaw_;
    const double relGoalY =
        -(goalX_ - vx_) * sinYaw_ + (goalY_ - vy_) * cosYaw_;
    const double goalDirectionBodyRad = std::atan2(relGoalY, relGoalX);
    result.nearFieldStop =
        checkNearFieldStop(obstacle_pts, n_pts, goalDirectionBodyRad) ||
        (p_.traversabilityNearFieldStop &&
         checkNearFieldTraversability(goalDirectionBodyRad));
    buildPlannerCloud(obstacle_pts, n_pts);
    buildRecoveryPath(result, goalDirectionBodyRad, true);
    populateRecoveryResult(result);
    return result;
  }

  /// Explicit lifecycle boundary used when missions or control modes change.
  void resetRecovery() {
    recoverySessionActive_ = false;
    recoveryState_ = 0;
    blockedStartTime_ = -1.0;
    recoveryPhaseStart_ = -1.0;
    recoveryCycleCount_ = 0;
    recoveryAction_ = RecoveryAction::None;
    recoveryWorldPath_.clear();
    recoveryPathCumulative_.clear();
    recoveryPathLength_ = 0.0;
    recoveryTargetYawDelta_ = 0.0;
    recoveryRotationDirection_ = 0;
    recoveryDirectionBin_ = -1;
    recoveryRejectedTranslationMask_ = 0;
    recoveryRejectedRotationMask_ = 0;
    recoveryLastProgress_ = 0.0;
    recoveryLastProgressTime_ = -1.0;
    recoveryCandidateCount_ = 0;
    recoveryReason_.clear();
  }

  /// Plan from an operator motion intent without requiring a global path.
  /// Automatic recovery is disabled for assisted teleop.
  LocalPlanResult planIntent(const float* obstacle_pts,
                             int n_pts,
                             double timestamp,
                             double dir_body_deg,
                             double speed_norm,
                             double horizon_m,
                             double max_dir_deviation_deg) {
    if (!pathsLoaded_) return {};
    freezeStatus_ = 0;
    resetRecovery();
    return planForIntent(
        obstacle_pts,
        n_pts,
        timestamp,
        std::max(0.0, horizon_m),
        std::remainder(dir_body_deg, 360.0),
        std::clamp(speed_norm, 0.0, 1.0),
        std::clamp(max_dir_deviation_deg, 0.0, 180.0),
        true,
        false);
  }

 private:
  LocalPlanResult planForIntent(const float* obstacle_pts,
                                int n_pts,
                                double timestamp,
                                double relGoalDis,
                                 double joyDir,
                                 double joySpeed,
                                 double dirThre,
                                 bool hardPathDirectionLimit,
                                 bool allowRecovery) {
    LocalPlanResult result;
    odomTime_ = timestamp;
    debugSnapshot_ = {};

    // Near-field stop check covers explicit obstacle points and native
    // traversability risk cells in front of the body.
    const double intentDirRad = joyDir * M_PI / 180.0;
    result.nearFieldStop =
        checkNearFieldStop(obstacle_pts, n_pts, intentDirRad) ||
        (p_.traversabilityNearFieldStop &&
         checkNearFieldTraversability(intentDirRad));

    // Transform obstacles to body frame and crop
    buildPlannerCloud(obstacle_pts, n_pts);

    // Path scoring loop with scale degradation
    double pathRange = p_.adjacentRange;
    if (p_.pathRangeBySpeed) pathRange = p_.adjacentRange * joySpeed;
    if (pathRange < p_.minPathRange) pathRange = p_.minPathRange;

    // Guard against degenerate scale params: pathScale <= 0 would divide by
    // zero below (pathRange = ... / defPathScale), and minPathScale <= 0 would
    // make 1/curPathScale blow up inside scoreAndSelect.
    double defPathScale = p_.pathScale;
    if (defPathScale <= 1e-3) defPathScale = 1.0;
    const double minPathScale = std::max(p_.minPathScale, 1e-3);
    double curPathScale = defPathScale;
    if (p_.pathScaleBySpeed) curPathScale = defPathScale * joySpeed;
    if (curPathScale < minPathScale) curPathScale = minPathScale;

    bool pathFound = false;

    while (curPathScale >= minPathScale && pathRange >= p_.minPathRange) {
      int selectedGroupID = scoreAndSelect(
          curPathScale,
          pathRange,
          relGoalDis,
          joyDir,
          joySpeed,
          dirThre,
          hardPathDirectionLimit,
          result.slowDown);
      captureDebugSnapshot(
          selectedGroupID,
          curPathScale,
          pathRange,
          relGoalDis,
          joyDir,
          dirThre,
          hardPathDirectionLimit,
          timestamp);

      if (selectedGroupID >= 0) {
        buildOutputPath(selectedGroupID, curPathScale, pathRange, relGoalDis, result);
        if (result.pathFound) {
          pathFound = true;
          resetRecovery();
        }
        break;
      }
      if (curPathScale >= minPathScale + p_.pathScaleStep) {
        curPathScale -= p_.pathScaleStep;
        pathRange = p_.adjacentRange * curPathScale / defPathScale;
      } else {
        pathRange -= p_.pathRangeStep;
      }
    }

    result.pathFound = pathFound;

    if (!pathFound && allowRecovery) {
      buildRecoveryPath(result, joyDir * M_PI / 180.0, false);
    }

    populateRecoveryResult(result);
    return result;
  }

 public:
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
  LocalPlannerDebugSnapshot debugSnapshot_;
  double lastMinObstacleAngleCw_{-180.0};
  double lastMinObstacleAngleCcw_{180.0};

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

  // Recovery execution state. Candidate generation stays stateless in
  // RecoveryPlanner; this class owns odometry progress and retry policy.
  bool recoverySessionActive_ = false;
  int recoveryState_ = 0;
  double blockedStartTime_ = -1.0;
  double recoveryPhaseStart_ = -1.0;
  int recoveryCycleCount_ = 0;
  RecoveryAction recoveryAction_ = RecoveryAction::None;
  std::vector<Vec3> recoveryWorldPath_;
  std::vector<double> recoveryPathCumulative_;
  double recoveryPathLength_ = 0.0;
  double recoveryTargetYawDelta_ = 0.0;
  double recoveryStartX_ = 0.0;
  double recoveryStartY_ = 0.0;
  double recoveryStartYaw_ = 0.0;
  double recoveryLastProgress_ = 0.0;
  double recoveryLastProgressTime_ = -1.0;
  int recoveryCandidateCount_ = 0;
  int recoveryRotationDirection_ = 0;
  int recoveryDirectionBin_ = -1;
  std::uint32_t recoveryRejectedTranslationMask_ = 0;
  int recoveryRejectedRotationMask_ = 0;
  std::string recoveryReason_;

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

  bool pointInIntentSweep(float bx,
                          float by,
                          double intentDirRad,
                          double inflation = 0.0) const {
    const double sweepDistance = std::max(0.0, p_.nearFieldStopDis);
    if (sweepDistance <= 0.0) return false;

    const double halfLength = footprintHalfLength() + std::max(0.0, inflation);
    const double halfWidth = footprintHalfWidth() + std::max(0.0, inflation);
    const double ux = std::cos(intentDirRad);
    const double uy = std::sin(intentDirRad);
    const double forwardProjection = static_cast<double>(bx) * ux +
                                     static_cast<double>(by) * uy;
    if (forwardProjection <= 0.0) return false;
    double enter = 0.0;
    double exit = sweepDistance;

    auto clipAxis = [&](double coordinate, double direction, double halfExtent) {
      constexpr double kDirectionEpsilon = 1e-9;
      if (std::fabs(direction) < kDirectionEpsilon) {
        return std::fabs(coordinate) <= halfExtent;
      }
      double t0 = (coordinate - halfExtent) / direction;
      double t1 = (coordinate + halfExtent) / direction;
      if (t0 > t1) std::swap(t0, t1);
      enter = std::max(enter, t0);
      exit = std::min(exit, t1);
      return enter <= exit + kDirectionEpsilon;
    };

    return clipAxis(static_cast<double>(bx), ux, halfLength) &&
           clipAxis(static_cast<double>(by), uy, halfWidth) &&
           exit > 0.0;
  }

  bool checkNearFieldStop(const float* pts, int n, double intentDirRad) {
    if (!p_.checkObstacle) return false;
    for (int i = 0; i < n; i++) {
      float px = pts[i*4], py = pts[i*4+1], h = pts[i*4+3];
      if (!heightInsideBodyEnvelope(h)) continue;
      float dx = px - (float)vx_, dy = py - (float)vy_;
      // To body frame
      float bx = dx * (float)cosYaw_ + dy * (float)sinYaw_;
      float by = -dx * (float)sinYaw_ + dy * (float)cosYaw_;
      if (insideFootprint(bx, by)) continue;
      if (pointInIntentSweep(bx, by, intentDirRad) &&
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
      return 100.0f;
    }
    float risk = traversabilityGrid_[row * traversabilityCols_ + col];
    if (!std::isfinite(risk)) return 100.0f;
    return std::clamp(risk, 0.0f, 100.0f);
  }

  float traversabilityRiskAtBody(double bx, double by) const {
    double wx = vx_ + bx * cosYaw_ - by * sinYaw_;
    double wy = vy_ + bx * sinYaw_ + by * cosYaw_;
    return traversabilityRiskAtWorld(wx, wy);
  }

  bool checkNearFieldTraversability(double intentDirRad) const {
    if (!p_.checkObstacle || !p_.useTraversabilityCost ||
        traversabilityGrid_.empty() || traversabilityResolution_ <= 0.0) {
      return false;
    }

    const double ux = std::cos(intentDirRad);
    const double uy = std::sin(intentDirRad);
    const double travel = std::max(0.0, p_.nearFieldStopDis);
    const double halfLength = footprintHalfLength();
    const double halfWidth = footprintHalfWidth();
    const double endX = travel * ux;
    const double endY = travel * uy;

    double minWorldX = std::numeric_limits<double>::infinity();
    double maxWorldX = -std::numeric_limits<double>::infinity();
    double minWorldY = std::numeric_limits<double>::infinity();
    double maxWorldY = -std::numeric_limits<double>::infinity();
    for (double centerX : {0.0, endX}) {
      for (double centerY : {0.0, endY}) {
        for (double cornerX : {-halfLength, halfLength}) {
          for (double cornerY : {-halfWidth, halfWidth}) {
            const double bx = centerX + cornerX;
            const double by = centerY + cornerY;
            const double wx = vx_ + bx * cosYaw_ - by * sinYaw_;
            const double wy = vy_ + bx * sinYaw_ + by * cosYaw_;
            minWorldX = std::min(minWorldX, wx);
            maxWorldX = std::max(maxWorldX, wx);
            minWorldY = std::min(minWorldY, wy);
            maxWorldY = std::max(maxWorldY, wy);
          }
        }
      }
    }

    const int minCol = std::max(
        0,
        static_cast<int>(std::floor(
            (minWorldX - traversabilityOriginX_) / traversabilityResolution_)));
    const int maxCol = std::min(
        traversabilityCols_ - 1,
        static_cast<int>(std::floor(
            (maxWorldX - traversabilityOriginX_) / traversabilityResolution_)));
    const int minRow = std::max(
        0,
        static_cast<int>(std::floor(
            (minWorldY - traversabilityOriginY_) / traversabilityResolution_)));
    const int maxRow = std::min(
        traversabilityRows_ - 1,
        static_cast<int>(std::floor(
            (maxWorldY - traversabilityOriginY_) / traversabilityResolution_)));
    const double cellInflation = traversabilityResolution_ * 0.5;

    for (int row = minRow; row <= maxRow; ++row) {
      const double wy = traversabilityOriginY_ +
          (static_cast<double>(row) + 0.5) * traversabilityResolution_;
      for (int col = minCol; col <= maxCol; ++col) {
        const float risk = traversabilityGrid_[row * traversabilityCols_ + col];
        if (!std::isfinite(risk) ||
            risk < static_cast<float>(p_.traversabilityHardCost)) {
          continue;
        }
        const double wx = traversabilityOriginX_ +
            (static_cast<double>(col) + 0.5) * traversabilityResolution_;
        const double dx = wx - vx_;
        const double dy = wy - vy_;
        const float bx = static_cast<float>(dx * cosYaw_ + dy * sinYaw_);
        const float by = static_cast<float>(-dx * sinYaw_ + dy * cosYaw_);
        if (insideFootprint(bx, by)) continue;
        if (pointInIntentSweep(bx, by, intentDirRad, cellInflation)) {
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

  bool heightInsideBodyEnvelope(float relative_height) const {
    return std::isfinite(relative_height) &&
           relative_height <= static_cast<float>(p_.obstacleHeightMax);
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
        if (!heightInsideBodyEnvelope(pts[i*4+3])) continue;
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
        if (!heightInsideBodyEnvelope(h)) continue;
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
                     double joyDir, double joySpeed, double dirThre,
                     bool hardPathDirectionLimit,
                     int& slowDown) {
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
      bool skip = (a > dirThre && !p_.dirToVehicle) ||
                  (std::fabs(lut.rotAngDeg[d]) > dirThre && std::fabs(joyDir) <= 90.0 && p_.dirToVehicle) ||
                  ((rotDeg > dirThre && 360.0f - rotDeg > dirThre) && std::fabs(joyDir) > 90.0 && p_.dirToVehicle);
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
      const float halfLenScale = static_cast<float>(footprintHalfLength()) * invPS;
      const float halfWidScale = static_cast<float>(footprintHalfWidth()) * invPS;
      const float diameterScaleSq = halfLenScale * halfLenScale + halfWidScale * halfWidScale;
      const float angOffset = std::atan2(halfWidScale, halfLenScale) *
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
    lastMinObstacleAngleCw_ = minObsAngCW;
    lastMinObstacleAngleCcw_ = minObsAngCCW;

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
      const int scoringThreads = std::clamp(p_.scoringThreads, 1, 4);
      if (nValidRotDirs_ >= 4 && cloudSize >= 100 && scoringThreads > 1) {
        // Allocate per-thread scratch (one pair per rotation)
        parRotBufs_.resize(nValidRotDirs_ * 2 * cloudSize);
        // A bounded team avoids oversubscribing the control process when SLAM
        // and traversability are running concurrently on the same robot CPU.
        #pragma omp parallel for schedule(static) num_threads(scoringThreads) if(nValidRotDirs_ >= 6)
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
    std::vector<float> outputEndDirectionCache(totalGroups, 1000.0f);

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
        if (hardPathDirectionLimit && dirDiff > dirThre) continue;
        int grp = pathList_[pathIdx];
        int riskIdx = groupBase + grp;
        if (hardPathDirectionLimit) {
          float& outputEndDirection = outputEndDirectionCache[riskIdx];
          if (outputEndDirection > 360.0f) {
            outputEndDirection = static_cast<float>(outputPathEndDirectionDeg(
                rotDir, grp, pathScale, pathRange, relGoalDis));
          }
          if (angDiffDeg(joyDir, outputEndDirection) > dirThre) continue;
        }
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

  int collisionFreePathCount(int rotDir, int groupId) const {
    if (rotDir < 0 || rotDir >= kRotDirs || groupId < 0 || groupId >= kGroupNum) {
      return 0;
    }
    int count = 0;
    const int pathBase = rotDir * kPathNum;
    for (int pathIdx = 0; pathIdx < kPathNum; ++pathIdx) {
      if (pathList_[pathIdx] == groupId &&
          clearPathList_[pathBase + pathIdx] < p_.pointPerPathThre) {
        ++count;
      }
    }
    return count;
  }

  std::vector<Vec3> buildDebugCandidatePath(
      int rotDir,
      int groupId,
      double pathScale,
      double pathRange,
      double relGoalDis) const {
    constexpr std::size_t kDebugPointLimit = 16;
    std::vector<Vec3> full;
    if (rotDir < 0 || rotDir >= kRotDirs || groupId < 0 || groupId >= kGroupNum) {
      return full;
    }
    const auto& segment = startPaths_[groupId];
    if (segment.empty()) {
      return full;
    }
    const auto& lut = rotLUT();
    const double rc = lut.c[rotDir];
    const double rs = lut.s[rotDir];
    const double pathRangeScaleSq =
        (pathRange / pathScale) * (pathRange / pathScale);
    const double relGoalScaledSq =
        (relGoalDis / pathScale) * (relGoalDis / pathScale);
    full.reserve(segment.size());
    for (const auto& point : segment) {
      const double distanceSq = point.x * point.x + point.y * point.y;
      if (distanceSq > pathRangeScaleSq || distanceSq > relGoalScaledSq) {
        break;
      }
      full.push_back({
          pathScale * (rc * point.x - rs * point.y),
          pathScale * (rs * point.x + rc * point.y),
          pathScale * point.z,
      });
    }
    if (full.size() <= kDebugPointLimit) {
      return full;
    }
    std::vector<Vec3> sampled;
    sampled.reserve(kDebugPointLimit);
    for (std::size_t index = 0; index < kDebugPointLimit; ++index) {
      const std::size_t source =
          index * (full.size() - 1) / (kDebugPointLimit - 1);
      sampled.push_back(full[source]);
    }
    return sampled;
  }

  void traceDebugCandidateGates(
      LocalPlanCandidate& candidate,
      double pathScale,
      double pathRange,
      double relGoalDis,
      double joyDir,
      double dirThre,
      bool hardPathDirectionLimit) const {
    const int rotDir = candidate.rotationIndex;
    const int groupId = candidate.groupId;
    const int pathBase = rotDir * kPathNum;
    const auto& lut = rotLUT();
    PathScoreParams scoreParams;
    scoreParams.dirWeight = p_.dirWeight;
    scoreParams.slopeWeight = p_.slopeWeight;
    scoreParams.omniDirGoalThre = p_.omniDirGoalThre;
    const double rotWeight4 = lut.rotDirW4[rotDir];
    const double groupWeight2 = lut.groupDirW2[groupId];
    const float rotAngle = lut.rotAngDeg[rotDir];
    double outputEndDirection = 1000.0;

    candidate.rotationAllowed = rotationPassesObstacleGate(
        rotDir,
        lastMinObstacleAngleCw_,
        lastMinObstacleAngleCcw_,
        p_.twoWayDrive,
        p_.checkRotObstacle);
    candidate.terrainRisk = -1.0;
    for (int pathIdx = 0; pathIdx < kPathNum; ++pathIdx) {
      if (pathList_[pathIdx] != groupId) {
        continue;
      }
      ++candidate.totalPathCount;
      const int flatPath = pathBase + pathIdx;
      if (clearPathList_[flatPath] >= p_.pointPerPathThre) {
        continue;
      }
      ++candidate.collisionFreePathCount;

      const double directionDifference = angDiffDeg(
          joyDir, static_cast<double>(endDirPathList_[pathIdx]) + rotAngle);
      if (hardPathDirectionLimit && directionDifference > dirThre) {
        continue;
      }
      if (hardPathDirectionLimit) {
        if (outputEndDirection > 360.0) {
          outputEndDirection = outputPathEndDirectionDeg(
              rotDir, groupId, pathScale, pathRange, relGoalDis);
        }
        if (angDiffDeg(joyDir, outputEndDirection) > dirThre) {
          continue;
        }
      }
      ++candidate.directionAllowedPathCount;

      if (candidate.terrainRisk < 0.0) {
        candidate.terrainRisk = traversabilityRiskForGroup(
            rotDir, groupId, pathScale, pathRange, relGoalDis);
      }
      if (p_.useTraversabilityCost &&
          candidate.terrainRisk >= p_.traversabilityHardCost) {
        continue;
      }
      ++candidate.terrainAllowedPathCount;

      const double directionScore = scorePathFast(
          directionDifference,
          rotWeight4,
          groupWeight2,
          0.0f,
          relGoalDis,
          scoreParams,
          lut);
      if (directionScore <= 0.0) {
        continue;
      }
      ++candidate.directionScoredPathCount;

      double score = scorePathFast(
          directionDifference,
          rotWeight4,
          groupWeight2,
          pathPenaltyList_[flatPath],
          relGoalDis,
          scoreParams,
          lut);
      if (score <= 0.0) {
        continue;
      }
      ++candidate.heightCostAllowedPathCount;

      if (p_.useTraversabilityCost &&
          candidate.terrainRisk > p_.traversabilitySoftCost) {
        const double penalty = std::clamp(
            1.0 - p_.traversabilityWeight *
                (candidate.terrainRisk - p_.traversabilitySoftCost),
            0.0,
            1.0);
        if (penalty < 1.0) {
          ++candidate.terrainSoftPenalizedPathCount;
        }
        score *= penalty;
      }
      if (score > 0.0) {
        ++candidate.contributingPathCount;
      }
    }

    if (!candidate.rotationAllowed) {
      candidate.state = LocalCandidateState::RotationBlocked;
    } else if (candidate.totalPathCount > 0 &&
               candidate.collisionFreePathCount == 0) {
      candidate.state = LocalCandidateState::CollisionBlocked;
    } else if (candidate.directionAllowedPathCount == 0) {
      candidate.state = LocalCandidateState::DirectionRejected;
    } else if (candidate.terrainAllowedPathCount == 0) {
      candidate.state = LocalCandidateState::TerrainBlocked;
    } else if (candidate.directionScoredPathCount == 0) {
      candidate.state = LocalCandidateState::DirectionRejected;
    } else if (candidate.heightCostAllowedPathCount == 0 ||
               candidate.contributingPathCount == 0) {
      candidate.state = LocalCandidateState::TerrainBlocked;
    } else if (candidate.terrainSoftPenalizedPathCount > 0) {
      candidate.state = LocalCandidateState::TerrainCost;
    } else {
      candidate.state = LocalCandidateState::Feasible;
    }
  }

  void captureDebugSnapshot(
      int selectedGroupId,
      double pathScale,
      double pathRange,
      double relGoalDis,
      double joyDir,
      double dirThre,
      bool hardPathDirectionLimit,
      double timestamp) {
    debugSnapshot_ = {};
    if (p_.debugCandidateLimit <= 0) {
      return;
    }
    debugSnapshot_.valid = true;
    debugSnapshot_.timestampS = timestamp;
    debugSnapshot_.pathScale = pathScale;
    debugSnapshot_.pathRange = pathRange;
    debugSnapshot_.relativeGoalDistanceM = relGoalDis;
    debugSnapshot_.traversabilitySoftCost = p_.traversabilitySoftCost;
    debugSnapshot_.traversabilityHardCost = p_.traversabilityHardCost;
    debugSnapshot_.validRotationCount = nValidRotDirs_;
    debugSnapshot_.selectedGroupId = selectedGroupId;

    std::vector<LocalPlanCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(nValidRotDirs_));
    const auto& lut = rotLUT();
    const int selectedRotation =
        selectedGroupId >= 0 ? selectedGroupId / kGroupNum : -1;
    const int selectedGroup =
        selectedGroupId >= 0 ? selectedGroupId % kGroupNum : -1;
    for (int validIndex = 0; validIndex < nValidRotDirs_; ++validIndex) {
      const int rotDir = validRotDirs_[validIndex];
      int representativeGroup = 3;
      double bestScore = -1.0;
      int bestCollisionFree = -1;
      for (int groupId = 0; groupId < kGroupNum; ++groupId) {
        const int flatGroup = rotDir * kGroupNum + groupId;
        const double score = clearPathPerGroupScore_[flatGroup];
        const int collisionFree = collisionFreePathCount(rotDir, groupId);
        if (score > bestScore + 1e-12 ||
            (std::abs(score - bestScore) <= 1e-12 &&
             collisionFree > bestCollisionFree) ||
            (std::abs(score - bestScore) <= 1e-12 &&
             collisionFree == bestCollisionFree &&
             std::abs(groupId - 3) < std::abs(representativeGroup - 3))) {
          representativeGroup = groupId;
          bestScore = score;
          bestCollisionFree = collisionFree;
        }
      }
      if (rotDir == selectedRotation) {
        representativeGroup = selectedGroup;
      }

      const int flatGroup = rotDir * kGroupNum + representativeGroup;
      LocalPlanCandidate candidate;
      candidate.rotationIndex = rotDir;
      candidate.groupId = representativeGroup;
      candidate.rotationDeg = lut.rotAngDeg[rotDir];
      candidate.aggregateScore = clearPathPerGroupScore_[flatGroup];
      candidate.selected = flatGroup == selectedGroupId;
      traceDebugCandidateGates(
          candidate,
          pathScale,
          pathRange,
          relGoalDis,
          joyDir,
          dirThre,
          hardPathDirectionLimit);
      candidate.path = buildDebugCandidatePath(
          rotDir,
          representativeGroup,
          pathScale,
          pathRange,
          relGoalDis);
      candidates.push_back(std::move(candidate));
    }

    const std::size_t limit = static_cast<std::size_t>(
        std::clamp(p_.debugCandidateLimit, 0, kRotDirs));
    if (candidates.size() <= limit) {
      debugSnapshot_.candidates = std::move(candidates);
      return;
    }
    std::vector<LocalPlanCandidate> sampled;
    sampled.reserve(limit);
    for (std::size_t index = 0; index < limit; ++index) {
      const std::size_t source = limit == 1
          ? candidates.size() / 2
          : index * (candidates.size() - 1) / (limit - 1);
      sampled.push_back(candidates[source]);
    }
    const auto selectedIt = std::find_if(
        candidates.begin(),
        candidates.end(),
        [](const LocalPlanCandidate& candidate) { return candidate.selected; });
    if (selectedIt != candidates.end() &&
        std::none_of(
            sampled.begin(),
            sampled.end(),
            [](const LocalPlanCandidate& candidate) { return candidate.selected; })) {
      sampled.back() = *selectedIt;
      std::sort(
          sampled.begin(),
          sampled.end(),
          [](const LocalPlanCandidate& lhs, const LocalPlanCandidate& rhs) {
            return lhs.rotationIndex < rhs.rotationIndex;
          });
    }
    debugSnapshot_.candidates = std::move(sampled);
  }

  double outputPathEndDirectionDeg(int rotDir,
                                   int groupID,
                                   double pathScale,
                                   double pathRange,
                                   double relGoalDis) const {
    if (groupID < 0 || groupID >= kGroupNum) return 0.0;
    const auto& segment = startPaths_[groupID];
    if (segment.size() < 2) return rotLUT().rotAngDeg[rotDir];

    const double pathRangeScaleSq =
        (pathRange / pathScale) * (pathRange / pathScale);
    const double relGoalScaledSq =
        (relGoalDis / pathScale) * (relGoalDis / pathScale);
    double previousX = 0.0;
    double previousY = 0.0;
    double lastX = 0.0;
    double lastY = 0.0;
    int pointCount = 0;
    for (const auto& point : segment) {
      const double distanceSq = point.x * point.x + point.y * point.y;
      if (distanceSq > pathRangeScaleSq || distanceSq > relGoalScaledSq) break;
      previousX = lastX;
      previousY = lastY;
      lastX = point.x;
      lastY = point.y;
      ++pointCount;
    }
    if (pointCount < 2) return rotLUT().rotAngDeg[rotDir];

    const double dx = lastX - previousX;
    const double dy = lastY - previousY;
    const auto& lut = rotLUT();
    const double rotatedDx = lut.c[rotDir] * dx - lut.s[rotDir] * dy;
    const double rotatedDy = lut.s[rotDir] * dx + lut.c[rotDir] * dy;
    return std::atan2(rotatedDy, rotatedDx) * 180.0 / M_PI;
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

  enum class RecoveryUpdate { Active, Completed, Failed };

  struct RecoveryPathProjection {
    double progress{0.0};
    double alongDistance{0.0};
    double crossTrackDistance{std::numeric_limits<double>::infinity()};
    std::size_t segmentIndex{0};
  };

  RecoveryPlannerParams makeRecoveryPlannerParams() const {
    RecoveryPlannerParams params;
    params.vehicleLength = p_.vehicleLength;
    params.vehicleWidth = p_.vehicleWidth;
    params.footprintPadding = p_.footprintPadding;
    params.obstacleHeightThreshold = p_.obstacleHeightThre;
    params.useTerrainAnalysis = p_.useTerrainAnalysis;
    params.checkObstacles = p_.checkObstacle;
    params.requireTraversability = p_.useTraversabilityCost;
    params.traversabilityHardCost = p_.traversabilityHardCost;

    const double configuredRange =
        std::isfinite(p_.adjacentRange) ? std::max(0.0, p_.adjacentRange)
                                        : params.searchRadius;
    params.searchRadius = std::max(
        params.minTranslationDistance + params.latticeResolution,
        std::min(params.searchRadius, configuredRange));
    return params;
  }

  Pose recoveryVehiclePose() const {
    Pose pose;
    pose.position = {vx_, vy_, vz_};
    pose.yaw = vyaw_;
    return pose;
  }

  RecoveryPlannerInput makeRecoveryPlannerInput(
      double goalDirectionBodyRad) const {
    RecoveryPlannerInput input;
    input.vehiclePose = recoveryVehiclePose();
    if (p_.checkObstacle && cloud_.size > 0) {
      input.obstacleX = cloud_.x.data();
      input.obstacleY = cloud_.y.data();
      input.obstacleHeight = cloud_.h.data();
      input.obstacleCount = cloud_.size;
    }
    if (!traversabilityGrid_.empty()) {
      input.traversabilityGrid = traversabilityGrid_.data();
      input.traversabilityRows = traversabilityRows_;
      input.traversabilityCols = traversabilityCols_;
      input.traversabilityResolution = traversabilityResolution_;
      input.traversabilityOriginX = traversabilityOriginX_;
      input.traversabilityOriginY = traversabilityOriginY_;
    }
    input.goalDirectionBodyRad = goalDirectionBodyRad;
    input.rejectedTranslationDirectionMask =
        recoveryRejectedTranslationMask_;
    input.rejectedRotationDirectionMask = recoveryRejectedRotationMask_;
    return input;
  }

  static const char* recoverySafetyFailureName(SafetyFailure failure) {
    switch (failure) {
      case SafetyFailure::None:
        return "none";
      case SafetyFailure::InvalidInput:
        return "invalid_input";
      case SafetyFailure::ObstacleCollision:
        return "obstacle_collision";
      case SafetyFailure::TraversabilityUnavailable:
        return "traversability_unavailable";
      case SafetyFailure::TraversabilityOutOfBounds:
        return "traversability_out_of_bounds";
      case SafetyFailure::TraversabilityNonFinite:
        return "traversability_non_finite";
      case SafetyFailure::TraversabilityInvalidValue:
        return "traversability_invalid_value";
      case SafetyFailure::TraversabilityBlocked:
        return "traversability_blocked";
    }
    return "invalid_input";
  }

  void populateRecoveryResult(LocalPlanResult& result) const {
    result.recoveryState = recoveryState_;
    result.recoveryActive = recoverySessionActive_;
    result.recoveryVerified = recoveryState_ != 0;
    result.recoveryDirectCommand =
        result.recoveryVerified && recoveryAction_ == RecoveryAction::Rotate;
    result.recoveryAction =
        result.recoveryVerified ? recoveryAction_ : RecoveryAction::None;
    result.recoveryReason = recoveryReason_;
    result.recoveryProgress =
        std::clamp(recoveryLastProgress_, 0.0, 1.0);
    result.recoveryAttempt =
        recoveryState_ != 0 ? recoveryCycleCount_ + 1 : recoveryCycleCount_;
    result.recoveryCandidateCount = recoveryCandidateCount_;
    result.recoveryRotationDirection =
        recoveryState_ == 1 ? recoveryRotationDirection_ : 0;
  }

  void clearActiveRecovery() {
    recoveryState_ = 0;
    recoveryPhaseStart_ = -1.0;
    recoveryAction_ = RecoveryAction::None;
    recoveryWorldPath_.clear();
    recoveryPathCumulative_.clear();
    recoveryPathLength_ = 0.0;
    recoveryTargetYawDelta_ = 0.0;
    recoveryRotationDirection_ = 0;
    recoveryDirectionBin_ = -1;
  }

  void finishRecovery(const char* reason) {
    clearActiveRecovery();
    recoverySessionActive_ = false;
    blockedStartTime_ = -1.0;
    recoveryCycleCount_ = 0;
    recoveryRejectedTranslationMask_ = 0;
    recoveryRejectedRotationMask_ = 0;
    recoveryLastProgress_ = 1.0;
    recoveryLastProgressTime_ = odomTime_;
    recoveryReason_ = reason;
  }

  void rejectActiveRecovery(const std::string& reason) {
    if (recoveryAction_ == RecoveryAction::Translate &&
        recoveryDirectionBin_ >= 0 && recoveryDirectionBin_ < 16) {
      recoveryRejectedTranslationMask_ |=
          std::uint32_t{1} << recoveryDirectionBin_;
    } else if (recoveryAction_ == RecoveryAction::Rotate) {
      recoveryRejectedRotationMask_ |=
          recoveryRotationDirection_ > 0 ? 0x1 : 0x2;
    }
    ++recoveryCycleCount_;
    clearActiveRecovery();
    recoveryReason_ = reason;
  }

  void markRecoveryExhausted(
      LocalPlanResult& result,
      const std::string& reason) {
    clearActiveRecovery();
    recoverySessionActive_ = false;
    result.path.clear();
    result.pathFound = false;
    result.recoveryExhausted = true;
    recoveryReason_ = reason;
  }

  void buildRecoveryPathCumulative() {
    recoveryPathCumulative_.assign(recoveryWorldPath_.size(), 0.0);
    for (std::size_t i = 1; i < recoveryWorldPath_.size(); ++i) {
      recoveryPathCumulative_[i] =
          recoveryPathCumulative_[i - 1] +
          distance2D(recoveryWorldPath_[i - 1], recoveryWorldPath_[i]);
    }
    recoveryPathLength_ =
        recoveryPathCumulative_.empty() ? 0.0
                                        : recoveryPathCumulative_.back();
  }

  RecoveryPathProjection projectRecoveryPath() const {
    RecoveryPathProjection projection;
    if (recoveryWorldPath_.size() < 2 ||
        recoveryPathCumulative_.size() != recoveryWorldPath_.size() ||
        recoveryPathLength_ <= 1e-9) {
      return projection;
    }

    const Vec3 vehicle{vx_, vy_, vz_};
    double bestDistanceSq = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < recoveryWorldPath_.size(); ++i) {
      const Vec3& a = recoveryWorldPath_[i];
      const Vec3& b = recoveryWorldPath_[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double segmentLengthSq = dx * dx + dy * dy;
      if (segmentLengthSq <= 1e-12) continue;

      const double t = std::clamp(
          ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy) /
              segmentLengthSq,
          0.0,
          1.0);
      const double nearestX = a.x + t * dx;
      const double nearestY = a.y + t * dy;
      const double errorX = vehicle.x - nearestX;
      const double errorY = vehicle.y - nearestY;
      const double distanceSq = errorX * errorX + errorY * errorY;
      if (distanceSq < bestDistanceSq) {
        bestDistanceSq = distanceSq;
        const double segmentLength = std::sqrt(segmentLengthSq);
        projection.segmentIndex = i;
        projection.alongDistance =
            recoveryPathCumulative_[i] + t * segmentLength;
      }
    }

    projection.crossTrackDistance = std::sqrt(bestDistanceSq);
    projection.progress = std::clamp(
        projection.alongDistance / recoveryPathLength_, 0.0, 1.0);
    return projection;
  }

  std::vector<Vec3> remainingRecoveryPathBody(
      std::size_t segmentIndex) const {
    const Pose currentPose = recoveryVehiclePose();
    const std::vector<Vec3> body =
        RecoveryPlanner::worldPathToBody(recoveryWorldPath_, currentPose);
    std::vector<Vec3> remaining;
    remaining.reserve(body.size() + 1);
    remaining.push_back({0.0, 0.0, 0.0});
    if (body.empty()) return remaining;

    const std::size_t first =
        std::min(segmentIndex + 1, body.size() - 1);
    for (std::size_t i = first; i < body.size(); ++i) {
      if (distance2D(remaining.back(), body[i]) > 1e-4) {
        remaining.push_back(body[i]);
      }
    }
    return remaining;
  }

  bool activateRecoveryCandidate(const RecoveryPlanResult& candidate) {
    if (!candidate.found()) return false;

    recoveryAction_ = candidate.action;
    recoveryStartX_ = vx_;
    recoveryStartY_ = vy_;
    recoveryStartYaw_ = vyaw_;
    recoveryPhaseStart_ = odomTime_;
    recoveryLastProgress_ = 0.0;
    recoveryLastProgressTime_ = odomTime_;
    recoveryCandidateCount_ =
        candidate.diagnostics.candidateCount +
        candidate.diagnostics.rotationCandidateCount;
    recoveryDirectionBin_ = candidate.diagnostics.selectedDirectionBin;

    if (candidate.action == RecoveryAction::Translate) {
      recoveryWorldPath_ = candidate.pathWorld;
      if (recoveryWorldPath_.empty()) {
        recoveryWorldPath_ = RecoveryPlanner::bodyPathToWorld(
            candidate.pathBody, recoveryVehiclePose());
      }
      const Vec3 start{vx_, vy_, vz_};
      if (recoveryWorldPath_.empty() ||
          distance2D(recoveryWorldPath_.front(), start) > 1e-4) {
        recoveryWorldPath_.insert(recoveryWorldPath_.begin(), start);
      }
      buildRecoveryPathCumulative();
      if (recoveryWorldPath_.size() < 2 ||
          recoveryPathLength_ <= 1e-6 ||
          recoveryDirectionBin_ < 0 ||
          recoveryDirectionBin_ >= 16) {
        clearActiveRecovery();
        return false;
      }
      recoveryState_ = 2;
      recoveryRotationDirection_ = 0;
      recoveryReason_ = "recovery_translation_active";
      return true;
    }

    if (candidate.action == RecoveryAction::Rotate &&
        std::isfinite(candidate.rotationDeltaRad) &&
        std::fabs(candidate.rotationDeltaRad) > 1e-6) {
      recoveryState_ = 1;
      recoveryTargetYawDelta_ = candidate.rotationDeltaRad;
      recoveryRotationDirection_ =
          candidate.rotationDeltaRad > 0.0 ? 1 : -1;
      recoveryReason_ = "recovery_rotation_active";
      return true;
    }

    clearActiveRecovery();
    return false;
  }

  RecoveryUpdate updateActiveRecovery(
      const RecoveryPlanner& planner,
      const RecoveryPlannerInput& input,
      LocalPlanResult& result,
      std::string* failureReason) {
    if (recoveryLastProgressTime_ > odomTime_) {
      recoveryLastProgressTime_ = odomTime_;
    }

    if (recoveryAction_ == RecoveryAction::Translate) {
      const RecoveryPathProjection projection = projectRecoveryPath();
      const double crossTrackLimit =
          std::max(0.20, footprintHalfWidth());
      const double observedProgress =
          projection.crossTrackDistance <= crossTrackLimit
              ? projection.progress
              : recoveryLastProgress_;
      if (observedProgress > recoveryLastProgress_ + 0.01) {
        recoveryLastProgress_ = observedProgress;
        recoveryLastProgressTime_ = odomTime_;
      }

      const double completionDistance =
          std::max(0.08, makeRecoveryPlannerParams().latticeResolution * 1.25);
      if (recoveryPathLength_ - projection.alongDistance <=
          completionDistance) {
        finishRecovery("recovery_translation_complete");
        return RecoveryUpdate::Completed;
      }

      std::vector<Vec3> remaining =
          remainingRecoveryPathBody(projection.segmentIndex);
      SafetyFailure failure = SafetyFailure::None;
      if (remaining.size() < 2 ||
          !planner.validateBodyPath(input, remaining, &failure)) {
        if (failureReason != nullptr) {
          *failureReason =
              std::string("recovery_translation_unsafe_") +
              recoverySafetyFailureName(failure);
        }
        return RecoveryUpdate::Failed;
      }

      const double timeout =
          std::isfinite(p_.recoveryBackupTime)
              ? std::max(0.05, p_.recoveryBackupTime)
              : 0.05;
      if (odomTime_ - recoveryLastProgressTime_ >= timeout) {
        if (failureReason != nullptr) {
          *failureReason = "recovery_translation_no_progress";
        }
        return RecoveryUpdate::Failed;
      }

      result.path = std::move(remaining);
      result.pathFound = result.path.size() >= 2;
      recoveryReason_ = "recovery_translation_active";
      return RecoveryUpdate::Active;
    }

    if (recoveryAction_ == RecoveryAction::Rotate) {
      const double target = std::fabs(recoveryTargetYawDelta_);
      const double signedTravel =
          normalizeAngle(vyaw_ - recoveryStartYaw_) *
          static_cast<double>(recoveryRotationDirection_);
      const double observedProgress =
          target > 1e-9
              ? std::clamp(std::max(0.0, signedTravel) / target, 0.0, 1.0)
              : 0.0;
      if (observedProgress > recoveryLastProgress_ + 0.01) {
        recoveryLastProgress_ = observedProgress;
        recoveryLastProgressTime_ = odomTime_;
      }

      const double remainingMagnitude =
          std::max(0.0, target - std::max(0.0, signedTravel));
      if (remainingMagnitude <= 0.05) {
        result.recoveryObservationRefreshRequired = true;
        finishRecovery("recovery_rotation_complete");
        return RecoveryUpdate::Completed;
      }

      const double remainingDelta =
          static_cast<double>(recoveryRotationDirection_) *
          remainingMagnitude;
      SafetyFailure failure = SafetyFailure::None;
      if (!planner.validateRotation(input, remainingDelta, &failure)) {
        if (failureReason != nullptr) {
          *failureReason =
              std::string("recovery_rotation_unsafe_") +
              recoverySafetyFailureName(failure);
        }
        return RecoveryUpdate::Failed;
      }

      const double timeout =
          std::isfinite(p_.recoveryRotateTime)
              ? std::max(0.05, p_.recoveryRotateTime)
              : 0.05;
      if (odomTime_ - recoveryLastProgressTime_ >= timeout) {
        if (failureReason != nullptr) {
          *failureReason = "recovery_rotation_no_progress";
        }
        return RecoveryUpdate::Failed;
      }

      result.path.clear();
      result.pathFound = false;
      recoveryReason_ = "recovery_rotation_active";
      return RecoveryUpdate::Active;
    }

    if (failureReason != nullptr) {
      *failureReason = "recovery_invalid_active_action";
    }
    return RecoveryUpdate::Failed;
  }

  void buildRecoveryPath(
      LocalPlanResult& result,
      double goalDirectionBodyRad,
      bool forceStart) {
    result.path.clear();
    result.pathFound = false;

    if (!std::isfinite(odomTime_) ||
        !std::isfinite(goalDirectionBodyRad)) {
      recoveryReason_ = "recovery_invalid_input";
      return;
    }
    if (blockedStartTime_ < 0.0 || blockedStartTime_ > odomTime_) {
      blockedStartTime_ = odomTime_;
    }

    const int maxAttempts = std::max(0, p_.recoveryMaxCycles);
    if (maxAttempts == 0 || recoveryCycleCount_ >= maxAttempts) {
      markRecoveryExhausted(result, "recovery_exhausted");
      return;
    }

    const RecoveryPlanner planner(makeRecoveryPlannerParams());
    RecoveryPlannerInput input =
        makeRecoveryPlannerInput(goalDirectionBodyRad);
    bool retryAfterFailure = false;

    if (recoveryState_ != 0) {
      std::string failureReason;
      const RecoveryUpdate update =
          updateActiveRecovery(planner, input, result, &failureReason);
      if (update == RecoveryUpdate::Active ||
          update == RecoveryUpdate::Completed) {
        return;
      }

      rejectActiveRecovery(failureReason);
      retryAfterFailure = true;
      if (recoveryCycleCount_ >= maxAttempts) {
        markRecoveryExhausted(
            result, failureReason + "_exhausted");
        return;
      }
      input = makeRecoveryPlannerInput(goalDirectionBodyRad);
    }

    const double blockedThreshold =
        std::isfinite(p_.recoveryBlockedThre)
            ? std::max(0.0, p_.recoveryBlockedThre)
            : 0.0;
    if (!forceStart && !retryAfterFailure &&
        odomTime_ - blockedStartTime_ < blockedThreshold) {
      recoveryReason_ = "recovery_blocked_wait";
      return;
    }

    recoverySessionActive_ = true;
    const RecoveryPlanResult candidate = planner.plan(input);
    recoveryCandidateCount_ =
        candidate.diagnostics.candidateCount +
        candidate.diagnostics.rotationCandidateCount;
    if (!candidate.found() || !activateRecoveryCandidate(candidate)) {
      ++recoveryCycleCount_;
      recoveryReason_ =
          std::string("recovery_no_safe_candidate_") +
          recoverySafetyFailureName(candidate.safetyFailure);
      if (recoveryCycleCount_ >= maxAttempts) {
        markRecoveryExhausted(
            result, recoveryReason_ + "_exhausted");
      }
      return;
    }

    input = makeRecoveryPlannerInput(goalDirectionBodyRad);
    std::string failureReason;
    const RecoveryUpdate update =
        updateActiveRecovery(planner, input, result, &failureReason);
    if (update == RecoveryUpdate::Failed) {
      rejectActiveRecovery(failureReason);
      if (recoveryCycleCount_ >= maxAttempts) {
        markRecoveryExhausted(
            result, failureReason + "_exhausted");
      }
    }
  }
};

}  // namespace nav_kernel
