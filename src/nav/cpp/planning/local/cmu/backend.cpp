#include "planning/local/cmu/backend.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nav_kernel/simd_accel.hpp"

namespace nav_kernel {

namespace {

constexpr double kCmuFollowerLookAheadM = 0.5;

struct RotLUT {
  static constexpr int kPow025Size = 361;

  std::array<double, kRotDirs> s{};
  std::array<double, kRotDirs> c{};
  std::array<double, kRotDirs> rotDirW4{};
  std::array<double, kGroupNum> groupDirW2{};
  std::array<float, kRotDirs> rotAngDeg{};
  std::array<float, kPow025Size> pow025{};

  RotLUT() {
    for (int i = 0; i < kRotDirs; ++i) {
      const double angle = (10.0 * i - 180.0) * M_PI / 180.0;
      s[i] = std::sin(angle);
      c[i] = std::cos(angle);
      rotAngDeg[i] = 10.0f * i - 180.0f;
      const double weight = i < 18 ? std::fabs(i - 9.0) + 1.0 : std::fabs(i - 27.0) + 1.0;
      rotDirW4[i] = weight * weight * weight * weight;
    }
    for (int group = 0; group < kGroupNum; ++group) {
      const double weight = 4.0 - std::fabs(group - 3.0);
      groupDirW2[group] = weight * weight;
    }
    for (int i = 1; i < kPow025Size; ++i) {
      pow025[i] = static_cast<float>(std::sqrt(std::sqrt(i * 0.01)));
    }
  }
};

const RotLUT &rotLUT() {
  static const RotLUT lookup;
  return lookup;
}

double angDiffDeg(double a, double b) {
  double difference = std::fmod(std::fabs(a - b), 360.0);
  return difference > 180.0 ? 360.0 - difference : difference;
}

struct PathScoreParams {
  double dirWeight{0.02};
  double slopeWeight{0.0};
  double omniDirGoalThre{5.0};
};

double scorePathFast(double directionDifferenceDeg, double rotationDirectionWeight4,
                     double groupDirectionWeight2, float terrainPenalty,
                     double relativeGoalDistance, const PathScoreParams &params,
                     const RotLUT &lookup) {
  const float weightedDifference =
      static_cast<float>(std::fabs(params.dirWeight * directionDifferenceDeg));
  const int index =
      std::min(static_cast<int>(weightedDifference * 100.0f), RotLUT::kPow025Size - 1);
  const double base = 1.0 - static_cast<double>(lookup.pow025[index]);
  if (base <= 0.0)
    return 0.0;

  const double terrainFactor =
      params.slopeWeight > 0.0 ? std::max(0.0, 1.0 - params.slopeWeight * terrainPenalty) : 1.0;
  return base *
         (relativeGoalDistance < params.omniDirGoalThre ? groupDirectionWeight2
                                                        : rotationDirectionWeight4) *
         terrainFactor;
}

bool rotationPassesObstacleGate(int rotationDirection, double minObstacleAngleClockwise,
                                double minObstacleAngleCounterClockwise, bool twoWayDrive,
                                bool checkRotationObstacle) noexcept {
  if (!checkRotationObstacle)
    return true;
  const double rotationAngleDeg = 10.0 * rotationDirection - 180.0;
  double wrappedRotationDeg = 10.0 * rotationDirection;
  if (wrappedRotationDeg > 180.0)
    wrappedRotationDeg -= 360.0;
  return (rotationAngleDeg > minObstacleAngleClockwise &&
          rotationAngleDeg < minObstacleAngleCounterClockwise) ||
         (twoWayDrive && wrappedRotationDeg > minObstacleAngleClockwise &&
          wrappedRotationDeg < minObstacleAngleCounterClockwise);
}

struct GroupSelectionResult {
  int selectedGroupID{-1};
  double maxScore{0.0};
};

GroupSelectionResult selectBestGroup(const std::vector<double> &groupScores, int groupCount,
                                     double minObstacleAngleClockwise,
                                     double minObstacleAngleCounterClockwise, bool twoWayDrive,
                                     bool checkRotationObstacle) {
  GroupSelectionResult result;
  for (int index = 0; index < static_cast<int>(groupScores.size()); ++index) {
    const int rotationDirection = index / groupCount;
    if (rotationPassesObstacleGate(rotationDirection, minObstacleAngleClockwise,
                                   minObstacleAngleCounterClockwise, twoWayDrive,
                                   checkRotationObstacle) &&
        groupScores[index] > result.maxScore) {
      result.maxScore = groupScores[index];
      result.selectedGroupID = index;
    }
  }
  return result;
}

}  // namespace

class local::cmu::Backend::Impl {
 public:
  explicit Impl(const LocalPlannerParams &params = LocalPlannerParams()) : p_(params) {}

  /// Load pre-computed path library from directory containing PLY files.
  /// Returns true on success.
  bool loadPaths(const std::string &pathsDir) {
    Impl loaded(p_);
    if (!loaded.loadSearchRadius(pathsDir + "/search_radius.txt") ||
        !loaded.loadStartPaths(pathsDir + "/startPaths.ply") ||
        !loaded.loadPathList(pathsDir + "/pathList.ply") ||
        !loaded.loadCorrespondences(pathsDir + "/correspondences.txt")) {
      return false;
    }

    startPaths_.swap(loaded.startPaths_);
    pathList_.swap(loaded.pathList_);
    endDirPathList_.swap(loaded.endDirPathList_);
    corrOffset_.swap(loaded.corrOffset_);
    corrData_.swap(loaded.corrData_);
    clearPathList_.swap(loaded.clearPathList_);
    pathPenaltyList_.swap(loaded.pathPenaltyList_);
    clearPathPerGroupScore_.swap(loaded.clearPathPerGroupScore_);
    clearPathPerGroupNum_.swap(loaded.clearPathPerGroupNum_);
    pathPenaltyPerGroupScore_.swap(loaded.pathPenaltyPerGroupScore_);
    searchRadius_ = loaded.searchRadius_;
    pathsLoaded_ = true;
    return true;
  }

  bool pathsLoaded() const { return pathsLoaded_; }

  LocalPlannerDebugSnapshot debugSnapshot() const { return debugSnapshot_; }

  /// Apply one complete planning frame. Views remain valid for the synchronous
  /// plan call and are never retained by the public API.
  void applyInput(const LocalPlanRequest &input) {
    const auto &vehicle = input.robot.pose;
    const auto *route = input.route();
    applyVehicle(vehicle.position.x, vehicle.position.y, vehicle.position.z, vehicle.yaw);
    const Vec3 terminal = route != nullptr ? route->target() : vehicle.position;
    routeHorizonDistance_ =
        std::hypot(terminal.x - vehicle.position.x, terminal.y - vehicle.position.y);
    const Vec3 target = route != nullptr ? routeGuideTarget(*route) : terminal;
    applyGoal(target.x, target.y);
    applyTraversability(input.environment.traversability);
  }

  Vec3 routeGuideTarget(const LocalRouteView &route) const {
    const Vec3 terminal = route.target();
    if (route.count <= 2)
      return terminal;

    const Vec3 start = route.points[0];
    const double dx = terminal.x - start.x;
    const double dy = terminal.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-12)
      return terminal;

    const double bend_threshold = std::max(0.05, 0.10 * std::max(0.0, p_.vehicleWidth));
    Vec3 salient = terminal;
    double maximum_deviation = bend_threshold;
    for (int index = 1; index + 1 < route.count; ++index) {
      const Vec3 point = route.points[index];
      const double ratio = std::clamp(
          ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0);
      const double guide_x = start.x + ratio * dx;
      const double guide_y = start.y + ratio * dy;
      const double deviation = std::hypot(point.x - guide_x, point.y - guide_y);
      if (deviation > maximum_deviation) {
        maximum_deviation = deviation;
        salient = point;
      }
    }
    return salient;
  }

  void applyVehicle(double x, double y, double z, double yaw) {
    cosYaw_ = std::cos(yaw);
    sinYaw_ = std::sin(yaw);
    vx_ = x;
    vy_ = y;
    vz_ = z;
    vyaw_ = yaw;
  }

  /// Update goal position in the current planning frame.
  void applyGoal(double gx, double gy) {
    goalX_ = gx;
    goalY_ = gy;
  }

  /// Set a 0..100 traversability risk grid in the current planning frame.
  void applyTraversability(const LocalTraversabilityView &grid) {
    traversabilityGrid_ = nullptr;
    traversabilityRows_ = 0;
    traversabilityCols_ = 0;
    traversabilityResolution_ = 0.0;
    if (!grid.valid())
      return;
    traversabilityGrid_ = grid.values;
    traversabilityRows_ = grid.rows;
    traversabilityCols_ = grid.cols;
    traversabilityResolution_ = grid.resolution;
    traversabilityOriginX_ = grid.originX;
    traversabilityOriginY_ = grid.originY;
  }

  LocalPlan plan(const LocalPlanRequest &input) {
    applyInput(input);
    LocalPlan result;
    if (const auto *intent = input.intent()) {
      freezeStatus_ = 0;
      reset();
      result = planForIntent(
          input.environment.obstacles.xyzh, input.environment.obstacles.count,
          input.clock.timestampS, std::max(0.0, intent->horizonM),
          std::remainder(intent->directionBodyDeg, 360.0),
          std::clamp(intent->speedNormalized, 0.0, 1.0),
          std::clamp(intent->maxDirectionDeviationDeg, 0.0, 180.0), true);
    } else {
      result = planCurrent(input.environment.obstacles.xyzh, input.environment.obstacles.count,
                           input.clock.timestampS);
    }
    releaseInputViews();
    return result;
  }

  /// Run one planning cycle for the already-applied frame.
  LocalPlan planCurrent(const float *obstacle_pts, int n_pts, double timestamp) {
    if (!pathsLoaded_)
      return LocalPlan::stopped(LocalPlanStatus::NotConfigured);
    odomTime_ = timestamp;

    // Joy speed for autonomy mode
    double joySpeed = p_.autonomySpeed / p_.maxSpeed;
    joySpeed = std::clamp(joySpeed, 0.0, 1.0);

    // Transform goal to body frame
    const double relGoalX = (goalX_ - vx_) * cosYaw_ + (goalY_ - vy_) * sinYaw_;
    const double relGoalY = -(goalX_ - vx_) * sinYaw_ + (goalY_ - vy_) * cosYaw_;
    double relGoalDis = routeHorizonDistance_;
    double joyDir = std::atan2(relGoalY, relGoalX) * 180.0 / M_PI;

    if (!p_.twoWayDrive) {
      updateFreezeState(joyDir);
      if (freezeStatus_ == 1) {
        relGoalDis = 0;
        joyDir = 0;
      }
      joyDir = std::clamp(joyDir, -90.0, 90.0);
    } else {
      freezeStatus_ = 0;
    }
    return planForIntent(obstacle_pts, n_pts, timestamp, relGoalDis, joyDir, joySpeed, p_.dirThre,
                         false);
  }

  void reset() {
    freezeStatus_ = 0;
    freezeStartTime_ = 0.0;
    debugSnapshot_ = {};
  }

 private:
  void releaseInputViews() {
    traversabilityGrid_ = nullptr;
    traversabilityRows_ = 0;
    traversabilityCols_ = 0;
    traversabilityResolution_ = 0.0;
  }

  LocalPlan planForIntent(const float *obstacle_pts, int n_pts, double timestamp,
                                double relGoalDis, double joyDir, double joySpeed, double dirThre,
                                bool hardPathDirectionLimit) {
    std::vector<Vec3> path;
    int slowdown_level = 0;
    odomTime_ = timestamp;
    debugSnapshot_ = {};
    debugSnapshot_.valid = true;
    debugSnapshot_.backend = LocalPlannerBackend::Cmu;
    debugSnapshot_.timestampS = timestamp;
    debugSnapshot_.searchReason = "planning";

    // Near-field stop check covers explicit obstacle points and native
    // traversability risk cells in front of the body.
    const double intentDirRad = joyDir * M_PI / 180.0;
    bool near_field_stop =
        checkNearFieldStop(obstacle_pts, n_pts, intentDirRad) ||
        (p_.traversabilityNearFieldStop && checkNearFieldTraversability(intentDirRad));

    // Transform obstacles to body frame and crop
    buildPlannerCloud(obstacle_pts, n_pts);
    debugSnapshot_.collisionPointCount = cloud_.size;

    // Path scoring loop with scale degradation
    double pathRange = p_.adjacentRange;
    if (p_.pathRangeBySpeed)
      pathRange = p_.adjacentRange * joySpeed;
    if (pathRange < p_.minPathRange)
      pathRange = p_.minPathRange;

    // Guard against degenerate scale params: pathScale <= 0 would divide by
    // zero below (pathRange = ... / defPathScale), and minPathScale <= 0 would
    // make 1/curPathScale blow up inside scoreAndSelect.
    double defPathScale = p_.pathScale;
    if (defPathScale <= 1e-3)
      defPathScale = 1.0;
    const double minPathScale = std::max(p_.minPathScale, 1e-3);
    double curPathScale = defPathScale;
    if (p_.pathScaleBySpeed)
      curPathScale = defPathScale * joySpeed;
    if (curPathScale < minPathScale)
      curPathScale = minPathScale;

    bool pathFound = false;

    while (curPathScale >= minPathScale && pathRange >= p_.minPathRange) {
      int selectedGroupID = scoreAndSelect(curPathScale, pathRange, relGoalDis, joyDir, joySpeed,
                                           dirThre, hardPathDirectionLimit, slowdown_level);
      captureDebugSnapshot(selectedGroupID, curPathScale, pathRange, relGoalDis, joyDir, dirThre,
                           hardPathDirectionLimit, timestamp);

      if (selectedGroupID >= 0) {
        buildOutputPath(selectedGroupID, curPathScale, pathRange, relGoalDis, path);
        pathFound = !path.empty();
        break;
      }
      if (curPathScale >= minPathScale + p_.pathScaleStep) {
        curPathScale -= p_.pathScaleStep;
        pathRange = p_.adjacentRange * curPathScale / defPathScale;
      } else {
        pathRange -= p_.pathRangeStep;
      }
    }

    if (pathFound) {
      // The route-intent probe above is only a pre-plan warning. Once an exact
      // local path exists, evaluate the hard near-field gate along that path.
      // Using its first millimetres as a straight ray falsely stops curved
      // detours whose swept footprint has already been proven collision-free.
      near_field_stop = checkNearFieldPath(path);
    }
    const ControlHints hints{slowdown_level, true};
    if (near_field_stop) {
      return LocalPlan::path(std::move(path), LocalPlanStatus::NearFieldStop, hints);
    }
    return pathFound ? LocalPlan::path(std::move(path), hints)
                     : LocalPlan::stopped(LocalPlanStatus::NoPath, hints);
  }

 public:
  const LocalPlannerParams &params() const { return p_; }

 private:
  struct FootprintPenetration {
    double longitudinal{0.0};
    double lateral{0.0};
  };

  static double overlapDepth(const FootprintPenetration &penetration) {
    return std::max(0.0, penetration.longitudinal) *
           std::max(0.0, penetration.lateral);
  }

  LocalPlannerParams p_;
  bool pathsLoaded_ = false;
  LocalPlannerDebugSnapshot debugSnapshot_;
  double lastMinObstacleAngleCw_{-180.0};
  double lastMinObstacleAngleCcw_{180.0};

  // Vehicle state
  double vx_ = 0, vy_ = 0, vz_ = 0, vyaw_ = 0;
  double cosYaw_ = 1.0, sinYaw_ = 0.0;
  double goalX_ = 0, goalY_ = 0;
  // Route bends steer CMU, while the forward segment endpoint owns path
  // cropping. Coupling both to a near bend can produce a path shorter than
  // the follower stop distance even though the route continues for metres.
  double routeHorizonDistance_ = 0.0;
  double odomTime_ = 0;
  const float *traversabilityGrid_ = nullptr;
  int traversabilityRows_ = 0;
  int traversabilityCols_ = 0;
  double traversabilityResolution_ = 0.0;
  double traversabilityOriginX_ = 0.0;
  double traversabilityOriginY_ = 0.0;

  // Freeze state
  int freezeStatus_ = 0;
  double freezeStartTime_ = 0;

  // Path library
  struct PathPoint {
    float x, y, z;
  };
  std::array<std::vector<PathPoint>, kGroupNum> startPaths_;
  std::array<int, kPathNum> pathList_{};
  std::array<float, kPathNum> endDirPathList_{};

  // Correspondences: voxel_id -> path_ids (CSR format for cache locality)
  int gridVoxelNumX_ = 161;
  int gridVoxelNumY_ = 451;
  int gridVoxelNum_ = gridVoxelNumX_ * gridVoxelNumY_;
  double searchRadius_ = 0.0;
  // CSR: corrOffset_[i] .. corrOffset_[i+1] index into corrData_
  std::vector<int> corrOffset_;  // size = gridVoxelNum_ + 1
  std::vector<int> corrData_;    // flat array of all path IDs

  // Per-frame scratch buffers, using SoA layout for cache-friendly scoring.
  struct PlannerCloudSoA {
    std::vector<float> x, y, h;  // body-frame x, y + terrain height
    int size = 0;
    void clear() {
      x.clear();
      y.clear();
      h.clear();
      size = 0;
    }
    void reserve(int n) {
      x.reserve(n);
      y.reserve(n);
      h.reserve(n);
    }
    void push(float bx, float by, float bh) {
      x.push_back(bx);
      y.push_back(by);
      h.push_back(bh);
      size++;
    }
  } cloud_;

  static constexpr int kObstacleIndexCellsPerAxis = 64;
  std::vector<int> obstacleIndexHeads_;
  std::vector<int> obstacleIndexNext_;
  double obstacleIndexOrigin_ = 0.0;
  double obstacleIndexResolution_ = 0.0;

  // Pre-filtered valid rotation indices (avoids branch in inner loop)
  std::array<int, kRotDirs> validRotDirs_{};
  int nValidRotDirs_ = 0;

  // SIMD scratch buffers (reused across frames, no per-frame alloc)
  std::vector<float> scaledX_, scaledY_, disSqBuf_;
  std::vector<float> rotX_, rotY_;
  std::vector<float> parRotBufs_;  // parallel per-rotation scratch

  // Scoring arrays
  std::vector<int> clearPathList_;
  std::vector<float> pathPenaltyList_;
  std::vector<double> clearPathPerGroupScore_;
  std::vector<int> clearPathPerGroupNum_;
  std::vector<float> pathPenaltyPerGroupScore_;

  // File loading

  bool loadSearchRadius(const std::string &filename) {
    std::ifstream f(filename);
    double value = 0.0;
    if (!f.is_open() || !(f >> value) || !std::isfinite(value) || value <= 0.0) {
      return false;
    }
    f >> std::ws;
    if (!f.eof()) {
      return false;
    }
    searchRadius_ = value;
    return true;
  }

  int readPlyHeader(std::ifstream &f) {
    std::string line;
    int pointNum = -1;
    bool endHeaderSeen = false;
    while (std::getline(f, line)) {
      // Strip trailing \r (Windows CRLF safety)
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      if (line == "end_header") {
        endHeaderSeen = true;
        break;
      }
      // Parse "element vertex N"
      if (line.find("element vertex") != std::string::npos) {
        std::istringstream iss(line);
        std::string a, b;
        int n = 0;
        if (!(iss >> a >> b >> n) || n < 0)
          return -1;
        pointNum = n;
      }
    }
    return endHeaderSeen ? pointNum : -1;
  }

  bool loadStartPaths(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open())
      return false;
    int n = readPlyHeader(f);
    if (n <= 0)
      return false;
    std::array<std::vector<PathPoint>, kGroupNum> loaded;
    for (int i = 0; i < n; i++) {
      float x, y, z;
      int groupID;
      if (!(f >> x >> y >> z >> groupID) || !std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(z) || groupID < 0 || groupID >= kGroupNum) {
        return false;
      }
      loaded[groupID].push_back({x, y, z});
    }
    if (std::any_of(loaded.begin(), loaded.end(),
                    [](const auto &group) { return group.empty(); })) {
      return false;
    }
    startPaths_ = std::move(loaded);
    return true;
  }

  bool loadPathList(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open())
      return false;
    int n = readPlyHeader(f);
    if (n != kPathNum)
      return false;
    std::array<int, kPathNum> loadedPathList{};
    std::array<float, kPathNum> loadedEndDirections{};
    std::array<bool, kPathNum> seen{};
    for (int i = 0; i < kPathNum; i++) {
      float ex, ey, ez;
      int pathID, groupID;
      if (!(f >> ex >> ey >> ez >> pathID >> groupID) || !std::isfinite(ex) || !std::isfinite(ey) ||
          !std::isfinite(ez) || pathID < 0 || pathID >= kPathNum || groupID < 0 ||
          groupID >= kGroupNum || seen[pathID]) {
        return false;
      }
      seen[pathID] = true;
      loadedPathList[pathID] = groupID;
      loadedEndDirections[pathID] = 2.0f * std::atan2(ey, ex) * 180.0f / (float)M_PI;
    }
    pathList_ = loadedPathList;
    endDirPathList_ = loadedEndDirections;
    return true;
  }

  bool loadCorrespondences(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open())
      return false;

    // First pass: load into temporary vector-of-vectors
    std::vector<std::vector<int>> tmp(gridVoxelNum_);
    for (int i = 0; i < gridVoxelNum_; i++) {
      int gridVoxelID;
      if (!(f >> gridVoxelID) || gridVoxelID < 0 || gridVoxelID >= gridVoxelNum_) {
        return false;
      }
      int pathID;
      bool terminated = false;
      while (f >> pathID) {
        if (pathID == -1) {
          terminated = true;
          break;
        }
        if (pathID < 0 || pathID >= kPathNum)
          return false;
        tmp[gridVoxelID].push_back(pathID);
      }
      if (!terminated)
        return false;
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
      std::copy(tmp[i].begin(), tmp[i].end(), corrData_.begin() + corrOffset_[i]);
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

  // Frame processing.

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

  bool pointInIntentSweep(float bx, float by, double intentDirRad, double inflation = 0.0) const {
    const double sweepDistance = std::max(0.0, p_.nearFieldStopDis);
    if (sweepDistance <= 0.0)
      return false;

    const double halfLength = footprintHalfLength() + std::max(0.0, inflation);
    const double halfWidth = footprintHalfWidth() + std::max(0.0, inflation);
    const double ux = std::cos(intentDirRad);
    const double uy = std::sin(intentDirRad);
    const double forwardProjection = static_cast<double>(bx) * ux + static_cast<double>(by) * uy;
    if (forwardProjection <= 0.0)
      return false;
    double enter = 0.0;
    double exit = sweepDistance;

    auto clipAxis = [&](double coordinate, double direction, double halfExtent) {
      constexpr double kDirectionEpsilon = 1e-9;
      if (std::fabs(direction) < kDirectionEpsilon) {
        return std::fabs(coordinate) <= halfExtent;
      }
      double t0 = (coordinate - halfExtent) / direction;
      double t1 = (coordinate + halfExtent) / direction;
      if (t0 > t1)
        std::swap(t0, t1);
      enter = std::max(enter, t0);
      exit = std::min(exit, t1);
      return enter <= exit + kDirectionEpsilon;
    };

    return clipAxis(static_cast<double>(bx), ux, halfLength) &&
           clipAxis(static_cast<double>(by), uy, halfWidth) && exit > 0.0;
  }

  bool checkNearFieldStop(const float *pts, int n, double intentDirRad) {
    if (!p_.checkObstacle)
      return false;
    for (int i = 0; i < n; i++) {
      float px = pts[i * 4], py = pts[i * 4 + 1], h = pts[i * 4 + 3];
      if (!heightInsideBodyEnvelope(h))
        continue;
      float dx = px - (float)vx_, dy = py - (float)vy_;
      // To body frame
      float bx = dx * (float)cosYaw_ + dy * (float)sinYaw_;
      float by = -dx * (float)sinYaw_ + dy * (float)cosYaw_;
      if (insideSelfFilterFootprint(bx, by))
        continue;
      if (pointInIntentSweep(bx, by, intentDirRad) &&
          (h >= (float)p_.obstacleHeightThre || !p_.useTerrainAnalysis)) {
        return true;
      }
    }
    return false;
  }

  bool checkNearFieldPath(const std::vector<Vec3> &path) const {
    if (path.empty() || p_.nearFieldStopDis <= 0.0)
      return false;
    if (initialRotationBlocked(path))
      return true;

    const double sampleStep =
        std::clamp(std::min(footprintHalfLength(), footprintHalfWidth()) * 0.25, 0.02, 0.05);
    double remaining = p_.nearFieldStopDis;
    double previousX = 0.0;
    double previousY = 0.0;
    bool sampledStart = false;

    for (const Vec3 &point : path) {
      const double dx = point.x - previousX;
      const double dy = point.y - previousY;
      const double segmentLength = std::hypot(dx, dy);
      if (segmentLength <= 1e-9)
        continue;

      const double checkedLength = std::min(segmentLength, remaining);
      // The near-field translation sweep is expressed in the current body
      // frame. Any required initial yaw change was validated above.
      constexpr double yawBody = 0.0;
      if (!sampledStart) {
        if (obstacleFootprintBlockedAtBody(previousX, previousY, yawBody, true) ||
            (p_.traversabilityNearFieldStop && p_.useTraversabilityCost &&
             traversabilityFootprintRiskAtBody(previousX, previousY, yawBody, true) >=
                 static_cast<float>(p_.traversabilityHardCost))) {
          return true;
        }
        sampledStart = true;
      }

      const int steps = std::max(1, static_cast<int>(std::ceil(checkedLength / sampleStep)));
      for (int step = 1; step <= steps; ++step) {
        const double distance =
            checkedLength * static_cast<double>(step) / static_cast<double>(steps);
        const double ratio = distance / segmentLength;
        const double sampleX = previousX + dx * ratio;
        const double sampleY = previousY + dy * ratio;
        if (obstacleFootprintBlockedAtBody(sampleX, sampleY, yawBody, true) ||
            (p_.traversabilityNearFieldStop && p_.useTraversabilityCost &&
             traversabilityFootprintRiskAtBody(sampleX, sampleY, yawBody, true) >=
                 static_cast<float>(p_.traversabilityHardCost))) {
          return true;
        }
      }

      remaining -= checkedLength;
      if (remaining <= 1e-9)
        break;
      previousX = point.x;
      previousY = point.y;
    }
    return false;
  }

  bool initialRotationBlocked(const std::vector<Vec3> &path) const {
    double targetYaw = 0.0;
    bool rotates = false;
    for (const Vec3 &point : path) {
      if (std::hypot(point.x, point.y) <= 1e-6)
        continue;
      targetYaw = std::atan2(point.y, point.x);
      if (p_.twoWayDrive) {
        if (targetYaw > 0.5 * M_PI)
          targetYaw -= M_PI;
        else if (targetYaw < -0.5 * M_PI)
          targetYaw += M_PI;
      }
      rotates = std::abs(targetYaw) > 1e-6;
      break;
    }
    if (!rotates)
      return false;

    constexpr double kYawStep = 5.0 * M_PI / 180.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(targetYaw) / kYawStep)));
    for (int step = 1; step <= steps; ++step) {
      const double yaw = targetYaw * static_cast<double>(step) / static_cast<double>(steps);
      if (obstacleFootprintBlockedAtBody(0.0, 0.0, yaw, true) ||
          (p_.traversabilityNearFieldStop && p_.useTraversabilityCost &&
           traversabilityFootprintRiskAtBody(0.0, 0.0, yaw, true) >=
               static_cast<float>(p_.traversabilityHardCost))) {
        return true;
      }
    }
    return false;
  }

  float traversabilityRiskAtWorld(double wx, double wy) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_ == nullptr || traversabilityRows_ <= 0 ||
        traversabilityCols_ <= 0 || traversabilityResolution_ <= 0.0) {
      return 0.0f;
    }
    const int col =
        static_cast<int>(std::floor((wx - traversabilityOriginX_) / traversabilityResolution_));
    const int row =
        static_cast<int>(std::floor((wy - traversabilityOriginY_) / traversabilityResolution_));
    if (row < 0 || row >= traversabilityRows_ || col < 0 || col >= traversabilityCols_) {
      return 100.0f;
    }
    float risk = traversabilityGrid_[row * traversabilityCols_ + col];
    if (!std::isfinite(risk))
      return 100.0f;
    return std::clamp(risk, 0.0f, 100.0f);
  }

  float traversabilityRiskAtBody(double bx, double by) const {
    double wx = vx_ + bx * cosYaw_ - by * sinYaw_;
    double wy = vy_ + bx * sinYaw_ + by * cosYaw_;
    return traversabilityRiskAtWorld(wx, wy);
  }

  static bool footprintIntersectsGridCell(double centerX, double centerY, double c, double s,
                                          double halfLength, double halfWidth, double cellX,
                                          double cellY, double cellHalf) {
    constexpr double kOverlapEpsilon = 1e-9;
    const double dx = cellX - centerX;
    const double dy = cellY - centerY;
    const double absC = std::fabs(c);
    const double absS = std::fabs(s);
    if (std::fabs(dx) >= cellHalf + halfLength * absC + halfWidth * absS - kOverlapEpsilon) {
      return false;
    }
    if (std::fabs(dy) >= cellHalf + halfLength * absS + halfWidth * absC - kOverlapEpsilon) {
      return false;
    }
    if (std::fabs(dx * c + dy * s) >= halfLength + cellHalf * (absC + absS) - kOverlapEpsilon) {
      return false;
    }
    if (std::fabs(-dx * s + dy * c) >= halfWidth + cellHalf * (absS + absC) - kOverlapEpsilon) {
      return false;
    }
    return true;
  }

  bool gridCellFullyInsideCurrentFootprint(double cellX, double cellY, double cellHalf) const {
    const double dx = cellX - vx_;
    const double dy = cellY - vy_;
    const double localX = dx * cosYaw_ + dy * sinYaw_;
    const double localY = -dx * sinYaw_ + dy * cosYaw_;
    const double projectedCellHalf = cellHalf * (std::fabs(cosYaw_) + std::fabs(sinYaw_));
    constexpr double kOverlapEpsilon = 1e-9;
    return std::fabs(localX) + projectedCellHalf <= footprintHalfLength() + kOverlapEpsilon &&
           std::fabs(localY) + projectedCellHalf <= footprintHalfWidth() + kOverlapEpsilon;
  }

  bool traversabilityFootprintCoveredAtBody(double centerBx, double centerBy,
                                            double yawBody) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_ == nullptr || traversabilityRows_ <= 0 ||
        traversabilityCols_ <= 0 || traversabilityResolution_ <= 0.0 || !std::isfinite(centerBx) ||
        !std::isfinite(centerBy) || !std::isfinite(yawBody)) {
      return false;
    }
    constexpr double kOverlapEpsilon = 1e-9;
    const double centerX = vx_ + centerBx * cosYaw_ - centerBy * sinYaw_;
    const double centerY = vy_ + centerBx * sinYaw_ + centerBy * cosYaw_;
    const double yawWorld = vyaw_ + yawBody;
    const double c = std::cos(yawWorld);
    const double s = std::sin(yawWorld);
    const double halfLength = footprintHalfLength();
    const double halfWidth = footprintHalfWidth();
    const double extentX = halfLength * std::fabs(c) + halfWidth * std::fabs(s);
    const double extentY = halfLength * std::fabs(s) + halfWidth * std::fabs(c);
    const double gridMaxX = traversabilityOriginX_ +
                            static_cast<double>(traversabilityCols_) * traversabilityResolution_;
    const double gridMaxY = traversabilityOriginY_ +
                            static_cast<double>(traversabilityRows_) * traversabilityResolution_;
    return centerX - extentX >= traversabilityOriginX_ - kOverlapEpsilon &&
           centerY - extentY >= traversabilityOriginY_ - kOverlapEpsilon &&
           centerX + extentX <= gridMaxX + kOverlapEpsilon &&
           centerY + extentY <= gridMaxY + kOverlapEpsilon;
  }

  FootprintPenetration traversabilityCellPenetrationAtBody(
      double centerBx, double centerBy, double yawBody, double cellX, double cellY,
      double cellHalf) const {
    const double centerX = vx_ + centerBx * cosYaw_ - centerBy * sinYaw_;
    const double centerY = vy_ + centerBx * sinYaw_ + centerBy * cosYaw_;
    const double yawWorld = vyaw_ + yawBody;
    const double c = std::cos(yawWorld);
    const double s = std::sin(yawWorld);
    const double dx = cellX - centerX;
    const double dy = cellY - centerY;
    const double localX = c * dx + s * dy;
    const double localY = -s * dx + c * dy;
    const double projectedCellHalf = cellHalf * (std::fabs(c) + std::fabs(s));
    return {footprintHalfLength() + projectedCellHalf - std::fabs(localX),
            footprintHalfWidth() + projectedCellHalf - std::fabs(localY)};
  }

  float traversabilityFootprintRiskAtBody(double centerBx, double centerBy, double yawBody,
                                          bool allowInitialEscape = false) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_ == nullptr || traversabilityRows_ <= 0 ||
        traversabilityCols_ <= 0 || traversabilityResolution_ <= 0.0 || !std::isfinite(centerBx) ||
        !std::isfinite(centerBy) || !std::isfinite(yawBody)) {
      return 100.0f;
    }
    constexpr double kOverlapEpsilon = 1e-9;
    const double centerX = vx_ + centerBx * cosYaw_ - centerBy * sinYaw_;
    const double centerY = vy_ + centerBx * sinYaw_ + centerBy * cosYaw_;
    const double yawWorld = vyaw_ + yawBody;
    const double c = std::cos(yawWorld);
    const double s = std::sin(yawWorld);
    const double halfLength = footprintHalfLength();
    const double halfWidth = footprintHalfWidth();
    const double extentX = halfLength * std::fabs(c) + halfWidth * std::fabs(s);
    const double extentY = halfLength * std::fabs(s) + halfWidth * std::fabs(c);
    const double minX = centerX - extentX;
    const double maxX = centerX + extentX;
    const double minY = centerY - extentY;
    const double maxY = centerY + extentY;
    const double gridMaxX = traversabilityOriginX_ +
                            static_cast<double>(traversabilityCols_) * traversabilityResolution_;
    const double gridMaxY = traversabilityOriginY_ +
                            static_cast<double>(traversabilityRows_) * traversabilityResolution_;
    if (minX < traversabilityOriginX_ - kOverlapEpsilon ||
        minY < traversabilityOriginY_ - kOverlapEpsilon || maxX > gridMaxX + kOverlapEpsilon ||
        maxY > gridMaxY + kOverlapEpsilon) {
      return 100.0f;
    }

    const int minCol = std::max(0, static_cast<int>(std::floor((minX - traversabilityOriginX_) /
                                                               traversabilityResolution_)));
    const int maxCol =
        std::min(traversabilityCols_ - 1,
                 static_cast<int>(std::floor((maxX - traversabilityOriginX_ - kOverlapEpsilon) /
                                             traversabilityResolution_)));
    const int minRow = std::max(0, static_cast<int>(std::floor((minY - traversabilityOriginY_) /
                                                               traversabilityResolution_)));
    const int maxRow =
        std::min(traversabilityRows_ - 1,
                 static_cast<int>(std::floor((maxY - traversabilityOriginY_ - kOverlapEpsilon) /
                                             traversabilityResolution_)));
    const double cellHalf = traversabilityResolution_ * 0.5;
    float maxRisk = 0.0f;
    bool sampled = false;
    for (int row = minRow; row <= maxRow; ++row) {
      const double cellY =
          traversabilityOriginY_ + (static_cast<double>(row) + 0.5) * traversabilityResolution_;
      for (int col = minCol; col <= maxCol; ++col) {
        const double cellX =
            traversabilityOriginX_ + (static_cast<double>(col) + 0.5) * traversabilityResolution_;
        if (!footprintIntersectsGridCell(centerX, centerY, c, s, halfLength, halfWidth, cellX,
                                         cellY, cellHalf)) {
          continue;
        }
        sampled = true;
        if (gridCellFullyInsideCurrentFootprint(cellX, cellY, cellHalf)) {
          continue;
        }
        const float risk = traversabilityGrid_[row * traversabilityCols_ + col];
        if (!std::isfinite(risk) || risk < 0.0f || risk > 100.0f) {
          return 100.0f;
        }
        if (allowInitialEscape && risk >= p_.traversabilityHardCost) {
          constexpr double kEscapeEpsilon = 1e-6;
          const FootprintPenetration initial =
              traversabilityCellPenetrationAtBody(0.0, 0.0, 0.0, cellX, cellY, cellHalf);
          const FootprintPenetration penetration = traversabilityCellPenetrationAtBody(
              centerBx, centerBy, yawBody, cellX, cellY, cellHalf);
          if (initial.longitudinal >= -kEscapeEpsilon &&
              initial.lateral >= -kEscapeEpsilon &&
              overlapDepth(penetration) <= overlapDepth(initial) + kEscapeEpsilon) {
            continue;
          }
        }
        maxRisk = std::max(maxRisk, risk);
        if (maxRisk >= p_.traversabilityHardCost)
          return maxRisk;
      }
    }
    return sampled ? maxRisk : 100.0f;
  }

  bool checkNearFieldTraversability(double intentDirRad) const {
    if (!p_.checkObstacle || !p_.useTraversabilityCost || traversabilityGrid_ == nullptr ||
        traversabilityResolution_ <= 0.0) {
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

    const int minCol =
        std::max(0, static_cast<int>(std::floor((minWorldX - traversabilityOriginX_) /
                                                traversabilityResolution_)));
    const int maxCol = std::min(traversabilityCols_ - 1,
                                static_cast<int>(std::floor((maxWorldX - traversabilityOriginX_) /
                                                            traversabilityResolution_)));
    const int minRow =
        std::max(0, static_cast<int>(std::floor((minWorldY - traversabilityOriginY_) /
                                                traversabilityResolution_)));
    const int maxRow = std::min(traversabilityRows_ - 1,
                                static_cast<int>(std::floor((maxWorldY - traversabilityOriginY_) /
                                                            traversabilityResolution_)));
    const double cellInflation = traversabilityResolution_ * 0.5;

    for (int row = minRow; row <= maxRow; ++row) {
      const double wy =
          traversabilityOriginY_ + (static_cast<double>(row) + 0.5) * traversabilityResolution_;
      for (int col = minCol; col <= maxCol; ++col) {
        const float risk = traversabilityGrid_[row * traversabilityCols_ + col];
        if (!std::isfinite(risk) || risk < static_cast<float>(p_.traversabilityHardCost)) {
          continue;
        }
        const double wx =
            traversabilityOriginX_ + (static_cast<double>(col) + 0.5) * traversabilityResolution_;
        const double dx = wx - vx_;
        const double dy = wy - vy_;
        const float bx = static_cast<float>(dx * cosYaw_ + dy * sinYaw_);
        const float by = static_cast<float>(-dx * sinYaw_ + dy * cosYaw_);
        if (insideFootprint(bx, by))
          continue;
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

  double footprintFrontEdge() const { return footprintHalfLength(); }

  double collisionCloudRange() const {
    const double planningRange =
        std::isfinite(p_.adjacentRange) ? std::max(0.1, p_.adjacentRange) : 0.1;
    const double footprintRadius = std::hypot(footprintHalfLength(), footprintHalfWidth());
    return std::isfinite(footprintRadius) ? planningRange + footprintRadius : planningRange;
  }

  bool insideFootprint(float bx, float by) const {
    return std::fabs(bx) <= static_cast<float>(footprintHalfLength()) &&
           std::fabs(by) <= static_cast<float>(footprintHalfWidth());
  }

  bool insideSelfFilterFootprint(float bx, float by) const {
    const double padding = std::max(0.0, p_.selfFilterPadding);
    const double halfLength = std::max(0.0, p_.vehicleLength * 0.5) + padding;
    const double halfWidth = std::max(0.0, p_.vehicleWidth * 0.5) + padding;
    return std::fabs(bx) <= static_cast<float>(halfLength) &&
           std::fabs(by) <= static_cast<float>(halfWidth);
  }

  bool heightInsideBodyEnvelope(float relative_height) const {
    return std::isfinite(relative_height) &&
           relative_height >= static_cast<float>(p_.obstacleHeightThre) &&
           relative_height <= static_cast<float>(p_.obstacleHeightMax);
  }

  bool obstacleHeightBlocks(float height) const {
    return !p_.useTerrainAnalysis || height >= static_cast<float>(p_.obstacleHeightThre);
  }

  void rebuildObstacleIndex() {
    obstacleIndexHeads_.assign(kObstacleIndexCellsPerAxis * kObstacleIndexCellsPerAxis, -1);
    obstacleIndexNext_.assign(static_cast<std::size_t>(cloud_.size), -1);
    const double range = collisionCloudRange();
    obstacleIndexOrigin_ = -range;
    obstacleIndexResolution_ = (2.0 * range) / static_cast<double>(kObstacleIndexCellsPerAxis);
    for (int index = 0; index < cloud_.size; ++index) {
      if (!obstacleHeightBlocks(cloud_.h[index]))
        continue;
      const int col = static_cast<int>(
          std::floor((static_cast<double>(cloud_.x[index]) - obstacleIndexOrigin_) /
                     obstacleIndexResolution_));
      const int row = static_cast<int>(
          std::floor((static_cast<double>(cloud_.y[index]) - obstacleIndexOrigin_) /
                     obstacleIndexResolution_));
      if (row < 0 || row >= kObstacleIndexCellsPerAxis || col < 0 ||
          col >= kObstacleIndexCellsPerAxis) {
        continue;
      }
      const int cell = row * kObstacleIndexCellsPerAxis + col;
      obstacleIndexNext_[static_cast<std::size_t>(index)] =
          obstacleIndexHeads_[static_cast<std::size_t>(cell)];
      obstacleIndexHeads_[static_cast<std::size_t>(cell)] = index;
    }
  }

  void buildPlannerCloud(const float *pts, int n) {
    cloud_.clear();
    cloud_.reserve(n);
    const double collisionRange = collisionCloudRange();
    const float adjRangeSq = static_cast<float>(collisionRange * collisionRange);
    float cosY = static_cast<float>(cosYaw_), sinY = static_cast<float>(sinYaw_);
    float fx = static_cast<float>(vx_), fy = static_cast<float>(vy_);
    bool useTerrain = p_.useTerrainAnalysis;

    // Two-pass: 1) gather dx/dy into temp SoA, 2) SIMD batch disSq + rotate
    // For small clouds, scalar is fine. For large clouds, avoid AoS stride-4 penalty.
    if (n >= 256 && useTerrain) {
      // Phase 1: gather from AoS to SoA and apply the range filter.
      bpcDx_.resize(n);
      bpcDy_.resize(n);
      bpcH_.resize(n);
      int kept = 0;
      for (int i = 0; i < n; i++) {
        float dx = pts[i * 4] - fx, dy = pts[i * 4 + 1] - fy;
        if (dx * dx + dy * dy >= adjRangeSq)
          continue;
        if (!heightInsideBodyEnvelope(pts[i * 4 + 3]))
          continue;
        bpcDx_[kept] = dx;
        bpcDy_[kept] = dy;
        bpcH_[kept] = pts[i * 4 + 3];
        kept++;
      }
      // Phase 2: SIMD batch rotation on the kept points
      cloud_.x.resize(kept);
      cloud_.y.resize(kept);
      cloud_.h.resize(kept);
      cloud_.size = kept;
      simd::rotateCloud(bpcDx_.data(), bpcDy_.data(), cosY, sinY, cloud_.x.data(), cloud_.y.data(),
                        kept);
      std::memcpy(cloud_.h.data(), bpcH_.data(), kept * sizeof(float));
      int admitted = 0;
      for (int i = 0; i < kept; ++i) {
        if (insideSelfFilterFootprint(cloud_.x[i], cloud_.y[i]))
          continue;
        cloud_.x[admitted] = cloud_.x[i];
        cloud_.y[admitted] = cloud_.y[i];
        cloud_.h[admitted] = cloud_.h[i];
        ++admitted;
      }
      cloud_.x.resize(admitted);
      cloud_.y.resize(admitted);
      cloud_.h.resize(admitted);
      cloud_.size = admitted;
    } else {
      for (int i = 0; i < n; i++) {
        float px = pts[i * 4], py = pts[i * 4 + 1], h = pts[i * 4 + 3];
        float dx = px - fx, dy = py - fy;
        if (dx * dx + dy * dy >= adjRangeSq)
          continue;
        if (!heightInsideBodyEnvelope(h))
          continue;
        float bx = dx * cosY + dy * sinY;
        float by = -dx * sinY + dy * cosY;
        if (insideSelfFilterFootprint(bx, by))
          continue;
        cloud_.push(bx, by, h);
      }
    }
    rebuildObstacleIndex();
  }

  // Scratch buffers for buildPlannerCloud SIMD path
  std::vector<float> bpcDx_, bpcDy_, bpcH_;

  int scoreAndSelect(double pathScale, double pathRange, double relGoalDis, double joyDir,
                     double joySpeed, double dirThre, bool hardPathDirectionLimit, int &slowDown) {
    const auto &lut = rotLUT();
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
      if (a > 180.0f)
        a = 360.0f - a;
      float rotDeg = 10.0f * d;
      bool skip =
          (a > dirThre && !p_.dirToVehicle) ||
          (std::fabs(lut.rotAngDeg[d]) > dirThre && std::fabs(joyDir) <= 90.0 && p_.dirToVehicle) ||
          ((rotDeg > dirThre && 360.0f - rotDeg > dirThre) && std::fabs(joyDir) > 90.0 &&
           p_.dirToVehicle);
      if (!skip) {
        validRotDirs_[nValidRotDirs_++] = d;
      }
    }

    // Voxel grid constants (precomputed as float for inner loop)
    float invPS = 1.0f / static_cast<float>(pathScale);
    float invGS = 1.0f / 0.02f;
    float offXH = 3.2f + 0.02f * 0.5f;
    float offYH = 4.5f + 0.02f * 0.5f;
    float scaleA = static_cast<float>(searchRadius_ / 4.5);
    float scaleB = 1.0f / 3.2f;
    float pathRangeScaleSq = static_cast<float>((pathRange / pathScale) * (pathRange / pathScale));
    float goalClearScaleSq = static_cast<float>(((relGoalDis + p_.goalClearRange) / pathScale) *
                                                ((relGoalDis + p_.goalClearRange) / pathScale));
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
      const float angOffset =
          std::atan2(halfWidScale, halfLenScale) * 180.0f / static_cast<float>(M_PI);
      for (int i = 0; i < cloudSize; i++) {
        const float x = cloud_.x[i] * invPS;
        const float y = cloud_.y[i] * invPS;
        const float disSq = x * x + y * y;
        const float h = cloud_.h[i];
        if (disSq < diameterScaleSq &&
            (std::fabs(x) > halfLenScale || std::fabs(y) > halfWidScale) &&
            (h >= obsThre || !useTerrain)) {
          const float angObs = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
          if (angObs > 0) {
            if (minObsAngCCW > angObs - angOffset)
              minObsAngCCW = angObs - angOffset;
            if (minObsAngCW < angObs + angOffset - 180.0f)
              minObsAngCW = angObs + angOffset - 180.0f;
          } else {
            if (minObsAngCW < angObs + angOffset)
              minObsAngCW = angObs + angOffset;
            if (minObsAngCCW > 180.0f + angObs - angOffset)
              minObsAngCCW = 180.0f + angObs - angOffset;
          }
        }
      }
      if (minObsAngCW > 0)
        minObsAngCW = 0;
      if (minObsAngCCW < 0)
        minObsAngCCW = 0;
    }
    lastMinObstacleAngleCw_ = minObsAngCW;
    lastMinObstacleAngleCcw_ = minObsAngCCW;

    // Score obstacles by rotation, using batched SIMD transforms.
    if (p_.checkObstacle && cloudSize > 0) {
      const float *ch = cloud_.h.data();

      // Pre-scale cloud by invPS once (reused across all rotations)
      scaledX_.resize(cloudSize);
      scaledY_.resize(cloudSize);
      disSqBuf_.resize(cloudSize);
      {
        const float *cx = cloud_.x.data();
        const float *cy = cloud_.y.data();
        float *sx = scaledX_.data();
        float *sy = scaledY_.data();
        for (int i = 0; i < cloudSize; i++) {
          sx[i] = cx[i] * invPS;
          sy[i] = cy[i] * invPS;
        }
        simd::distSqBatch(sx, sy, disSqBuf_.data(), cloudSize);
      }

      // Per-rotation scoring lambda (each rotation writes to non-overlapping
      // clearPathList_[kPathNum*d..kPathNum*(d+1)], so parallel-safe)
      const float *sxp = scaledX_.data();
      const float *syp = scaledY_.data();
      const float *dsqp = disSqBuf_.data();
      const int *corrOff = corrOffset_.data();
      const int *corrDat = corrData_.data();
      int *clearArr = clearPathList_.data();
      float *penArr = pathPenaltyList_.data();
      int gvnx = gridVoxelNumX_, gvny = gridVoxelNumY_;

      auto scoreRotation = [&](int d, float *rxBuf, float *ryBuf) {
        float cosD = static_cast<float>(lut.c[d]);
        float sinD = static_cast<float>(lut.s[d]);
        int base = kPathNum * d;

        // SIMD batch rotation
        simd::rotateCloud(sxp, syp, cosD, sinD, rxBuf, ryBuf, cloudSize);

        // Voxel lookup (sequential, cache-friendly CSR)
        for (int i = 0; i < cloudSize; i++) {
          float disSq = dsqp[i];
          if (disSq >= pathRangeScaleSq)
            continue;
          if (cropByGoal && disSq > goalClearScaleSq)
            continue;

          float x2 = rxBuf[i];
          float y2 = ryBuf[i];
          float sy = x2 * scaleB + scaleA * (3.2f - x2) * scaleB;
          if (std::fabs(sy) < 1e-6f)
            continue;

          int indX = static_cast<int>((offXH - x2) * invGS);
          int indY = static_cast<int>((offYH - y2 / sy) * invGS);
          if (indX < 0 || indX >= gvnx || indY < 0 || indY >= gvny)
            continue;

          float h = ch[i];
          int ind = gvny * indX + indY;
// Prefetch next iteration's CSR offset (reduces cache miss stalls)
#if defined(__GNUC__) || defined(__clang__)
          if (i + 1 < cloudSize)
            __builtin_prefetch(corrOff + ind + 2, 0, 1);
#endif
          const int *pb = corrDat + corrOff[ind];
          const int *pe = corrDat + corrOff[ind + 1];
          for (const int *pp = pb; pp != pe; ++pp) {
            int idx = base + *pp;
            if (h >= obsThre || !useTerrain) {
              clearArr[idx]++;
            } else if (useCost && h > gndThre) {
              if (penArr[idx] < h)
                penArr[idx] = h;
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
#pragma omp parallel for schedule(static) num_threads(scoringThreads) if (nValidRotDirs_ >= 6)
        for (int vi = 0; vi < nValidRotDirs_; vi++) {
          float *rxBuf = parRotBufs_.data() + vi * 2 * cloudSize;
          float *ryBuf = rxBuf + cloudSize;
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

    // Aggregate with nested loops and a shared direction-difference term.
    PathScoreParams sp;
    sp.dirWeight = p_.dirWeight;
    sp.slopeWeight = p_.slopeWeight;
    sp.omniDirGoalThre = p_.omniDirGoalThre;
    std::vector<float> outputEndDirectionCache(totalGroups, 1000.0f);
    std::vector<float> outputLookaheadDirectionCache(totalGroups, 1000.0f);

    for (int vi = 0; vi < nValidRotDirs_; vi++) {
      int rotDir = validRotDirs_[vi];
      int pathBase = kPathNum * rotDir;
      int groupBase = kGroupNum * rotDir;
      double rotW4 = lut.rotDirW4[rotDir];
      float rotAng = lut.rotAngDeg[rotDir];

      for (int pathIdx = 0; pathIdx < kPathNum; pathIdx++) {
        int flatIdx = pathBase + pathIdx;
        if (clearPathList_[flatIdx] >= pptThre)
          continue;

        // angDiffDeg hoisted: only rotAng changes per outer loop
        double dirDiff = angDiffDeg(joyDir, static_cast<double>(endDirPathList_[pathIdx]) + rotAng);
        if (hardPathDirectionLimit && dirDiff > dirThre)
          continue;
        int grp = pathList_[pathIdx];
        int riskIdx = groupBase + grp;
        if (hardPathDirectionLimit) {
          float &outputEndDirection = outputEndDirectionCache[riskIdx];
          if (outputEndDirection > 360.0f) {
            outputEndDirection = static_cast<float>(
                outputPathEndDirectionDeg(rotDir, grp, pathScale, pathRange, relGoalDis));
          }
          if (angDiffDeg(joyDir, outputEndDirection) > dirThre)
            continue;
          float &outputLookaheadDirection = outputLookaheadDirectionCache[riskIdx];
          if (outputLookaheadDirection > 360.0f) {
            outputLookaheadDirection = static_cast<float>(outputPathLookaheadDirectionDeg(
                rotDir, grp, pathScale, pathRange, relGoalDis));
          }
          if (angDiffDeg(joyDir, outputLookaheadDirection) > dirThre)
            continue;
        }
        double grpW2 = lut.groupDirW2[grp];
        double score =
            scorePathFast(dirDiff, rotW4, grpW2, pathPenaltyList_[flatIdx], relGoalDis, sp, lut);
        if (score > 0) {
          int groupIdx = groupBase + grp;
          clearPathPerGroupScore_[groupIdx] += score;
          clearPathPerGroupNum_[groupIdx]++;
          pathPenaltyPerGroupScore_[groupIdx] += pathPenaltyList_[flatIdx];
        }
      }
    }

    // Select the best group lazily. The correspondence table is an efficient
    // broad phase, but the emitted group path is not one of its individual
    // member paths. Validate that exact output with the same rectangular
    // footprint configured for this robot before declaring it feasible.
    int selectedGroupID = selectBestGroup(clearPathPerGroupScore_, kGroupNum, minObsAngCW,
                                          minObsAngCCW, p_.twoWayDrive, p_.checkRotObstacle)
                              .selectedGroupID;
    if (selectedGroupID >= 0 && (p_.checkObstacle || p_.useTraversabilityCost)) {
      std::vector<std::int8_t> evaluatedFootprintCollision(totalGroups, -1);
      std::vector<float> evaluatedTerrainRisk(totalGroups, -1.0f);
      std::vector<bool> terrainScoreAdjusted(totalGroups, false);
      bool candidateAccepted = false;
      for (int attempt = 0; attempt < totalGroups; ++attempt) {
        selectedGroupID = selectBestGroup(clearPathPerGroupScore_, kGroupNum, minObsAngCW,
                                          minObsAngCCW, p_.twoWayDrive, p_.checkRotObstacle)
                              .selectedGroupID;
        if (selectedGroupID < 0)
          break;
        if (p_.checkObstacle) {
          auto &collision = evaluatedFootprintCollision[selectedGroupID];
          if (collision < 0) {
            collision = obstacleFootprintBlockedForGroup(selectedGroupID / kGroupNum,
                                                         selectedGroupID % kGroupNum, pathScale,
                                                         pathRange, relGoalDis)
                            ? 1
                            : 0;
          }
          if (collision != 0) {
            clearPathPerGroupScore_[selectedGroupID] = 0.0;
            continue;
          }
        }
        if (!p_.useTraversabilityCost) {
          candidateAccepted = true;
          break;
        }
        float &selectedRisk = evaluatedTerrainRisk[selectedGroupID];
        if (selectedRisk < 0.0f) {
          selectedRisk =
              traversabilityRiskForGroup(selectedGroupID / kGroupNum, selectedGroupID % kGroupNum,
                                         pathScale, pathRange, relGoalDis);
        }
        if (selectedRisk >= static_cast<float>(p_.traversabilityHardCost)) {
          clearPathPerGroupScore_[selectedGroupID] = 0.0;
          continue;
        }
        if (selectedRisk > static_cast<float>(p_.traversabilitySoftCost) &&
            !terrainScoreAdjusted[selectedGroupID]) {
          const double penalty =
              std::clamp(1.0 - p_.traversabilityWeight *
                                   (static_cast<double>(selectedRisk) - p_.traversabilitySoftCost),
                         0.0, 1.0);
          clearPathPerGroupScore_[selectedGroupID] *= penalty;
          terrainScoreAdjusted[selectedGroupID] = true;
          continue;
        }
        candidateAccepted = true;
        break;
      }
      if (!candidateAccepted) {
        selectedGroupID = -1;
      }
    }

    // Compute slow-down
    if (selectedGroupID >= 0) {
      int num = clearPathPerGroupNum_[selectedGroupID];
      float penaltyScore = (num > 0) ? pathPenaltyPerGroupScore_[selectedGroupID] / num : 0;
      if (penaltyScore > p_.costHeightThre1)
        slowDown = 1;
      else if (penaltyScore > p_.costHeightThre2)
        slowDown = 2;
      else if (num < p_.slowPathNumThre && std::abs(selectedGroupID - 129) > p_.slowGroupNumThre)
        slowDown = 3;
      else
        slowDown = 0;

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

  std::vector<Vec3> buildDebugCandidatePath(int rotDir, int groupId, double pathScale,
                                            double pathRange, double relGoalDis) const {
    constexpr std::size_t kDebugPointLimit = 16;
    std::vector<Vec3> full;
    if (rotDir < 0 || rotDir >= kRotDirs || groupId < 0 || groupId >= kGroupNum) {
      return full;
    }
    const auto &segment = startPaths_[groupId];
    if (segment.empty()) {
      return full;
    }
    const auto &lut = rotLUT();
    const double rc = lut.c[rotDir];
    const double rs = lut.s[rotDir];
    const double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    const double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);
    full.reserve(segment.size());
    for (const auto &point : segment) {
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
      const std::size_t source = index * (full.size() - 1) / (kDebugPointLimit - 1);
      sampled.push_back(full[source]);
    }
    return sampled;
  }

  void traceDebugCandidateGates(LocalPlanCandidate &candidate, double pathScale, double pathRange,
                                double relGoalDis, double joyDir, double dirThre,
                                bool hardPathDirectionLimit) const {
    const int rotDir = candidate.rotationIndex;
    const int groupId = candidate.groupId;
    const int pathBase = rotDir * kPathNum;
    const auto &lut = rotLUT();
    PathScoreParams scoreParams;
    scoreParams.dirWeight = p_.dirWeight;
    scoreParams.slopeWeight = p_.slopeWeight;
    scoreParams.omniDirGoalThre = p_.omniDirGoalThre;
    const double rotWeight4 = lut.rotDirW4[rotDir];
    const double groupWeight2 = lut.groupDirW2[groupId];
    const float rotAngle = lut.rotAngDeg[rotDir];
    double outputEndDirection = 1000.0;
    double outputLookaheadDirection = 1000.0;

    candidate.rotationAllowed =
        rotationPassesObstacleGate(rotDir, lastMinObstacleAngleCw_, lastMinObstacleAngleCcw_,
                                   p_.twoWayDrive, p_.checkRotObstacle);
    const bool outputFootprintBlocked =
        obstacleFootprintBlockedForGroup(rotDir, groupId, pathScale, pathRange, relGoalDis);
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

      const double directionDifference =
          angDiffDeg(joyDir, static_cast<double>(endDirPathList_[pathIdx]) + rotAngle);
      if (hardPathDirectionLimit && directionDifference > dirThre) {
        continue;
      }
      if (hardPathDirectionLimit) {
        if (outputEndDirection > 360.0) {
          outputEndDirection =
              outputPathEndDirectionDeg(rotDir, groupId, pathScale, pathRange, relGoalDis);
        }
        if (angDiffDeg(joyDir, outputEndDirection) > dirThre) {
          continue;
        }
        if (outputLookaheadDirection > 360.0) {
          outputLookaheadDirection = outputPathLookaheadDirectionDeg(
              rotDir, groupId, pathScale, pathRange, relGoalDis);
        }
        if (angDiffDeg(joyDir, outputLookaheadDirection) > dirThre) {
          continue;
        }
      }
      ++candidate.directionAllowedPathCount;

      if (candidate.terrainRisk < 0.0) {
        candidate.terrainRisk =
            traversabilityRiskForGroup(rotDir, groupId, pathScale, pathRange, relGoalDis);
      }
      if (p_.useTraversabilityCost && candidate.terrainRisk >= p_.traversabilityHardCost) {
        continue;
      }
      ++candidate.terrainAllowedPathCount;

      const double directionScore = scorePathFast(directionDifference, rotWeight4, groupWeight2,
                                                  0.0f, relGoalDis, scoreParams, lut);
      if (directionScore <= 0.0) {
        continue;
      }
      ++candidate.directionScoredPathCount;

      double score = scorePathFast(directionDifference, rotWeight4, groupWeight2,
                                   pathPenaltyList_[flatPath], relGoalDis, scoreParams, lut);
      if (score <= 0.0) {
        continue;
      }
      ++candidate.heightCostAllowedPathCount;

      if (p_.useTraversabilityCost && candidate.terrainRisk > p_.traversabilitySoftCost) {
        const double penalty = std::clamp(
            1.0 - p_.traversabilityWeight * (candidate.terrainRisk - p_.traversabilitySoftCost),
            0.0, 1.0);
        if (penalty < 1.0) {
          ++candidate.terrainSoftPenalizedPathCount;
        }
        score *= penalty;
      }
      if (score > 0.0) {
        ++candidate.contributingPathCount;
      }
    }

    if (outputFootprintBlocked) {
      candidate.collisionFreePathCount = 0;
    }

    if (!candidate.rotationAllowed) {
      candidate.state = LocalCandidateState::RotationBlocked;
    } else if (candidate.totalPathCount > 0 && candidate.collisionFreePathCount == 0) {
      candidate.state = LocalCandidateState::CollisionBlocked;
    } else if (candidate.directionAllowedPathCount == 0) {
      candidate.state = LocalCandidateState::DirectionRejected;
    } else if (candidate.terrainAllowedPathCount == 0) {
      candidate.state = LocalCandidateState::TerrainBlocked;
    } else if (candidate.directionScoredPathCount == 0) {
      candidate.state = LocalCandidateState::DirectionRejected;
    } else if (candidate.heightCostAllowedPathCount == 0 || candidate.contributingPathCount == 0) {
      candidate.state = LocalCandidateState::TerrainBlocked;
    } else if (candidate.terrainSoftPenalizedPathCount > 0) {
      candidate.state = LocalCandidateState::TerrainCost;
    } else {
      candidate.state = LocalCandidateState::Feasible;
    }
  }

  void captureDebugSnapshot(int selectedGroupId, double pathScale, double pathRange,
                            double relGoalDis, double joyDir, double dirThre,
                            bool hardPathDirectionLimit, double timestamp) {
    debugSnapshot_ = {};
    debugSnapshot_.valid = true;
    debugSnapshot_.backend = LocalPlannerBackend::Cmu;
    debugSnapshot_.timestampS = timestamp;
    debugSnapshot_.searchReason =
        selectedGroupId >= 0 ? "candidate_selected" : "no_scored_candidate";
    debugSnapshot_.collisionPointCount = cloud_.size;
    debugSnapshot_.pathScale = pathScale;
    debugSnapshot_.pathRange = pathRange;
    debugSnapshot_.relativeGoalDistanceM = relGoalDis;
    debugSnapshot_.traversabilitySoftCost = p_.traversabilitySoftCost;
    debugSnapshot_.traversabilityHardCost = p_.traversabilityHardCost;
    debugSnapshot_.validRotationCount = nValidRotDirs_;
    debugSnapshot_.selectedGroupId = selectedGroupId;

    if (p_.debugCandidateLimit <= 0) {
      return;
    }

    const int selectedRotation = selectedGroupId >= 0 ? selectedGroupId / kGroupNum : -1;
    const int selectedGroup = selectedGroupId >= 0 ? selectedGroupId % kGroupNum : -1;
    const std::size_t limit =
        static_cast<std::size_t>(std::clamp(p_.debugCandidateLimit, 0, kRotDirs));
    if (limit == 0) {
      return;
    }

    std::vector<int> sampledValidIndices;
    sampledValidIndices.reserve(limit);
    if (static_cast<std::size_t>(nValidRotDirs_) <= limit) {
      for (int validIndex = 0; validIndex < nValidRotDirs_; ++validIndex) {
        sampledValidIndices.push_back(validIndex);
      }
    } else {
      for (std::size_t index = 0; index < limit; ++index) {
        const std::size_t source =
            limit == 1 ? static_cast<std::size_t>(nValidRotDirs_) / 2
                       : index * (static_cast<std::size_t>(nValidRotDirs_) - 1) / (limit - 1);
        sampledValidIndices.push_back(static_cast<int>(source));
      }
      if (selectedRotation >= 0) {
        const auto selectedIt = std::find(validRotDirs_.begin(),
                                          validRotDirs_.begin() + nValidRotDirs_, selectedRotation);
        if (selectedIt != validRotDirs_.begin() + nValidRotDirs_) {
          const int selectedValidIndex =
              static_cast<int>(std::distance(validRotDirs_.begin(), selectedIt));
          if (std::find(sampledValidIndices.begin(), sampledValidIndices.end(),
                        selectedValidIndex) == sampledValidIndices.end()) {
            sampledValidIndices.back() = selectedValidIndex;
            std::sort(sampledValidIndices.begin(), sampledValidIndices.end());
          }
        }
      }
    }

    std::vector<LocalPlanCandidate> candidates;
    candidates.reserve(sampledValidIndices.size());
    const auto &lut = rotLUT();
    for (const int validIndex : sampledValidIndices) {
      const int rotDir = validRotDirs_[validIndex];
      int representativeGroup = 3;
      double bestScore = -1.0;
      int bestCollisionFree = -1;
      for (int groupId = 0; groupId < kGroupNum; ++groupId) {
        const int flatGroup = rotDir * kGroupNum + groupId;
        const double score = clearPathPerGroupScore_[flatGroup];
        const int collisionFree = collisionFreePathCount(rotDir, groupId);
        if (score > bestScore + 1e-12 ||
            (std::abs(score - bestScore) <= 1e-12 && collisionFree > bestCollisionFree) ||
            (std::abs(score - bestScore) <= 1e-12 && collisionFree == bestCollisionFree &&
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
      traceDebugCandidateGates(candidate, pathScale, pathRange, relGoalDis, joyDir, dirThre,
                               hardPathDirectionLimit);
      candidate.path =
          buildDebugCandidatePath(rotDir, representativeGroup, pathScale, pathRange, relGoalDis);
      candidates.push_back(std::move(candidate));
    }
    debugSnapshot_.candidates = std::move(candidates);
  }

  double outputPathEndDirectionDeg(int rotDir, int groupID, double pathScale, double pathRange,
                                   double relGoalDis) const {
    if (groupID < 0 || groupID >= kGroupNum)
      return 0.0;
    const auto &segment = startPaths_[groupID];
    if (segment.size() < 2)
      return rotLUT().rotAngDeg[rotDir];

    const double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    const double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);
    double previousX = 0.0;
    double previousY = 0.0;
    double lastX = 0.0;
    double lastY = 0.0;
    int pointCount = 0;
    for (const auto &point : segment) {
      const double distanceSq = point.x * point.x + point.y * point.y;
      if (distanceSq > pathRangeScaleSq || distanceSq > relGoalScaledSq)
        break;
      previousX = lastX;
      previousY = lastY;
      lastX = point.x;
      lastY = point.y;
      ++pointCount;
    }
    if (pointCount < 2)
      return rotLUT().rotAngDeg[rotDir];

    const double dx = lastX - previousX;
    const double dy = lastY - previousY;
    const auto &lut = rotLUT();
    const double rotatedDx = lut.c[rotDir] * dx - lut.s[rotDir] * dy;
    const double rotatedDy = lut.s[rotDir] * dx + lut.c[rotDir] * dy;
    return std::atan2(rotatedDy, rotatedDx) * 180.0 / M_PI;
  }

  double outputPathLookaheadDirectionDeg(int rotDir, int groupID, double pathScale,
                                         double pathRange, double relGoalDis) const {
    if (rotDir < 0 || rotDir >= kRotDirs || groupID < 0 || groupID >= kGroupNum)
      return 0.0;
    const auto &segment = startPaths_[groupID];
    if (segment.empty())
      return rotLUT().rotAngDeg[rotDir];

    const double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    const double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);
    const auto &lut = rotLUT();
    double lastBx = 0.0;
    double lastBy = 0.0;
    for (const auto &point : segment) {
      const double distanceSq = point.x * point.x + point.y * point.y;
      if (distanceSq > pathRangeScaleSq || distanceSq > relGoalScaledSq)
        break;
      lastBx = pathScale * (lut.c[rotDir] * point.x - lut.s[rotDir] * point.y);
      lastBy = pathScale * (lut.s[rotDir] * point.x + lut.c[rotDir] * point.y);
      if (std::hypot(lastBx, lastBy) >= kCmuFollowerLookAheadM)
        break;
    }
    if (std::hypot(lastBx, lastBy) <= 1e-9)
      return rotLUT().rotAngDeg[rotDir];
    return std::atan2(lastBy, lastBx) * 180.0 / M_PI;
  }

  FootprintPenetration obstacleFootprintPenetrationAtBody(
      int index, double centerBx, double centerBy, double yawBody) const {
    const double c = std::cos(yawBody);
    const double s = std::sin(yawBody);
    const double dx = static_cast<double>(cloud_.x[index]) - centerBx;
    const double dy = static_cast<double>(cloud_.y[index]) - centerBy;
    const double localX = dx * c + dy * s;
    const double localY = -dx * s + dy * c;
    return {footprintHalfLength() - std::fabs(localX),
            footprintHalfWidth() - std::fabs(localY)};
  }

  bool obstacleFootprintBlockedAtBody(double centerBx, double centerBy, double yawBody,
                                      bool allowInitialEscape = false) const {
    if (!p_.checkObstacle || cloud_.size <= 0 || obstacleIndexHeads_.empty() ||
        obstacleIndexResolution_ <= 0.0) {
      return false;
    }
    if (!std::isfinite(centerBx) || !std::isfinite(centerBy) || !std::isfinite(yawBody)) {
      return true;
    }

    const double c = std::cos(yawBody);
    const double s = std::sin(yawBody);
    const double halfLength = footprintHalfLength();
    const double halfWidth = footprintHalfWidth();
    if (!std::isfinite(halfLength) || !std::isfinite(halfWidth)) {
      return true;
    }
    const double extentX = halfLength * std::fabs(c) + halfWidth * std::fabs(s);
    const double extentY = halfLength * std::fabs(s) + halfWidth * std::fabs(c);
    const double indexMax =
        obstacleIndexOrigin_ + obstacleIndexResolution_ * kObstacleIndexCellsPerAxis;
    if (centerBx + extentX < obstacleIndexOrigin_ || centerBy + extentY < obstacleIndexOrigin_ ||
        centerBx - extentX >= indexMax || centerBy - extentY >= indexMax) {
      return false;
    }
    const auto cellFor = [&](double coordinate) {
      return std::clamp(static_cast<int>(std::floor((coordinate - obstacleIndexOrigin_) /
                                                    obstacleIndexResolution_)),
                        0, kObstacleIndexCellsPerAxis - 1);
    };
    const int minCol = cellFor(centerBx - extentX);
    const int maxCol = cellFor(centerBx + extentX);
    const int minRow = cellFor(centerBy - extentY);
    const int maxRow = cellFor(centerBy + extentY);
    for (int row = minRow; row <= maxRow; ++row) {
      for (int col = minCol; col <= maxCol; ++col) {
        int index =
            obstacleIndexHeads_[static_cast<std::size_t>(row * kObstacleIndexCellsPerAxis + col)];
        while (index >= 0) {
          const FootprintPenetration penetration =
              obstacleFootprintPenetrationAtBody(index, centerBx, centerBy, yawBody);
          if (penetration.longitudinal >= 0.0 && penetration.lateral >= 0.0) {
            if (allowInitialEscape) {
              constexpr double kEscapeEpsilon = 1e-6;
              const FootprintPenetration initial =
                  obstacleFootprintPenetrationAtBody(index, 0.0, 0.0, 0.0);
              if (initial.longitudinal >= -kEscapeEpsilon &&
                  initial.lateral >= -kEscapeEpsilon &&
                  overlapDepth(penetration) <= overlapDepth(initial) + kEscapeEpsilon) {
                index = obstacleIndexNext_[static_cast<std::size_t>(index)];
                continue;
              }
            }
            return true;
          }
          index = obstacleIndexNext_[static_cast<std::size_t>(index)];
        }
      }
    }
    return false;
  }

  bool obstacleFootprintBlockedForGroup(int rotDir, int groupID, double pathScale, double pathRange,
                                        double relGoalDis) const {
    if (!p_.checkObstacle || cloud_.size <= 0 || rotDir < 0 || rotDir >= kRotDirs || groupID < 0 ||
        groupID >= kGroupNum) {
      return false;
    }
    const auto &segment = startPaths_[groupID];
    if (segment.empty())
      return false;

    const double horizon = std::max(0.0, std::min(pathRange, relGoalDis));
    if (horizon <= 0.0)
      return false;
    const auto &lut = rotLUT();
    const double rc = lut.c[rotDir];
    const double rs = lut.s[rotDir];
    const double sampleStep =
        std::clamp(std::min(footprintHalfLength(), footprintHalfWidth()) * 0.25, 0.02, 0.05);
    double traveled = 0.0;
    double previousBx = 0.0;
    double previousBy = 0.0;
    for (const auto &point : segment) {
      const double targetBx = pathScale * (rc * point.x - rs * point.y);
      const double targetBy = pathScale * (rs * point.x + rc * point.y);
      double bx = targetBx;
      double by = targetBy;
      bool clippedToHorizon = false;
      if (std::hypot(targetBx, targetBy) > horizon + 1e-9) {
        const double targetDx = targetBx - previousBx;
        const double targetDy = targetBy - previousBy;
        const double a = targetDx * targetDx + targetDy * targetDy;
        if (a <= 1e-12)
          break;
        const double b = 2.0 * (previousBx * targetDx + previousBy * targetDy);
        const double c = previousBx * previousBx + previousBy * previousBy - horizon * horizon;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
          return true;
        const double alpha = std::clamp((-b + std::sqrt(discriminant)) / (2.0 * a), 0.0, 1.0);
        bx = previousBx + alpha * targetDx;
        by = previousBy + alpha * targetDy;
        clippedToHorizon = true;
      }

      const double dx = bx - previousBx;
      const double dy = by - previousBy;
      const double segmentLength = std::hypot(dx, dy);
      const double tangentYawBody = segmentLength > 1e-6 ? std::atan2(dy, dx) : std::atan2(by, bx);
      const int steps = std::max(1, static_cast<int>(std::ceil(segmentLength / sampleStep)));
      for (int step = 1; step <= steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(steps);
        const double sweptDistance = traveled + alpha * segmentLength;
        const double yawBody =
            sweptDistance <= std::max(0.0, p_.nearFieldStopDis) + 1e-9 ? 0.0 : tangentYawBody;
        if (obstacleFootprintBlockedAtBody(previousBx + dx * alpha, previousBy + dy * alpha,
                                           yawBody, true)) {
          return true;
        }
      }
      traveled += segmentLength;
      previousBx = bx;
      previousBy = by;
      if (clippedToHorizon)
        break;
    }
    return false;
  }

  float traversabilityRiskForGroup(int rotDir, int groupID, double pathScale, double pathRange,
                                   double relGoalDis) const {
    if (!p_.useTraversabilityCost || traversabilityGrid_ == nullptr || traversabilityRows_ <= 0 ||
        traversabilityCols_ <= 0 || traversabilityResolution_ <= 0.0 || groupID < 0 ||
        groupID >= kGroupNum) {
      return 0.0f;
    }
    const auto &seg = startPaths_[groupID];
    if (seg.empty())
      return 0.0f;
    if (!traversabilityFootprintCoveredAtBody(0.0, 0.0, 0.0)) {
      return 100.0f;
    }

    const auto &lut = rotLUT();
    const double rc = lut.c[rotDir];
    const double rs = lut.s[rotDir];
    const double horizon = std::max(0.0, std::min(pathRange, relGoalDis));
    float maxRisk = 0.0f;
    double traveled = 0.0;
    double previousBx = 0.0;
    double previousBy = 0.0;
    for (const auto &pt : seg) {
      const double targetBx = pathScale * (rc * pt.x - rs * pt.y);
      const double targetBy = pathScale * (rs * pt.x + rc * pt.y);
      double bx = targetBx;
      double by = targetBy;
      bool clippedToHorizon = false;
      if (std::hypot(targetBx, targetBy) > horizon + 1e-9) {
        const double targetDx = targetBx - previousBx;
        const double targetDy = targetBy - previousBy;
        const double a = targetDx * targetDx + targetDy * targetDy;
        if (a <= 1e-12 || horizon <= 0.0)
          break;
        const double b = 2.0 * (previousBx * targetDx + previousBy * targetDy);
        const double c = previousBx * previousBx + previousBy * previousBy - horizon * horizon;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
          return 100.0f;
        const double alpha = std::clamp((-b + std::sqrt(discriminant)) / (2.0 * a), 0.0, 1.0);
        bx = previousBx + alpha * targetDx;
        by = previousBy + alpha * targetDy;
        clippedToHorizon = true;
      }
      const double dx = bx - previousBx;
      const double dy = by - previousBy;
      const double segmentLength = std::hypot(dx, dy);
      const double tangentYawBody = segmentLength > 1e-6 ? std::atan2(dy, dx) : std::atan2(by, bx);
      const int steps =
          std::max(1, static_cast<int>(std::ceil(segmentLength /
                                                 std::max(0.05, traversabilityResolution_ * 0.5))));
      for (int step = 1; step <= steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(steps);
        const double sampleBx = previousBx + dx * alpha;
        const double sampleBy = previousBy + dy * alpha;
        const double sweptDistance = traveled + alpha * segmentLength;
        const double yawBody =
            sweptDistance <= std::max(0.0, p_.nearFieldStopDis) + 1e-9 ? 0.0 : tangentYawBody;
        float risk = traversabilityFootprintRiskAtBody(sampleBx, sampleBy, yawBody, true);
        if (risk > maxRisk)
          maxRisk = risk;
        if (maxRisk >= p_.traversabilityHardCost)
          return maxRisk;
      }
      traveled += segmentLength;
      previousBx = bx;
      previousBy = by;
      if (clippedToHorizon)
        break;
    }
    return std::clamp(maxRisk, 0.0f, 100.0f);
  }

  void buildOutputPath(int selectedGroupID, double pathScale, double pathRange, double relGoalDis,
                       std::vector<Vec3> &path) {
    const auto &lut = rotLUT();
    int rotDir = selectedGroupID / kGroupNum;
    int groupID = selectedGroupID % kGroupNum;
    double rc = lut.c[rotDir], rs = lut.s[rotDir];

    double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);

    auto &seg = startPaths_[groupID];
    path.clear();
    path.reserve(seg.size());

    for (auto &pt : seg) {
      double disSq = pt.x * pt.x + pt.y * pt.y;
      if (disSq > pathRangeScaleSq || disSq > relGoalScaledSq)
        break;
      // Rotate back to body frame, then scale
      double bx = pathScale * (rc * pt.x - rs * pt.y);
      double by = pathScale * (rs * pt.x + rc * pt.y);
      double bz = pathScale * pt.z;
      path.push_back({bx, by, bz});
    }
  }
};

const char *localCandidateStateName(LocalCandidateState state) {
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

local::cmu::Backend::Backend(const LocalPlannerParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

local::cmu::Backend::~Backend() = default;
local::cmu::Backend::Backend(local::cmu::Backend &&) noexcept = default;
local::cmu::Backend &local::cmu::Backend::operator=(local::cmu::Backend &&) noexcept = default;

bool local::cmu::Backend::loadPaths(const std::string &pathsDir) {
  return impl_->loadPaths(pathsDir);
}

bool local::cmu::Backend::pathsLoaded() const {
  return impl_->pathsLoaded();
}

LocalPlannerDebugSnapshot local::cmu::Backend::debugSnapshot() const {
  return impl_->debugSnapshot();
}

LocalPlan local::cmu::Backend::plan(const LocalPlanRequest &request) {
  return impl_->plan(request);
}

void local::cmu::Backend::reset() {
  impl_->reset();
}

const LocalPlannerParams &local::cmu::Backend::params() const {
  return impl_->params();
}

}  // namespace nav_kernel
