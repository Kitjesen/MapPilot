/**
 * local_planner/cpp/local_planner_scoring.hpp -- scoring helpers for the CMU local planner.
 *
 * Extracted from the old ROS localPlanner.cpp:
 *   - worldToVoxel(): project obstacle points into the path voxel grid.
 *   - scorePath(): direction/terrain/path-group scoring formula.
 *   - selectBestGroup(): choose the highest scoring direction group.
 *   - angDiffDeg(): angular distance in degrees.
 *
 * No ROS2 or PCL dependency.
 */
#pragma once

#include "nav_kernel/types.hpp"
#include <cmath>
#include <cstring>
#include <vector>
#include <array>

namespace nav_kernel {

// ── 36 方向旋转查找表 (localPlanner.cpp:48-58) ──

struct RotLUT {
  std::array<double, 36> s, c;
  std::array<double, 36> rotDirW;       // precomputed per-rotation weight
  std::array<double, 7>  groupDirW;     // precomputed per-group weight
  std::array<double, 36> rotDirW4;      // rotDirW^4 (normal mode)
  std::array<double, 7>  groupDirW2;    // groupDirW^2 (near-goal mode)
  std::array<float, 36>  rotAngDeg;     // 10*d - 180
  // pow(x, 0.25) LUT for dirWeight*dirDiff in [0, 3.6] (dirWeight=0.02, max dirDiff=180)
  // 360 entries at 0.01 resolution covers the full range.
  static constexpr int kPow025Size = 361;
  std::array<float, kPow025Size> pow025;  // pow025[i] = pow(i * 0.01, 0.25)

  RotLUT() {
    for (int i = 0; i < 36; i++) {
      double a = (10.0 * i - 180.0) * M_PI / 180.0;
      s[i] = std::sin(a);
      c[i] = std::cos(a);
      rotAngDeg[i] = 10.0f * i - 180.0f;
      // rotDirW (localPlanner.cpp:1132-1133)
      double w = (i < 18) ? std::fabs(std::fabs(i - 9.0) + 1.0)
                           : std::fabs(std::fabs(i - 27.0) + 1.0);
      rotDirW[i] = w;
      rotDirW4[i] = w * w * w * w;
    }
    for (int g = 0; g < 7; g++) {
      double w = 4.0 - std::fabs(g - 3.0);
      groupDirW[g] = w;
      groupDirW2[g] = w * w;
    }
    // pow(x, 0.25) LUT
    pow025[0] = 0.0f;
    for (int i = 1; i < kPow025Size; i++) {
      pow025[i] = static_cast<float>(std::sqrt(std::sqrt(i * 0.01)));
    }
  }
};

inline const RotLUT& rotLUT() {
  static const RotLUT lut;
  return lut;
}

// ── 角度差 (degree, 0~180) ──

inline double angDiffDeg(double a, double b) {
  double d = std::fmod(std::fabs(a - b), 360.0);
  if (d > 180.0) d = 360.0 - d;
  return d;
}

// ── 体素网格投影 (localPlanner.cpp:1071-1077) ──
// 将旋转后的 (x2, y2) 投影到体素索引 (indX, indY)
// 返回 false 表示超出网格范围

struct VoxelGridParams {
  double gridVoxelSize    = 0.02;
  double gridVoxelOffsetX = 3.2;
  double gridVoxelOffsetY = 4.5;
  double searchRadius     = 0.45;
  int    gridVoxelNumX    = 161;   // 默认值, 启动时从 paths 文件读取
  int    gridVoxelNumY    = 451;
};

inline bool worldToVoxel(double x2, double y2,
                         const VoxelGridParams& g,
                         int& indX, int& indY) {
  double invGridVoxelSize = 1.0 / g.gridVoxelSize;
  double offsetXHalf = g.gridVoxelOffsetX + g.gridVoxelSize * 0.5;
  double offsetYHalf = g.gridVoxelOffsetY + g.gridVoxelSize * 0.5;
  double scaleYCoefA = g.searchRadius / g.gridVoxelOffsetY;
  double scaleYCoefB = 1.0 / g.gridVoxelOffsetX;

  double scaleY = x2 * scaleYCoefB + scaleYCoefA * (g.gridVoxelOffsetX - x2) * scaleYCoefB;
  if (std::fabs(scaleY) < 1e-6) return false;

  indX = static_cast<int>((offsetXHalf - x2) * invGridVoxelSize);
  indY = static_cast<int>((offsetYHalf - y2 / scaleY) * invGridVoxelSize);
  return (indX >= 0 && indX < g.gridVoxelNumX &&
          indY >= 0 && indY < g.gridVoxelNumY);
}

// ── 路径评分公式 (localPlanner.cpp:1135-1145) ──
// dirDiff: 方向差 (degrees, 0~180)
// dirWeight: 方向惩罚系数 (默认 0.02)
// rotDirW: 旋转方向偏好权重 (1~10)
// groupDirW: 路径组方向权重 (1~4)
// terrainPenalty: 路径上最大地形高度代价
// slopeWeight: 坡度惩罚系数 (默认 0, 建议 3~6)
// relativeGoalDis: 到目标的距离
// omniDirGoalThre: 近目标阈值 (近目标时用 groupDirW 替代 rotDirW)

struct PathScoreParams {
  double dirWeight       = 0.02;
  double slopeWeight     = 0.0;
  double omniDirGoalThre = 5.0;
};

inline double scorePath(double dirDiffDeg,
                        double rotDirW,
                        double groupDirW,
                        double terrainPenalty,
                        double relativeGoalDis,
                        const PathScoreParams& p) {
  double dw = std::fabs(p.dirWeight * dirDiffDeg);
  double sqrtSqrtDw = std::sqrt(std::sqrt(dw));

  double terrainFactor = (p.slopeWeight > 0.0)
      ? std::max(0.0, 1.0 - p.slopeWeight * terrainPenalty)
      : 1.0;

  double score;
  if (relativeGoalDis < p.omniDirGoalThre) {
    score = (1.0 - sqrtSqrtDw) * groupDirW * groupDirW * terrainFactor;
  } else {
    double rotDirW2 = rotDirW * rotDirW;
    score = (1.0 - sqrtSqrtDw) * rotDirW2 * rotDirW2 * terrainFactor;
  }
  return score;
}

/// Fast scoring with precomputed LUT — avoids sqrt(sqrt()) per call.
/// rotDirW4: precomputed rotDirW^4,  groupDirW2: precomputed groupDirW^2.
inline double scorePathFast(double dirDiffDeg,
                            double rotDirW4_val,
                            double groupDirW2_val,
                            float  terrainPenalty,
                            double relativeGoalDis,
                            const PathScoreParams& p,
                            const RotLUT& lut) {
  float dw = static_cast<float>(std::fabs(p.dirWeight * dirDiffDeg));
  // LUT lookup: index = dw / 0.01, clamped to [0, 360]
  int idx = static_cast<int>(dw * 100.0f);
  if (idx >= RotLUT::kPow025Size) idx = RotLUT::kPow025Size - 1;
  float sqrtSqrtDw = lut.pow025[idx];

  double base = 1.0 - static_cast<double>(sqrtSqrtDw);
  if (base <= 0.0) return 0.0;

  double terrainFactor = (p.slopeWeight > 0.0)
      ? std::max(0.0, 1.0 - p.slopeWeight * terrainPenalty)
      : 1.0;

  return base * ((relativeGoalDis < p.omniDirGoalThre)
                 ? groupDirW2_val : rotDirW4_val) * terrainFactor;
}

// ── rotDirW 计算 (localPlanner.cpp:1132-1133) ──
// rotDir: 旋转方向索引 [0, 36)

inline double computeRotDirW(int rotDir) {
  if (rotDir < 18) return std::fabs(std::fabs(rotDir - 9.0) + 1.0);
  return std::fabs(std::fabs(rotDir - 27.0) + 1.0);
}

// ── groupDirW 计算 (localPlanner.cpp:1134) ──
// pathGroup: 路径组索引 [0, groupNum_)
// pathList_[pathIdx] 映射到组号

inline double computeGroupDirW(int pathGroup) {
  return 4.0 - std::fabs(pathGroup - 3.0);
}

// ── 方向组选择 (localPlanner.cpp:1155-1172) ──
// groupScores: 36*groupNum 个分数
// groupNum: 组数 (默认 7)
// minObsAngCW, minObsAngCCW: 旋转障碍物角度限制
// twoWayDrive, checkRotObstacle: 模式开关

struct GroupSelectionResult {
  int   selectedGroupID = -1;  // -1 = 没找到可行路径
  double maxScore        = 0;
};

inline GroupSelectionResult selectBestGroup(
    const std::vector<double>& groupScores,
    int groupNum,
    double minObsAngCW,
    double minObsAngCCW,
    bool twoWayDrive,
    bool checkRotObstacle) {
  GroupSelectionResult result;
  int total = static_cast<int>(groupScores.size());

  for (int i = 0; i < total; i++) {
    int rotDir = i / groupNum;
    double rotAngDeg = 10.0 * rotDir - 180.0;
    double rotDeg    = 10.0 * rotDir;
    if (rotDeg > 180.0) rotDeg -= 360.0;

    bool rotOk = (rotAngDeg > minObsAngCW && rotAngDeg < minObsAngCCW) ||
                 (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) ||
                 !checkRotObstacle;

    if (rotOk && groupScores[i] > result.maxScore) {
      result.maxScore = groupScores[i];
      result.selectedGroupID = i;
    }
  }
  return result;
}

}  // namespace nav_kernel
