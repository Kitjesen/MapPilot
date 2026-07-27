#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "frame_transform.hpp"
#include "nav/cpp/planning/global/global_planner_contract.hpp"
#include "lingtu_slam.h"
#include "motion/teleop_safety.hpp"
#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

struct ObstacleMergeConfig {
  double voxel_size_m{0.08};
  double registered_share{0.55};
  double terrain_share{0.30};
  double terrain_ext_share{0.0};
};

struct GoalTarget {
  nav_kernel::Vec3 position{};
  double yaw{0.0};
};

template <typename T>
struct Decoded {
  T value{};
  std::string error;

  bool ok() const { return error.empty(); }
};

class PathEcho {
 public:
  void arm(const std::vector<nav_kernel::Vec3> &path, double stamp_s);
  bool take(const std::vector<nav_kernel::Vec3> &path, double stamp_s, double now_s);
  void reset();

 private:
  std::vector<nav_kernel::Vec3> path_;
  double stamp_s_{-1.0};
  bool armed_{false};
};

std::string headerFrameId(const lingtu_dds_Header &header);
double headerStampSeconds(const lingtu_dds_Header &header);
std::string textData(const lingtu_dds_Text &msg);
std::string sourceStampError(const std::string &prefix, double source_stamp_s, double receive_s,
                             double max_age_s, double future_tolerance_s);
bool sourceStampPredates(double source_stamp_s, double not_before_s);

nav_kernel::Pose toPose(const lingtu_dds_Odometry &msg);
std::vector<nav_kernel::Vec3> toPath(const lingtu_dds_Path &msg);
nav_kernel::Vec3 toGoalPoint(const lingtu_dds_PoseStamped &msg);
std::vector<nav_kernel::Vec3>
toNavPath(const std::vector<lingtu::nav::plan::GlobalPlanPoint> &path);
nav_kernel::Twist toTwist(const lingtu_dds_TwistStamped &msg);
TraversabilityGrid toTraversabilityGrid(const lingtu_dds_OccupancyGrid &msg);

Decoded<std::vector<nav_kernel::Vec3>> decodePath(const lingtu_dds_Path &msg,
                                                  const std::optional<RigidTransform> &map_odom);
Decoded<GoalTarget> decodeGoal(const lingtu_dds_PoseStamped &msg,
                               const std::optional<RigidTransform> &map_odom);
Decoded<nav_kernel::Twist> decodeTwist(const lingtu_dds_TwistStamped &msg);
Decoded<TraversabilityGrid> decodeGrid(const lingtu_dds_OccupancyGrid &msg);

double vecDistance(const nav_kernel::Vec3 &a, const nav_kernel::Vec3 &b);

std::vector<float> cloudToXyzh(const lingtu_dds_PointCloud2 &msg, std::size_t max_points,
                               const std::optional<RigidTransform> &map_body,
                               const std::optional<RigidTransform> &map_odom);

void buildPlannerObstacleCloud(std::vector<float> &out, const std::vector<float> &registered_xyzh,
                               const std::vector<float> &terrain_xyzh, bool terrain_map_fresh,
                               const std::vector<float> &terrain_ext_xyzh, bool terrain_ext_fresh,
                               std::size_t max_points, const ObstacleMergeConfig &config = {});

}  // namespace lingtu::nav::endpoint
