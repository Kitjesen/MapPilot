#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "dds/frame.hpp"
#include "command/goal.hpp"
#include "input/samples.hpp"
#include "messages.h"
#include "safety/command.hpp"
#include "nav_kernel/types.hpp"
#include "planning/local/planner.hpp"

namespace lingtu::nav::endpoint {

struct CommandIngressRequest;

template <typename T>
struct Decoded {
  T value{};
  std::string error;

  bool ok() const { return error.empty(); }
};

std::string headerFrameId(const lingtu_dds_Header &header);
std::string stringValue(const char *value);
double headerStampSeconds(const lingtu_dds_Header &header);
std::string textData(const lingtu_dds_Text &msg);
bool sourceStampPredates(double source_stamp_s, double not_before_s);
[[nodiscard]] CommandIngressRequest
commandIngressRequestFromDds(const lingtu_dds_NavigationCommandRequest &message);

InputSample<TransformSample> copyTransformSample(const lingtu_dds_TFMessage &message);
InputSample<OdometrySample> copyOdometrySample(const lingtu_dds_Odometry &message);
InputSample<PointCloudSample> copyPointCloudSample(const lingtu_dds_PointCloud2 &message,
                                                   bool terrain_height);
InputSample<GridSample> copyGridSample(const lingtu_dds_OccupancyGrid &message,
                                      const char *required_frame);
InputSample<LocalCollisionMap> copyLocalCollisionSample(
    const lingtu_dds_MapCollisionLayer &message);
DriverControlSample copyDriverControlSample(const lingtu_dds_DriverControlState &message);

nav_kernel::Pose toPose(const lingtu_dds_Odometry &msg);
Decoded<GoalTarget> decodeGoal(const lingtu_dds_PoseStamped &msg,
                               const std::optional<RigidTransform> &map_odom);
Decoded<nav_kernel::Twist> decodeTwist(const lingtu_dds_TwistStamped &msg);
Decoded<TraversabilityGrid> decodeGrid(const lingtu_dds_OccupancyGrid &msg);
Decoded<TraversabilityGrid> decodeLocalRiskGrid(const lingtu_dds_OccupancyGrid &msg);
Decoded<LocalCollisionMap> decodeLocalCollisionMap(
    const lingtu_dds_MapCollisionLayer &msg);

std::vector<float> cloudToXyzh(const lingtu_dds_PointCloud2 &msg, std::size_t max_points,
                               const std::optional<RigidTransform> &map_body,
                               const std::optional<RigidTransform> &map_odom);
std::vector<float> terrainCloudToXyzh(const lingtu_dds_PointCloud2 &msg,
                                      std::size_t max_points,
                                      const std::optional<RigidTransform> &map_body,
                                      const std::optional<RigidTransform> &map_odom);

}  // namespace lingtu::nav::endpoint
