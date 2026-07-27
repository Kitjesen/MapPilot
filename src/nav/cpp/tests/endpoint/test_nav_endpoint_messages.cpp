#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_endpoint_messages.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testTerrainExtDefaultZeroShareDoesNotEnterPlannerObstacles() {
  const std::vector<float> ext_only{
      1.0f, 0.0f, 0.4f, -0.4f, 2.0f, 0.0f, 0.5f, 0.5f,
  };

  std::vector<float> out;
  lingtu::nav::endpoint::buildPlannerObstacleCloud(out, {}, {}, false, ext_only, true, 100, {});
  require(out.empty(), "terrain_map_ext default zero share must preserve prior behavior");
}

void testFreshTerrainExtEntersPlannerObstaclesWhenEnabled() {
  const std::vector<float> ext_only{
      1.0f, 0.0f, 0.4f, 0.4f, 2.0f, 0.0f, 0.5f, 0.5f,
  };
  lingtu::nav::endpoint::ObstacleMergeConfig config;
  config.registered_share = 0.0;
  config.terrain_share = 0.0;
  config.terrain_ext_share = 1.0;

  std::vector<float> out;
  lingtu::nav::endpoint::buildPlannerObstacleCloud(out, {}, {}, false, ext_only, true, 100, config);
  require(out.size() == ext_only.size(),
          "enabled fresh terrain_map_ext must enter planner obstacles");

  lingtu::nav::endpoint::buildPlannerObstacleCloud(out, {}, {}, false, ext_only, false, 100,
                                                   config);
  require(out.empty(), "stale terrain_map_ext must not enter planner obstacles");
}

void testCleanTerrainStillEntersPlannerObstacles() {
  const std::vector<float> terrain{
      1.0f,
      0.0f,
      0.4f,
      0.4f,
  };
  std::vector<float> out;
  lingtu::nav::endpoint::buildPlannerObstacleCloud(out, {}, terrain, true, {}, false, 100, {});
  require(out.size() == 4, "fresh clean terrain must remain a planner obstacle source");
}

lingtu_dds_Header header(const char *frame, double stamp = 10.0) {
  lingtu_dds_Header out{};
  out.frame_id = const_cast<char *>(frame);
  out.stamp.sec = static_cast<std::int32_t>(stamp);
  out.stamp.nanosec =
      static_cast<std::uint32_t>((stamp - static_cast<double>(out.stamp.sec)) * 1e9);
  return out;
}

lingtu::nav::endpoint::RigidTransform mapOdom() {
  lingtu::nav::endpoint::RigidTransform tf;
  tf.translation = {10.0, 20.0, 1.0};
  tf.rotation.w = 1.0;
  tf.valid = true;
  return tf;
}

void testCanonicalFrameDecoders() {
  std::vector<lingtu_dds_PoseStamped> poses(2);
  poses[0].pose.position.x = 1.0;
  poses[1].pose.position.x = 2.0;
  lingtu_dds_Path path_msg{};
  path_msg.header = header("odom");
  path_msg.poses._length = static_cast<std::uint32_t>(poses.size());
  path_msg.poses._maximum = path_msg.poses._length;
  path_msg.poses._buffer = poses.data();
  const auto path = lingtu::nav::endpoint::decodePath(path_msg, mapOdom());
  require(path.ok(), "odom path with map<-odom TF must decode");
  require(path.value.size() == 2, "decoded path size must be preserved");
  require(std::abs(path.value[0].x - 11.0) < 1e-9, "odom path must transform to map");

  lingtu_dds_PoseStamped goal_msg{};
  goal_msg.header = header("odom");
  goal_msg.pose.position.x = 3.0;
  goal_msg.pose.orientation.z = std::sin(0.25);
  goal_msg.pose.orientation.w = std::cos(0.25);
  const auto goal = lingtu::nav::endpoint::decodeGoal(goal_msg, mapOdom());
  require(goal.ok(), "odom goal with map<-odom TF must decode");
  require(std::abs(goal.value.position.x - 13.0) < 1e-9, "odom goal must transform to map");
  require(std::abs(goal.value.yaw - 0.5) < 1e-9, "goal yaw must be preserved");

  path_msg.header = header("camera_link");
  require(!lingtu::nav::endpoint::decodePath(path_msg, mapOdom()).ok(),
          "unsupported path frame must fail closed");

  std::vector<std::uint8_t> grid_data(4, 0);
  lingtu_dds_OccupancyGrid grid_msg{};
  grid_msg.header = header("odom");
  grid_msg.info.width = 2;
  grid_msg.info.height = 2;
  grid_msg.info.resolution = 0.2f;
  grid_msg.data._length = static_cast<std::uint32_t>(grid_data.size());
  grid_msg.data._maximum = grid_msg.data._length;
  grid_msg.data._buffer = grid_data.data();
  require(!lingtu::nav::endpoint::decodeGrid(grid_msg).ok(),
          "occupancy grids must require the canonical map frame");
  grid_msg.header = header("map");
  require(lingtu::nav::endpoint::decodeGrid(grid_msg).ok(), "map grid must decode");

  lingtu_dds_TwistStamped twist_msg{};
  twist_msg.header = header("map");
  require(!lingtu::nav::endpoint::decodeTwist(twist_msg).ok(),
          "teleop twist outside the body frame must fail closed");
  twist_msg.header = header("base_link");
  require(lingtu::nav::endpoint::decodeTwist(twist_msg).ok(), "body twist must decode");
}

void testPathEchoUsesStampAndContent() {
  const std::vector<nav_kernel::Vec3> own{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const std::vector<nav_kernel::Vec3> external{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  lingtu::nav::endpoint::PathEcho echo;
  echo.arm(own, 10.0);
  require(!echo.take(external, 10.0, 10.1), "same-size external path must not be suppressed");
  require(!echo.take(own, 11.0, 11.1), "different-stamp path must not be suppressed");
  require(echo.take(own, 10.0, 11.2), "matching self echo must be suppressed");
  require(!echo.take(own, 10.0, 11.3), "self echo must be consumed once");
}

void testSourceStampValidationRejectsReplayAndFutureCommands() {
  using lingtu::nav::endpoint::sourceStampError;
  require(sourceStampError("clear_estop", 10.0, 10.1, 0.35, 0.05).empty(),
          "fresh clear-estop timestamp must pass");
  require(sourceStampError("clear_estop", 0.0, 10.0, 0.35, 0.05) ==
              "clear_estop_source_stamp_invalid",
          "zero clear-estop timestamp must fail");
  require(sourceStampError("clear_estop", 10.2, 10.0, 0.35, 0.05) ==
              "clear_estop_source_stamp_future",
          "future clear-estop timestamp must fail");
  require(sourceStampError("clear_estop", 9.0, 10.0, 0.35, 0.05) ==
              "clear_estop_source_stamp_stale",
          "replayed clear-estop timestamp must fail");
}

void testResumeBoundaryRejectsOldMotionRequests() {
  using lingtu::nav::endpoint::sourceStampPredates;
  require(!sourceStampPredates(1.0, 0.0),
          "requests before the first takeover must not require a resume boundary");
  require(sourceStampPredates(0.0, 10.0), "unstamped requests must fail after a resume boundary");
  require(sourceStampPredates(9.0, 10.0), "pre-resume requests must be rejected");
  require(sourceStampPredates(10.0, 10.0),
          "requests must be strictly newer than the resume boundary");
  require(!sourceStampPredates(10.1, 10.0), "fresh post-resume requests must pass");
}

void testPointCloudLayoutValidation() {
  lingtu_dds_PointField fields[3]{};
  const char *names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    fields[i].name = const_cast<char *>(names[i]);
    fields[i].offset = static_cast<std::uint32_t>(i * 4);
    fields[i].datatype = 7;
    fields[i].count = 1;
  }
  const float xyz[3] = {1.0f, 2.0f, 3.0f};
  std::vector<std::uint8_t> data(sizeof(xyz));
  std::memcpy(data.data(), xyz, sizeof(xyz));
  lingtu_dds_PointCloud2 msg{};
  msg.header = header("map");
  msg.height = 1;
  msg.width = 1;
  msg.fields._length = 3;
  msg.fields._maximum = 3;
  msg.fields._buffer = fields;
  msg.point_step = 12;
  msg.row_step = 12;
  msg.data._length = static_cast<std::uint32_t>(data.size());
  msg.data._maximum = msg.data._length;
  msg.data._buffer = data.data();
  require(lingtu::nav::endpoint::cloudToXyzh(msg, 0, std::nullopt, std::nullopt).size() == 4,
          "valid float32 cloud must decode");

  fields[0].datatype = 2;
  require(lingtu::nav::endpoint::cloudToXyzh(msg, 0, std::nullopt, std::nullopt).empty(),
          "non-float coordinate field must be rejected");
  fields[0].datatype = 7;
  fields[2].offset = 10;
  require(lingtu::nav::endpoint::cloudToXyzh(msg, 0, std::nullopt, std::nullopt).empty(),
          "field crossing point_step must be rejected");
  fields[2].offset = 8;
  msg.is_bigendian = true;
  require(lingtu::nav::endpoint::cloudToXyzh(msg, 0, std::nullopt, std::nullopt).empty(),
          "unsupported big-endian cloud must be rejected");

  const float body_xyz[3] = {0.0f, 1.0f, 0.0f};
  std::memcpy(data.data(), body_xyz, sizeof(body_xyz));
  msg.header = header("body");
  msg.is_bigendian = false;
  lingtu::nav::endpoint::RigidTransform map_body;
  map_body.translation.z = 1.0;
  map_body.rotation.x = std::sin(M_PI * 0.25);
  map_body.rotation.w = std::cos(M_PI * 0.25);
  map_body.valid = true;
  const auto transformed = lingtu::nav::endpoint::cloudToXyzh(msg, 0, map_body, std::nullopt);
  require(transformed.size() == 4, "body-frame cloud must decode with a full transform");
  require(std::abs(transformed[1]) < 1e-5, "body roll must rotate y out of the map plane");
  require(std::abs(transformed[2] - 2.0) < 1e-5, "body roll must affect map z");
  require(std::abs(transformed[3] - 1.0) < 1e-5, "derived height must use transformed map z");
}

}  // namespace

int main() {
  testTerrainExtDefaultZeroShareDoesNotEnterPlannerObstacles();
  testFreshTerrainExtEntersPlannerObstaclesWhenEnabled();
  testCleanTerrainStillEntersPlannerObstacles();
  testCanonicalFrameDecoders();
  testPathEchoUsesStampAndContent();
  testSourceStampValidationRejectsReplayAndFutureCommands();
  testResumeBoundaryRejectsOldMotionRequests();
  testPointCloudLayoutValidation();
  std::cout << "test_nav_endpoint_messages passed\n";
  return 0;
}
