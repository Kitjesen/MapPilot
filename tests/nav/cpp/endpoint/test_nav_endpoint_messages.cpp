#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dds/codec.hpp"
#include "input/planner.hpp"
#include "safety/command.hpp"
#include "status/nav_status_writer.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testCmuPlannerInputPrefersFreshTerrainAndFallsBackToRegisteredScan() {
  using namespace lingtu::nav::endpoint;
  TraversabilityGrid traversability;
  double traversability_received_s = 0.0;
  const std::vector<float> registered{1.0f, 0.0f, 0.2f, 0.2f};
  const std::vector<float> predicted{4.0f, 0.0f, 0.5f, 0.5f};
  const std::vector<float> terrain{2.0f, 0.0f, 0.3f, 0.3f};
  double terrain_received_s = 10.0;
  const std::vector<float> terrain_ext{3.0f, 0.0f, 0.4f, 0.4f};
  double terrain_ext_received_s = 10.0;
  std::vector<float> planner_obstacles;
  PlanData data{traversability,
                traversability_received_s,
                registered,
                predicted,
                terrain,
                terrain_received_s,
                terrain_ext,
                terrain_ext_received_s,
                planner_obstacles};
  PlanConfig config;
  config.check_obstacle = true;
  config.terrain_max_age_s = 1.0;
  config.max_obstacle_points = 100;
  config.use_terrain_cloud = true;
  TimingDiagnostics timing;

  const PlanView view = makePlanView(config, data, 10.1, timing, false);
  require(view.obstacles != nullptr, "CMU planner obstacle view must exist");
  require(view.terrain_selected, "fresh CMU terrain must replace the registered scan");
  const std::vector<float> expected_terrain{2.0f, 0.0f, 0.3f, 0.3f,
                                            4.0f, 0.0f, 0.5f, 0.5f};
  require(*view.obstacles == expected_terrain,
          "CMU terrain input must retain dynamic predicted obstacles");

  terrain_received_s = 8.0;
  const PlanView stale = makePlanView(config, data, 10.1, timing, false);
  const std::vector<float> expected_fallback{1.0f, 0.0f, 0.2f, 0.2f,
                                             4.0f, 0.0f, 0.5f, 0.5f};
  require(!stale.terrain_selected && *stale.obstacles == expected_fallback,
          "stale terrain must fall back to the registered scan");

  terrain_received_s = 10.0;
  config.use_terrain_cloud = false;
  const PlanView disabled = makePlanView(config, data, 10.1, timing, false);
  require(!disabled.terrain_selected && *disabled.obstacles == expected_fallback,
          "non-CMU planners must keep the registered scan input");
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
  lingtu_dds_PoseStamped goal_msg{};
  goal_msg.header = header("odom");
  goal_msg.pose.position.x = 3.0;
  goal_msg.pose.orientation.z = std::sin(0.25);
  goal_msg.pose.orientation.w = std::cos(0.25);
  const auto goal = lingtu::nav::endpoint::decodeGoal(goal_msg, mapOdom());
  require(goal.ok(), "odom goal with map<-odom TF must decode");
  require(std::abs(goal.value.position.x - 13.0) < 1e-9, "odom goal must transform to map");
  require(goal.value.yaw && std::abs(*goal.value.yaw - 0.5) < 1e-9,
          "goal yaw must be preserved");

  lingtu_dds_PoseStamped position_only_goal{};
  position_only_goal.header = header("map");
  position_only_goal.pose.position.x = 4.0;
  const auto position_only =
      lingtu::nav::endpoint::decodeGoal(position_only_goal, std::nullopt);
  require(position_only.ok(), "zero quaternion must encode a position-only goal");
  require(!position_only.value.yaw, "position-only goal must not request final yaw alignment");

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

  lingtu_dds_PointField terrain_fields[4]{};
  const char *terrain_names[4] = {"x", "y", "z", "intensity"};
  for (int index = 0; index < 4; ++index) {
    terrain_fields[index].name = const_cast<char *>(terrain_names[index]);
    terrain_fields[index].offset = static_cast<std::uint32_t>(index * 4);
    terrain_fields[index].datatype = 7;
    terrain_fields[index].count = 1;
  }
  const float terrain_xyzi[4] = {1.0f, 2.0f, 3.0f, 0.42f};
  std::vector<std::uint8_t> terrain_data(sizeof(terrain_xyzi));
  std::memcpy(terrain_data.data(), terrain_xyzi, sizeof(terrain_xyzi));
  lingtu_dds_PointCloud2 terrain_msg{};
  terrain_msg.header = header("map");
  terrain_msg.height = 1;
  terrain_msg.width = 1;
  terrain_msg.fields._length = 4;
  terrain_msg.fields._maximum = 4;
  terrain_msg.fields._buffer = terrain_fields;
  terrain_msg.point_step = 16;
  terrain_msg.row_step = 16;
  terrain_msg.data._length = static_cast<std::uint32_t>(terrain_data.size());
  terrain_msg.data._maximum = terrain_msg.data._length;
  terrain_msg.data._buffer = terrain_data.data();

  const auto ordinary =
      lingtu::nav::endpoint::cloudToXyzh(terrain_msg, 0, std::nullopt, std::nullopt);
  const auto terrain_height =
      lingtu::nav::endpoint::terrainCloudToXyzh(terrain_msg, 0, std::nullopt, std::nullopt);
  require(std::abs(ordinary[3] - 3.0f) < 1e-6f,
          "ordinary scan intensity must not become obstacle height");
  require(std::abs(terrain_height[3] - 0.42f) < 1e-6f,
          "CMU terrain intensity must decode as height above ground");
}

void testLocalCollisionLayerDecodeKeepsCompletenessAndIdentity() {
  std::vector<std::uint8_t> bits(50000U, 0U);
  bits[42] = 0x05U;
  lingtu_dds_MapCollisionLayer message{};
  message.header = header("map");
  message.reset_epoch = 4U;
  message.observation_sequence = 9U;
  message.generation = 12U;
  message.live = true;
  message.resolution = 0.10F;
  message.aabb_min.x = -5.0;
  message.aabb_min.y = -5.0;
  message.aabb_min.z = -2.0;
  message.aabb_max.x = 5.0;
  message.aabb_max.y = 5.0;
  message.aabb_max.z = 2.0;
  message.size_x = 100U;
  message.size_y = 100U;
  message.size_z = 40U;
  message.complete = false;
  message.inflated_occupied_bits._length = static_cast<std::uint32_t>(bits.size());
  message.inflated_occupied_bits._maximum = message.inflated_occupied_bits._length;
  message.inflated_occupied_bits._buffer = bits.data();

  auto decoded = lingtu::nav::endpoint::decodeLocalCollisionMap(message);
  require(decoded.ok(), "structurally valid incomplete collision layer must decode");
  require(decoded.value.inflated_occupied_bits &&
              *decoded.value.inflated_occupied_bits == bits,
          "collision decoder must copy the packed occupancy loan");
  const auto view = decoded.value.view();
  require(view.present(), "decoded collision view must be present");
  require(view.inflatedStorage == decoded.value.inflated_occupied_bits,
          "planner view must share the decoded collision bitmap");
  require(!view.complete, "wire completeness flag must survive the owning copy");
  require(view.resetEpoch == 4U && view.observationSequence == 9U &&
              view.generation == 12U,
          "collision identity must survive the owning copy");

  message.header = header("odom");
  decoded = lingtu::nav::endpoint::decodeLocalCollisionMap(message);
  require(!decoded.ok(), "collision payload outside map frame must fail closed");
}

}  // namespace

int main() {
  testCmuPlannerInputPrefersFreshTerrainAndFallsBackToRegisteredScan();
  testCanonicalFrameDecoders();
  testSourceStampValidationRejectsReplayAndFutureCommands();
  testResumeBoundaryRejectsOldMotionRequests();
  testPointCloudLayoutValidation();
  testLocalCollisionLayerDecodeKeepsCompletenessAndIdentity();
  std::cout << "test_nav_endpoint_messages passed\n";
  return 0;
}
