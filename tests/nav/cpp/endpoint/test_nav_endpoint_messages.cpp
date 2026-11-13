#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "dds/codec.hpp"
#include "collision/body.hpp"
#include "input/planner.hpp"
#include "planning/local/scan/grid.hpp"
#include "safety/command.hpp"
#include "status/nav_status_writer.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testLocalCollisionEvidenceRequiresFreshObservation() {
  using lingtu::nav::endpoint::localCollisionEvidence;
  nav_kernel::LocalPlannerDebugSnapshot debug;
  debug.collisionEvidence.emplace();
  auto &evidence = *debug.collisionEvidence;
  evidence.timestampS = 10.0;
  evidence.identity.frameEpoch = 1;
  evidence.collisionResetEpoch = 1;
  evidence.collisionObservationSequence = 2;
  evidence.collisionGeneration = 3;
  evidence.voxelResolution = .1;
  evidence.measuredPoints = std::make_shared<const std::vector<nav_kernel::Vec3>>(
      std::vector<nav_kernel::Vec3>{{1, 2, .4}});
  require(localCollisionEvidence(debug, 10.1, .35) == &evidence,
          "generic measured evidence must not depend on SCAN diagnostics");
  require(!localCollisionEvidence(debug, 10.4, .35), "old evidence must expire");
  debug.timestampS = 10.4;
  require(!localCollisionEvidence(debug, 10.4, .35), "status tick must not renew evidence");
  require(!localCollisionEvidence(debug, 9.9, .35), "future evidence must not guide replan");
  debug.collisionEvidence.reset();
  require(!localCollisionEvidence(debug, 10.1, .35), "absent evidence must not fabricate points");
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
  require(sourceStampError("goal", 10.0, 10.1, 0.35, 0.05).empty(),
          "fresh goal timestamp must pass");
  require(sourceStampError("goal", 0.0, 10.0, 0.35, 0.05) ==
              "goal_source_stamp_invalid",
          "zero goal timestamp must fail");
  require(sourceStampError("goal", 10.2, 10.0, 0.35, 0.05) ==
              "goal_source_stamp_future",
          "future goal timestamp must fail");
  require(sourceStampError("goal", 9.0, 10.0, 0.35, 0.05) ==
              "goal_source_stamp_stale",
          "replayed goal timestamp must fail");
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
  std::vector<std::uint8_t> measured(bits.size(), 0U), free(bits.size(), 0U);
  measured[7] = 0x20U;
  free[8] = 0x40U;
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
  message.measured_occupied_bits = message.inflated_occupied_bits;
  message.known_free_bits = message.inflated_occupied_bits;
  message.measured_occupied_bits._buffer = measured.data();
  message.known_free_bits._buffer = free.data();
  lingtu_dds_MapCollisionHitBounds hit{};
  hit.cell_index = 61U;
  hit.minimum = {-5.0 + 61.0 * message.resolution + .01, -4.99, -1.99};
  hit.maximum = {hit.minimum.x + .02, -4.97, -1.97};
  message.hit_bounds._length = message.hit_bounds._maximum = 1U;
  message.hit_bounds._buffer = &hit;

  auto decoded = lingtu::nav::endpoint::decodeLocalCollisionMap(message);
  require(decoded.ok(), "structurally valid incomplete collision layer must decode");
  require(decoded.value.inflated_occupied_bits &&
              *decoded.value.inflated_occupied_bits == bits,
          "collision decoder must copy the packed occupancy loan");
  const auto view = decoded.value.view();
  require(view.present(), "decoded collision view must be present");
  require(view.inflatedStorage == decoded.value.inflated_occupied_bits,
          "planner view must share the decoded collision bitmap");
  require(view.measuredOccupiedStorage == decoded.value.measured_occupied_bits &&
              view.knownFreeStorage == decoded.value.known_free_bits &&
              *view.measuredOccupiedStorage == measured && *view.knownFreeStorage == free,
          "3D evidence must survive the DDS loan and share lifetime with planner views");
  require(view.terrain != nullptr, "actual hit bounds must reach the planner view");
  hit.maximum.z = -1.5;
  require(!lingtu::nav::endpoint::decodeLocalCollisionMap(message).ok(),
          "hit evidence outside its occupancy cell must be rejected");
  hit.maximum.z = -1.97;
  message.known_free_bits._length = 0;
  require(!lingtu::nav::endpoint::decodeLocalCollisionMap(message).ok(),
          "missing ground evidence must reject the incomplete wire payload");
  message.known_free_bits._length = message.inflated_occupied_bits._length;
  require(decoded.value.occupied_cells_known && view.occupiedCount() == 2U,
          "collision occupancy count must be cached while the DDS loan is decoded");
  require(!view.complete, "wire completeness flag must survive the owning copy");
  require(view.resetEpoch == 4U && view.observationSequence == 9U &&
              view.generation == 12U,
          "collision identity must survive the owning copy");

  message.header = header("odom");
  decoded = lingtu::nav::endpoint::decodeLocalCollisionMap(message);
  require(!decoded.ok(), "collision payload outside map frame must fail closed");
}

void testCollisionHitBoundsReachTheRealLocalQuery() {
  constexpr int size = 40;
  constexpr double resolution = .05;
  const std::size_t cell_count = size * size * size;
  std::vector<std::uint8_t> occupied((cell_count + 7U) / 8U, 0U), free(occupied.size(), 0U);
  std::vector<lingtu_dds_MapCollisionHitBounds> hits;
  for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) {
    const auto index = static_cast<std::uint32_t>((10 * size + y) * size + x);
    occupied[index / 8U] |= static_cast<std::uint8_t>(1U << (index % 8U));
    lingtu_dds_MapCollisionHitBounds hit{};
    hit.cell_index = index;
    hit.minimum = {-1 + x * resolution + .01, -1 + y * resolution + .01, .002};
    hit.maximum = {-1 + x * resolution + .04, -1 + y * resolution + .04, .002};
    hits.push_back(hit);
  }
  lingtu_dds_MapCollisionLayer message{};
  message.header = header("map");
  message.reset_epoch = message.observation_sequence = message.generation = 1;
  message.live = message.complete = true;
  message.size_x = message.size_y = message.size_z = size;
  message.resolution = resolution;
  message.aabb_min = {-1,-1,-.5};
  message.aabb_max = {1,1,1.5};
  message.inflated_occupied_bits._length = message.inflated_occupied_bits._maximum =
      static_cast<std::uint32_t>(occupied.size());
  message.inflated_occupied_bits._buffer = occupied.data();
  message.measured_occupied_bits = message.inflated_occupied_bits;
  message.known_free_bits = message.inflated_occupied_bits;
  message.known_free_bits._buffer = free.data();
  message.hit_bounds._length = message.hit_bounds._maximum = static_cast<std::uint32_t>(hits.size());
  message.hit_bounds._buffer = hits.data();

  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  lingtu::nav::collision::Part foot;
  foot.shape = lingtu::nav::collision::Shape::Sphere;
  foot.radius_m = .1;
  foot.half_size = {.1,.1,.1};
  foot.center = {0,0,.136};
  foot.support_contact = true;
  body->parts.push_back(foot);
  nav_kernel::LocalPlannerParams params;
  params.scan.voxelResolution = resolution;
  nav_kernel::LocalPlanRequest request;
  request.body = body;
  auto check = [&] {
    const auto decoded = lingtu::nav::endpoint::decodeLocalCollisionMap(message);
    require(decoded.ok(), "hit bounds did not decode");
    request.environment.collision = decoded.value.view();
    const nav_kernel::local::scan::Grid grid(params, request);
    require(grid.valid(), "decoded map did not reach local collision query");
    return grid.inflatedOccupancy({}, 0);
  };
  require(check() == 0, "actual ground hits must not block the foot");
  hits[20 * size + 20].maximum.z = .045;
  require(check() != 0, "a real raised hit in the same cell must remain an obstacle");
  message.hit_bounds._length = 0;
  require(check() != 0, "a cell without hit evidence must retain ordinary collision");
}

}  // namespace

int main() {
  testLocalCollisionEvidenceRequiresFreshObservation();
  testCmuPlannerInputPrefersFreshTerrainAndFallsBackToRegisteredScan();
  testCanonicalFrameDecoders();
  testSourceStampValidationRejectsReplayAndFutureCommands();
  testResumeBoundaryRejectsOldMotionRequests();
  testPointCloudLayoutValidation();
  testLocalCollisionLayerDecodeKeepsCompletenessAndIdentity();
  testCollisionHitBoundsReachTheRealLocalQuery();
  std::cout << "test_nav_endpoint_messages passed\n";
  return 0;
}
