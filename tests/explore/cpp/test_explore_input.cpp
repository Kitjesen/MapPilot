#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "endpoint/input_gate.hpp"
#include "endpoint/route.hpp"

namespace {

lingtu_dds_ExplorationGrid makeSnapshot(std::vector<std::uint8_t> &data, char *frame,
                                        char *session) {
  lingtu_dds_ExplorationGrid message{};
  message.header.stamp.sec = 100;
  message.header.frame_id = frame;
  message.info.resolution = 0.5F;
  message.info.width = 3U;
  message.info.height = 2U;
  message.info.origin.position.x = -1.0;
  message.info.origin.position.y = -2.0;
  message.info.origin.orientation.w = 1.0;
  message.data._buffer = data.data();
  message.data._length = static_cast<std::uint32_t>(data.size());
  message.data._maximum = static_cast<std::uint32_t>(data.size());
  message.session_id = session;
  message.map_content_epoch = 0;
  message.reset_epoch = 2U;
  message.generation = 7U;
  message.live = true;
  return message;
}

lingtu_dds_Odometry makeOdometry(char *frame, double stamp_s, char *child_frame = nullptr) {
  lingtu_dds_Odometry message{};
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  message.header.stamp.nanosec = static_cast<std::uint32_t>((stamp_s - std::floor(stamp_s)) * 1e9);
  message.header.frame_id = frame;
  message.child_frame_id = child_frame;
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = 2.0;
  message.pose.pose.orientation.w = 1.0;
  return message;
}

void testValidSnapshot() {
  std::vector<std::uint8_t> data{0U, 100U, 255U, 0U, 0U, 100U};
  char frame[] = "map";
  char session[] = "mapping-session";
  auto message = makeSnapshot(data, frame, session);
  std::string reason;
  const auto parsed = lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason);
  assert(parsed.has_value());
  assert(reason == "accepted");
  assert(parsed->grid.valid());
  assert(parsed->grid.cells[2] == lingtu::explore::kUnknown);
  assert(parsed->identity.generation == 7U);
}

void testStrictTrinaryAndIdentity() {
  std::vector<std::uint8_t> data{0U, 100U, 42U, 0U, 0U, 100U};
  char frame[] = "map";
  char session[] = "mapping-session";
  auto message = makeSnapshot(data, frame, session);
  std::string reason;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason));
  assert(reason == "snapshot_not_trinary");

  data[2] = 255U;
  message.session_id = nullptr;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason));
  assert(reason == "snapshot_identity_invalid");
}

void testSnapshotRequiresCanonicalRouteMapIdentity() {
  std::vector<std::uint8_t> data{0U, 100U, 255U, 0U, 0U, 100U};
  char frame[] = "map";
  char session[] = "mapping-session";
  char map_id[] = "map-a";
  auto message = makeSnapshot(data, frame, session);
  std::string reason;

  message.map_id = map_id;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason));
  assert(reason == "snapshot_route_map_identity_invalid");

  message.map_id = nullptr;
  message.map_content_epoch = 1;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason));
  assert(reason == "snapshot_route_map_identity_invalid");

  message.live = false;
  message.map_content_epoch = 0;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}, &reason));
  assert(reason == "snapshot_identity_invalid");
}

void testRejectRotatedGrid() {
  std::vector<std::uint8_t> data{0U, 100U, 255U, 0U, 0U, 100U};
  char frame[] = "map";
  char session[] = "mapping-session";
  auto message = makeSnapshot(data, frame, session);
  message.info.origin.orientation.z = 0.1;
  assert(!lingtu::nav::endpoint::parseExplorationSnapshot(message, {}));
}

void testFreshness() {
  using lingtu::nav::endpoint::sourceStampFresh;
  assert(sourceStampFresh(100.0, 100.4, 0.5, 0.1));
  assert(!sourceStampFresh(100.0, 100.6, 0.5, 0.1));
  assert(!sourceStampFresh(101.0, 100.0, 0.5, 0.1));
}

void testRouteContract() {
  using lingtu::nav::endpoint::isCanonicalExploreRouteMapIdentity;
  using lingtu::nav::endpoint::isCanonicalExploreSnapshotBinding;
  using lingtu::nav::endpoint::parseRoute;
  using lingtu::nav::endpoint::Route;
  using lingtu::nav::endpoint::routeName;
  using lingtu::nav::endpoint::usesLiveSegment;

  assert(parseRoute("map") == Route::Map);
  assert(parseRoute("live") == Route::Live);
  assert(!parseRoute("auto").has_value());
  assert(!parseRoute("").has_value());
  assert(routeName(Route::Map) == "map");
  assert(routeName(Route::Live) == "live");
  assert(!usesLiveSegment(Route::Map));
  assert(usesLiveSegment(Route::Live));
  assert(isCanonicalExploreRouteMapIdentity(Route::Live, "", 0));
  assert(!isCanonicalExploreRouteMapIdentity(Route::Live, "map-a", 0));
  assert(!isCanonicalExploreRouteMapIdentity(Route::Live, "", 1));
  assert(isCanonicalExploreRouteMapIdentity(Route::Map, "map-a", 1));
  assert(!isCanonicalExploreRouteMapIdentity(Route::Map, "", 1));
  assert(!isCanonicalExploreRouteMapIdentity(Route::Map, "map-a", 0));
  assert(isCanonicalExploreSnapshotBinding(Route::Live, "product-a", "", 0, "product-a", true,
                                           "", 0));
  assert(!isCanonicalExploreSnapshotBinding(Route::Live, "product-a", "", 0, "product-b", true,
                                            "", 0));
  assert(!isCanonicalExploreSnapshotBinding(Route::Live, "product-a", "", 0, "product-a", false,
                                            "", 0));
  assert(isCanonicalExploreSnapshotBinding(Route::Map, "product-a", "map-a", 1, "product-a",
                                           false, "map-a", 1));
  assert(!isCanonicalExploreSnapshotBinding(Route::Map, "product-a", "map-a", 1, "product-a",
                                            false, "map-b", 1));
  assert(!isCanonicalExploreSnapshotBinding(Route::Map, "product-a", "map-a", 1, "product-a",
                                            false, "map-a", 2));
}

void testMapFrameOdometry() {
  char map_frame[] = "map";
  char body_frame[] = "body";
  const auto message = makeOdometry(map_frame, 100.0, body_frame);
  std::string reason;
  const auto pose =
      lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {}, &reason);
  assert(pose.has_value());
  assert(reason == "accepted");
  assert(std::abs(pose->pose.x - 1.0) < 1e-9);
  assert(std::abs(pose->pose.y - 2.0) < 1e-9);
}

void testOdomFrameRequiresFreshTransform() {
  char odom_frame[] = "odom";
  char body_frame[] = "body";
  const auto message = makeOdometry(odom_frame, 100.0, body_frame);
  std::string reason;
  assert(
      !lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {}, &reason));
  assert(reason == "map_odom_transform_missing");

  lingtu::nav::endpoint::RigidTransform map_odom;
  map_odom.translation = {10.0, -2.0, 0.0};
  map_odom.rotation.w = 1.0;
  map_odom.stamp_s = 100.0;
  map_odom.valid = true;
  const auto pose =
      lingtu::nav::endpoint::mapPoseFromOdometry(message, map_odom, 100.05, 100.1, {}, &reason);
  assert(pose.has_value());
  assert(std::abs(pose->pose.x - 11.0) < 1e-9);
  assert(std::abs(pose->pose.y - 0.0) < 1e-9);

  assert(!lingtu::nav::endpoint::mapPoseFromOdometry(message, map_odom, 98.0, 100.1, {}, &reason));
  assert(reason == "map_odom_transform_stale");
}

void testOdometryRequiresCanonicalFramePair() {
  char map_frame[] = "map";
  char empty_frame[] = "";
  char body_frame[] = "body";
  char base_link_frame[] = "base_link";
  std::string reason;

  auto message = makeOdometry(empty_frame, 100.0, body_frame);
  assert(!lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {},
                                                     &reason));
  assert(reason == "odom_frame_empty");

  char sensor_frame[] = "camera";
  message = makeOdometry(sensor_frame, 100.0, body_frame);
  assert(!lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {},
                                                     &reason));
  assert(reason == "odom_frame_unsupported");

  message = makeOdometry(map_frame, 100.0, empty_frame);
  assert(!lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {},
                                                     &reason));
  assert(reason == "odom_child_frame_empty");

  message = makeOdometry(map_frame, 100.0, base_link_frame);
  assert(!lingtu::nav::endpoint::mapPoseFromOdometry(message, std::nullopt, -1.0, 100.1, {},
                                                     &reason));
  assert(reason == "odom_child_frame_unsupported");
}

}  // namespace

int main() {
  testValidSnapshot();
  testStrictTrinaryAndIdentity();
  testSnapshotRequiresCanonicalRouteMapIdentity();
  testRejectRotatedGrid();
  testFreshness();
  testRouteContract();
  testMapFrameOdometry();
  testOdomFrameRequiresFreshTransform();
  testOdometryRequiresCanonicalFramePair();
  std::cout << "test_explore_input passed\n";
  return 0;
}
