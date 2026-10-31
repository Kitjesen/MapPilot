#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "collision_bitmap.hpp"
#include "lingtu/maps/io/json.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "status/nav_status_writer.hpp"

namespace {
namespace collision = lingtu::nav::collision;
using Json = lingtu::maps::JsonValue;

const Json &field(const Json &json, const char *name) {
  return json.AsObject(name).at(name);
}

double number(const Json &json) { return std::get<double>(json.value); }
bool boolean(const Json &json) { return std::get<bool>(json.value); }

nav_kernel::Vec3 point(const Json &json) {
  const auto &values = json.AsArray("point");
  return {number(values.at(0)), number(values.at(1)), number(values.at(2))};
}

std::shared_ptr<const std::vector<std::uint8_t>> bitmap(const Json &json) {
  if (std::holds_alternative<std::nullptr_t>(json.value)) return {};
  const auto &hex = json.AsString("bitmap");
  auto bytes = std::make_shared<std::vector<std::uint8_t>>();
  for (std::size_t i = 0; i < hex.size(); i += 2U)
    bytes->push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2U), nullptr, 16)));
  return bytes;
}

std::shared_ptr<const collision::Body> body(const Json &json) {
  if (std::holds_alternative<std::nullptr_t>(json.value)) return {};
  auto result = std::make_shared<collision::Body>();
  result->margin_m = number(field(json, "margin_m"));
  result->horizontal_margin_m = number(field(json, "horizontal_margin_m"));
  for (const auto &value : field(json, "parts").AsArray("parts")) {
    collision::Part part;
    part.link = field(value, "link").AsString("link");
    const auto center = point(field(value, "center"));
    const auto half = point(field(value, "half_size"));
    part.center = {center.x, center.y, center.z};
    part.half_size = {half.x, half.y, half.z};
    const auto &rotation = field(value, "rotation").AsArray("rotation");
    for (std::size_t i = 0; i < part.rotation.size(); ++i)
      part.rotation[i] = number(rotation.at(i));
    const auto &shape = field(value, "shape").AsString("shape");
    if (shape == "box") part.shape = collision::Shape::Box;
    else if (shape == "sphere") part.shape = collision::Shape::Sphere;
    else if (shape == "cylinder") part.shape = collision::Shape::Cylinder;
    else if (shape == "capsule") part.shape = collision::Shape::Capsule;
    else throw std::invalid_argument("unknown body shape");
    part.radius_m = number(field(value, "radius_m"));
    part.half_length_m = number(field(value, "half_length_m"));
    part.support_contact = boolean(field(value, "support_contact"));
    result->parts.push_back(part);
  }
  return result;
}

struct FailureFixture {
  nav_kernel::LocalPlannerParams params;
  std::vector<nav_kernel::Vec3> route{{0, 0, .5}, {2, 0, .5}};
  std::vector<nav_kernel::PredictedObstacle> predictions{{{2, 2, .5}, {2.1, 2, .5}, .1, .1, .8}};
  lingtu::nav::tests::CollisionBitmap raw{{-3, -3, -1}, {3, 3, 2}, .1};
  nav_kernel::LocalPlanRequest request;

  FailureFixture() {
    params.backend = nav_kernel::LocalPlannerBackend::Scan;
    params.scan.voxelResolution = .1;
    params.scan.maxVelocity = .4;
    params.scan.maxAcceleration = 1;
    params.scan.cylinderOffset = 0;
    for (int y = -30; y < 30; ++y)
      for (int x = -30; x < 30; ++x)
        raw.occupy({(x + .5) * .1, (y + .5) * .1, -.05});
    raw.occupy({.05, .05, .55});

    auto model = std::make_shared<collision::Body>();
    model->margin_m = .012345678901234567;
    model->horizontal_margin_m = .023456789012345678;
    collision::Part part;
    part.link = "body\"frame";
    part.center = {0, 0, .05};
    part.half_size = {.2, .1, .1};
    const double angle = .12345678901234567;
    part.rotation = {std::cos(angle), -std::sin(angle), 0,
                     std::sin(angle), std::cos(angle), 0, 0, 0, 1};
    model->parts.push_back(part);
    for (auto shape : {collision::Shape::Sphere, collision::Shape::Cylinder,
                       collision::Shape::Capsule}) {
      part.link = "sensor";
      part.shape = shape;
      part.half_size = {.01, .01, .02};
      part.radius_m = .01;
      part.half_length_m = .02;
      model->parts.push_back(part);
    }
    collision::Part contact;
    contact.link = "foot";
    contact.shape = collision::Shape::Sphere;
    contact.center = {0, 0, -.45};
    contact.half_size = {.05, .05, .05};
    contact.radius_m = .05;
    contact.support_contact = true;
    model->parts.push_back(contact);
    auto observed = std::make_shared<collision::Body>(*model);
    observed->parts.front().center.y = .023456789012345678;
    auto &map = request.environment.collision;
    map = raw.view();
    map.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(
        map.inflatedBits, map.inflatedBits + map.inflatedBytes);
    auto free = std::make_shared<std::vector<std::uint8_t>>(*map.measuredOccupiedStorage);
    for (auto &byte : *free) byte = static_cast<std::uint8_t>(~byte);
    map.knownFreeStorage = free;
    map.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes, 0U);
    map.inflatedBits = map.inflatedStorage->data();
    map.terrain = std::make_shared<const collision::Terrain>(
        std::vector<collision::Aabb>{{{.05, .05, .55}, {.01, .02, .01}}}, .1);
    request.body = model;
    request.observedBody = observed;
    request.bodyHeading = 0;
    request.supportHeightM = .5;
    request.supportHeightToleranceM = .012345678901234567;
    request.robot.pose = {{0, 0, .5}, 0};
    request.objective = nav_kernel::RouteTarget{{route.data(), static_cast<int>(route.size()), 1}};
    request.clock = {1, false, nav_kernel::PlanClockMode::External};
    request.identity = {1, 1, 0};
    request.environment.predictions.obstacles = predictions.data();
    request.environment.predictions.count = predictions.size();
    request.environment.predictions.observedAtS = .9;
    request.environment.predictions.expiresAtS = 8.123456789012345;
  }

  std::shared_ptr<const nav_kernel::ScanFailureSnapshot> capture() {
    nav_kernel::local::scan::Backend backend(params);
    for (int tick = 0; tick < 20 && !backend.debugSnapshot().lastScanFailure; ++tick) {
      request.clock.timestampS += .01;
      backend.tick(request);
    }
    return backend.debugSnapshot().lastScanFailure;
  }
};

TEST(ScanFailureSnapshot, CapturesBodyAndRawEvidenceForIndependentGeometryReplay) {
  FailureFixture fixture;
  const auto failure = fixture.capture();
  ASSERT_TRUE(failure) << "fixture must reach the real Backend failure capture";
  ASSERT_FALSE(failure->attempt.success);
  const auto saved = lingtu::maps::ParseJson(
      lingtu::nav::endpoint::scanFailureSnapshotJson(*failure, "snapshot-test"));
  const auto &root = saved.AsObject("failure");
  ASSERT_NE(root.find("body"), root.end());
  ASSERT_NE(root.find("observed_body"), root.end());
  ASSERT_NE(root.find("support_height_m"), root.end());
  ASSERT_NE(root.find("support_height_tolerance_m"), root.end());

  nav_kernel::LocalPlanRequest replay;
  const auto &robot = field(saved, "robot");
  replay.robot.pose = {point(field(robot, "position")), number(field(robot, "yaw"))};
  replay.robot.kinematics.valid = boolean(field(robot, "kinematics_valid"));
  replay.robot.kinematics.linearVelocity = point(field(robot, "linear_velocity"));
  replay.robot.kinematics.linearAcceleration = point(field(robot, "linear_acceleration"));
  replay.robot.kinematics.yawRate = number(field(robot, "yaw_rate"));
  const auto &clock = field(saved, "clock");
  replay.clock.timestampS = number(field(clock, "timestamp_s"));
  replay.clock.executionFrozen = boolean(field(clock, "execution_frozen"));
  replay.clock.mode = field(clock, "mode").AsString("mode") == "external"
      ? nav_kernel::PlanClockMode::External : nav_kernel::PlanClockMode::Steady;
  replay.maxLinearSpeedMps = number(field(saved, "max_linear_speed_mps"));
  replay.bodyHeading = number(field(saved, "body_heading"));
  replay.body = body(field(saved, "body"));
  replay.observedBody = body(field(saved, "observed_body"));
  replay.supportHeightM = number(field(saved, "support_height_m"));
  replay.supportHeightToleranceM = number(field(saved, "support_height_tolerance_m"));
  const auto &map = field(saved, "collision_map");
  auto &collisionMap = replay.environment.collision;
  collisionMap.inflatedStorage = bitmap(field(map, "bits"));
  collisionMap.inflatedBits = collisionMap.inflatedStorage->data();
  collisionMap.inflatedBytes = collisionMap.inflatedStorage->size();
  collisionMap.measuredOccupiedStorage = bitmap(field(map, "measured_occupied_bits"));
  collisionMap.knownFreeStorage = bitmap(field(map, "known_free_bits"));
  const auto &size = field(map, "size").AsArray("map size");
  collisionMap.sizeX = static_cast<int>(number(size.at(0)));
  collisionMap.sizeY = static_cast<int>(number(size.at(1)));
  collisionMap.sizeZ = static_cast<int>(number(size.at(2)));
  collisionMap.resolution = number(field(map, "resolution"));
  std::vector<collision::Aabb> hits;
  for (const auto &value : field(map, "hit_bounds").AsArray("hit bounds")) {
    const auto center = point(field(value, "center"));
    const auto half = point(field(value, "half_size"));
    hits.push_back({{center.x, center.y, center.z}, {half.x, half.y, half.z}});
  }
  collisionMap.terrain = std::make_shared<const collision::Terrain>(
      std::move(hits), collisionMap.resolution);
  collisionMap.aabbMin = point(field(map, "aabb_min"));
  collisionMap.aabbMax = point(field(map, "aabb_max"));
  collisionMap.resetEpoch = field(map, "reset_epoch").AsUnsigned("reset epoch");
  collisionMap.observationSequence = field(map, "observation_sequence").AsUnsigned("observation sequence");
  collisionMap.generation = field(map, "generation").AsUnsigned("generation");
  collisionMap.stampS = number(field(map, "stamp_s"));
  collisionMap.receiveStampS = number(field(map, "receive_stamp_s"));
  collisionMap.complete = boolean(field(map, "complete"));
  collisionMap.live = boolean(field(map, "live"));
  collisionMap.gridFromPlanningTranslation = point(field(map, "grid_from_planning_translation"));
  collisionMap.gridFromPlanningYaw = number(field(map, "grid_from_planning_yaw"));
  const auto &predictions = field(saved, "predictions");
  replay.environment.predictions.expiresAtS = number(field(predictions, "expires_at_s"));
  replay.environment.predictions.observedAtS = number(field(predictions, "observed_at_s"));
  replay.environment.predictions.horizonS = number(field(predictions, "horizon_s"));
  std::vector<nav_kernel::PredictedObstacle> replayPredictions;
  for (const auto &value : field(predictions, "volumes").AsArray("prediction volumes"))
    replayPredictions.push_back({point(field(value, "start")), point(field(value, "end")),
        number(field(value, "radius")), number(field(value, "min_z")), number(field(value, "max_z"))});
  replay.environment.predictions.obstacles = replayPredictions.data();
  replay.environment.predictions.count = replayPredictions.size();
  nav_kernel::LocalPlannerParams replayParams;
  replayParams.backend = nav_kernel::LocalPlannerBackend::Scan;
  replayParams.checkObstacle = boolean(field(saved, "check_obstacle"));
  const auto &params = field(saved, "params");
  replayParams.scan.voxelResolution = number(field(params, "voxelResolution"));
  replayParams.scan.bodyClearanceBelow = number(field(params, "bodyClearanceBelow"));
  replayParams.scan.bodyClearanceAbove = number(field(params, "bodyClearanceAbove"));
  replayParams.scan.cylinderRadius = number(field(params, "cylinderRadius"));
  replayParams.scan.cylinderOffset = number(field(params, "cylinderOffset"));
  ASSERT_TRUE(replay.body);
  ASSERT_TRUE(replay.observedBody);
  EXPECT_EQ(replay.body->margin_m, fixture.request.body->margin_m);
  EXPECT_EQ(replay.body->horizontal_margin_m, fixture.request.body->horizontal_margin_m);
  EXPECT_EQ(replay.observedBody->parts.front().center.y,
            fixture.request.observedBody->parts.front().center.y);
  ASSERT_EQ(replay.body->parts.size(), fixture.request.body->parts.size());
  for (std::size_t i = 0; i < replay.body->parts.size(); ++i) {
    const auto &actual = replay.body->parts[i];
    const auto &expected = fixture.request.body->parts[i];
    EXPECT_EQ(actual.link, expected.link);
    EXPECT_EQ(actual.rotation, expected.rotation);
    EXPECT_EQ(actual.shape, expected.shape);
    EXPECT_EQ(actual.radius_m, expected.radius_m);
    EXPECT_EQ(actual.half_length_m, expected.half_length_m);
    EXPECT_EQ(actual.support_contact, expected.support_contact);
  }
  EXPECT_EQ(replay.supportHeightM, fixture.request.supportHeightM);
  EXPECT_EQ(replay.supportHeightToleranceM, fixture.request.supportHeightToleranceM);
  EXPECT_EQ(number(field(field(saved, "predictions"), "expires_at_s")),
            fixture.request.environment.predictions.expiresAtS);
  EXPECT_EQ(replayPredictions.size(), fixture.predictions.size());
  EXPECT_TRUE(replay.environment.predictions.fresh(replay.clock.timestampS));
  EXPECT_FALSE(replay.environment.predictions.fresh(replay.environment.predictions.expiresAtS));
  EXPECT_EQ(*replay.environment.collision.measuredOccupiedStorage,
            *fixture.request.environment.collision.measuredOccupiedStorage);
  EXPECT_EQ(*replay.environment.collision.knownFreeStorage,
            *fixture.request.environment.collision.knownFreeStorage);
  ASSERT_NE(replay.environment.collision.terrain->hitBounds(), nullptr);
  const auto &restoredHits = *replay.environment.collision.terrain->hitBounds();
  ASSERT_EQ(restoredHits.size(), 1U);
  EXPECT_EQ(restoredHits[0].center.z, .55);
  EXPECT_EQ(restoredHits[0].half_size.y, .02);

  fixture.raw.clear();
  const nav_kernel::local::scan::Grid original(fixture.params, fixture.request);
  const nav_kernel::local::scan::Grid restored(replayParams, replay);
  ASSERT_TRUE(original.valid()) << original.reason();
  ASSERT_TRUE(restored.valid()) << restored.reason();
  for (const auto probe : {nav_kernel::Vec3{0, 0, .5}, {1, 0, .5}, {1, 0, .8}}) {
    EXPECT_EQ(original.inflatedOccupancy(probe, 0), restored.inflatedOccupancy(probe, 0));
  }
  EXPECT_NE(restored.inflatedOccupancy({0, 0, .5}, 0), 0);
  EXPECT_EQ(restored.inflatedOccupancy({1, 0, .5}, 0), 0);
  EXPECT_EQ(restored.inflatedOccupancy({1, 0, .8}, 0), 0);
}

TEST(ScanFailureSnapshot, AbsentBodyAndRawLayersStayAbsentRatherThanBecomingFreeSpace) {
  nav_kernel::ScanFailureSnapshot failure;
  const auto saved = lingtu::maps::ParseJson(
      lingtu::nav::endpoint::scanFailureSnapshotJson(failure, "snapshot-test"));
  EXPECT_FALSE(body(field(saved, "body")));
  EXPECT_FALSE(body(field(saved, "observed_body")));
  EXPECT_TRUE(std::holds_alternative<std::nullptr_t>(field(saved, "support_height_m").value));
  EXPECT_TRUE(std::holds_alternative<std::nullptr_t>(field(saved, "support_height_tolerance_m").value));
  const auto &map = field(saved, "collision_map");
  EXPECT_FALSE(bitmap(field(map, "measured_occupied_bits")));
  EXPECT_FALSE(bitmap(field(map, "known_free_bits")));
  EXPECT_TRUE(std::holds_alternative<std::nullptr_t>(field(map, "hit_bounds").value));
}

}  // namespace
