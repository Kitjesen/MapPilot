#include "client.hpp"
#include "client_c.h"
#include "clock_sync.hpp"

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/exploration_command.hpp"
#include "message/cpp/inspection_command.hpp"
#include "message/cpp/navigation_command.hpp"
#include "message/cpp/operator_motion.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <unistd.h>

namespace lingtu::nav::commands {
namespace {

using CommandKind = lingtu::message::NavigationCommandKind;
using ExplorationKind = lingtu::message::ExplorationCommandKind;
using OperatorMotionAction = lingtu::message::OperatorMotionAction;
using lingtu::message::explorationCommandKindName;
using lingtu::message::navigationCommandKindName;
using lingtu::message::operatorMotionActionName;

using SteadyClock = std::chrono::steady_clock;

// ACK.header.stamp is the endpoint's ACK publication time.  The client may
// only use it as a lower-bound clock-offset sample when the request/ACK round
// trip is small enough that the unknown return-path delay cannot consume the
// command freshness budget.  100 ms leaves at least 150 ms of the field
// profile's 250 ms teleop budget for command delivery and endpoint scheduling.
constexpr double kMaximumClockSampleRoundTripS = 0.10;
constexpr std::size_t kGoalStatusEventCapacity = 256U;
constexpr std::size_t kGoalStatusRetentionCapacity = 256U;
constexpr std::size_t kInspectionTaskEventCapacity = 512U;
constexpr std::size_t kExplorationRunEventCapacity = 512U;
constexpr std::size_t kMaximumPathPointCount = 1'000'000U;
constexpr std::size_t kMaximumMapScenePointsPerLayer =
    LINGTU_NAV_MAP_SCENE_MAX_POINTS_PER_LAYER;
constexpr std::size_t kMaximumMapSceneTotalPoints =
    LINGTU_NAV_MAP_SCENE_MAX_TOTAL_POINTS;
constexpr std::size_t kMaximumMapSceneGridCellsPerLayer =
    LINGTU_NAV_MAP_SCENE_MAX_GRID_CELLS_PER_LAYER;
constexpr std::size_t kMaximumMapSceneTotalGridCells =
    LINGTU_NAV_MAP_SCENE_MAX_TOTAL_GRID_CELLS;
constexpr std::size_t kMaximumMapScenePayloadBytes =
    LINGTU_NAV_MAP_SCENE_MAX_PAYLOAD_BYTES;
constexpr std::size_t kMaximumMapSceneCloudBytes = 16U * 1024U * 1024U;
constexpr std::size_t kMaximumMapScenePointStep = 64U;
constexpr std::size_t kMaximumMapSceneFields = 8U;
constexpr std::size_t kMaximumMapSceneTextBytes = 128U;
constexpr std::uint8_t kPointFieldFloat32 = 7U;

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillStamp(lingtu_dds_Header& header, double stamp_s) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  fillStamp(header, stamp_s);
  header.frame_id = const_cast<char*>(frame_id);
}

double stampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void requireFinite(double value, const char* label) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(label) + " must be finite");
  }
}

constexpr double kDirectedTargetMaxCoordinateM = 1'000'000.0;
constexpr double kDirectedTargetMaxTtlS = 3'600.0;

void requireDirectedTarget(double x, double y, double ttl_s) {
  requireFinite(x, "directed target x");
  requireFinite(y, "directed target y");
  requireFinite(ttl_s, "directed target ttl_s");
  if (std::abs(x) > kDirectedTargetMaxCoordinateM ||
      std::abs(y) > kDirectedTargetMaxCoordinateM) {
    throw std::invalid_argument("directed target coordinates are out of bounds");
  }
  if (ttl_s <= 0.0 || ttl_s > kDirectedTargetMaxTtlS) {
    throw std::invalid_argument("directed target ttl_s is out of bounds");
  }
}

void requireDirectedTargetSessionId(const std::string& session_id) {
  if (session_id.empty()) {
    throw std::invalid_argument("directed exploration session_id is required");
  }
}

void requireExplorationIdentity(
    const std::string& exploration_run_id,
    const std::string& session_id) {
  if (!lingtu::message::isValidExplorationRunId(exploration_run_id)) {
    throw std::invalid_argument(
        "exploration_run_id must be a canonical uppercase 26-character ULID");
  }
  if (session_id.empty()) {
    throw std::invalid_argument("exploration session_id is required");
  }
}

std::string text(const char* value) {
  return value == nullptr ? std::string{} : std::string(value);
}

enum class SceneDecodeResult {
  Valid,
  Invalid,
  Capacity,
};

bool copyBoundedText(
    const char* value,
    std::size_t maximum_bytes,
    const char* label,
    std::string* out,
    std::string* error,
    bool required = false) {
  if (value == nullptr) {
    if (required) {
      *error = std::string(label) + " is missing";
      return false;
    }
    out->clear();
    return true;
  }
  std::size_t length = 0U;
  while (length < maximum_bytes && value[length] != '\0') {
    ++length;
  }
  if (length == maximum_bytes) {
    *error = std::string(label) + " exceeds map scene text limit";
    return false;
  }
  if (required && length == 0U) {
    *error = std::string(label) + " is empty";
    return false;
  }
  out->assign(value, length);
  return true;
}

bool addPayloadBytes(std::size_t bytes, std::size_t* total) {
  if (bytes > kMaximumMapScenePayloadBytes ||
      *total > kMaximumMapScenePayloadBytes - bytes) {
    return false;
  }
  *total += bytes;
  return true;
}

bool sameSceneIdentity(
    const lingtu_dds_MapCloudLayer& layer,
    const lingtu_dds_MapScene& scene) {
  return layer.reset_epoch == scene.reset_epoch &&
      layer.observation_sequence == scene.observation_sequence &&
      layer.generation == scene.generation &&
      static_cast<bool>(layer.live) == static_cast<bool>(scene.live);
}

bool sameSceneIdentity(
    const lingtu_dds_MapGrid& layer,
    const lingtu_dds_MapScene& scene) {
  return layer.reset_epoch == scene.reset_epoch &&
      layer.observation_sequence == scene.observation_sequence &&
      layer.generation == scene.generation &&
      static_cast<bool>(layer.live) == static_cast<bool>(scene.live);
}

SceneDecodeResult decodeSceneCloud(
    const lingtu_dds_MapCloudLayer& layer,
    const lingtu_dds_MapScene& scene,
    const char* expected_layer,
    std::vector<MapScenePoint>* points,
    std::size_t* total_points,
    std::size_t* payload_bytes,
    std::string* error) {
  std::string layer_name;
  if (!copyBoundedText(
          layer.layer,
          kMaximumMapSceneTextBytes,
          "map scene cloud layer",
          &layer_name,
          error,
          true) ||
      layer_name != expected_layer || !sameSceneIdentity(layer, scene)) {
    if (error->empty()) {
      *error = std::string("map scene cloud identity mismatch: ") +
          expected_layer;
    }
    return SceneDecodeResult::Invalid;
  }

  const auto& cloud = layer.cloud;
  std::string cloud_frame;
  if (!copyBoundedText(
          cloud.header.frame_id,
          kMaximumMapSceneTextBytes,
          "map scene cloud frame",
          &cloud_frame,
          error,
          true) ||
      cloud.is_bigendian || cloud.height == 0U ||
      cloud.point_step == 0U ||
      cloud.point_step > kMaximumMapScenePointStep) {
    if (error->empty()) {
      *error = "map scene cloud geometry or endianness is unsupported";
    }
    return SceneDecodeResult::Invalid;
  }
  if (cloud.fields._maximum < cloud.fields._length ||
      cloud.fields._length > kMaximumMapSceneFields ||
      (cloud.fields._length > 0U && cloud.fields._buffer == nullptr)) {
    *error = "map scene cloud fields are malformed";
    return SceneDecodeResult::Invalid;
  }
  if (cloud.data._maximum < cloud.data._length ||
      cloud.data._length > kMaximumMapSceneCloudBytes ||
      (cloud.data._length > 0U && cloud.data._buffer == nullptr)) {
    *error = "map scene cloud byte payload is malformed or oversized";
    return cloud.data._length > kMaximumMapSceneCloudBytes
        ? SceneDecodeResult::Capacity
        : SceneDecodeResult::Invalid;
  }

  const std::size_t rows = cloud.height;
  const std::size_t columns = cloud.width;
  if (columns > kMaximumMapScenePointsPerLayer ||
      (columns > 0U && rows > kMaximumMapScenePointsPerLayer / columns)) {
    *error = "map scene cloud point count exceeds per-layer limit";
    return SceneDecodeResult::Capacity;
  }
  const std::size_t point_count = rows * columns;
  if (point_count > kMaximumMapScenePointsPerLayer ||
      *total_points > kMaximumMapSceneTotalPoints - point_count) {
    *error = "map scene cloud point count exceeds product limit";
    return SceneDecodeResult::Capacity;
  }
  if (columns > std::numeric_limits<std::size_t>::max() / cloud.point_step) {
    *error = "map scene cloud row size overflows";
    return SceneDecodeResult::Invalid;
  }
  const std::size_t minimum_row_step = columns * cloud.point_step;
  const std::size_t row_step =
      cloud.row_step == 0U ? minimum_row_step : cloud.row_step;
  if (row_step < minimum_row_step ||
      (rows > 1U &&
       row_step >
           (std::numeric_limits<std::size_t>::max() - minimum_row_step) /
               (rows - 1U))) {
    *error = "map scene cloud row stride is invalid";
    return SceneDecodeResult::Invalid;
  }
  const std::size_t required =
      rows == 0U ? 0U : (rows - 1U) * row_step + minimum_row_step;
  if (required > cloud.data._length) {
    *error = "map scene cloud byte payload is truncated";
    return SceneDecodeResult::Invalid;
  }

  std::size_t x_offset = 0U;
  std::size_t y_offset = 0U;
  std::size_t z_offset = 0U;
  std::size_t intensity_offset = 0U;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  for (std::size_t index = 0U; index < cloud.fields._length; ++index) {
    const auto& field = cloud.fields._buffer[index];
    std::string name;
    if (!copyBoundedText(
            field.name,
            32U,
            "map scene point field",
            &name,
            error,
            true)) {
      return SceneDecodeResult::Invalid;
    }
    if (name != "x" && name != "y" && name != "z" &&
        name != "intensity") {
      continue;
    }
    if (field.datatype != kPointFieldFloat32 || field.count != 1U ||
        field.offset > cloud.point_step ||
        sizeof(float) > cloud.point_step - field.offset) {
      *error = "map scene point fields must be scalar float32 values";
      return SceneDecodeResult::Invalid;
    }
    if (name == "x") {
      x_offset = field.offset;
      has_x = true;
    } else if (name == "y") {
      y_offset = field.offset;
      has_y = true;
    } else if (name == "z") {
      z_offset = field.offset;
      has_z = true;
    } else {
      intensity_offset = field.offset;
      has_intensity = true;
    }
  }
  if (!has_x || !has_y || !has_z) {
    *error = "map scene cloud is missing x/y/z fields";
    return SceneDecodeResult::Invalid;
  }
  const std::size_t copied_bytes = point_count * sizeof(MapScenePoint);
  if (!addPayloadBytes(copied_bytes, payload_bytes)) {
    *error = "map scene cloud copy exceeds total payload limit";
    return SceneDecodeResult::Capacity;
  }

  points->clear();
  points->reserve(point_count);
  if (point_count == 0U) {
    return SceneDecodeResult::Valid;
  }
  for (std::size_t row = 0U; row < rows; ++row) {
    const std::uint8_t* row_data = cloud.data._buffer + row * row_step;
    for (std::size_t column = 0U; column < columns; ++column) {
      const std::uint8_t* raw = row_data + column * cloud.point_step;
      MapScenePoint point;
      std::memcpy(&point.x, raw + x_offset, sizeof(float));
      std::memcpy(&point.y, raw + y_offset, sizeof(float));
      std::memcpy(&point.z, raw + z_offset, sizeof(float));
      if (has_intensity) {
        std::memcpy(
            &point.intensity, raw + intensity_offset, sizeof(float));
      }
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
        *error = "map scene cloud contains non-finite point data";
        points->clear();
        return SceneDecodeResult::Invalid;
      }
      points->push_back(point);
    }
  }
  *total_points += point_count;
  return SceneDecodeResult::Valid;
}

SceneDecodeResult decodeSceneGrid(
    const lingtu_dds_MapGrid& layer,
    const lingtu_dds_MapScene& scene,
    const char* expected_layer,
    MapSceneGridSnapshot* grid,
    std::size_t* total_grid_cells,
    std::size_t* payload_bytes,
    std::string* error) {
  std::string layer_name;
  if (!copyBoundedText(
          layer.layer,
          kMaximumMapSceneTextBytes,
          "map scene grid layer",
          &layer_name,
          error,
          true) ||
      layer_name != expected_layer || !sameSceneIdentity(layer, scene)) {
    if (error->empty()) {
      *error = std::string("map scene grid identity mismatch: ") +
          expected_layer;
    }
    return SceneDecodeResult::Invalid;
  }
  if (layer.data._maximum < layer.data._length ||
      (layer.data._length > 0U && layer.data._buffer == nullptr)) {
    *error = "map scene grid data sequence is malformed";
    return SceneDecodeResult::Invalid;
  }
  const std::size_t width = layer.info.width;
  const std::size_t height = layer.info.height;
  if (width > 0U &&
      height > kMaximumMapSceneGridCellsPerLayer / width) {
    *error = "map scene grid dimensions exceed per-layer limit";
    return SceneDecodeResult::Capacity;
  }
  const std::size_t cell_count = width * height;
  if (cell_count != layer.data._length) {
    *error = "map scene grid dimensions do not match its data length";
    return SceneDecodeResult::Invalid;
  }
  if (cell_count > kMaximumMapSceneGridCellsPerLayer ||
      *total_grid_cells >
          kMaximumMapSceneTotalGridCells - cell_count) {
    *error = "map scene grid cells exceed product limit";
    return SceneDecodeResult::Capacity;
  }
  const std::size_t copied_bytes = cell_count * sizeof(float);
  if (!addPayloadBytes(copied_bytes, payload_bytes)) {
    *error = "map scene grid copy exceeds total payload limit";
    return SceneDecodeResult::Capacity;
  }
  const auto& origin = layer.info.origin;
  if (!std::isfinite(layer.info.resolution) ||
      (cell_count > 0U && !(layer.info.resolution > 0.0F)) ||
      !std::isfinite(origin.position.x) ||
      !std::isfinite(origin.position.y) ||
      !std::isfinite(origin.position.z) ||
      !std::isfinite(origin.orientation.x) ||
      !std::isfinite(origin.orientation.y) ||
      !std::isfinite(origin.orientation.z) ||
      !std::isfinite(origin.orientation.w)) {
    *error = "map scene grid metadata contains non-finite values";
    return SceneDecodeResult::Invalid;
  }

  grid->width = layer.info.width;
  grid->height = layer.info.height;
  grid->resolution = layer.info.resolution;
  grid->origin_x = origin.position.x;
  grid->origin_y = origin.position.y;
  grid->origin_z = origin.position.z;
  grid->origin_qx = origin.orientation.x;
  grid->origin_qy = origin.orientation.y;
  grid->origin_qz = origin.orientation.z;
  grid->origin_qw = origin.orientation.w;
  if (cell_count == 0U) {
    grid->cells.clear();
  } else {
    grid->cells.assign(
        layer.data._buffer, layer.data._buffer + layer.data._length);
  }
  *total_grid_cells += cell_count;
  return SceneDecodeResult::Valid;
}

SceneDecodeResult decodeMapScene(
    const lingtu_dds_MapScene& message,
    MapSceneSnapshot* scene,
    std::string* error) {
  scene->timestamp_s = stampSeconds(message.header.stamp);
  if (!std::isfinite(scene->timestamp_s) || scene->timestamp_s <= 0.0 ||
      !copyBoundedText(
          message.header.frame_id,
          kMaximumMapSceneTextBytes,
          "map scene frame",
          &scene->frame_id,
          error,
          true) ||
      !copyBoundedText(
          message.producer_boot_id,
          kMaximumMapSceneTextBytes,
          "map scene producer boot id",
          &scene->producer_boot_id,
          error,
          true) ||
      message.observation_sequence == 0U || message.generation == 0U) {
    if (error->empty()) {
      *error = "map scene header is invalid";
    }
    return SceneDecodeResult::Invalid;
  }
  const auto& pose = message.map_sensor;
  if (!std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) ||
      !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) ||
      !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w)) {
    *error = "map scene sensor pose contains non-finite values";
    return SceneDecodeResult::Invalid;
  }
  scene->reset_epoch = message.reset_epoch;
  scene->observation_sequence = message.observation_sequence;
  scene->generation = message.generation;
  scene->live = message.live;
  scene->sensor_x = pose.position.x;
  scene->sensor_y = pose.position.y;
  scene->sensor_z = pose.position.z;
  scene->sensor_qx = pose.orientation.x;
  scene->sensor_qy = pose.orientation.y;
  scene->sensor_qz = pose.orientation.z;
  scene->sensor_qw = pose.orientation.w;

  std::size_t total_points = 0U;
  std::size_t total_grid_cells = 0U;
  std::size_t payload_bytes = 0U;
  for (const auto& item : {
           std::tuple<
               const lingtu_dds_MapCloudLayer*,
               const char*,
               std::vector<MapScenePoint>*>{
               &message.live_cloud, "live", &scene->live_points},
           {
               &message.voxel_cloud,
               "voxel",
               &scene->voxel_points},
           {
               &message.accumulated_cloud,
               "accumulated",
               &scene->accumulated_points},
       }) {
    const auto result = decodeSceneCloud(
        *std::get<0>(item),
        message,
        std::get<1>(item),
        std::get<2>(item),
        &total_points,
        &payload_bytes,
        error);
    if (result != SceneDecodeResult::Valid) {
      return result;
    }
  }
  for (const auto& item : {
           std::tuple<
               const lingtu_dds_MapGrid*,
               const char*,
           MapSceneGridSnapshot*>{
               &message.occupancy, "occupancy", &scene->occupancy},
           {&message.elevation, "elevation", &scene->elevation},
           {&message.esdf, "esdf", &scene->esdf},
       }) {
    const auto result = decodeSceneGrid(
        *std::get<0>(item),
        message,
        std::get<1>(item),
        std::get<2>(item),
        &total_grid_cells,
        &payload_bytes,
        error);
    if (result != SceneDecodeResult::Valid) {
      return result;
    }
  }
  scene->payload_bytes = payload_bytes;
  return SceneDecodeResult::Valid;
}

bool decodeMapRuntimeState(
    const lingtu_dds_MapRuntimeState& message,
    MapSceneHealthSnapshot* health,
    std::int64_t* stamp_ns,
    std::string* error) {
  if (message.header.stamp.sec <= 0 ||
      message.header.stamp.nanosec >= 1'000'000'000U) {
    *error = "map runtime state timestamp is invalid";
    return false;
  }
  *stamp_ns =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
  std::string frame_id;
  std::string engine_error;
  std::string input_error;
  std::string output_error;
  if (!copyBoundedText(
          message.header.frame_id,
          kMaximumMapSceneTextBytes,
          "map runtime state frame",
          &frame_id,
          error,
          true) ||
      !copyBoundedText(
          message.producer_boot_id,
          kMaximumMapSceneTextBytes,
          "map runtime state producer boot id",
          &health->state_producer_boot_id,
          error,
          true) ||
      !copyBoundedText(
          message.engine_error,
          512U,
          "map runtime engine error",
          &engine_error,
          error) ||
      !copyBoundedText(
          message.input_error,
          512U,
          "map runtime input error",
          &input_error,
          error) ||
      !copyBoundedText(
          message.output_error,
          512U,
          "map runtime output error",
          &output_error,
          error)) {
    return false;
  }
  if (message.current_generation_published &&
      (message.state_published_generation < message.generation ||
       message.realtime_clouds_published_generation < message.generation ||
       message.map_layers_published_generation < message.generation ||
       message.scene_published_generation < message.generation)) {
    *error = "map runtime state publication cursors are inconsistent";
    return false;
  }

  health->state_timestamp_s = stampSeconds(message.header.stamp);
  health->state_received = true;
  health->state_running = message.running;
  health->state_live = message.live;
  health->state_required_publications_ready =
      message.required_publications_ready;
  health->state_current_generation_published =
      message.current_generation_published;
  health->state_capacity_limited = message.capacity_limited;
  health->state_reset_epoch = message.reset_epoch;
  health->state_observation_sequence = message.observation_sequence;
  health->state_generation = message.generation;
  health->state_scene_published_generation =
      message.scene_published_generation;
  health->state_error.clear();
  for (const auto& item : {
           std::pair<const char*, const std::string*>{
               "engine", &engine_error},
           {"input", &input_error},
           {"output", &output_error},
       }) {
    if (item.second->empty()) {
      continue;
    }
    if (!health->state_error.empty()) {
      health->state_error += "; ";
    }
    health->state_error += item.first;
    health->state_error += ":";
    health->state_error += *item.second;
  }
  return true;
}

bool diagnosticsEnabled() {
  const char* raw = std::getenv("LINGTU_NAV_CLIENT_DIAGNOSTICS");
  if (raw == nullptr) {
    return false;
  }
  const std::string value(raw);
  return !value.empty() && value != "0" && value != "false" && value != "off";
}

std::string makeRequestId(CommandKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "nav-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeTaskId(CommandKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "nav-task-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeClientId() {
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return "nav-client-" + std::to_string(static_cast<long long>(getpid())) +
      "-" + std::to_string(ticks);
}

std::string makeExplorationRequestId(ExplorationKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "explore-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeInspectionRequestId(
    lingtu::message::InspectionCommandKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "inspection-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeOperatorMotionRequestId(OperatorMotionAction action) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "operator-motion-" +
      std::to_string(static_cast<std::int32_t>(action)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeOperatorMotionSampleRequestId() {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "operator-motion-sample-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::uint64_t sourceStampNs(double stamp_s) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp_s * 1e9);
}

bool isRecoverableClockRejection(CommandKind kind, const std::string& reason) {
  if (kind == CommandKind::ClearEstop) {
    return reason == "clear_estop_source_stamp_stale" ||
        reason == "clear_estop_source_stamp_future";
  }
  if (kind == CommandKind::ResumeAutonomy) {
    return reason == "resume_autonomy_source_stamp_stale" ||
        reason == "resume_autonomy_source_stamp_future";
  }
  return false;
}

void requireAcceptedNavigationReceipt(
    const NavigationCommandReceipt& receipt) {
  if (receipt.accepted) {
    return;
  }
  throw std::runtime_error(
      "navigation command rejected: " +
      (!receipt.diagnostic.empty()
           ? receipt.diagnostic
           : (receipt.reason.empty() ? std::string("unspecified") :
              receipt.reason) +
               " [task_id=" + receipt.task_id + " request_id=" +
               receipt.request_id + " kind=" + navigationCommandKindName(
                   static_cast<CommandKind>(receipt.kind)) + "]"));
}

void requireAcceptedOperatorMotionReceipt(
    const OperatorMotionCommandReceipt& receipt) {
  if (receipt.accepted) {
    return;
  }
  throw std::runtime_error(
      "operator motion command rejected: " +
      (receipt.reason.empty() ? std::string("unspecified") : receipt.reason) +
      " [request_id=" + receipt.request_id + " action=" +
      operatorMotionActionName(static_cast<OperatorMotionAction>(receipt.action)) + "]");
}

}  // namespace

struct Client::Impl {
  struct AckObservation {
    bool accepted{false};
    std::string task_id;
    std::string request_id;
    std::int32_t kind{0};
    std::string reason;
    double endpoint_stamp_s{0.0};
    double local_receive_wall_s{0.0};
    SteadyClock::time_point received_steady{};
    double round_trip_s{0.0};
    std::string run_id;
    bool duplicate{false};
    bool correlation_valid{true};
  };

  struct PendingNavigationAck {
    CommandKind kind;
    std::string task_id;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  struct PendingExplorationAck {
    ExplorationKind kind;
    std::string exploration_run_id;
    std::string session_id;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  struct PendingInspectionAck {
    lingtu::message::InspectionCommandKind kind;
    std::string task_id;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  struct PendingOperatorMotionAck {
    OperatorMotionAction action;
    std::string source_id;
    std::uint64_t source_epoch{0U};
    std::uint64_t sequence{0U};
    SteadyClock::time_point sent_steady;
    std::optional<OperatorMotionCommandReceipt> receipt;
  };

  explicit Impl(int domain_id) {
    participant = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(nav_client)");
    try {
      publisher = checked(
          dds_create_publisher(participant, nullptr, nullptr),
          "dds_create_publisher(nav_client)");
      subscriber = checked(
          dds_create_subscriber(participant, nullptr, nullptr),
          "dds_create_subscriber(nav_client)");
      command_writer = createWriter(
          lingtu::message::kNavCommandRequest,
          &lingtu_dds_NavigationCommandRequest_desc,
          "nav_command_request");
      ack_reader = createReader(
          lingtu::message::kNavCommandAck,
          &lingtu_dds_NavigationCommandAck_desc,
          "nav_command_ack");
      exploration_writer = createWriter(
          lingtu::message::kNavExplorationCommand,
          &lingtu_dds_ExplorationCommandRequest_desc,
          "exploration_command");
      exploration_ack_reader = createReader(
          lingtu::message::kNavExplorationAck,
          &lingtu_dds_ExplorationCommandAck_desc,
          "exploration_ack");
      exploration_run_event_reader = createReader(
          lingtu::message::kNavExplorationRunEvent,
          &lingtu_dds_ExplorationRunEvent_desc,
          "exploration_run_event");
      inspection_task_writer = createWriter(
          lingtu::message::kNavInspectionTaskRequest,
          &lingtu_dds_InspectionTaskRequest_desc,
          "inspection_task_request");
      inspection_task_ack_reader = createReader(
          lingtu::message::kNavInspectionTaskAck,
          &lingtu_dds_InspectionTaskAck_desc,
          "inspection_task_ack");
      inspection_task_event_reader = createReader(
          lingtu::message::kNavInspectionTaskEvent,
          &lingtu_dds_InspectionTaskEvent_desc,
          "inspection_task_event");
      operator_motion_control_writer = createWriter(
          lingtu::message::kOperatorMotionControl,
          &lingtu_dds_OperatorMotionControl_desc,
          "operator_motion_control");
      operator_motion_sample_writer = createWriter(
          lingtu::message::kOperatorMotionSample,
          &lingtu_dds_OperatorMotionSample_desc,
          "operator_motion_sample");
      operator_motion_ack_reader = createReader(
          lingtu::message::kOperatorMotionAck,
          &lingtu_dds_OperatorMotionAck_desc,
          "operator_motion_ack");
      navigation_goal_status_reader = createReader(
          lingtu::message::kNavGoalStatus,
          &lingtu_dds_NavigationGoalStatus_desc,
          "navigation_goal_status");
      navigation_state_reader = createReader(
          lingtu::message::kNavState,
          &lingtu_dds_NavigationState_desc,
          "navigation_state");
      global_path_reader = createReader(
          lingtu::message::kNavGlobalPath,
          &lingtu_dds_Path_desc,
          "global_path");
      local_path_reader = createReader(
          lingtu::message::kNavLocalPath,
          &lingtu_dds_Path_desc,
          "local_path");
      map_scene_reader = createReader(
          lingtu::message::kMapsScene,
          &lingtu_dds_MapScene_desc,
          "map_scene");
      map_state_reader = createReader(
          lingtu::message::kMapsState,
          &lingtu_dds_MapRuntimeState_desc,
          "map_state");
      ack_receiver = std::thread([this]() { ackReceiverLoop(); });
    } catch (...) {
      dds_delete(participant);
      participant = 0;
      throw;
    }
  }

  ~Impl() {
    ack_receiver_running.store(false, std::memory_order_release);
    if (ack_receiver.joinable()) {
      ack_receiver.join();
    }
    if (participant > 0) {
      dds_delete(participant);
    }
  }

  dds_entity_t createWriter(
      const lingtu::message::TopicContract& contract,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant,
            descriptor,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(
        dds_create_writer(publisher, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t createReader(
      const lingtu::message::TopicContract& contract,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant,
            descriptor,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(
        dds_create_reader(subscriber, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  void waitForReader(
      dds_entity_t writer,
      const char* label,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(0, timeout_ms));
    do {
      const dds_return_t count =
          dds_get_matched_subscriptions(writer, nullptr, 0);
      if (count > 0) {
        return;
      }
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_get_matched_subscriptions(") + label + "): " +
            dds_strretcode(-count));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (SteadyClock::now() < deadline);
    throw std::runtime_error(std::string("no matched DDS reader for ") + label);
  }

  void waitForWriter(
      dds_entity_t reader,
      const char* label,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(0, timeout_ms));
    do {
      const dds_return_t count = dds_get_matched_publications(reader, nullptr, 0);
      if (count > 0) {
        return;
      }
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_get_matched_publications(") + label + "): " +
            dds_strretcode(-count));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (SteadyClock::now() < deadline);
    throw std::runtime_error(std::string("no matched DDS writer for ") + label);
  }

  bool updateEndpointClock(const AckObservation& observation) const {
    if (!std::isfinite(observation.endpoint_stamp_s) ||
        observation.endpoint_stamp_s <= 0.0 ||
        !std::isfinite(observation.local_receive_wall_s) ||
        observation.local_receive_wall_s <= 0.0 ||
        !std::isfinite(observation.round_trip_s) ||
        observation.round_trip_s < 0.0 ||
        observation.round_trip_s > kMaximumClockSampleRoundTripS) {
      return false;
    }
    endpoint_clock_offset_s.store(
        endpointClockOffset(
            observation.endpoint_stamp_s,
            observation.local_receive_wall_s),
        std::memory_order_relaxed);
    return true;
  }

  double endpointClockOffsetOrZero() const {
    const double offset = endpoint_clock_offset_s.load(std::memory_order_relaxed);
    return std::isfinite(offset) ? offset : 0.0;
  }

  std::string rejectionMessage(
      const std::string& request_id,
      CommandKind kind,
      const AckObservation& observation) const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(9)
        << "navigation command rejected: "
        << (observation.reason.empty() ? "unspecified" : observation.reason)
        << " [request_id=" << request_id
        << " kind=" << navigationCommandKindName(kind)
        << " sync_rtt_ms="
        << last_sync_rtt_s.load(std::memory_order_relaxed) * 1000.0
        << " endpoint_stamp_s="
        << last_sync_endpoint_stamp_s.load(std::memory_order_relaxed)
        << " local_wall_receive_s="
        << last_sync_local_receive_wall_s.load(std::memory_order_relaxed)
        << " clock_offset_s="
        << last_send_clock_offset_s.load(std::memory_order_relaxed)
        << " send_source_stamp_s="
        << last_send_source_stamp_s.load(std::memory_order_relaxed)
        << " post_ack_clock_offset_s=" << endpointClockOffsetOrZero()
        << " ack_rtt_ms=" << observation.round_trip_s * 1000.0
        << " ack_endpoint_stamp_s=" << observation.endpoint_stamp_s
        << " ack_local_wall_receive_s=" << observation.local_receive_wall_s
        << ']';
    return out.str();
  }

  void logClockEvent(
      const char* event,
      const std::string& request_id,
      CommandKind kind,
      const AckObservation* observation,
      double send_source_stamp_s) const {
    if (!diagnostics_enabled) {
      return;
    }
    std::fprintf(
        stderr,
        "nav_client_clock: event=%s request_id=%s kind=%s sync_rtt_ms=%.3f endpoint_stamp_s=%.9f local_wall_receive_s=%.9f clock_offset_s=%.9f send_source_stamp_s=%.9f ack_rtt_ms=%.3f ack_endpoint_stamp_s=%.9f ack_local_wall_receive_s=%.9f\n",
        event,
        request_id.c_str(),
        navigationCommandKindName(kind),
        last_sync_rtt_s.load(std::memory_order_relaxed) * 1000.0,
        last_sync_endpoint_stamp_s.load(std::memory_order_relaxed),
        last_sync_local_receive_wall_s.load(std::memory_order_relaxed),
        send_source_stamp_s > 0.0
            ? last_send_clock_offset_s.load(std::memory_order_relaxed)
            : endpointClockOffsetOrZero(),
        send_source_stamp_s,
        observation == nullptr ? -1.0 : observation->round_trip_s * 1000.0,
        observation == nullptr ? 0.0 : observation->endpoint_stamp_s,
        observation == nullptr ? 0.0 : observation->local_receive_wall_s);
  }

  double sourceNowSeconds() const {
    const double local_wall_s = nowSeconds();
    const double offset = endpoint_clock_offset_s.load(std::memory_order_relaxed);
    return std::isfinite(offset)
        ? endpointSourceTime(local_wall_s, offset)
        : local_wall_s;
  }

  using NavigationPending = std::shared_ptr<PendingNavigationAck>;
  using ExplorationPending = std::shared_ptr<PendingExplorationAck>;
  using InspectionPending = std::shared_ptr<PendingInspectionAck>;
  using OperatorMotionPending = std::shared_ptr<PendingOperatorMotionAck>;

  NavigationPending registerNavigationAck(
      const std::string& request_id,
      const std::string& task_id,
      CommandKind kind,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingNavigationAck>(
        PendingNavigationAck{kind, task_id, sent_steady, std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_navigation_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight navigation request id: " + request_id);
    }
    return pending;
  }

  ExplorationPending registerExplorationAck(
      const std::string& request_id,
      ExplorationKind kind,
      const std::string& exploration_run_id,
      const std::string& session_id,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingExplorationAck>(
        PendingExplorationAck{
            kind,
            exploration_run_id,
            session_id,
            sent_steady,
            std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_exploration_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight exploration request id: " + request_id);
    }
    return pending;
  }

  InspectionPending registerInspectionAck(
      const std::string& request_id,
      lingtu::message::InspectionCommandKind kind,
      SteadyClock::time_point sent_steady,
      const std::string& task_id) const {
    auto pending = std::make_shared<PendingInspectionAck>(
        PendingInspectionAck{kind, task_id, sent_steady, std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_inspection_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight inspection request id: " + request_id);
    }
    return pending;
  }

  OperatorMotionPending registerOperatorMotionAck(
      const std::string& request_id,
      OperatorMotionAction action,
      const std::string& source_id,
      std::uint64_t source_epoch,
      std::uint64_t sequence,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingOperatorMotionAck>(
        PendingOperatorMotionAck{
            action,
            source_id,
            source_epoch,
            sequence,
            sent_steady,
            std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_operator_motion_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight operator motion request id: " + request_id);
    }
    return pending;
  }

  void unregisterNavigationAck(
      const std::string& request_id,
      const NavigationPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_navigation_acks.find(request_id);
    if (it != pending_navigation_acks.end() && it->second == pending) {
      pending_navigation_acks.erase(it);
    }
  }

  void unregisterExplorationAck(
      const std::string& request_id,
      const ExplorationPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_exploration_acks.find(request_id);
    if (it != pending_exploration_acks.end() && it->second == pending) {
      pending_exploration_acks.erase(it);
    }
  }

  void unregisterInspectionAck(
      const std::string& request_id,
      const InspectionPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_inspection_acks.find(request_id);
    if (it != pending_inspection_acks.end() && it->second == pending) {
      pending_inspection_acks.erase(it);
    }
  }

  void unregisterOperatorMotionAck(
      const std::string& request_id,
      const OperatorMotionPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_operator_motion_acks.find(request_id);
    if (it != pending_operator_motion_acks.end() && it->second == pending) {
      pending_operator_motion_acks.erase(it);
    }
  }

  AckObservation waitForAck(
      const std::string& request_id,
      const NavigationPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_navigation_acks.find(request_id);
    if (it != pending_navigation_acks.end() && it->second == pending) {
      pending_navigation_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for navigation command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  AckObservation waitForExplorationAck(
      const std::string& request_id,
      const ExplorationPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_exploration_acks.find(request_id);
    if (it != pending_exploration_acks.end() && it->second == pending) {
      pending_exploration_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for exploration command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  AckObservation waitForInspectionAck(
      const std::string& request_id,
      const InspectionPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_inspection_acks.find(request_id);
    if (it != pending_inspection_acks.end() && it->second == pending) {
      pending_inspection_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for inspection command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  OperatorMotionCommandReceipt waitForOperatorMotionAck(
      const std::string& request_id,
      const OperatorMotionPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->receipt.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_operator_motion_acks.find(request_id);
    if (it != pending_operator_motion_acks.end() && it->second == pending) {
      pending_operator_motion_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for operator motion ACK: " + request_id);
    }
    if (!pending->receipt) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->receipt;
  }

  bool takeNavigationAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count =
        dds_take(ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(nav_command_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_NavigationCommandAck*>(samples[i]);
        const auto it = pending_navigation_acks.find(text(ack.request_id));
        if (it == pending_navigation_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            text(ack.task_id),
            text(ack.request_id),
            ack.kind,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
            "",
        };
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "command_ack_kind_mismatch";
        }
        if (text(ack.task_id) != pending.task_id) {
          observation.accepted = false;
          observation.reason = "command_ack_task_id_mismatch";
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(ack_reader, samples, count),
          "dds_return_loan(nav_command_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeExplorationAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        exploration_ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(exploration_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_ExplorationCommandAck*>(samples[i]);
        const auto it = pending_exploration_acks.find(text(ack.request_id));
        if (it == pending_exploration_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            "",
            text(ack.request_id),
            ack.kind,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
            text(ack.exploration_run_id),
        };
        observation.duplicate = ack.duplicate;
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "exploration_ack_kind_mismatch";
          observation.correlation_valid = false;
        }
        if (text(ack.exploration_run_id) != pending.exploration_run_id) {
          observation.accepted = false;
          observation.reason = "exploration_ack_run_id_mismatch";
          observation.correlation_valid = false;
        }
        if (text(ack.session_id) != pending.session_id) {
          observation.accepted = false;
          observation.reason = "exploration_ack_session_mismatch";
          observation.correlation_valid = false;
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(exploration_ack_reader, samples, count),
          "dds_return_loan(exploration_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeInspectionTaskAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        inspection_task_ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(inspection_task_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack = *static_cast<lingtu_dds_InspectionTaskAck*>(samples[i]);
        const auto it = pending_inspection_acks.find(text(ack.request_id));
        if (it == pending_inspection_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            text(ack.task_id),
            text(ack.request_id),
            ack.kind,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
            text(ack.run_id),
        };
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "inspection_task_ack_kind_mismatch";
        }
        if (text(ack.task_id) != pending.task_id) {
          observation.accepted = false;
          observation.reason = "inspection_task_ack_task_id_mismatch";
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(inspection_task_ack_reader, samples, count),
          "dds_return_loan(inspection_task_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeOperatorMotionAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        operator_motion_ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(operator_motion_ack): ") +
          dds_strretcode(-count));
    }
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_OperatorMotionAck*>(samples[i]);
        const auto it = pending_operator_motion_acks.find(text(ack.request_id));
        if (it == pending_operator_motion_acks.end() ||
            it->second->receipt) {
          continue;
        }
        auto& pending = *it->second;
        if (ack.action != static_cast<std::int32_t>(pending.action) ||
            text(ack.source_id) != pending.source_id ||
            ack.source_epoch != pending.source_epoch ||
            ack.source_sequence != pending.sequence) {
          // request_id values are supplied by adapters and are not globally
          // unique across DDS participants. A same-id ACK for another source
          // must not complete (or reject) this caller's pending operation.
          continue;
        }
        OperatorMotionCommandReceipt receipt{
            ack.accepted,
            ack.action,
            text(ack.request_id),
            text(ack.source_id),
            ack.source_epoch,
            ack.source_sequence,
            ack.accepted_sequence,
            ack.final_output_sequence,
            stampSeconds(ack.header.stamp),
            text(ack.reason),
        };
        pending.receipt = std::move(receipt);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(operator_motion_ack_reader, samples, count),
          "dds_return_loan(operator_motion_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeNavigationStates() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        navigation_state_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(navigation_state): ") +
          dds_strretcode(-count));
    }
    {
      std::lock_guard<std::mutex> lock(navigation_state_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& state =
            *static_cast<lingtu_dds_NavigationState*>(samples[i]);
        const std::string boot_id = text(state.boot_id);
        if (boot_id.empty() || state.state_sequence == 0U) {
          continue;
        }
        if (latest_navigation_state.has_value() &&
            latest_navigation_state->boot_id == boot_id &&
            state.state_sequence <= latest_navigation_state->sequence) {
          continue;
        }
        latest_navigation_state = NavigationStateSnapshot{
            stampSeconds(state.header.stamp),
            text(state.header.frame_id),
            boot_id,
            state.state_sequence,
            state.control_mode,
            state.lifecycle_state,
            text(state.active_task_id),
            text(state.active_request_id),
            state.goal_epoch,
            text(state.map_id),
            state.map_version,
            text(state.map_hash),
            state.planning_state,
            state.execution_state,
            state.recovery_state,
            state.progress,
            text(state.authority),
            text(state.hold_reason),
            text(state.failure_code),
        };
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(navigation_state_reader, samples, count),
          "dds_return_loan(navigation_state)");
    }
    return count > 0;
  }

  bool takeNavigationGoalStatuses() {
    constexpr std::size_t kMaxSamples = 64;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        navigation_goal_status_reader,
        samples,
        infos,
        kMaxSamples,
        kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(navigation_goal_status): ") +
          dds_strretcode(-count));
    }
    {
      std::lock_guard<std::mutex> lock(navigation_goal_status_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& status =
            *static_cast<lingtu_dds_NavigationGoalStatus*>(samples[i]);
        const std::string boot_id = text(status.boot_id);
        const std::string task_id = text(status.task_id);
        const std::string request_id = text(status.request_id);
        if (boot_id.empty() || task_id.empty() || request_id.empty() ||
            status.event_sequence == 0U ||
            !lingtu::message::isKnownNavigationGoalState(status.state)) {
          continue;
        }
        auto& last_sequence = goal_status_sequences_by_boot[boot_id];
        if (status.event_sequence <= last_sequence) {
          continue;
        }
        last_sequence = status.event_sequence;

        NavigationGoalStatusSnapshot snapshot{
            stampSeconds(status.header.stamp),
            text(status.header.frame_id),
            boot_id,
            status.event_sequence,
            task_id,
            request_id,
            status.state,
            status.goal_epoch,
            text(status.reason),
        };
        if (retained_goal_statuses.find(request_id) ==
            retained_goal_statuses.end()) {
          retained_goal_status_order.push_back(request_id);
        }
        retained_goal_statuses[request_id] = snapshot;
        if (retained_task_goal_statuses.find(task_id) ==
            retained_task_goal_statuses.end()) {
          retained_task_goal_status_order.push_back(task_id);
        }
        retained_task_goal_statuses[task_id] = snapshot;
        while (retained_goal_status_order.size() >
               kGoalStatusRetentionCapacity) {
          const std::string expired = retained_goal_status_order.front();
          retained_goal_status_order.pop_front();
          retained_goal_statuses.erase(expired);
        }
        while (retained_task_goal_status_order.size() >
               kGoalStatusRetentionCapacity) {
          const std::string expired = retained_task_goal_status_order.front();
          retained_task_goal_status_order.pop_front();
          retained_task_goal_statuses.erase(expired);
        }
        if (pending_goal_status_events.size() >= kGoalStatusEventCapacity) {
          pending_goal_status_events.pop_front();
        }
        pending_goal_status_events.push_back(std::move(snapshot));
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(navigation_goal_status_reader, samples, count),
          "dds_return_loan(navigation_goal_status)");
    }
    return count > 0;
  }

  bool takeInspectionTaskEvents() {
    constexpr std::size_t kMaxSamples = 64U;
    std::size_t capacity = 0U;
    {
      std::lock_guard<std::mutex> lock(inspection_task_event_mutex);
      if (pending_inspection_task_events.size() >=
          kInspectionTaskEventCapacity) {
        // Leave samples in the DDS reader rather than silently dropping a
        // lifecycle fact. A later DDS retention overrun becomes a visible
        // sequence gap at the Host projection.
        return false;
      }
      capacity = kInspectionTaskEventCapacity -
          pending_inspection_task_events.size();
    }
    const std::size_t max_samples = std::min(kMaxSamples, capacity);
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        inspection_task_event_reader,
        samples,
        infos,
        max_samples,
        max_samples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(inspection_task_event): ") +
          dds_strretcode(-count));
    }
    {
      std::lock_guard<std::mutex> lock(inspection_task_event_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& event =
            *static_cast<lingtu_dds_InspectionTaskEvent*>(samples[i]);
        const std::string boot_id = text(event.boot_id);
        const std::string task_id = text(event.task_id);
        const std::string request_id = text(event.request_id);
        const std::string command_request_id = text(event.command_request_id);
        const std::string map_id = text(event.map_id);
        const std::string route_id = text(event.route_id);
        const bool known_kind = event.kind >= 1 && event.kind <= 5;
        const bool known_state = event.state >= 0 && event.state <= 13;
        if (boot_id.empty() || task_id.empty() || request_id.empty() ||
            command_request_id.empty() || map_id.empty() || route_id.empty() ||
            event.event_sequence == 0U || event.route_revision == 0U ||
            event.map_version < 0 || !known_kind || !known_state) {
          continue;
        }
        auto& last_sequence = inspection_task_event_sequences_by_boot[boot_id];
        if (event.event_sequence <= last_sequence) {
          continue;
        }
        last_sequence = event.event_sequence;
        pending_inspection_task_events.push_back(InspectionTaskEventSnapshot{
            stampSeconds(event.header.stamp),
            text(event.header.frame_id),
            boot_id,
            event.event_sequence,
            event.kind,
            task_id,
            request_id,
            command_request_id,
            event.state,
            map_id,
            event.map_version,
            route_id,
            event.route_revision,
            event.point_index,
            event.point_count,
            event.loop_index,
            event.retry_count,
            text(event.point_id),
            text(event.action),
            text(event.action_request_id),
            text(event.evidence_id),
            text(event.reason),
        });
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(inspection_task_event_reader, samples, count),
          "dds_return_loan(inspection_task_event)");
    }
    return count > 0;
  }

  bool takeExplorationRunEvents() {
    constexpr std::size_t kMaxSamples = 64U;
    std::size_t capacity = 0U;
    {
      std::lock_guard<std::mutex> lock(exploration_run_event_mutex);
      if (pending_exploration_run_events.size() >=
          kExplorationRunEventCapacity) {
        return false;
      }
      capacity = kExplorationRunEventCapacity -
          pending_exploration_run_events.size();
    }
    const std::size_t max_samples = std::min(kMaxSamples, capacity);
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        exploration_run_event_reader,
        samples,
        infos,
        max_samples,
        max_samples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(exploration_run_event): ") +
          dds_strretcode(-count));
    }
    {
      std::lock_guard<std::mutex> lock(exploration_run_event_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& event =
            *static_cast<lingtu_dds_ExplorationRunEvent*>(samples[i]);
        const std::string frame_id = text(event.frame_id);
        const std::string boot_id = text(event.boot_id);
        const std::string exploration_run_id = text(event.exploration_run_id);
        const std::string start_request_id = text(event.start_request_id);
        const std::string command_request_id = text(event.command_request_id);
        const std::string product_session_id = text(event.product_session_id);
        const std::string route = text(event.route);
        const std::string map_id = text(event.map_id);
        const std::string artifact_hash = text(event.artifact_hash);
        const std::string reason = text(event.reason);
        const std::string motion_stop_reason = text(event.motion_stop_reason);
        const bool known_kind =
            lingtu::message::isKnownExplorationRunEventKind(event.kind);
        const bool known_state =
            lingtu::message::isKnownExplorationRunState(event.state);
        const bool route_valid =
            (route == "live" && map_id.empty() && event.map_version == 0 &&
             artifact_hash.empty()) ||
            (route == "map" && !map_id.empty() && event.map_version > 0 &&
             !artifact_hash.empty());
        if (!std::isfinite(event.timestamp_s) || event.timestamp_s <= 0.0 ||
            frame_id != "map" || boot_id.empty() || event.event_sequence == 0U ||
            !known_kind || !known_state ||
            !lingtu::message::isValidExplorationRunId(exploration_run_id) ||
            start_request_id.empty() || command_request_id.empty() ||
            product_session_id.empty() || reason.empty() || !route_valid ||
            exploration_run_id == start_request_id ||
            exploration_run_id == command_request_id) {
          continue;
        }
        const auto kind = static_cast<lingtu::message::ExplorationRunEventKind>(
            event.kind);
        const auto state = static_cast<lingtu::message::ExplorationRunState>(
            event.state);
        const bool admitted_kind =
            kind == lingtu::message::ExplorationRunEventKind::kAdmitted;
        const bool admitted_state =
            state == lingtu::message::ExplorationRunState::kAdmitted;
        const bool stopped_state_parked =
            !lingtu::message::explorationRunStateRequiresMotionStop(state) ||
            (event.motion_stop_confirmed && !motion_stop_reason.empty());
        const bool transition_not_prematurely_parked =
            ((state != lingtu::message::ExplorationRunState::kPausing &&
              state != lingtu::message::ExplorationRunState::kCancelling) ||
             !event.motion_stop_confirmed);
        const bool stop_failure_valid =
            kind != lingtu::message::ExplorationRunEventKind::kStopConfirmationFailed ||
            ((state == lingtu::message::ExplorationRunState::kPausing ||
              state == lingtu::message::ExplorationRunState::kCancelling) &&
             !event.motion_stop_confirmed && !motion_stop_reason.empty());
        if (admitted_kind != admitted_state || !stopped_state_parked ||
            !transition_not_prematurely_parked ||
            !stop_failure_valid) {
          continue;
        }
        auto& last_sequence = exploration_run_event_sequences_by_boot[boot_id];
        if (event.event_sequence <= last_sequence) {
          continue;
        }
        last_sequence = event.event_sequence;
        pending_exploration_run_events.push_back(ExplorationRunEventSnapshot{
            event.timestamp_s,
            frame_id,
            boot_id,
            event.event_sequence,
            event.kind,
            exploration_run_id,
            start_request_id,
            command_request_id,
            product_session_id,
            event.state,
            route,
            map_id,
            event.map_version,
            artifact_hash,
            reason,
            event.motion_stop_confirmed,
            motion_stop_reason,
        });
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(exploration_run_event_reader, samples, count),
          "dds_return_loan(exploration_run_event)");
    }
    return count > 0;
  }

  bool takePathSamples(
      dds_entity_t reader,
      std::mutex& mutex,
      std::optional<PathSnapshot>& pending,
      std::uint64_t& receive_sequence,
      const char* label) {
    constexpr std::size_t kMaxSamples = 8U;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count =
        dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(") + label + "): " +
          dds_strretcode(-count));
    }
    try {
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& message = *static_cast<lingtu_dds_Path*>(samples[i]);
        const std::size_t point_count =
            static_cast<std::size_t>(message.poses._length);
        if (point_count > kMaximumPathPointCount ||
            (point_count > 0U && message.poses._buffer == nullptr)) {
          continue;
        }
        PathSnapshot candidate;
        candidate.timestamp_s = stampSeconds(message.header.stamp);
        candidate.frame_id = text(message.header.frame_id);
        if (candidate.frame_id.empty()) {
          candidate.frame_id = "map";
        }
        candidate.points.reserve(point_count);
        bool valid = std::isfinite(candidate.timestamp_s) &&
            candidate.timestamp_s > 0.0;
        for (std::size_t point_index = 0U;
             valid && point_index < point_count;
             ++point_index) {
          const auto& position =
              message.poses._buffer[point_index].pose.position;
          valid = std::isfinite(position.x) &&
              std::isfinite(position.y) &&
              std::isfinite(position.z);
          if (valid) {
            candidate.points.push_back(
                PathPoint{position.x, position.y, position.z});
          }
        }
        if (!valid ||
            receive_sequence == std::numeric_limits<std::uint64_t>::max()) {
          continue;
        }
        candidate.receive_sequence = ++receive_sequence;
        std::lock_guard<std::mutex> lock(mutex);
        pending = std::move(candidate);
      }
    } catch (...) {
      if (count > 0) {
        dds_return_loan(reader, samples, count);
      }
      throw;
    }
    if (count > 0) {
      checked(
          dds_return_loan(reader, samples, count),
          (std::string("dds_return_loan(") + label + ")").c_str());
    }
    return count > 0;
  }

  bool takeMapScenes() {
    constexpr std::size_t kMaxSamples = 4U;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        map_scene_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(map_scene): ") + dds_strretcode(-count));
    }
    try {
      for (dds_return_t index = 0; index < count; ++index) {
        if (!infos[index].valid_data) {
          continue;
        }
        MapSceneSnapshot candidate;
        std::string error;
        const auto result = decodeMapScene(
            *static_cast<lingtu_dds_MapScene*>(samples[index]),
            &candidate,
            &error);
        std::lock_guard<std::mutex> lock(map_scene_mutex);
        ++map_scene_health.received_samples;
        if (result != SceneDecodeResult::Valid) {
          if (result == SceneDecodeResult::Capacity) {
            ++map_scene_health.capacity_rejections;
          } else {
            ++map_scene_health.invalid_samples;
          }
          map_scene_health.last_error =
              error.empty() ? "map scene sample was rejected" : error;
          continue;
        }
        const bool stale =
            candidate.producer_boot_id == last_map_scene_boot_id &&
            (candidate.reset_epoch < last_map_scene_reset_epoch ||
             (candidate.reset_epoch == last_map_scene_reset_epoch &&
              candidate.generation <= last_map_scene_generation));
        if (stale) {
          ++map_scene_health.stale_samples;
          continue;
        }
        if (map_scene_receive_sequence ==
            std::numeric_limits<std::uint64_t>::max()) {
          ++map_scene_health.invalid_samples;
          map_scene_health.last_error =
              "map scene receive sequence exhausted";
          continue;
        }
        candidate.receive_sequence = ++map_scene_receive_sequence;
        if (pending_map_scene.has_value()) {
          ++map_scene_health.replaced_samples;
        }
        last_map_scene_boot_id = candidate.producer_boot_id;
        last_map_scene_reset_epoch = candidate.reset_epoch;
        last_map_scene_generation = candidate.generation;
        ++map_scene_health.valid_samples;
        map_scene_health.last_receive_sequence =
            candidate.receive_sequence;
        map_scene_health.last_generation = candidate.generation;
        map_scene_health.last_sample_timestamp_s =
            candidate.timestamp_s;
        map_scene_health.last_error.clear();
        pending_map_scene = std::move(candidate);
      }
    } catch (...) {
      if (count > 0) {
        dds_return_loan(map_scene_reader, samples, count);
      }
      throw;
    }
    if (count > 0) {
      checked(
          dds_return_loan(map_scene_reader, samples, count),
          "dds_return_loan(map_scene)");
    }
    return count > 0;
  }

  bool takeMapRuntimeStates() {
    constexpr std::size_t kMaxSamples = 8U;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        map_state_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(map_state): ") + dds_strretcode(-count));
    }
    try {
      for (dds_return_t index = 0; index < count; ++index) {
        if (!infos[index].valid_data) {
          continue;
        }
        MapSceneHealthSnapshot decoded;
        std::int64_t stamp_ns = 0;
        std::string error;
        const bool valid = decodeMapRuntimeState(
            *static_cast<lingtu_dds_MapRuntimeState*>(samples[index]),
            &decoded,
            &stamp_ns,
            &error);
        std::lock_guard<std::mutex> lock(map_scene_mutex);
        ++map_scene_health.state_received_samples;
        if (!valid) {
          ++map_scene_health.state_invalid_samples;
          map_scene_health.state_error =
              error.empty() ? "map runtime state was rejected" : error;
          continue;
        }
        const bool stale =
            decoded.state_producer_boot_id == map_state_boot_id &&
            stamp_ns <= map_state_stamp_ns;
        if (stale) {
          ++map_scene_health.state_stale_samples;
          continue;
        }
        map_state_boot_id = decoded.state_producer_boot_id;
        map_state_stamp_ns = stamp_ns;
        ++map_scene_health.state_valid_samples;
        map_scene_health.state_timestamp_s = decoded.state_timestamp_s;
        map_scene_health.state_producer_boot_id =
            std::move(decoded.state_producer_boot_id);
        map_scene_health.state_received = decoded.state_received;
        map_scene_health.state_running = decoded.state_running;
        map_scene_health.state_live = decoded.state_live;
        map_scene_health.state_required_publications_ready =
            decoded.state_required_publications_ready;
        map_scene_health.state_current_generation_published =
            decoded.state_current_generation_published;
        map_scene_health.state_capacity_limited =
            decoded.state_capacity_limited;
        map_scene_health.state_reset_epoch = decoded.state_reset_epoch;
        map_scene_health.state_observation_sequence =
            decoded.state_observation_sequence;
        map_scene_health.state_generation = decoded.state_generation;
        map_scene_health.state_scene_published_generation =
            decoded.state_scene_published_generation;
        map_scene_health.state_error = std::move(decoded.state_error);
      }
    } catch (...) {
      if (count > 0) {
        dds_return_loan(map_state_reader, samples, count);
      }
      throw;
    }
    if (count > 0) {
      checked(
          dds_return_loan(map_state_reader, samples, count),
          "dds_return_loan(map_state)");
    }
    return count > 0;
  }

  void failAckReceiver(const std::string& reason) noexcept {
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      if (ack_receiver_error.empty()) {
        ack_receiver_error = "navigation ACK receiver failed: " + reason;
      }
    }
    ack_cv.notify_all();
  }

  void ackReceiverLoop() noexcept {
    while (ack_receiver_running.load(std::memory_order_acquire)) {
      try {
        const bool received =
            takeNavigationAcks() | takeExplorationAcks() |
            takeInspectionTaskAcks() |
            takeOperatorMotionAcks() |
            takeNavigationGoalStatuses() | takeInspectionTaskEvents() |
            takeExplorationRunEvents() |
            takeNavigationStates() |
            takePathSamples(
                global_path_reader,
                global_path_mutex,
                pending_global_path,
                global_path_receive_sequence,
                "global_path") |
            takePathSamples(
                local_path_reader,
                local_path_mutex,
                pending_local_path,
                local_path_receive_sequence,
                "local_path") |
            takeMapRuntimeStates() | takeMapScenes();
        if (!received) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      } catch (const std::exception& exc) {
        failAckReceiver(exc.what());
        return;
      } catch (...) {
        failAckReceiver("unknown error");
        return;
      }
    }
  }

  ExplorationCommandReceipt writeExplorationCommand(
      ExplorationKind kind,
      const std::string& exploration_run_id,
      const std::string& session_id,
      const std::string& requested_reason,
      int timeout_ms,
      const std::string& requested_id,
      bool has_directed_target = false,
      double directed_target_x = 0.0,
      double directed_target_y = 0.0,
      double directed_target_ttl_s = 0.0) const {
    requireExplorationIdentity(exploration_run_id, session_id);
    const std::string request_id = requested_id.empty()
        ? makeExplorationRequestId(kind)
        : requested_id;
    const std::string reason = requested_reason.empty()
        ? explorationCommandKindName(kind)
        : requested_reason;
    waitForReader(exploration_writer, "exploration_command", timeout_ms);
    waitForWriter(exploration_ack_reader, "exploration_ack", timeout_ms);
    lingtu_dds_ExplorationCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.request_id = const_cast<char*>(request_id.c_str());
    message.exploration_run_id = const_cast<char*>(exploration_run_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.session_id = const_cast<char*>(session_id.c_str());
    message.has_directed_target = has_directed_target;
    message.directed_target_x = directed_target_x;
    message.directed_target_y = directed_target_y;
    message.directed_target_ttl_s = directed_target_ttl_s;
    message.reason = const_cast<char*>(reason.c_str());
    const auto sent = SteadyClock::now();
    const auto pending = registerExplorationAck(
        request_id, kind, exploration_run_id, session_id, sent);
    try {
      checked(
          dds_write(exploration_writer, static_cast<const void*>(&message)),
          "dds_write(exploration_command)");
    } catch (...) {
      unregisterExplorationAck(request_id, pending);
      throw;
    }
    const auto observation = waitForExplorationAck(
        request_id, pending, timeout_ms);
    if (!observation.correlation_valid) {
      throw std::runtime_error(
          "exploration command ACK correlation failed: " +
          (observation.reason.empty() ? std::string("unspecified") : observation.reason) +
          " [request_id=" + request_id +
          " kind=" + explorationCommandKindName(kind) + "]");
    }
    return ExplorationCommandReceipt{
        observation.accepted,
        observation.request_id,
        observation.run_id,
        observation.reason,
        observation.duplicate,
    };
  }

  InspectionTaskCommandReceipt writeInspectionTaskCommand(
      lingtu::message::InspectionCommandKind kind,
      const std::string& task_id,
      const std::string& route_id,
      std::uint64_t route_revision,
      const std::string& reason,
      int timeout_ms,
      const std::string& requested_id) const {
    if (task_id.empty()) {
      throw std::invalid_argument("inspection task id is required");
    }
    const std::string request_id = requested_id.empty()
        ? makeInspectionRequestId(kind)
        : requested_id;
    waitForReader(inspection_task_writer, "inspection_task_request", timeout_ms);
    waitForWriter(inspection_task_ack_reader, "inspection_task_ack", timeout_ms);
    lingtu_dds_InspectionTaskRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.task_id = const_cast<char*>(task_id.c_str());
    message.request_id = const_cast<char*>(request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.route_id = const_cast<char*>(route_id.c_str());
    message.route_revision = route_revision;
    message.reason = const_cast<char*>(reason.c_str());
    const auto sent = SteadyClock::now();
    const auto pending = registerInspectionAck(request_id, kind, sent, task_id);
    try {
      checked(
          dds_write(inspection_task_writer, static_cast<const void*>(&message)),
          "dds_write(inspection_task_request)");
    } catch (...) {
      unregisterInspectionAck(request_id, pending);
      throw;
    }
    const auto observation = waitForInspectionAck(request_id, pending, timeout_ms);
    InspectionTaskCommandReceipt receipt{
        observation.task_id,
        observation.request_id,
        observation.accepted,
        observation.kind,
        observation.reason,
        observation.run_id,
        observation.endpoint_stamp_s,
        "",
    };
    if (!observation.accepted) {
      throw std::runtime_error(
          "inspection task command rejected: " +
          (observation.reason.empty() ? std::string("unspecified") : observation.reason) +
          " [task_id=" + task_id +
          " request_id=" + request_id +
          " kind=" + lingtu::message::inspectionCommandKindName(kind) + "]");
    }
    return receipt;
  }

  void requireOperatorSource(
      const std::string& source_id,
      std::uint64_t source_epoch) const {
    if (source_id.empty()) {
      throw std::invalid_argument("operator motion source_id is required");
    }
    if (source_epoch == 0U) {
      throw std::invalid_argument("operator motion source_epoch is required");
    }
  }

  OperatorMotionCommandReceipt writeOperatorMotionControl(
      OperatorMotionAction action,
      const std::string& source_id,
      std::uint64_t source_epoch,
      std::uint64_t sequence,
      std::uint32_t lease_ttl_ms,
      const std::string& requested_reason,
      int timeout_ms,
      const std::string& requested_id) const {
    requireOperatorSource(source_id, source_epoch);
    if (sequence == 0U) {
      throw std::invalid_argument("operator motion sequence is required");
    }
    const std::string request_id = requested_id.empty()
        ? makeOperatorMotionRequestId(action)
        : requested_id;
    const std::string reason = requested_reason.empty()
        ? operatorMotionActionName(action)
        : requested_reason;
    waitForReader(
        operator_motion_control_writer, "operator_motion_control", timeout_ms);
    waitForWriter(operator_motion_ack_reader, "operator_motion_ack", timeout_ms);
    lingtu_dds_OperatorMotionControl message{};
    fillHeader(message.header, nowSeconds(), "");
    message.source_id = const_cast<char*>(source_id.c_str());
    message.source_epoch = source_epoch;
    message.source_sequence = sequence;
    message.request_id = const_cast<char*>(request_id.c_str());
    message.action = static_cast<std::int32_t>(action);
    message.lease_ttl_ms = lease_ttl_ms;
    message.reason = const_cast<char*>(reason.c_str());
    const auto sent = SteadyClock::now();
    const auto pending = registerOperatorMotionAck(
        request_id, action, source_id, source_epoch, sequence, sent);
    try {
      checked(
          dds_write(
              operator_motion_control_writer,
              static_cast<const void*>(&message)),
          "dds_write(operator_motion_control)");
    } catch (...) {
      unregisterOperatorMotionAck(request_id, pending);
      throw;
    }
    return waitForOperatorMotionAck(request_id, pending, timeout_ms);
  }

  void writeOperatorMotionSample(
      const std::string& source_id,
      std::uint64_t source_epoch,
      std::uint64_t sequence,
      double vx,
      double vy,
      double wz,
      bool deadman,
      std::uint32_t freshness_budget_ms,
      int timeout_ms,
      const std::string& requested_id) const {
    requireOperatorSource(source_id, source_epoch);
    if (sequence == 0U) {
      throw std::invalid_argument("operator motion sequence is required");
    }
    if (freshness_budget_ms == 0U) {
      throw std::invalid_argument(
          "operator motion freshness_budget_ms is required");
    }
    requireFinite(vx, "operator motion vx");
    requireFinite(vy, "operator motion vy");
    requireFinite(wz, "operator motion wz");
    const std::string request_id = requested_id.empty()
        ? makeOperatorMotionSampleRequestId()
        : requested_id;
    std::unique_lock<std::mutex> clock_lane_lock(clock_command_mutex);
    waitForReader(
        operator_motion_sample_writer, "operator_motion_sample", timeout_ms);
    if (!std::isfinite(
            endpoint_clock_offset_s.load(std::memory_order_relaxed))) {
      synchronizeEndpointClock(timeout_ms);
    }
    lingtu_dds_OperatorMotionSample message{};
    message.source_id = const_cast<char*>(source_id.c_str());
    message.source_epoch = source_epoch;
    message.source_sequence = sequence;
    message.request_id = const_cast<char*>(request_id.c_str());
    message.deadman = deadman;
    message.velocity.linear.x = vx;
    message.velocity.linear.y = vy;
    message.velocity.angular.z = wz;
    message.freshness_budget_ms = freshness_budget_ms;
    const double send_source_stamp_s = sourceNowSeconds();
    message.source_stamp_ns = sourceStampNs(send_source_stamp_s);
    fillHeader(message.header, send_source_stamp_s, "body");
    checked(
        dds_write(
            operator_motion_sample_writer,
            static_cast<const void*>(&message)),
        "dds_write(operator_motion_sample)");
  }

  static bool requiresEndpointClock(CommandKind kind) {
    return kind == CommandKind::ClearEstop ||
        kind == CommandKind::ResumeAutonomy;
  }

  void synchronizeEndpointClock(int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    double last_round_trip_s = 0.0;
    while (SteadyClock::now() < deadline) {
      const std::string request_id = makeRequestId(CommandKind::Stop);
      lingtu_dds_NavigationCommandRequest sync{};
      fillHeader(sync.header, nowSeconds(), "body");
      sync.client_id = const_cast<char*>(client_id.c_str());
      sync.request_id = const_cast<char*>(request_id.c_str());
      sync.kind = static_cast<std::int32_t>(CommandKind::Stop);
      sync.reason = const_cast<char*>("client_clock_sync");
      const auto sent_steady = SteadyClock::now();
      const auto pending = registerNavigationAck(
          request_id, "", CommandKind::Stop, sent_steady);
      try {
        checked(
            dds_write(command_writer, static_cast<const void*>(&sync)),
            "dds_write(nav_command_clock_sync)");
      } catch (...) {
        unregisterNavigationAck(request_id, pending);
        throw;
      }
      const auto remaining_ms = std::max<long long>(
          1,
          std::chrono::duration_cast<std::chrono::milliseconds>(
              deadline - SteadyClock::now())
              .count());
      const auto observation = waitForAck(
          request_id, pending, static_cast<int>(remaining_ms));
      if (!observation.accepted) {
        logClockEvent(
            "sync_rejected", request_id, CommandKind::Stop, &observation, 0.0);
        throw std::runtime_error(
            rejectionMessage(request_id, CommandKind::Stop, observation));
      }
      last_round_trip_s = observation.round_trip_s;
      const bool usable = updateEndpointClock(observation);
      if (usable) {
        last_sync_rtt_s.store(
            observation.round_trip_s, std::memory_order_relaxed);
        last_sync_endpoint_stamp_s.store(
            observation.endpoint_stamp_s, std::memory_order_relaxed);
        last_sync_local_receive_wall_s.store(
            observation.local_receive_wall_s, std::memory_order_relaxed);
      }
      logClockEvent(
          usable ? "sync_accepted" : "sync_resample",
          request_id,
          CommandKind::Stop,
          &observation,
          0.0);
      if (usable) {
        return;
      }
    }
    throw std::runtime_error(
        "navigation endpoint clock sync uncertainty too high: last_rtt_ms=" +
        std::to_string(last_round_trip_s * 1000.0));
  }

  NavigationCommandReceipt writeCommandReceipt(
      lingtu_dds_NavigationCommandRequest message,
      const std::string& task_id,
      const std::string& request_id,
      CommandKind kind,
      int timeout_ms) const {
    std::unique_lock<std::mutex> clock_lane_lock;
    if (requiresEndpointClock(kind)) {
      clock_lane_lock = std::unique_lock<std::mutex>(clock_command_mutex);
    }
    waitForReader(command_writer, "nav_command_request", timeout_ms);
    waitForWriter(ack_reader, "nav_command_ack", timeout_ms);
    if (requiresEndpointClock(kind) &&
        !std::isfinite(
            endpoint_clock_offset_s.load(std::memory_order_relaxed))) {
      // Header timestamps are evaluated in the endpoint's wall-clock domain.
      // WSL2 may slew CLOCK_REALTIME while a new interop process starts, and
      // remote operators are not guaranteed to share the robot's wall clock.
      // A safe Stop/ACK handshake establishes the endpoint clock without
      // weakening the endpoint's future/stale safety gates.
      synchronizeEndpointClock(timeout_ms);
    }
    std::string active_request_id = request_id;
    message.client_id = const_cast<char*>(client_id.c_str());
    message.task_id = const_cast<char*>(task_id.c_str());
    for (int attempt = 0; attempt < 2; ++attempt) {
      if (attempt > 0) {
        active_request_id = request_id + "-clock-retry-1";
      }
      message.request_id = const_cast<char*>(active_request_id.c_str());
      // DDS discovery can consume most of the command freshness budget on the
      // first request from a new client. Source time describes the actual
      // publication, so refresh it only after both endpoints are matched.
      const double send_source_stamp_s = sourceNowSeconds();
      last_send_source_stamp_s.store(
          send_source_stamp_s, std::memory_order_relaxed);
      last_send_clock_offset_s.store(
          endpointClockOffsetOrZero(), std::memory_order_relaxed);
      fillStamp(message.header, send_source_stamp_s);
      logClockEvent(
          "command_send",
          active_request_id,
          kind,
          nullptr,
          send_source_stamp_s);
      const auto sent_steady = SteadyClock::now();
      const auto pending = registerNavigationAck(
          active_request_id, task_id, kind, sent_steady);
      try {
        checked(
            dds_write(command_writer, static_cast<const void*>(&message)),
            "dds_write(nav_command_request)");
      } catch (...) {
        unregisterNavigationAck(active_request_id, pending);
        throw;
      }
      // The typed application ACK is stronger evidence than a DDS protocol ACK:
      // it proves that the endpoint received, decoded, and accepted the exact
      // request id/kind. Waiting for the protocol ACK first can turn a transient
      // middleware timeout into a false command failure even when the endpoint's
      // NavigationCommandAck is already available.
      const auto observation = waitForAck(
          active_request_id, pending, timeout_ms);
      (void)updateEndpointClock(observation);
      NavigationCommandReceipt receipt{
          task_id,
          active_request_id,
          observation.accepted,
          static_cast<std::int32_t>(kind),
          observation.reason,
          observation.endpoint_stamp_s,
          observation.accepted
              ? std::string{}
              : rejectionMessage(active_request_id, kind, observation),
      };
      if (observation.accepted) {
        return receipt;
      }
      logClockEvent(
          "command_rejected",
          active_request_id,
          kind,
          &observation,
          send_source_stamp_s);
      if (attempt == 0 && isRecoverableClockRejection(kind, observation.reason)) {
        logClockEvent(
            "clock_recovery",
            active_request_id,
            kind,
            &observation,
            send_source_stamp_s);
        // The rejected motion was never applied.  Establish a new endpoint
        // clock sample through a safe Stop before issuing one fresh request id.
        endpoint_clock_offset_s.store(
            std::numeric_limits<double>::quiet_NaN(),
            std::memory_order_relaxed);
        synchronizeEndpointClock(timeout_ms);
        continue;
      }
      return receipt;
    }
    throw std::logic_error("navigation command retry loop exhausted");
  }

  std::string writeCommand(
      lingtu_dds_NavigationCommandRequest message,
      const std::string& task_id,
      const std::string& request_id,
      CommandKind kind,
      int timeout_ms) const {
    const auto receipt = writeCommandReceipt(
        message, task_id, request_id, kind, timeout_ms);
    requireAcceptedNavigationReceipt(receipt);
    return receipt.request_id;
  }
  void writeReasonCommand(
      CommandKind kind,
      const std::string& requested_reason,
      const char* default_reason,
      int timeout_ms,
      const std::string& requested_id) const {
    const std::string reason =
        requested_reason.empty() ? std::string(default_reason) : requested_reason;
    const std::string request_id =
        requested_id.empty() ? makeRequestId(kind) : requested_id;
    lingtu_dds_NavigationCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "body");
    message.request_id = const_cast<char*>(request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.reason = const_cast<char*>(reason.c_str());
    writeCommand(message, "", request_id, kind, timeout_ms);
  }

  mutable std::mutex ack_mutex;
  mutable std::condition_variable ack_cv;
  mutable std::unordered_map<std::string, NavigationPending>
      pending_navigation_acks;
  mutable std::unordered_map<std::string, ExplorationPending>
      pending_exploration_acks;
  mutable std::unordered_map<std::string, InspectionPending>
      pending_inspection_acks;
  mutable std::unordered_map<std::string, OperatorMotionPending>
      pending_operator_motion_acks;
  mutable std::string ack_receiver_error;
  mutable std::mutex navigation_state_mutex;
  std::optional<NavigationStateSnapshot> latest_navigation_state;
  mutable std::mutex navigation_goal_status_mutex;
  std::deque<NavigationGoalStatusSnapshot> pending_goal_status_events;
  std::unordered_map<std::string, NavigationGoalStatusSnapshot>
      retained_goal_statuses;
  std::deque<std::string> retained_goal_status_order;
  std::unordered_map<std::string, NavigationGoalStatusSnapshot>
      retained_task_goal_statuses;
  std::deque<std::string> retained_task_goal_status_order;
  std::unordered_map<std::string, std::uint64_t>
      goal_status_sequences_by_boot;
  mutable std::mutex inspection_task_event_mutex;
  std::deque<InspectionTaskEventSnapshot> pending_inspection_task_events;
  std::unordered_map<std::string, std::uint64_t>
      inspection_task_event_sequences_by_boot;
  mutable std::mutex exploration_run_event_mutex;
  std::deque<ExplorationRunEventSnapshot> pending_exploration_run_events;
  std::unordered_map<std::string, std::uint64_t>
      exploration_run_event_sequences_by_boot;
  mutable std::mutex global_path_mutex;
  std::optional<PathSnapshot> pending_global_path;
  std::uint64_t global_path_receive_sequence{0U};
  mutable std::mutex local_path_mutex;
  std::optional<PathSnapshot> pending_local_path;
  std::uint64_t local_path_receive_sequence{0U};
  mutable std::mutex map_scene_mutex;
  std::optional<MapSceneSnapshot> pending_map_scene;
  MapSceneHealthSnapshot map_scene_health;
  std::string last_map_scene_boot_id;
  std::uint64_t last_map_scene_reset_epoch{0U};
  std::uint64_t last_map_scene_generation{0U};
  std::uint64_t map_scene_receive_sequence{0U};
  std::string map_state_boot_id;
  std::int64_t map_state_stamp_ns{0};
  std::atomic<bool> ack_receiver_running{true};
  std::thread ack_receiver;
  mutable std::mutex clock_command_mutex;
  dds_entity_t participant{0};
  dds_entity_t publisher{0};
  dds_entity_t subscriber{0};
  dds_entity_t command_writer{0};
  dds_entity_t ack_reader{0};
  dds_entity_t exploration_writer{0};
  dds_entity_t exploration_ack_reader{0};
  dds_entity_t exploration_run_event_reader{0};
  dds_entity_t inspection_task_writer{0};
  dds_entity_t inspection_task_ack_reader{0};
  dds_entity_t inspection_task_event_reader{0};
  dds_entity_t operator_motion_control_writer{0};
  dds_entity_t operator_motion_sample_writer{0};
  dds_entity_t operator_motion_ack_reader{0};
  dds_entity_t navigation_goal_status_reader{0};
  dds_entity_t navigation_state_reader{0};
  dds_entity_t global_path_reader{0};
  dds_entity_t local_path_reader{0};
  dds_entity_t map_scene_reader{0};
  dds_entity_t map_state_reader{0};
  const std::string client_id{makeClientId()};
  bool diagnostics_enabled{diagnosticsEnabled()};
  mutable std::atomic<double> endpoint_clock_offset_s{
      std::numeric_limits<double>::quiet_NaN()};
  mutable std::atomic<double> last_sync_rtt_s{-0.001};
  mutable std::atomic<double> last_sync_endpoint_stamp_s{0.0};
  mutable std::atomic<double> last_sync_local_receive_wall_s{0.0};
  mutable std::atomic<double> last_send_source_stamp_s{0.0};
  mutable std::atomic<double> last_send_clock_offset_s{0.0};
};

Client::Client(int domain_id)
    : impl_(std::make_unique<Impl>(domain_id)),
      navigation_(*this),
      exploration_(*this),
      inspection_(*this),
      operator_motion_(*this) {}

Client::~Client() = default;

std::optional<NavigationStateSnapshot> Client::latestNavigationState() const {
  std::lock_guard<std::mutex> lock(impl_->navigation_state_mutex);
  return impl_->latest_navigation_state;
}

bool Client::takeNavigationGoalStatus(NavigationGoalStatusSnapshot* status) {
  if (status == nullptr) {
    throw std::invalid_argument("navigation goal status output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->navigation_goal_status_mutex);
  if (impl_->pending_goal_status_events.empty()) {
    return false;
  }
  *status = std::move(impl_->pending_goal_status_events.front());
  impl_->pending_goal_status_events.pop_front();
  return true;
}

bool Client::takeInspectionTaskEvent(InspectionTaskEventSnapshot* event) {
  if (event == nullptr) {
    throw std::invalid_argument("inspection task event output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->inspection_task_event_mutex);
  if (impl_->pending_inspection_task_events.empty()) {
    return false;
  }
  *event = std::move(impl_->pending_inspection_task_events.front());
  impl_->pending_inspection_task_events.pop_front();
  return true;
}

bool Client::takeExplorationRunEvent(ExplorationRunEventSnapshot* event) {
  if (event == nullptr) {
    throw std::invalid_argument("exploration run event output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->exploration_run_event_mutex);
  if (impl_->pending_exploration_run_events.empty()) {
    return false;
  }
  *event = std::move(impl_->pending_exploration_run_events.front());
  impl_->pending_exploration_run_events.pop_front();
  return true;
}

std::optional<NavigationGoalStatusSnapshot> Client::navigationGoalStatus(
    const std::string& request_id) const {
  if (request_id.empty()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(impl_->navigation_goal_status_mutex);
  const auto found = impl_->retained_goal_statuses.find(request_id);
  if (found == impl_->retained_goal_statuses.end()) {
    return std::nullopt;
  }
  return found->second;
}

std::optional<NavigationGoalStatusSnapshot> Client::navigationTaskStatus(
    const std::string& task_id) const {
  if (task_id.empty()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(impl_->navigation_goal_status_mutex);
  const auto found = impl_->retained_task_goal_statuses.find(task_id);
  if (found == impl_->retained_task_goal_statuses.end()) {
    return std::nullopt;
  }
  return found->second;
}

bool Client::takeGlobalPath(PathSnapshot* path) {
  if (path == nullptr) {
    throw std::invalid_argument("global path output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->global_path_mutex);
  if (!impl_->pending_global_path.has_value()) {
    return false;
  }
  *path = std::move(*impl_->pending_global_path);
  impl_->pending_global_path.reset();
  return true;
}

bool Client::takeLocalPath(PathSnapshot* path) {
  if (path == nullptr) {
    throw std::invalid_argument("local path output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->local_path_mutex);
  if (!impl_->pending_local_path.has_value()) {
    return false;
  }
  *path = std::move(*impl_->pending_local_path);
  impl_->pending_local_path.reset();
  return true;
}

bool Client::takeMapScene(MapSceneSnapshot* scene) {
  if (scene == nullptr) {
    throw std::invalid_argument("map scene output is null");
  }
  std::lock_guard<std::mutex> lock(impl_->map_scene_mutex);
  if (!impl_->pending_map_scene.has_value()) {
    return false;
  }
  *scene = std::move(*impl_->pending_map_scene);
  impl_->pending_map_scene.reset();
  return true;
}

MapSceneHealthSnapshot Client::mapSceneHealth() const {
  std::lock_guard<std::mutex> lock(impl_->map_scene_mutex);
  auto health = impl_->map_scene_health;
  health.pending = impl_->pending_map_scene.has_value();
  return health;
}

NavigationCommandReceipt Client::NavigationCommands::startTask(
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms,
    const std::string& requested_task_id,
    const std::string& requested_id) {
  requireFinite(x, "goal x");
  requireFinite(y, "goal y");
  requireFinite(z, "goal z");
  requireFinite(yaw, "goal yaw");
  const std::string task_id = requested_task_id.empty()
      ? makeTaskId(CommandKind::Goal)
      : requested_task_id;
  const std::string request_id =
      requested_id.empty() ? makeRequestId(CommandKind::Goal) : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.task_id = const_cast<char*>(task_id.c_str());
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::Goal);
  message.goal.position.x = x;
  message.goal.position.y = y;
  message.goal.position.z = z;
  message.goal.orientation = quaternionFromYaw(yaw);
  message.reason = const_cast<char*>("");
  return owner_.impl_->writeCommandReceipt(
      message, task_id, request_id, CommandKind::Goal, timeout_ms);
}

NavigationCommandReceipt Client::NavigationCommands::cancelTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  if (task_id.empty()) {
    throw std::invalid_argument("navigation cancel task_id is required");
  }
  const std::string text = reason.empty() ? "cancel" : reason;
  const std::string request_id =
      requested_id.empty() ? makeRequestId(CommandKind::TaskCancel) : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.task_id = const_cast<char*>(task_id.c_str());
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::TaskCancel);
  message.reason = const_cast<char*>(text.c_str());
  return owner_.impl_->writeCommandReceipt(
      message, task_id, request_id, CommandKind::TaskCancel, timeout_ms);
}

NavigationCommandReceipt Client::NavigationCommands::pauseTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  if (task_id.empty()) {
    throw std::invalid_argument("navigation pause task_id is required");
  }
  const std::string text = reason.empty() ? "operator_pause" : reason;
  const std::string request_id = requested_id.empty()
      ? makeRequestId(CommandKind::TaskPause)
      : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.task_id = const_cast<char*>(task_id.c_str());
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::TaskPause);
  message.reason = const_cast<char*>(text.c_str());
  return owner_.impl_->writeCommandReceipt(
      message, task_id, request_id, CommandKind::TaskPause, timeout_ms);
}

NavigationCommandReceipt Client::NavigationCommands::resumeTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  if (task_id.empty()) {
    throw std::invalid_argument("navigation resume task_id is required");
  }
  const std::string text = reason.empty() ? "operator_resume" : reason;
  const std::string request_id = requested_id.empty()
      ? makeRequestId(CommandKind::TaskResume)
      : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.task_id = const_cast<char*>(task_id.c_str());
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::TaskResume);
  message.reason = const_cast<char*>(text.c_str());
  return owner_.impl_->writeCommandReceipt(
      message, task_id, request_id, CommandKind::TaskResume, timeout_ms);
}

void Client::NavigationCommands::stop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::Stop, reason, "stop", timeout_ms, requested_id);
}

void Client::NavigationCommands::estop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::Estop, reason, "estop", timeout_ms, requested_id);
}

void Client::NavigationCommands::clearEstop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::ClearEstop,
      reason,
      "clear_estop",
      timeout_ms,
      requested_id);
}

void Client::NavigationCommands::resumeAutonomy(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::ResumeAutonomy,
      reason,
      "resume_autonomy",
      timeout_ms,
      requested_id);
}

ExplorationCommandReceipt Client::ExplorationCommands::start(
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kStart,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_start" : reason,
      timeout_ms,
      request_id);
}

ExplorationCommandReceipt Client::ExplorationCommands::pause(
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kPause,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_pause" : reason,
      timeout_ms,
      request_id);
}

ExplorationCommandReceipt Client::ExplorationCommands::resume(
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kResume,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_resume" : reason,
      timeout_ms,
      request_id);
}

ExplorationCommandReceipt Client::ExplorationCommands::stop(
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kStop,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_stop" : reason,
      timeout_ms,
      request_id);
}

ExplorationCommandReceipt Client::ExplorationCommands::setDirectedTarget(
    double x,
    double y,
    double ttl_s,
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  requireDirectedTarget(x, y, ttl_s);
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kSetDirectedTarget,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_directed_explore" : reason,
      timeout_ms,
      request_id,
      true,
      x,
      y,
      ttl_s);
}

ExplorationCommandReceipt Client::ExplorationCommands::clearDirectedTarget(
    const std::string& exploration_run_id,
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  requireDirectedTargetSessionId(session_id);
  return owner_.impl_->writeExplorationCommand(
      ExplorationKind::kClearDirectedTarget,
      exploration_run_id,
      session_id,
      reason.empty() ? "operator_clear_directed_explore" : reason,
      timeout_ms,
      request_id);
}

InspectionTaskCommandReceipt Client::InspectionCommands::startTask(
    const std::string& task_id,
    const std::string& route_id,
    std::uint64_t route_revision,
    int timeout_ms,
    const std::string& request_id) {
  if (route_id.empty()) throw std::invalid_argument("inspection route id is required");
  return owner_.impl_->writeInspectionTaskCommand(
      lingtu::message::InspectionCommandKind::kStart,
      task_id,
      route_id,
      route_revision,
      "",
      timeout_ms,
      request_id);
}

InspectionTaskCommandReceipt Client::InspectionCommands::pauseTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeInspectionTaskCommand(
      lingtu::message::InspectionCommandKind::kPause,
      task_id,
      "",
      0U,
      reason.empty() ? "operator_pause" : reason,
      timeout_ms,
      request_id);
}

InspectionTaskCommandReceipt Client::InspectionCommands::resumeTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeInspectionTaskCommand(
      lingtu::message::InspectionCommandKind::kResume,
      task_id,
      "",
      0U,
      reason.empty() ? "operator_resume" : reason,
      timeout_ms,
      request_id);
}

InspectionTaskCommandReceipt Client::InspectionCommands::cancelTask(
    const std::string& task_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeInspectionTaskCommand(
      lingtu::message::InspectionCommandKind::kCancel,
      task_id,
      "",
      0U,
      reason.empty() ? "operator_cancel" : reason,
      timeout_ms,
      request_id);
}

OperatorMotionCommandReceipt
Client::OperatorMotionCommands::claimWithReceipt(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    std::uint32_t lease_ttl_ms,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeOperatorMotionControl(
      OperatorMotionAction::Claim,
      source_id,
      source_epoch,
      sequence,
      lease_ttl_ms,
      "operator_claim",
      timeout_ms,
      request_id);
}
void Client::OperatorMotionCommands::claim(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    std::uint32_t lease_ttl_ms,
    int timeout_ms,
    const std::string& request_id) {
  requireAcceptedOperatorMotionReceipt(claimWithReceipt(
      source_id,
      source_epoch,
      sequence,
      lease_ttl_ms,
      timeout_ms,
      request_id));
}

void Client::OperatorMotionCommands::sample(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    double vx,
    double vy,
    double wz,
    bool deadman,
    std::uint32_t freshness_budget_ms,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeOperatorMotionSample(
      source_id,
      source_epoch,
      sequence,
      vx,
      vy,
      wz,
      deadman,
      freshness_budget_ms,
      timeout_ms,
      request_id);
}

OperatorMotionCommandReceipt
Client::OperatorMotionCommands::holdWithReceipt(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeOperatorMotionControl(
      OperatorMotionAction::Hold,
      source_id,
      source_epoch,
      sequence,
      0U,
      reason.empty() ? "operator_hold" : reason,
      timeout_ms,
      request_id);
}
void Client::OperatorMotionCommands::hold(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  requireAcceptedOperatorMotionReceipt(holdWithReceipt(
      source_id,
      source_epoch,
      sequence,
      reason,
      timeout_ms,
      request_id));
}

OperatorMotionCommandReceipt
Client::OperatorMotionCommands::releaseWithReceipt(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  return owner_.impl_->writeOperatorMotionControl(
      OperatorMotionAction::Release,
      source_id,
      source_epoch,
      sequence,
      0U,
      reason.empty() ? "operator_release" : reason,
      timeout_ms,
      request_id);
}
void Client::OperatorMotionCommands::release(
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  requireAcceptedOperatorMotionReceipt(releaseWithReceipt(
      source_id,
      source_epoch,
      sequence,
      reason,
      timeout_ms,
      request_id));
}

}  // namespace lingtu::nav::commands
