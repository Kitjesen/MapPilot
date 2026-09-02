#include "dds.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace lingtu::maps::mapd {
namespace {

constexpr std::uint8_t kPointFieldFloat32 = 7U;
constexpr std::size_t kTakeBatch = 8U;

double NowSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::int64_t HeaderStampNs(const lingtu_dds_Header &header) {
  if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) {
    return 0;
  }
  return static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(header.stamp.nanosec);
}

void FillHeader(lingtu_dds_Header &header, std::int64_t stamp_ns, const char *frame_id) {
  if (stamp_ns <= 0) {
    stamp_ns = static_cast<std::int64_t>(NowSeconds() * 1.0e9);
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  header.frame_id = const_cast<char *>(frame_id);
}

bool CopyBoundedString(const char *value, std::size_t max_bytes, const char *field,
                       std::string *out, std::string *error) {
  if (out == nullptr || error == nullptr || max_bytes == 0U) {
    return false;
  }
  if (value == nullptr) {
    out->clear();
    return true;
  }
  std::size_t length = 0U;
  while (length <= max_bytes && value[length] != '\0') {
    ++length;
  }
  if (length > max_bytes) {
    *error = std::string(field) + " exceeds mapd string limit";
    return false;
  }
  out->assign(value, length);
  return true;
}

std::optional<ActivationOperation>
DecodeActivationOperation(lingtu_dds_MapActivationOperation operation) {
  switch (operation) {
    case lingtu_dds_MAP_ACTIVATION_STAGE:
      return ActivationOperation::kStage;
    case lingtu_dds_MAP_ACTIVATION_RESTORE:
      return ActivationOperation::kRestore;
    case lingtu_dds_MAP_ACTIVATION_VERIFY:
      return ActivationOperation::kVerify;
  }
  return std::nullopt;
}

lingtu_dds_MapActivationOperation EncodeActivationOperation(ActivationOperation operation) {
  switch (operation) {
    case ActivationOperation::kStage:
      return lingtu_dds_MAP_ACTIVATION_STAGE;
    case ActivationOperation::kRestore:
      return lingtu_dds_MAP_ACTIVATION_RESTORE;
    case ActivationOperation::kVerify:
      return lingtu_dds_MAP_ACTIVATION_VERIFY;
  }
  return lingtu_dds_MAP_ACTIVATION_VERIFY;
}

bool DecodeMapIdentity(const lingtu_dds_MapIdentity &message, const DdsLimits &limits,
                       const char *field, MapIdentity *identity, std::string *error) {
  constexpr std::uint32_t kMaxIdentityArtifacts = 32U;
  if (identity == nullptr || error == nullptr ||
      message.artifacts._maximum < message.artifacts._length ||
      message.artifacts._length > kMaxIdentityArtifacts ||
      (message.artifacts._length > 0U && message.artifacts._buffer == nullptr)) {
    *error = std::string(field) + " artifact sequence is malformed";
    return false;
  }
  identity->present = message.present;
  identity->content_epoch = message.content_epoch;
  if (!CopyBoundedString(message.map_id, limits.max_string_bytes, field, &identity->map_id,
                         error) ||
      !CopyBoundedString(message.frame_id, limits.max_string_bytes, field, &identity->frame_id,
                         error)) {
    return false;
  }
  identity->artifacts.clear();
  identity->artifacts.reserve(message.artifacts._length);
  for (std::uint32_t index = 0U; index < message.artifacts._length; ++index) {
    const auto &source = message.artifacts._buffer[index];
    ArtifactIdentity artifact;
    if (!CopyBoundedString(source.type, limits.max_string_bytes, field, &artifact.type, error) ||
        !CopyBoundedString(source.uri, limits.max_string_bytes, field, &artifact.uri, error)) {
      return false;
    }
    identity->artifacts.push_back(std::move(artifact));
  }
  return true;
}

void FillMapIdentity(const MapIdentity &identity, lingtu_dds_MapIdentity *message,
                     std::vector<lingtu_dds_MapArtifactIdentity> *artifacts) {
  message->present = identity.present;
  message->map_id = const_cast<char *>(identity.map_id.c_str());
  message->content_epoch = identity.content_epoch;
  message->frame_id = const_cast<char *>(identity.frame_id.c_str());
  artifacts->resize(identity.artifacts.size());
  for (std::size_t index = 0U; index < identity.artifacts.size(); ++index) {
    const auto &source = identity.artifacts[index];
    auto &destination = (*artifacts)[index];
    destination.type = const_cast<char *>(source.type.c_str());
    destination.uri = const_cast<char *>(source.uri.c_str());
  }
  message->artifacts._maximum = static_cast<std::uint32_t>(artifacts->size());
  message->artifacts._length = static_cast<std::uint32_t>(artifacts->size());
  message->artifacts._buffer = artifacts->data();
  message->artifacts._release = false;
}

dds_entity_t Checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::string MakeProducerBootId() {
  std::string host_boot_id;
#if !defined(_WIN32)
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::getline(input, host_boot_id);
#endif
  if (host_boot_id.empty()) {
    host_boot_id = "host";
  }
#if defined(_WIN32)
  const int process_id = _getpid();
#else
  const int process_id = getpid();
#endif
  const auto started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
  return host_boot_id + ":" + std::to_string(process_id) + ":" + std::to_string(started_ns);
}

struct FieldOffsets {
  std::uint32_t x{0U};
  std::uint32_t y{0U};
  std::uint32_t z{0U};
  bool has_x{false};
  bool has_y{false};
  bool has_z{false};
};

bool FindFieldOffsets(const lingtu_dds_PointCloud2 &cloud, const DdsLimits &limits,
                      FieldOffsets *offsets, std::string *error) {
  if (offsets == nullptr || cloud.fields._maximum < cloud.fields._length ||
      (cloud.fields._length > 0U && cloud.fields._buffer == nullptr)) {
    *error = "point field sequence is malformed";
    return false;
  }
  if (cloud.fields._length > limits.max_point_fields) {
    *error = "point field count exceeds mapd limit";
    return false;
  }
  for (std::uint32_t index = 0U; index < cloud.fields._length; ++index) {
    const auto &field = cloud.fields._buffer[index];
    std::string name;
    if (!CopyBoundedString(field.name, limits.max_string_bytes, "point field name", &name, error)) {
      return false;
    }
    if (name != "x" && name != "y" && name != "z") {
      continue;
    }
    if (field.datatype != kPointFieldFloat32 || field.count != 1U ||
        field.offset > cloud.point_step || sizeof(float) > cloud.point_step - field.offset) {
      *error = "x/y/z fields must be scalar float32 values";
      return false;
    }
    if (name == "x") {
      offsets->x = field.offset;
      offsets->has_x = true;
    } else if (name == "y") {
      offsets->y = field.offset;
      offsets->has_y = true;
    } else {
      offsets->z = field.offset;
      offsets->has_z = true;
    }
  }
  if (!offsets->has_x || !offsets->has_y || !offsets->has_z) {
    *error = "point cloud is missing scalar float32 x/y/z fields";
    return false;
  }
  return true;
}

bool DecodeCloud(const lingtu_dds_PointCloud2 &cloud, const DdsLimits &limits, OwnedPointCloud *out,
                 std::string *error) {
  if (out == nullptr || cloud.is_bigendian || cloud.height == 0U || cloud.width == 0U ||
      cloud.point_step == 0U) {
    *error = "point cloud geometry or endianness is unsupported";
    return false;
  }
  if (cloud.point_step > limits.max_point_step) {
    *error = "point cloud point_step exceeds mapd limit";
    return false;
  }
  if (cloud.data._maximum < cloud.data._length || cloud.data._length > limits.max_cloud_bytes) {
    *error = "point cloud byte sequence exceeds mapd limit or is malformed";
    return false;
  }
  const std::size_t rows = cloud.height;
  const std::size_t columns = cloud.width;
  if (columns > limits.max_points_per_observation ||
      rows > limits.max_points_per_observation / columns) {
    *error = "point cloud exceeds mapd point limit";
    return false;
  }
  const std::size_t point_count = rows * columns;
  const std::size_t point_step = cloud.point_step;
  if (columns > std::numeric_limits<std::size_t>::max() / point_step) {
    *error = "point cloud row size overflows";
    return false;
  }
  const std::size_t minimum_row_step = columns * point_step;
  const std::size_t row_step = cloud.row_step == 0U ? minimum_row_step : cloud.row_step;
  if (row_step < minimum_row_step || row_step > limits.max_cloud_bytes ||
      rows - 1U > (std::numeric_limits<std::size_t>::max() - minimum_row_step) / row_step) {
    *error = "point cloud row stride is invalid";
    return false;
  }
  const std::size_t required = (rows - 1U) * row_step + minimum_row_step;
  if (required > limits.max_cloud_bytes) {
    *error = "point cloud decoded footprint exceeds mapd byte limit";
    return false;
  }
  if (cloud.data._maximum < cloud.data._length || cloud.data._length < required ||
      cloud.data._buffer == nullptr) {
    *error = "point cloud byte storage is truncated";
    return false;
  }

  FieldOffsets offsets;
  if (!FindFieldOffsets(cloud, limits, &offsets, error)) {
    return false;
  }
  if (!CopyBoundedString(cloud.header.frame_id, limits.max_string_bytes, "scan frame_id",
                         &out->frame_id, error)) {
    return false;
  }
  out->stamp_ns = HeaderStampNs(cloud.header);
  out->layout = CloudLayout::kXyzF32SoA;
  out->point_count = point_count;
  out->x.resize(point_count);
  out->y.resize(point_count);
  out->z.resize(point_count);
  std::size_t output_index = 0U;
  for (std::size_t row = 0U; row < rows; ++row) {
    const std::uint8_t *row_data = cloud.data._buffer + row * row_step;
    for (std::size_t column = 0U; column < columns; ++column) {
      const std::uint8_t *point = row_data + column * point_step;
      std::memcpy(&out->x[output_index], point + offsets.x, sizeof(float));
      std::memcpy(&out->y[output_index], point + offsets.y, sizeof(float));
      std::memcpy(&out->z[output_index], point + offsets.z, sizeof(float));
      ++output_index;
    }
  }
  return true;
}

std::optional<Observation> DecodeObservation(const lingtu_dds_MapObservation &message,
                                             const DdsLimits &limits, std::string *error) {
  Observation observation;
  observation.reset_epoch = message.reset_epoch;
  observation.sequence = message.observation_sequence;
  observation.stamp_ns = HeaderStampNs(message.header);
  if (!CopyBoundedString(message.header.frame_id, limits.max_string_bytes, "map frame_id",
                         &observation.map_frame, error) ||
      !CopyBoundedString(message.sensor_frame, limits.max_string_bytes, "sensor_frame",
                         &observation.sensor_frame, error) ||
      !CopyBoundedString(message.pose_state, limits.max_string_bytes, "pose_state",
                         &observation.pose_state, error) ||
      !CopyBoundedString(message.pose_reason, limits.max_string_bytes, "pose_reason",
                         &observation.pose_reason, error)) {
    return std::nullopt;
  }
  observation.map_sensor.x = message.map_sensor.translation.x;
  observation.map_sensor.y = message.map_sensor.translation.y;
  observation.map_sensor.z = message.map_sensor.translation.z;
  observation.map_sensor.qx = message.map_sensor.rotation.x;
  observation.map_sensor.qy = message.map_sensor.rotation.y;
  observation.map_sensor.qz = message.map_sensor.rotation.z;
  observation.map_sensor.qw = message.map_sensor.rotation.w;
  observation.sensor_origin_x_m = static_cast<float>(message.sensor_origin.x);
  observation.sensor_origin_y_m = static_cast<float>(message.sensor_origin.y);
  observation.sensor_origin_z_m = static_cast<float>(message.sensor_origin.z);
  observation.pose_quality = std::min(message.pose_confidence, message.localization_quality);
  if (!DecodeCloud(message.scan, limits, &observation.scan, error)) {
    return std::nullopt;
  }
  if (observation.scan.frame_id.empty()) {
    observation.scan.frame_id = observation.sensor_frame;
  }
  if (observation.scan.stamp_ns == 0) {
    observation.scan.stamp_ns = observation.stamp_ns;
  }
  return observation;
}

bool NewerIdentity(const Observation &lhs, const Observation &rhs) {
  return lhs.reset_epoch > rhs.reset_epoch ||
         (lhs.reset_epoch == rhs.reset_epoch && lhs.sequence > rhs.sequence);
}

bool ReadOwnedPoint(const OwnedPointCloud &cloud, std::size_t index, float *x, float *y, float *z,
                    float *intensity) {
  const auto view = cloud.View();
  switch (view.layout) {
    case CloudLayout::kXyzF32Interleaved:
    case CloudLayout::kXyziF32Interleaved: {
      const std::size_t stride = view.layout == CloudLayout::kXyziF32Interleaved ? 4U : 3U;
      const std::size_t offset = index * stride;
      if (view.interleaved.data == nullptr || offset + stride - 1U >= view.interleaved.size) {
        return false;
      }
      *x = view.interleaved.data[offset];
      *y = view.interleaved.data[offset + 1U];
      *z = view.interleaved.data[offset + 2U];
      *intensity = stride == 4U ? view.interleaved.data[offset + 3U] : 0.0F;
      return true;
    }
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA:
      if (view.x.data == nullptr || view.y.data == nullptr || view.z.data == nullptr ||
          index >= view.x.size || index >= view.y.size || index >= view.z.size) {
        return false;
      }
      *x = view.x.data[index];
      *y = view.y.data[index];
      *z = view.z.data[index];
      *intensity = view.intensity.data != nullptr && index < view.intensity.size
                       ? view.intensity.data[index]
                       : 0.0F;
      return true;
  }
  return false;
}

OwnedPointCloud AccumulatedAsCloud(const BlockGridSnapshot &grid) {
  OwnedPointCloud cloud;
  cloud.frame_id = grid.frame_id;
  cloud.stamp_ns = grid.stamp_ns;
  cloud.layout = CloudLayout::kXyziF32Interleaved;
  cloud.interleaved.reserve(grid.Size() * 4U);
  for (std::size_t index = 0U; index < grid.Size(); ++index) {
    if (grid.occupancy_probability[index] < 0.55F) {
      continue;
    }
    cloud.interleaved.push_back(grid.center_x_m[index]);
    cloud.interleaved.push_back(grid.center_y_m[index]);
    cloud.interleaved.push_back(grid.center_z_m[index]);
    cloud.interleaved.push_back(grid.occupancy_probability[index]);
  }
  cloud.point_count = cloud.interleaved.size() / 4U;
  return cloud;
}

struct CloudMessage {
  lingtu_dds_MapCloudLayer message{};
  std::string layer;
  std::string frame;
  std::array<lingtu_dds_PointField, 4U> fields{};
  std::vector<std::uint8_t> bytes;

  CloudMessage(const char *layer_name, const OwnedPointCloud &source, std::uint64_t reset_epoch,
               std::uint64_t observation_sequence, std::uint64_t generation, bool live,
               bool include_intensity)
      : layer(layer_name), frame(source.frame_id.empty() ? "map" : source.frame_id) {
    const std::size_t stride = include_intensity ? 4U : 3U;
    bytes.reserve(source.point_count * stride * sizeof(float));
    for (std::size_t index = 0U; index < source.point_count; ++index) {
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      float intensity = 0.0F;
      if (!ReadOwnedPoint(source, index, &x, &y, &z, &intensity) || !std::isfinite(x) ||
          !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const std::size_t old_size = bytes.size();
      bytes.resize(old_size + stride * sizeof(float));
      std::memcpy(bytes.data() + old_size, &x, sizeof(float));
      std::memcpy(bytes.data() + old_size + sizeof(float), &y, sizeof(float));
      std::memcpy(bytes.data() + old_size + 2U * sizeof(float), &z, sizeof(float));
      if (include_intensity) {
        std::memcpy(bytes.data() + old_size + 3U * sizeof(float), &intensity, sizeof(float));
      }
    }

    FillHeader(message.header, source.stamp_ns, frame.c_str());
    message.layer = const_cast<char *>(layer.c_str());
    message.reset_epoch = reset_epoch;
    message.observation_sequence = observation_sequence;
    message.generation = generation;
    message.live = live;
    FillHeader(message.cloud.header, source.stamp_ns, frame.c_str());
    message.cloud.height = 1U;
    message.cloud.width = static_cast<std::uint32_t>(bytes.size() / (stride * sizeof(float)));
    message.cloud.is_bigendian = false;
    message.cloud.point_step = static_cast<std::uint32_t>(stride * sizeof(float));
    message.cloud.row_step = message.cloud.width * message.cloud.point_step;
    message.cloud.is_dense = false;

    const char *names[4] = {"x", "y", "z", "intensity"};
    for (std::size_t index = 0U; index < stride; ++index) {
      fields[index].name = const_cast<char *>(names[index]);
      fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      fields[index].datatype = kPointFieldFloat32;
      fields[index].count = 1U;
    }
    message.cloud.fields._maximum = static_cast<std::uint32_t>(stride);
    message.cloud.fields._length = static_cast<std::uint32_t>(stride);
    message.cloud.fields._buffer = fields.data();
    message.cloud.fields._release = false;
    message.cloud.data._maximum = static_cast<std::uint32_t>(bytes.size());
    message.cloud.data._length = static_cast<std::uint32_t>(bytes.size());
    message.cloud.data._buffer = bytes.data();
    message.cloud.data._release = false;
  }
};

struct CollisionMessage {
  lingtu_dds_MapCollisionLayer message{};
  std::string frame;

  CollisionMessage(const Snapshot &snapshot, bool live)
      : frame(snapshot.frame_id.empty() ? "map" : snapshot.frame_id) {
    FillHeader(message.header, snapshot.stamp_ns, frame.c_str());
    message.reset_epoch = snapshot.reset_epoch;
    message.observation_sequence = snapshot.sequence;
    message.generation = snapshot.collision.generation;
    message.live = live;
    message.resolution = snapshot.collision.resolution_m;
    message.size_x = static_cast<std::uint32_t>(snapshot.collision.size_x);
    message.size_y = static_cast<std::uint32_t>(snapshot.collision.size_y);
    message.size_z = static_cast<std::uint32_t>(snapshot.collision.size_z);
    message.aabb_min.x = snapshot.collision.min_x_m;
    message.aabb_min.y = snapshot.collision.min_y_m;
    message.aabb_min.z = snapshot.collision.min_z_m;
    message.aabb_max.x = snapshot.collision.max_x_m;
    message.aabb_max.y = snapshot.collision.max_y_m;
    message.aabb_max.z = snapshot.collision.max_z_m;
    message.complete = snapshot.collision.complete;
    message.inflated_occupied_bits._maximum =
        static_cast<std::uint32_t>(snapshot.collision.occupied_bits.size());
    message.inflated_occupied_bits._length =
        static_cast<std::uint32_t>(snapshot.collision.occupied_bits.size());
    message.inflated_occupied_bits._buffer =
        const_cast<std::uint8_t *>(snapshot.collision.occupied_bits.data());
    message.inflated_occupied_bits._release = false;
  }
};

struct GridMessage {
  lingtu_dds_MapGrid message{};
  std::string layer;
  std::string frame;

  GridMessage(const char *layer_name, const layers::Grid2D &grid, const Snapshot &snapshot,
              bool live)
      : layer(layer_name), frame(snapshot.frame_id.empty() ? "map" : snapshot.frame_id) {
    FillHeader(message.header, snapshot.stamp_ns, frame.c_str());
    message.layer = const_cast<char *>(layer.c_str());
    message.info.map_load_time = message.header.stamp;
    message.info.resolution = static_cast<float>(grid.resolution);
    message.info.width = static_cast<std::uint32_t>(std::max(0, grid.cols));
    message.info.height = static_cast<std::uint32_t>(std::max(0, grid.rows));
    message.info.origin.position.x = grid.originX;
    message.info.origin.position.y = grid.originY;
    message.info.origin.orientation.w = 1.0;
    message.data._maximum = static_cast<std::uint32_t>(grid.data.size());
    message.data._length = static_cast<std::uint32_t>(grid.data.size());
    message.data._buffer = const_cast<float *>(grid.data.data());
    message.data._release = false;
    message.reset_epoch = snapshot.reset_epoch;
    message.observation_sequence = snapshot.sequence;
    message.generation = snapshot.generation;
    message.live = live;
  }
};

bool AddBytes(std::size_t value, std::size_t *total) {
  if (total == nullptr || value > std::numeric_limits<std::size_t>::max() - *total) {
    return false;
  }
  *total += value;
  return true;
}

bool AddPointCloudBytes(const OwnedPointCloud &cloud, std::size_t stride, std::size_t *total) {
  const std::size_t bytes_per_point = stride * sizeof(float);
  if (cloud.point_count > std::numeric_limits<std::size_t>::max() / bytes_per_point) {
    return false;
  }
  return AddBytes(cloud.point_count * bytes_per_point, total);
}

std::optional<std::size_t> EstimateSceneBytes(const State &state, const Snapshot &snapshot) {
  std::size_t total = 0U;
  if ((state.extended_layers_enabled &&
       (!AddPointCloudBytes(snapshot.live_cloud, 3U, &total) ||
        !AddPointCloudBytes(snapshot.voxel_cloud, 3U, &total))) ||
      snapshot.accumulated_cloud.Size() >
          std::numeric_limits<std::size_t>::max() / (4U * sizeof(float)) ||
      !AddBytes(snapshot.accumulated_cloud.Size() * 4U * sizeof(float), &total)) {
    return std::nullopt;
  }
  const std::array<std::size_t, 3U> grid_cells{snapshot.occupancy.data.size(),
                                               snapshot.elevation.minZ.data.size(),
                                               snapshot.esdf.distance.data.size()};
  for (const std::size_t cells : grid_cells) {
    if (cells > std::numeric_limits<std::size_t>::max() / sizeof(float) ||
        !AddBytes(cells * sizeof(float), &total)) {
      return std::nullopt;
    }
  }
  return total;
}

}  // namespace

struct Dds::Impl {
  explicit Impl(int domain_id, DdsLimits configured_limits)
      : limits(std::move(configured_limits)), producer_boot_id(MakeProducerBootId()) {
    if (limits.max_points_per_observation == 0U || limits.max_cloud_bytes == 0U ||
        limits.max_point_fields < 3U || limits.max_point_step < 3U * sizeof(float) ||
        limits.max_string_bytes == 0U || limits.max_scene_bytes == 0U ||
        limits.max_cloud_bytes > std::numeric_limits<std::uint32_t>::max() ||
        limits.max_scene_bytes > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("mapd DDS limits are invalid");
    }
    participant =
        Checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(mapd)");
    subscriber = Checked(dds_create_subscriber(participant, nullptr, nullptr),
                         "dds_create_subscriber(mapd)");
    publisher =
        Checked(dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher(mapd)");
    observation_reader = Reader(lingtu::message::kSlamMapObservation.dds_topic.data(),
                                &lingtu_dds_MapObservation_desc);
    activation_request_reader = Reader(lingtu::message::kMapsActivationRequest.dds_topic.data(),
                                       &lingtu_dds_MapActivationRequest_desc);
    snapshot_ack_reader = Reader(lingtu::message::kSlamMapSnapshotAck.dds_topic.data(),
                                 &lingtu_dds_SlamMapSnapshotAck_desc);
    state_writer =
        Writer(lingtu::message::kMapsState.dds_topic.data(), &lingtu_dds_MapRuntimeState_desc);
    live_cloud_writer =
        Writer(lingtu::message::kMapsLiveCloud.dds_topic.data(), &lingtu_dds_MapCloudLayer_desc);
    voxel_cloud_writer =
        Writer(lingtu::message::kMapsVoxelCloud.dds_topic.data(), &lingtu_dds_MapCloudLayer_desc);
    local_collision_writer = Writer(lingtu::message::kMapsLocalCollision.dds_topic.data(),
                                    &lingtu_dds_MapCollisionLayer_desc);
    accumulated_cloud_writer = Writer(lingtu::message::kMapsAccumulatedCloud.dds_topic.data(),
                                      &lingtu_dds_MapCloudLayer_desc);
    occupancy_writer =
        Writer(lingtu::message::kMapsOccupancy.dds_topic.data(), &lingtu_dds_MapGrid_desc);
    elevation_writer =
        Writer(lingtu::message::kMapsElevation.dds_topic.data(), &lingtu_dds_MapGrid_desc);
    esdf_writer = Writer(lingtu::message::kMapsEsdf.dds_topic.data(), &lingtu_dds_MapGrid_desc);
    scene_writer = Writer(lingtu::message::kMapsScene.dds_topic.data(), &lingtu_dds_MapScene_desc);
    activation_ack_writer = Writer(lingtu::message::kMapsActivationAck.dds_topic.data(),
                                   &lingtu_dds_MapActivationAck_desc);
    snapshot_request_writer = Writer(lingtu::message::kSlamMapSnapshotRequest.dds_topic.data(),
                                     &lingtu_dds_SlamMapSnapshotRequest_desc);
  }

  ~Impl() {
    if (participant > 0) {
      dds_delete(participant);
    }
  }

  dds_entity_t Reader(const char *topic_name, const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic =
        Checked(dds_create_topic(participant, descriptor, topic_name, nullptr, nullptr),
                "dds_create_topic(mapd reader)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return Checked(dds_create_reader(subscriber, topic, qos.get(), nullptr),
                   "dds_create_reader(mapd)");
  }

  dds_entity_t Writer(const char *topic_name, const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic =
        Checked(dds_create_topic(participant, descriptor, topic_name, nullptr, nullptr),
                "dds_create_topic(mapd writer)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return Checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                   "dds_create_writer(mapd)");
  }

  bool Write(dds_entity_t writer, const void *message, const char *channel) {
    ++output_state.write_attempts;
    const dds_return_t result = dds_write(writer, message);
    if (result < 0) {
      ++output_state.write_failures;
      writer_errors[channel] = dds_strretcode(-result);
      RefreshOutputHealth();
      return false;
    }
    writer_errors.erase(channel);
    RefreshOutputHealth();
    return true;
  }

  bool RejectSerialization(const char *channel, const std::string &reason, bool scene_oversize) {
    ++output_state.write_attempts;
    ++output_state.write_failures;
    ++output_state.serialization_rejections;
    if (scene_oversize) {
      ++output_state.scene_oversize_rejections;
    }
    writer_errors[channel] = reason;
    RefreshOutputHealth();
    return false;
  }

  void RefreshOutputHealth() {
    output_state.unhealthy_writers = static_cast<std::uint32_t>(writer_errors.size());
    output_state.last_error.clear();
    for (const auto &[channel, error] : writer_errors) {
      if (!output_state.last_error.empty()) {
        output_state.last_error += ";";
      }
      output_state.last_error += channel + ":" + error;
    }
  }

  DdsLimits limits;
  std::string producer_boot_id;
  DdsInputState input_state;
  DdsOutputState output_state;
  std::map<std::string, std::string> writer_errors;
  dds_entity_t participant{0};
  dds_entity_t subscriber{0};
  dds_entity_t publisher{0};
  dds_entity_t observation_reader{0};
  dds_entity_t activation_request_reader{0};
  dds_entity_t snapshot_ack_reader{0};
  dds_entity_t state_writer{0};
  dds_entity_t live_cloud_writer{0};
  dds_entity_t voxel_cloud_writer{0};
  dds_entity_t local_collision_writer{0};
  dds_entity_t accumulated_cloud_writer{0};
  dds_entity_t occupancy_writer{0};
  dds_entity_t elevation_writer{0};
  dds_entity_t esdf_writer{0};
  dds_entity_t scene_writer{0};
  dds_entity_t activation_ack_writer{0};
  dds_entity_t snapshot_request_writer{0};
};

Dds::Dds(int domain_id, std::size_t max_points_per_observation)
    : Dds(domain_id, DdsLimits{max_points_per_observation, 16U * 1024U * 1024U, 16U, 64U, 4096U,
                               32U * 1024U * 1024U}) {}

Dds::Dds(int domain_id, DdsLimits limits)
    : impl_(std::make_unique<Impl>(domain_id, std::move(limits))) {}

Dds::~Dds() = default;

std::optional<Observation> Dds::TakeLatestObservation() {
  void *samples[kTakeBatch]{};
  dds_sample_info_t infos[kTakeBatch]{};
  for (void *&sample : samples) {
    sample = dds_alloc(sizeof(lingtu_dds_MapObservation));
    std::memset(sample, 0, sizeof(lingtu_dds_MapObservation));
  }
  const dds_return_t count =
      dds_take(impl_->observation_reader, samples, infos, kTakeBatch, kTakeBatch);
  std::optional<Observation> newest;
  if (count < 0) {
    impl_->input_state.last_error = dds_strretcode(-count);
  } else {
    for (dds_return_t index = 0; index < count; ++index) {
      if (!infos[index].valid_data) {
        continue;
      }
      ++impl_->input_state.received_samples;
      std::string error;
      auto decoded = DecodeObservation(*static_cast<lingtu_dds_MapObservation *>(samples[index]),
                                       impl_->limits, &error);
      if (!decoded.has_value()) {
        ++impl_->input_state.rejected_samples;
        impl_->input_state.last_error = std::move(error);
        continue;
      }
      ++impl_->input_state.decoded_samples;
      if (!newest.has_value() || NewerIdentity(*decoded, *newest)) {
        newest = std::move(decoded);
      }
    }
  }
  for (void *sample : samples) {
    dds_sample_free(sample, &lingtu_dds_MapObservation_desc, DDS_FREE_ALL);
  }
  return newest;
}

std::vector<ActivationRequest> Dds::TakeActivationRequests() {
  void *samples[kTakeBatch]{};
  dds_sample_info_t infos[kTakeBatch]{};
  for (void *&sample : samples) {
    sample = dds_alloc(sizeof(lingtu_dds_MapActivationRequest));
    std::memset(sample, 0, sizeof(lingtu_dds_MapActivationRequest));
  }
  const dds_return_t count =
      dds_take(impl_->activation_request_reader, samples, infos, kTakeBatch, kTakeBatch);
  std::vector<ActivationRequest> requests;
  if (count < 0) {
    impl_->input_state.last_error = dds_strretcode(-count);
  } else {
    for (dds_return_t index = 0; index < count; ++index) {
      if (!infos[index].valid_data) {
        continue;
      }
      const auto *sample = static_cast<lingtu_dds_MapActivationRequest *>(samples[index]);
      ++impl_->input_state.received_samples;
      ActivationRequest request;
      std::string error;
      const auto operation = DecodeActivationOperation(sample->operation);
      if (!operation.has_value()) {
        ++impl_->input_state.rejected_samples;
        impl_->input_state.last_error = "map activation operation is invalid";
        continue;
      }
      request.operation = *operation;
      if (!CopyBoundedString(sample->request_id, impl_->limits.max_string_bytes,
                             "map activation request_id", &request.request_id, &error) ||
          !DecodeMapIdentity(sample->target, impl_->limits, "map activation target",
                             &request.target, &error) ||
          !DecodeMapIdentity(sample->previous, impl_->limits, "map activation previous",
                             &request.previous, &error) ||
          !CopyBoundedString(sample->caller, impl_->limits.max_string_bytes,
                             "map activation caller", &request.caller, &error) ||
          !CopyBoundedString(sample->reason, impl_->limits.max_string_bytes,
                             "map activation reason", &request.reason, &error)) {
        ++impl_->input_state.rejected_samples;
        impl_->input_state.last_error = std::move(error);
        continue;
      }
      ++impl_->input_state.decoded_samples;
      requests.push_back(std::move(request));
    }
  }
  for (void *sample : samples) {
    dds_sample_free(sample, &lingtu_dds_MapActivationRequest_desc, DDS_FREE_ALL);
  }
  return requests;
}

bool Dds::PublishState(const State &state, const PublicationProgress &publications,
                       const MapIdentity &active_map) {
  lingtu_dds_MapRuntimeState message{};
  const std::string frame = "map";
  FillHeader(message.header, static_cast<std::int64_t>(NowSeconds() * 1.0e9), frame.c_str());
  message.producer_boot_id = const_cast<char *>(impl_->producer_boot_id.c_str());
  std::vector<lingtu_dds_MapArtifactIdentity> active_artifacts;
  FillMapIdentity(active_map, &message.active_map, &active_artifacts);
  message.running = state.running;
  message.live = state.live;
  message.reset_epoch = state.reset_epoch;
  message.observation_sequence = state.sequence;
  message.generation = state.generation;
  message.accepted_observations = state.accepted_observations;
  message.processed_observations = state.processed_observations;
  message.replaced_observations = state.replaced_observations;
  message.stale_observations = state.stale_observations;
  message.invalid_observations = state.invalid_observations + impl_->input_state.rejected_samples;
  message.epoch_resets = state.epoch_resets;
  message.queue_depth = static_cast<std::uint32_t>(
      std::min<std::size_t>(state.queue_depth, std::numeric_limits<std::uint32_t>::max()));
  message.live_points = state.live_points;
  message.voxel_points = state.voxel_points;
  message.voxel_cells = state.voxel_cells;
  message.voxel_snapshot_omitted_cells = state.voxel_snapshot_omitted_cells;
  message.voxel_capacity_rejections = state.voxel_capacity_rejections;
  message.accumulated_cells = state.accumulated_cells;
  message.accumulated_snapshot_cells = state.accumulated_snapshot_cells;
  message.accumulated_capacity_rejections = state.accumulated_capacity_rejections;
  message.capacity_limited = state.capacity_limited;
  message.pose_quality = state.pose_quality;
  message.pose_state = const_cast<char *>(state.pose_state.c_str());
  message.pose_reason = const_cast<char *>(state.pose_reason.c_str());
  message.dds_received_samples = impl_->input_state.received_samples;
  message.dds_decoded_samples = impl_->input_state.decoded_samples;
  message.dds_rejected_samples = impl_->input_state.rejected_samples;
  message.dds_write_attempts = impl_->output_state.write_attempts;
  message.dds_write_failures = impl_->output_state.write_failures;
  message.dds_serialization_rejections = impl_->output_state.serialization_rejections;
  message.dds_scene_oversize_rejections = impl_->output_state.scene_oversize_rejections;
  message.dds_unhealthy_writers = impl_->output_state.unhealthy_writers;
  // A reader that receives this sample has proof that the state channel
  // itself reached the current generation. The remaining cursors describe
  // the other required mapd publications.
  message.required_publications_ready = publications.realtime_clouds.published_once &&
                                        publications.map_layers.published_once &&
                                        publications.scene.published_once;
  message.current_generation_published =
      publications.CurrentGenerationPublishedWhenStateArrives(state);
  message.state_published_generation = state.generation;
  message.realtime_clouds_published_generation = publications.realtime_clouds.generation;
  message.map_layers_published_generation = publications.map_layers.generation;
  message.scene_published_generation = publications.scene.generation;
  message.engine_error = const_cast<char *>(state.last_error.c_str());
  message.input_error = const_cast<char *>(impl_->input_state.last_error.c_str());
  message.output_error = const_cast<char *>(impl_->output_state.last_error.c_str());
  return impl_->Write(impl_->state_writer, &message, "state");
}

bool Dds::PublishRealtimeClouds(const State &state, const Snapshot &snapshot) {
  OwnedPointCloud empty_cloud;
  empty_cloud.frame_id = snapshot.frame_id;
  empty_cloud.stamp_ns = snapshot.stamp_ns;
  const OwnedPointCloud &live_cloud =
      state.extended_layers_enabled ? snapshot.live_cloud : empty_cloud;
  const OwnedPointCloud &voxel_cloud =
      state.extended_layers_enabled ? snapshot.voxel_cloud : empty_cloud;
  CloudMessage live("live", live_cloud, snapshot.reset_epoch, snapshot.sequence,
                    snapshot.generation, state.live, false);
  CloudMessage voxel("voxel", voxel_cloud, snapshot.reset_epoch, snapshot.sequence,
                     snapshot.generation, state.live, false);
  CollisionMessage collision(snapshot, state.live);
  bool success = true;
  success = impl_->Write(impl_->live_cloud_writer, &live.message, "live_cloud") && success;
  success = impl_->Write(impl_->voxel_cloud_writer, &voxel.message, "voxel_cloud") && success;
  success = impl_->Write(impl_->local_collision_writer, &collision.message, "local_collision") &&
            success;
  return success;
}

std::vector<SlamSnapshotAck> Dds::TakeAcks() {
  void *samples[kTakeBatch]{};
  dds_sample_info_t infos[kTakeBatch]{};
  for (void *&sample : samples) {
    sample = dds_alloc(sizeof(lingtu_dds_SlamMapSnapshotAck));
    std::memset(sample, 0, sizeof(lingtu_dds_SlamMapSnapshotAck));
  }
  const dds_return_t count =
      dds_take(impl_->snapshot_ack_reader, samples, infos, kTakeBatch, kTakeBatch);
  std::vector<SlamSnapshotAck> acks;
  if (count < 0) {
    impl_->input_state.last_error = dds_strretcode(-count);
  } else {
    for (dds_return_t index = 0; index < count; ++index) {
      if (!infos[index].valid_data) {
        continue;
      }
      ++impl_->input_state.received_samples;
      const auto *sample = static_cast<lingtu_dds_SlamMapSnapshotAck *>(samples[index]);
      SlamSnapshotAck ack;
      std::string output_path;
      std::string error;
      if (!CopyBoundedString(sample->request_id, impl_->limits.max_string_bytes,
                             "snapshot ack request_id", &ack.request_id, &error) ||
          !CopyBoundedString(sample->map_id, impl_->limits.max_string_bytes, "snapshot ack map_id",
                             &ack.map_id, &error) ||
          !CopyBoundedString(sample->message, impl_->limits.max_string_bytes,
                             "snapshot ack message", &ack.message, &error) ||
          !CopyBoundedString(sample->output_path, impl_->limits.max_string_bytes,
                             "snapshot ack output_path", &output_path, &error) ||
          !CopyBoundedString(sample->runtime_instance_id, impl_->limits.max_string_bytes,
                             "snapshot ack runtime_instance_id", &ack.runtime_instance_id,
                             &error) ||
          !CopyBoundedString(sample->product_session_id, impl_->limits.max_string_bytes,
                             "snapshot ack product_session_id", &ack.product_session_id, &error) ||
          !CopyBoundedString(sample->frame_id, impl_->limits.max_string_bytes,
                             "snapshot ack frame_id", &ack.frame_id, &error) ||
          !CopyBoundedString(sample->state, impl_->limits.max_string_bytes, "snapshot ack state",
                             &ack.state, &error) ||
          !CopyBoundedString(sample->health_message, impl_->limits.max_string_bytes,
                             "snapshot ack health_message", &ack.health_message, &error)) {
        ++impl_->input_state.rejected_samples;
        impl_->input_state.last_error = std::move(error);
        continue;
      }
      ack.output_path = std::move(output_path);
      ack.success = sample->success;
      ack.reset_epoch = sample->reset_epoch;
      ack.observation_sequence = sample->observation_sequence;
      ack.captured_at_ns = sample->captured_at_ns;
      ack.point_count = sample->point_count;
      ack.healthy = sample->healthy;
      ++impl_->input_state.decoded_samples;
      acks.push_back(std::move(ack));
    }
  }
  for (void *sample : samples) {
    dds_sample_free(sample, &lingtu_dds_SlamMapSnapshotAck_desc, DDS_FREE_ALL);
  }
  return acks;
}

bool Dds::PublishMapLayers(const State &state, const Snapshot &snapshot) {
  const OwnedPointCloud accumulated = AccumulatedAsCloud(snapshot.accumulated_cloud);
  CloudMessage accumulated_message("accumulated", accumulated, snapshot.reset_epoch,
                                   snapshot.sequence, snapshot.generation, state.live, true);
  GridMessage occupancy("occupancy", snapshot.occupancy, snapshot, state.live);
  GridMessage elevation("elevation", snapshot.elevation.minZ, snapshot, state.live);
  GridMessage esdf("esdf", snapshot.esdf.distance, snapshot, state.live);
  bool success = true;
  success = impl_->Write(impl_->accumulated_cloud_writer, &accumulated_message.message,
                         "accumulated_cloud") &&
            success;
  success = impl_->Write(impl_->occupancy_writer, &occupancy.message, "occupancy") && success;
  success = impl_->Write(impl_->elevation_writer, &elevation.message, "elevation") && success;
  success = impl_->Write(impl_->esdf_writer, &esdf.message, "esdf") && success;
  return success;
}

bool Dds::PublishScene(const State &state, const Snapshot &snapshot) {
  const auto estimated_bytes = EstimateSceneBytes(state, snapshot);
  if (!estimated_bytes.has_value()) {
    return impl_->RejectSerialization("scene", "scene serialized size overflows size_t", true);
  }
  if (*estimated_bytes > impl_->limits.max_scene_bytes) {
    return impl_->RejectSerialization("scene", "scene serialized payload exceeds mapd limit", true);
  }
  const OwnedPointCloud accumulated = AccumulatedAsCloud(snapshot.accumulated_cloud);
  OwnedPointCloud empty_cloud;
  empty_cloud.frame_id = snapshot.frame_id;
  empty_cloud.stamp_ns = snapshot.stamp_ns;
  const OwnedPointCloud &live_cloud =
      state.extended_layers_enabled ? snapshot.live_cloud : empty_cloud;
  const OwnedPointCloud &voxel_cloud =
      state.extended_layers_enabled ? snapshot.voxel_cloud : empty_cloud;
  CloudMessage live("live", live_cloud, snapshot.reset_epoch, snapshot.sequence,
                    snapshot.generation, state.live, false);
  CloudMessage voxel("voxel", voxel_cloud, snapshot.reset_epoch, snapshot.sequence,
                     snapshot.generation, state.live, false);
  CloudMessage accumulated_message("accumulated", accumulated, snapshot.reset_epoch,
                                   snapshot.sequence, snapshot.generation, state.live, true);
  GridMessage occupancy("occupancy", snapshot.occupancy, snapshot, state.live);
  GridMessage elevation("elevation", snapshot.elevation.minZ, snapshot, state.live);
  GridMessage esdf("esdf", snapshot.esdf.distance, snapshot, state.live);

  lingtu_dds_MapScene message{};
  const std::string frame = snapshot.frame_id.empty() ? "map" : snapshot.frame_id;
  FillHeader(message.header, snapshot.stamp_ns, frame.c_str());
  message.producer_boot_id = const_cast<char *>(impl_->producer_boot_id.c_str());
  message.reset_epoch = snapshot.reset_epoch;
  message.observation_sequence = snapshot.sequence;
  message.generation = snapshot.generation;
  message.live = state.live;
  message.map_sensor.position.x = snapshot.map_sensor.x;
  message.map_sensor.position.y = snapshot.map_sensor.y;
  message.map_sensor.position.z = snapshot.map_sensor.z;
  message.map_sensor.orientation.x = snapshot.map_sensor.qx;
  message.map_sensor.orientation.y = snapshot.map_sensor.qy;
  message.map_sensor.orientation.z = snapshot.map_sensor.qz;
  message.map_sensor.orientation.w = snapshot.map_sensor.qw;
  message.live_cloud = live.message;
  message.voxel_cloud = voxel.message;
  message.accumulated_cloud = accumulated_message.message;
  message.occupancy = occupancy.message;
  message.elevation = elevation.message;
  message.esdf = esdf.message;
  return impl_->Write(impl_->scene_writer, &message, "scene");
}

bool Dds::PublishActivationAck(const ActivationResult &ack) {
  lingtu_dds_MapActivationAck message{};
  message.request_id = const_cast<char *>(ack.request_id.c_str());
  message.operation = EncodeActivationOperation(ack.operation);
  message.accepted = ack.accepted ? 1U : 0U;
  message.message = const_cast<char *>(ack.message.c_str());
  message.changed = ack.changed ? 1U : 0U;
  std::vector<lingtu_dds_MapArtifactIdentity> target_artifacts;
  std::vector<lingtu_dds_MapArtifactIdentity> previous_artifacts;
  std::vector<lingtu_dds_MapArtifactIdentity> active_artifacts;
  FillMapIdentity(ack.target, &message.target, &target_artifacts);
  FillMapIdentity(ack.previous, &message.previous, &previous_artifacts);
  FillMapIdentity(ack.active, &message.active, &active_artifacts);
  message.producer_boot_id = const_cast<char *>(ack.producer_boot_id.c_str());
  return impl_->Write(impl_->activation_ack_writer, &message, "activation_ack");
}

bool Dds::Publish(const SlamSnapshotRequest &request) {
  lingtu_dds_SlamMapSnapshotRequest message{};
  const std::string output_path = request.output_path.string();
  message.request_id = const_cast<char *>(request.request_id.c_str());
  message.map_id = const_cast<char *>(request.map_id.c_str());
  message.product_session_id = const_cast<char *>(request.product_session_id.c_str());
  message.output_path = const_cast<char *>(output_path.c_str());
  message.save_patches = request.save_patches;
  return impl_->Write(impl_->snapshot_request_writer, &message, "slam_snapshot_request");
}

const std::string &Dds::ProducerBootId() const {
  return impl_->producer_boot_id;
}

DdsInputState Dds::GetInputState() const {
  return impl_->input_state;
}

DdsOutputState Dds::GetOutputState() const {
  return impl_->output_state;
}

}  // namespace lingtu::maps::mapd
