#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/topics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using WallClock = std::chrono::system_clock;

enum class Kind {
  LivoxFrame,
  Imu,
  Odometry,
  PointCloud2,
  Image,
  CameraInfo,
  GnssFix,
  GnssStatus,
  DriverControlState,
  Float32,
  Bool,
  Text,
  Path,
  PoseStamped,
  OccupancyGrid,
  TwistStamped,
  FinalVelocityCommand,
  TFMessage,
  MapObservation,
  MapRuntimeState,
  MapCloudLayer,
  MapCollisionLayer,
  MapGrid,
  MapScene,
  ExplorationExecutionGrid,
  SlamMapSnapshotRequest,
  SlamMapSnapshotAck,
  RelocalizationRequest,
  RelocalizationResponse,
};

struct TopicSpec {
  std::string_view topic;
  std::string_view wire_topic;
  const dds_topic_descriptor_t* desc;
  Kind kind;
  void* (*alloc)();
  void (*free_sample)(void*);
};

struct TopicStats {
  std::string label;
  std::string topic;
  std::string wire_topic;
  std::uint64_t samples{0};
  double first_ts{0.0};
  double last_ts{0.0};
  double first_steady_s{0.0};
  double last_steady_s{0.0};
  double max_gap_s{0.0};
  std::string frame_id;
  long long points{-1};
  bool has_map_identity{false};
  std::uint64_t reset_epoch{0};
  std::uint64_t observation_sequence{0};
  std::uint64_t generation{0};
  bool has_live{false};
  bool live{false};

  double hz() const {
    if (samples < 2 || last_steady_s <= first_steady_s) {
      return 0.0;
    }
    return static_cast<double>(samples - 1) / (last_steady_s - first_steady_s);
  }
};

constexpr dds_free_op_t kFreeAll =
    static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT);

void* alloc_livox_frame() { return lingtu_dds_LivoxFrame__alloc(); }
void free_livox_frame(void* p) { lingtu_dds_LivoxFrame_free(p, kFreeAll); }
void* alloc_imu() { return lingtu_dds_Imu__alloc(); }
void free_imu(void* p) { lingtu_dds_Imu_free(p, kFreeAll); }
void* alloc_odometry() { return lingtu_dds_Odometry__alloc(); }
void free_odometry(void* p) { lingtu_dds_Odometry_free(p, kFreeAll); }
void* alloc_point_cloud() { return lingtu_dds_PointCloud2__alloc(); }
void free_point_cloud(void* p) { lingtu_dds_PointCloud2_free(p, kFreeAll); }
void* alloc_image() { return lingtu_dds_Image__alloc(); }
void free_image(void* p) { lingtu_dds_Image_free(p, kFreeAll); }
void* alloc_camera_info() { return lingtu_dds_CameraInfo__alloc(); }
void free_camera_info(void* p) { lingtu_dds_CameraInfo_free(p, kFreeAll); }
void* alloc_gnss_fix() { return lingtu_dds_GnssFix__alloc(); }
void free_gnss_fix(void* p) { lingtu_dds_GnssFix_free(p, kFreeAll); }
void* alloc_gnss_status() { return lingtu_dds_GnssStatus__alloc(); }
void free_gnss_status(void* p) { lingtu_dds_GnssStatus_free(p, kFreeAll); }
void* alloc_driver_control_state() {
  return lingtu_dds_DriverControlState__alloc();
}
void free_driver_control_state(void* p) {
  lingtu_dds_DriverControlState_free(p, kFreeAll);
}
void* alloc_float32() { return lingtu_dds_Float32__alloc(); }
void free_float32(void* p) { lingtu_dds_Float32_free(p, kFreeAll); }
void* alloc_bool() { return lingtu_dds_Bool__alloc(); }
void free_bool(void* p) { lingtu_dds_Bool_free(p, kFreeAll); }
void* alloc_text() { return lingtu_dds_Text__alloc(); }
void free_text(void* p) { lingtu_dds_Text_free(p, kFreeAll); }
void* alloc_path() { return lingtu_dds_Path__alloc(); }
void free_path(void* p) { lingtu_dds_Path_free(p, kFreeAll); }
void* alloc_pose_stamped() { return lingtu_dds_PoseStamped__alloc(); }
void free_pose_stamped(void* p) { lingtu_dds_PoseStamped_free(p, kFreeAll); }
void* alloc_occupancy_grid() { return lingtu_dds_OccupancyGrid__alloc(); }
void free_occupancy_grid(void* p) { lingtu_dds_OccupancyGrid_free(p, kFreeAll); }
void* alloc_twist_stamped() { return lingtu_dds_TwistStamped__alloc(); }
void free_twist_stamped(void* p) { lingtu_dds_TwistStamped_free(p, kFreeAll); }
void* alloc_final_velocity_command() {
  return lingtu_dds_FinalVelocityCommand__alloc();
}
void free_final_velocity_command(void* p) {
  lingtu_dds_FinalVelocityCommand_free(p, kFreeAll);
}
void* alloc_tf_message() { return lingtu_dds_TFMessage__alloc(); }
void free_tf_message(void* p) { lingtu_dds_TFMessage_free(p, kFreeAll); }
void* alloc_map_observation() { return lingtu_dds_MapObservation__alloc(); }
void free_map_observation(void* p) { lingtu_dds_MapObservation_free(p, kFreeAll); }
void* alloc_map_runtime_state() { return lingtu_dds_MapRuntimeState__alloc(); }
void free_map_runtime_state(void* p) { lingtu_dds_MapRuntimeState_free(p, kFreeAll); }
void* alloc_map_cloud_layer() { return lingtu_dds_MapCloudLayer__alloc(); }
void free_map_cloud_layer(void* p) { lingtu_dds_MapCloudLayer_free(p, kFreeAll); }
void* alloc_map_collision_layer() { return lingtu_dds_MapCollisionLayer__alloc(); }
void free_map_collision_layer(void* p) { lingtu_dds_MapCollisionLayer_free(p, kFreeAll); }
void* alloc_map_grid() { return lingtu_dds_MapGrid__alloc(); }
void free_map_grid(void* p) { lingtu_dds_MapGrid_free(p, kFreeAll); }
void* alloc_map_scene() { return lingtu_dds_MapScene__alloc(); }
void free_map_scene(void* p) { lingtu_dds_MapScene_free(p, kFreeAll); }
void* alloc_exploration_execution_grid() {
  return lingtu_dds_ExplorationExecutionGrid__alloc();
}
void free_exploration_execution_grid(void* p) {
  lingtu_dds_ExplorationExecutionGrid_free(p, kFreeAll);
}
void* alloc_slam_map_snapshot_request() {
  return lingtu_dds_SlamMapSnapshotRequest__alloc();
}
void free_slam_map_snapshot_request(void* p) {
  lingtu_dds_SlamMapSnapshotRequest_free(p, kFreeAll);
}
void* alloc_slam_map_snapshot_ack() {
  return lingtu_dds_SlamMapSnapshotAck__alloc();
}
void free_slam_map_snapshot_ack(void* p) {
  lingtu_dds_SlamMapSnapshotAck_free(p, kFreeAll);
}
void* alloc_reloc_request() { return lingtu_dds_RelocalizationRequest__alloc(); }
void free_reloc_request(void* p) {
  lingtu_dds_RelocalizationRequest_free(p, kFreeAll);
}
void* alloc_reloc_response() { return lingtu_dds_RelocalizationResponse__alloc(); }
void free_reloc_response(void* p) {
  lingtu_dds_RelocalizationResponse_free(p, kFreeAll);
}

std::string as_string(std::string_view value) {
  return std::string(value.data(), value.size());
}

const TopicSpec kSpecs[] = {
    {lingtu::message::kLidarRawFrame.topic, lingtu::message::kLidarRawFrame.dds_topic,
     &lingtu_dds_LivoxFrame_desc, Kind::LivoxFrame, alloc_livox_frame, free_livox_frame},
    {lingtu::message::kLidarRawPacket.topic, lingtu::message::kLidarRawPacket.dds_topic,
     &lingtu_dds_LivoxFrame_desc, Kind::LivoxFrame, alloc_livox_frame, free_livox_frame},
    {lingtu::message::kImuRaw.topic, lingtu::message::kImuRaw.dds_topic,
     &lingtu_dds_Imu_desc, Kind::Imu, alloc_imu, free_imu},
    {lingtu::message::kSlamOdomPrior.topic, lingtu::message::kSlamOdomPrior.dds_topic,
     &lingtu_dds_Odometry_desc, Kind::Odometry, alloc_odometry, free_odometry},
    {lingtu::message::kDriverOdometry.topic, lingtu::message::kDriverOdometry.dds_topic,
     &lingtu_dds_Odometry_desc, Kind::Odometry, alloc_odometry, free_odometry},
    {lingtu::message::kDriverControlState.topic, lingtu::message::kDriverControlState.dds_topic,
     &lingtu_dds_DriverControlState_desc, Kind::DriverControlState, alloc_driver_control_state,
     free_driver_control_state},
    {lingtu::message::kSlamOdometry.topic, lingtu::message::kSlamOdometry.dds_topic,
     &lingtu_dds_Odometry_desc, Kind::Odometry, alloc_odometry, free_odometry},
    {lingtu::message::kSlamStateAtScan.topic, lingtu::message::kSlamStateAtScan.dds_topic,
     &lingtu_dds_Odometry_desc, Kind::Odometry, alloc_odometry, free_odometry},
    {lingtu::message::kSlamRegisteredCloud.topic, lingtu::message::kSlamRegisteredCloud.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kSlamMapObservation.topic, lingtu::message::kSlamMapObservation.dds_topic,
     &lingtu_dds_MapObservation_desc, Kind::MapObservation, alloc_map_observation,
     free_map_observation},
    {lingtu::message::kSlamMapCloud.topic, lingtu::message::kSlamMapCloud.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kSlamCumulativeMapCloud.topic, lingtu::message::kSlamCumulativeMapCloud.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kSlamSavedMapCloud.topic, lingtu::message::kSlamSavedMapCloud.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kMapsState.topic, lingtu::message::kMapsState.dds_topic,
     &lingtu_dds_MapRuntimeState_desc, Kind::MapRuntimeState, alloc_map_runtime_state,
     free_map_runtime_state},
    {lingtu::message::kMapsLiveCloud.topic, lingtu::message::kMapsLiveCloud.dds_topic,
     &lingtu_dds_MapCloudLayer_desc, Kind::MapCloudLayer, alloc_map_cloud_layer,
     free_map_cloud_layer},
    {lingtu::message::kMapsVoxelCloud.topic, lingtu::message::kMapsVoxelCloud.dds_topic,
     &lingtu_dds_MapCloudLayer_desc, Kind::MapCloudLayer, alloc_map_cloud_layer,
     free_map_cloud_layer},
    {lingtu::message::kMapsLocalCollision.topic, lingtu::message::kMapsLocalCollision.dds_topic,
     &lingtu_dds_MapCollisionLayer_desc, Kind::MapCollisionLayer, alloc_map_collision_layer,
     free_map_collision_layer},
    {lingtu::message::kMapsAccumulatedCloud.topic,
     lingtu::message::kMapsAccumulatedCloud.dds_topic, &lingtu_dds_MapCloudLayer_desc,
     Kind::MapCloudLayer, alloc_map_cloud_layer, free_map_cloud_layer},
    {lingtu::message::kMapsOccupancy.topic, lingtu::message::kMapsOccupancy.dds_topic,
     &lingtu_dds_MapGrid_desc, Kind::MapGrid, alloc_map_grid, free_map_grid},
    {lingtu::message::kMapsElevation.topic, lingtu::message::kMapsElevation.dds_topic,
     &lingtu_dds_MapGrid_desc, Kind::MapGrid, alloc_map_grid, free_map_grid},
    {lingtu::message::kMapsEsdf.topic, lingtu::message::kMapsEsdf.dds_topic,
     &lingtu_dds_MapGrid_desc, Kind::MapGrid, alloc_map_grid, free_map_grid},
    {lingtu::message::kMapsScene.topic, lingtu::message::kMapsScene.dds_topic,
     &lingtu_dds_MapScene_desc, Kind::MapScene, alloc_map_scene, free_map_scene},
    {lingtu::message::kCameraColor.topic, lingtu::message::kCameraColor.dds_topic,
     &lingtu_dds_Image_desc, Kind::Image, alloc_image, free_image},
    {lingtu::message::kCameraDepth.topic, lingtu::message::kCameraDepth.dds_topic,
     &lingtu_dds_Image_desc, Kind::Image, alloc_image, free_image},
    {lingtu::message::kCameraInfo.topic, lingtu::message::kCameraInfo.dds_topic,
     &lingtu_dds_CameraInfo_desc, Kind::CameraInfo, alloc_camera_info, free_camera_info},
    {lingtu::message::kGnssFix.topic, lingtu::message::kGnssFix.dds_topic,
     &lingtu_dds_GnssFix_desc, Kind::GnssFix, alloc_gnss_fix, free_gnss_fix},
    {lingtu::message::kGnssStatus.topic, lingtu::message::kGnssStatus.dds_topic,
     &lingtu_dds_GnssStatus_desc, Kind::GnssStatus, alloc_gnss_status, free_gnss_status},
    {lingtu::message::kGnssOdom.topic, lingtu::message::kGnssOdom.dds_topic,
     &lingtu_dds_Odometry_desc, Kind::Odometry, alloc_odometry, free_odometry},
    {lingtu::message::kSlamMapSnapshotRequest.topic,
     lingtu::message::kSlamMapSnapshotRequest.dds_topic,
     &lingtu_dds_SlamMapSnapshotRequest_desc, Kind::SlamMapSnapshotRequest,
     alloc_slam_map_snapshot_request, free_slam_map_snapshot_request},
    {lingtu::message::kSlamMapSnapshotAck.topic,
     lingtu::message::kSlamMapSnapshotAck.dds_topic,
     &lingtu_dds_SlamMapSnapshotAck_desc, Kind::SlamMapSnapshotAck,
     alloc_slam_map_snapshot_ack, free_slam_map_snapshot_ack},
    {lingtu::message::kSlamRelocalizationRequest.topic, lingtu::message::kSlamRelocalizationRequest.dds_topic,
     &lingtu_dds_RelocalizationRequest_desc, Kind::RelocalizationRequest, alloc_reloc_request, free_reloc_request},
    {lingtu::message::kSlamRelocalizationResponse.topic, lingtu::message::kSlamRelocalizationResponse.dds_topic,
     &lingtu_dds_RelocalizationResponse_desc, Kind::RelocalizationResponse, alloc_reloc_response, free_reloc_response},
    {lingtu::message::kSlamLocalizationQuality.topic, lingtu::message::kSlamLocalizationQuality.dds_topic,
     &lingtu_dds_Float32_desc, Kind::Float32, alloc_float32, free_float32},
    {lingtu::message::kSlamLocalizationHealth.topic, lingtu::message::kSlamLocalizationHealth.dds_topic,
     &lingtu_dds_Text_desc, Kind::Text, alloc_text, free_text},
    {lingtu::message::kNavGlobalPath.topic, lingtu::message::kNavGlobalPath.dds_topic,
     &lingtu_dds_Path_desc, Kind::Path, alloc_path, free_path},
    {lingtu::message::kNavLocalPath.topic, lingtu::message::kNavLocalPath.dds_topic,
     &lingtu_dds_Path_desc, Kind::Path, alloc_path, free_path},
    {lingtu::message::kNavWayPoint.topic, lingtu::message::kNavWayPoint.dds_topic,
     &lingtu_dds_PoseStamped_desc, Kind::PoseStamped, alloc_pose_stamped, free_pose_stamped},
    {lingtu::message::kNavSemanticInstruction.topic, lingtu::message::kNavSemanticInstruction.dds_topic,
     &lingtu_dds_Text_desc, Kind::Text, alloc_text, free_text},
    {lingtu::message::kNavTraversability.topic, lingtu::message::kNavTraversability.dds_topic,
     &lingtu_dds_OccupancyGrid_desc, Kind::OccupancyGrid, alloc_occupancy_grid, free_occupancy_grid},
    {lingtu::message::kNavTerrainMap.topic, lingtu::message::kNavTerrainMap.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kNavTerrainMapExt.topic, lingtu::message::kNavTerrainMapExt.dds_topic,
     &lingtu_dds_PointCloud2_desc, Kind::PointCloud2, alloc_point_cloud, free_point_cloud},
    {lingtu::message::kNavMapClearing.topic, lingtu::message::kNavMapClearing.dds_topic,
     &lingtu_dds_Bool_desc, Kind::Bool, alloc_bool, free_bool},
    {lingtu::message::kNavCloudClearing.topic, lingtu::message::kNavCloudClearing.dds_topic,
     &lingtu_dds_Bool_desc, Kind::Bool, alloc_bool, free_bool},
    {lingtu::message::kNavCmdVel.topic, lingtu::message::kNavCmdVel.dds_topic,
     &lingtu_dds_FinalVelocityCommand_desc, Kind::FinalVelocityCommand,
     alloc_final_velocity_command, free_final_velocity_command},
    {lingtu::message::kNavExplorationGrid.topic, lingtu::message::kNavExplorationGrid.dds_topic,
     &lingtu_dds_OccupancyGrid_desc, Kind::OccupancyGrid, alloc_occupancy_grid, free_occupancy_grid},
    {lingtu::message::kNavExplorationExecutionSnapshot.topic,
     lingtu::message::kNavExplorationExecutionSnapshot.dds_topic,
     &lingtu_dds_ExplorationExecutionGrid_desc, Kind::ExplorationExecutionGrid,
     alloc_exploration_execution_grid, free_exploration_execution_grid},
    {lingtu::message::kTf.topic, lingtu::message::kTf.dds_topic,
     &lingtu_dds_TFMessage_desc, Kind::TFMessage, alloc_tf_message, free_tf_message},
    {lingtu::message::kTfStatic.topic, lingtu::message::kTfStatic.dds_topic,
     &lingtu_dds_TFMessage_desc, Kind::TFMessage, alloc_tf_message, free_tf_message},
};

std::string normalize(std::string_view input) {
  std::string value = as_string(input);
  if (value.rfind("rt/", 0) == 0 || value.rfind("/", 0) == 0) {
    return value;
  }
  return "/" + value;
}

const TopicSpec* find_spec(std::string_view input) {
  const std::string value = normalize(input);
  for (const auto& spec : kSpecs) {
    if (value == spec.topic || value == spec.wire_topic) {
      return &spec;
    }
  }
  const std::string wire = value.rfind("/", 0) == 0 ? "rt" + value : "rt/" + value;
  for (const auto& spec : kSpecs) {
    if (wire == spec.wire_topic) {
      return &spec;
    }
  }
  return nullptr;
}

double wall_seconds() {
  const auto now = WallClock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

std::string frame_from_header(const lingtu_dds_Header& header) {
  return header.frame_id ? std::string(header.frame_id) : std::string();
}

double steady_seconds() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

void observe_map_identity(
    TopicStats& stats,
    std::uint64_t reset_epoch,
    std::uint64_t observation_sequence,
    std::uint64_t generation,
    bool has_live,
    bool live) {
  stats.has_map_identity = true;
  stats.reset_epoch = reset_epoch;
  stats.observation_sequence = observation_sequence;
  stats.generation = generation;
  stats.has_live = has_live;
  stats.live = live;
}

void observe(TopicStats& stats, const TopicSpec& spec, const void* sample) {
  const double wall_now = wall_seconds();
  const double steady_now = steady_seconds();
  if (stats.samples == 0) {
    stats.first_ts = wall_now;
    stats.first_steady_s = steady_now;
  } else {
    stats.max_gap_s = std::max(stats.max_gap_s, steady_now - stats.last_steady_s);
  }
  stats.samples += 1;
  stats.last_ts = wall_now;
  stats.last_steady_s = steady_now;

  switch (spec.kind) {
    case Kind::LivoxFrame: {
      const auto* msg = static_cast<const lingtu_dds_LivoxFrame*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->point_num);
      break;
    }
    case Kind::Imu: {
      const auto* msg = static_cast<const lingtu_dds_Imu*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::Odometry: {
      const auto* msg = static_cast<const lingtu_dds_Odometry*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::PointCloud2: {
      const auto* msg = static_cast<const lingtu_dds_PointCloud2*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->width) * static_cast<long long>(msg->height);
      break;
    }
    case Kind::Image: {
      const auto* msg = static_cast<const lingtu_dds_Image*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->width) * static_cast<long long>(msg->height);
      break;
    }
    case Kind::CameraInfo: {
      const auto* msg = static_cast<const lingtu_dds_CameraInfo*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->width) * static_cast<long long>(msg->height);
      break;
    }
    case Kind::GnssFix: {
      const auto* msg = static_cast<const lingtu_dds_GnssFix*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::GnssStatus: {
      const auto* msg = static_cast<const lingtu_dds_GnssStatus*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::DriverControlState: {
      const auto* msg =
          static_cast<const lingtu_dds_DriverControlState*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::Path: {
      const auto* msg = static_cast<const lingtu_dds_Path*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->poses._length);
      break;
    }
    case Kind::PoseStamped: {
      const auto* msg = static_cast<const lingtu_dds_PoseStamped*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::OccupancyGrid: {
      const auto* msg = static_cast<const lingtu_dds_OccupancyGrid*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->info.width) * static_cast<long long>(msg->info.height);
      break;
    }
    case Kind::TwistStamped: {
      const auto* msg = static_cast<const lingtu_dds_TwistStamped*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      break;
    }
    case Kind::FinalVelocityCommand: {
      stats.frame_id = "body";
      break;
    }
    case Kind::TFMessage: {
      const auto* msg = static_cast<const lingtu_dds_TFMessage*>(sample);
      stats.points = static_cast<long long>(msg->transforms._length);
      break;
    }
    case Kind::MapObservation: {
      const auto* msg = static_cast<const lingtu_dds_MapObservation*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->scan.width) *
          static_cast<long long>(msg->scan.height);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, 0U, false, false);
      break;
    }
    case Kind::MapRuntimeState: {
      const auto* msg = static_cast<const lingtu_dds_MapRuntimeState*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->live_points);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, msg->generation, true, msg->live);
      break;
    }
    case Kind::MapCloudLayer: {
      const auto* msg = static_cast<const lingtu_dds_MapCloudLayer*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->cloud.width) *
          static_cast<long long>(msg->cloud.height);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, msg->generation, true, msg->live);
      break;
    }
    case Kind::MapCollisionLayer: {
      const auto* msg = static_cast<const lingtu_dds_MapCollisionLayer*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->occupied.width) *
          static_cast<long long>(msg->occupied.height);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, msg->generation, true, msg->live);
      break;
    }
    case Kind::MapGrid: {
      const auto* msg = static_cast<const lingtu_dds_MapGrid*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->data._length);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, msg->generation, true, msg->live);
      break;
    }
    case Kind::MapScene: {
      const auto* msg = static_cast<const lingtu_dds_MapScene*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->voxel_cloud.cloud.width) *
          static_cast<long long>(msg->voxel_cloud.cloud.height);
      observe_map_identity(
          stats, msg->reset_epoch, msg->observation_sequence, msg->generation, true, msg->live);
      break;
    }
    case Kind::ExplorationExecutionGrid: {
      const auto* msg =
          static_cast<const lingtu_dds_ExplorationExecutionGrid*>(sample);
      stats.frame_id = frame_from_header(msg->header);
      stats.points = static_cast<long long>(msg->info.width) *
          static_cast<long long>(msg->info.height);
      observe_map_identity(
          stats, msg->reset_epoch, 0U, msg->generation, true, msg->live);
      break;
    }
    default:
      break;
  }
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    std::cerr << what << ": " << dds_strretcode(-value) << "\n";
    std::exit(2);
  }
  return static_cast<dds_entity_t>(value);
}

std::string json_escape(std::string_view input) {
  std::ostringstream out;
  for (char c : input) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << c; break;
    }
  }
  return out.str();
}

struct Reader {
  const TopicSpec* spec{nullptr};
  TopicStats stats;
  dds_entity_t topic{DDS_RETCODE_ERROR};
  dds_entity_t reader{DDS_RETCODE_ERROR};
};

void print_table(const std::vector<Reader>& readers) {
  std::cout << std::left << std::setw(36) << "topic"
            << std::right << std::setw(8) << "samples"
            << std::setw(8) << "hz"
            << " " << std::left << std::setw(16) << "frame"
            << std::right << std::setw(9) << "points"
            << std::setw(8) << "age_s" << "\n";
  const double now = wall_seconds();
  for (const auto& reader : readers) {
    const auto& item = reader.stats;
    const double age = item.last_ts > 0.0 ? now - item.last_ts : 0.0;
    const std::string points = item.points >= 0 ? std::to_string(item.points) : "";
    std::cout << std::left << std::setw(36) << item.label
              << std::right << std::setw(8) << item.samples
              << std::setw(8) << std::fixed << std::setprecision(2) << item.hz()
              << " " << std::left << std::setw(16) << item.frame_id.substr(0, 16)
              << std::right << std::setw(9) << points
              << std::setw(8) << std::fixed << std::setprecision(2) << age << "\n";
  }
}

void print_json(const std::vector<Reader>& readers) {
  std::cout << "[";
  for (std::size_t i = 0; i < readers.size(); ++i) {
    const auto& item = readers[i].stats;
    if (i > 0) {
      std::cout << ",";
    }
    std::cout << "{\"topic\":\"" << json_escape(item.label)
              << "\",\"canonical_topic\":\"" << json_escape(item.topic)
              << "\",\"wire_topic\":\"" << json_escape(item.wire_topic)
              << "\",\"samples\":" << item.samples
              << ",\"hz\":" << std::fixed << std::setprecision(6) << item.hz()
              << ",\"first_ts\":" << std::fixed << std::setprecision(6) << item.first_ts
              << ",\"last_ts\":" << std::fixed << std::setprecision(6) << item.last_ts
              << ",\"max_gap_s\":" << std::fixed << std::setprecision(6) << item.max_gap_s
              << ",\"frame_id\":\"" << json_escape(item.frame_id) << "\"";
    if (item.points >= 0) {
      std::cout << ",\"points\":" << item.points;
    }
    if (item.has_map_identity) {
      std::cout << ",\"reset_epoch\":" << item.reset_epoch
                << ",\"observation_sequence\":" << item.observation_sequence
                << ",\"generation\":" << item.generation;
    }
    if (item.has_live) {
      std::cout << ",\"live\":" << (item.live ? "true" : "false");
    }
    std::cout << "}";
  }
  std::cout << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
  double seconds = 10.0;
  int domain_id = 0;
  bool json = false;
  std::vector<std::string> requested;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << arg << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--seconds") {
      seconds = std::stod(next());
    } else if (arg == "--domain") {
      domain_id = std::stoi(next());
    } else if (arg == "--json") {
      json = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: lingtu_dds_probe [--seconds N] [--domain N] [--json] [topics...]\n";
      return 0;
    } else {
      requested.push_back(arg);
    }
  }

  if (requested.empty()) {
    requested = {
        as_string(lingtu::message::kLidarRawFrame.topic),
        as_string(lingtu::message::kImuRaw.topic),
        as_string(lingtu::message::kSlamOdometry.topic),
        as_string(lingtu::message::kSlamRegisteredCloud.topic),
        as_string(lingtu::message::kSlamMapCloud.topic),
        as_string(lingtu::message::kSlamLocalizationHealth.topic),
    };
  }

  dds_entity_t participant = checked(
      dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant");
  std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)> qos(dds_create_qos(), dds_delete_qos);
  dds_qset_reliability(qos.get(), DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
  dds_qset_history(qos.get(), DDS_HISTORY_KEEP_LAST, 128);

  std::vector<Reader> readers;
  readers.reserve(requested.size());
  for (const auto& name : requested) {
    const TopicSpec* spec = find_spec(name);
    if (spec == nullptr) {
      std::cerr << "no typed DDS contract for " << name << "\n";
      dds_delete(participant);
      return 2;
    }
    Reader reader;
    reader.spec = spec;
    reader.stats.label = name;
    reader.stats.topic = as_string(spec->topic);
    reader.stats.wire_topic = as_string(spec->wire_topic);
    const std::string wire_topic = reader.stats.wire_topic;
    reader.topic = checked(
        dds_create_topic(participant, spec->desc, wire_topic.c_str(), nullptr, nullptr),
        "dds_create_topic");
    reader.reader = checked(
        dds_create_reader(participant, reader.topic, qos.get(), nullptr),
        "dds_create_reader");
    readers.push_back(std::move(reader));
  }

  const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
  constexpr std::size_t kMaxTake = 64U;
  while (Clock::now() < deadline) {
    for (auto& reader : readers) {
      void* samples[kMaxTake];
      dds_sample_info_t infos[kMaxTake];
      for (std::size_t i = 0; i < kMaxTake; ++i) {
        samples[i] = reader.spec->alloc();
      }
      const int ret = dds_take(reader.reader, samples, infos, kMaxTake, kMaxTake);
      if (ret < 0) {
        std::cerr << "dds_take(" << reader.stats.label << "): "
                  << dds_strretcode(-ret) << "\n";
        dds_delete(participant);
        return 3;
      }
      for (int i = 0; i < ret; ++i) {
        if (infos[i].valid_data) {
          observe(reader.stats, *reader.spec, samples[i]);
        }
      }
      for (std::size_t i = 0; i < kMaxTake; ++i) {
        reader.spec->free_sample(samples[i]);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (json) {
    print_json(readers);
  } else {
    print_table(readers);
  }

  bool ok = true;
  for (const auto& reader : readers) {
    ok = ok && reader.stats.samples > 0;
  }
  dds_delete(participant);
  return ok ? 0 : 1;
}
