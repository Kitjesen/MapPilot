#pragma once
// DDS QoS profiles used by LingTu native services.

#include "dds/dds.h"
#include "message/cpp/topics.hpp"
#include <memory>
#include <string_view>

namespace lingtu::dds {

enum class QosProfile {
  Default,           // CycloneDDS defaults (no explicit QoS)
  SensorStream,     // lidar/imu: BEST_EFFORT, KEEP_LAST=256
  RawLidarStream,   // raw lidar: BEST_EFFORT, KEEP_LAST=2, lifespan=350ms
  HighFreqState,    // odometry/state: BEST_EFFORT, KEEP_LAST=5, deadline=20ms
  LocalizationHealth,  // localization health: RELIABLE, VOLATILE, KEEP_LAST=10
  ControlCommand,   // stop/control: RELIABLE, KEEP_LAST=1, deadline=50ms
  FinalVelocityCommand,  // final cmd_vel plus a 200ms DDS lifespan
  CommandRequest,   // typed requests: RELIABLE, VOLATILE, KEEP_LAST=32
  CommandAck,       // typed replies: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=64
  OperatorMotionControl,  // authority claim/release/hold: RELIABLE, KEEP_LAST=32
  OperatorMotionSample,   // latest-only velocity intent: BEST_EFFORT, KEEP_LAST=1
  OperatorMotionAck,      // admission replies: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=64
  OperatorMotionStatus,   // authority/output state: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  InspectionEvidence,  // evidence handshake: RELIABLE, bounded and expiring
  TaskEvent,           // immutable task facts: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=512
  CameraStream,     // color/depth: BEST_EFFORT, KEEP_LAST=1
  CameraInfo,       // camera_info: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  GlobalPath,       // paths: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  SystemStatus,     // health/state: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  Event,            // low-rate nav/semantic events: RELIABLE, TRANSIENT_LOCAL
  MapGrid,          // traversability/exploration grids: RELIABLE, TRANSIENT_LOCAL
  LocalRiskGrid,    // local odom risk: RELIABLE, VOLATILE, KEEP_LAST=1, lifespan=500ms
  TfDynamic,        // tf: BEST_EFFORT, KEEP_LAST=100
  TfStatic,         // tf_static: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  LidarPointcloud,  // registered_cloud/terrain_map: BEST_EFFORT, KEEP_LAST=2, lifespan=200ms
  MapCloud,         // live map clouds: BEST_EFFORT, VOLATILE, KEEP_LAST=1
  MapScene,         // coherent scene: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
};

/// Apply a named QoS profile to an existing dds_qos_t*.
/// Does nothing for QosProfile::Default.
inline void apply_qos_profile(dds_qos_t* qos, QosProfile profile) {
  switch (profile) {
    case QosProfile::Default:
      break;
    case QosProfile::SensorStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 256);
      break;
    case QosProfile::RawLidarStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 2);
      dds_qset_lifespan(qos, DDS_MSECS(350));
      dds_qset_resource_limits(qos, 2, 1, 2);
      break;
    case QosProfile::HighFreqState:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 5);
      dds_qset_deadline(qos, DDS_MSECS(20));
      break;
    case QosProfile::LocalizationHealth:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);
      break;
    case QosProfile::ControlCommand:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      break;
    case QosProfile::FinalVelocityCommand:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      dds_qset_lifespan(qos, DDS_MSECS(200));
      break;
    case QosProfile::CommandRequest:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      break;
    case QosProfile::CommandAck:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 64);
      break;
    case QosProfile::OperatorMotionControl:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      break;
    case QosProfile::OperatorMotionSample:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_deadline(qos, DDS_MSECS(50));
      dds_qset_lifespan(qos, DDS_MSECS(350));
      break;
    case QosProfile::OperatorMotionAck:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 64);
      break;
    case QosProfile::OperatorMotionStatus:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::InspectionEvidence:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      dds_qset_deadline(qos, DDS_SECS(5));
      dds_qset_lifespan(qos, DDS_SECS(35));
      break;
    case QosProfile::TaskEvent:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 512);
      break;
    case QosProfile::CameraStream:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::CameraInfo:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::GlobalPath:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::SystemStatus:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::Event:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::MapGrid:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::LocalRiskGrid:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_MSECS(500));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
    case QosProfile::TfDynamic:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 100);
      break;
    case QosProfile::TfStatic:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      break;
    case QosProfile::LidarPointcloud:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 2);
      dds_qset_lifespan(qos, DDS_MSECS(200));
      break;
    case QosProfile::MapCloud:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_MSECS(500));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
    case QosProfile::MapScene:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(100));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      dds_qset_lifespan(qos, DDS_SECS(2));
      dds_qset_resource_limits(qos, 1, 1, 1);
      break;
  }
}

/// Lookup recommended QoS profile by DDS topic name (rt/ prefix).
inline QosProfile qos_for_topic(std::string_view dds_topic) {
  const auto is = [dds_topic](const message::TopicContract& topic) {
    return dds_topic == topic.dds_topic;
  };

  // Sensor streams
  if (is(message::kLidarRawFrame) ||
      is(message::kSimLidarRawFrame) ||
      is(message::kLidarRawPacket))
    return QosProfile::RawLidarStream;
  if (is(message::kImuRaw) || is(message::kSlamOdomPrior))
    return QosProfile::SensorStream;
  // Camera
  if (is(message::kCameraColor) || is(message::kCameraDepth))
    return QosProfile::CameraStream;
  if (is(message::kCameraInfo))
    return QosProfile::CameraInfo;
  // High freq state
  if (is(message::kDriverOdometry) ||
      is(message::kSlamOdometry) ||
      is(message::kSlamStateAtScan) ||
      is(message::kSlamLocalizationQuality) ||
      is(message::kGnssFix) ||
      is(message::kGnssOdom))
    return QosProfile::HighFreqState;
  if (is(message::kSlamLocalizationHealth))
    return QosProfile::LocalizationHealth;
  // Control
  if (is(message::kNavCmdVel))
    return QosProfile::FinalVelocityCommand;
  if (is(message::kNavCommandRequest) ||
      is(message::kNavPlanRequest) ||
      is(message::kNavPlanResult) ||
      is(message::kNavGeofenceCommand) ||
      is(message::kMapsActivationRequest) ||
      is(message::kSlamMapSnapshotRequest) ||
      is(message::kSlamRelocalizationRequest) ||
      is(message::kNavExplorationCommand) ||
      is(message::kNavExplorationSegmentRequest) ||
      is(message::kNavInspectionTaskRequest))
    return QosProfile::CommandRequest;
  if (is(message::kOperatorMotionControl))
    return QosProfile::OperatorMotionControl;
  if (is(message::kOperatorMotionSample))
    return QosProfile::OperatorMotionSample;
  if (is(message::kOperatorMotionAck))
    return QosProfile::OperatorMotionAck;
  if (is(message::kOperatorMotionStatus))
    return QosProfile::OperatorMotionStatus;
  if (is(message::kNavCommandAck) ||
      is(message::kNavGeofenceResponse) ||
      is(message::kMapsActivationAck) ||
      is(message::kSlamMapSnapshotAck) ||
      is(message::kSlamRelocalizationResponse) ||
      is(message::kNavGoalStatus) ||
      is(message::kNavExplorationAck) ||
      is(message::kNavExplorationSegmentAck) ||
      is(message::kNavExplorationSegmentStatus) ||
      is(message::kNavInspectionTaskAck))
    return QosProfile::CommandAck;
  if (is(message::kNavInspectionEvidenceRequest) ||
      is(message::kNavInspectionEvidenceResult))
    return QosProfile::InspectionEvidence;
  if (is(message::kNavInspectionTaskEvent) ||
      is(message::kNavExplorationRunEvent))
    return QosProfile::TaskEvent;
  if (is(message::kNavInspectionStatus) ||
      is(message::kNavState) ||
      is(message::kNavGeofenceAlert) ||
      is(message::kMapsState) ||
      is(message::kDriverControlState))
    return QosProfile::SystemStatus;
  if (is(message::kNavWayPoint) ||
      is(message::kNavMapClearing) ||
      is(message::kNavCloudClearing))
    return QosProfile::ControlCommand;
  // Paths
  if (is(message::kNavGlobalPath) || is(message::kNavLocalPath))
    return QosProfile::GlobalPath;
  // System status
  if (is(message::kGnssStatus))
    return QosProfile::SystemStatus;
  // Event and grid state
  if (is(message::kNavSemanticInstruction))
    return QosProfile::Event;
  if (is(message::kNavLocalTraversability) ||
      is(message::kMapsLocalCollision))
    return QosProfile::LocalRiskGrid;
  if (is(message::kNavTraversability) ||
      is(message::kNavExplorationGrid) ||
      is(message::kNavExplorationSnapshot) ||
      is(message::kNavExplorationExecutionSnapshot) ||
      is(message::kMapsOccupancy) ||
      is(message::kMapsElevation) ||
      is(message::kMapsEsdf))
    return QosProfile::MapGrid;
  // TF
  if (is(message::kTf))
    return QosProfile::TfDynamic;
  if (is(message::kTfStatic))
    return QosProfile::TfStatic;
  // Point clouds
  if (is(message::kSlamRegisteredCloud) ||
      is(message::kSlamMapObservation) ||
      is(message::kSlamMapCloud) ||
      is(message::kSlamCumulativeMapCloud) ||
      is(message::kSlamSavedMapCloud) ||
      is(message::kNavTerrainMap) ||
      is(message::kNavTerrainMapExt))
    return QosProfile::LidarPointcloud;
  if (is(message::kMapsLiveCloud) ||
      is(message::kMapsVoxelCloud) ||
      is(message::kMapsAccumulatedCloud))
    return QosProfile::MapCloud;
  if (is(message::kMapsScene))
    return QosProfile::MapScene;
  // Exploration planner (low-rate, reliable, latching)
  if (is(message::kExplorationWayPoint) ||
      is(message::kExplorationLocalPath) ||
      is(message::kExplorationRuntime) ||
      is(message::kExplorationFinish) ||
      is(message::kExplorationStart))
    return QosProfile::Event;

  return QosProfile::Default;
}

/// RAII convenience: create a QoS object with the given profile applied.
struct QosDeleter { void operator()(dds_qos_t* q) const { if (q) dds_delete_qos(q); } };
using UniqueQos = std::unique_ptr<dds_qos_t, QosDeleter>;

inline UniqueQos make_qos(QosProfile profile) {
  UniqueQos qos(dds_create_qos());
  if (qos && profile != QosProfile::Default) {
    apply_qos_profile(qos.get(), profile);
  }
  return qos;
}

}  // namespace lingtu::dds
