#pragma once
// Unified DDS QoS Profiles for LingTu native services.
// Semantically aligned with config/qos_profiles.yaml.

#include "dds/dds.h"
#include <memory>
#include <string_view>

namespace lingtu::dds {

enum class QosProfile {
  Default,           // CycloneDDS defaults (no explicit QoS)
  SensorStream,     // lidar/imu: BEST_EFFORT, KEEP_LAST=256
  HighFreqState,    // odometry/state: BEST_EFFORT, KEEP_LAST=5, deadline=20ms
  ControlCommand,   // stop/control: RELIABLE, KEEP_LAST=1, deadline=50ms
  FinalVelocityCommand,  // final cmd_vel plus a 200ms DDS lifespan
  CommandRequest,   // typed requests: RELIABLE, VOLATILE, KEEP_LAST=32
  CommandAck,       // typed replies: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=64
  InspectionEvidence,  // evidence handshake: RELIABLE, bounded and expiring
  CameraStream,     // color/depth: BEST_EFFORT, KEEP_LAST=1
  CameraInfo,       // camera_info: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  GlobalPath,       // paths: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  SystemStatus,     // health/state: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  Event,            // low-rate nav/semantic events: RELIABLE, TRANSIENT_LOCAL
  MapGrid,          // traversability/exploration grids: RELIABLE, TRANSIENT_LOCAL
  TfDynamic,        // tf: BEST_EFFORT, KEEP_LAST=100
  TfStatic,         // tf_static: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=1
  LidarPointcloud,  // registered_cloud/terrain_map: BEST_EFFORT, KEEP_LAST=2, lifespan=200ms
  SemanticScene,    // scene_graph: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST=2, lifespan=5s
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
    case QosProfile::HighFreqState:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 5);
      dds_qset_deadline(qos, DDS_MSECS(20));
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
    case QosProfile::InspectionEvidence:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
      dds_qset_deadline(qos, DDS_SECS(5));
      dds_qset_lifespan(qos, DDS_SECS(35));
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
    case QosProfile::SemanticScene:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 2);
      dds_qset_lifespan(qos, DDS_SECS(5));
      break;
  }
}

/// Lookup recommended QoS profile by DDS topic name (rt/ prefix).
inline QosProfile qos_for_topic(std::string_view dds_topic) {
  // Sensor streams
  if (dds_topic == "rt/lidar/raw_frame" ||
      dds_topic == "rt/lidar/raw_packet" ||
      dds_topic == "rt/imu/raw" ||
      dds_topic == "rt/slam/odom_prior")
    return QosProfile::SensorStream;
  // Camera
  if (dds_topic == "rt/camera/color" || dds_topic == "rt/camera/depth")
    return QosProfile::CameraStream;
  if (dds_topic == "rt/camera/info")
    return QosProfile::CameraInfo;
  // High freq state
  if (dds_topic == "rt/driver/odometry" ||
      dds_topic == "rt/slam/odometry" ||
      dds_topic == "rt/gnss/fix" ||
      dds_topic == "rt/gnss/odom")
    return QosProfile::HighFreqState;
  // Control
  if (dds_topic == "rt/nav/cmd_vel")
    return QosProfile::FinalVelocityCommand;
  if (dds_topic == "rt/nav/stop")
    return QosProfile::ControlCommand;
  if (dds_topic == "rt/nav/command/request" ||
      dds_topic == "rt/nav/inspection/command")
    return QosProfile::CommandRequest;
  if (dds_topic == "rt/nav/command/ack" ||
      dds_topic == "rt/nav/inspection/ack")
    return QosProfile::CommandAck;
  if (dds_topic == "rt/nav/inspection/evidence/request" ||
      dds_topic == "rt/nav/inspection/evidence/result")
    return QosProfile::InspectionEvidence;
  if (dds_topic == "rt/nav/inspection/status" ||
      dds_topic == "rt/driver/control_state")
    return QosProfile::SystemStatus;
  if (dds_topic == "rt/nav/way_point" ||
      dds_topic == "rt/nav/teleop_cmd_vel" ||
      dds_topic == "rt/nav/cancel" ||
      dds_topic == "rt/nav/map_clearing" ||
      dds_topic == "rt/nav/cloud_clearing")
    return QosProfile::ControlCommand;
  // Paths
  if (dds_topic == "rt/nav/global_path" || dds_topic == "rt/nav/local_path")
    return QosProfile::GlobalPath;
  // System status
  if (dds_topic == "rt/nav/health_status" ||
      dds_topic == "rt/robot_state" ||
      dds_topic == "rt/gnss/status")
    return QosProfile::SystemStatus;
  // Event and grid state
  if (dds_topic == "rt/nav/goal_pose" ||
      dds_topic == "rt/nav/geofence_boundary" ||
      dds_topic == "rt/nav/semantic/instruction" ||
      dds_topic == "rt/nav/semantic/resolved_goal" ||
      dds_topic == "rt/nav/semantic/status")
    return QosProfile::Event;
  if (dds_topic == "rt/nav/traversability" ||
      dds_topic == "rt/nav/exploration_grid")
    return QosProfile::MapGrid;
  // TF
  if (dds_topic == "rt/tf")
    return QosProfile::TfDynamic;
  if (dds_topic == "rt/tf_static")
    return QosProfile::TfStatic;
  // Point clouds
  if (dds_topic == "rt/slam/registered_cloud" ||
      dds_topic == "rt/slam/map_cloud" ||
      dds_topic == "rt/slam/cumulative_map_cloud" ||
      dds_topic == "rt/slam/saved_map_cloud" ||
      dds_topic == "rt/nav/terrain_map" ||
      dds_topic == "rt/nav/terrain_map_ext" ||
      dds_topic == "rt/nav/scan_cloud")
    return QosProfile::LidarPointcloud;
  // Semantic
  if (dds_topic == "rt/nav/semantic/scene_graph")
    return QosProfile::SemanticScene;
  // Exploration planner (low-rate, reliable, latching)
  if (dds_topic == "rt/exploration/way_point" ||
      dds_topic == "rt/exploration/local_path" ||
      dds_topic == "rt/exploration/runtime" ||
      dds_topic == "rt/exploration/finish" ||
      dds_topic == "rt/exploration/start")
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
