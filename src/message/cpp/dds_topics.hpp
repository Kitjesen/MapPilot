#pragma once

#include <string_view>

// Static native DDS topic/type contract.
// Direct native DDS readers/writers use idlc C-generated types for the matching
// idl_type. ROS 2 message aliases belong in compat/adapters, never here.

namespace lingtu::message {

struct TopicContract {
  std::string_view topic;
  std::string_view dds_topic;
  std::string_view idl_type;
  std::string_view cpp_type;
};

inline constexpr TopicContract kLidarRawFrame{
    // Native SLAM wire: scan-level LivoxFrame, not one SDK packet.
    "/lidar/raw_frame", "rt/lidar/raw_frame",
    "lingtu.dds.LivoxFrame", "lingtu::dds::LivoxFrame"};
inline constexpr TopicContract kLidarRawPacket{
    // Diagnostic-only SDK packet stream. Do not feed this into SLAM.
    "/lidar/raw_packet", "rt/lidar/raw_packet",
    "lingtu.dds.LivoxFrame", "lingtu::dds::LivoxFrame"};
inline constexpr TopicContract kTf{
    "/tf", "rt/tf", "lingtu.dds.TFMessage", "lingtu::dds::TFMessage"};
inline constexpr TopicContract kTfStatic{
    "/tf_static", "rt/tf_static", "lingtu.dds.TFMessage",
    "lingtu::dds::TFMessage"};
inline constexpr TopicContract kImuRaw{
    "/imu/raw", "rt/imu/raw", "lingtu.dds.Imu", "lingtu::dds::Imu"};
inline constexpr TopicContract kSlamOdomPrior{
    "/slam/odom_prior", "rt/slam/odom_prior",
    "lingtu.dds.Odometry", "lingtu::dds::Odometry"};
inline constexpr TopicContract kDriverOdometry{
    "/driver/odometry", "rt/driver/odometry",
    "lingtu.dds.Odometry", "lingtu::dds::Odometry"};
inline constexpr TopicContract kDriverControlState{
    "/driver/control_state", "rt/driver/control_state",
    "lingtu.dds.DriverControlState", "lingtu::dds::DriverControlState"};
inline constexpr TopicContract kSlamOdometry{
    "/slam/odometry", "rt/slam/odometry",
    "lingtu.dds.Odometry", "lingtu::dds::Odometry"};
inline constexpr TopicContract kSlamStateAtScan{
    "/slam/state_at_scan", "rt/slam/state_at_scan",
    "lingtu.dds.Odometry", "lingtu::dds::Odometry"};
inline constexpr TopicContract kSlamRegisteredCloud{
    "/slam/registered_cloud", "rt/slam/registered_cloud",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kSlamMapCloud{
    "/slam/map_cloud", "rt/slam/map_cloud",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kSlamCumulativeMapCloud{
    "/slam/cumulative_map_cloud", "rt/slam/cumulative_map_cloud",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kSlamSavedMapCloud{
    "/slam/saved_map_cloud", "rt/slam/saved_map_cloud",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kCameraColor{
    "/camera/color/image_raw", "rt/camera/color",
    "lingtu.dds.Image", "lingtu::dds::Image"};
inline constexpr TopicContract kCameraDepth{
    "/camera/depth/image_raw", "rt/camera/depth",
    "lingtu.dds.Image", "lingtu::dds::Image"};
inline constexpr TopicContract kCameraInfo{
    "/camera/color/camera_info", "rt/camera/info",
    "lingtu.dds.CameraInfo", "lingtu::dds::CameraInfo"};
inline constexpr TopicContract kGnssFix{
    "/gnss/fix", "rt/gnss/fix",
    "lingtu.dds.GnssFix", "lingtu::dds::GnssFix"};
inline constexpr TopicContract kGnssStatus{
    "/gnss/status", "rt/gnss/status",
    "lingtu.dds.GnssStatus", "lingtu::dds::GnssStatus"};
inline constexpr TopicContract kGnssOdom{
    "/gnss/odom", "rt/gnss/odom",
    "lingtu.dds.Odometry", "lingtu::dds::Odometry"};
inline constexpr TopicContract kSlamMapCommand{
    "/slam/map_command", "rt/slam/map_command",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kSlamMapEvent{
    "/slam/map_event", "rt/slam/map_event",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kSlamRelocalizationRequest{
    "/slam/relocalization/request", "rt/slam/relocalization/request",
    "lingtu.dds.RelocalizationRequest", "lingtu::dds::RelocalizationRequest"};
inline constexpr TopicContract kSlamRelocalizationResponse{
    "/slam/relocalization/response", "rt/slam/relocalization/response",
    "lingtu.dds.RelocalizationResponse", "lingtu::dds::RelocalizationResponse"};
inline constexpr TopicContract kSlamLocalizationQuality{
    "/slam/localization_quality", "rt/slam/localization_quality",
    "lingtu.dds.Float32", "lingtu::dds::Float32"};
inline constexpr TopicContract kSlamLocalizationHealth{
    "/slam/localization_health", "rt/slam/localization_health",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kNavGlobalPath{
    "/nav/global_path", "rt/nav/global_path",
    "lingtu.dds.Path", "lingtu::dds::Path"};
inline constexpr TopicContract kNavLocalPath{
    "/nav/local_path", "rt/nav/local_path",
    "lingtu.dds.Path", "lingtu::dds::Path"};
inline constexpr TopicContract kNavWayPoint{
    "/nav/way_point", "rt/nav/way_point",
    "lingtu.dds.PoseStamped", "lingtu::dds::PoseStamped"};
inline constexpr TopicContract kNavGoalPose{
    "/nav/goal_pose", "rt/nav/goal_pose",
    "lingtu.dds.PoseStamped", "lingtu::dds::PoseStamped"};
inline constexpr TopicContract kNavCancel{
    "/nav/cancel", "rt/nav/cancel", "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kNavCommandRequest{
    "/nav/command/request", "rt/nav/command/request",
    "lingtu.dds.NavigationCommandRequest",
    "lingtu::dds::NavigationCommandRequest"};
inline constexpr TopicContract kNavCommandAck{
    "/nav/command/ack", "rt/nav/command/ack",
    "lingtu.dds.NavigationCommandAck",
    "lingtu::dds::NavigationCommandAck"};
inline constexpr TopicContract kNavExplorationCommand{
    "/nav/exploration/command", "rt/nav/exploration/command",
    "lingtu.dds.ExplorationCommandRequest",
    "lingtu::dds::ExplorationCommandRequest"};
inline constexpr TopicContract kNavExplorationAck{
    "/nav/exploration/ack", "rt/nav/exploration/ack",
    "lingtu.dds.ExplorationCommandAck",
    "lingtu::dds::ExplorationCommandAck"};
inline constexpr TopicContract kNavInspectionCommand{
    "/nav/inspection/command", "rt/nav/inspection/command",
    "lingtu.dds.InspectionCommandRequest",
    "lingtu::dds::InspectionCommandRequest"};
inline constexpr TopicContract kNavInspectionAck{
    "/nav/inspection/ack", "rt/nav/inspection/ack",
    "lingtu.dds.InspectionCommandAck",
    "lingtu::dds::InspectionCommandAck"};
inline constexpr TopicContract kNavInspectionEvidenceRequest{
    "/nav/inspection/evidence/request", "rt/nav/inspection/evidence/request",
    "lingtu.dds.InspectionEvidenceRequest",
    "lingtu::dds::InspectionEvidenceRequest"};
inline constexpr TopicContract kNavInspectionEvidenceResult{
    "/nav/inspection/evidence/result", "rt/nav/inspection/evidence/result",
    "lingtu.dds.InspectionEvidenceResult",
    "lingtu::dds::InspectionEvidenceResult"};
inline constexpr TopicContract kNavInspectionStatus{
    "/nav/inspection/status", "rt/nav/inspection/status",
    "lingtu.dds.InspectionStatus",
    "lingtu::dds::InspectionStatus"};
inline constexpr TopicContract kNavSemanticInstruction{
    "/nav/semantic/instruction", "rt/nav/semantic/instruction",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kNavTraversability{
    "/nav/traversability", "rt/nav/traversability",
    "lingtu.dds.OccupancyGrid", "lingtu::dds::OccupancyGrid"};
inline constexpr TopicContract kNavTerrainMap{
    "/nav/terrain_map", "rt/nav/terrain_map",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kNavTerrainMapExt{
    "/nav/terrain_map_ext", "rt/nav/terrain_map_ext",
    "lingtu.dds.PointCloud2", "lingtu::dds::PointCloud2"};
inline constexpr TopicContract kNavMapClearing{
    "/nav/map_clearing", "rt/nav/map_clearing",
    "lingtu.dds.Bool", "lingtu::dds::Bool"};
inline constexpr TopicContract kNavCloudClearing{
    "/nav/cloud_clearing", "rt/nav/cloud_clearing",
    "lingtu.dds.Bool", "lingtu::dds::Bool"};
inline constexpr TopicContract kNavCmdVel{
    "/nav/cmd_vel", "rt/nav/cmd_vel",
    "lingtu.dds.FinalVelocityCommand",
    "lingtu::dds::FinalVelocityCommand"};
inline constexpr TopicContract kNavTeleopCmdVel{
    "/nav/teleop_cmd_vel", "rt/nav/teleop_cmd_vel",
    "lingtu.dds.TwistStamped", "lingtu::dds::TwistStamped"};
inline constexpr TopicContract kNavExplorationGrid{
    "/nav/exploration_grid", "rt/nav/exploration_grid",
    "lingtu.dds.OccupancyGrid", "lingtu::dds::OccupancyGrid"};
inline constexpr TopicContract kNavExplorationSnapshot{
    "/nav/exploration_snapshot", "rt/nav/exploration_snapshot",
    "lingtu.dds.ExplorationGrid", "lingtu::dds::ExplorationGrid"};

// ── Exploration planner topics ────────────────────────────────────────
inline constexpr TopicContract kExplorationWayPoint{
    "/exploration/way_point", "rt/exploration/way_point",
    "lingtu.dds.PointStamped", "lingtu::dds::PointStamped"};
inline constexpr TopicContract kExplorationLocalPath{
    "/exploration/local_path", "rt/exploration/local_path",
    "lingtu.dds.Path", "lingtu::dds::Path"};
inline constexpr TopicContract kExplorationRuntime{
    "/exploration/runtime", "rt/exploration/runtime",
    "lingtu.dds.Float32", "lingtu::dds::Float32"};
inline constexpr TopicContract kExplorationFinish{
    "/exploration/finish", "rt/exploration/finish",
    "lingtu.dds.Bool", "lingtu::dds::Bool"};
inline constexpr TopicContract kExplorationStart{
    "/exploration/start", "rt/exploration/start",
    "lingtu.dds.Bool", "lingtu::dds::Bool"};

inline constexpr TopicContract kTopicContracts[] = {
    kTf,
    kTfStatic,
    kLidarRawFrame,
    kLidarRawPacket,
    kImuRaw,
    kSlamOdomPrior,
    kDriverOdometry,
    kDriverControlState,
    kSlamOdometry,
    kSlamStateAtScan,
    kSlamRegisteredCloud,
    kSlamMapCloud,
    kSlamCumulativeMapCloud,
    kSlamSavedMapCloud,
    kCameraColor,
    kCameraDepth,
    kCameraInfo,
    kGnssFix,
    kGnssStatus,
    kGnssOdom,
    kSlamMapCommand,
    kSlamMapEvent,
    kSlamRelocalizationRequest,
    kSlamRelocalizationResponse,
    kSlamLocalizationQuality,
    kSlamLocalizationHealth,
    kNavGlobalPath,
    kNavLocalPath,
    kNavWayPoint,
    kNavGoalPose,
    kNavCancel,
    kNavCommandRequest,
    kNavCommandAck,
    kNavExplorationCommand,
    kNavExplorationAck,
    kNavInspectionCommand,
    kNavInspectionAck,
    kNavInspectionEvidenceRequest,
    kNavInspectionEvidenceResult,
    kNavInspectionStatus,
    kNavSemanticInstruction,
    kNavTraversability,
    kNavTerrainMap,
    kNavTerrainMapExt,
    kNavMapClearing,
    kNavCloudClearing,
    kNavCmdVel,
    kNavTeleopCmdVel,
    kNavExplorationGrid,
    kNavExplorationSnapshot,
    kExplorationWayPoint,
    kExplorationLocalPath,
    kExplorationRuntime,
    kExplorationFinish,
    kExplorationStart,
};

inline constexpr const TopicContract* find_topic_contract(std::string_view topic) noexcept {
  for (const auto& contract : kTopicContracts) {
    if (contract.topic == topic) {
      return &contract;
    }
  }
  return nullptr;
}

}  // namespace lingtu::message
