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
inline constexpr TopicContract kSlamMapCommand{
    "/slam/map_command", "rt/slam/map_command",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kSlamMapEvent{
    "/slam/map_event", "rt/slam/map_event",
    "lingtu.dds.Text", "lingtu::dds::Text"};
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
inline constexpr TopicContract kNavSemanticInstruction{
    "/nav/semantic/instruction", "rt/nav/semantic/instruction",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kNavCmdVel{
    "/nav/cmd_vel", "rt/nav/cmd_vel",
    "lingtu.dds.TwistStamped", "lingtu::dds::TwistStamped"};
inline constexpr TopicContract kNavExplorationGrid{
    "/nav/exploration_grid", "rt/nav/exploration_grid",
    "lingtu.dds.OccupancyGrid", "lingtu::dds::OccupancyGrid"};

inline constexpr TopicContract kTopicContracts[] = {
    kTf,
    kTfStatic,
    kLidarRawFrame,
    kLidarRawPacket,
    kImuRaw,
    kSlamOdometry,
    kSlamStateAtScan,
    kSlamRegisteredCloud,
    kSlamMapCloud,
    kSlamCumulativeMapCloud,
    kSlamSavedMapCloud,
    kSlamMapCommand,
    kSlamMapEvent,
    kSlamLocalizationQuality,
    kSlamLocalizationHealth,
    kNavGlobalPath,
    kNavLocalPath,
    kNavWayPoint,
    kNavGoalPose,
    kNavCancel,
    kNavSemanticInstruction,
    kNavCmdVel,
    kNavExplorationGrid,
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
