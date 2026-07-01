#pragma once

#include <string_view>

// Static native DDS topic/type contract plus optional ROS 2 adapter aliases.
// Direct CycloneDDS C++ data readers/writers must use idlcxx-generated types
// for the matching idl_type; the fallback structs below are contract tags only.

#if __has_include(<livox_ros_driver2/msg/custom_msg.hpp>)
#include <livox_ros_driver2/msg/custom_msg.hpp>
#define LINGTU_MESSAGE_HAS_LIVOX_ROS_DRIVER2 1
#else
#define LINGTU_MESSAGE_HAS_LIVOX_ROS_DRIVER2 0
#endif

#if __has_include(<geometry_msgs/msg/pose_stamped.hpp>) && __has_include(<geometry_msgs/msg/twist_stamped.hpp>)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#define LINGTU_MESSAGE_HAS_GEOMETRY_MSGS 1
#else
#define LINGTU_MESSAGE_HAS_GEOMETRY_MSGS 0
#endif

#if __has_include(<nav_msgs/msg/odometry.hpp>) && __has_include(<nav_msgs/msg/path.hpp>) && __has_include(<nav_msgs/msg/occupancy_grid.hpp>)
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#define LINGTU_MESSAGE_HAS_NAV_MSGS 1
#else
#define LINGTU_MESSAGE_HAS_NAV_MSGS 0
#endif

#if __has_include(<sensor_msgs/msg/imu.hpp>) && __has_include(<sensor_msgs/msg/point_cloud2.hpp>)
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#define LINGTU_MESSAGE_HAS_SENSOR_MSGS 1
#else
#define LINGTU_MESSAGE_HAS_SENSOR_MSGS 0
#endif

#if __has_include(<std_msgs/msg/float32.hpp>) && __has_include(<std_msgs/msg/string.hpp>)
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#define LINGTU_MESSAGE_HAS_STD_MSGS 1
#else
#define LINGTU_MESSAGE_HAS_STD_MSGS 0
#endif

namespace lingtu::message {

#if LINGTU_MESSAGE_HAS_LIVOX_ROS_DRIVER2
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
#else
struct LivoxCustomMsg {};
#endif

#if LINGTU_MESSAGE_HAS_SENSOR_MSGS
using Imu = sensor_msgs::msg::Imu;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
#else
struct Imu {};
struct PointCloud2 {};
#endif

#if LINGTU_MESSAGE_HAS_NAV_MSGS
using Odometry = nav_msgs::msg::Odometry;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Path = nav_msgs::msg::Path;
#else
struct Odometry {};
struct OccupancyGrid {};
struct Path {};
#endif

#if LINGTU_MESSAGE_HAS_GEOMETRY_MSGS
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;
#else
struct PoseStamped {};
struct TwistStamped {};
#endif

#if LINGTU_MESSAGE_HAS_STD_MSGS
using Float32 = std_msgs::msg::Float32;
using String = std_msgs::msg::String;
#else
struct Float32 {};
struct String {};
#endif

struct TopicContract {
  std::string_view topic;
  std::string_view dds_topic;
  std::string_view idl_type;
  std::string_view cpp_type;
};

inline constexpr TopicContract kLidarRawFrame{
    // Native hardware wire. ROS2/Livox adapters must translate into this schema
    // before entering the SLAM hot path.
    "/lidar/raw_frame", "rt/lidar/raw_frame",
    "lingtu.dds.LivoxFrame", "lingtu::dds::LivoxFrame"};
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
inline constexpr TopicContract kSlamLocalizationQuality{
    "/slam/localization_quality", "rt/slam/localization_quality",
    "lingtu.dds.Float32", "lingtu::dds::Float32"};
inline constexpr TopicContract kSlamLocalizationHealth{
    "/slam/localization_health", "rt/slam/localization_health",
    "lingtu.dds.Text", "lingtu::dds::Text"};
inline constexpr TopicContract kNavGlobalPath{
    "/nav/global_path", "rt/nav/global_path",
    "nav_msgs/msg/Path", "nav_msgs::msg::Path"};
inline constexpr TopicContract kNavLocalPath{
    "/nav/local_path", "rt/nav/local_path",
    "nav_msgs/msg/Path", "nav_msgs::msg::Path"};
inline constexpr TopicContract kNavWayPoint{
    "/nav/way_point", "rt/nav/way_point",
    "geometry_msgs/msg/PoseStamped", "geometry_msgs::msg::PoseStamped"};
inline constexpr TopicContract kNavGoalPose{
    "/nav/goal_pose", "rt/nav/goal_pose",
    "geometry_msgs/msg/PoseStamped", "geometry_msgs::msg::PoseStamped"};
inline constexpr TopicContract kNavCancel{
    "/nav/cancel", "rt/nav/cancel", "std_msgs/msg/String", "std_msgs::msg::String"};
inline constexpr TopicContract kNavSemanticInstruction{
    "/nav/semantic/instruction", "rt/nav/semantic/instruction",
    "std_msgs/msg/String", "std_msgs::msg::String"};
inline constexpr TopicContract kNavCmdVel{
    "/nav/cmd_vel", "rt/nav/cmd_vel",
    "geometry_msgs/msg/TwistStamped", "geometry_msgs::msg::TwistStamped"};
inline constexpr TopicContract kNavExplorationGrid{
    "/nav/exploration_grid", "rt/nav/exploration_grid",
    "nav_msgs/msg/OccupancyGrid", "nav_msgs::msg::OccupancyGrid"};

inline constexpr TopicContract kTopicContracts[] = {
    kLidarRawFrame,
    kImuRaw,
    kSlamOdometry,
    kSlamStateAtScan,
    kSlamRegisteredCloud,
    kSlamMapCloud,
    kSlamCumulativeMapCloud,
    kSlamSavedMapCloud,
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
