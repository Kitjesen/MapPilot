/**
 * pathFollower.cpp — ROS2 Thin Shell
 *
 * 核心算法: nav_core::computeControl (path_follower_core.hpp)
 * 本文件只负责: ROS2 通信 (pub/sub/参数) + 类型转换
 */
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include "nav_core/path_follower_core.hpp"

using namespace std;

const double PI = M_PI;  // 使用系统精确值，避免截断误差

/**
 * @class PathFollower
 * @brief Pure Pursuit path following controller with adaptive lookahead
 */
class PathFollower : public rclcpp::Node
{
public:
  PathFollower() : Node("pathFollower")
  {
    // --- Parameter Declaration ---
    declare_parameter<double>("sensorOffsetX", sensorOffsetX_);
    declare_parameter<double>("sensorOffsetY", sensorOffsetY_);
    declare_parameter<int>("pubSkipNum", pubSkipNum_);
    declare_parameter<bool>("twoWayDrive", twoWayDrive_);
    declare_parameter<double>("lookAheadDis", lookAheadDis_);
    declare_parameter<double>("baseLookAheadDis", baseLookAheadDis_);
    declare_parameter<double>("lookAheadRatio", lookAheadRatio_);
    declare_parameter<double>("minLookAheadDis", minLookAheadDis_);
    declare_parameter<double>("maxLookAheadDis", maxLookAheadDis_);
    declare_parameter<double>("yawRateGain", yawRateGain_);
    declare_parameter<double>("stopYawRateGain", stopYawRateGain_);
    declare_parameter<double>("maxYawRate", maxYawRate_);
    declare_parameter<double>("maxSpeed", maxSpeed_);
    declare_parameter<double>("maxAccel", maxAccel_);
    declare_parameter<double>("switchTimeThre", switchTimeThre_);
    declare_parameter<double>("dirDiffThre", dirDiffThre_);
    declare_parameter<double>("omniDirGoalThre", omniDirGoalThre_);
    declare_parameter<double>("omniDirDiffThre", omniDirDiffThre_);
    declare_parameter<double>("stopDisThre", stopDisThre_);
    declare_parameter<double>("slowDwnDisThre", slowDwnDisThre_);
    declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow_);
    declare_parameter<double>("inclRateThre", inclRateThre_);
    declare_parameter<double>("slowRate1", slowRate1_);
    declare_parameter<double>("slowRate2", slowRate2_);
    declare_parameter<double>("slowRate3", slowRate3_);
    declare_parameter<double>("slowTime1", slowTime1_);
    declare_parameter<double>("slowTime2", slowTime2_);
    declare_parameter<bool>("useInclToStop", useInclToStop_);
    declare_parameter<double>("inclThre", inclThre_);
    declare_parameter<double>("stopTime", stopTime_);
    declare_parameter<bool>("noRotAtStop", noRotAtStop_);
    declare_parameter<bool>("noRotAtGoal", noRotAtGoal_);
    declare_parameter<bool>("autonomyMode", autonomyMode_);
    declare_parameter<double>("autonomySpeed", autonomySpeed_);
    declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay_);
    declare_parameter<int>("joy_axis_fwd",       4);
    declare_parameter<int>("joy_axis_left",      3);
    declare_parameter<int>("joy_axis_yaw",       0);
    declare_parameter<int>("joy_axis_autonomy",  2);
    declare_parameter<int>("joy_axis_obstacle",  5);
    declare_parameter<double>("stuck_timeout",   10.0);
    declare_parameter<double>("stuck_dist_thre",  0.15);

    // --- Get Parameters ---
    sensorOffsetX_ = get_parameter("sensorOffsetX").as_double();
    sensorOffsetY_ = get_parameter("sensorOffsetY").as_double();
    pubSkipNum_ = get_parameter("pubSkipNum").as_int();
    twoWayDrive_ = get_parameter("twoWayDrive").as_bool();
    lookAheadDis_ = get_parameter("lookAheadDis").as_double();
    baseLookAheadDis_ = get_parameter("baseLookAheadDis").as_double();
    lookAheadRatio_ = get_parameter("lookAheadRatio").as_double();
    minLookAheadDis_ = get_parameter("minLookAheadDis").as_double();
    maxLookAheadDis_ = get_parameter("maxLookAheadDis").as_double();
    yawRateGain_ = get_parameter("yawRateGain").as_double();
    stopYawRateGain_ = get_parameter("stopYawRateGain").as_double();
    maxYawRate_ = get_parameter("maxYawRate").as_double();
    maxSpeed_ = get_parameter("maxSpeed").as_double();
    maxAccel_ = get_parameter("maxAccel").as_double();
    switchTimeThre_ = get_parameter("switchTimeThre").as_double();
    dirDiffThre_ = get_parameter("dirDiffThre").as_double();
    omniDirGoalThre_ = get_parameter("omniDirGoalThre").as_double();
    omniDirDiffThre_ = get_parameter("omniDirDiffThre").as_double();
    stopDisThre_ = get_parameter("stopDisThre").as_double();
    slowDwnDisThre_ = get_parameter("slowDwnDisThre").as_double();
    useInclRateToSlow_ = get_parameter("useInclRateToSlow").as_bool();
    inclRateThre_ = get_parameter("inclRateThre").as_double();
    slowRate1_ = get_parameter("slowRate1").as_double();
    slowRate2_ = get_parameter("slowRate2").as_double();
    slowRate3_ = get_parameter("slowRate3").as_double();
    slowTime1_ = get_parameter("slowTime1").as_double();
    slowTime2_ = get_parameter("slowTime2").as_double();
    useInclToStop_ = get_parameter("useInclToStop").as_bool();
    inclThre_ = get_parameter("inclThre").as_double();
    stopTime_ = get_parameter("stopTime").as_double();
    noRotAtStop_ = get_parameter("noRotAtStop").as_bool();
    noRotAtGoal_ = get_parameter("noRotAtGoal").as_bool();
    autonomyMode_ = get_parameter("autonomyMode").as_bool();
    autonomySpeed_ = get_parameter("autonomySpeed").as_double();
    joyToSpeedDelay_ = get_parameter("joyToSpeedDelay").as_double();
    joy_axis_fwd_      = get_parameter("joy_axis_fwd").as_int();
    joy_axis_left_     = get_parameter("joy_axis_left").as_int();
    joy_axis_yaw_      = get_parameter("joy_axis_yaw").as_int();
    joy_axis_autonomy_ = get_parameter("joy_axis_autonomy").as_int();
    joy_axis_obstacle_ = get_parameter("joy_axis_obstacle").as_int();
    stuckTimeout_  = get_parameter("stuck_timeout").as_double();
    stuckDistThre_ = get_parameter("stuck_dist_thre").as_double();

    // --- Dynamic Parameter Callback ---
    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&PathFollower::onParamChange, this, std::placeholders::_1));

    // --- ROS Interfaces ---
    subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 5, std::bind(&PathFollower::odomHandler, this, std::placeholders::_1));

    subPath_ = create_subscription<nav_msgs::msg::Path>(
        "/path", 5, std::bind(&PathFollower::pathHandler, this, std::placeholders::_1));

    subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, std::bind(&PathFollower::joystickHandler, this, std::placeholders::_1));

    subSpeed_ = create_subscription<std_msgs::msg::Float32>(
        "/speed", 5, std::bind(&PathFollower::speedHandler, this, std::placeholders::_1));

    subStop_ = create_subscription<std_msgs::msg::Int8>(
        "/stop", 5, std::bind(&PathFollower::stopHandler, this, std::placeholders::_1));

    subSlowDown_ = create_subscription<std_msgs::msg::Int8>(
        "/slow_down", 5, std::bind(&PathFollower::slowDownHandler, this, std::placeholders::_1));

    pubSpeed_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
    pubGoalStatus_ = create_publisher<std_msgs::msg::String>("/nav/planner_status", 10);

    // --- Sync params to nav_core ---
    syncCoreParams();

    // --- Initialization ---
    if (autonomyMode_) {
      joySpeed_ = autonomySpeed_ / maxSpeed_;
      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    }

    // Control loop timer (100Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&PathFollower::controlLoop, this));
  }

private:
  // --- Parameters ---
  double sensorOffsetX_ = 0;
  double sensorOffsetY_ = 0;
  int pubSkipNum_ = 1;
  bool twoWayDrive_ = true;
  double lookAheadDis_ = 0.5;
  double baseLookAheadDis_ = 0.3;
  double lookAheadRatio_ = 0.5;
  double minLookAheadDis_ = 0.2;
  double maxLookAheadDis_ = 2.0;
  double yawRateGain_ = 7.5;
  double stopYawRateGain_ = 7.5;
  double maxYawRate_ = 45.0;
  double maxSpeed_ = 1.0;
  double maxAccel_ = 1.0;
  double switchTimeThre_ = 1.0;
  double dirDiffThre_ = 0.1;
  double omniDirGoalThre_ = 1.0;
  double omniDirDiffThre_ = 1.5;
  double stopDisThre_ = 0.2;
  double slowDwnDisThre_ = 1.0;
  bool useInclRateToSlow_ = false;
  double inclRateThre_ = 120.0;
  double slowRate1_ = 0.25;
  double slowRate2_ = 0.5;
  double slowRate3_ = 0.75;
  double slowTime1_ = 2.0;
  double slowTime2_ = 2.0;
  bool useInclToStop_ = false;
  double inclThre_ = 45.0;
  double stopTime_ = 5.0;
  bool noRotAtStop_ = false;
  bool noRotAtGoal_ = true;
  bool autonomyMode_ = false;
  double autonomySpeed_ = 1.0;
  double joyToSpeedDelay_ = 2.0;
  int joy_axis_fwd_      = 4;
  int joy_axis_left_     = 3;
  int joy_axis_yaw_      = 0;
  int joy_axis_autonomy_ = 2;
  int joy_axis_obstacle_ = 5;

  // --- Stuck Detection Parameters ---
  double stuckTimeout_   = 10.0;
  double stuckDistThre_  = 0.15;

  // --- Dynamic Parameter Callback ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  rcl_interfaces::msg::SetParametersResult
  onParamChange(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &p : params) {
      const auto &name = p.get_name();
      if (name == "yawRateGain") yawRateGain_ = p.as_double();
      else if (name == "stopYawRateGain") stopYawRateGain_ = p.as_double();
      else if (name == "maxYawRate") maxYawRate_ = p.as_double();
      else if (name == "maxSpeed") maxSpeed_ = p.as_double();
      else if (name == "maxAccel") maxAccel_ = p.as_double();
      else if (name == "lookAheadDis") lookAheadDis_ = p.as_double();
      else if (name == "baseLookAheadDis") baseLookAheadDis_ = p.as_double();
      else if (name == "lookAheadRatio") lookAheadRatio_ = p.as_double();
      else if (name == "minLookAheadDis") minLookAheadDis_ = p.as_double();
      else if (name == "maxLookAheadDis") maxLookAheadDis_ = p.as_double();
      else if (name == "switchTimeThre") switchTimeThre_ = p.as_double();
      else if (name == "dirDiffThre") dirDiffThre_ = p.as_double();
      else if (name == "inclRateThre") inclRateThre_ = p.as_double();
      else if (name == "slowRate1") slowRate1_ = p.as_double();
      else if (name == "slowRate2") slowRate2_ = p.as_double();
      else if (name == "slowRate3") slowRate3_ = p.as_double();
      else if (name == "slowTime1") slowTime1_ = p.as_double();
      else if (name == "slowTime2") slowTime2_ = p.as_double();
      else if (name == "inclThre") inclThre_ = p.as_double();
      else if (name == "stopTime") stopTime_ = p.as_double();
      else if (name == "stopDisThre") stopDisThre_ = p.as_double();
      else if (name == "slowDwnDisThre") slowDwnDisThre_ = p.as_double();
      else if (name == "autonomySpeed") autonomySpeed_ = p.as_double();
      else if (name == "pubSkipNum") pubSkipNum_ = p.as_int();
      else if (name == "useInclRateToSlow") useInclRateToSlow_ = p.as_bool();
      else if (name == "useInclToStop") useInclToStop_ = p.as_bool();
      else if (name == "noRotAtStop") noRotAtStop_ = p.as_bool();
      else if (name == "noRotAtGoal") noRotAtGoal_ = p.as_bool();
      else if (name == "twoWayDrive") twoWayDrive_ = p.as_bool();
      else {
        RCLCPP_WARN(get_logger(), "Unknown dynamic param: %s", name.c_str());
      }
    }
    syncCoreParams();
    return result;
  }

  // --- State Variables ---
  float joySpeed_ = 0;
  float joySpeedRaw_ = 0;
  float joyYaw_ = 0;
  float joyManualFwd_ = 0;
  float joyManualLeft_ = 0;
  float joyManualYaw_ = 0;
  int safetyStop_ = 0;
  int stopClearCount_ = 0;  // 连续收到 stop=0 的计数, 防止单发布者误清除
  int slowDown_ = 0;

  float vehicleX_ = 0;
  float vehicleY_ = 0;
  float vehicleZ_ = 0;
  float vehicleRoll_ = 0;
  float vehiclePitch_ = 0;
  float vehicleYaw_ = 0;

  float vehicleXRec_ = 0;
  float vehicleYRec_ = 0;
  float vehicleZRec_ = 0;
  float vehicleRollRec_ = 0;
  float vehiclePitchRec_ = 0;
  float vehicleYawRec_ = 0;
  // P5: cos/sin(vehicleYawRec_) 缓存（路径开始时设一次，控制循环 100Hz 复用）
  float cosVehicleYawRec_ = 1.0f;
  float sinVehicleYawRec_ = 0.0f;

  float vehicleYawRate_ = 0;
  float vehicleSpeed_ = 0;

  double odomTime_ = 0;
  double joyTime_ = 0;
  double slowInitTime_ = 0;
  double stopInitTime_ = 0;
  int pathPointID_ = 0;
  int lastPathPointID_ = 0;
  int lastPathSize_ = 0;
  bool pathInit_ = false;
  bool navFwd_ = true;
  double switchTime_ = 0;

  int pubSkipCount_ = 0;
  bool manualMode_ = false;

  // --- Stuck Detection State ---
  rclcpp::Time stuckCheckTime_;
  double stuckCheckX_    = 0.0;
  double stuckCheckY_    = 0.0;
  bool stuckCheckInit_   = false;
  bool stuckWarnPublished_ = false;  // R10: 渐进警告已发布标记

  // R10-3: 卡死恢复确认 — 需连续 N 帧速度 >阈值才清除卡死状态
  int stuckRecoverCount_   = 0;
  bool stuckRecovering_    = false;
  static constexpr int kStuckRecoverFrames = 3;
  static constexpr double kStuckRecoverSpeed = 0.05;  // m/s

  nav_msgs::msg::Path path_;

  // ── nav_core 算法 (替代 controlLoop 中的手动数学) ──
  nav_core::PathFollowerParams coreParams_;
  nav_core::PathFollowerState  coreState_;

  void syncCoreParams() {
    coreParams_.sensorOffsetX    = sensorOffsetX_;
    coreParams_.sensorOffsetY    = sensorOffsetY_;
    coreParams_.baseLookAheadDis = baseLookAheadDis_;
    coreParams_.lookAheadRatio   = lookAheadRatio_;
    coreParams_.minLookAheadDis  = minLookAheadDis_;
    coreParams_.maxLookAheadDis  = maxLookAheadDis_;
    coreParams_.yawRateGain      = yawRateGain_;
    coreParams_.stopYawRateGain  = stopYawRateGain_;
    coreParams_.maxYawRate       = maxYawRate_;
    coreParams_.maxSpeed         = maxSpeed_;
    coreParams_.maxAccel         = maxAccel_;
    coreParams_.switchTimeThre   = switchTimeThre_;
    coreParams_.dirDiffThre      = dirDiffThre_;
    coreParams_.omniDirGoalThre  = omniDirGoalThre_;
    coreParams_.omniDirDiffThre  = omniDirDiffThre_;
    coreParams_.stopDisThre      = stopDisThre_;
    coreParams_.slowDwnDisThre   = slowDwnDisThre_;
    coreParams_.twoWayDrive      = twoWayDrive_;
    coreParams_.noRotAtGoal      = noRotAtGoal_;
  }

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subStop_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subSlowDown_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeed_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubGoalStatus_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goalReachedPublished_ = false;

  // --- Callbacks ---
  void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
  {
    odomTime_ = rclcpp::Time(odomIn->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll_ = roll;
    vehiclePitch_ = pitch;
    vehicleYaw_ = yaw;
    // P6: cos/sin(yaw) 各用两次，缓存一次避免重复计算
    const double cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
    vehicleX_ = odomIn->pose.pose.position.x - cosYaw * sensorOffsetX_ + sinYaw * sensorOffsetY_;
    vehicleY_ = odomIn->pose.pose.position.y - sinYaw * sensorOffsetX_ - cosYaw * sensorOffsetY_;
    vehicleZ_ = odomIn->pose.pose.position.z;

    // Stuck detection
    if (!stuckCheckInit_) {
      stuckCheckTime_ = now();
      stuckCheckX_    = vehicleX_;
      stuckCheckY_    = vehicleY_;
      stuckCheckInit_ = true;
      stuckWarnPublished_ = false;
    } else {
      double elapsed = (now() - stuckCheckTime_).seconds();
      double sdx = vehicleX_ - stuckCheckX_;
      double sdy = vehicleY_ - stuckCheckY_;
      double moved = std::sqrt(sdx*sdx + sdy*sdy);
      bool hasActivePath = path_.poses.size() > 0 && safetyStop_ == 0;

      // R10-1: 渐进警告 — 5s 时发 WARN_STUCK, 10s 时发 STUCK
      if (hasActivePath && moved < stuckDistThre_ && elapsed >= stuckTimeout_ * 0.5
          && !stuckWarnPublished_) {
        auto warnMsg = std_msgs::msg::String();
        warnMsg.data = "WARN_STUCK";
        pubGoalStatus_->publish(warnMsg);
        stuckWarnPublished_ = true;
        RCLCPP_WARN(get_logger(),
          "Stuck warning: moved %.3fm in %.1fs (threshold=%.1fs)",
          moved, elapsed, stuckTimeout_);
      }

      if (elapsed >= stuckTimeout_) {
        if (hasActivePath && moved < stuckDistThre_) {
          auto stuckMsg = std_msgs::msg::String();
          stuckMsg.data = "STUCK";
          pubGoalStatus_->publish(stuckMsg);
          stuckRecovering_ = true;
          stuckRecoverCount_ = 0;
          RCLCPP_WARN(get_logger(),
            "Stuck detected: moved %.3fm in %.1fs (thre=%.3fm)", moved, elapsed, stuckDistThre_);
        }
        // 重置检查点
        stuckCheckTime_ = now();
        stuckCheckX_    = vehicleX_;
        stuckCheckY_    = vehicleY_;
        stuckWarnPublished_ = false;
      }
    }

    // R10-2: 反向卡死检测 — 命令前进但实际后退时提前触发
    if (path_.poses.size() > 0 && safetyStop_ == 0 && stuckCheckInit_) {
      double cmdVx = coreState_.vehicleSpeed;
      double actualVx = odomIn->twist.twist.linear.x;
      // 命令前进 >0.1 但实际后退 <-0.05: 视为异常, 加速卡死判定
      if (cmdVx > 0.1 && actualVx < -0.05) {
        double elapsed = (now() - stuckCheckTime_).seconds();
        double remaining = stuckTimeout_ - elapsed;
        // 将剩余时间压缩到最多 3s, 提前触发
        if (remaining > 3.0) {
          stuckCheckTime_ = now() - rclcpp::Duration::from_seconds(stuckTimeout_ - 3.0);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "Reverse motion detected (cmd=%.2f, actual=%.2f), "
            "accelerating stuck detection", cmdVx, actualVx);
        }
      }
    }

    // R10-3: 卡死恢复确认 — 连续 3 帧速度 >0.05 才清除卡死状态
    if (stuckRecovering_) {
      double speed = std::sqrt(
        odomIn->twist.twist.linear.x * odomIn->twist.twist.linear.x +
        odomIn->twist.twist.linear.y * odomIn->twist.twist.linear.y);
      if (speed > kStuckRecoverSpeed) {
        stuckRecoverCount_++;
        if (stuckRecoverCount_ >= kStuckRecoverFrames) {
          stuckRecovering_ = false;
          stuckRecoverCount_ = 0;
          RCLCPP_INFO(get_logger(), "Stuck recovery confirmed (speed=%.2f)", speed);
        }
      } else {
        stuckRecoverCount_ = 0;  // 重置计数
      }
    }

    if ((fabs(roll) > inclThre_ * PI / 180.0 || fabs(pitch) > inclThre_ * PI / 180.0) && useInclToStop_) {
      stopInitTime_ = rclcpp::Time(odomIn->header.stamp).seconds();
    }

    if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre_ * PI / 180.0 || 
         fabs(odomIn->twist.twist.angular.y) > inclRateThre_ * PI / 180.0) && useInclRateToSlow_) {
      slowInitTime_ = rclcpp::Time(odomIn->header.stamp).seconds();
    }
  }

  void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
  {
    int pathSize = pathIn->poses.size();
    path_.poses.resize(pathSize);
    for (int i = 0; i < pathSize; i++) {
      path_.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
      path_.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
      path_.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }

    vehicleXRec_ = vehicleX_;
    vehicleYRec_ = vehicleY_;
    vehicleZRec_ = vehicleZ_;
    vehicleRollRec_ = vehicleRoll_;
    vehiclePitchRec_ = vehiclePitch_;
    vehicleYawRec_ = vehicleYaw_;
    cosVehicleYawRec_ = static_cast<float>(std::cos(vehicleYaw_));  // P5
    sinVehicleYawRec_ = static_cast<float>(std::sin(vehicleYaw_));

    pathPointID_ = 0;
    pathInit_ = true;
    goalReachedPublished_ = false;
    stuckCheckInit_ = false;  // 路径更新时重置 stuck 计时器
  }

  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    joyTime_ = now().seconds();
    joySpeedRaw_ = sqrt(joy->axes[joy_axis_left_] * joy->axes[joy_axis_left_] + joy->axes[joy_axis_fwd_] * joy->axes[joy_axis_fwd_]);
    joySpeed_ = joySpeedRaw_;
    if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    if (joy->axes[joy_axis_fwd_] == 0) joySpeed_ = 0;
    joyYaw_ = joy->axes[joy_axis_left_];
    if (joySpeed_ == 0 && noRotAtStop_) joyYaw_ = 0;

    if (joy->axes[joy_axis_fwd_] < 0 && !twoWayDrive_) {
      joySpeed_ = 0;
      joyYaw_ = 0;
    }

    joyManualFwd_ = joy->axes[joy_axis_fwd_];
    joyManualLeft_ = joy->axes[joy_axis_left_];
    joyManualYaw_ = joy->axes[joy_axis_yaw_];

    // joy_axis_autonomy_ (LT): 控制自主/手动模式
    // 松开 LT = 自主导航（默认）；按下 LT = 手动接管
    if (joy->axes[joy_axis_autonomy_] > -0.1) {
      autonomyMode_ = true;    // 松开 = 自主模式（默认）
      manualMode_ = false;     // 跟踪路径
    } else {
      autonomyMode_ = false;   // 按下 = 手动模式（接管）
      manualMode_ = true;      // 纯摇杆控制
    }
    // joy_axis_obstacle_ (RT) 保留作避障开关
  }

  void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
  {
    double speedTime = now().seconds();
    if (autonomyMode_ && speedTime - joyTime_ > joyToSpeedDelay_ && joySpeedRaw_ == 0) {
      joySpeed_ = speed->data / maxSpeed_;

      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    }
  }

  void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
  {
    // 多发布者优先级: 取当前值和新值的 max
    // 只有收到 0 时才允许降级 (清除需要所有发布者都发 0)
    int val = stop->data;
    if (val > safetyStop_) {
      safetyStop_ = val;  // 升级立即生效
    } else if (val == 0) {
      stopClearCount_++;
      // 连续收到 3 个 0 才清除 (防止单个发布者误清除)
      if (stopClearCount_ >= 3) {
        safetyStop_ = 0;
        stopClearCount_ = 0;
      }
    } else {
      stopClearCount_ = 0;
    }
  }

  void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
  {
    slowDown_ = slow->data;
  }

  // --- Control Loop (Thin Shell — 核心算法在 nav_core::computeControl) ---
  void controlLoop()
  {
    if (pathInit_) {
      int pathSize = path_.poses.size();
      if (pathSize == 0) return;

      // ── 1. 坐标变换: 机器人在路径参考帧中的位置 ──
      float dx = vehicleX_ - vehicleXRec_, dy = vehicleY_ - vehicleYRec_;
      float vehicleXRel =  cosVehicleYawRec_ * dx + sinVehicleYawRec_ * dy;
      float vehicleYRel = -sinVehicleYawRec_ * dx + cosVehicleYawRec_ * dy;

      // ── 2. ROS2 Path → nav_core Vec3 ──
      std::vector<nav_core::Vec3> pathPoints(pathSize);
      for (int i = 0; i < pathSize; i++) {
        pathPoints[i].x = path_.poses[i].pose.position.x;
        pathPoints[i].y = path_.poses[i].pose.position.y;
        pathPoints[i].z = path_.poses[i].pose.position.z;
      }

      // ── 3. 计算 slowFactor (ROS2 Shell 处理减速/停车时间逻辑) ──
      double slowFactor = 1.0;
      if ((odomTime_ < slowInitTime_ + slowTime1_ && slowInitTime_ > 0) || slowDown_ == 1)
        slowFactor = slowRate1_;
      else if ((odomTime_ < slowInitTime_ + slowTime1_ + slowTime2_ && slowInitTime_ > 0) || slowDown_ == 2)
        slowFactor = slowRate2_;
      else if (slowDown_ == 3)
        slowFactor = slowRate3_;

      // ── 4. 调用核心算法 ──
      nav_core::Vec3 vehicleRel{vehicleXRel, vehicleYRel, 0};
      double yawDiff = vehicleYaw_ - vehicleYawRec_;

      auto out = nav_core::computeControl(
        vehicleRel, yawDiff, pathPoints,
        joySpeed_, odomTime_, slowFactor, safetyStop_,
        coreParams_, coreState_);

      // ── 5. ROS2 Shell 后处理 ──
      // 倾斜停车
      if (odomTime_ < stopInitTime_ + stopTime_ && stopInitTime_ > 0) {
        coreState_.vehicleSpeed = 0;
        out.cmd.wz = 0;
      }

      // 非自主模式 + 零速: 手动 yaw 控制
      if (joySpeed_ == 0 && !autonomyMode_) {
        out.cmd.wz = maxYawRate_ * joyYaw_ * PI / 180.0;
      }

      // 同步 vehicleSpeed_ 供其他回调使用
      vehicleSpeed_ = coreState_.vehicleSpeed;
      vehicleYawRate_ = out.cmd.wz;
      navFwd_ = coreState_.navFwd;

      // ── 6. 发布 ──
      pubSkipCount_--;
      if (pubSkipCount_ < 0) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "body";
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        cmd_vel.twist.linear.x = out.cmd.vx;
        cmd_vel.twist.linear.y = out.cmd.vy;
        cmd_vel.twist.angular.z = out.cmd.wz;

        if (manualMode_) {
          cmd_vel.twist.linear.x = maxSpeed_ * joyManualFwd_;
          if (omniDirGoalThre_ > 0) cmd_vel.twist.linear.y = maxSpeed_ / 2.0 * joyManualLeft_;
          cmd_vel.twist.angular.z = maxYawRate_ * PI / 180.0 * joyManualYaw_;
          if (safetyStop_ >= 1) { cmd_vel.twist.linear.x = 0; cmd_vel.twist.linear.y = 0; }
          if (safetyStop_ >= 2) cmd_vel.twist.angular.z = 0;
        }

        pubSpeed_->publish(cmd_vel);
        pubSkipCount_ = pubSkipNum_;
      }

      // ── 7. 目标到达检测 ──
      if (!goalReachedPublished_ && pathSize > 0) {
        float goalDx = path_.poses.back().pose.position.x - vehicleX_;
        float goalDy = path_.poses.back().pose.position.y - vehicleY_;
        float goalDist = std::sqrt(goalDx * goalDx + goalDy * goalDy);
        if (goalDist < stopDisThre_) {
          std_msgs::msg::String status_msg;
          status_msg.data = "GOAL_REACHED";
          pubGoalStatus_->publish(status_msg);
          goalReachedPublished_ = true;
        }
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
