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


using namespace std;

const double PI = 3.1415926;

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

  nav_msgs::msg::Path path_;

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subStop_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subSlowDown_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeed_;
  rclcpp::TimerBase::SharedPtr timer_;

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
    vehicleX_ = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX_ + sin(yaw) * sensorOffsetY_;
    vehicleY_ = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX_ - cos(yaw) * sensorOffsetY_;
    vehicleZ_ = odomIn->pose.pose.position.z;

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

    pathPointID_ = 0;
    pathInit_ = true;
  }

  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    joyTime_ = now().seconds();
    joySpeedRaw_ = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed_ = joySpeedRaw_;
    if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    if (joy->axes[4] == 0) joySpeed_ = 0;
    joyYaw_ = joy->axes[3];
    if (joySpeed_ == 0 && noRotAtStop_) joyYaw_ = 0;

    if (joy->axes[4] < 0 && !twoWayDrive_) {
      joySpeed_ = 0;
      joyYaw_ = 0;
    }

    joyManualFwd_ = joy->axes[4];
    joyManualLeft_ = joy->axes[3];
    joyManualYaw_ = joy->axes[0];

    // axes[2] (LT): 控制自主/手动模式
    // 松开 LT = 自主导航（默认）；按下 LT = 手动接管
    if (joy->axes[2] > -0.1) {
      autonomyMode_ = true;    // 松开 = 自主模式（默认）
      manualMode_ = false;     // 跟踪路径
    } else {
      autonomyMode_ = false;   // 按下 = 手动模式（接管）
      manualMode_ = true;      // 纯摇杆控制
    }
    // axes[5] (RT) 保留作避障开关
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

  // --- Control Loop ---
  void controlLoop()
  {
    if (pathInit_) {
      float vehicleXRel = cos(vehicleYawRec_) * (vehicleX_ - vehicleXRec_) 
                        + sin(vehicleYawRec_) * (vehicleY_ - vehicleYRec_);
      float vehicleYRel = -sin(vehicleYawRec_) * (vehicleX_ - vehicleXRec_) 
                        + cos(vehicleYawRec_) * (vehicleY_ - vehicleYRec_);

      int pathSize = path_.poses.size();
      if (pathSize == 0) return;  // 空路径，直接返回
      
      float endDisX = path_.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path_.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      // ===== Adaptive Lookahead Distance =====
      float currentSpeed = fabs(vehicleSpeed_);
      float adaptiveLookAhead = baseLookAheadDis_ + lookAheadRatio_ * currentSpeed;
      
      if (adaptiveLookAhead < minLookAheadDis_) adaptiveLookAhead = minLookAheadDis_;
      else if (adaptiveLookAhead > maxLookAheadDis_) adaptiveLookAhead = maxLookAheadDis_;
      
      lookAheadDis_ = adaptiveLookAhead;

      // ===== Optimized Path Point Search =====
      if (pathSize != lastPathSize_) {
        lastPathPointID_ = 0;
        lastPathSize_ = pathSize;
      }
      
      pathPointID_ = (lastPathPointID_ > 2) ? (lastPathPointID_ - 2) : 0;
      
      float disX, disY, dis;
      while (pathPointID_ < pathSize - 1) {
        disX = path_.poses[pathPointID_].pose.position.x - vehicleXRel;
        disY = path_.poses[pathPointID_].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis_) {
          pathPointID_++;
        } else {
          break;
        }
      }
      
      lastPathPointID_ = pathPointID_;

      disX = path_.poses[pathPointID_].pose.position.x - vehicleXRel;
      disY = path_.poses[pathPointID_].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw_ - vehicleYawRec_ - pathDir;
      // 一次归一化到 (-π, π] 即可 (两个 [-π,π] 量之差在 [-2π, 2π] 内)
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive_) {
        double time = now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd_ && time - switchTime_ > switchTimeThre_) {
          navFwd_ = false;
          switchTime_ = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd_ && time - switchTime_ > switchTimeThre_) {
          navFwd_ = true;
          switchTime_ = time;
        }
      }

      float joySpeed2 = maxSpeed_ * joySpeed_;
      if (!navFwd_) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed_) < 2.0 * maxAccel_ / 100.0) vehicleYawRate_ = -stopYawRateGain_ * dirDiff;
      else vehicleYawRate_ = -yawRateGain_ * dirDiff;

      if (vehicleYawRate_ > maxYawRate_ * PI / 180.0) vehicleYawRate_ = maxYawRate_ * PI / 180.0;
      else if (vehicleYawRate_ < -maxYawRate_ * PI / 180.0) vehicleYawRate_ = -maxYawRate_ * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode_) {
        vehicleYawRate_ = maxYawRate_ * joyYaw_ * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre_ && noRotAtGoal_)) {
        vehicleYawRate_ = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre_ < joySpeed_) {
        joySpeed2 *= endDis / slowDwnDisThre_;
      }

      float joySpeed3 = joySpeed2;
      if ((odomTime_ < slowInitTime_ + slowTime1_ && slowInitTime_ > 0) || slowDown_ == 1) joySpeed3 *= slowRate1_;
      else if ((odomTime_ < slowInitTime_ + slowTime1_ + slowTime2_ && slowInitTime_ > 0) || slowDown_ == 2) joySpeed3 *= slowRate2_;
      else if (slowDown_ == 3) joySpeed3 *= slowRate3_;

      if ((fabs(dirDiff) < dirDiffThre_ || (dis < omniDirGoalThre_ && fabs(dirDiff) < omniDirDiffThre_)) && dis > stopDisThre_) {
        if (vehicleSpeed_ < joySpeed3) vehicleSpeed_ += maxAccel_ / 100.0;
        else if (vehicleSpeed_ > joySpeed3) vehicleSpeed_ -= maxAccel_ / 100.0;
      } else {
        if (vehicleSpeed_ > 0) vehicleSpeed_ -= maxAccel_ / 100.0;
        else if (vehicleSpeed_ < 0) vehicleSpeed_ += maxAccel_ / 100.0;
      }

      if (odomTime_ < stopInitTime_ + stopTime_ && stopInitTime_ > 0) {
        vehicleSpeed_ = 0;
        vehicleYawRate_ = 0;
      }

      if (safetyStop_ >= 1) vehicleSpeed_ = 0;
      if (safetyStop_ >= 2) vehicleYawRate_ = 0;

      pubSkipCount_--;
      if (pubSkipCount_ < 0) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "body";  // Use body frame (consistent with Fast-LIO2 TF)
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.angular.z = vehicleYawRate_;

        if (fabs(vehicleSpeed_) > maxAccel_ / 100.0) {
          if (omniDirGoalThre_ > 0) {
            cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed_;
            cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed_;
          } else {
            cmd_vel.twist.linear.x = vehicleSpeed_;
          }
        }

        if (manualMode_) {
          cmd_vel.twist.linear.x = maxSpeed_ * joyManualFwd_;
          if (omniDirGoalThre_ > 0) cmd_vel.twist.linear.y = maxSpeed_ / 2.0 * joyManualLeft_;
          cmd_vel.twist.angular.z = maxYawRate_ * PI / 180.0 * joyManualYaw_;
        }

        pubSpeed_->publish(cmd_vel);
        pubSkipCount_ = pubSkipNum_;

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
