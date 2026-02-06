#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace remote_monitoring {
namespace core {

SafetyGate::SafetyGate(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<double>("deadman_timeout_ms", 300.0);
  node_->declare_parameter<double>("max_speed", 1.0);
  node_->declare_parameter<double>("max_angular", 1.0);
  node_->declare_parameter<double>("tilt_limit_deg", 30.0);
  node_->declare_parameter<std::string>("safety_odom_topic", "/Odometry");
  node_->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  node_->declare_parameter<std::string>("stop_topic", "/stop");
  
  deadman_timeout_ms_ = std::chrono::milliseconds(
    static_cast<int>(node_->get_parameter("deadman_timeout_ms").as_double()));
  max_speed_ = node_->get_parameter("max_speed").as_double();
  max_angular_ = node_->get_parameter("max_angular").as_double();
  tilt_limit_deg_ = node_->get_parameter("tilt_limit_deg").as_double();
  
  const auto odom_topic = node_->get_parameter("safety_odom_topic").as_string();
  const auto cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
  const auto stop_topic = node_->get_parameter("stop_topic").as_string();
  
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&SafetyGate::OdomCallback, this, std::placeholders::_1));
  
  pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic, 5);

  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>(stop_topic, 5);
}

robot::v1::Twist SafetyGate::ProcessTeleopCommand(const robot::v1::TeleopCommand &cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  last_teleop_time_ = std::chrono::system_clock::now();
  limit_reasons_.clear();
  
  // 应用限幅
  return ApplyLimits(cmd.target_velocity());
}

robot::v1::SafetyStatus SafetyGate::GetSafetyStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  robot::v1::SafetyStatus status;
  status.set_estop_active(estop_active_);
  
  // 检查 deadman（仅 TELEOP 模式下有意义）
  const auto mode = current_mode_.load();
  if (mode == robot::v1::ROBOT_MODE_TELEOP) {
    const auto now = std::chrono::system_clock::now();
    const bool deadman_active = (now - last_teleop_time_) > deadman_timeout_ms_;
    status.set_deadman_active(deadman_active);
  } else {
    status.set_deadman_active(false);
  }
  
  // 检查倾斜
  const bool tilt_limit = std::abs(roll_deg_) > tilt_limit_deg_ ||
                          std::abs(pitch_deg_) > tilt_limit_deg_;
  status.set_tilt_limit_active(tilt_limit);
  
  status.set_speed_limited(!limit_reasons_.empty());
  status.set_max_allowed_speed(max_speed_);
  
  std::string msg;
  for (const auto &reason : limit_reasons_) {
    msg += reason + "; ";
  }
  status.set_safety_message(msg);
  
  return status;
}

std::vector<std::string> SafetyGate::GetLimitReasons() {
  std::lock_guard<std::mutex> lock(mutex_);
  return limit_reasons_;
}

void SafetyGate::SetEmergencyStop(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool changed = (estop_active_ != active);
  estop_active_ = active;

  // 发布零速度并通知 pathFollower
  if (active) {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = node_->now();
    zero.header.frame_id = "body";
    pub_cmd_vel_->publish(zero);

    // 通知 pathFollower 停止
    std_msgs::msg::Int8 stop_msg;
    stop_msg.data = 1;
    pub_stop_->publish(stop_msg);
  } else {
    // 清除急停时，通知 pathFollower 恢复
    std_msgs::msg::Int8 stop_msg;
    stop_msg.data = 0;
    pub_stop_->publish(stop_msg);
  }

  if (changed && event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_SAFETY_ESTOP,
        active ? robot::v1::EVENT_SEVERITY_CRITICAL
               : robot::v1::EVENT_SEVERITY_INFO,
        active ? "Emergency stop activated" : "Emergency stop cleared",
        active ? "Robot halted by operator E-stop"
               : "E-stop cleared, robot may resume operations");
  }
}

void SafetyGate::CheckDeadman() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Deadman 仅在 TELEOP 模式下生效
  const auto mode = current_mode_.load();
  if (mode != robot::v1::ROBOT_MODE_TELEOP) {
    deadman_event_sent_ = false;
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const bool timed_out = (now - last_teleop_time_) > deadman_timeout_ms_;

  if (timed_out) {
    // 发布零速度
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = node_->now();
    zero.header.frame_id = "body";
    pub_cmd_vel_->publish(zero);

    // 仅首次超时时生成事件
    if (!deadman_event_sent_ && event_buffer_) {
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SAFETY_ESTOP,
          robot::v1::EVENT_SEVERITY_WARNING,
          "Deadman timeout",
          "No teleop command received within " +
              std::to_string(deadman_timeout_ms_.count()) + "ms, stopping robot");
      deadman_event_sent_ = true;
    }
  } else {
    // 恢复后重置标志
    deadman_event_sent_ = false;
  }
}

void SafetyGate::SetCurrentMode(robot::v1::RobotMode mode) {
  current_mode_.store(mode);
}

void SafetyGate::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  std::lock_guard<std::mutex> lock(mutex_);
  roll_deg_ = roll * 180.0 / M_PI;
  pitch_deg_ = pitch * 180.0 / M_PI;
}

robot::v1::Twist SafetyGate::ApplyLimits(const robot::v1::Twist &target) {
  robot::v1::Twist limited = target;
  
  // 速度限幅
  double speed = std::sqrt(target.linear().x() * target.linear().x() +
                           target.linear().y() * target.linear().y());
  if (speed > max_speed_) {
    const double scale = max_speed_ / speed;
    limited.mutable_linear()->set_x(target.linear().x() * scale);
    limited.mutable_linear()->set_y(target.linear().y() * scale);
    limit_reasons_.push_back("max_speed");
  }
  
  // 角速度限幅
  if (std::abs(target.angular().z()) > max_angular_) {
    limited.mutable_angular()->set_z(
      std::copysign(max_angular_, target.angular().z()));
    limit_reasons_.push_back("max_angular");
  }
  
  // 倾斜限制
  const bool tilt_active = std::abs(roll_deg_) > tilt_limit_deg_ ||
                           std::abs(pitch_deg_) > tilt_limit_deg_;
  if (tilt_active) {
    limited.mutable_linear()->set_x(0);
    limited.mutable_linear()->set_y(0);
    limited.mutable_angular()->set_z(0);
    limit_reasons_.push_back("tilt_protection");

    if (!tilt_event_sent_ && event_buffer_) {
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SAFETY_TILT_WARNING,
          robot::v1::EVENT_SEVERITY_WARNING,
          "Tilt protection active",
          "Roll=" + std::to_string(roll_deg_) +
              " Pitch=" + std::to_string(pitch_deg_) +
              " exceeds limit " + std::to_string(tilt_limit_deg_) + " deg");
      tilt_event_sent_ = true;
    }
  } else {
    tilt_event_sent_ = false;
  }
  
  // 急停
  if (estop_active_) {
    limited.mutable_linear()->set_x(0);
    limited.mutable_linear()->set_y(0);
    limited.mutable_linear()->set_z(0);
    limited.mutable_angular()->set_x(0);
    limited.mutable_angular()->set_y(0);
    limited.mutable_angular()->set_z(0);
    limit_reasons_.push_back("estop");
  }
  
  // 发布安全速度到 ROS
  geometry_msgs::msg::TwistStamped safe_cmd;
  safe_cmd.header.stamp = node_->now();
  safe_cmd.header.frame_id = "body";
  safe_cmd.twist.linear.x = limited.linear().x();
  safe_cmd.twist.linear.y = limited.linear().y();
  safe_cmd.twist.linear.z = limited.linear().z();
  safe_cmd.twist.angular.x = limited.angular().x();
  safe_cmd.twist.angular.y = limited.angular().y();
  safe_cmd.twist.angular.z = limited.angular().z();
  pub_cmd_vel_->publish(safe_cmd);
  
  return limited;
}

}  // namespace core
}  // namespace remote_monitoring
