#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

class SafetyGate {
public:
  explicit SafetyGate(rclcpp::Node *node);

  // 设置事件缓冲区（用于自动生成安全事件）
  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }
  
  // 处理遥操作命令（返回限幅后的安全速度）
  robot::v1::Twist ProcessTeleopCommand(const robot::v1::TeleopCommand &cmd);
  
  // 获取当前安全状态
  robot::v1::SafetyStatus GetSafetyStatus();

  // 返回最近一次限幅原因，供 TeleopFeedback 透传。
  std::vector<std::string> GetLimitReasons();

  // 设置急停状态（true: 急停，false: 清除）。
  void SetEmergencyStop(bool active);
  
  // 检查 deadman 超时（仅在 TELEOP 模式下生效）
  void CheckDeadman();

  // 设置当前运行模式（由 ModeManager 调用）
  void SetCurrentMode(robot::v1::RobotMode mode);

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  robot::v1::Twist ApplyLimits(const robot::v1::Twist &target);
  
  rclcpp::Node *node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  
  std::mutex mutex_;
  std::chrono::system_clock::time_point last_teleop_time_;
  std::chrono::milliseconds deadman_timeout_ms_{300};
  
  bool estop_active_{false};
  double roll_deg_{0.0};
  double pitch_deg_{0.0};
  double max_speed_{1.0};
  double max_angular_{1.0};
  double tilt_limit_deg_{30.0};
  
  std::vector<std::string> limit_reasons_;

  // 当前运行模式（决定 deadman 是否生效）
  std::atomic<robot::v1::RobotMode> current_mode_{robot::v1::ROBOT_MODE_IDLE};

  // 事件生成
  std::shared_ptr<EventBuffer> event_buffer_;
  bool deadman_event_sent_{false};   // 避免重复发送
  bool tilt_event_sent_{false};
};

}  // namespace core
}  // namespace remote_monitoring
