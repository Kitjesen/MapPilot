#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class SafetyGate;
class EventBuffer;

/// ModeManager 协调模式切换与导航栈的交互。
///
/// 模式行为：
///   IDLE       - pathFollower 停止，SafetyGate 不接收 teleop
///   TELEOP     - pathFollower 停止，SafetyGate 接收 teleop 发送 /cmd_vel
///   AUTONOMOUS - pathFollower 运行，SafetyGate 不发 cmd_vel（由 pathFollower 控制）
///   MAPPING    - pathFollower 停止，SLAM 建图模式
///   MANUAL     - 手柄直接控制（本模块不干预）
///   ESTOP      - 一切停止
class ModeManager {
public:
  explicit ModeManager(rclcpp::Node *node);

  void SetSafetyGate(std::shared_ptr<SafetyGate> safety_gate) {
    safety_gate_ = std::move(safety_gate);
  }

  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }

  /// 切换模式，协调导航栈各组件。返回是否成功。
  bool SwitchMode(robot::v1::RobotMode new_mode);

  /// 发布导航目标点到全局规划器（/goal_pose）
  void PublishGoalPose(double x, double y, double z,
                       double qx = 0.0, double qy = 0.0,
                       double qz = 0.0, double qw = 1.0);

  /// 取消当前导航任务
  void CancelNavigation();

  robot::v1::RobotMode GetCurrentMode() const { return current_mode_.load(); }

private:
  void StopPathFollower();
  void StartPathFollower();

  rclcpp::Node *node_;
  std::shared_ptr<SafetyGate> safety_gate_;
  std::shared_ptr<EventBuffer> event_buffer_;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  std::atomic<robot::v1::RobotMode> current_mode_{robot::v1::ROBOT_MODE_IDLE};
  std::mutex mutex_;
};

}  // namespace core
}  // namespace remote_monitoring
