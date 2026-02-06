#include "remote_monitoring/core/mode_manager.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/safety_gate.hpp"

namespace remote_monitoring {
namespace core {

ModeManager::ModeManager(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<std::string>("mode_stop_topic", "/stop");
  node_->declare_parameter<std::string>("mode_speed_topic", "/speed");
  node_->declare_parameter<std::string>("goal_pose_topic", "/goal_pose");

  const auto stop_topic = node_->get_parameter("mode_stop_topic").as_string();
  const auto speed_topic = node_->get_parameter("mode_speed_topic").as_string();
  const auto goal_topic = node_->get_parameter("goal_pose_topic").as_string();

  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>(stop_topic, 5);
  pub_speed_ = node_->create_publisher<std_msgs::msg::Float32>(speed_topic, 5);
  pub_goal_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_topic, rclcpp::QoS(1).transient_local());
}

bool ModeManager::SwitchMode(robot::v1::RobotMode new_mode) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto old_mode = current_mode_.load();
  if (old_mode == new_mode) {
    return true;
  }

  RCLCPP_INFO(node_->get_logger(), "Mode transition: %d -> %d",
              static_cast<int>(old_mode), static_cast<int>(new_mode));

  // 通知 SafetyGate 当前模式
  if (safety_gate_) {
    safety_gate_->SetCurrentMode(new_mode);
  }

  switch (new_mode) {
  case robot::v1::ROBOT_MODE_IDLE:
    StopPathFollower();
    break;

  case robot::v1::ROBOT_MODE_TELEOP:
    // Teleop 模式：停止 pathFollower，由 SafetyGate 控制 /cmd_vel
    StopPathFollower();
    break;

  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    // 自主模式：启动 pathFollower，SafetyGate 不发 cmd_vel
    StartPathFollower();
    break;

  case robot::v1::ROBOT_MODE_MAPPING:
    // 建图模式：停止 pathFollower，SLAM 全速运行
    StopPathFollower();
    break;

  case robot::v1::ROBOT_MODE_MANUAL:
    // 手动模式：停止 pathFollower（手柄直接控制）
    StopPathFollower();
    break;

  case robot::v1::ROBOT_MODE_ESTOP:
    // 急停模式：停止一切
    StopPathFollower();
    if (safety_gate_) {
      safety_gate_->SetEmergencyStop(true);
    }
    break;

  default:
    RCLCPP_WARN(node_->get_logger(), "Unknown mode: %d",
                static_cast<int>(new_mode));
    return false;
  }

  current_mode_.store(new_mode);
  return true;
}

void ModeManager::PublishGoalPose(double x, double y, double z,
                                  double qx, double qy,
                                  double qz, double qw) {
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.x = qx;
  goal.pose.orientation.y = qy;
  goal.pose.orientation.z = qz;
  goal.pose.orientation.w = qw;

  pub_goal_pose_->publish(goal);
  RCLCPP_INFO(node_->get_logger(),
              "Published goal pose: (%.2f, %.2f, %.2f)", x, y, z);
}

void ModeManager::CancelNavigation() {
  StopPathFollower();

  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_TASK_FAILED,
        robot::v1::EVENT_SEVERITY_INFO,
        "Navigation cancelled",
        "Navigation task cancelled by operator");
  }

  RCLCPP_INFO(node_->get_logger(), "Navigation cancelled");
}

void ModeManager::StopPathFollower() {
  // 发布停止信号给 pathFollower
  std_msgs::msg::Int8 stop_msg;
  stop_msg.data = 1;
  pub_stop_->publish(stop_msg);

  // 设置速度为 0
  std_msgs::msg::Float32 speed_msg;
  speed_msg.data = 0.0f;
  pub_speed_->publish(speed_msg);
}

void ModeManager::StartPathFollower() {
  // 解除停止信号
  std_msgs::msg::Int8 stop_msg;
  stop_msg.data = 0;
  pub_stop_->publish(stop_msg);
}

}  // namespace core
}  // namespace remote_monitoring
