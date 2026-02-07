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

// ================================================================
//  形式化转换: δ(from, to) → {success, reason}
// ================================================================

TransitionResult ModeManager::CheckTransition(
    robot::v1::RobotMode from, robot::v1::RobotMode to) const {

  // 规则 1: ESTOP 只能通过 ClearEmergencyStop() 退出
  if (from == robot::v1::ROBOT_MODE_ESTOP &&
      to != robot::v1::ROBOT_MODE_IDLE) {
    return {false, "ESTOP can only transition to IDLE via ClearEmergencyStop()"};
  }

  // 规则 2: 不允许跳到 ESTOP (必须走 EmergencyStop() 独立路径)
  if (to == robot::v1::ROBOT_MODE_ESTOP) {
    return {false, "Use EmergencyStop() instead of SwitchMode(ESTOP)"};
  }

  // 转换守卫矩阵
  switch (to) {
  case robot::v1::ROBOT_MODE_IDLE:
    // 任何非 ESTOP 状态都可以回 IDLE
    return {true, ""};

  case robot::v1::ROBOT_MODE_MANUAL:
    if (from != robot::v1::ROBOT_MODE_IDLE) {
      return {false, "MANUAL only reachable from IDLE"};
    }
    return {true, ""};

  case robot::v1::ROBOT_MODE_TELEOP:
    if (from != robot::v1::ROBOT_MODE_IDLE &&
        from != robot::v1::ROBOT_MODE_AUTONOMOUS) {
      return {false, "TELEOP only reachable from IDLE or AUTONOMOUS"};
    }
    if (guards_.has_lease && !guards_.has_lease()) {
      return {false, "TELEOP requires valid lease"};
    }
    return {true, ""};

  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    if (from != robot::v1::ROBOT_MODE_IDLE &&
        from != robot::v1::ROBOT_MODE_TELEOP) {
      return {false, "AUTONOMOUS only reachable from IDLE or TELEOP"};
    }
    if (guards_.tf_ok && !guards_.tf_ok()) {
      return {false, "AUTONOMOUS requires complete TF chain (map→odom→body)"};
    }
    if (guards_.localization_valid && !guards_.localization_valid()) {
      return {false, "AUTONOMOUS requires valid localization"};
    }
    return {true, ""};

  case robot::v1::ROBOT_MODE_MAPPING:
    if (from != robot::v1::ROBOT_MODE_IDLE) {
      return {false, "MAPPING only reachable from IDLE"};
    }
    return {true, ""};

  default:
    return {false, "Unknown target mode"};
  }
}

TransitionResult ModeManager::SwitchMode(robot::v1::RobotMode new_mode) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto old_mode = current_mode_.load();
  if (old_mode == new_mode) {
    return {true, "Already in requested mode"};
  }

  // 守卫检查
  auto result = CheckTransition(old_mode, new_mode);
  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "Mode transition %d → %d DENIED: %s",
                static_cast<int>(old_mode), static_cast<int>(new_mode),
                result.reason.c_str());
    return result;
  }

  RCLCPP_INFO(node_->get_logger(), "Mode transition: %d → %d",
              static_cast<int>(old_mode), static_cast<int>(new_mode));

  // 退出旧状态
  ExitState(old_mode);

  // 通知 SafetyGate
  if (safety_gate_) {
    safety_gate_->SetCurrentMode(new_mode);
  }

  // 进入新状态
  EnterState(new_mode);

  // 更新状态
  current_mode_.store(new_mode);

  // 事件记录
  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_MODE_CHANGE,
        robot::v1::EVENT_SEVERITY_INFO,
        "Mode changed to " + std::to_string(static_cast<int>(new_mode)),
        "Transition: " + std::to_string(static_cast<int>(old_mode)) +
            " → " + std::to_string(static_cast<int>(new_mode)));
  }

  return {true, ""};
}

// ================================================================
//  急停: 独立路径, lock-free, 最高优先级
// ================================================================

void ModeManager::EmergencyStop(const std::string &reason) {
  // 1. 先无条件停车 (lock-free, atomic)
  if (safety_gate_) {
    safety_gate_->SetEmergencyStop(true);
  }

  // 2. 发布停止到 pathFollower (独立通道)
  std_msgs::msg::Int8 stop_msg;
  stop_msg.data = 2;  // Level 2: 全停
  pub_stop_->publish(stop_msg);

  // 3. 更新模式 (atomic, 不需要锁)
  const auto old_mode = current_mode_.exchange(robot::v1::ROBOT_MODE_ESTOP);

  // 4. 事件通知 (尽力而为, 不影响停车)
  if (event_buffer_ && old_mode != robot::v1::ROBOT_MODE_ESTOP) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_SAFETY_ESTOP,
        robot::v1::EVENT_SEVERITY_CRITICAL,
        "Emergency stop: " + reason,
        "Automatic ESTOP triggered. Reason: " + reason +
            ". Previous mode: " + std::to_string(static_cast<int>(old_mode)));
  }

  RCLCPP_ERROR(node_->get_logger(), "EMERGENCY STOP: %s (was mode %d)",
               reason.c_str(), static_cast<int>(old_mode));
}

TransitionResult ModeManager::ClearEmergencyStop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_mode_.load() != robot::v1::ROBOT_MODE_ESTOP) {
    return {false, "Not in ESTOP mode"};
  }

  // 检查所有安全条件
  if (guards_.tilt_safe && !guards_.tilt_safe()) {
    return {false, "Cannot clear ESTOP: tilt angle exceeds limit"};
  }
  if (guards_.fence_safe && !guards_.fence_safe()) {
    return {false, "Cannot clear ESTOP: geofence violation active"};
  }

  // 清除急停
  if (safety_gate_) {
    safety_gate_->SetEmergencyStop(false);
  }

  // 回到 IDLE
  current_mode_.store(robot::v1::ROBOT_MODE_IDLE);

  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_MODE_CHANGE,
        robot::v1::EVENT_SEVERITY_INFO,
        "Emergency stop cleared",
        "ESTOP → IDLE. Operator manually cleared emergency stop.");
  }

  RCLCPP_INFO(node_->get_logger(), "Emergency stop cleared → IDLE");
  return {true, ""};
}

// ================================================================
//  状态进入/退出动作
// ================================================================

void ModeManager::ExitState(robot::v1::RobotMode state) {
  switch (state) {
  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    StopPathFollower();
    break;
  default:
    break;
  }
}

void ModeManager::EnterState(robot::v1::RobotMode state) {
  switch (state) {
  case robot::v1::ROBOT_MODE_IDLE:
    StopPathFollower();
    break;
  case robot::v1::ROBOT_MODE_TELEOP:
    StopPathFollower();
    break;
  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    StartPathFollower();
    break;
  case robot::v1::ROBOT_MODE_MAPPING:
    StopPathFollower();
    break;
  case robot::v1::ROBOT_MODE_MANUAL:
    StopPathFollower();
    break;
  default:
    break;
  }
}

// ================================================================
//  导航与路径控制
// ================================================================

void ModeManager::PublishGoalPose(double x, double y, double z, double qx,
                                  double qy, double qz, double qw) {
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
  RCLCPP_INFO(node_->get_logger(), "Published goal pose: (%.2f, %.2f, %.2f)",
              x, y, z);
}

void ModeManager::CancelNavigation() {
  StopPathFollower();

  if (event_buffer_) {
    event_buffer_->AddEvent(robot::v1::EVENT_TYPE_TASK_FAILED,
                            robot::v1::EVENT_SEVERITY_INFO,
                            "Navigation cancelled",
                            "Navigation task cancelled by operator");
  }

  RCLCPP_INFO(node_->get_logger(), "Navigation cancelled");
}

void ModeManager::StopPathFollower() {
  std_msgs::msg::Int8 stop_msg;
  stop_msg.data = 1;
  pub_stop_->publish(stop_msg);

  std_msgs::msg::Float32 speed_msg;
  speed_msg.data = 0.0f;
  pub_speed_->publish(speed_msg);
}

void ModeManager::StartPathFollower() {
  std_msgs::msg::Int8 stop_msg;
  stop_msg.data = 0;
  pub_stop_->publish(stop_msg);
}

}  // namespace core
}  // namespace remote_monitoring
