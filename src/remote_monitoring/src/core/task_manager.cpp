#include "remote_monitoring/core/task_manager.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace remote_monitoring {
namespace core {

TaskManager::TaskManager(rclcpp::Node *node) : node_(node) {
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&TaskManager::OdomCallback, this, std::placeholders::_1));

  pub_waypoint_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      "/way_point", rclcpp::QoS(1).transient_local());

  // 到达检测定时器: 5 Hz
  check_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&TaskManager::CheckArrival, this));
}

// ================================================================
//  公共接口
// ================================================================

std::string TaskManager::StartTask(const TaskParams &params) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (state_.load() != TaskState::IDLE) {
    RCLCPP_WARN(node_->get_logger(),
                "TaskManager: Cannot start task — another task is active");
    return "";
  }

  if (params.waypoints.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "TaskManager: Cannot start task — no waypoints");
    return "";
  }

  task_id_ = GenerateTaskId();
  task_params_ = params;
  current_waypoint_idx_ = 0;
  state_.store(TaskState::RUNNING);

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Starting task %s with %zu waypoints (loop=%s)",
              task_id_.c_str(), params.waypoints.size(),
              params.loop ? "true" : "false");

  // 事件通知
  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_TASK_STARTED,
        robot::v1::EVENT_SEVERITY_INFO,
        "Task started: " + task_id_,
        "Waypoints: " + std::to_string(params.waypoints.size()) +
            ", Type: " + std::to_string(static_cast<int>(params.type)));
  }

  // 下发第一个航点
  PublishCurrentWaypoint();

  // 进度回调
  if (progress_callback_) {
    progress_callback_(task_id_, robot::v1::TASK_STATUS_RUNNING, 0.0f,
                       "Started");
  }

  return task_id_;
}

bool TaskManager::PauseTask(const std::string &task_id) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (task_id != task_id_ || state_.load() != TaskState::RUNNING) {
    return false;
  }

  state_.store(TaskState::PAUSED);
  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s paused at waypoint %zu/%zu",
              task_id_.c_str(), current_waypoint_idx_ + 1,
              task_params_.waypoints.size());

  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_TASK_STARTED,
        robot::v1::EVENT_SEVERITY_INFO, "Task paused: " + task_id_,
        "Paused at waypoint " + std::to_string(current_waypoint_idx_ + 1));
  }

  if (progress_callback_) {
    const float pct = task_params_.waypoints.empty()
                          ? 0.0f
                          : 100.0f * static_cast<float>(current_waypoint_idx_) /
                                static_cast<float>(task_params_.waypoints.size());
    progress_callback_(task_id_, robot::v1::TASK_STATUS_PAUSED, pct, "Paused");
  }

  return true;
}

bool TaskManager::ResumeTask(const std::string &task_id) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (task_id != task_id_ || state_.load() != TaskState::PAUSED) {
    return false;
  }

  state_.store(TaskState::RUNNING);
  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s resumed", task_id_.c_str());

  // 重新发布当前航点
  PublishCurrentWaypoint();

  if (progress_callback_) {
    const float pct = task_params_.waypoints.empty()
                          ? 0.0f
                          : 100.0f * static_cast<float>(current_waypoint_idx_) /
                                static_cast<float>(task_params_.waypoints.size());
    progress_callback_(task_id_, robot::v1::TASK_STATUS_RUNNING, pct, "Resumed");
  }

  return true;
}

bool TaskManager::CancelTask(const std::string &task_id) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (task_id != task_id_ || state_.load() == TaskState::IDLE) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s cancelled",
              task_id_.c_str());
  CompleteTask(robot::v1::TASK_STATUS_CANCELLED, "Cancelled by operator");
  return true;
}

robot::v1::Task TaskManager::GetCurrentTask() const {
  std::lock_guard<std::mutex> lock(task_mutex_);

  robot::v1::Task task;
  task.set_task_id(task_id_);
  task.set_type(task_params_.type);

  switch (state_.load()) {
  case TaskState::IDLE:
    task.set_status(task_id_.empty() ? robot::v1::TASK_STATUS_UNSPECIFIED
                                     : robot::v1::TASK_STATUS_COMPLETED);
    break;
  case TaskState::RUNNING:
    task.set_status(robot::v1::TASK_STATUS_RUNNING);
    break;
  case TaskState::PAUSED:
    task.set_status(robot::v1::TASK_STATUS_PAUSED);
    break;
  }

  if (!task_params_.waypoints.empty()) {
    task.set_progress_percent(
        100.0f * static_cast<float>(current_waypoint_idx_) /
        static_cast<float>(task_params_.waypoints.size()));
  }

  return task;
}

// ================================================================
//  内部逻辑
// ================================================================

void TaskManager::OdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void TaskManager::CheckArrival() {
  if (state_.load() != TaskState::RUNNING) {
    return;
  }

  double rx, ry;
  bool have_odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    rx = robot_x_;
    ry = robot_y_;
    have_odom = odom_received_;
  }

  if (!have_odom) {
    return;
  }

  std::lock_guard<std::mutex> lock(task_mutex_);

  if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
    return;
  }

  const auto &wp = task_params_.waypoints[current_waypoint_idx_];
  const double dx = rx - wp.x;
  const double dy = ry - wp.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist <= task_params_.arrival_radius) {
    RCLCPP_INFO(node_->get_logger(),
                "TaskManager: Reached waypoint %zu/%zu (dist=%.2fm)",
                current_waypoint_idx_ + 1, task_params_.waypoints.size(),
                dist);
    AdvanceToNextWaypoint();
  }
}

void TaskManager::AdvanceToNextWaypoint() {
  // 已持有 task_mutex_
  current_waypoint_idx_++;

  if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
    if (task_params_.loop) {
      // 循环巡检: 重新开始
      current_waypoint_idx_ = 0;
      RCLCPP_INFO(node_->get_logger(),
                  "TaskManager: Loop patrol — restarting from waypoint 1");

      if (event_buffer_) {
        event_buffer_->AddEvent(
            robot::v1::EVENT_TYPE_NAV_GOAL_REACHED,
            robot::v1::EVENT_SEVERITY_INFO,
            "Patrol loop restarting",
            "Completed one cycle of " +
                std::to_string(task_params_.waypoints.size()) + " waypoints");
      }
    } else {
      // 所有航点已完成
      CompleteTask(robot::v1::TASK_STATUS_COMPLETED,
                   "All waypoints reached");
      return;
    }
  }

  PublishCurrentWaypoint();

  // 进度回调
  if (progress_callback_) {
    const float pct =
        100.0f * static_cast<float>(current_waypoint_idx_) /
        static_cast<float>(task_params_.waypoints.size());
    progress_callback_(
        task_id_, robot::v1::TASK_STATUS_RUNNING, pct,
        "Navigating to waypoint " +
            std::to_string(current_waypoint_idx_ + 1) + "/" +
            std::to_string(task_params_.waypoints.size()));
  }
}

void TaskManager::PublishCurrentWaypoint() {
  // 已持有 task_mutex_
  if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
    return;
  }

  const auto &wp = task_params_.waypoints[current_waypoint_idx_];

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.point.x = wp.x;
  msg.point.y = wp.y;
  msg.point.z = wp.z;

  pub_waypoint_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Published waypoint %zu/%zu (%.2f, %.2f, %.2f)%s",
              current_waypoint_idx_ + 1, task_params_.waypoints.size(),
              wp.x, wp.y, wp.z,
              wp.label.empty() ? "" : (" [" + wp.label + "]").c_str());
}

void TaskManager::CompleteTask(robot::v1::TaskStatus status,
                               const std::string &reason) {
  // 已持有 task_mutex_
  state_.store(TaskState::IDLE);

  const auto event_type =
      (status == robot::v1::TASK_STATUS_COMPLETED)
          ? robot::v1::EVENT_TYPE_TASK_COMPLETED
          : robot::v1::EVENT_TYPE_TASK_FAILED;
  const auto severity =
      (status == robot::v1::TASK_STATUS_COMPLETED)
          ? robot::v1::EVENT_SEVERITY_INFO
          : robot::v1::EVENT_SEVERITY_WARNING;

  if (event_buffer_) {
    event_buffer_->AddEvent(event_type, severity,
                            "Task " + task_id_ + ": " + reason,
                            "Status: " + std::to_string(static_cast<int>(status)));
  }

  if (progress_callback_) {
    const float pct =
        (status == robot::v1::TASK_STATUS_COMPLETED) ? 100.0f
        : task_params_.waypoints.empty()
            ? 0.0f
            : 100.0f * static_cast<float>(current_waypoint_idx_) /
                  static_cast<float>(task_params_.waypoints.size());
    progress_callback_(task_id_, status, pct, reason);
  }

  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s finished — %s",
              task_id_.c_str(), reason.c_str());
}

std::string TaskManager::GenerateTaskId() {
  task_counter_++;
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  std::ostringstream oss;
  oss << "task_" << std::hex << (now_ms & 0xFFFFFFFF) << "_"
      << std::setfill('0') << std::setw(4) << task_counter_;
  return oss.str();
}

}  // namespace core
}  // namespace remote_monitoring
