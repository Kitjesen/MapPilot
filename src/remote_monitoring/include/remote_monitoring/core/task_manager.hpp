#pragma once
/**
 * TaskManager — 航点序列任务管理
 *
 * 功能:
 *   - 接收多航点列表，按序下发到 local_planner
 *   - 任务状态机: PENDING → RUNNING → PAUSED → COMPLETED / FAILED / CANCELLED
 *   - 到达判定 (基于 /Odometry)
 *   - 循环巡检模式 (可选)
 *   - 进度回调 → EventBuffer → gRPC → App
 *
 * 接口:
 *   - 发布: /way_point (geometry_msgs/PointStamped, odom 坐标系)
 *   - 订阅: /Odometry (nav_msgs/Odometry)
 */

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "common.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 单个航点
struct Waypoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::string label;  // 可选标签 (如 "检查点A")
};

/// 任务参数
struct TaskParams {
  robot::v1::TaskType type{robot::v1::TASK_TYPE_NAVIGATION};
  std::vector<Waypoint> waypoints;
  bool loop{false};             // 循环巡检
  double arrival_radius{1.0};   // 到达判定半径 (m)
};

/// 任务执行状态 (内部)
enum class TaskState {
  IDLE,       // 无任务
  RUNNING,    // 执行中
  PAUSED,     // 暂停
};

/// 进度回调
using TaskProgressCallback =
    std::function<void(const std::string &task_id,
                       robot::v1::TaskStatus status,
                       float progress_percent,
                       const std::string &message)>;

class TaskManager {
public:
  explicit TaskManager(rclcpp::Node *node);

  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }

  /// 设置进度回调 (可选, 供 ControlService 轮询)
  void SetProgressCallback(TaskProgressCallback callback) {
    progress_callback_ = std::move(callback);
  }

  /// 启动任务。返回 task_id (空字符串表示失败)。
  std::string StartTask(const TaskParams &params);

  /// 暂停当前任务
  bool PauseTask(const std::string &task_id);

  /// 恢复暂停的任务
  bool ResumeTask(const std::string &task_id);

  /// 取消任务
  bool CancelTask(const std::string &task_id);

  /// 获取当前任务快照
  robot::v1::Task GetCurrentTask() const;

  /// 是否有正在执行的任务
  bool HasActiveTask() const { return state_.load() != TaskState::IDLE; }

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void CheckArrival();
  void AdvanceToNextWaypoint();
  void PublishCurrentWaypoint();
  void CompleteTask(robot::v1::TaskStatus status, const std::string &reason);
  std::string GenerateTaskId();

  rclcpp::Node *node_;

  // ROS 接口
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_waypoint_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // 当前任务
  mutable std::mutex task_mutex_;
  std::string task_id_;
  TaskParams task_params_;
  size_t current_waypoint_idx_{0};
  std::atomic<TaskState> state_{TaskState::IDLE};

  // 机器人位置
  std::mutex odom_mutex_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool odom_received_{false};

  // 事件
  std::shared_ptr<EventBuffer> event_buffer_;
  TaskProgressCallback progress_callback_;

  // 计数器
  uint64_t task_counter_{0};
};

}  // namespace core
}  // namespace remote_monitoring
