#pragma once
/**
 * TaskManager — 航点唯一发布者 + 任务管理
 *
 * 设计:
 *   TaskManager 是 /way_point 的唯一发布者, 解决了 App 和全局规划器
 *   同时发布航点导致 local_planner 来回抖动的冲突问题.
 *
 * 航点来源 (互斥):
 *   1. APP     — 通过 StartTask RPC 下发的航点列表 (map 坐标系, 内部转 odom)
 *   2. PLANNER — 全局规划器 (pct_path_adapter) 发布到 /planner_waypoint
 *
 * 优先级: APP > PLANNER
 *   - App 任务活跃时, 忽略全局规划器的航点
 *   - App 任务结束后, 自动恢复转发全局规划器航点
 *   - ClearWaypoints 可以清除任何来源的航点并停止机器人
 *
 * 安全联动:
 *   - ModeManager 通过 OnModeChanged() 通知模式切换
 *   - 非 AUTONOMOUS 模式时自动暂停任务, 回 AUTONOMOUS 时自动恢复
 *   - E-stop 时暂停, 不允许航点推进
 *
 * 坐标系:
 *   - 航点存储: map 坐标系 (App/用户视角)
 *   - 发布到 /way_point: odom 坐标系 (local_planner 期望)
 *   - 通过 tf2 (map→odom) 实时变换
 *
 * 接口:
 *   - 发布: /way_point (PointStamped, odom), /stop (Int8)
 *   - 订阅: /Odometry, /planner_waypoint, /pct_path
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "common.pb.h"
#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 单个航点 (map 坐标系)
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
  double max_speed{0.0};        // 任务期望最大速度 (m/s), 0 = 使用系统默认

  // ── 建图任务参数 ──
  std::string map_name;         // 地图保存名称
  bool save_on_complete{true};  // 完成后自动保存
  double resolution{0.1};       // 地图分辨率 (m)

  // ── 循迹任务参数 ──
  double tracking_tolerance{0.5}; // 循迹容差 (m)
};

/// 任务执行状态 (内部)
enum class TaskState {
  IDLE,       // 无任务
  RUNNING,    // 执行中
  PAUSED,     // 暂停 (用户手动暂停)
};

/// 航点来源
enum class WaypointSourceInternal {
  NONE,       // 无活跃航点
  APP,        // 来自 App (StartTask)
  PLANNER,    // 来自全局规划器 (pct_path_adapter)
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

  void SetProgressCallback(TaskProgressCallback callback) {
    progress_callback_ = std::move(callback);
  }

  // ---- App 任务控制 ----

  /// 启动任务。返回 task_id (空字符串表示失败)。
  std::string StartTask(const TaskParams &params);

  /// 暂停当前任务 (用户手动)
  bool PauseTask(const std::string &task_id);

  /// 恢复暂停的任务
  bool ResumeTask(const std::string &task_id);

  /// 取消任务
  bool CancelTask(const std::string &task_id);

  /// 停止建图任务 (以 COMPLETED 状态结束, 而非 CANCELLED)。
  /// 返回 {成功, map_name, save_on_complete}。
  struct MappingResult {
    bool ok{false};
    std::string map_name;
    bool save_on_complete{false};
  };
  MappingResult StopMapping(const std::string &task_id);

  /// 获取当前任务快照
  robot::v1::Task GetCurrentTask() const;

  /// 是否有正在执行的 App 任务 (包括暂停和系统挂起)
  bool HasActiveTask() const { return state_.load() != TaskState::IDLE; }

  // ---- 航点管理 (供 ControlService 调用) ----

  /// 获取当前活跃航点列表
  robot::v1::GetActiveWaypointsResponse GetActiveWaypoints() const;

  /// 清除所有航点 (App 任务 + 规划器), 发布 /stop 立即停车
  robot::v1::ClearWaypointsResponse ClearWaypoints();

  /// 获取当前航点来源
  WaypointSourceInternal GetCurrentSource() const;

  // ---- 模式联动 (由 ModeManager 调用) ----

  /// 模式切换时由 ModeManager 调用
  /// 非 AUTONOMOUS 模式 → 自动挂起任务; AUTONOMOUS → 自动恢复
  void OnModeChanged(int new_mode);

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void PlannerWaypointCallback(
      const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void PctPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void CheckArrival();
  void AdvanceToNextWaypoint();
  void PublishCurrentWaypoint();
  void PublishStopWaypoint();
  void PublishStop(int level);       // 发布 /stop (0=clear, 1=lin, 2=full)
  void CompleteTask(robot::v1::TaskStatus status, const std::string &reason);
  std::string GenerateTaskId();

  /// map→odom 坐标变换 (返回 false 表示 tf 不可用)
  bool TransformToOdom(double map_x, double map_y, double map_z,
                       double &odom_x, double &odom_y, double &odom_z);

  rclcpp::Node *node_;

  // ROS 接口
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      sub_planner_waypoint_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_pct_path_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_waypoint_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string odom_frame_{"odom"};  // 从 /Odometry 动态获取

  // App 任务
  mutable std::mutex task_mutex_;
  std::string task_id_;
  TaskParams task_params_;
  size_t current_waypoint_idx_{0};
  std::atomic<TaskState> state_{TaskState::IDLE};

  // 系统挂起 (由 ModeManager 触发, 区别于用户 PauseTask)
  std::atomic<bool> system_suspended_{false};

  // 航点超时
  double waypoint_timeout_sec_{300.0};  // 单个航点超时 (秒), 0=禁用
  std::chrono::steady_clock::time_point waypoint_start_time_;

  // 规划器转发
  mutable std::mutex planner_mutex_;
  bool planner_active_{false};
  bool planner_suppressed_{false};
  geometry_msgs::msg::PointStamped last_planner_waypoint_;

  // 机器人位置 (odom 坐标系)
  std::mutex odom_mutex_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool odom_received_{false};

  // TF 缓存: 用上一次成功的 map→odom 变换做粗略距离预判,
  // 距离远时跳过实时 TF 查询 (节省 ~95% 的 lookupTransform 调用)
  geometry_msgs::msg::TransformStamped cached_tf_;
  bool cached_tf_valid_{false};
  std::chrono::steady_clock::time_point cached_tf_time_;

  // 事件
  std::shared_ptr<EventBuffer> event_buffer_;
  TaskProgressCallback progress_callback_;

  // 计数器
  uint64_t task_counter_{0};
};

}  // namespace core
}  // namespace remote_monitoring
