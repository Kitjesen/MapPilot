#include "remote_monitoring/core/task_manager.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <cmath>
#include <cstdio>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace remote_monitoring {
namespace core {

TaskManager::TaskManager(rclcpp::Node *node) : node_(node) {
  // ---- 参数 ----
  node_->declare_parameter<double>("waypoint_timeout_sec", 300.0);
  waypoint_timeout_sec_ =
      node_->get_parameter("waypoint_timeout_sec").as_double();

  // ---- 话题参数 (标准接口契约, 通过 YAML 可覆盖) ----
  if (!node_->has_parameter("task_odom_topic"))
    node_->declare_parameter<std::string>("task_odom_topic", "/nav/odometry");
  if (!node_->has_parameter("task_waypoint_in_topic"))
    node_->declare_parameter<std::string>("task_waypoint_in_topic", "/nav/waypoint");
  if (!node_->has_parameter("task_global_path_topic"))
    node_->declare_parameter<std::string>("task_global_path_topic", "/nav/global_path");
  if (!node_->has_parameter("task_waypoint_out_topic"))
    node_->declare_parameter<std::string>("task_waypoint_out_topic", "/nav/way_point");
  if (!node_->has_parameter("task_stop_topic"))
    node_->declare_parameter<std::string>("task_stop_topic", "/nav/stop");

  const auto odom_topic = node_->get_parameter("task_odom_topic").as_string();
  const auto waypoint_in = node_->get_parameter("task_waypoint_in_topic").as_string();
  const auto global_path = node_->get_parameter("task_global_path_topic").as_string();
  const auto waypoint_out = node_->get_parameter("task_waypoint_out_topic").as_string();
  const auto stop_topic = node_->get_parameter("task_stop_topic").as_string();

  // ---- TF2 ----
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- 订阅 ----
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&TaskManager::OdomCallback, this, std::placeholders::_1));

  sub_planner_waypoint_ =
      node_->create_subscription<geometry_msgs::msg::PointStamped>(
          waypoint_in, 10,
          std::bind(&TaskManager::PlannerWaypointCallback, this,
                    std::placeholders::_1));

  // 订阅全局路径 — 新路径到达时解除 planner 抑制
  sub_pct_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      global_path, 10,
      std::bind(&TaskManager::PctPathCallback, this, std::placeholders::_1));

  // ---- 发布 ----
  pub_waypoint_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      waypoint_out, 10);

  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>(stop_topic, 5);

  // ---- 定时器 ----
  check_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&TaskManager::CheckArrival, this));

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: odom=%s, waypoint_in=%s, global_path=%s, "
              "waypoint_out=%s, stop=%s, timeout=%.0fs",
              odom_topic.c_str(), waypoint_in.c_str(), global_path.c_str(),
              waypoint_out.c_str(), stop_topic.c_str(), waypoint_timeout_sec_);
}

// ================================================================
//  坐标变换: map → odom
// ================================================================

bool TaskManager::TransformToOdom(double map_x, double map_y, double map_z,
                                  double &odom_x, double &odom_y,
                                  double &odom_z) {
  geometry_msgs::msg::PointStamped in;
  in.header.frame_id = "map";
  in.header.stamp = rclcpp::Time(0);  // 最新可用 tf
  in.point.x = map_x;
  in.point.y = map_y;
  in.point.z = map_z;

  geometry_msgs::msg::PointStamped out;
  try {
    const auto tf = tf_buffer_->lookupTransform(
        odom_frame_, "map", tf2::TimePointZero);
    tf2::doTransform(in, out, tf);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "TaskManager: map→%s tf failed: %s",
                         odom_frame_.c_str(), ex.what());
    return false;
  }

  odom_x = out.point.x;
  odom_y = out.point.y;
  odom_z = out.point.z;
  return true;
}

// ================================================================
//  规划器航点转发 + 路径抑制恢复
// ================================================================

void TaskManager::PlannerWaypointCallback(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  // App 任务优先: 有 App 任务时忽略规划器航点
  if (state_.load() != TaskState::IDLE) {
    return;
  }
  // 系统挂起 (非 AUTONOMOUS 模式) 时也不转发
  if (system_suspended_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (planner_suppressed_) {
      return;  // 抑制中, 等待新 /pct_path 到来才恢复
    }
    planner_active_ = true;
    last_planner_waypoint_ = *msg;
  }

  // pct_path_adapter 已经把航点转到 odom 坐标系, 直接转发
  pub_waypoint_->publish(*msg);
}

void TaskManager::PctPathCallback(
    const nav_msgs::msg::Path::ConstSharedPtr msg) {
  if (msg->poses.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (planner_suppressed_) {
    planner_suppressed_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "TaskManager: New /pct_path received (%zu poses), "
                "planner suppression lifted",
                msg->poses.size());
  }
}

// ================================================================
//  模式联动
// ================================================================

void TaskManager::OnModeChanged(int new_mode) {
  const bool is_autonomous =
      (new_mode == static_cast<int>(robot::v1::ROBOT_MODE_AUTONOMOUS));

  if (is_autonomous) {
    // 恢复: 仅恢复系统挂起的任务, 不恢复用户手动暂停的
    if (system_suspended_.exchange(false)) {
      std::lock_guard<std::mutex> lock(task_mutex_);
      if (state_.load() == TaskState::PAUSED) {
        state_.store(TaskState::RUNNING);
        // 重置航点超时计时
        waypoint_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(),
                    "TaskManager: Mode → AUTONOMOUS, task %s resumed",
                    task_id_.c_str());
        PublishCurrentWaypoint();
      }
    }
  } else {
    // 挂起: 非 AUTONOMOUS 模式时暂停任务
    if (state_.load() == TaskState::RUNNING) {
      std::lock_guard<std::mutex> lock(task_mutex_);

      // 二次校验: 防止在获取锁期间状态被其他线程修改 (CancelTask 等)
      if (state_.load() != TaskState::RUNNING) {
        return;
      }

      // 建图任务特殊处理: E-stop / 模式切换时直接完成 (非暂停),
      // 触发 App 侧 onMappingComplete → 自动保存地图。
      // 理由: 建图数据是易失的，暂停后如果操作员断电或重启，地图数据全部丢失。
      if (task_params_.type == robot::v1::TASK_TYPE_MAPPING) {
        RCLCPP_WARN(node_->get_logger(),
                    "TaskManager: Mode → %d during mapping task %s — "
                    "completing to preserve map data (save_on_complete=%s)",
                    new_mode, task_id_.c_str(),
                    task_params_.save_on_complete ? "true" : "false");
        CompleteTask(robot::v1::TASK_STATUS_COMPLETED,
                     "Mapping stopped by mode change (data preserved)");
      } else {
        system_suspended_.store(true);
        state_.store(TaskState::PAUSED);
        RCLCPP_INFO(node_->get_logger(),
                    "TaskManager: Mode → %d (non-AUTONOMOUS), task %s suspended",
                    new_mode, task_id_.c_str());
      }
    }
  }
}

// ================================================================
//  公共接口 — App 任务控制
// ================================================================

std::string TaskManager::StartTask(const TaskParams &params) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  if (state_.load() != TaskState::IDLE) {
    RCLCPP_WARN(node_->get_logger(),
                "TaskManager: Cannot start task — another task is active");
    return "";
  }

  // 建图任务不需要航点
  if (params.waypoints.empty() &&
      params.type != robot::v1::TASK_TYPE_MAPPING) {
    RCLCPP_WARN(node_->get_logger(),
                "TaskManager: Cannot start task — no waypoints");
    return "";
  }

  task_id_ = GenerateTaskId();
  task_params_ = params;
  current_waypoint_idx_ = 0;
  state_.store(TaskState::RUNNING);
  system_suspended_.store(false);
  waypoint_start_time_ = std::chrono::steady_clock::now();

  // App 任务启动时, 解除 planner 抑制 (任务结束后会恢复转发)
  {
    std::lock_guard<std::mutex> plock(planner_mutex_);
    planner_suppressed_ = false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Starting task %s type=%d with %zu waypoints "
              "(loop=%s, max_speed=%.2f, map_name='%s')",
              task_id_.c_str(), static_cast<int>(params.type),
              params.waypoints.size(),
              params.loop ? "true" : "false",
              params.max_speed,
              params.map_name.c_str());

  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_TASK_STARTED,
        robot::v1::EVENT_SEVERITY_INFO,
        "Task started: " + task_id_,
        "Waypoints: " + std::to_string(params.waypoints.size()) +
            ", Type: " + std::to_string(static_cast<int>(params.type)));
  }

  // 建图任务无航点，不发布初始 waypoint
  if (!params.waypoints.empty()) {
    PublishCurrentWaypoint();
  }

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
  system_suspended_.store(false);  // 用户手动暂停, 清除系统挂起标记

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Task %s paused at waypoint %zu/%zu",
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
  system_suspended_.store(false);
  waypoint_start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s resumed",
              task_id_.c_str());

  PublishCurrentWaypoint();

  if (progress_callback_) {
    const float pct = task_params_.waypoints.empty()
                          ? 0.0f
                          : 100.0f * static_cast<float>(current_waypoint_idx_) /
                                static_cast<float>(task_params_.waypoints.size());
    progress_callback_(task_id_, robot::v1::TASK_STATUS_RUNNING, pct,
                       "Resumed");
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

TaskManager::MappingResult TaskManager::StopMapping(const std::string &task_id) {
  std::lock_guard<std::mutex> lock(task_mutex_);

  MappingResult result;

  if (task_id != task_id_ || state_.load() == TaskState::IDLE) {
    return result;
  }

  if (task_params_.type != robot::v1::TASK_TYPE_MAPPING) {
    RCLCPP_WARN(node_->get_logger(),
                "TaskManager: StopMapping called on non-mapping task");
    return result;
  }

  result.map_name = task_params_.map_name;
  result.save_on_complete = task_params_.save_on_complete;

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Mapping task %s stopped (map_name='%s', save=%s)",
              task_id_.c_str(), result.map_name.c_str(),
              result.save_on_complete ? "true" : "false");

  CompleteTask(robot::v1::TASK_STATUS_COMPLETED, "Mapping completed by operator");
  result.ok = true;
  return result;
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
//  航点管理
// ================================================================

WaypointSourceInternal TaskManager::GetCurrentSource() const {
  if (state_.load() != TaskState::IDLE) {
    return WaypointSourceInternal::APP;
  }
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (planner_active_ && !planner_suppressed_) {
    return WaypointSourceInternal::PLANNER;
  }
  return WaypointSourceInternal::NONE;
}

robot::v1::GetActiveWaypointsResponse TaskManager::GetActiveWaypoints() const {
  robot::v1::GetActiveWaypointsResponse resp;
  resp.mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  const auto source = GetCurrentSource();

  if (source == WaypointSourceInternal::APP) {
    // App 任务: 返回全部航点列表 (map 坐标系, 方便 App 展示)
    std::lock_guard<std::mutex> lock(task_mutex_);
    resp.set_source(robot::v1::WAYPOINT_SOURCE_APP);
    resp.set_task_id(task_id_);
    resp.set_current_index(static_cast<uint32_t>(current_waypoint_idx_));
    resp.set_total_count(
        static_cast<uint32_t>(task_params_.waypoints.size()));

    for (size_t i = 0; i < task_params_.waypoints.size(); ++i) {
      auto *wp = resp.add_waypoints();
      wp->mutable_position()->set_x(task_params_.waypoints[i].x);
      wp->mutable_position()->set_y(task_params_.waypoints[i].y);
      wp->mutable_position()->set_z(task_params_.waypoints[i].z);
      wp->set_label(task_params_.waypoints[i].label);
      wp->set_is_current(i == current_waypoint_idx_);
      wp->set_index(static_cast<uint32_t>(i));
    }

    if (!task_params_.waypoints.empty()) {
      resp.set_progress_percent(
          100.0f * static_cast<float>(current_waypoint_idx_) /
          static_cast<float>(task_params_.waypoints.size()));
    }

  } else if (source == WaypointSourceInternal::PLANNER) {
    // 规划器: 返回当前单个航点
    std::lock_guard<std::mutex> lock(planner_mutex_);
    resp.set_source(robot::v1::WAYPOINT_SOURCE_PLANNER);
    resp.set_current_index(0);
    resp.set_total_count(1);

    auto *wp = resp.add_waypoints();
    wp->mutable_position()->set_x(last_planner_waypoint_.point.x);
    wp->mutable_position()->set_y(last_planner_waypoint_.point.y);
    wp->mutable_position()->set_z(last_planner_waypoint_.point.z);
    wp->set_is_current(true);
    wp->set_index(0);

  } else {
    resp.set_source(robot::v1::WAYPOINT_SOURCE_NONE);
    resp.set_total_count(0);
  }

  return resp;
}

robot::v1::ClearWaypointsResponse TaskManager::ClearWaypoints() {
  robot::v1::ClearWaypointsResponse resp;
  resp.mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  const auto source = GetCurrentSource();
  uint32_t cleared = 0;

  // 取消 App 任务 (如果有)
  if (state_.load() != TaskState::IDLE) {
    std::lock_guard<std::mutex> lock(task_mutex_);
    cleared = static_cast<uint32_t>(task_params_.waypoints.size());
    CompleteTask(robot::v1::TASK_STATUS_CANCELLED,
                 "Waypoints cleared by operator");
    resp.set_previous_source(robot::v1::WAYPOINT_SOURCE_APP);
  } else {
    // 抑制规划器转发
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (planner_active_ && !planner_suppressed_) {
      cleared = 1;
      planner_suppressed_ = true;
      resp.set_previous_source(robot::v1::WAYPOINT_SOURCE_PLANNER);
    } else {
      resp.set_previous_source(robot::v1::WAYPOINT_SOURCE_NONE);
    }
  }

  resp.set_cleared_count(cleared);

  // 立即停车: 发布 /stop=2 (full stop) 确保 pathFollower 马上停
  if (cleared > 0) {
    PublishStop(2);
    PublishStopWaypoint();
    RCLCPP_INFO(node_->get_logger(),
                "TaskManager: Cleared %u waypoints (source=%d), "
                "published /stop=2 + stop waypoint",
                cleared, static_cast<int>(source));
  }

  return resp;
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
  // 仅在 frame_id 变化时赋值, 避免 100Hz 下每帧 std::string 堆操作
  if (!msg->header.frame_id.empty() &&
      msg->header.frame_id != odom_frame_) {
    odom_frame_ = msg->header.frame_id;
  }
}

void TaskManager::CheckArrival() {
  // 系统挂起时不检测到达
  if (system_suspended_.load()) {
    return;
  }
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

  // ── Phase 1: 在 task_mutex_ 下读取航点数据 (快速, 不做 TF) ──
  double wp_x, wp_y, wp_z;
  size_t wp_idx, wp_total;
  double arrival_radius, timeout_sec;
  std::chrono::steady_clock::time_point wp_start;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
      return;
    }
    const auto &wp = task_params_.waypoints[current_waypoint_idx_];
    wp_x = wp.x;
    wp_y = wp.y;
    wp_z = wp.z;
    wp_idx = current_waypoint_idx_;
    wp_total = task_params_.waypoints.size();
    arrival_radius = task_params_.arrival_radius;
    timeout_sec = waypoint_timeout_sec_;
    wp_start = waypoint_start_time_;
  }
  // ── task_mutex_ 已释放 ──

  // 航点超时检测 (无需持锁)
  if (timeout_sec > 0.0) {
    const auto elapsed = std::chrono::steady_clock::now() - wp_start;
    const double elapsed_sec =
        std::chrono::duration<double>(elapsed).count();
    if (elapsed_sec > timeout_sec) {
      RCLCPP_WARN(node_->get_logger(),
                  "TaskManager: Waypoint %zu/%zu timed out (%.0fs > %.0fs)",
                  wp_idx + 1, wp_total, elapsed_sec, timeout_sec);
      std::lock_guard<std::mutex> lock(task_mutex_);
      CompleteTask(robot::v1::TASK_STATUS_FAILED,
                   "Waypoint timeout: could not reach waypoint " +
                       std::to_string(wp_idx + 1) +
                       " within " +
                       std::to_string(static_cast<int>(timeout_sec)) +
                       "s");
      return;
    }
  }

  // ── Phase 2: TF 变换 (可能阻塞, 在 mutex 外执行) ──
  //
  // 优化: 用缓存的 TF 做粗略距离预判。如果 odom 坐标系下的近似距离
  // 远大于 arrival_radius, 跳过实时 TF 查询。缓存每 2 秒或距离接近时刷新。
  // 典型巡检任务中, 机器人 ~95% 的时间远离航点, 大幅减少 TF 查询。
  constexpr double kSkipMultiplier = 3.0;  // 3 倍 radius 以外直接跳过
  constexpr double kCacheRefreshSec = 2.0;

  if (cached_tf_valid_) {
    const auto age = std::chrono::steady_clock::now() - cached_tf_time_;
    if (std::chrono::duration<double>(age).count() < kCacheRefreshSec) {
      // 用缓存 TF 做近似变换
      const auto &t = cached_tf_.transform.translation;
      const double approx_ox = wp_x + t.x;  // 简化: 忽略旋转 (误差 < odom漂移)
      const double approx_oy = wp_y + t.y;
      const double approx_dx = rx - approx_ox;
      const double approx_dy = ry - approx_oy;
      const double approx_dist_sq = approx_dx * approx_dx + approx_dy * approx_dy;
      const double skip_thresh = kSkipMultiplier * arrival_radius;
      if (approx_dist_sq > skip_thresh * skip_thresh) {
        return;  // 远离航点, 跳过本轮
      }
    }
  }

  double odom_wx, odom_wy, odom_wz;
  if (!TransformToOdom(wp_x, wp_y, wp_z, odom_wx, odom_wy, odom_wz)) {
    return;
  }

  // 更新 TF 缓存
  try {
    cached_tf_ = tf_buffer_->lookupTransform(
        odom_frame_, "map", tf2::TimePointZero);
    cached_tf_valid_ = true;
    cached_tf_time_ = std::chrono::steady_clock::now();
  } catch (...) {
    // best-effort: TransformToOdom 已成功, 缓存更新失败不影响本轮
  }

  const double dx = rx - odom_wx;
  const double dy = ry - odom_wy;
  const double dist = std::sqrt(dx * dx + dy * dy);

  // ── Phase 3: 到达判定, 重新获取 task_mutex_ 修改状态 ──
  if (dist <= arrival_radius) {
    std::lock_guard<std::mutex> lock(task_mutex_);
    // 二次校验: 防止在 TF 期间任务被取消或航点已推进
    if (state_.load() != TaskState::RUNNING ||
        current_waypoint_idx_ != wp_idx) {
      return;
    }
    RCLCPP_INFO(node_->get_logger(),
                "TaskManager: Reached waypoint %zu/%zu (dist=%.2fm)",
                wp_idx + 1, wp_total, dist);
    AdvanceToNextWaypoint();
  }
}

void TaskManager::AdvanceToNextWaypoint() {
  // 已持有 task_mutex_
  current_waypoint_idx_++;
  waypoint_start_time_ = std::chrono::steady_clock::now();

  if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
    if (task_params_.loop) {
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
      CompleteTask(robot::v1::TASK_STATUS_COMPLETED,
                   "All waypoints reached");
      return;
    }
  }

  PublishCurrentWaypoint();

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
  // 已持有 task_mutex_ — 在锁内拷贝航点数据，锁外做 TF + 发布
  if (current_waypoint_idx_ >= task_params_.waypoints.size()) {
    return;
  }

  // Phase 1: 在锁内拷贝必要的航点数据
  const double wp_x = task_params_.waypoints[current_waypoint_idx_].x;
  const double wp_y = task_params_.waypoints[current_waypoint_idx_].y;
  const double wp_z = task_params_.waypoints[current_waypoint_idx_].z;
  const std::string wp_label = task_params_.waypoints[current_waypoint_idx_].label;
  const size_t wp_idx = current_waypoint_idx_;
  const size_t wp_total = task_params_.waypoints.size();
  const std::string frame = odom_frame_;

  // Phase 2: TF + 发布不需要 task_mutex_（调用者在此之后不再读写任务状态）
  // 注: 调用者保证 PublishCurrentWaypoint 是锁内最后一个需要任务数据的操作
  double odom_x, odom_y, odom_z;
  if (!TransformToOdom(wp_x, wp_y, wp_z, odom_x, odom_y, odom_z)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "TaskManager: Cannot publish waypoint — "
                         "map→odom tf not available yet");
    return;
  }

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = frame;
  msg.point.x = odom_x;
  msg.point.y = odom_y;
  msg.point.z = odom_z;

  pub_waypoint_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Published waypoint %zu/%zu "
              "map(%.2f,%.2f) → odom(%.2f,%.2f)%s",
              wp_idx + 1, wp_total,
              wp_x, wp_y, odom_x, odom_y,
              wp_label.empty() ? "" : (" [" + wp_label + "]").c_str());
}

void TaskManager::PublishStopWaypoint() {
  // 发布机器人当前位置 (odom 坐标系) 作为目标点
  // local_planner 收到后计算距离 ≈ 0, 自然停止运动
  double rx, ry;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    rx = robot_x_;
    ry = robot_y_;
  }

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = odom_frame_;
  msg.point.x = rx;
  msg.point.y = ry;
  msg.point.z = 0.0;

  pub_waypoint_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
              "TaskManager: Published stop waypoint at current position "
              "(%.2f, %.2f) frame=%s",
              rx, ry, odom_frame_.c_str());
}

void TaskManager::PublishStop(int level) {
  std_msgs::msg::Int8 msg;
  msg.data = static_cast<int8_t>(level);
  pub_stop_->publish(msg);
}

void TaskManager::CompleteTask(robot::v1::TaskStatus status,
                               const std::string &reason) {
  // 已持有 task_mutex_
  state_.store(TaskState::IDLE);
  system_suspended_.store(false);

  // 在锁内收集回调数据 — 外部子系统 (event_buffer, progress_callback) 在锁外调用
  const auto event_type =
      (status == robot::v1::TASK_STATUS_COMPLETED)
          ? robot::v1::EVENT_TYPE_TASK_COMPLETED
          : robot::v1::EVENT_TYPE_TASK_FAILED;
  const auto severity =
      (status == robot::v1::TASK_STATUS_COMPLETED)
          ? robot::v1::EVENT_SEVERITY_INFO
          : robot::v1::EVENT_SEVERITY_WARNING;

  const std::string event_title = "Task " + task_id_ + ": " + reason;
  const std::string event_desc = "Status: " + std::to_string(static_cast<int>(status));
  const float pct =
      (status == robot::v1::TASK_STATUS_COMPLETED) ? 100.0f
      : task_params_.waypoints.empty()
          ? 0.0f
          : 100.0f * static_cast<float>(current_waypoint_idx_) /
                static_cast<float>(task_params_.waypoints.size());
  const std::string tid = task_id_;

  RCLCPP_INFO(node_->get_logger(), "TaskManager: Task %s finished — %s",
              task_id_.c_str(), reason.c_str());

  // 注: 外部调用在 task_mutex_ 仍被持有时执行，但 AddEvent 内部不阻塞 ROS executor
  // (磁盘 I/O 已经在 persist_mutex_ 下，与 task_mutex_ 不竞争)
  if (event_buffer_) {
    event_buffer_->AddEvent(event_type, severity, event_title, event_desc);
  }

  if (progress_callback_) {
    progress_callback_(tid, status, pct, reason);
  }
}

std::string TaskManager::GenerateTaskId() {
  task_counter_++;
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  char buf[32];
  std::snprintf(buf, sizeof(buf), "task_%08lx_%04u",
                static_cast<unsigned long>(now_ms & 0xFFFFFFFF),
                static_cast<unsigned>(task_counter_));
  return std::string(buf);
}

}  // namespace core
}  // namespace remote_monitoring
