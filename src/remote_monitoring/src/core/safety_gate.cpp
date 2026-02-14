#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <cmath>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace remote_monitoring {
namespace core {

SafetyGate::SafetyGate(rclcpp::Node *node) : node_(node) {
  // --- 基础参数 ---
  node_->declare_parameter<double>("deadman_timeout_ms", 300.0);
  node_->declare_parameter<double>("max_speed", 1.0);
  node_->declare_parameter<double>("max_angular", 1.0);
  node_->declare_parameter<double>("tilt_limit_deg", 30.0);
  node_->declare_parameter<std::string>("safety_odom_topic", "/nav/odometry");
  node_->declare_parameter<std::string>("cmd_vel_topic", "/nav/cmd_vel");
  node_->declare_parameter<std::string>("stop_topic", "/nav/stop");

  // --- 近场避障参数 ---
  node_->declare_parameter<double>("obstacle_height_thre", 0.2);
  node_->declare_parameter<double>("stop_distance", 0.8);
  node_->declare_parameter<double>("slow_distance", 2.0);
  node_->declare_parameter<double>("vehicle_width", 0.6);
  node_->declare_parameter<double>("vehicle_width_margin", 0.1);
  // terrain_map_topic 已由 StatusAggregator 声明，此处直接复用
  
  deadman_timeout_ms_ = std::chrono::milliseconds(
    static_cast<int>(node_->get_parameter("deadman_timeout_ms").as_double()));
  max_speed_ = node_->get_parameter("max_speed").as_double();
  max_angular_ = node_->get_parameter("max_angular").as_double();
  tilt_limit_deg_ = node_->get_parameter("tilt_limit_deg").as_double();

  obstacle_height_thre_ = node_->get_parameter("obstacle_height_thre").as_double();
  stop_distance_ = node_->get_parameter("stop_distance").as_double();
  slow_distance_ = node_->get_parameter("slow_distance").as_double();
  vehicle_width_ = node_->get_parameter("vehicle_width").as_double();
  vehicle_width_margin_ = node_->get_parameter("vehicle_width_margin").as_double();
  
  const auto odom_topic = node_->get_parameter("safety_odom_topic").as_string();
  const auto cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
  const auto stop_topic = node_->get_parameter("stop_topic").as_string();
  const auto terrain_map_topic = node_->get_parameter("terrain_map_topic").as_string();
  
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&SafetyGate::OdomCallback, this, std::placeholders::_1));

  sub_terrain_map_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_map_topic, 5,
    std::bind(&SafetyGate::TerrainMapCallback, this, std::placeholders::_1));
  
  pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic, 5);

  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>(stop_topic, 5);

  RCLCPP_INFO(node_->get_logger(),
    "SafetyGate: obstacle avoidance enabled — "
    "height_thre=%.2f, stop_dist=%.2f, slow_dist=%.2f, width=%.2f",
    obstacle_height_thre_, stop_distance_, slow_distance_, vehicle_width_);
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

SafetyGate::TeleopResult SafetyGate::ProcessTeleopFull(
    const robot::v1::TeleopCommand &cmd) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ── Process ──
  last_teleop_time_ = std::chrono::system_clock::now();
  limit_reasons_.clear();
  TeleopResult result;
  result.velocity = ApplyLimits(cmd.target_velocity());

  // ── SafetyStatus (inline, same as GetSafetyStatus body) ──
  result.safety_status.set_estop_active(estop_active_);
  const auto mode = current_mode_.load();
  if (mode == robot::v1::ROBOT_MODE_TELEOP) {
    const auto now = std::chrono::system_clock::now();
    result.safety_status.set_deadman_active(
        (now - last_teleop_time_) > deadman_timeout_ms_);
  } else {
    result.safety_status.set_deadman_active(false);
  }
  const bool tilt_limit = std::abs(roll_deg_) > tilt_limit_deg_ ||
                          std::abs(pitch_deg_) > tilt_limit_deg_;
  result.safety_status.set_tilt_limit_active(tilt_limit);
  result.safety_status.set_speed_limited(!limit_reasons_.empty());
  result.safety_status.set_max_allowed_speed(max_speed_);
  std::string msg;
  for (const auto &reason : limit_reasons_) {
    msg += reason + "; ";
  }
  result.safety_status.set_safety_message(msg);

  // ── LimitReasons (move, 避免拷贝) ──
  result.limit_reasons = std::move(limit_reasons_);
  limit_reasons_.clear();  // move 后保持一致状态

  return result;
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

  // 保存位姿用于 terrain_map → body 坐标转换
  vehicle_x_ = msg->pose.pose.position.x;
  vehicle_y_ = msg->pose.pose.position.y;
  vehicle_yaw_ = yaw;
}

void SafetyGate::TerrainMapCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // 获取当前位姿快照 (odom 坐标系)
  double vx, vy, vyaw;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vx = vehicle_x_;
    vy = vehicle_y_;
    vyaw = vehicle_yaw_;
  }

  const double cos_yaw = std::cos(vyaw);
  const double sin_yaw = std::sin(vyaw);
  const double half_width = vehicle_width_ / 2.0 + vehicle_width_margin_;

  bool found_stop = false;
  bool found_slow = false;
  float min_dist = 999.0f;

  // 遍历 terrain_map 点云 (odom 坐标系, intensity = 障碍物高度)
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_i) {
    const float h = *it_i;  // 障碍物高度
    if (h <= obstacle_height_thre_) {
      continue;  // 低于阈值，不是障碍物
    }

    // odom → body 坐标转换 (2D 平面旋转)
    const float dx = *it_x - static_cast<float>(vx);
    const float dy = *it_y - static_cast<float>(vy);
    const float px = static_cast<float>( cos_yaw * dx + sin_yaw * dy);  // 前方距离
    const float py = static_cast<float>(-sin_yaw * dx + cos_yaw * dy);  // 左右偏移

    // 仅检测前方 (px > 0) 且在车身宽度内的障碍物
    if (px > 0.0f && std::fabs(py) < half_width) {
      if (px < min_dist) {
        min_dist = px;
      }
      if (px < static_cast<float>(stop_distance_)) {
        found_stop = true;
      } else if (px < static_cast<float>(slow_distance_)) {
        found_slow = true;
      }
    }
  }

  // 原子更新检测结果
  obstacle_stop_.store(found_stop);
  obstacle_slow_.store(found_slow);
  nearest_obstacle_dist_.store(min_dist);
}

robot::v1::Twist SafetyGate::ApplyLimits(const robot::v1::Twist &target) {
  // ── 模式门禁：仅在 TELEOP 模式下发布 /cmd_vel ──
  if (current_mode_.load() != robot::v1::ROBOT_MODE_TELEOP) {
    limit_reasons_.push_back("not_teleop_mode");
    // 返回零速度，但不发布到 /cmd_vel，避免与 pathFollower 冲突
    robot::v1::Twist zero;
    return zero;
  }

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

  // ── 定位健康评分驱动的自动降速 ──
  if (loc_speed_scale_provider_) {
    const float scale = loc_speed_scale_provider_();
    if (scale < 1.0f) {
      limited.mutable_linear()->set_x(limited.linear().x() * scale);
      limited.mutable_linear()->set_y(limited.linear().y() * scale);
      limited.mutable_angular()->set_z(limited.angular().z() * scale);
      if (scale < 0.01f) {
        limit_reasons_.push_back("loc_lost_stop");
      } else {
        limit_reasons_.push_back("loc_degraded_slow");
      }
    }
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

  // ── 近场避障（仅作用于线速度，允许原地转向避让）──
  if (obstacle_stop_.load()) {
    // 急停区：前方 < stop_distance_ 有超高障碍物
    limited.mutable_linear()->set_x(0);
    limited.mutable_linear()->set_y(0);
    limit_reasons_.push_back("obstacle_stop");

    if (!obstacle_event_sent_ && event_buffer_) {
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SAFETY_ESTOP,
          robot::v1::EVENT_SEVERITY_WARNING,
          "Obstacle stop",
          "Obstacle detected at " +
              std::to_string(nearest_obstacle_dist_.load()) +
              "m (< " + std::to_string(stop_distance_) + "m), speed zeroed");
      obstacle_event_sent_ = true;
    }
  } else if (obstacle_slow_.load()) {
    // 减速区：stop_distance_ < 前方 < slow_distance_
    const float dist = nearest_obstacle_dist_.load();
    const double scale = std::max(0.2, static_cast<double>(dist) / slow_distance_);
    limited.mutable_linear()->set_x(limited.linear().x() * scale);
    limited.mutable_linear()->set_y(limited.linear().y() * scale);
    limit_reasons_.push_back("obstacle_slow");
    obstacle_event_sent_ = false;
  } else {
    obstacle_event_sent_ = false;
  }
  
  // 急停（最高优先级，覆盖所有速度）
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
