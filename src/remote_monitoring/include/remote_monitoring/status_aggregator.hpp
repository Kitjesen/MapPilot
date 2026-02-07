#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "control.pb.h"
#include "telemetry.pb.h"

namespace remote_monitoring {

class TopicRate {
public:
  void tick();
  void reset(double window_sec);
  float rate_hz() const;

private:
  std::atomic<int> count_{0};
  std::atomic<float> rate_hz_{0.0f};
};

// 模式查询回调类型
using ModeProvider = std::function<robot::v1::RobotMode()>;

namespace core {
class HealthMonitor;
class GeofenceMonitor;
}  // namespace core

class StatusAggregator {
public:
  explicit StatusAggregator(rclcpp::Node *node);

  // 获取最新状态
  robot::v1::FastState GetFastState();
  robot::v1::SlowState GetSlowState();
  
  double fast_state_hz() const { return fast_hz_; }
  double slow_state_hz() const { return slow_hz_; }

  // 设置模式提供者（由 GrpcGateway 注入，从 ControlService 读取真实模式）
  void SetModeProvider(ModeProvider provider);

  // 设置健康和围栏监控（由 GrpcGateway 注入，供 SlowState 填充）
  void SetHealthMonitor(std::shared_ptr<core::HealthMonitor> monitor) {
    health_monitor_ = std::move(monitor);
  }
  void SetGeofenceMonitor(std::shared_ptr<core::GeofenceMonitor> monitor) {
    geofence_monitor_ = std::move(monitor);
  }

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void TerrainCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void PathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void SlowDownCallback(const std_msgs::msg::Int8::ConstSharedPtr msg);
  
  void update_rates();
  void update_fast_state();
  void update_slow_state();
  bool check_tf(const std::string &target, const std::string &source);

  // 读取 /proc 系统资源（Linux）
  static double ReadCpuUsage();
  static double ReadMemUsage();
  static double ReadCpuTemp();

  rclcpp::Node *node_;
  double fast_hz_{30.0};
  double slow_hz_{1.0};
  double window_sec_{2.0};
  std::string odom_topic_;
  std::string terrain_map_topic_;
  std::string path_topic_;
  std::string tf_map_frame_;
  std::string tf_odom_frame_;
  std::string tf_body_frame_;

  TopicRate odom_rate_;
  TopicRate terrain_rate_;
  TopicRate path_rate_;
  TopicRate lidar_rate_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_slow_down_;
  
  rclcpp::TimerBase::SharedPtr rate_timer_;
  rclcpp::TimerBase::SharedPtr fast_state_timer_;
  rclcpp::TimerBase::SharedPtr slow_state_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex fast_mutex_;
  std::mutex slow_mutex_;
  robot::v1::FastState latest_fast_state_;
  robot::v1::SlowState latest_slow_state_;
  
  // 缓存最新 odom
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;

  // 缓存 slow_down 值
  std::atomic<int8_t> slow_down_level_{0};

  // 模式提供者
  ModeProvider mode_provider_;

  // 健康和围栏监控 (用于 SlowState 扩展)
  std::shared_ptr<core::HealthMonitor> health_monitor_;
  std::shared_ptr<core::GeofenceMonitor> geofence_monitor_;

  // CPU 使用率计算所需的上一次采样值
  mutable uint64_t prev_cpu_total_{0};
  mutable uint64_t prev_cpu_idle_{0};
};

}  // namespace remote_monitoring
