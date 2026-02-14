#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// 自定义机器人状态消息 (关节角度 + 电池 + IMU)
#include "interface/msg/robot_state.hpp"

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
class LocalizationScorer;
class FlightRecorder;
}  // namespace core

class StatusAggregator {
public:
  explicit StatusAggregator(rclcpp::Node *node);
  ~StatusAggregator();

  // 获取最新状态 — 返回 shared_ptr 避免 protobuf 深拷贝 (30Hz 热路径)
  std::shared_ptr<const robot::v1::FastState> GetFastState();
  std::shared_ptr<const robot::v1::SlowState> GetSlowState();

  /// 快速 TF 状态查询 — 无锁，无 protobuf 拷贝。
  /// 用于 ModeManager 转换守卫等仅需一个 bool 的场景。
  bool IsTfOk() const { return cached_tf_ok_.load(std::memory_order_relaxed); }
  
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
  void SetLocalizationScorer(std::shared_ptr<core::LocalizationScorer> scorer) {
    localization_scorer_ = std::move(scorer);
  }
  void SetFlightRecorder(std::shared_ptr<core::FlightRecorder> recorder) {
    flight_recorder_ = std::move(recorder);
  }

  /// 设置运行参数提供者 (由 GrpcGateway 注入, 从 SystemService 读取)
  using ConfigProvider = std::function<std::pair<std::string, uint64_t>()>;
  void SetConfigProvider(ConfigProvider provider) {
    config_provider_ = std::move(provider);
  }

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void TerrainCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void PathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void SlowDownCallback(const std_msgs::msg::Int8::ConstSharedPtr msg);
  void RobotStateCallback(const interface::msg::RobotState::ConstSharedPtr msg);
  
  void update_rates();
  void update_fast_state();
  void update_slow_state();
  bool check_tf(const std::string &target, const std::string &source);

  // 读取 /proc 系统资源 — 在后台线程缓存, 不阻塞 ROS executor
  double ReadCpuUsage();
  double ReadMemUsage();
  double ReadCpuTemp();
  void UpdateSysResourcesBackground();  // 后台采样, 由独立线程调用

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
  rclcpp::Subscription<interface::msg::RobotState>::SharedPtr sub_robot_state_;
  
  rclcpp::TimerBase::SharedPtr rate_timer_;
  rclcpp::TimerBase::SharedPtr fast_state_timer_;
  rclcpp::TimerBase::SharedPtr slow_state_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex fast_mutex_;
  std::mutex slow_mutex_;
  std::shared_ptr<const robot::v1::FastState> latest_fast_state_;
  std::shared_ptr<const robot::v1::SlowState> latest_slow_state_;

  // scratch protobuf — update_*_state() 用 Clear()+重填+Swap 复用内部缓冲区
  robot::v1::FastState scratch_fast_state_;
  robot::v1::SlowState scratch_slow_state_;

  // 无锁 TF 状态缓存
  std::atomic<bool> cached_tf_ok_{false};
  
  // 缓存最新 odom
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;

  // 缓存 RobotState (关节角度 + 电池 + IMU)
  interface::msg::RobotState::ConstSharedPtr latest_robot_state_;
  std::mutex robot_state_mutex_;

  // 模拟电池（当 RobotState 不可用时使用）
  bool use_simulated_battery_{true};
  double simulated_battery_{85.0};
  std::chrono::steady_clock::time_point battery_sim_start_;

  // 缓存 slow_down 值
  std::atomic<int8_t> slow_down_level_{0};

  // 模式提供者
  ModeProvider mode_provider_;

  // 配置提供者 (返回 JSON + version)
  ConfigProvider config_provider_;

  // 健康和围栏监控 (用于 SlowState 扩展)
  std::shared_ptr<core::HealthMonitor> health_monitor_;
  std::shared_ptr<core::GeofenceMonitor> geofence_monitor_;
  std::shared_ptr<core::LocalizationScorer> localization_scorer_;
  std::shared_ptr<core::FlightRecorder> flight_recorder_;

  // CPU 使用率计算所需的上一次采样值 (由后台线程写入)
  uint64_t prev_cpu_total_{0};
  uint64_t prev_cpu_idle_{0};

  // /proc 读取结果缓存 (由后台线程更新, ROS 线程只读 atomic)
  std::atomic<double> cached_cpu_percent_{0.0};
  std::atomic<double> cached_mem_percent_{0.0};
  std::atomic<double> cached_cpu_temp_{0.0};
  std::thread sys_resource_thread_;
  std::atomic<bool> sys_resource_stop_{false};
};

}  // namespace remote_monitoring
