#pragma once
/**
 * GeofenceMonitor — 独立围栏越界检测模块
 *
 * 解耦原则:
 *   - 只订阅 /Odometry + /navigation_boundary
 *   - 只发布 /geofence/status + /geofence/violation + /stop
 *   - 不依赖 ModeManager, SafetyGate, StatusAggregator
 *   - 即使 remote_monitoring 主进程崩溃, 此模块仍可独立运行
 *
 * 三级围栏策略:
 *   WARNING_ZONE:  距围栏 < margin_warn_m → 发布警告事件
 *   FENCE:         距围栏 < margin_stop_m → 发布 /stop + 急停事件
 *   VIOLATION:     机器人在围栏外 → 持续急停 + CRITICAL 事件
 */

#include <atomic>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 围栏状态枚举
enum class GeofenceState {
  NO_FENCE,    // 未设置围栏
  SAFE,        // 安全区内
  WARNING,     // 接近围栏边界
  VIOLATION    // 越界
};

class GeofenceMonitor {
public:
  explicit GeofenceMonitor(rclcpp::Node *node);

  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }

  /// 当前围栏状态 (lock-free 读取, 供外部查询)
  GeofenceState GetState() const { return state_.load(); }

  /// 到最近围栏边的距离 (m), 负值表示越界
  double GetMarginDistance() const { return margin_distance_.load(); }

  /// 围栏是否已设置
  bool HasFence() const { return has_fence_.load(); }

private:
  // ROS 回调
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void BoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg);
  void CheckLoop();

  // 几何计算
  bool PointInPolygon(double px, double py) const;
  double PointToPolygonDistance(double px, double py) const;
  double PointToSegmentDistance(double px, double py,
                                double ax, double ay,
                                double bx, double by) const;

  rclcpp::Node *node_;

  // 订阅
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_boundary_;

  // 发布 (独立的安全通道)
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  // 定时检查
  rclcpp::TimerBase::SharedPtr check_timer_;

  // 参数
  double margin_warn_m_{3.0};   // 警告距离
  double margin_stop_m_{0.5};   // 急停距离
  double check_hz_{20.0};       // 检查频率

  // 围栏几何 (mutex 保护)
  mutable std::mutex fence_mutex_;
  std::vector<geometry_msgs::msg::Point32> polygon_;
  std::atomic<bool> has_fence_{false};

  // 机器人位置 (mutex 保护)
  mutable std::mutex odom_mutex_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool odom_received_{false};

  // 状态 (atomic, lock-free 读取)
  std::atomic<GeofenceState> state_{GeofenceState::NO_FENCE};
  std::atomic<double> margin_distance_{0.0};

  // 事件去重
  std::shared_ptr<EventBuffer> event_buffer_;
  GeofenceState last_reported_state_{GeofenceState::NO_FENCE};
};

}  // namespace core
}  // namespace remote_monitoring
