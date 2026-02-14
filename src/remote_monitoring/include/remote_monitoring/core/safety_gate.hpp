#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int8.hpp"

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

class SafetyGate {
public:
  explicit SafetyGate(rclcpp::Node *node);

  // 设置事件缓冲区（用于自动生成安全事件）
  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }
  
  // 处理遥操作命令（返回限幅后的安全速度）
  robot::v1::Twist ProcessTeleopCommand(const robot::v1::TeleopCommand &cmd);
  
  // 获取当前安全状态
  robot::v1::SafetyStatus GetSafetyStatus();

  // 返回最近一次限幅原因，供 TeleopFeedback 透传。
  std::vector<std::string> GetLimitReasons();

  /// 合并调用: 单次加锁完成 ProcessTeleopCommand + GetSafetyStatus + GetLimitReasons。
  /// 消除 teleop 热路径上的 3 次 lock/unlock (10-30Hz)。
  struct TeleopResult {
    robot::v1::Twist velocity;
    robot::v1::SafetyStatus safety_status;
    std::vector<std::string> limit_reasons;
  };
  TeleopResult ProcessTeleopFull(const robot::v1::TeleopCommand &cmd);

  // 设置急停状态（true: 急停，false: 清除）。
  void SetEmergencyStop(bool active);
  
  // 检查 deadman 超时（仅在 TELEOP 模式下生效）
  void CheckDeadman();

  // 设置当前运行模式（由 ModeManager 调用）
  void SetCurrentMode(robot::v1::RobotMode mode);

  /// 注入定位健康评分速度缩放 (由 LocalizationScorer 提供)
  /// SafetyGate 在 ApplyLimits 中乘以此因子限速
  void SetLocSpeedScaleProvider(std::function<float()> provider) {
    loc_speed_scale_provider_ = std::move(provider);
  }

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void TerrainMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  robot::v1::Twist ApplyLimits(const robot::v1::Twist &target);
  
  rclcpp::Node *node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_map_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  
  std::mutex mutex_;
  std::chrono::system_clock::time_point last_teleop_time_;
  std::chrono::milliseconds deadman_timeout_ms_{300};
  
  bool estop_active_{false};
  double roll_deg_{0.0};
  double pitch_deg_{0.0};
  double max_speed_{1.0};
  double max_angular_{1.0};
  double tilt_limit_deg_{30.0};
  
  std::vector<std::string> limit_reasons_;

  // 当前运行模式（决定 deadman 是否生效 + /cmd_vel 发布权）
  std::atomic<robot::v1::RobotMode> current_mode_{robot::v1::ROBOT_MODE_IDLE};

  // ---- 近场避障 ----
  // 从 /Odometry 提取的位姿 (odom 坐标系), 用于 terrain_map → body 转换
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double vehicle_yaw_{0.0};

  // 障碍物检测结果 (由 TerrainMapCallback 更新, ApplyLimits 读取)
  std::atomic<bool> obstacle_stop_{false};    // 急停区有障碍
  std::atomic<bool> obstacle_slow_{false};    // 减速区有障碍
  std::atomic<float> nearest_obstacle_dist_{999.0f};  // 前方最近障碍距离 (m)

  // 避障参数 (从 yaml 读取)
  double obstacle_height_thre_{0.2};   // 障碍物高度阈值 (m)
  double stop_distance_{0.8};          // 急停距离 (m)
  double slow_distance_{2.0};          // 减速距离 (m)
  double vehicle_width_{0.6};          // 车身宽度 (m)
  double vehicle_width_margin_{0.1};   // 宽度安全裕度 (m)

  // 事件生成
  std::shared_ptr<EventBuffer> event_buffer_;
  bool deadman_event_sent_{false};   // 避免重复发送
  bool tilt_event_sent_{false};
  bool obstacle_event_sent_{false};  // 避免障碍物事件重复发送

  // 定位健康评分速度缩放 (由 LocalizationScorer 注入)
  std::function<float()> loc_speed_scale_provider_;
};

}  // namespace core
}  // namespace remote_monitoring
