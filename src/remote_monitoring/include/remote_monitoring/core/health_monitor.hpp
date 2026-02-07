#pragma once
/**
 * HealthMonitor — 独立子系统健康监控 + 自动降级
 *
 * 解耦原则:
 *   - 只订阅各子系统话题做频率统计
 *   - 只发布 /robot_health (聚合状态)
 *   - 通过回调通知 ModeManager 降级, 但 ModeManager 崩溃不影响本模块
 *
 * 健康等级:
 *   OK       - 一切正常
 *   DEGRADED - 可运行, 但有警告 (降速)
 *   CRITICAL - 需要干预 (降速 50%)
 *   FAULT    - 必须停车 (触发 ESTOP)
 *
 * 判定规则 (基于话题频率):
 *   SLAM:     odom_hz > 50 → OK,  > 20 → DEGRADED,  else → FAULT
 *   地形分析: terrain_hz > 3 → OK, > 1 → DEGRADED,  else → CRITICAL
 *   局部规划: path_hz > 3 → OK,   > 1 → DEGRADED,  else → CRITICAL
 *   TF链:    tf_ok → OK,          else → FAULT
 */

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 健康等级
enum class HealthLevel {
  OK = 0,
  DEGRADED = 1,
  CRITICAL = 2,
  FAULT = 3
};

/// 子系统健康信息
struct SubsystemHealth {
  std::string name;
  HealthLevel level{HealthLevel::OK};
  std::string message;
  double expected_hz{0.0};
  double actual_hz{0.0};
};

/// 机器人整体健康
struct RobotHealth {
  HealthLevel overall{HealthLevel::OK};
  std::vector<SubsystemHealth> subsystems;
};

/// 降级回调类型
using DegradeCallback = std::function<void(HealthLevel level, const std::string &reason)>;

class HealthMonitor {
public:
  explicit HealthMonitor(rclcpp::Node *node);

  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }

  /// 注入降级回调 (由 ModeManager 提供的 EmergencyStop 等)
  void SetDegradeCallback(DegradeCallback callback) {
    degrade_callback_ = std::move(callback);
  }

  /// 获取当前健康状态 (线程安全)
  RobotHealth GetHealth() const;

  /// 整体健康等级 (lock-free)
  HealthLevel GetOverallLevel() const { return overall_level_.load(); }

private:
  // 话题频率计数
  struct RateCounter {
    std::atomic<int> count{0};
    std::atomic<float> hz{0.0f};
    void tick() { count++; }
    void reset(double window_sec) {
      if (window_sec > 0.0) {
        hz.store(static_cast<float>(count.load() / window_sec));
      }
      count.store(0);
    }
  };

  void OdomTick(const nav_msgs::msg::Odometry::ConstSharedPtr) { odom_rate_.tick(); }
  void TerrainTick(const sensor_msgs::msg::PointCloud2::ConstSharedPtr) { terrain_rate_.tick(); }
  void PathTick(const nav_msgs::msg::Path::ConstSharedPtr) { path_rate_.tick(); }
  void LocalizationQualityCallback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
    localization_fitness_score_.store(msg->data);
  }

  void EvaluateLoop();
  bool CheckTf(const std::string &target, const std::string &source);

  rclcpp::Node *node_;

  // 订阅 (只做频率统计, 不处理数据)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_localization_quality_;

  // 发布
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;  // 直接停车 (不依赖 ControlService)

  // TF 检查
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 定时器
  rclcpp::TimerBase::SharedPtr rate_timer_;
  rclcpp::TimerBase::SharedPtr eval_timer_;

  // 频率计数器
  RateCounter odom_rate_;
  RateCounter terrain_rate_;
  RateCounter path_rate_;

  // 定位质量 (ICP fitness score, lower=better)
  std::atomic<float> localization_fitness_score_{-1.0f};  // -1 = not available

  // 参数
  double rate_window_sec_{2.0};
  double eval_hz_{5.0};
  std::string tf_map_frame_{"map"};
  std::string tf_odom_frame_{"odom"};
  std::string tf_body_frame_{"body"};

  // 状态
  mutable std::mutex health_mutex_;
  RobotHealth latest_health_;
  std::atomic<HealthLevel> overall_level_{HealthLevel::OK};
  HealthLevel last_reported_level_{HealthLevel::OK};

  // 回调
  std::shared_ptr<EventBuffer> event_buffer_;
  DegradeCallback degrade_callback_;
};

}  // namespace core
}  // namespace remote_monitoring
