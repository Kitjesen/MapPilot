#pragma once
/**
 * ModeManager — 形式化有限状态机
 *
 * Mealy 机 M = (S, Σ, Δ, δ, λ, s₀) 其中:
 *   S  = {IDLE, MANUAL, TELEOP, AUTONOMOUS, MAPPING, ESTOP}
 *   Σ  = {setMode(target), estop, clearEstop, locLost, tiltOver, fenceViolation, ...}
 *   δ  = 转换函数 (带守卫条件)
 *   λ  = 输出函数 (进入/退出动作)
 *   s₀ = IDLE
 *
 * 关键安全不变量:
 *   1. * → ESTOP 无条件、lock-free (不经过 SwitchMode)
 *   2. ESTOP → * 必须手动清除且所有安全条件满足
 *   3. → AUTONOMOUS 必须 tf_ok ∧ localization_valid
 *   4. → TELEOP 必须有有效 lease
 */

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class SafetyGate;
class EventBuffer;
class GeofenceMonitor;

/// 转换守卫: 外部注入的条件查询函数
struct TransitionGuards {
  std::function<bool()> tf_ok;               // TF 链完整
  std::function<bool()> localization_valid;   // 定位有效
  std::function<bool()> has_lease;            // 持有有效 lease
  std::function<bool()> slam_running;         // SLAM 在线
  std::function<bool()> estop_clear;          // 急停已清除
  std::function<bool()> tilt_safe;            // 倾斜正常
  std::function<bool()> fence_safe;           // 围栏安全
};

/// 转换拒绝原因
struct TransitionResult {
  bool success;
  std::string reason;  // 失败原因 (人类可读)
};

class ModeManager {
public:
  explicit ModeManager(rclcpp::Node *node);

  void SetSafetyGate(std::shared_ptr<SafetyGate> safety_gate) {
    safety_gate_ = std::move(safety_gate);
  }

  void SetEventBuffer(std::shared_ptr<EventBuffer> event_buffer) {
    event_buffer_ = std::move(event_buffer);
  }

  void SetGeofenceMonitor(std::shared_ptr<GeofenceMonitor> geofence) {
    geofence_ = std::move(geofence);
  }

  /// 注入转换守卫条件 (由 GrpcGateway 组装)
  void SetTransitionGuards(TransitionGuards guards) {
    guards_ = std::move(guards);
  }

  /// 切换模式 (带守卫检查)。返回详细结果。
  TransitionResult SwitchMode(robot::v1::RobotMode new_mode);

  /// 紧急停车 — lock-free, 独立路径, 最高优先级
  /// 不经过 SwitchMode, 不等锁, 直接停车
  void EmergencyStop(const std::string &reason = "operator");

  /// 清除急停 (需要所有安全条件满足)
  TransitionResult ClearEmergencyStop();

  /// 发布导航目标点
  void PublishGoalPose(double x, double y, double z,
                       double qx = 0.0, double qy = 0.0,
                       double qz = 0.0, double qw = 1.0);

  /// 取消导航
  void CancelNavigation();

  robot::v1::RobotMode GetCurrentMode() const { return current_mode_.load(); }

private:
  /// 检查转换是否合法 (守卫条件)
  TransitionResult CheckTransition(robot::v1::RobotMode from,
                                   robot::v1::RobotMode to) const;

  /// 执行退出当前状态的动作
  void ExitState(robot::v1::RobotMode state);
  /// 执行进入新状态的动作
  void EnterState(robot::v1::RobotMode state);

  void StopPathFollower();
  void StartPathFollower();

  rclcpp::Node *node_;
  std::shared_ptr<SafetyGate> safety_gate_;
  std::shared_ptr<EventBuffer> event_buffer_;
  std::shared_ptr<GeofenceMonitor> geofence_;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stop_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  std::atomic<robot::v1::RobotMode> current_mode_{robot::v1::ROBOT_MODE_IDLE};
  std::mutex mutex_;

  TransitionGuards guards_;
};

}  // namespace core
}  // namespace remote_monitoring
