#pragma once
/**
 * LocalizationScorer — 定位健康评分器（全新子系统）
 *
 * 从零创建，非基于已有代码的补丁。
 *
 * 原理:
 *   聚合 4 个独立信号源，加权计算 0-100 综合定位健康分:
 *     1) ICP 匹配质量 (40 分)  — /localization_quality 话题
 *     2) TF 新鲜度    (30 分)  — map→odom 变换的年龄
 *     3) TF 稳定性    (15 分)  — 变换发布间隔的抖动(标准差)
 *     4) 帧间一致性   (15 分)  — map→odom 跳变检测
 *
 * 输出:
 *   - score        [0-100]     → StatusAggregator 填入 FastState
 *   - speed_scale  [0.0-1.0]   → SafetyGate 限速因子
 *   - quality      LocQuality  → 事件生成 (定位丢失/恢复)
 *
 * 线程安全:
 *   - 所有输出通过 atomic 发布, 读取端无锁
 *   - 仅 EvaluateTick 内部使用 mutex (评估频率 10Hz, 无争用)
 */

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 评分详情分解
struct ScoreBreakdown {
  float icp_quality;        // 0-40
  float tf_freshness;       // 0-30
  float tf_stability;       // 0-15
  float frame_consistency;  // 0-15
  float total;              // 0-100
  float speed_scale;        // 0.0-1.0
};

/// 定位质量等级
enum class LocQuality : uint8_t {
  EXCELLENT = 0,  // 80-100
  GOOD      = 1,  // 60-80
  DEGRADED  = 2,  // 40-60
  POOR      = 3,  // 20-40
  LOST      = 4   //  0-20
};

class LocalizationScorer {
public:
  explicit LocalizationScorer(rclcpp::Node *node);

  void SetEventBuffer(std::shared_ptr<EventBuffer> eb) {
    event_buffer_ = std::move(eb);
  }

  /// 综合分数 [0,100] — lock-free
  float GetScore() const {
    return score_.load(std::memory_order_relaxed);
  }

  /// 速度缩放因子 [0.0,1.0] — lock-free
  float GetSpeedScale() const {
    return speed_scale_.load(std::memory_order_relaxed);
  }

  /// 质量等级 — lock-free
  LocQuality GetQuality() const {
    return static_cast<LocQuality>(quality_.load(std::memory_order_relaxed));
  }

  /// 获取详细分解 (加锁)
  ScoreBreakdown GetBreakdown() const;

private:
  // ---- 回调 ----
  void IcpCallback(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void EvaluateTick();

  // ---- 评分组件 ----
  float ComputeIcpScore(float fitness) const;
  float ComputeTfFreshness() const;
  float ComputeTfStability();
  float ComputeFrameConsistency();
  static LocQuality ScoreToQuality(float score);
  static float ScoreToSpeedScale(float score);

  rclcpp::Node *node_;
  std::shared_ptr<EventBuffer> event_buffer_;

  // 自有 TF 基础设施 (独立于 StatusAggregator, 解耦)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_icp_;
  rclcpp::TimerBase::SharedPtr eval_timer_;

  // ---- 输入信号 ----
  std::atomic<float> icp_fitness_{-1.0f};  // lower=better, -1=不可用

  // TF 时间戳历史 (计算抖动)
  std::mutex tf_mutex_;
  std::deque<std::chrono::steady_clock::time_point> tf_timestamps_;
  static constexpr size_t kMaxTfHistory = 50;

  // 帧间一致性: map→odom 跳变滑动窗口
  std::deque<double> drift_history_;
  static constexpr size_t kMaxDriftHistory = 20;
  double last_map_x_{0.0}, last_map_y_{0.0};
  bool has_consistency_baseline_{false};

  // ---- 输出 (lock-free) ----
  std::atomic<float> score_{0.0f};
  std::atomic<float> speed_scale_{1.0f};
  std::atomic<int> quality_{static_cast<int>(LocQuality::LOST)};

  mutable std::mutex breakdown_mutex_;
  ScoreBreakdown latest_breakdown_{};

  // 事件去重
  LocQuality last_event_quality_{LocQuality::EXCELLENT};

  // ---- 参数 ----
  std::string tf_map_frame_{"map"};
  std::string tf_odom_frame_{"odom"};
  std::string tf_body_frame_{"body"};
  float icp_excellent_{0.05f};   // fitness ≤ this → 满分 40
  float icp_poor_{1.0f};         // fitness ≥ this → 零分
  double tf_max_age_ms_{500.0};  // TF 年龄 ≥ this → 零分
};

}  // namespace core
}  // namespace remote_monitoring
