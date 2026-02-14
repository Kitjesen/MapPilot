#include "remote_monitoring/core/localization_scorer.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>

namespace remote_monitoring {
namespace core {

// ================================================================
//  构造: 声明参数、订阅话题、启动评估定时器
// ================================================================

LocalizationScorer::LocalizationScorer(rclcpp::Node *node)
    : node_(node),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_) {
  // 声明参数 (不与其他模块冲突)
  if (!node_->has_parameter("loc_scorer_hz"))
    node_->declare_parameter<double>("loc_scorer_hz", 10.0);
  if (!node_->has_parameter("icp_excellent_threshold"))
    node_->declare_parameter<double>("icp_excellent_threshold", 0.05);
  if (!node_->has_parameter("icp_poor_threshold"))
    node_->declare_parameter<double>("icp_poor_threshold", 1.0);
  if (!node_->has_parameter("tf_max_age_ms"))
    node_->declare_parameter<double>("tf_max_age_ms", 500.0);

  const double eval_hz = node_->get_parameter("loc_scorer_hz").as_double();
  icp_excellent_ = static_cast<float>(
      node_->get_parameter("icp_excellent_threshold").as_double());
  icp_poor_ = static_cast<float>(
      node_->get_parameter("icp_poor_threshold").as_double());
  tf_max_age_ms_ = node_->get_parameter("tf_max_age_ms").as_double();

  // 复用已声明的 TF frame 参数
  if (node_->has_parameter("tf_map_frame"))
    tf_map_frame_ = node_->get_parameter("tf_map_frame").as_string();
  if (node_->has_parameter("tf_odom_frame"))
    tf_odom_frame_ = node_->get_parameter("tf_odom_frame").as_string();
  if (node_->has_parameter("tf_body_frame"))
    tf_body_frame_ = node_->get_parameter("tf_body_frame").as_string();

  // 订阅 ICP 质量 — 话题名可通过参数覆盖 (标准接口契约)
  if (!node_->has_parameter("localization_quality_topic"))
    node_->declare_parameter<std::string>(
        "localization_quality_topic", "/nav/localization_quality");
  const auto loc_quality_topic =
      node_->get_parameter("localization_quality_topic").as_string();
  sub_icp_ = node_->create_subscription<std_msgs::msg::Float32>(
      loc_quality_topic, 10,
      std::bind(&LocalizationScorer::IcpCallback, this,
                std::placeholders::_1));

  // 评估定时器
  eval_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(eval_hz, 1.0)),
      std::bind(&LocalizationScorer::EvaluateTick, this));

  RCLCPP_INFO(node_->get_logger(),
              "[LocalizationScorer] started: eval_hz=%.1f, "
              "icp_excellent=%.3f, icp_poor=%.3f, tf_max_age=%.0fms",
              eval_hz, icp_excellent_, icp_poor_, tf_max_age_ms_);
}

// ================================================================
//  回调
// ================================================================

void LocalizationScorer::IcpCallback(
    const std_msgs::msg::Float32::ConstSharedPtr msg) {
  icp_fitness_.store(msg->data, std::memory_order_relaxed);
}

ScoreBreakdown LocalizationScorer::GetBreakdown() const {
  std::lock_guard<std::mutex> lock(breakdown_mutex_);
  return latest_breakdown_;
}

// ================================================================
//  评分组件
// ================================================================

/// 1) ICP 匹配质量 (0-40 分)
/// fitness ≤ icp_excellent_ → 40, ≥ icp_poor_ → 0, 中间线性
float LocalizationScorer::ComputeIcpScore(float fitness) const {
  if (fitness < 0.0f) return 0.0f;  // 不可用
  if (fitness <= icp_excellent_) return 40.0f;
  if (fitness >= icp_poor_) return 0.0f;
  const float t = (fitness - icp_excellent_) / (icp_poor_ - icp_excellent_);
  return 40.0f * (1.0f - t);
}

/// 2) TF 新鲜度 (0-30 分)
/// 计算 map→odom TF 的时间戳距当前的年龄
float LocalizationScorer::ComputeTfFreshness() const {
  try {
    auto tf = tf_buffer_.lookupTransform(
        tf_map_frame_, tf_odom_frame_, tf2::TimePointZero);

    const rclcpp::Time tf_stamp(tf.header.stamp);
    const rclcpp::Time now = node_->get_clock()->now();
    const double age_ms = (now - tf_stamp).nanoseconds() / 1e6;

    if (age_ms < 0.0) return 30.0f;  // 时钟偏移, 给满分
    if (age_ms <= 100.0) return 30.0f;
    if (age_ms >= tf_max_age_ms_) return 0.0f;
    return 30.0f * static_cast<float>(
               1.0 - (age_ms - 100.0) / (tf_max_age_ms_ - 100.0));
  } catch (...) {
    return 0.0f;  // TF 不可用
  }
}

/// 3) TF 稳定性 (0-15 分)
/// 通过评估 TF 成功查询的时间间隔方差来衡量抖动
float LocalizationScorer::ComputeTfStability() {
  bool tf_available = false;
  try {
    tf_buffer_.lookupTransform(
        tf_map_frame_, tf_odom_frame_, tf2::TimePointZero);
    tf_available = true;
  } catch (...) {}

  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(tf_mutex_);
  if (tf_available) {
    tf_timestamps_.push_back(now);
    if (tf_timestamps_.size() > kMaxTfHistory) {
      tf_timestamps_.pop_front();
    }
  }

  if (tf_timestamps_.size() < 3) return 0.0f;

  // 计算间隔标准差
  std::vector<double> intervals;
  intervals.reserve(tf_timestamps_.size() - 1);
  for (size_t i = 1; i < tf_timestamps_.size(); ++i) {
    intervals.push_back(
        std::chrono::duration<double, std::milli>(
            tf_timestamps_[i] - tf_timestamps_[i - 1])
            .count());
  }

  const double mean =
      std::accumulate(intervals.begin(), intervals.end(), 0.0) /
      static_cast<double>(intervals.size());
  double variance = 0.0;
  for (double interval : intervals) {
    variance += (interval - mean) * (interval - mean);
  }
  variance /= static_cast<double>(intervals.size());
  const double stddev = std::sqrt(variance);

  // stddev ≤ 10ms → 15 分 (完美稳定)
  // stddev ≥ 200ms → 0 分 (严重抖动)
  if (stddev <= 10.0) return 15.0f;
  if (stddev >= 200.0) return 0.0f;
  return 15.0f * static_cast<float>(1.0 - (stddev - 10.0) / 190.0);
}

/// 4) 帧间一致性 (0-15 分)
/// 跟踪 map→odom TF 变化的平滑度 — 突变表示定位重置或漂移
float LocalizationScorer::ComputeFrameConsistency() {
  try {
    auto tf_map_odom = tf_buffer_.lookupTransform(
        tf_map_frame_, tf_odom_frame_, tf2::TimePointZero);
    const double mx = tf_map_odom.transform.translation.x;
    const double my = tf_map_odom.transform.translation.y;

    if (!has_consistency_baseline_) {
      last_map_x_ = mx;
      last_map_y_ = my;
      has_consistency_baseline_ = true;
      return 15.0f;  // 初始状态假设良好
    }

    // map→odom 变化量 (每个 eval tick)
    const double dx = mx - last_map_x_;
    const double dy = my - last_map_y_;
    const double jump = std::sqrt(dx * dx + dy * dy);

    last_map_x_ = mx;
    last_map_y_ = my;

    // 滑动窗口记录跳变
    drift_history_.push_back(jump);
    if (drift_history_.size() > kMaxDriftHistory) {
      drift_history_.pop_front();
    }

    // 取最大跳变
    const double max_jump =
        *std::max_element(drift_history_.begin(), drift_history_.end());

    // max_jump ≤ 0.01m → 15 分 (非常稳定)
    // max_jump ≥ 0.5m  → 0 分  (大幅跳变)
    if (max_jump <= 0.01) return 15.0f;
    if (max_jump >= 0.5) return 0.0f;
    return 15.0f * static_cast<float>(1.0 - (max_jump - 0.01) / 0.49);
  } catch (...) {
    return 0.0f;
  }
}

// ================================================================
//  辅助: 分数 → 质量等级 / 速度缩放
// ================================================================

LocQuality LocalizationScorer::ScoreToQuality(float score) {
  if (score >= 80.0f) return LocQuality::EXCELLENT;
  if (score >= 60.0f) return LocQuality::GOOD;
  if (score >= 40.0f) return LocQuality::DEGRADED;
  if (score >= 20.0f) return LocQuality::POOR;
  return LocQuality::LOST;
}

float LocalizationScorer::ScoreToSpeedScale(float score) {
  if (score >= 80.0f) return 1.0f;
  if (score >= 60.0f) return 0.7f;
  if (score >= 40.0f) return 0.4f;
  if (score >= 20.0f) return 0.2f;
  return 0.0f;  // 定位丢失 → 完全停车
}

// ================================================================
//  主评估循环 (定时器回调, ~10Hz)
// ================================================================

void LocalizationScorer::EvaluateTick() {
  const float fitness = icp_fitness_.load(std::memory_order_relaxed);

  const float icp_sc   = ComputeIcpScore(fitness);
  const float fresh_sc = ComputeTfFreshness();
  const float stab_sc  = ComputeTfStability();
  const float cons_sc  = ComputeFrameConsistency();

  const float total = std::clamp(
      icp_sc + fresh_sc + stab_sc + cons_sc, 0.0f, 100.0f);
  const float scale = ScoreToSpeedScale(total);
  const LocQuality q = ScoreToQuality(total);

  // 写入 atomic
  score_.store(total, std::memory_order_relaxed);
  speed_scale_.store(scale, std::memory_order_relaxed);
  quality_.store(static_cast<int>(q), std::memory_order_relaxed);

  // 写入详细分解
  {
    std::lock_guard<std::mutex> lock(breakdown_mutex_);
    latest_breakdown_ = {icp_sc, fresh_sc, stab_sc, cons_sc, total, scale};
  }

  // ---- 事件生成: 质量等级变化 ----
  if (event_buffer_ && q != last_event_quality_) {
    const LocQuality old_q = last_event_quality_;
    last_event_quality_ = q;

    if (q == LocQuality::LOST) {
      char buf[256];
      snprintf(buf, sizeof(buf),
               "Score %.0f (ICP=%.0f TF=%.0f Stab=%.0f Cons=%.0f)",
               total, icp_sc, fresh_sc, stab_sc, cons_sc);
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_LOCALIZATION_LOST,
          robot::v1::EVENT_SEVERITY_CRITICAL,
          "Localization lost", buf);
    } else if (q == LocQuality::POOR &&
               old_q < LocQuality::POOR /* was better */) {
      char buf[64];
      snprintf(buf, sizeof(buf), "Score=%.0f", total);
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_LOCALIZATION_LOST,
          robot::v1::EVENT_SEVERITY_WARNING,
          "Localization degraded", buf);
    } else if (q <= LocQuality::GOOD &&
               old_q >= LocQuality::POOR /* was worse */) {
      char buf[64];
      snprintf(buf, sizeof(buf), "Score=%.0f", total);
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_LOCALIZATION_LOST,
          robot::v1::EVENT_SEVERITY_INFO,
          "Localization recovered", buf);
    }
  }
}

}  // namespace core
}  // namespace remote_monitoring
