#include "remote_monitoring/core/health_monitor.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <algorithm>
#include <sstream>

namespace remote_monitoring {
namespace core {

HealthMonitor::HealthMonitor(rclcpp::Node *node)
    : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_) {

  node_->declare_parameter<double>("health_rate_window_sec", 2.0);
  node_->declare_parameter<double>("health_eval_hz", 5.0);
  node_->declare_parameter<std::string>("health_odom_topic", "/Odometry");
  node_->declare_parameter<std::string>("health_terrain_topic", "/terrain_map");
  node_->declare_parameter<std::string>("health_path_topic", "/path");
  node_->declare_parameter<std::string>("health_tf_map", "map");
  node_->declare_parameter<std::string>("health_tf_odom", "odom");
  node_->declare_parameter<std::string>("health_tf_body", "body");

  rate_window_sec_ = node_->get_parameter("health_rate_window_sec").as_double();
  eval_hz_ = node_->get_parameter("health_eval_hz").as_double();
  tf_map_frame_ = node_->get_parameter("health_tf_map").as_string();
  tf_odom_frame_ = node_->get_parameter("health_tf_odom").as_string();
  tf_body_frame_ = node_->get_parameter("health_tf_body").as_string();

  const auto odom_topic = node_->get_parameter("health_odom_topic").as_string();
  const auto terrain_topic =
      node_->get_parameter("health_terrain_topic").as_string();
  const auto path_topic = node_->get_parameter("health_path_topic").as_string();

  // 订阅 — 只做频率统计
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&HealthMonitor::OdomTick, this, std::placeholders::_1));
  sub_terrain_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      terrain_topic, 5,
      std::bind(&HealthMonitor::TerrainTick, this, std::placeholders::_1));
  sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      path_topic, 5,
      std::bind(&HealthMonitor::PathTick, this, std::placeholders::_1));

  // 定位质量 (ICP fitness score)
  sub_localization_quality_ =
      node_->create_subscription<std_msgs::msg::Float32>(
          "/localization_quality", 10,
          std::bind(&HealthMonitor::LocalizationQualityCallback, this,
                    std::placeholders::_1));

  // 发布
  pub_health_ =
      node_->create_publisher<std_msgs::msg::String>("/robot_health", 5);
  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>("/stop", 5);

  // 频率重置定时器
  rate_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(rate_window_sec_), [this]() {
        odom_rate_.reset(rate_window_sec_);
        terrain_rate_.reset(rate_window_sec_);
        path_rate_.reset(rate_window_sec_);
      });

  // 健康评估定时器
  eval_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(eval_hz_, 0.1)),
      std::bind(&HealthMonitor::EvaluateLoop, this));

  RCLCPP_INFO(node_->get_logger(),
              "HealthMonitor initialized (eval=%.0fHz, window=%.1fs)",
              eval_hz_, rate_window_sec_);
}

RobotHealth HealthMonitor::GetHealth() const {
  std::lock_guard<std::mutex> lock(health_mutex_);
  return latest_health_;
}

bool HealthMonitor::CheckTf(const std::string &target,
                            const std::string &source) {
  try {
    return tf_buffer_.canTransform(target, source, tf2::TimePointZero,
                                   tf2::durationFromSec(0.1));
  } catch (...) {
    return false;
  }
}

void HealthMonitor::EvaluateLoop() {
  RobotHealth health;

  // ---- SLAM (里程计频率) ----
  {
    SubsystemHealth sub;
    sub.name = "slam";
    sub.expected_hz = 100.0;
    sub.actual_hz = odom_rate_.hz.load();
    if (sub.actual_hz > 50.0) {
      sub.level = HealthLevel::OK;
      sub.message = "SLAM running normally";
    } else if (sub.actual_hz > 20.0) {
      sub.level = HealthLevel::DEGRADED;
      sub.message = "SLAM rate low: " + std::to_string(sub.actual_hz) + " Hz";
    } else {
      sub.level = HealthLevel::FAULT;
      sub.message =
          "SLAM rate critical: " + std::to_string(sub.actual_hz) + " Hz";
    }
    health.subsystems.push_back(sub);
  }

  // ---- 地形分析 ----
  {
    SubsystemHealth sub;
    sub.name = "terrain_analysis";
    sub.expected_hz = 5.0;
    sub.actual_hz = terrain_rate_.hz.load();
    if (sub.actual_hz > 3.0) {
      sub.level = HealthLevel::OK;
      sub.message = "Terrain analysis running";
    } else if (sub.actual_hz > 1.0) {
      sub.level = HealthLevel::DEGRADED;
      sub.message =
          "Terrain analysis slow: " + std::to_string(sub.actual_hz) + " Hz";
    } else {
      sub.level = HealthLevel::CRITICAL;
      sub.message =
          "Terrain analysis stalled: " + std::to_string(sub.actual_hz) + " Hz";
    }
    health.subsystems.push_back(sub);
  }

  // ---- 局部规划 ----
  {
    SubsystemHealth sub;
    sub.name = "local_planner";
    sub.expected_hz = 5.0;
    sub.actual_hz = path_rate_.hz.load();
    if (sub.actual_hz > 3.0) {
      sub.level = HealthLevel::OK;
      sub.message = "Local planner running";
    } else if (sub.actual_hz > 1.0) {
      sub.level = HealthLevel::DEGRADED;
      sub.message =
          "Local planner slow: " + std::to_string(sub.actual_hz) + " Hz";
    } else {
      sub.level = HealthLevel::CRITICAL;
      sub.message =
          "Local planner stalled: " + std::to_string(sub.actual_hz) + " Hz";
    }
    health.subsystems.push_back(sub);
  }

  // ---- 定位质量 (ICP fitness score) ----
  {
    SubsystemHealth sub;
    sub.name = "localization";
    sub.expected_hz = 0;  // 非频率指标
    const float score = localization_fitness_score_.load();
    if (score < 0.0f) {
      // 尚未收到数据 — 不影响整体判定
      sub.level = HealthLevel::OK;
      sub.message = "Localization quality not available yet";
    } else if (score < 0.1f) {
      sub.level = HealthLevel::OK;
      sub.message = "Localization good (score=" + std::to_string(score) + ")";
    } else if (score < 0.3f) {
      sub.level = HealthLevel::DEGRADED;
      sub.message =
          "Localization degraded (score=" + std::to_string(score) + ")";
    } else {
      sub.level = HealthLevel::CRITICAL;
      sub.message =
          "Localization poor (score=" + std::to_string(score) + ")";
    }
    health.subsystems.push_back(sub);
  }

  // ---- TF 链完整性 ----
  {
    SubsystemHealth sub;
    sub.name = "tf_chain";
    sub.expected_hz = 0;  // 非频率指标
    bool tf_map_odom = CheckTf(tf_map_frame_, tf_odom_frame_);
    bool tf_odom_body = CheckTf(tf_odom_frame_, tf_body_frame_);
    if (tf_map_odom && tf_odom_body) {
      sub.level = HealthLevel::OK;
      sub.message = "TF chain complete";
    } else if (tf_odom_body) {
      sub.level = HealthLevel::DEGRADED;
      sub.message = "map→odom TF missing (localization may be down)";
    } else {
      sub.level = HealthLevel::FAULT;
      sub.message = "TF chain broken";
    }
    health.subsystems.push_back(sub);
  }

  // ---- 总体判定: 取最差的子系统 ----
  health.overall = HealthLevel::OK;
  for (const auto &sub : health.subsystems) {
    if (static_cast<int>(sub.level) > static_cast<int>(health.overall)) {
      health.overall = sub.level;
    }
  }

  overall_level_.store(health.overall);

  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    latest_health_ = health;
  }

  // ---- 发布健康状态 (ROS2 话题, 供外部设备订阅) ----
  std_msgs::msg::String msg;
  std::ostringstream oss;
  const char *level_str[] = {"OK", "DEGRADED", "CRITICAL", "FAULT"};
  oss << level_str[static_cast<int>(health.overall)];
  for (const auto &sub : health.subsystems) {
    oss << "|" << sub.name << "=" << level_str[static_cast<int>(sub.level)];
  }
  msg.data = oss.str();
  pub_health_->publish(msg);

  // ---- 自动降级联动 ----
  if (health.overall != last_reported_level_) {
    // 事件生成
    if (event_buffer_) {
      robot::v1::EventSeverity sev;
      switch (health.overall) {
      case HealthLevel::OK:
        sev = robot::v1::EVENT_SEVERITY_INFO;
        break;
      case HealthLevel::DEGRADED:
        sev = robot::v1::EVENT_SEVERITY_WARNING;
        break;
      case HealthLevel::CRITICAL:
        sev = robot::v1::EVENT_SEVERITY_ERROR;
        break;
      case HealthLevel::FAULT:
        sev = robot::v1::EVENT_SEVERITY_CRITICAL;
        break;
      }

      std::string desc;
      for (const auto &sub : health.subsystems) {
        if (sub.level != HealthLevel::OK) {
          desc += sub.name + ": " + sub.message + "; ";
        }
      }
      event_buffer_->AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT, sev,
                              "Health: " + std::string(level_str[static_cast<int>(health.overall)]),
                              desc.empty() ? "All subsystems OK" : desc);
    }

    // FAULT → 双路径停车 (冗余保证)
    if (health.overall == HealthLevel::FAULT) {
      // 路径 A: 直接发布 /stop (不依赖任何上层模块)
      std_msgs::msg::Int8 stop_msg;
      stop_msg.data = 2;  // Level 2: 全停
      pub_stop_->publish(stop_msg);
    }

    // 路径 B: 回调通知 ModeManager (best-effort, 可能失败)
    if (degrade_callback_) {
      std::string reason;
      for (const auto &sub : health.subsystems) {
        if (sub.level == health.overall) {
          reason = sub.name + ": " + sub.message;
          break;
        }
      }
      degrade_callback_(health.overall, reason);
    }

    last_reported_level_ = health.overall;
  }
}

}  // namespace core
}  // namespace remote_monitoring
