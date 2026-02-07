#include "remote_monitoring/status_aggregator.hpp"
#include "remote_monitoring/core/geofence_monitor.hpp"
#include "remote_monitoring/core/health_monitor.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace remote_monitoring {

void TopicRate::tick() { count_++; }

void TopicRate::reset(double window_sec) {
  if (window_sec > 0.0) {
    rate_hz_ = static_cast<float>(count_ / window_sec);
  } else {
    rate_hz_ = 0.0f;
  }
  count_ = 0;
}

float TopicRate::rate_hz() const { return rate_hz_; }

StatusAggregator::StatusAggregator(rclcpp::Node *node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_) {

  node_->declare_parameter<double>("fast_state_hz", 30.0);
  node_->declare_parameter<double>("slow_state_hz", 1.0);
  node_->declare_parameter<double>("rate_window_sec", 2.0);
  node_->declare_parameter<std::string>("odom_topic", "/Odometry");
  node_->declare_parameter<std::string>("terrain_map_topic", "/terrain_map");
  node_->declare_parameter<std::string>("path_topic", "/path");
  node_->declare_parameter<std::string>("slow_down_topic", "/slow_down");
  node_->declare_parameter<std::string>("tf_map_frame", "map");
  node_->declare_parameter<std::string>("tf_odom_frame", "odom");
  node_->declare_parameter<std::string>("tf_body_frame", "body");

  fast_hz_ = node_->get_parameter("fast_state_hz").as_double();
  slow_hz_ = node_->get_parameter("slow_state_hz").as_double();
  window_sec_ = node_->get_parameter("rate_window_sec").as_double();
  odom_topic_ = node_->get_parameter("odom_topic").as_string();
  terrain_map_topic_ = node_->get_parameter("terrain_map_topic").as_string();
  path_topic_ = node_->get_parameter("path_topic").as_string();
  tf_map_frame_ = node_->get_parameter("tf_map_frame").as_string();
  tf_odom_frame_ = node_->get_parameter("tf_odom_frame").as_string();
  tf_body_frame_ = node_->get_parameter("tf_body_frame").as_string();

  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&StatusAggregator::OdomCallback, this, std::placeholders::_1));
  sub_terrain_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_map_topic_, 5, std::bind(&StatusAggregator::TerrainCallback, this, std::placeholders::_1));
  sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 5, std::bind(&StatusAggregator::PathCallback, this, std::placeholders::_1));
  sub_slow_down_ = node_->create_subscription<std_msgs::msg::Int8>(
    node_->get_parameter("slow_down_topic").as_string(), 5,
    std::bind(&StatusAggregator::SlowDownCallback, this, std::placeholders::_1));

  rate_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(window_sec_), [this]() { update_rates(); });
  fast_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(fast_hz_, 0.1)), [this]() { update_fast_state(); });
  slow_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(slow_hz_, 0.1)), [this]() { update_slow_state(); });
}

void StatusAggregator::SetModeProvider(ModeProvider provider) {
  mode_provider_ = std::move(provider);
}

void StatusAggregator::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  odom_rate_.tick();
  latest_odom_ = msg;
}

void StatusAggregator::TerrainCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
  terrain_rate_.tick();
}

void StatusAggregator::PathCallback(const nav_msgs::msg::Path::ConstSharedPtr) {
  path_rate_.tick();
}

void StatusAggregator::SlowDownCallback(const std_msgs::msg::Int8::ConstSharedPtr msg) {
  if (msg) {
    slow_down_level_.store(msg->data);
  }
}

void StatusAggregator::update_rates() {
  odom_rate_.reset(window_sec_);
  terrain_rate_.reset(window_sec_);
  path_rate_.reset(window_sec_);
  lidar_rate_.reset(window_sec_);
}

bool StatusAggregator::check_tf(const std::string &target, const std::string &source) {
  try {
    return tf_buffer_.canTransform(target, source, tf2::TimePointZero,
                                   tf2::durationFromSec(0.1));
  } catch (...) {
    return false;
  }
}

robot::v1::FastState StatusAggregator::GetFastState() {
  std::lock_guard<std::mutex> lock(fast_mutex_);
  return latest_fast_state_;
}

robot::v1::SlowState StatusAggregator::GetSlowState() {
  std::lock_guard<std::mutex> lock(slow_mutex_);
  return latest_slow_state_;
}

void StatusAggregator::update_fast_state() {
  robot::v1::FastState state;
  
  // 设置 header
  auto *header = state.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  // 从 latest_odom_ 提取位姿和速度
  if (latest_odom_) {
    auto *pose = state.mutable_pose();
    pose->mutable_position()->set_x(latest_odom_->pose.pose.position.x);
    pose->mutable_position()->set_y(latest_odom_->pose.pose.position.y);
    pose->mutable_position()->set_z(latest_odom_->pose.pose.position.z);
    pose->mutable_orientation()->set_x(latest_odom_->pose.pose.orientation.x);
    pose->mutable_orientation()->set_y(latest_odom_->pose.pose.orientation.y);
    pose->mutable_orientation()->set_z(latest_odom_->pose.pose.orientation.z);
    pose->mutable_orientation()->set_w(latest_odom_->pose.pose.orientation.w);
    
    auto *vel = state.mutable_velocity();
    vel->mutable_linear()->set_x(latest_odom_->twist.twist.linear.x);
    vel->mutable_linear()->set_y(latest_odom_->twist.twist.linear.y);
    vel->mutable_linear()->set_z(latest_odom_->twist.twist.linear.z);
    vel->mutable_angular()->set_x(latest_odom_->twist.twist.angular.x);
    vel->mutable_angular()->set_y(latest_odom_->twist.twist.angular.y);
    vel->mutable_angular()->set_z(latest_odom_->twist.twist.angular.z);
    
    // 计算 RPY
    tf2::Quaternion q(latest_odom_->pose.pose.orientation.x,
                      latest_odom_->pose.pose.orientation.y,
                      latest_odom_->pose.pose.orientation.z,
                      latest_odom_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state.mutable_rpy_deg()->set_x(roll * 180.0 / M_PI);
    state.mutable_rpy_deg()->set_y(pitch * 180.0 / M_PI);
    state.mutable_rpy_deg()->set_z(yaw * 180.0 / M_PI);
  }
  
  // TF 状态
  const bool tf_ok = check_tf(tf_map_frame_, tf_odom_frame_) &&
                     check_tf(tf_odom_frame_, tf_body_frame_);
  state.set_tf_ok(tf_ok);
  
  std::lock_guard<std::mutex> lock(fast_mutex_);
  latest_fast_state_ = state;
}

void StatusAggregator::update_slow_state() {
  robot::v1::SlowState state;
  
  auto *header = state.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  // 从 ControlService 获取真实模式
  if (mode_provider_) {
    const auto mode = mode_provider_();
    switch (mode) {
      case robot::v1::ROBOT_MODE_IDLE:       state.set_current_mode("idle"); break;
      case robot::v1::ROBOT_MODE_MANUAL:     state.set_current_mode("manual"); break;
      case robot::v1::ROBOT_MODE_TELEOP:     state.set_current_mode("teleop"); break;
      case robot::v1::ROBOT_MODE_AUTONOMOUS: state.set_current_mode("autonomous"); break;
      case robot::v1::ROBOT_MODE_MAPPING:    state.set_current_mode("mapping"); break;
      case robot::v1::ROBOT_MODE_ESTOP:      state.set_current_mode("estop"); break;
      default:                               state.set_current_mode("unknown"); break;
    }
  } else {
    state.set_current_mode("idle");
  }

  // 系统资源
  auto *resources = state.mutable_resources();
  resources->set_cpu_percent(ReadCpuUsage());
  resources->set_mem_percent(ReadMemUsage());
  resources->set_cpu_temp(ReadCpuTemp());
  // battery_percent / battery_voltage 需要外部话题或驱动，暂留 0
  
  // 话题频率
  auto *rates = state.mutable_topic_rates();
  rates->set_odom_hz(odom_rate_.rate_hz());
  rates->set_terrain_map_hz(terrain_rate_.rate_hz());
  rates->set_path_hz(path_rate_.rate_hz());
  rates->set_lidar_hz(lidar_rate_.rate_hz());

  // 健康状态
  if (health_monitor_) {
    auto robot_health = health_monitor_->GetHealth();
    auto *health_status = state.mutable_health();

    const char *level_names[] = {"OK", "DEGRADED", "CRITICAL", "FAULT"};
    health_status->set_overall_level(
        level_names[static_cast<int>(robot_health.overall)]);

    for (const auto &sub : robot_health.subsystems) {
      auto *proto_sub = health_status->add_subsystems();
      proto_sub->set_name(sub.name);
      proto_sub->set_level(level_names[static_cast<int>(sub.level)]);
      proto_sub->set_message(sub.message);
      proto_sub->set_expected_hz(sub.expected_hz);
      proto_sub->set_actual_hz(sub.actual_hz);
    }
  }

  // 围栏状态
  if (geofence_monitor_) {
    auto *geofence = state.mutable_geofence();
    geofence->set_has_fence(geofence_monitor_->HasFence());
    geofence->set_margin_distance(geofence_monitor_->GetMarginDistance());

    switch (geofence_monitor_->GetState()) {
    case core::GeofenceState::NO_FENCE:
      geofence->set_state("NO_FENCE");
      break;
    case core::GeofenceState::SAFE:
      geofence->set_state("SAFE");
      break;
    case core::GeofenceState::WARNING:
      geofence->set_state("WARNING");
      break;
    case core::GeofenceState::VIOLATION:
      geofence->set_state("VIOLATION");
      break;
    }
  }
  
  std::lock_guard<std::mutex> lock(slow_mutex_);
  latest_slow_state_ = state;
}

// ==================== 系统资源读取（Linux /proc） ====================

double StatusAggregator::ReadCpuUsage() {
  // 读取 /proc/stat 计算整体 CPU 使用率
  // 格式: cpu user nice system idle iowait irq softirq steal guest guest_nice
  std::ifstream file("/proc/stat");
  if (!file.is_open()) {
    return 0.0;
  }

  std::string label;
  uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
  file >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  if (label != "cpu") {
    return 0.0;
  }

  const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
  const uint64_t idle_total = idle + iowait;

  // 首次调用无法计算差值，返回 0
  // 注意：这是一个简化实现，static 变量在多实例时有问题，
  // 但本系统只有一个 StatusAggregator 实例。
  static uint64_t s_prev_total = 0;
  static uint64_t s_prev_idle = 0;

  if (s_prev_total == 0) {
    s_prev_total = total;
    s_prev_idle = idle_total;
    return 0.0;
  }

  const uint64_t d_total = total - s_prev_total;
  const uint64_t d_idle = idle_total - s_prev_idle;
  s_prev_total = total;
  s_prev_idle = idle_total;

  if (d_total == 0) {
    return 0.0;
  }

  return 100.0 * static_cast<double>(d_total - d_idle) / static_cast<double>(d_total);
}

double StatusAggregator::ReadMemUsage() {
  // 读取 /proc/meminfo
  std::ifstream file("/proc/meminfo");
  if (!file.is_open()) {
    return 0.0;
  }

  uint64_t mem_total = 0, mem_available = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (line.rfind("MemTotal:", 0) == 0) {
      std::istringstream iss(line);
      std::string key;
      iss >> key >> mem_total;
    } else if (line.rfind("MemAvailable:", 0) == 0) {
      std::istringstream iss(line);
      std::string key;
      iss >> key >> mem_available;
    }
    if (mem_total > 0 && mem_available > 0) {
      break;
    }
  }

  if (mem_total == 0) {
    return 0.0;
  }

  return 100.0 * static_cast<double>(mem_total - mem_available)
       / static_cast<double>(mem_total);
}

double StatusAggregator::ReadCpuTemp() {
  // 读取 /sys/class/thermal/thermal_zone0/temp（毫摄氏度）
  std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
  if (!file.is_open()) {
    return 0.0;
  }

  int temp_milli = 0;
  file >> temp_milli;
  return static_cast<double>(temp_milli) / 1000.0;
}

}  // namespace remote_monitoring
