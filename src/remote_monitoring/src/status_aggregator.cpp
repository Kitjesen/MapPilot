#include "remote_monitoring/status_aggregator.hpp"
#include "remote_monitoring/core/flight_recorder.hpp"
#include "remote_monitoring/core/geofence_monitor.hpp"
#include "remote_monitoring/core/health_monitor.hpp"
#include "remote_monitoring/core/localization_scorer.hpp"

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
  node_->declare_parameter<std::string>("odom_topic", "/nav/odometry");
  node_->declare_parameter<std::string>("terrain_map_topic", "/nav/terrain_map");
  node_->declare_parameter<std::string>("path_topic", "/nav/local_path");
  node_->declare_parameter<std::string>("slow_down_topic", "/nav/slow_down");
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

  // RobotState: 关节角度 + 电池 + IMU
  node_->declare_parameter<std::string>("robot_state_topic", "/robot_state");
  const auto robot_state_topic = node_->get_parameter("robot_state_topic").as_string();
  sub_robot_state_ = node_->create_subscription<interface::msg::RobotState>(
    robot_state_topic, 10,
    std::bind(&StatusAggregator::RobotStateCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(),
              "Subscribed to RobotState on '%s' for joint angles + battery",
              robot_state_topic.c_str());

  // 模拟电池初始化
  battery_sim_start_ = std::chrono::steady_clock::now();

  // 后台线程: /proc 系统资源采样 (避免在 ROS executor 上做阻塞 I/O)
  sys_resource_thread_ = std::thread([this]() { UpdateSysResourcesBackground(); });

  rate_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(window_sec_), [this]() { update_rates(); });
  fast_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(fast_hz_, 0.1)), [this]() { update_fast_state(); });
  slow_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(slow_hz_, 0.1)), [this]() { update_slow_state(); });
}

StatusAggregator::~StatusAggregator() {
  sys_resource_stop_.store(true, std::memory_order_relaxed);
  if (sys_resource_thread_.joinable()) {
    sys_resource_thread_.join();
  }
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

void StatusAggregator::RobotStateCallback(
    const interface::msg::RobotState::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  latest_robot_state_ = msg;
  // 有真实 RobotState 数据时，不再使用模拟电池
  if (use_simulated_battery_) {
    use_simulated_battery_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "RobotState received, switching to real battery data");
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
    // 非阻塞: timeout=0 避免在 ROS 执行器线程上阻塞 (之前 0.1s 会卡死执行器)
    return tf_buffer_.canTransform(target, source, tf2::TimePointZero,
                                   tf2::durationFromSec(0.0));
  } catch (...) {
    return false;
  }
}

std::shared_ptr<const robot::v1::FastState> StatusAggregator::GetFastState() {
  std::lock_guard<std::mutex> lock(fast_mutex_);
  return latest_fast_state_;  // shared_ptr 拷贝 = 原子引用计数，几乎免费
}

std::shared_ptr<const robot::v1::SlowState> StatusAggregator::GetSlowState() {
  std::lock_guard<std::mutex> lock(slow_mutex_);
  return latest_slow_state_;
}

void StatusAggregator::update_fast_state() {
  // 复用 scratch protobuf 内部缓冲区 — Clear() 不释放已分配的子消息内存
  scratch_fast_state_.Clear();
  
  // 设置 header
  auto *header = scratch_fast_state_.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  // 从 latest_odom_ 提取位姿和速度
  if (latest_odom_) {
    auto *pose = scratch_fast_state_.mutable_pose();
    pose->mutable_position()->set_x(latest_odom_->pose.pose.position.x);
    pose->mutable_position()->set_y(latest_odom_->pose.pose.position.y);
    pose->mutable_position()->set_z(latest_odom_->pose.pose.position.z);
    pose->mutable_orientation()->set_x(latest_odom_->pose.pose.orientation.x);
    pose->mutable_orientation()->set_y(latest_odom_->pose.pose.orientation.y);
    pose->mutable_orientation()->set_z(latest_odom_->pose.pose.orientation.z);
    pose->mutable_orientation()->set_w(latest_odom_->pose.pose.orientation.w);
    
    auto *vel = scratch_fast_state_.mutable_velocity();
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
    scratch_fast_state_.mutable_rpy_deg()->set_x(roll * 180.0 / M_PI);
    scratch_fast_state_.mutable_rpy_deg()->set_y(pitch * 180.0 / M_PI);
    scratch_fast_state_.mutable_rpy_deg()->set_z(yaw * 180.0 / M_PI);
  }
  
  // TF 状态
  const bool tf_ok = check_tf(tf_map_frame_, tf_odom_frame_) &&
                     check_tf(tf_odom_frame_, tf_body_frame_);
  scratch_fast_state_.set_tf_ok(tf_ok);
  cached_tf_ok_.store(tf_ok, std::memory_order_relaxed);

  // 从 RobotState 提取：关节角度 + IMU (加速度 / 角速度)
  {
    std::lock_guard<std::mutex> rs_lock(robot_state_mutex_);
    if (latest_robot_state_) {
      for (int leg = 0; leg < 4; ++leg) {
        scratch_fast_state_.add_joint_angles(latest_robot_state_->joint_positions[leg * 3 + 0]);
        scratch_fast_state_.add_joint_angles(latest_robot_state_->joint_positions[leg * 3 + 1]);
        scratch_fast_state_.add_joint_angles(latest_robot_state_->joint_positions[leg * 3 + 2]);
        scratch_fast_state_.add_joint_angles(0.0f);
      }

      scratch_fast_state_.mutable_linear_acceleration()->set_x(
          static_cast<double>(latest_robot_state_->imu_accelerometer[0]));
      scratch_fast_state_.mutable_linear_acceleration()->set_y(
          static_cast<double>(latest_robot_state_->imu_accelerometer[1]));
      scratch_fast_state_.mutable_linear_acceleration()->set_z(
          static_cast<double>(latest_robot_state_->imu_accelerometer[2]));

      scratch_fast_state_.mutable_angular_velocity()->set_x(
          static_cast<double>(latest_robot_state_->imu_gyroscope[0]));
      scratch_fast_state_.mutable_angular_velocity()->set_y(
          static_cast<double>(latest_robot_state_->imu_gyroscope[1]));
      scratch_fast_state_.mutable_angular_velocity()->set_z(
          static_cast<double>(latest_robot_state_->imu_gyroscope[2]));
    }
  }

  // 定位健康评分 (由 LocalizationScorer 以 10Hz 计算)
  if (localization_scorer_) {
    scratch_fast_state_.set_localization_score(localization_scorer_->GetScore());
    scratch_fast_state_.set_speed_scale(localization_scorer_->GetSpeedScale());
  }

  // O(1) Swap 替代深拷贝 — 将 scratch 转为 shared_ptr<const> 发布给所有客户端
  auto new_state = std::make_shared<robot::v1::FastState>();
  new_state->Swap(&scratch_fast_state_);
  {
    std::lock_guard<std::mutex> lock(fast_mutex_);
    latest_fast_state_ = std::move(new_state);
  }

  // 黑盒录制: 将当前帧推入 FlightRecorder 环形缓冲
  if (flight_recorder_) {
    core::FlightSnapshot snap{};
    snap.timestamp_sec = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count()) / 1e6;

    if (latest_odom_) {
      snap.x = static_cast<float>(latest_odom_->pose.pose.position.x);
      snap.y = static_cast<float>(latest_odom_->pose.pose.position.y);
      snap.z = static_cast<float>(latest_odom_->pose.pose.position.z);
      snap.vx = static_cast<float>(latest_odom_->twist.twist.linear.x);
      snap.vy = static_cast<float>(latest_odom_->twist.twist.linear.y);
      snap.wz = static_cast<float>(latest_odom_->twist.twist.angular.z);
    }

    // RPY 来自已计算的 new_state (swap 后读取 latest_fast_state_)
    {
      std::lock_guard<std::mutex> fsl(fast_mutex_);
      if (latest_fast_state_ && latest_fast_state_->has_rpy_deg()) {
        snap.roll  = static_cast<float>(latest_fast_state_->rpy_deg().x());
        snap.pitch = static_cast<float>(latest_fast_state_->rpy_deg().y());
        snap.yaw   = static_cast<float>(latest_fast_state_->rpy_deg().z());
      }
    }

    if (localization_scorer_) {
      snap.loc_health_score = localization_scorer_->GetScore();
      snap.icp_fitness = -1.0f;  // 由 scorer 内部管理
    }
    // 电池: 复用 SlowState 的逻辑 (真实 > 模拟)
    {
      std::lock_guard<std::mutex> rs_lock(robot_state_mutex_);
      if (!use_simulated_battery_ && latest_robot_state_) {
        snap.battery_percent =
            static_cast<float>(latest_robot_state_->battery.percentage);
      } else {
        snap.battery_percent = static_cast<float>(simulated_battery_);
      }
    }
    snap.cpu_temp =
        static_cast<float>(cached_cpu_temp_.load(std::memory_order_relaxed));
    snap.tf_ok = tf_ok ? 1 : 0;
    const auto current_mode =
        mode_provider_ ? mode_provider_() : robot::v1::ROBOT_MODE_IDLE;
    snap.mode = static_cast<uint8_t>(current_mode);
    snap.estop =
        (current_mode == robot::v1::ROBOT_MODE_ESTOP) ? uint8_t{1} : uint8_t{0};
    if (health_monitor_) {
      snap.health_level =
          static_cast<uint8_t>(health_monitor_->GetOverallLevel());
    }

    flight_recorder_->RecordSnapshot(snap);
  }
}

void StatusAggregator::update_slow_state() {
  scratch_slow_state_.Clear();
  
  auto *header = scratch_slow_state_.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  // 从 ControlService 获取真实模式
  if (mode_provider_) {
    const auto mode = mode_provider_();
    switch (mode) {
      case robot::v1::ROBOT_MODE_IDLE:       scratch_slow_state_.set_current_mode("idle"); break;
      case robot::v1::ROBOT_MODE_MANUAL:     scratch_slow_state_.set_current_mode("manual"); break;
      case robot::v1::ROBOT_MODE_TELEOP:     scratch_slow_state_.set_current_mode("teleop"); break;
      case robot::v1::ROBOT_MODE_AUTONOMOUS: scratch_slow_state_.set_current_mode("autonomous"); break;
      case robot::v1::ROBOT_MODE_MAPPING:    scratch_slow_state_.set_current_mode("mapping"); break;
      case robot::v1::ROBOT_MODE_ESTOP:      scratch_slow_state_.set_current_mode("estop"); break;
      default:                               scratch_slow_state_.set_current_mode("unknown"); break;
    }
  } else {
    scratch_slow_state_.set_current_mode("idle");
  }

  // 系统资源 — 从后台线程缓存读取, 不阻塞 executor
  auto *resources = scratch_slow_state_.mutable_resources();
  resources->set_cpu_percent(cached_cpu_percent_.load(std::memory_order_relaxed));
  resources->set_mem_percent(cached_mem_percent_.load(std::memory_order_relaxed));
  resources->set_cpu_temp(cached_cpu_temp_.load(std::memory_order_relaxed));

  // 电池电量：优先使用 RobotState.battery，否则使用模拟值
  {
    std::lock_guard<std::mutex> rs_lock(robot_state_mutex_);
    if (!use_simulated_battery_ && latest_robot_state_) {
      // 真实电池数据 from interface::msg::BatteryState
      resources->set_battery_percent(
          static_cast<double>(latest_robot_state_->battery.percentage));
      resources->set_battery_voltage(
          static_cast<double>(latest_robot_state_->battery.voltage));
    } else {
      // 模拟电池：从 85% 以 0.01%/s 匀速下降
      const auto elapsed =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - battery_sim_start_)
              .count();
      simulated_battery_ = std::max(0.0, 85.0 - elapsed * 0.01);
      resources->set_battery_percent(simulated_battery_);
      resources->set_battery_voltage(24.0 + simulated_battery_ * 0.04);
      // 每 60 秒打一次日志提醒
      static int sim_log_counter = 0;
      if (sim_log_counter++ % 60 == 0) {
        RCLCPP_WARN(node_->get_logger(),
                    "[SIMULATED] Battery: %.1f%% (no RobotState received, "
                    "using simulated drain)",
                    simulated_battery_);
      }
    }
  }
  
  // 话题频率
  auto *rates = scratch_slow_state_.mutable_topic_rates();
  rates->set_odom_hz(odom_rate_.rate_hz());
  rates->set_terrain_map_hz(terrain_rate_.rate_hz());
  rates->set_path_hz(path_rate_.rate_hz());
  rates->set_lidar_hz(lidar_rate_.rate_hz());

  // 健康状态
  if (health_monitor_) {
    auto robot_health = health_monitor_->GetHealth();
    auto *health_status = scratch_slow_state_.mutable_health();

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
    auto *geofence = scratch_slow_state_.mutable_geofence();
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
  
  // 运行参数 (JSON)
  if (config_provider_) {
    auto [config_json, version] = config_provider_();
    if (!config_json.empty()) {
      scratch_slow_state_.set_config_json(std::move(config_json));
      scratch_slow_state_.set_config_version(version);
    }
  }

  auto new_state = std::make_shared<robot::v1::SlowState>();
  new_state->Swap(&scratch_slow_state_);
  {
    std::lock_guard<std::mutex> lock(slow_mutex_);
    latest_slow_state_ = std::move(new_state);
  }
}

// ==================== 系统资源读取（Linux /proc） ====================
//
// 这些方法在后台线程中以 ~1Hz 调用, 结果写入 atomic 缓存。
// ROS executor 线程只读取 atomic, 不做文件 I/O。

void StatusAggregator::UpdateSysResourcesBackground() {
  while (!sys_resource_stop_.load(std::memory_order_relaxed)) {
    cached_cpu_percent_.store(ReadCpuUsage(), std::memory_order_relaxed);
    cached_mem_percent_.store(ReadMemUsage(), std::memory_order_relaxed);
    cached_cpu_temp_.store(ReadCpuTemp(), std::memory_order_relaxed);

    // 1 秒采样周期, 可中断退出
    for (int i = 0; i < 10 && !sys_resource_stop_.load(std::memory_order_relaxed); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

double StatusAggregator::ReadCpuUsage() {
  std::ifstream file("/proc/stat");
  if (!file.is_open()) return 0.0;

  std::string label;
  uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
  file >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  if (label != "cpu") return 0.0;

  const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
  const uint64_t idle_total = idle + iowait;

  if (prev_cpu_total_ == 0) {
    prev_cpu_total_ = total;
    prev_cpu_idle_ = idle_total;
    return 0.0;
  }

  const uint64_t d_total = total - prev_cpu_total_;
  const uint64_t d_idle = idle_total - prev_cpu_idle_;
  prev_cpu_total_ = total;
  prev_cpu_idle_ = idle_total;

  if (d_total == 0) return 0.0;
  return 100.0 * static_cast<double>(d_total - d_idle) / static_cast<double>(d_total);
}

double StatusAggregator::ReadMemUsage() {
  std::ifstream file("/proc/meminfo");
  if (!file.is_open()) return 0.0;

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
    if (mem_total > 0 && mem_available > 0) break;
  }

  if (mem_total == 0) return 0.0;
  return 100.0 * static_cast<double>(mem_total - mem_available)
       / static_cast<double>(mem_total);
}

double StatusAggregator::ReadCpuTemp() {
  std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
  if (!file.is_open()) return 0.0;

  int temp_milli = 0;
  file >> temp_milli;
  return static_cast<double>(temp_milli) / 1000.0;
}

}  // namespace remote_monitoring
