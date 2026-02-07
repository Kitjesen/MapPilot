#include "remote_monitoring/core/geofence_monitor.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace remote_monitoring {
namespace core {

GeofenceMonitor::GeofenceMonitor(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<double>("geofence_warn_margin_m", 3.0);
  node_->declare_parameter<double>("geofence_stop_margin_m", 0.5);
  node_->declare_parameter<double>("geofence_check_hz", 20.0);
  node_->declare_parameter<std::string>("geofence_odom_topic", "/Odometry");
  node_->declare_parameter<std::string>("geofence_boundary_topic",
                                        "/navigation_boundary");
  node_->declare_parameter<std::string>("geofence_stop_topic", "/stop");

  margin_warn_m_ = node_->get_parameter("geofence_warn_margin_m").as_double();
  margin_stop_m_ = node_->get_parameter("geofence_stop_margin_m").as_double();
  check_hz_ = node_->get_parameter("geofence_check_hz").as_double();

  const auto odom_topic =
      node_->get_parameter("geofence_odom_topic").as_string();
  const auto boundary_topic =
      node_->get_parameter("geofence_boundary_topic").as_string();
  const auto stop_topic =
      node_->get_parameter("geofence_stop_topic").as_string();

  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&GeofenceMonitor::OdomCallback, this, std::placeholders::_1));

  sub_boundary_ =
      node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
          boundary_topic, 5,
          std::bind(&GeofenceMonitor::BoundaryCallback, this,
                    std::placeholders::_1));

  pub_stop_ = node_->create_publisher<std_msgs::msg::Int8>(stop_topic, 5);
  pub_status_ =
      node_->create_publisher<std_msgs::msg::String>("/geofence/status", 5);

  // 独立检查定时器
  check_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(check_hz_, 1.0)),
      std::bind(&GeofenceMonitor::CheckLoop, this));

  RCLCPP_INFO(node_->get_logger(),
              "GeofenceMonitor initialized (warn=%.1fm, stop=%.1fm, "
              "check=%.0fHz)",
              margin_warn_m_, margin_stop_m_, check_hz_);
}

void GeofenceMonitor::OdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void GeofenceMonitor::BoundaryCallback(
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
  if (msg->polygon.points.size() < 3) {
    RCLCPP_WARN(node_->get_logger(),
                "GeofenceMonitor: Boundary polygon has < 3 points, ignoring");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(fence_mutex_);
    polygon_ = std::vector<geometry_msgs::msg::Point32>(
        msg->polygon.points.begin(), msg->polygon.points.end());
  }
  has_fence_.store(true);
  RCLCPP_INFO(node_->get_logger(),
              "GeofenceMonitor: Boundary set with %zu vertices",
              msg->polygon.points.size());
}

void GeofenceMonitor::CheckLoop() {
  if (!has_fence_.load() || !odom_received_) {
    state_.store(has_fence_.load() ? GeofenceState::SAFE
                                   : GeofenceState::NO_FENCE);
    return;
  }

  double px, py;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    px = robot_x_;
    py = robot_y_;
  }

  // 点-多边形距离 (正值=在内部, 负值=越界)
  bool inside = PointInPolygon(px, py);
  double dist = PointToPolygonDistance(px, py);
  double signed_dist = inside ? dist : -dist;
  margin_distance_.store(signed_dist);

  // 三级判定
  GeofenceState new_state;
  if (!inside) {
    new_state = GeofenceState::VIOLATION;
  } else if (dist < margin_stop_m_) {
    new_state = GeofenceState::VIOLATION;  // 太近也视为越界
  } else if (dist < margin_warn_m_) {
    new_state = GeofenceState::WARNING;
  } else {
    new_state = GeofenceState::SAFE;
  }

  state_.store(new_state);

  // 越界 → 发布急停 (独立通道, 不经过 SafetyGate/ModeManager)
  if (new_state == GeofenceState::VIOLATION) {
    std_msgs::msg::Int8 stop_msg;
    stop_msg.data = 2;  // Level 2: 停止线速度 + 角速度
    pub_stop_->publish(stop_msg);
  }

  // 发布状态字符串
  std_msgs::msg::String status_msg;
  std::ostringstream oss;
  const char *state_str = "UNKNOWN";
  switch (new_state) {
  case GeofenceState::NO_FENCE: state_str = "NO_FENCE"; break;
  case GeofenceState::SAFE:     state_str = "SAFE"; break;
  case GeofenceState::WARNING:  state_str = "WARNING"; break;
  case GeofenceState::VIOLATION: state_str = "VIOLATION"; break;
  }
  oss << state_str << "|margin=" << std::fixed
      << std::setprecision(2) << signed_dist << "m";
  status_msg.data = oss.str();
  pub_status_->publish(status_msg);

  // 事件生成 (仅在状态变化时)
  if (event_buffer_ && new_state != last_reported_state_) {
    switch (new_state) {
    case GeofenceState::WARNING:
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SAFETY_COLLISION_WARNING,
          robot::v1::EVENT_SEVERITY_WARNING, "Approaching geofence",
          "Distance to boundary: " + std::to_string(dist) + "m");
      break;
    case GeofenceState::VIOLATION:
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SAFETY_ESTOP,
          robot::v1::EVENT_SEVERITY_CRITICAL, "Geofence violation",
          "Robot is outside boundary or too close (margin=" +
              std::to_string(signed_dist) + "m). Emergency stop triggered.");
      break;
    case GeofenceState::SAFE:
      if (last_reported_state_ == GeofenceState::VIOLATION ||
          last_reported_state_ == GeofenceState::WARNING) {
        event_buffer_->AddEvent(
            robot::v1::EVENT_TYPE_SAFETY_COLLISION_WARNING,
            robot::v1::EVENT_SEVERITY_INFO, "Geofence: back in safe zone",
            "Distance to boundary: " + std::to_string(dist) + "m");
      }
      break;
    default:
      break;
    }
    last_reported_state_ = new_state;
  }
}

// ================================================================
//  几何计算 — 射线法 + 点到多边形距离
// ================================================================

bool GeofenceMonitor::PointInPolygon(double px, double py) const {
  // 射线法 (Ray Casting): O(N)
  std::lock_guard<std::mutex> lock(fence_mutex_);
  const auto &poly = polygon_;
  const int n = static_cast<int>(poly.size());
  if (n < 3)
    return false;

  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

double GeofenceMonitor::PointToPolygonDistance(double px, double py) const {
  std::lock_guard<std::mutex> lock(fence_mutex_);
  const auto &poly = polygon_;
  const int n = static_cast<int>(poly.size());
  if (n < 3)
    return 1e9;

  double min_dist = 1e9;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double d = PointToSegmentDistance(px, py, poly[j].x, poly[j].y, poly[i].x,
                                     poly[i].y);
    if (d < min_dist)
      min_dist = d;
  }
  return min_dist;
}

double GeofenceMonitor::PointToSegmentDistance(double px, double py, double ax,
                                               double ay, double bx,
                                               double by) const {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len_sq = dx * dx + dy * dy;
  if (len_sq < 1e-12) {
    // 退化线段 (两端点重合)
    return std::sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
  }
  // 参数化投影 t ∈ [0, 1]
  double t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
  t = std::max(0.0, std::min(1.0, t));
  const double proj_x = ax + t * dx;
  const double proj_y = ay + t * dy;
  return std::sqrt((px - proj_x) * (px - proj_x) +
                   (py - proj_y) * (py - proj_y));
}

}  // namespace core
}  // namespace remote_monitoring
