#include <cmath>
#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PCTPathAdapter : public rclcpp::Node
{
public:
  PCTPathAdapter()
  : Node("pct_path_adapter"),
    current_waypoint_idx_(0),
    path_received_(false),
    robot_pos_received_(false)
  {
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/pct_path",
      10,
      std::bind(&PCTPathAdapter::path_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry",
      10,
      std::bind(&PCTPathAdapter::odom_callback, this, _1));

    // 发布到 /planner_waypoint, 由 TaskManager 统一转发到 /way_point
    // 避免与 App 下发的航点冲突
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/planner_waypoint",
      10);

    // 航点到达事件 → TaskManager / App 可订阅进度
    // 格式: {"event": "waypoint_reached"|"goal_reached"|"path_received",
    //         "index": N, "total": M}
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/nav/planner_status",
      10);

    declare_parameter<double>("waypoint_distance", 0.5);
    declare_parameter<double>("arrival_threshold", 0.5);

    waypoint_distance_ = get_parameter("waypoint_distance").as_double();
    arrival_threshold_ = get_parameter("arrival_threshold").as_double();

    declare_parameter<double>("stuck_timeout_sec", 10.0);
    stuck_timeout_sec_ = get_parameter("stuck_timeout_sec").as_double();
    last_progress_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // ── 第1级重规划: stuck 时自动重发 goal_pose 触发 PCT 重新规划 ──
    declare_parameter<int>("max_replan_count", 2);
    declare_parameter<double>("replan_cooldown_sec", 5.0);
    max_replan_count_ = get_parameter("max_replan_count").as_int();
    replan_cooldown_sec_ = get_parameter("replan_cooldown_sec").as_double();
    replan_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/nav/goal_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(100ms, std::bind(&PCTPathAdapter::control_loop, this));

    RCLCPP_INFO(get_logger(), "PCT Path Adapter initialized");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_pos_ = msg->pose.pose.position;
    odom_frame_ = msg->header.frame_id;
    robot_pos_received_ = true;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      // 空路径 = 规划失败信号
      if (path_received_) {
        RCLCPP_WARN(get_logger(),
          "Received empty path (planning failed). "
          "Keeping stuck detection active (replan_count=%d/%d).",
          replan_count_, max_replan_count_);
        // 不清除 current_path_ 和 path_received_:
        // 保留旧路径目标供重规划使用，stuck 检测继续运行
        // replan_count_ 不重置，让系统继续升级到 stuck_final
      }
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received new Global Path with %zu points",
      msg->poses.size());

    current_path_ = downsample_path(*msg);
    current_waypoint_idx_ = 0;
    path_received_ = true;
    goal_reached_reported_ = false;
    last_progress_time_ = now();
    replan_count_ = 0;  // 新路径 → 重置重规划计数

    RCLCPP_INFO(
      get_logger(),
      "Path processed into %zu waypoints",
      current_path_.size());
    publish_status("path_received", 0, current_path_.size());
  }

  std::vector<geometry_msgs::msg::PoseStamped> downsample_path(
    const nav_msgs::msg::Path & path) const
  {
    if (path.poses.empty()) {
      return {};
    }

    std::vector<geometry_msgs::msg::PoseStamped> downsampled;
    downsampled.reserve(path.poses.size());
    downsampled.push_back(path.poses.front());

    auto last_pose = path.poses.front();
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i];
      const double dx = pose.pose.position.x - last_pose.pose.position.x;
      const double dy = pose.pose.position.y - last_pose.pose.position.y;
      const double dz = pose.pose.position.z - last_pose.pose.position.z;
      // 使用 3D 距离: 坡度路径上 2D 距离会低估实际路长，导致航点稀疏
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist >= waypoint_distance_) {
        downsampled.push_back(pose);
        last_pose = pose;
      }
    }

    downsampled.push_back(path.poses.back());
    return downsampled;
  }

  static double get_distance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
  {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
  }

  bool transform_point(
    const geometry_msgs::msg::Point & point_in_map,
    geometry_msgs::msg::Point & point_out)
  {
    if (odom_frame_.empty()) {
      return false;
    }

    geometry_msgs::msg::PointStamped input;
    input.header.frame_id = "map";
    input.header.stamp = rclcpp::Time(0);
    input.point = point_in_map;

    geometry_msgs::msg::PointStamped output;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        odom_frame_,
        "map",
        tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Could not transform waypoint: %s",
        ex.what());
      return false;
    }

    point_out = output.point;
    return true;
  }

  void control_loop()
  {
    if (!path_received_ || current_path_.empty() || !robot_pos_received_ || odom_frame_.empty()) {
      return;
    }

    // ── Stuck detection + 两级自动重规划 ──
    if (!goal_reached_reported_ && last_progress_time_.nanoseconds() > 0) {
      const double elapsed = (now() - last_progress_time_).seconds();
      if (elapsed > stuck_timeout_sec_) {
        const double now_sec = now().seconds();
        if (replan_count_ < max_replan_count_) {
          // 第1级: 几何层自动重规划 — 重发 goal_pose 让 PCT Planner 从当前位置重新规划
          if (now_sec - last_replan_time_ > replan_cooldown_sec_) {
            replan_count_++;
            last_replan_time_ = now_sec;

            auto goal_msg = geometry_msgs::msg::PoseStamped();
            goal_msg.header.stamp = now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose = current_path_.back().pose;
            replan_goal_pub_->publish(goal_msg);

            last_progress_time_ = now();  // 重置 stuck 计时

            RCLCPP_WARN(get_logger(),
              "[Replan L1] 几何层重规划 %d/%d, 从当前位置重新规划到目标",
              replan_count_, max_replan_count_);
            publish_status("replanning",
              static_cast<int>(current_waypoint_idx_),
              static_cast<int>(current_path_.size()));
          }
        } else {
          // 第2级: 几何层重规划耗尽 → 发布 stuck_final 让语义层接管
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Replan] 几何层重规划 %d 次仍失败, 上报 stuck_final",
            replan_count_);
          publish_status("stuck_final",
            static_cast<int>(current_waypoint_idx_),
            static_cast<int>(current_path_.size()));
        }
      }
    }

    // ── 航点跳跃: 限窗搜索最近航点 (防止 U 形路径跳过整段) ──
    // 正常跟踪: 只搜前方 5 个航点 (防止折叠路径跳到对岸)
    // 恢复/重规划后: 搜索全部 (机器人可能被移动到远处)
    const size_t total = current_path_.size();
    {
      const size_t search_window = 5;  // 正常模式下的搜索窗口
      const size_t search_end = std::min(
          current_waypoint_idx_ + search_window, total);
      size_t best_idx = current_waypoint_idx_;
      double best_dist = std::numeric_limits<double>::max();
      for (size_t i = current_waypoint_idx_; i < search_end; ++i) {
        geometry_msgs::msg::Point pt_odom;
        if (!transform_point(current_path_[i].pose.position, pt_odom)) continue;
        const double d = get_distance(robot_pos_, pt_odom);
        if (d < best_dist) {
          best_dist = d;
          best_idx = i;
        }
      }
      if (best_idx > current_waypoint_idx_) {
        RCLCPP_INFO(get_logger(),
          "Skipping waypoints %zu → %zu (closer after recovery/replan)",
          current_waypoint_idx_, best_idx);
        current_waypoint_idx_ = best_idx;
        last_progress_time_ = now();
      }
    }

    const auto & target_pose_map = current_path_[current_waypoint_idx_].pose.position;
    geometry_msgs::msg::Point target_pose_odom;
    if (!transform_point(target_pose_map, target_pose_odom)) {
      return;
    }

    const double dist_to_target = get_distance(robot_pos_, target_pose_odom);
    if (dist_to_target < arrival_threshold_) {
      if (current_waypoint_idx_ < total - 1) {
        RCLCPP_INFO(
          get_logger(),
          "Reached Waypoint %zu. Proceeding to next.",
          current_waypoint_idx_);
        publish_status("waypoint_reached", static_cast<int>(current_waypoint_idx_),
                       static_cast<int>(total));
        last_progress_time_ = now();
        current_waypoint_idx_++;
        return;
      } else {
        if (!goal_reached_reported_) {
          RCLCPP_INFO(get_logger(), "Goal Reached!");
          publish_status("goal_reached", static_cast<int>(current_waypoint_idx_),
                         static_cast<int>(total));
          goal_reached_reported_ = true;
        } else {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Goal Reached!");
        }
      }
    }

    // Don't keep sending waypoint after goal is reached — local planner should settle
    if (goal_reached_reported_) {
      return;
    }

    geometry_msgs::msg::PointStamped waypoint_msg;
    waypoint_msg.header.stamp = now();
    waypoint_msg.header.frame_id = odom_frame_;
    waypoint_msg.point = target_pose_odom;
    waypoint_pub_->publish(waypoint_msg);
  }

  void publish_status(const std::string & event, int index, int total)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"event\":\"" + event + "\","
               "\"index\":" + std::to_string(index) + ","
               "\"total\":" + std::to_string(total) + "}";
    status_pub_->publish(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  size_t current_waypoint_idx_;
  geometry_msgs::msg::Point robot_pos_;
  bool path_received_;
  bool robot_pos_received_;
  bool goal_reached_reported_{false};
  std::string odom_frame_;

  double waypoint_distance_;
  double arrival_threshold_;
  double stuck_timeout_sec_;
  rclcpp::Time last_progress_time_;

  // ── 第1级重规划 ──
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr replan_goal_pub_;
  int    replan_count_       = 0;
  int    max_replan_count_   = 2;
  double replan_cooldown_sec_ = 5.0;
  double last_replan_time_   = -1.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCTPathAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
