#include <cmath>
#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
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

    declare_parameter<double>("waypoint_distance", 0.5);
    declare_parameter<double>("arrival_threshold", 0.5);
    declare_parameter<double>("lookahead_dist", 1.0);

    waypoint_distance_ = get_parameter("waypoint_distance").as_double();
    arrival_threshold_ = get_parameter("arrival_threshold").as_double();
    lookahead_dist_ = get_parameter("lookahead_dist").as_double();

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
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received new Global Path with %zu points",
      msg->poses.size());

    current_path_ = downsample_path(*msg);
    current_waypoint_idx_ = 0;
    path_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Path processed into %zu waypoints",
      current_path_.size());
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
      const double dist = std::hypot(dx, dy);
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

    const auto & target_pose_map = current_path_[current_waypoint_idx_].pose.position;
    geometry_msgs::msg::Point target_pose_odom;
    if (!transform_point(target_pose_map, target_pose_odom)) {
      return;
    }

    const double dist_to_target = get_distance(robot_pos_, target_pose_odom);
    if (dist_to_target < arrival_threshold_) {
      if (current_waypoint_idx_ < current_path_.size() - 1) {
        RCLCPP_INFO(
          get_logger(),
          "Reached Waypoint %zu. Proceeding to next.",
          current_waypoint_idx_);
        current_waypoint_idx_++;
        return;
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Goal Reached!");
      }
    }

    geometry_msgs::msg::PointStamped waypoint_msg;
    waypoint_msg.header.stamp = now();
    waypoint_msg.header.frame_id = odom_frame_;
    waypoint_msg.point = target_pose_odom;
    waypoint_pub_->publish(waypoint_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  size_t current_waypoint_idx_;
  geometry_msgs::msg::Point robot_pos_;
  bool path_received_;
  bool robot_pos_received_;
  std::string odom_frame_;

  double waypoint_distance_;
  double arrival_threshold_;
  double lookahead_dist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCTPathAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
