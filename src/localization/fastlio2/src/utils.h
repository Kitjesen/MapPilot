#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

enum LID_TYPE { AVIA = 1, VELO16 = 2, OUST64 = 3 };
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

class Utils
{
public:
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int lidar_type, int filter_num, int scan_line, int time_unit, double min_range = 0.5, double max_range = 20.0);
    static builtin_interfaces::msg::Time getTime(const double& sec);
};