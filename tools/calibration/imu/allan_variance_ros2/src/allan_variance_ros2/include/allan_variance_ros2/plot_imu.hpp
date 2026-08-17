// ROS2
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp> 
#include "sensor_msgs/msg/imu.hpp"
#include <rerun.hpp>

void plot_imu(std::string bag_path, std::string imu_topic);
