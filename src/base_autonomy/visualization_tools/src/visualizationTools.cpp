#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

/**
 * @class VisualizationTools
 * @brief Real-time exploration visualization and metrics recording
 * 
 * This node provides:
 * - Trajectory visualization and recording
 * - Explored area/volume calculation
 * - Performance metrics (distance, volume, time)
 * - Optional data persistence to files
 */
class VisualizationTools : public rclcpp::Node
{
public:
  VisualizationTools() : Node("visualizationTools")
  {
    // --- Parameter Declaration ---
    declare_parameter<string>("metricFile", metricFile_);
    declare_parameter<string>("trajFile", trajFile_);
    declare_parameter<string>("pcdFile", pcdFile_);
    declare_parameter<string>("mapFile", mapFile_);
    declare_parameter<double>("overallMapVoxelSize", overallMapVoxelSize_);
    declare_parameter<double>("exploredAreaVoxelSize", exploredAreaVoxelSize_);
    declare_parameter<double>("exploredVolumeVoxelSize", exploredVolumeVoxelSize_);
    declare_parameter<double>("transInterval", transInterval_);
    declare_parameter<double>("yawInterval", yawInterval_);
    declare_parameter<int>("overallMapDisplayInterval", overallMapDisplayInterval_);
    declare_parameter<int>("exploredAreaDisplayInterval", exploredAreaDisplayInterval_);
    declare_parameter<bool>("saveMetric", saveMetric_);
    declare_parameter<bool>("saveTraj", saveTraj_);
    declare_parameter<bool>("savePcd", savePcd_);

    // --- Get Parameters ---
    metricFile_ = get_parameter("metricFile").as_string();
    trajFile_ = get_parameter("trajFile").as_string();
    pcdFile_ = get_parameter("pcdFile").as_string();
    mapFile_ = get_parameter("mapFile").as_string();
    overallMapVoxelSize_ = get_parameter("overallMapVoxelSize").as_double();
    exploredAreaVoxelSize_ = get_parameter("exploredAreaVoxelSize").as_double();
    exploredVolumeVoxelSize_ = get_parameter("exploredVolumeVoxelSize").as_double();
    transInterval_ = get_parameter("transInterval").as_double();
    yawInterval_ = get_parameter("yawInterval").as_double();
    overallMapDisplayInterval_ = get_parameter("overallMapDisplayInterval").as_int();
    exploredAreaDisplayInterval_ = get_parameter("exploredAreaDisplayInterval").as_int();
    saveMetric_ = get_parameter("saveMetric").as_bool();
    saveTraj_ = get_parameter("saveTraj").as_bool();
    savePcd_ = get_parameter("savePcd").as_bool();

    // Path replacements (ROS2 workaround)
    fixFilePaths();

    // --- ROS Interfaces ---
    subOdometry_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 5, std::bind(&VisualizationTools::odometryHandler, this, std::placeholders::_1));

    subLaserCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 5, std::bind(&VisualizationTools::laserCloudHandler, this, std::placeholders::_1));

    subRuntime_ = create_subscription<std_msgs::msg::Float32>(
        "/runtime", 5, std::bind(&VisualizationTools::runtimeHandler, this, std::placeholders::_1));

    pubOverallMap_ = create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 5);
    pubExploredArea_ = create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);
    pubTrajectory_ = create_publisher<sensor_msgs::msg::PointCloud2>("/trajectory", 5);
    pubExploredVolume_ = create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);
    pubTravelingDis_ = create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);
    pubTimeDuration_ = create_publisher<std_msgs::msg::Float32>("/time_duration", 5);

    // --- Initialization ---
    laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    overallMapCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    overallMapCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    exploredAreaCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    exploredAreaCloud2_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    exploredVolumeCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    exploredVolumeCloud2_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    trajectory_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    overallMapDwzFilter_.setLeafSize(overallMapVoxelSize_, overallMapVoxelSize_, overallMapVoxelSize_);
    exploredAreaDwzFilter_.setLeafSize(exploredAreaVoxelSize_, exploredAreaVoxelSize_, exploredAreaVoxelSize_);
    exploredVolumeDwzFilter_.setLeafSize(exploredVolumeVoxelSize_, exploredVolumeVoxelSize_, exploredVolumeVoxelSize_);

    // Load map
    loadOverallMap();

    // Open data files if needed
    openDataFiles();

    // Timer for periodic map publishing (100Hz check, actual publish based on interval)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&VisualizationTools::timerCallback, this));
  }

  ~VisualizationTools()
  {
    closeDataFiles();
    RCLCPP_INFO(get_logger(), "Exploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.");
  }

private:
  // --- Parameters ---
  string metricFile_;
  string trajFile_;
  string pcdFile_;
  string mapFile_;
  double overallMapVoxelSize_ = 0.5;
  double exploredAreaVoxelSize_ = 0.3;
  double exploredVolumeVoxelSize_ = 0.5;
  double transInterval_ = 0.2;
  double yawInterval_ = 10.0;
  int overallMapDisplayInterval_ = 2;
  int exploredAreaDisplayInterval_ = 1;
  bool saveMetric_ = false;
  bool saveTraj_ = false;
  bool savePcd_ = false;

  // --- State Variables ---
  int overallMapDisplayCount_ = 0;
  int exploredAreaDisplayCount_ = 0;
  int systemDelay_ = 5;
  int systemDelayCount_ = 0;
  bool systemDelayInited_ = false;
  double systemTime_ = 0;
  double systemInitTime_ = 0;
  bool systemInited_ = false;

  float vehicleYaw_ = 0;
  float vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;
  float exploredVolume_ = 0, travelingDis_ = 0, runtime_ = 0, timeDuration_ = 0;

  // Point Clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory_;

  // Filters
  pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter_;
  pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter_;
  pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter_;

  sensor_msgs::msg::PointCloud2 overallMap2_;

  // File streams (RAII)
  ofstream metricFileStream_;
  ofstream trajFileStream_;
  ofstream pcdFileStream_;

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subRuntime_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOverallMap_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExploredArea_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrajectory_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubExploredVolume_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTravelingDis_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTimeDuration_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Helper Functions ---
  void fixFilePaths()
  {
    if (mapFile_.find("/install/") != string::npos)
      mapFile_.replace(mapFile_.find("/install/"), 9, "/src/base_autonomy");
    if (metricFile_.find("/install/") != string::npos)
      metricFile_.replace(metricFile_.find("/install/"), 9, "/src/base_autonomy");
    if (trajFile_.find("/install/") != string::npos)
      trajFile_.replace(trajFile_.find("/install/"), 9, "/src/base_autonomy");
    if (pcdFile_.find("/install/") != string::npos)
      pcdFile_.replace(pcdFile_.find("/install/"), 9, "/src/base_autonomy");
  }

  void loadOverallMap()
  {
    pcl::PLYReader ply_reader;
    if (ply_reader.read(mapFile_, *overallMapCloud_) == -1) {
      RCLCPP_WARN(get_logger(), "Couldn't read pointcloud.ply file: %s", mapFile_.c_str());
      return;
    }

    overallMapCloudDwz_->clear();
    overallMapDwzFilter_.setInputCloud(overallMapCloud_);
    overallMapDwzFilter_.filter(*overallMapCloudDwz_);
    overallMapCloud_->clear();

    pcl::toROSMsg(*overallMapCloudDwz_, overallMap2_);
    RCLCPP_INFO(get_logger(), "Loaded overall map: %zu points", overallMapCloudDwz_->points.size());
  }

  void openDataFiles()
  {
    time_t logTime = time(0);
    tm *ltm = localtime(&logTime);
    string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + 
                        to_string(ltm->tm_mday) + "-" + to_string(ltm->tm_hour) + "-" + 
                        to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

    if (saveMetric_) {
      string metricFilePath = metricFile_ + "_" + timeString + ".txt";
      metricFileStream_.open(metricFilePath);
      if (!metricFileStream_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open metric file: %s", metricFilePath.c_str());
      }
    }

    if (saveTraj_) {
      string trajFilePath = trajFile_ + "_" + timeString + ".txt";
      trajFileStream_.open(trajFilePath);
      if (!trajFileStream_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open trajectory file: %s", trajFilePath.c_str());
      }
    }

    if (savePcd_) {
      string pcdFilePath = pcdFile_ + "_" + timeString + ".txt";
      pcdFileStream_.open(pcdFilePath);
      if (!pcdFileStream_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open PCD file: %s", pcdFilePath.c_str());
      }
    }
  }

  void closeDataFiles()
  {
    if (metricFileStream_.is_open()) metricFileStream_.close();
    if (trajFileStream_.is_open()) trajFileStream_.close();
    if (pcdFileStream_.is_open()) pcdFileStream_.close();
  }

  // --- Callbacks ---
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    systemTime_ = rclcpp::Time(odom->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    float dYaw = fabs(yaw - vehicleYaw_);
    if (dYaw > PI) dYaw = 2 * PI - dYaw;

    float dx = odom->pose.pose.position.x - vehicleX_;
    float dy = odom->pose.pose.position.y - vehicleY_;
    float dz = odom->pose.pose.position.z - vehicleZ_;
    float dis = sqrt(dx * dx + dy * dy + dz * dz);
    
    if (!systemDelayInited_) {
      vehicleYaw_ = yaw;
      vehicleX_ = odom->pose.pose.position.x;
      vehicleY_ = odom->pose.pose.position.y;
      vehicleZ_ = odom->pose.pose.position.z;
      return;
    }

    if (systemInited_) {
      timeDuration_ = systemTime_ - systemInitTime_;
      
      std_msgs::msg::Float32 timeDurationMsg;
      timeDurationMsg.data = timeDuration_;
      pubTimeDuration_->publish(timeDurationMsg);
    }

    if (dis < transInterval_ && dYaw < yawInterval_) {
      return;
    }

    if (!systemInited_) {
      dis = 0;
      systemInitTime_ = systemTime_;
      systemInited_ = true;
    }

    travelingDis_ += dis;

    vehicleYaw_ = yaw;
    vehicleX_ = odom->pose.pose.position.x;
    vehicleY_ = odom->pose.pose.position.y;
    vehicleZ_ = odom->pose.pose.position.z;

    if (saveTraj_ && trajFileStream_.is_open()) {
      trajFileStream_ << vehicleX_ << " " << vehicleY_ << " " << vehicleZ_ << " "
                      << roll << " " << pitch << " " << yaw << " " << timeDuration_ << "\n";
      trajFileStream_.flush();
    }

    pcl::PointXYZI point;
    point.x = vehicleX_;
    point.y = vehicleY_;
    point.z = vehicleZ_;
    point.intensity = travelingDis_;
    trajectory_->push_back(point);

    sensor_msgs::msg::PointCloud2 trajectory2;
    pcl::toROSMsg(*trajectory_, trajectory2);
    trajectory2.header.stamp = odom->header.stamp;
    trajectory2.header.frame_id = "map";
    pubTrajectory_->publish(trajectory2);
  }

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudIn)
  {
    if (!systemDelayInited_) {
      systemDelayCount_++;
      if (systemDelayCount_ > systemDelay_) {
        systemDelayInited_ = true;
      }
    }

    if (!systemInited_) {
      return;
    }

    laserCloud_->clear();
    pcl::fromROSMsg(*laserCloudIn, *laserCloud_);

    if (savePcd_ && pcdFileStream_.is_open()) {
      float timeDuration2 = rclcpp::Time(laserCloudIn->header.stamp).seconds() - systemInitTime_;
      int laserCloudSize = laserCloud_->points.size();
      for (int i = 0; i < laserCloudSize; i++) {
        pcdFileStream_ << laserCloud_->points[i].x << " " << laserCloud_->points[i].y << " "
                       << laserCloud_->points[i].z << " " << laserCloud_->points[i].intensity << " "
                       << timeDuration2 << "\n";
      }
      pcdFileStream_.flush();
    }

    *exploredVolumeCloud_ += *laserCloud_;

    exploredVolumeCloud2_->clear();
    exploredVolumeDwzFilter_.setInputCloud(exploredVolumeCloud_);
    exploredVolumeDwzFilter_.filter(*exploredVolumeCloud2_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud_;
    exploredVolumeCloud_ = exploredVolumeCloud2_;
    exploredVolumeCloud2_ = tempCloud;

    exploredVolume_ = exploredVolumeVoxelSize_ * exploredVolumeVoxelSize_ * 
                     exploredVolumeVoxelSize_ * exploredVolumeCloud_->points.size();

    *exploredAreaCloud_ += *laserCloud_;

    exploredAreaDisplayCount_++;
    if (exploredAreaDisplayCount_ >= 10 * exploredAreaDisplayInterval_) {
      exploredAreaCloud2_->clear();
      exploredAreaDwzFilter_.setInputCloud(exploredAreaCloud_);
      exploredAreaDwzFilter_.filter(*exploredAreaCloud2_);

      tempCloud = exploredAreaCloud_;
      exploredAreaCloud_ = exploredAreaCloud2_;
      exploredAreaCloud2_ = tempCloud;

      sensor_msgs::msg::PointCloud2 exploredArea2;
      pcl::toROSMsg(*exploredAreaCloud_, exploredArea2);
      exploredArea2.header.stamp = laserCloudIn->header.stamp;
      exploredArea2.header.frame_id = "map";
      pubExploredArea_->publish(exploredArea2);

      exploredAreaDisplayCount_ = 0;
    }

    if (saveMetric_ && metricFileStream_.is_open()) {
      metricFileStream_ << exploredVolume_ << " " << travelingDis_ << " " << runtime_ << " " << timeDuration_ << "\n";
      metricFileStream_.flush();
    }

    std_msgs::msg::Float32 exploredVolumeMsg;
    exploredVolumeMsg.data = exploredVolume_;
    pubExploredVolume_->publish(exploredVolumeMsg);

    std_msgs::msg::Float32 travelingDisMsg;
    travelingDisMsg.data = travelingDis_;
    pubTravelingDis_->publish(travelingDisMsg);
  }

  void runtimeHandler(const std_msgs::msg::Float32::ConstSharedPtr runtimeIn)
  {
    runtime_ = runtimeIn->data;
  }

  void timerCallback()
  {
    overallMapDisplayCount_++;
    if (overallMapDisplayCount_ >= 100 * overallMapDisplayInterval_) {
      if (overallMapCloudDwz_->points.size() > 0) {
        overallMap2_.header.stamp = rclcpp::Time(static_cast<uint64_t>(systemTime_ * 1e9));
        overallMap2_.header.frame_id = "map";
        pubOverallMap_->publish(overallMap2_);
      }

      overallMapDisplayCount_ = 0;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizationTools>());
  rclcpp::shutdown();
  return 0;
}
