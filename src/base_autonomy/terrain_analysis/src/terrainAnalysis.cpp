/**
 * @file terrainAnalysis.cpp
 * @brief Robot Terrain Analysis Node
 *
 * This node processes registered point clouds to analyze terrain traversability.
 * It maintains a local rolling map, estimates the ground plane, filters dynamic obstacles,
 * and outputs a terrain map containing only valid obstacles and slope information
 * for the local planner.
 *
 * Algorithm Overview:
 * 1. Maintain a rolling voxel grid (3D) around the robot.
 * 2. Stack incoming laser scans into these voxels.
 * 3. Downsample and filter old points.
 * 4. Project 3D voxels onto a 2D planar grid to estimate ground elevation.
 * 5. Compare points against estimated ground to identify obstacles.
 * 6. Filter out dynamic obstacles based on consistency.
 *
 * Refactored from original monolithic script to ROS 2 Node class.
 */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
 * @class TerrainAnalysis
 * @brief Core class for terrain analysis, inheriting from rclcpp::Node.
 */
class TerrainAnalysis : public rclcpp::Node
{
public:
  /**
   * @brief Constructor. Initializes parameters, ROS interfaces, and data structures.
   */
  TerrainAnalysis() : Node("terrainAnalysis")
  {
    // --- Parameter Initialization ---
    // Voxel sizes and decay times
    declare_parameter<double>("scanVoxelSize", 0.05);
    declare_parameter<double>("decayTime", 2.0);
    declare_parameter<double>("noDecayDis", 4.0);
    declare_parameter<double>("clearingDis", 8.0);
    
    // sorting and ground estimation parameters
    declare_parameter<bool>("useSorting", true);
    declare_parameter<double>("quantileZ", 0.25);
    declare_parameter<bool>("considerDrop", false);
    declare_parameter<bool>("limitGroundLift", false);
    declare_parameter<double>("maxGroundLift", 0.15);
    
    // Dynamic obstacle filtering
    declare_parameter<bool>("clearDyObs", false);
    declare_parameter<double>("minDyObsDis", 0.3);
    declare_parameter<double>("absDyObsRelZThre", 0.2);
    declare_parameter<double>("minDyObsVFOV", -16.0);
    declare_parameter<double>("maxDyObsVFOV", 16.0);
    declare_parameter<int>("minDyObsPointNum", 1);
    declare_parameter<int>("minOutOfFovPointNum", 2);
    
    // Obstacle thresholds
    declare_parameter<double>("obstacleHeightThre", 0.2);
    declare_parameter<bool>("noDataObstacle", false);
    declare_parameter<int>("noDataBlockSkipNum", 0);
    declare_parameter<int>("minBlockPointNum", 10);
    declare_parameter<double>("vehicleHeight", 1.5);
    
    // Update thresholds
    declare_parameter<int>("voxelPointUpdateThre", 100);
    declare_parameter<double>("voxelTimeUpdateThre", 2.0);
    declare_parameter<double>("minRelZ", -1.5);
    declare_parameter<double>("maxRelZ", 0.2);
    declare_parameter<double>("disRatioZ", 0.2);

    // Get parameters
    scanVoxelSize_ = get_parameter("scanVoxelSize").as_double();
    decayTime_ = get_parameter("decayTime").as_double();
    noDecayDis_ = get_parameter("noDecayDis").as_double();
    clearingDis_ = get_parameter("clearingDis").as_double();
    useSorting_ = get_parameter("useSorting").as_bool();
    quantileZ_ = get_parameter("quantileZ").as_double();
    considerDrop_ = get_parameter("considerDrop").as_bool();
    limitGroundLift_ = get_parameter("limitGroundLift").as_bool();
    maxGroundLift_ = get_parameter("maxGroundLift").as_double();
    clearDyObs_ = get_parameter("clearDyObs").as_bool();
    minDyObsDis_ = get_parameter("minDyObsDis").as_double();
    absDyObsRelZThre_ = get_parameter("absDyObsRelZThre").as_double();
    minDyObsVFOV_ = get_parameter("minDyObsVFOV").as_double();
    maxDyObsVFOV_ = get_parameter("maxDyObsVFOV").as_double();
    minDyObsPointNum_ = get_parameter("minDyObsPointNum").as_int();
    minOutOfFovPointNum_ = get_parameter("minOutOfFovPointNum").as_int();
    obstacleHeightThre_ = get_parameter("obstacleHeightThre").as_double();
    noDataObstacle_ = get_parameter("noDataObstacle").as_bool();
    noDataBlockSkipNum_ = get_parameter("noDataBlockSkipNum").as_int();
    minBlockPointNum_ = get_parameter("minBlockPointNum").as_int();
    vehicleHeight_ = get_parameter("vehicleHeight").as_double();
    voxelPointUpdateThre_ = get_parameter("voxelPointUpdateThre").as_int();
    voxelTimeUpdateThre_ = get_parameter("voxelTimeUpdateThre").as_double();
    minRelZ_ = get_parameter("minRelZ").as_double();
    maxRelZ_ = get_parameter("maxRelZ").as_double();
    disRatioZ_ = get_parameter("disRatioZ").as_double();

    // --- Dynamic Parameter Callback ---
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &p : params) {
            const auto &n = p.get_name();
            if (n == "obstacleHeightThre") obstacleHeightThre_ = p.as_double();
            else if (n == "useSorting") useSorting_ = p.as_bool();
            else if (n == "quantileZ") quantileZ_ = p.as_double();
            else if (n == "considerDrop") considerDrop_ = p.as_bool();
            else if (n == "limitGroundLift") limitGroundLift_ = p.as_bool();
            else if (n == "maxGroundLift") maxGroundLift_ = p.as_double();
            else if (n == "clearDyObs") clearDyObs_ = p.as_bool();
            else if (n == "minDyObsDis") minDyObsDis_ = p.as_double();
            else if (n == "absDyObsRelZThre") absDyObsRelZThre_ = p.as_double();
            else if (n == "noDataObstacle") noDataObstacle_ = p.as_bool();
            else if (n == "scanVoxelSize") {
              scanVoxelSize_ = p.as_double();
              downSizeFilter_.setLeafSize(scanVoxelSize_, scanVoxelSize_, scanVoxelSize_);
            }
            else if (n == "decayTime") decayTime_ = p.as_double();
            else if (n == "noDecayDis") noDecayDis_ = p.as_double();
            else if (n == "clearingDis") clearingDis_ = p.as_double();
            else if (n == "vehicleHeight") vehicleHeight_ = p.as_double();
            else if (n == "minRelZ") minRelZ_ = p.as_double();
            else if (n == "maxRelZ") maxRelZ_ = p.as_double();
            else if (n == "disRatioZ") disRatioZ_ = p.as_double();
            else {
              RCLCPP_WARN(get_logger(), "Unknown dynamic param: %s", n.c_str());
            }
          }
          return result;
        });

    // --- Data Structure Initialization ---
    // Allocate memory for point clouds
    laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudElev_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Initialize voxel grid arrays
    for (int i = 0; i < terrainVoxelNum_; i++) {
        terrainVoxelCloud_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainVoxelUpdateNum_[i] = 0;
        terrainVoxelUpdateTime_[i] = 0;
    }

    for (int i = 0; i < planarVoxelNum_; i++) {
        planarVoxelElev_[i] = 0;
        planarVoxelEdge_[i] = 0;
        planarVoxelDyObs_[i] = 0;
        planarVoxelOutOfFov_[i] = 0;
    }

    downSizeFilter_.setLeafSize(scanVoxelSize_, scanVoxelSize_, scanVoxelSize_);

    // --- ROS Interface Initialization ---
    // Subscriber: Odometry for vehicle state
    subOdometry_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 5, std::bind(&TerrainAnalysis::odometryHandler, this, std::placeholders::_1));

    // Subscriber: World Map Point Cloud (Input) - in odom coordinate frame
    subLaserCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", 5, std::bind(&TerrainAnalysis::laserCloudHandler, this, std::placeholders::_1));

    // Subscriber: Joystick for manual commands (e.g. reset map)
    subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, std::bind(&TerrainAnalysis::joystickHandler, this, std::placeholders::_1));

    // Subscriber: Map clearing command
    subClearing_ = create_subscription<std_msgs::msg::Float32>(
        "/map_clearing", 5, std::bind(&TerrainAnalysis::clearingHandler, this, std::placeholders::_1));

    // Publisher: Terrain Map (Output)
    pubLaserCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map", 2);

    // Timer: Main processing loop (100Hz approx)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&TerrainAnalysis::processLoop, this));
  }

private:
  /**
   * @brief Updates vehicle state from Odometry topic.
   */
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll_ = roll;
    vehiclePitch_ = pitch;
    vehicleYaw_ = yaw;
    vehicleX_ = odom->pose.pose.position.x;
    vehicleY_ = odom->pose.pose.position.y;
    vehicleZ_ = odom->pose.pose.position.z;

    sinVehicleRoll_ = sin(vehicleRoll_);
    cosVehicleRoll_ = cos(vehicleRoll_);
    sinVehiclePitch_ = sin(vehiclePitch_);
    cosVehiclePitch_ = cos(vehiclePitch_);
    sinVehicleYaw_ = sin(vehicleYaw_);
    cosVehicleYaw_ = cos(vehicleYaw_);

    // 预计算 R = R_yaw * R_pitch * R_roll (ZYX 欧拉序), 供障碍物体坐标变换用
    // 替代逐点三步旋转, 9 次乘法/点 vs 18 次
    vehicleRotBody_[0][0] =  cosVehicleYaw_ * cosVehiclePitch_;
    vehicleRotBody_[0][1] =  cosVehicleYaw_ * sinVehiclePitch_ * sinVehicleRoll_ - sinVehicleYaw_ * cosVehicleRoll_;
    vehicleRotBody_[0][2] =  cosVehicleYaw_ * sinVehiclePitch_ * cosVehicleRoll_ + sinVehicleYaw_ * sinVehicleRoll_;
    vehicleRotBody_[1][0] =  sinVehicleYaw_ * cosVehiclePitch_;
    vehicleRotBody_[1][1] =  sinVehicleYaw_ * sinVehiclePitch_ * sinVehicleRoll_ + cosVehicleYaw_ * cosVehicleRoll_;
    vehicleRotBody_[1][2] =  sinVehicleYaw_ * sinVehiclePitch_ * cosVehicleRoll_ - cosVehicleYaw_ * sinVehicleRoll_;
    vehicleRotBody_[2][0] = -sinVehiclePitch_;
    vehicleRotBody_[2][1] =  cosVehiclePitch_ * sinVehicleRoll_;
    vehicleRotBody_[2][2] =  cosVehiclePitch_ * cosVehicleRoll_;

    // Initialize previous position for distance check if this is the first data
    if (noDataInited_ == 0) {
      vehicleRecX_ = vehicleX_;
      vehicleRecY_ = vehicleY_;
      noDataInited_ = 1;
    }

    // Check if vehicle has moved enough to enable data accumulation
    if (noDataInited_ == 1) {
      float dis = sqrt((vehicleX_ - vehicleRecX_) * (vehicleX_ - vehicleRecX_) +
                       (vehicleY_ - vehicleRecY_) * (vehicleY_ - vehicleRecY_));
      if (dis > minDyObsDis_)
        noDataInited_ = 2; // Enabled
    }
  }

  /**
   * @brief Processes incoming registered point cloud.
   * Filters points based on distance and relationship to vehicle Z.
   */
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
  {
    laserCloudTime_ = rclcpp::Time(laserCloud2->header.stamp).seconds();
    if (!systemInited_) {
      systemInitTime_ = laserCloudTime_;
      systemInited_ = true;
    }

    laserCloud_->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud_);

    pcl::PointXYZI point;
    laserCloudCrop_->clear();
    int laserCloudSize = laserCloud_->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud_->points[i];

      // point is in body frame (from /cloud_registered)
      // Convert to odom frame to match vehicleX_/Y_ used in main loop
      float dx = point.x * cosVehicleYaw_ - point.y * sinVehicleYaw_;
      float dy = point.x * sinVehicleYaw_ + point.y * cosVehicleYaw_;
      float pointX = dx + vehicleX_;
      float pointY = dy + vehicleY_;
      float pointZ = point.z + vehicleZ_;

      // Distance from robot: sqrt(dx² + dy²) = sqrt(point.x² + point.y²)
      float dis = sqrt(point.x * point.x + point.y * point.y);
      
      // Filter points: must be within relevant height range relative to vehicle
      // and within mapping radius
      if (pointZ - vehicleZ_ > minRelZ_ - disRatioZ_ * dis &&
          pointZ - vehicleZ_ < maxRelZ_ + disRatioZ_ * dis &&
          dis < terrainVoxelSize_ * (terrainVoxelHalfWidth_ + 1)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime_ - systemInitTime_; // Store relative time in intensity
        laserCloudCrop_->push_back(point);
      }
    }

    newLaserCloud_ = true; // Signal main loop to process
  }

  /**
   * @brief Handles manual clearing commands from joystick (Button 5).
   */
  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    if (joy->buttons[5] > 0.5) {
      noDataInited_ = 0;
      clearingCloud_ = true;
    }
  }

  /**
   * @brief Handles programmed clearing commands.
   */
  void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
  {
    noDataInited_ = 0;
    clearingDis_ = dis->data;
    clearingCloud_ = true;
  }

  /**
   * @brief Main processing loop.
   * Executes the full terrain analysis pipeline: shifting map, stacking scans,
   * estimating ground, and publishing.
   */
  void processLoop()
  {
    if (!newLaserCloud_) return;

    newLaserCloud_ = false;

    // --- 1. Map Rolling / Shifting ---
    // Shift the local map voxels as the vehicle moves to keep vehicle centered.
    float terrainVoxelCenX = terrainVoxelSize_ * terrainVoxelShiftX_;
    float terrainVoxelCenY = terrainVoxelSize_ * terrainVoxelShiftY_;

    // Shift X directions
    while (vehicleX_ - terrainVoxelCenX < -terrainVoxelSize_) {
      for (int indY = 0; indY < terrainVoxelWidth_; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud_[terrainVoxelWidth_ * (terrainVoxelWidth_ - 1) + indY];
        for (int indX = terrainVoxelWidth_ - 1; indX >= 1; indX--) {
          terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY] =
              terrainVoxelCloud_[terrainVoxelWidth_ * (indX - 1) + indY];
        }
        terrainVoxelCloud_[indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud_[indY]->clear();
      }
      terrainVoxelShiftX_--;
      terrainVoxelCenX = terrainVoxelSize_ * terrainVoxelShiftX_;
    }

    while (vehicleX_ - terrainVoxelCenX > terrainVoxelSize_) {
      for (int indY = 0; indY < terrainVoxelWidth_; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud_[indY];
        for (int indX = 0; indX < terrainVoxelWidth_ - 1; indX++) {
          terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY] =
              terrainVoxelCloud_[terrainVoxelWidth_ * (indX + 1) + indY];
        }
        terrainVoxelCloud_[terrainVoxelWidth_ * (terrainVoxelWidth_ - 1) + indY] = terrainVoxelCloudPtr;
        terrainVoxelCloud_[terrainVoxelWidth_ * (terrainVoxelWidth_ - 1) + indY]->clear();
      }
      terrainVoxelShiftX_++;
      terrainVoxelCenX = terrainVoxelSize_ * terrainVoxelShiftX_;
    }

    // Shift Y directions
    while (vehicleY_ - terrainVoxelCenY < -terrainVoxelSize_) {
      for (int indX = 0; indX < terrainVoxelWidth_; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud_[terrainVoxelWidth_ * indX + (terrainVoxelWidth_ - 1)];
        for (int indY = terrainVoxelWidth_ - 1; indY >= 1; indY--) {
          terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY] =
              terrainVoxelCloud_[terrainVoxelWidth_ * indX + (indY - 1)];
        }
        terrainVoxelCloud_[terrainVoxelWidth_ * indX] = terrainVoxelCloudPtr;
        terrainVoxelCloud_[terrainVoxelWidth_ * indX]->clear();
      }
      terrainVoxelShiftY_--;
      terrainVoxelCenY = terrainVoxelSize_ * terrainVoxelShiftY_;
    }

    while (vehicleY_ - terrainVoxelCenY > terrainVoxelSize_) {
      for (int indX = 0; indX < terrainVoxelWidth_; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud_[terrainVoxelWidth_ * indX];
        for (int indY = 0; indY < terrainVoxelWidth_ - 1; indY++) {
          terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY] =
              terrainVoxelCloud_[terrainVoxelWidth_ * indX + (indY + 1)];
        }
        terrainVoxelCloud_[terrainVoxelWidth_ * indX + (terrainVoxelWidth_ - 1)] = terrainVoxelCloudPtr;
        terrainVoxelCloud_[terrainVoxelWidth_ * indX + (terrainVoxelWidth_ - 1)]->clear();
      }
      terrainVoxelShiftY_++;
      terrainVoxelCenY = terrainVoxelSize_ * terrainVoxelShiftY_;
    }

    // --- 2. Stack New Scans ---
    // Add points from the current scan to the corresponding voxels
    pcl::PointXYZI point;
    int laserCloudCropSize = laserCloudCrop_->points.size();
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laserCloudCrop_->points[i];

      int indX = int((point.x - vehicleX_ + terrainVoxelSize_ / 2) / terrainVoxelSize_) + terrainVoxelHalfWidth_;
      int indY = int((point.y - vehicleY_ + terrainVoxelSize_ / 2) / terrainVoxelSize_) + terrainVoxelHalfWidth_;

      if (point.x - vehicleX_ + terrainVoxelSize_ / 2 < 0) indX--;
      if (point.y - vehicleY_ + terrainVoxelSize_ / 2 < 0) indY--;

      if (indX >= 0 && indX < terrainVoxelWidth_ && indY >= 0 && indY < terrainVoxelWidth_) {
        terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY]->push_back(point);
        terrainVoxelUpdateNum_[terrainVoxelWidth_ * indX + indY]++;
      }
    }

    // --- 3. Filter Voxels ---
    // Downsample voxels if they have too many points or are too old
    for (int ind = 0; ind < terrainVoxelNum_; ind++) {
      if (terrainVoxelUpdateNum_[ind] >= voxelPointUpdateThre_ ||
          laserCloudTime_ - systemInitTime_ - terrainVoxelUpdateTime_[ind] >= voxelTimeUpdateThre_ ||
          clearingCloud_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud_[ind];

        laserCloudDwz_->clear();
        downSizeFilter_.setInputCloud(terrainVoxelCloudPtr);
        downSizeFilter_.filter(*laserCloudDwz_);

        terrainVoxelCloudPtr->clear();
        int laserCloudDwzSize = laserCloudDwz_->points.size();
        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = laserCloudDwz_->points[i];
          float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) +
                           (point.y - vehicleY_) * (point.y - vehicleY_));
          // Keep points that satisfy height and time decay constraints
          if (point.z - vehicleZ_ > minRelZ_ - disRatioZ_ * dis &&
              point.z - vehicleZ_ < maxRelZ_ + disRatioZ_ * dis &&
              (laserCloudTime_ - systemInitTime_ - point.intensity < decayTime_ || dis < noDecayDis_) &&
              !(dis < clearingDis_ && clearingCloud_)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        terrainVoxelUpdateNum_[ind] = 0;
        terrainVoxelUpdateTime_[ind] = laserCloudTime_ - systemInitTime_;
      }
    }

    // Merge all voxels into a single cloud for processing
    terrainCloud_->clear();
    //set the filter more small! 
    //实际处理的时候把网格缩小了
    for (int indX = terrainVoxelHalfWidth_ - 5; indX <= terrainVoxelHalfWidth_ + 5; indX++) {
      for (int indY = terrainVoxelHalfWidth_ - 5; indY <= terrainVoxelHalfWidth_ + 5; indY++) {
        *terrainCloud_ += *terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY];
      }
    }

    // --- 4. Ground Estimation ---
    // Analyze points in a finer planar grid to estimate ground elevation
    for (int i = 0; i < planarVoxelNum_; i++) {
      planarVoxelElev_[i] = 0;
      planarVoxelEdge_[i] = 0;
      planarVoxelDyObs_[i] = 0;
      planarVoxelOutOfFov_[i] = 0;
      planarPointElev_[i].clear();
    }

    int terrainCloudSize = terrainCloud_->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud_->points[i];

      int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
      int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

      if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
      if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

      // Add points to planar grid neighbors
      if (point.z - vehicleZ_ > minRelZ_ && point.z - vehicleZ_ < maxRelZ_) {
        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth_ &&
                indY + dY >= 0 && indY + dY < planarVoxelWidth_) {
              planarPointElev_[planarVoxelWidth_ * (indX + dX) + indY + dY].push_back(point.z);
            }
          }
        }
      }
    }

    // Calculate elevation for each planar voxel (using sorting or minimum)
    // height map think could be used for RL (NOTE)
    if (useSorting_) {
      for (int i = 0; i < planarVoxelNum_; i++) {
        int planarPointElevSize = planarPointElev_[i].size();
        if (planarPointElevSize > 0) {
          sort(planarPointElev_[i].begin(), planarPointElev_[i].end());

          int quantileID = int(quantileZ_ * planarPointElevSize);
          if (quantileID < 0)
            quantileID = 0;
          else if (quantileID >= planarPointElevSize)
            quantileID = planarPointElevSize - 1;

          if (planarPointElev_[i][quantileID] > planarPointElev_[i][0] + maxGroundLift_ && limitGroundLift_) {
            planarVoxelElev_[i] = planarPointElev_[i][0] + maxGroundLift_;
          } else {
            planarVoxelElev_[i] = planarPointElev_[i][quantileID];
          }
        }
      }
    } else {
      for (int i = 0; i < planarVoxelNum_; i++) {
        int planarPointElevSize = planarPointElev_[i].size();
        if (planarPointElevSize > 0) {
          float minZ = 1000.0;
          int minID = -1;
          for (int j = 0; j < planarPointElevSize; j++) {
            if (planarPointElev_[i][j] < minZ) {
              minZ = planarPointElev_[i][j];
              minID = j;
            }
          }

          if (minID != -1) {
            planarVoxelElev_[i] = planarPointElev_[i][minID];
          }
        }
      }
    }

    // --- 5. Dynamic Obstacle Filtering (Optional) ---
    if (clearDyObs_) {
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud_->points[i];
        
        // ... (Logic for identifying dynamic obstacles based on visibility) ...
        // [Simplified comment for brevity in logic]
        
        int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
        int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

        if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
        if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

        if (indX >= 0 && indX < planarVoxelWidth_ && indY >= 0 && indY < planarVoxelWidth_) {
           // ... (Detailed geometry checks omit here for brevity, keeping original logic) ...
           // Assuming dynamic obstacle detection implementation
           float pointX1 = point.x - vehicleX_;
           float pointY1 = point.y - vehicleY_;
           float pointZ1 = point.z - vehicleZ_;
           
           float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis_) {
            float h1 = point.z - planarVoxelElev_[planarVoxelWidth_ * indX + indY];
            if (h1 > obstacleHeightThre_) {
              // odom→body 旋转 (预计算的 R = R_yaw * R_pitch * R_roll)
              float pointX4 = vehicleRotBody_[0][0] * pointX1 + vehicleRotBody_[0][1] * pointY1 + vehicleRotBody_[0][2] * pointZ1;
              float pointY4 = vehicleRotBody_[1][0] * pointX1 + vehicleRotBody_[1][1] * pointY1 + vehicleRotBody_[1][2] * pointZ1;
              float pointZ4 = vehicleRotBody_[2][0] * pointX1 + vehicleRotBody_[2][1] * pointY1 + vehicleRotBody_[2][2] * pointZ1;

              float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
              float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
              if ((angle4 > minDyObsVFOV_ && angle4 < maxDyObsVFOV_) || fabs(pointZ4) < absDyObsRelZThre_) {
                planarVoxelDyObs_[planarVoxelWidth_ * indX + indY]++;
              } else if (angle4 <= minDyObsVFOV_) {
                planarVoxelOutOfFov_[planarVoxelWidth_ * indX + indY]++;
              }
            }
          } else {
            planarVoxelDyObs_[planarVoxelWidth_ * indX + indY] += minDyObsPointNum_;
          }
        }
      }
      
      // Additional pass to clear obstacles if needed 
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop_->points[i];
         int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
         int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

        if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
        if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

        if (indX >= 0 && indX < planarVoxelWidth_ && indY >= 0 && indY < planarVoxelWidth_) {
           float h1 = point.z - planarVoxelElev_[planarVoxelWidth_ * indX + indY];
           if (h1 > obstacleHeightThre_) {
             planarVoxelDyObs_[planarVoxelWidth_ * indX + indY] = -1;
           }
        }
      }
    }

    // --- 6. Terrain Map Generation ---
    // Generate the final terrain map (obstacles and elevated ground)
    terrainCloudElev_->clear();
    int terrainCloudElevSize = 0;
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud_->points[i];
      if (point.z - vehicleZ_ > minRelZ_ && point.z - vehicleZ_ < maxRelZ_) {
        int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
        int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

        if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
        if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

        if (indX >= 0 && indX < planarVoxelWidth_ && indY >= 0 && indY < planarVoxelWidth_) {
          int dyObsPointNum = planarVoxelDyObs_[planarVoxelWidth_ * indX + indY];
          if (dyObsPointNum < minDyObsPointNum_ || !clearDyObs_) {
            float disZ = point.z - planarVoxelElev_[planarVoxelWidth_ * indX + indY];
            if (considerDrop_)
              disZ = fabs(disZ);
            
            // Check if point constitutes an obstacle
            int planarPointElevSize = planarPointElev_[planarVoxelWidth_ * indX + indY].size();
            int outOfFovPointNum = planarVoxelOutOfFov_[planarVoxelWidth_ * indX + indY];
            if (disZ >= 0 && disZ < vehicleHeight_ && planarPointElevSize >= minBlockPointNum_ &&
                (outOfFovPointNum >= minOutOfFovPointNum_ || disZ < obstacleHeightThre_ || dyObsPointNum < 0 ||
                 !clearDyObs_)) {
              point.intensity = disZ; // Intensity stores height above ground
              terrainCloudElev_->push_back(point);
              terrainCloudElevSize++;
            }
          }
        }
      }
    }

    // --- 7. No Data Handling (Virtual Obstacles) ---
    // Handle cases where no data is available (potential void/cliff)
    if (noDataObstacle_ && noDataInited_ == 2) {
      for (int i = 0; i < planarVoxelNum_; i++) {
        int planarPointElevSize = planarPointElev_[i].size();
        if (planarPointElevSize < minBlockPointNum_) {
          planarVoxelEdge_[i] = 1;
        }
      }

      // Propagate edge information to fill gaps
      for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < noDataBlockSkipNum_; noDataBlockSkipCount++) {
        for (int i = 0; i < planarVoxelNum_; i++) {
          if (planarVoxelEdge_[i] >= 1) {
            int indX = int(i / planarVoxelWidth_);
            int indY = i % planarVoxelWidth_;
            bool edgeVoxel = false;
            for (int dX = -1; dX <= 1; dX++) {
              for (int dY = -1; dY <= 1; dY++) {
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth_ && indY + dY >= 0 && indY + dY < planarVoxelWidth_) {
                  if (planarVoxelEdge_[planarVoxelWidth_ * (indX + dX) + indY + dY] < planarVoxelEdge_[i]) {
                    edgeVoxel = true;
                  }
                }
              }
            }

            if (!edgeVoxel)
              planarVoxelEdge_[i]++;
          }
        }
      }

      // Create virtual obstacles at edges
      for (int i = 0; i < planarVoxelNum_; i++) {
        if (planarVoxelEdge_[i] > noDataBlockSkipNum_) {
          int indX = int(i / planarVoxelWidth_);
          int indY = i % planarVoxelWidth_;

          point.x = planarVoxelSize_ * (indX - planarVoxelHalfWidth_) + vehicleX_;
          point.y = planarVoxelSize_ * (indY - planarVoxelHalfWidth_) + vehicleY_;
          point.z = vehicleZ_;
          point.intensity = vehicleHeight_;

          // Create a small block of points for the virtual obstacle
          point.x -= planarVoxelSize_ / 4.0;
          point.y -= planarVoxelSize_ / 4.0;
          terrainCloudElev_->push_back(point);

          point.x += planarVoxelSize_ / 2.0;
          terrainCloudElev_->push_back(point);

          point.y += planarVoxelSize_ / 2.0;
          terrainCloudElev_->push_back(point);

          point.x -= planarVoxelSize_ / 2.0;
          terrainCloudElev_->push_back(point);
        }
      }
    }

    clearingCloud_ = false;

    // --- 8. Publish Result ---
    sensor_msgs::msg::PointCloud2 terrainCloud2;
    pcl::toROSMsg(*terrainCloudElev_, terrainCloud2);
    terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime_ * 1e9));
    terrainCloud2.header.frame_id = "odom";  // Output in odom coordinate frame
    pubLaserCloud_->publish(terrainCloud2);
  }

  // --- Member Variables and Parameters ---

  // Dynamic Parameter Callback Handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ROS Parameters
  double scanVoxelSize_;
  double decayTime_;
  double noDecayDis_;
  double clearingDis_;
  bool useSorting_;
  double quantileZ_;
  bool considerDrop_;
  bool limitGroundLift_;
  double maxGroundLift_;
  bool clearDyObs_;
  double minDyObsDis_;
  double absDyObsRelZThre_;
  double minDyObsVFOV_;
  double maxDyObsVFOV_;
  int minDyObsPointNum_;
  int minOutOfFovPointNum_;
  double obstacleHeightThre_;
  bool noDataObstacle_;
  int noDataBlockSkipNum_;
  int minBlockPointNum_;
  double vehicleHeight_;
  int voxelPointUpdateThre_;
  double voxelTimeUpdateThre_;
  double minRelZ_;
  double maxRelZ_;
  double disRatioZ_;

  // Voxel Grid Constants
  static const int terrainVoxelWidth_ = 21;
  static const int terrainVoxelNum_ = terrainVoxelWidth_ * terrainVoxelWidth_;
  static const int terrainVoxelHalfWidth_ = (terrainVoxelWidth_ - 1) / 2;
  static constexpr float terrainVoxelSize_ = 1.0;

  static const int planarVoxelWidth_ = 51;
  static const int planarVoxelNum_ = planarVoxelWidth_ * planarVoxelWidth_;
  static const int planarVoxelHalfWidth_ = (planarVoxelWidth_ - 1) / 2;
  static constexpr float planarVoxelSize_ = 0.2;

  // Vehicle State
  double vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;
  double vehicleRoll_ = 0, vehiclePitch_ = 0, vehicleYaw_ = 0;
  double sinVehicleRoll_ = 0, cosVehicleRoll_ = 0;
  double sinVehiclePitch_ = 0, cosVehiclePitch_ = 0;
  double sinVehicleYaw_ = 0, cosVehicleYaw_ = 0;
  double vehicleRotBody_[3][3] = {};  // 预计算 ZYX 欧拉旋转矩阵
  double vehicleRecX_ = 0, vehicleRecY_ = 0;
  
  // System State
  double laserCloudTime_ = 0;
  bool newLaserCloud_ = false;
  double systemInitTime_ = 0;
  bool systemInited_ = false;
  int noDataInited_ = 0;
  bool clearingCloud_ = false;

  // Map Management
  int terrainVoxelShiftX_ = 0;
  int terrainVoxelShiftY_ = 0;

  // Voxel Arrays (Fixed size for performance)
  int terrainVoxelUpdateNum_[terrainVoxelNum_];
  float terrainVoxelUpdateTime_[terrainVoxelNum_];
  float planarVoxelElev_[planarVoxelNum_];
  int planarVoxelEdge_[planarVoxelNum_];
  int planarVoxelDyObs_[planarVoxelNum_];
  int planarVoxelOutOfFov_[planarVoxelNum_];
  vector<float> planarPointElev_[planarVoxelNum_];

  // Point Cloud Pointers
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud_[terrainVoxelNum_];

  // Filters
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_;

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subClearing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainAnalysis>());
  rclcpp::shutdown();
  return 0;
}
