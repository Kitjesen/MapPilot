#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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
 * @class TerrainAnalysisExt
 * @brief Extended terrain analysis node for elevation mapping and connectivity checking
 * 
 * This node processes registered point clouds to generate an extended terrain map with:
 * - Time-decayed terrain voxel accumulation
 * - Ground elevation estimation
 * - Terrain connectivity validation
 * - Integration with local terrain maps
 */
class TerrainAnalysisExt : public rclcpp::Node
{
public:
  TerrainAnalysisExt() : Node("terrainAnalysisExt")
  {
    // --- Parameter Declaration ---
    declare_parameter<double>("scanVoxelSize", scanVoxelSize_);
    declare_parameter<double>("decayTime", decayTime_);
    declare_parameter<double>("noDecayDis", noDecayDis_);
    declare_parameter<double>("clearingDis", clearingDis_);
    declare_parameter<bool>("useSorting", useSorting_);
    declare_parameter<double>("quantileZ", quantileZ_);
    declare_parameter<double>("vehicleHeight", vehicleHeight_);
    declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre_);
    declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre_);
    declare_parameter<double>("lowerBoundZ", lowerBoundZ_);
    declare_parameter<double>("upperBoundZ", upperBoundZ_);
    declare_parameter<double>("disRatioZ", disRatioZ_);
    declare_parameter<bool>("checkTerrainConn", checkTerrainConn_);
    declare_parameter<double>("terrainUnderVehicle", terrainUnderVehicle_);
    declare_parameter<double>("terrainConnThre", terrainConnThre_);
    declare_parameter<double>("ceilingFilteringThre", ceilingFilteringThre_);
    declare_parameter<double>("localTerrainMapRadius", localTerrainMapRadius_);

    // --- Get Parameters ---
    scanVoxelSize_ = get_parameter("scanVoxelSize").as_double();
    decayTime_ = get_parameter("decayTime").as_double();
    noDecayDis_ = get_parameter("noDecayDis").as_double();
    clearingDis_ = get_parameter("clearingDis").as_double();
    useSorting_ = get_parameter("useSorting").as_bool();
    quantileZ_ = get_parameter("quantileZ").as_double();
    vehicleHeight_ = get_parameter("vehicleHeight").as_double();
    voxelPointUpdateThre_ = get_parameter("voxelPointUpdateThre").as_int();
    voxelTimeUpdateThre_ = get_parameter("voxelTimeUpdateThre").as_double();
    lowerBoundZ_ = get_parameter("lowerBoundZ").as_double();
    upperBoundZ_ = get_parameter("upperBoundZ").as_double();
    disRatioZ_ = get_parameter("disRatioZ").as_double();
    checkTerrainConn_ = get_parameter("checkTerrainConn").as_bool();
    terrainUnderVehicle_ = get_parameter("terrainUnderVehicle").as_double();
    terrainConnThre_ = get_parameter("terrainConnThre").as_double();
    ceilingFilteringThre_ = get_parameter("ceilingFilteringThre").as_double();
    localTerrainMapRadius_ = get_parameter("localTerrainMapRadius").as_double();

    // --- ROS Interfaces ---
    subOdometry_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 5, std::bind(&TerrainAnalysisExt::odometryHandler, this, std::placeholders::_1));

    subLaserCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 5, std::bind(&TerrainAnalysisExt::laserCloudHandler, this, std::placeholders::_1));

    subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, std::bind(&TerrainAnalysisExt::joystickHandler, this, std::placeholders::_1));

    subClearing_ = create_subscription<std_msgs::msg::Float32>(
        "/cloud_clearing", 5, std::bind(&TerrainAnalysisExt::clearingHandler, this, std::placeholders::_1));

    subTerrainCloudLocal_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/terrain_map", 2, std::bind(&TerrainAnalysisExt::terrainCloudLocalHandler, this, std::placeholders::_1));

    pubTerrainCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map_ext", 2);

    // --- Initialization ---
    laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudElev_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudLocal_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i < terrainVoxelNum_; i++) {
      terrainVoxelCloud_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    downSizeFilter_.setLeafSize(scanVoxelSize_, scanVoxelSize_, scanVoxelSize_);

    // Timer (100Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&TerrainAnalysisExt::processLoop, this));
  }

private:
  // --- Parameters ---
  double scanVoxelSize_ = 0.1;
  double decayTime_ = 10.0;
  double noDecayDis_ = 0;
  double clearingDis_ = 30.0;
  bool useSorting_ = false;
  double quantileZ_ = 0.25;
  double vehicleHeight_ = 1.5;
  int voxelPointUpdateThre_ = 100;
  double voxelTimeUpdateThre_ = 2.0;
  double lowerBoundZ_ = -1.5;
  double upperBoundZ_ = 1.0;
  double disRatioZ_ = 0.1;
  bool checkTerrainConn_ = true;
  double terrainUnderVehicle_ = -0.75;
  double terrainConnThre_ = 0.5;
  double ceilingFilteringThre_ = 2.0;
  double localTerrainMapRadius_ = 4.0;

  // Terrain voxel parameters
  float terrainVoxelSize_ = 2.0;
  int terrainVoxelShiftX_ = 0;
  int terrainVoxelShiftY_ = 0;
  static const int terrainVoxelWidth_ = 41;
  int terrainVoxelHalfWidth_ = (terrainVoxelWidth_ - 1) / 2;
  static const int terrainVoxelNum_ = terrainVoxelWidth_ * terrainVoxelWidth_;

  // Planar voxel parameters
  float planarVoxelSize_ = 0.4;
  static const int planarVoxelWidth_ = 101;
  int planarVoxelHalfWidth_ = (planarVoxelWidth_ - 1) / 2;
  static const int planarVoxelNum_ = planarVoxelWidth_ * planarVoxelWidth_;

  // --- State Variables ---
  bool clearingCloud_ = false;
  double laserCloudTime_ = 0;
  bool newlaserCloud_ = false;
  double systemInitTime_ = 0;
  bool systemInited_ = false;

  float vehicleRoll_ = 0, vehiclePitch_ = 0, vehicleYaw_ = 0;
  float vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;

  // Point Clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud_[terrainVoxelWidth_ * terrainVoxelWidth_];

  // Arrays
  int terrainVoxelUpdateNum_[terrainVoxelWidth_ * terrainVoxelWidth_] = {0};
  float terrainVoxelUpdateTime_[terrainVoxelWidth_ * terrainVoxelWidth_] = {0};
  float planarVoxelElev_[101 * 101] = {0};
  int planarVoxelConn_[101 * 101] = {0};
  vector<float> planarPointElev_[101 * 101];
  queue<int> planarVoxelQueue_;

  // Filters & Trees
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_;

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subClearing_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloudLocal_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTerrainCloud_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Callbacks ---
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
  }

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

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX_) * (pointX - vehicleX_) + (pointY - vehicleY_) * (pointY - vehicleY_));
      if (pointZ - vehicleZ_ > lowerBoundZ_ - disRatioZ_ * dis && pointZ - vehicleZ_ < upperBoundZ_ + disRatioZ_ * dis &&
          dis < terrainVoxelSize_ * (terrainVoxelHalfWidth_ + 1)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime_ - systemInitTime_;
        laserCloudCrop_->push_back(point);
      }
    }

    newlaserCloud_ = true;
  }

  void terrainCloudLocalHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2)
  {
    terrainCloudLocal_->clear();
    pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal_);
  }

  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    if (joy->buttons[5] > 0.5) {
      clearingCloud_ = true;
    }
  }

  void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
  {
    clearingDis_ = dis->data;
    clearingCloud_ = true;
  }

  // --- Main Processing Loop ---
  void processLoop()
  {
    if (newlaserCloud_) {
      newlaserCloud_ = false;

      // Terrain voxel roll over
      rollTerrainVoxels();

      // Stack registered laser scans
      stackLaserScans();

      // Update terrain voxels
      updateTerrainVoxels();

      // Build terrain cloud from voxels
      buildTerrainCloud();

      // Estimate ground elevation
      estimateGroundElevation();

      // Check terrain connectivity
      if (checkTerrainConn_) {
        checkTerrainConnectivity();
      }

      // Compute extended terrain map
      computeExtendedTerrainMap();

      // Merge local terrain map
      mergeLocalTerrainMap();

      clearingCloud_ = false;

      // Publish terrain map
      publishTerrainMap();
    }
  }

  void rollTerrainVoxels()
  {
    float terrainVoxelCenX = terrainVoxelSize_ * terrainVoxelShiftX_;
    float terrainVoxelCenY = terrainVoxelSize_ * terrainVoxelShiftY_;

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
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud_[terrainVoxelWidth_ * indX];
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
  }

  void stackLaserScans()
  {
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
  }

  void updateTerrainVoxels()
  {
    pcl::PointXYZI point;
    for (int ind = 0; ind < terrainVoxelNum_; ind++) {
      if (terrainVoxelUpdateNum_[ind] >= voxelPointUpdateThre_ ||
          laserCloudTime_ - systemInitTime_ - terrainVoxelUpdateTime_[ind] >= voxelTimeUpdateThre_ || clearingCloud_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud_[ind];

        laserCloudDwz_->clear();
        downSizeFilter_.setInputCloud(terrainVoxelCloudPtr);
        downSizeFilter_.filter(*laserCloudDwz_);

        terrainVoxelCloudPtr->clear();
        int laserCloudDwzSize = laserCloudDwz_->points.size();
        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = laserCloudDwz_->points[i];
          float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) + (point.y - vehicleY_) * (point.y - vehicleY_));
          if (point.z - vehicleZ_ > lowerBoundZ_ - disRatioZ_ * dis &&
              point.z - vehicleZ_ < upperBoundZ_ + disRatioZ_ * dis &&
              (laserCloudTime_ - systemInitTime_ - point.intensity < decayTime_ || dis < noDecayDis_) &&
              !(dis < clearingDis_ && clearingCloud_)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        terrainVoxelUpdateNum_[ind] = 0;
        terrainVoxelUpdateTime_[ind] = laserCloudTime_ - systemInitTime_;
      }
    }
  }

  void buildTerrainCloud()
  {
    terrainCloud_->clear();
    for (int indX = terrainVoxelHalfWidth_ - 10; indX <= terrainVoxelHalfWidth_ + 10; indX++) {
      for (int indY = terrainVoxelHalfWidth_ - 10; indY <= terrainVoxelHalfWidth_ + 10; indY++) {
        *terrainCloud_ += *terrainVoxelCloud_[terrainVoxelWidth_ * indX + indY];
      }
    }
  }

  void estimateGroundElevation()
  {
    for (int i = 0; i < planarVoxelNum_; i++) {
      planarVoxelElev_[i] = 0;
      planarVoxelConn_[i] = 0;
      planarPointElev_[i].clear();
    }

    pcl::PointXYZI point;
    int terrainCloudSize = terrainCloud_->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud_->points[i];
      float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) + (point.y - vehicleY_) * (point.y - vehicleY_));
      if (point.z - vehicleZ_ > lowerBoundZ_ - disRatioZ_ * dis && point.z - vehicleZ_ < upperBoundZ_ + disRatioZ_ * dis) {
        int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
        int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

        if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
        if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth_ && indY + dY >= 0 && indY + dY < planarVoxelWidth_) {
              planarPointElev_[planarVoxelWidth_ * (indX + dX) + indY + dY].push_back(point.z);
            }
          }
        }
      }
    }

    if (useSorting_) {
      for (int i = 0; i < planarVoxelNum_; i++) {
        int planarPointElevSize = planarPointElev_[i].size();
        if (planarPointElevSize > 0) {
          sort(planarPointElev_[i].begin(), planarPointElev_[i].end());

          int quantileID = int(quantileZ_ * planarPointElevSize);
          if (quantileID < 0) quantileID = 0;
          else if (quantileID >= planarPointElevSize) quantileID = planarPointElevSize - 1;

          planarVoxelElev_[i] = planarPointElev_[i][quantileID];
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
  }

  void checkTerrainConnectivity()
  {
    int ind = planarVoxelWidth_ * planarVoxelHalfWidth_ + planarVoxelHalfWidth_;
    if (planarPointElev_[ind].size() == 0)
      planarVoxelElev_[ind] = vehicleZ_ + terrainUnderVehicle_;

    planarVoxelQueue_.push(ind);
    planarVoxelConn_[ind] = 1;
    while (!planarVoxelQueue_.empty()) {
      int front = planarVoxelQueue_.front();
      planarVoxelConn_[front] = 2;
      planarVoxelQueue_.pop();

      int indX = int(front / planarVoxelWidth_);
      int indY = front % planarVoxelWidth_;
      for (int dX = -10; dX <= 10; dX++) {
        for (int dY = -10; dY <= 10; dY++) {
          if (indX + dX >= 0 && indX + dX < planarVoxelWidth_ && indY + dY >= 0 && indY + dY < planarVoxelWidth_) {
            ind = planarVoxelWidth_ * (indX + dX) + indY + dY;
            if (planarVoxelConn_[ind] == 0 && planarPointElev_[ind].size() > 0) {
              if (fabs(planarVoxelElev_[front] - planarVoxelElev_[ind]) < terrainConnThre_) {
                planarVoxelQueue_.push(ind);
                planarVoxelConn_[ind] = 1;
              } else if (fabs(planarVoxelElev_[front] - planarVoxelElev_[ind]) > ceilingFilteringThre_) {
                planarVoxelConn_[ind] = -1;
              }
            }
          }
        }
      }
    }
  }

  void computeExtendedTerrainMap()
  {
    terrainCloudElev_->clear();
    int terrainCloudElevSize = 0;
    int terrainCloudSize = terrainCloud_->points.size();
    pcl::PointXYZI point;
    
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud_->points[i];
      float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) + (point.y - vehicleY_) * (point.y - vehicleY_));
      if (point.z - vehicleZ_ > lowerBoundZ_ - disRatioZ_ * dis && point.z - vehicleZ_ < upperBoundZ_ + disRatioZ_ * dis && dis > localTerrainMapRadius_) {
        int indX = int((point.x - vehicleX_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;
        int indY = int((point.y - vehicleY_ + planarVoxelSize_ / 2) / planarVoxelSize_) + planarVoxelHalfWidth_;

        if (point.x - vehicleX_ + planarVoxelSize_ / 2 < 0) indX--;
        if (point.y - vehicleY_ + planarVoxelSize_ / 2 < 0) indY--;

        if (indX >= 0 && indX < planarVoxelWidth_ && indY >= 0 && indY < planarVoxelWidth_) {
          int ind = planarVoxelWidth_ * indX + indY;
          float disZ = fabs(point.z - planarVoxelElev_[ind]);
          if (disZ < vehicleHeight_ && (planarVoxelConn_[ind] == 2 || !checkTerrainConn_)) {
            terrainCloudElev_->push_back(point);
            terrainCloudElev_->points[terrainCloudElevSize].x = point.x;
            terrainCloudElev_->points[terrainCloudElevSize].y = point.y;
            terrainCloudElev_->points[terrainCloudElevSize].z = point.z;
            terrainCloudElev_->points[terrainCloudElevSize].intensity = disZ;

            terrainCloudElevSize++;
          }
        }
      }
    }
  }

  void mergeLocalTerrainMap()
  {
    int terrainCloudLocalSize = terrainCloudLocal_->points.size();
    pcl::PointXYZI point;
    for (int i = 0; i < terrainCloudLocalSize; i++) {
      point = terrainCloudLocal_->points[i];
      float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) + (point.y - vehicleY_) * (point.y - vehicleY_));
      if (dis <= localTerrainMapRadius_) {
        terrainCloudElev_->push_back(point);
      }
    }
  }

  void publishTerrainMap()
  {
    sensor_msgs::msg::PointCloud2 terrainCloud2;
    pcl::toROSMsg(*terrainCloudElev_, terrainCloud2);
    terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime_ * 1e9));
    terrainCloud2.header.frame_id = "map";
    pubTerrainCloud_->publish(terrainCloud2);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainAnalysisExt>());
  rclcpp::shutdown();
  return 0;
}
