#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

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

const double PI = M_PI;  // 使用系统精确值，避免截断误差

// RotLUT: 36 个固定旋转角的 sin/cos 查找表（仅初始化一次）
// 消除三重循环中层的 n×36 次重复 sin/cos 调用
namespace {
struct RotLUT {
  float s[36], c[36];
  RotLUT() {
    for (int i = 0; i < 36; i++) {
      double a = (10.0 * i - 180.0) * M_PI / 180.0;
      s[i] = static_cast<float>(std::sin(a));
      c[i] = static_cast<float>(std::cos(a));
    }
  }
};
const RotLUT kRotLUT;
}  // namespace

#define PLOTPATHSET 1

class LocalPlanner : public rclcpp::Node
{
public:
  LocalPlanner() : Node("localPlanner")
  {
    // --- Parameter Declaration ---
    declare_parameter<std::string>("pathFolder", pathFolder_);
    declare_parameter<double>("vehicleLength", vehicleLength_);
    declare_parameter<double>("vehicleWidth", vehicleWidth_);
    declare_parameter<double>("sensorOffsetX", sensorOffsetX_);
    declare_parameter<double>("sensorOffsetY", sensorOffsetY_);
    declare_parameter<bool>("twoWayDrive", twoWayDrive_);
    declare_parameter<double>("laserVoxelSize", laserVoxelSize_);
    declare_parameter<double>("terrainVoxelSize", terrainVoxelSize_);
    declare_parameter<bool>("useTerrainAnalysis", useTerrainAnalysis_);
    declare_parameter<bool>("checkObstacle", checkObstacle_);
    declare_parameter<bool>("checkRotObstacle", checkRotObstacle_);
    declare_parameter<double>("adjacentRange", adjacentRange_);
    declare_parameter<double>("obstacleHeightThre", obstacleHeightThre_);
    declare_parameter<double>("groundHeightThre", groundHeightThre_);
    declare_parameter<double>("costHeightThre1", costHeightThre1_);
    declare_parameter<double>("costHeightThre2", costHeightThre2_);
    declare_parameter<bool>("useCost", useCost_);
    declare_parameter<int>("slowPathNumThre", slowPathNumThre_);
    declare_parameter<int>("slowGroupNumThre", slowGroupNumThre_);
    declare_parameter<int>("pointPerPathThre", pointPerPathThre_);
    declare_parameter<double>("minRelZ", minRelZ_);
    declare_parameter<double>("maxRelZ", maxRelZ_);
    declare_parameter<double>("maxSpeed", maxSpeed_);
    declare_parameter<double>("dirWeight", dirWeight_);
    declare_parameter<double>("dirThre", dirThre_);
    declare_parameter<bool>("dirToVehicle", dirToVehicle_);
    declare_parameter<double>("pathScale", pathScale_);
    declare_parameter<double>("minPathScale", minPathScale_);
    declare_parameter<double>("pathScaleStep", pathScaleStep_);
    declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed_);
    declare_parameter<double>("minPathRange", minPathRange_);
    declare_parameter<double>("pathRangeStep", pathRangeStep_);
    declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed_);
    declare_parameter<bool>("pathCropByGoal", pathCropByGoal_);
    declare_parameter<bool>("autonomyMode", autonomyMode_);
    declare_parameter<double>("autonomySpeed", autonomySpeed_);
    declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay_);
    declare_parameter<double>("joyToCheckObstacleDelay", joyToCheckObstacleDelay_);
    declare_parameter<double>("freezeAng", freezeAng_);
    declare_parameter<double>("freezeTime", freezeTime_);
    declare_parameter<double>("omniDirGoalThre", omniDirGoalThre_);
    declare_parameter<double>("goalClearRange", goalClearRange_);
    declare_parameter<double>("goalBehindRange", goalBehindRange_);
    declare_parameter<double>("goalX", goalX_);
    declare_parameter<double>("goalY", goalY_);
    declare_parameter<double>("slopeWeight", slopeWeight_);

    // --- Parameter Getting ---
    pathFolder_ = get_parameter("pathFolder").as_string();
    vehicleLength_ = get_parameter("vehicleLength").as_double();
    vehicleWidth_ = get_parameter("vehicleWidth").as_double();
    sensorOffsetX_ = get_parameter("sensorOffsetX").as_double();
    sensorOffsetY_ = get_parameter("sensorOffsetY").as_double();
    twoWayDrive_ = get_parameter("twoWayDrive").as_bool();
    laserVoxelSize_ = get_parameter("laserVoxelSize").as_double();
    terrainVoxelSize_ = get_parameter("terrainVoxelSize").as_double();
    useTerrainAnalysis_ = get_parameter("useTerrainAnalysis").as_bool();
    checkObstacle_ = get_parameter("checkObstacle").as_bool();
    checkRotObstacle_ = get_parameter("checkRotObstacle").as_bool();
    adjacentRange_ = get_parameter("adjacentRange").as_double();
    obstacleHeightThre_ = get_parameter("obstacleHeightThre").as_double();
    groundHeightThre_ = get_parameter("groundHeightThre").as_double();
    costHeightThre1_ = get_parameter("costHeightThre1").as_double();
    costHeightThre2_ = get_parameter("costHeightThre2").as_double();
    useCost_ = get_parameter("useCost").as_bool();
    slowPathNumThre_ = get_parameter("slowPathNumThre").as_int();
    slowGroupNumThre_ = get_parameter("slowGroupNumThre").as_int();
    pointPerPathThre_ = get_parameter("pointPerPathThre").as_int();
    minRelZ_ = get_parameter("minRelZ").as_double();
    maxRelZ_ = get_parameter("maxRelZ").as_double();
    maxSpeed_ = get_parameter("maxSpeed").as_double();
    dirWeight_ = get_parameter("dirWeight").as_double();
    dirThre_ = get_parameter("dirThre").as_double();
    dirToVehicle_ = get_parameter("dirToVehicle").as_bool();
    pathScale_ = get_parameter("pathScale").as_double();
    minPathScale_ = get_parameter("minPathScale").as_double();
    pathScaleStep_ = get_parameter("pathScaleStep").as_double();
    pathScaleBySpeed_ = get_parameter("pathScaleBySpeed").as_bool();
    minPathRange_ = get_parameter("minPathRange").as_double();
    pathRangeStep_ = get_parameter("pathRangeStep").as_double();
    pathRangeBySpeed_ = get_parameter("pathRangeBySpeed").as_bool();
    pathCropByGoal_ = get_parameter("pathCropByGoal").as_bool();
    autonomyMode_ = get_parameter("autonomyMode").as_bool();
    autonomySpeed_ = get_parameter("autonomySpeed").as_double();
    joyToSpeedDelay_ = get_parameter("joyToSpeedDelay").as_double();
    joyToCheckObstacleDelay_ = get_parameter("joyToCheckObstacleDelay").as_double();
    freezeAng_ = get_parameter("freezeAng").as_double();
    freezeTime_ = get_parameter("freezeTime").as_double();
    omniDirGoalThre_ = get_parameter("omniDirGoalThre").as_double();
    goalClearRange_ = get_parameter("goalClearRange").as_double();
    goalBehindRange_ = get_parameter("goalBehindRange").as_double();
    goalX_ = get_parameter("goalX").as_double();
    goalY_ = get_parameter("goalY").as_double();
    slopeWeight_ = get_parameter("slopeWeight").as_double();

    // --- Dynamic Parameter Callback ---
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &p : params) {
            const auto &n = p.get_name();
            if (n == "twoWayDrive") twoWayDrive_ = p.as_bool();
            else if (n == "checkObstacle") checkObstacle_ = p.as_bool();
            else if (n == "checkRotObstacle") checkRotObstacle_ = p.as_bool();
            else if (n == "useTerrainAnalysis") useTerrainAnalysis_ = p.as_bool();
            else if (n == "useCost") useCost_ = p.as_bool();
            else if (n == "pathScaleBySpeed") pathScaleBySpeed_ = p.as_bool();
            else if (n == "pathRangeBySpeed") pathRangeBySpeed_ = p.as_bool();
            else if (n == "pathCropByGoal") pathCropByGoal_ = p.as_bool();
            else if (n == "pathScale") pathScale_ = p.as_double();
            else if (n == "minPathScale") minPathScale_ = p.as_double();
            else if (n == "pathScaleStep") pathScaleStep_ = p.as_double();
            else if (n == "minPathRange") minPathRange_ = p.as_double();
            else if (n == "pathRangeStep") pathRangeStep_ = p.as_double();
            else if (n == "maxSpeed") maxSpeed_ = p.as_double();
            else if (n == "dirWeight") dirWeight_ = p.as_double();
            else if (n == "dirThre") dirThre_ = p.as_double();
            else if (n == "obstacleHeightThre") obstacleHeightThre_ = p.as_double();
            else if (n == "groundHeightThre") groundHeightThre_ = p.as_double();
            else if (n == "adjacentRange") adjacentRange_ = p.as_double();
            else if (n == "costHeightThre1") costHeightThre1_ = p.as_double();
            else if (n == "costHeightThre2") costHeightThre2_ = p.as_double();
            else if (n == "laserVoxelSize") laserVoxelSize_ = p.as_double();
            else if (n == "terrainVoxelSize") terrainVoxelSize_ = p.as_double();
            else if (n == "minRelZ") minRelZ_ = p.as_double();
            else if (n == "maxRelZ") maxRelZ_ = p.as_double();
            else if (n == "goalClearRange") goalClearRange_ = p.as_double();
            else if (n == "goalBehindRange") goalBehindRange_ = p.as_double();
            else if (n == "freezeAng") freezeAng_ = p.as_double();
            else if (n == "freezeTime") freezeTime_ = p.as_double();
            else if (n == "slopeWeight") slopeWeight_ = p.as_double();
            else {
              RCLCPP_WARN(get_logger(), "Unknown dynamic param: %s", n.c_str());
            }
          }
          return result;
        });

    // --- ROS Interface ---
    subOdometry_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 5, std::bind(&LocalPlanner::odometryHandler, this, std::placeholders::_1));
    // Subscribe to world map cloud (odom frame) when not using terrain analysis
    subLaserCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", 5, std::bind(&LocalPlanner::laserCloudHandler, this, std::placeholders::_1));
    // Subscribe to terrain map (odom frame) when using terrain analysis
    subTerrainCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/terrain_map", 5, std::bind(&LocalPlanner::terrainCloudHandler, this, std::placeholders::_1));
    // Subscribe to extended terrain map (connectivity + time-decayed accumulation)
    subTerrainCloudExt_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/terrain_map_ext", 5, std::bind(&LocalPlanner::terrainCloudExtHandler, this, std::placeholders::_1));
    subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, std::bind(&LocalPlanner::joystickHandler, this, std::placeholders::_1));
    subGoal_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/way_point", 5, std::bind(&LocalPlanner::goalHandler, this, std::placeholders::_1));
    subSpeed_ = create_subscription<std_msgs::msg::Float32>(
        "/speed", 5, std::bind(&LocalPlanner::speedHandler, this, std::placeholders::_1));
    subBoundary_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/navigation_boundary", 5, std::bind(&LocalPlanner::boundaryHandler, this, std::placeholders::_1));
    subAddedObstacles_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/added_obstacles", 5, std::bind(&LocalPlanner::addedObstaclesHandler, this, std::placeholders::_1));
    subCheckObstacle_ = create_subscription<std_msgs::msg::Bool>(
        "/check_obstacle", 5, std::bind(&LocalPlanner::checkObstacleHandler, this, std::placeholders::_1));

    pubSlowDown_ = create_publisher<std_msgs::msg::Int8>("/slow_down", 5);
    pubStop_ = create_publisher<std_msgs::msg::Int8>("/stop", 5);
    pubPath_ = create_publisher<nav_msgs::msg::Path>("/path", 5);

    #if PLOTPATHSET == 1
    pubFreePaths_ = create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
    #endif

    // --- Initialization ---
    // Memory Allocation
    laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudExt_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudExtCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    boundaryCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    addedObstacles_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    #if PLOTPATHSET == 1
    freePaths_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    #endif

    for (int i = 0; i < laserCloudStackNum_; i++) {
        laserCloudStack_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (int i = 0; i < groupNum_; i++) {
        startPaths_[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    #if PLOTPATHSET == 1
    for (int i = 0; i < pathNum_; i++) {
        paths_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    #endif
    for (int i = 0; i < gridVoxelNum_; i++) {
        correspondences_[i].clear();
        correspondences_[i].reserve(8);  // P4: 预分配，避免 push_back 触发多次重新分配
    }
    
    laserDwzFilter_.setLeafSize(laserVoxelSize_, laserVoxelSize_, laserVoxelSize_);
    terrainDwzFilter_.setLeafSize(terrainVoxelSize_, terrainVoxelSize_, terrainVoxelSize_);

    // Logic Initialization
    if (autonomyMode_) {
        joySpeed_ = autonomySpeed_ / maxSpeed_;
        if (joySpeed_ < 0) joySpeed_ = 0;
        else if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    }

    readStartPaths();
    #if PLOTPATHSET == 1
    readPaths();
    #endif
    readPathList();
    readCorrespondences();

    RCLCPP_INFO(get_logger(), "Initialization complete.");

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&LocalPlanner::processLoop, this));
  }

private:
  // --- Dynamic Parameter Callback Handle ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // --- Parameters & Constants ---
  std::string pathFolder_;
  double vehicleLength_ = 0.6;
  double vehicleWidth_ = 0.6;
  double sensorOffsetX_ = 0;
  double sensorOffsetY_ = 0;
  bool twoWayDrive_ = true;
  double laserVoxelSize_ = 0.1;
  double terrainVoxelSize_ = 0.2;
  bool useTerrainAnalysis_ = false;
  bool checkObstacle_ = true;
  bool checkRotObstacle_ = false;
  double adjacentRange_ = 3.5;
  double obstacleHeightThre_ = 0.2;
  double groundHeightThre_ = 0.1;
  double costHeightThre1_ = 0.15;
  double costHeightThre2_ = 0.1;
  bool useCost_ = false;
  double slopeWeight_ = 0.0;  // 地形坡度惩罚权重 (0=关闭, 建议范围 2~8)
  int slowPathNumThre_ = 5;
  int slowGroupNumThre_ = 1;
  const int laserCloudStackNum_ = 1;
  int laserCloudCount_ = 0;
  int pointPerPathThre_ = 2;
  double minRelZ_ = -0.5;
  double maxRelZ_ = 0.25;
  double maxSpeed_ = 1.0;
  double dirWeight_ = 0.02;
  double dirThre_ = 90.0;
  bool dirToVehicle_ = false;
  double pathScale_ = 1.0;
  double minPathScale_ = 0.75;
  double pathScaleStep_ = 0.25;
  bool pathScaleBySpeed_ = true;
  double minPathRange_ = 2.5;
  double pathRangeStep_ = 0.5;
  bool pathRangeBySpeed_ = true;
  bool pathCropByGoal_ = true;
  bool autonomyMode_ = false;
  double autonomySpeed_ = 1.0;
  double joyToSpeedDelay_ = 2.0;
  double joyToCheckObstacleDelay_ = 5.0;
  double freezeAng_ = 90.0;
  double freezeTime_ = 2.0;
  double omniDirGoalThre_ = 1.0;
  double goalClearRange_ = 0.5;
  double goalBehindRange_ = 0.8;
  double goalX_ = 0;
  double goalY_ = 0;
  double nearFieldStopDis_ = 0.5;  // Near-field emergency stop distance (m)
  bool nearFieldStopped_ = false;   // Track near-field stop state

  // Grid Constants
  static const int pathNum_ = 343;
  static const int groupNum_ = 7;
  float gridVoxelSize_ = 0.02;
  float searchRadius_ = 0.45;
  float gridVoxelOffsetX_ = 3.2;
  float gridVoxelOffsetY_ = 4.5;
  static const int gridVoxelNumX_ = 161;
  static const int gridVoxelNumY_ = 451;
  static const int gridVoxelNum_ = gridVoxelNumX_ * gridVoxelNumY_;

  // --- State Variables ---
  float joySpeed_ = 0;
  float joySpeedRaw_ = 0;
  float joyDir_ = 0;
  double odomTime_ = 0;
  double joyTime_ = 0;
  double freezeStartTime_ = 0;
  int freezeStatus_ = 0;

  float vehicleRoll_ = 0, vehiclePitch_ = 0, vehicleYaw_ = 0;
  float vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;

  // ── Recovery state machine (三阶段卡死恢复) ──────────────────────────
  int    recoveryState_       = 0;    // 0=normal, 1=rotating, 2=backing_up
  double blockedStartTime_    = -1.0; // 首次 pathFound=false 的 odomTime_
  double recoveryPhaseStart_  = -1.0; // 当前恢复阶段的开始时刻
  double recoveryBlockedThre_ = 2.0;  // 触发恢复前的持续阻塞时间 (s)
  double recoveryRotateTime_  = 2.5;  // 旋转阶段持续时间 (s)
  double recoveryBackupTime_  = 1.5;  // 后退阶段持续时间 (s)

  bool newLaserCloud_ = false;
  bool newTerrainCloud_ = false;
  bool newTerrainCloudExt_ = false;

  // Arrays
  int pathList_[pathNum_] = {0};
  float endDirPathList_[pathNum_] = {0};
  int clearPathList_[36 * pathNum_] = {0};
  float pathPenaltyList_[36 * pathNum_] = {0};
  float clearPathPerGroupScore_[36 * groupNum_] = {0};
  int clearPathPerGroupNum_[36 * groupNum_] = {0};
  float pathPenaltyPerGroupScore_[36 * groupNum_] = {0};
  std::vector<int> correspondences_[gridVoxelNum_];

  // Point Clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudExt_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudExtCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack_[1]; // Assumes stackNum=1
  pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths_[groupNum_];
  #if PLOTPATHSET == 1
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths_[pathNum_];
  pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths_;
  #endif

  // Filters
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter_;
  pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter_;

  // ROS Handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloudExt_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subBoundary_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subAddedObstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subCheckObstacle_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubSlowDown_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubStop_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
  #if PLOTPATHSET == 1
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths_;
  #endif
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Callbacks ---
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    odomTime_ = rclcpp::Time(odom->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll_ = roll;
    vehiclePitch_ = pitch;
    vehicleYaw_ = yaw;
    vehicleX_ = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX_ + sin(yaw) * sensorOffsetY_;
    vehicleY_ = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX_ - cos(yaw) * sensorOffsetY_;
    vehicleZ_ = odom->pose.pose.position.z;
  }

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
  {
    //只有当不启用地形分析时，才直接处理原始点云
    if (!useTerrainAnalysis_) {
      laserCloud_->clear();
      pcl::fromROSMsg(*laserCloud2, *laserCloud_);

      pcl::PointXYZI point;
      laserCloudCrop_->clear();
      int laserCloudSize = laserCloud_->points.size();
      // O8: 平方距离比较消除 sqrt（雷达回调可能处理 10K+ 点/帧）
      const float adjRangeSqLaser = adjacentRange_ * adjacentRange_;
      for (int i = 0; i < laserCloudSize; i++) {
        point = laserCloud_->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dx = pointX - vehicleX_, dy = pointY - vehicleY_;
        if (dx * dx + dy * dy < adjRangeSqLaser) {
          point.x = pointX;
          point.y = pointY;
          point.z = pointZ;
          laserCloudCrop_->push_back(point);
        }
      }

      laserCloudDwz_->clear();
      laserDwzFilter_.setInputCloud(laserCloudCrop_);
      laserDwzFilter_.filter(*laserCloudDwz_);

      newLaserCloud_ = true;
    }
  }

  void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
  {
    if (useTerrainAnalysis_) {
      terrainCloud_->clear();
      pcl::fromROSMsg(*terrainCloud2, *terrainCloud_);

      pcl::PointXYZI point;
      terrainCloudCrop_->clear();
      int terrainCloudSize = terrainCloud_->points.size();
      // O8: 平方距离比较（地形回调同样处理大量点）
      const float adjRangeSqTerrain = adjacentRange_ * adjacentRange_;
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud_->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dx = pointX - vehicleX_, dy = pointY - vehicleY_;
        if (dx * dx + dy * dy < adjRangeSqTerrain && (point.intensity > obstacleHeightThre_ || (point.intensity > groundHeightThre_ && useCost_))) {
          point.x = pointX;
          point.y = pointY;
          point.z = pointZ;
          terrainCloudCrop_->push_back(point);
        }
      }

      terrainCloudDwz_->clear();
      terrainDwzFilter_.setInputCloud(terrainCloudCrop_);
      terrainDwzFilter_.filter(*terrainCloudDwz_);

      newTerrainCloud_ = true;
    }
  }

  void terrainCloudExtHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudExt2)
  {
    if (useTerrainAnalysis_) {
      terrainCloudExt_->clear();
      pcl::fromROSMsg(*terrainCloudExt2, *terrainCloudExt_);

      pcl::PointXYZI point;
      terrainCloudExtCrop_->clear();
      int cloudSize = terrainCloudExt_->points.size();
      // O8: 平方距离比较（扩展地形回调）
      const float adjRangeSqExt = adjacentRange_ * adjacentRange_;
      for (int i = 0; i < cloudSize; i++) {
        point = terrainCloudExt_->points[i];
        float dx = point.x - vehicleX_, dy = point.y - vehicleY_;
        if (dx * dx + dy * dy < adjRangeSqExt &&
            (point.intensity > obstacleHeightThre_ ||
             (point.intensity > groundHeightThre_ && useCost_))) {
          terrainCloudExtCrop_->push_back(point);
        }
      }

      newTerrainCloudExt_ = true;
    }
  }

  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    joyTime_ = now().seconds();
    joySpeedRaw_ = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed_ = joySpeedRaw_;
    if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    if (joy->axes[4] == 0) joySpeed_ = 0;

    if (joySpeed_ > 0) {
      joyDir_ = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
      if (joy->axes[4] < 0) joyDir_ *= -1;
    }

    if (joy->axes[4] < 0 && !twoWayDrive_) joySpeed_ = 0;

    // axes[2] (LT): 控制自主/手动模式
    // 松开 LT = 自主导航（默认）；按下 LT = 手动接管
    if (joy->axes[2] > -0.1) {
      autonomyMode_ = true;   // 松开 = 自主模式（默认）
    } else {
      autonomyMode_ = false;  // 按下 = 手动模式（接管）
    }

    // axes[5] (RT): 控制是否避障
    // 松开 RT = 启用避障（默认安全）；按下 RT = 关闭避障（强制通过）
    if (joy->axes[5] > -0.1) {
      checkObstacle_ = true;   // 启用避障
    } else {
      checkObstacle_ = false;  // 关闭避障
    }
  }


  //Find indeed in local the goal is 2d!  But after think this is enough.
  void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
  {
    goalX_ = goal->point.x;
    goalY_ = goal->point.y;
  }

  void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
  {
    double speedTime = now().seconds();
    if (autonomyMode_ && speedTime - joyTime_ > joyToSpeedDelay_ && joySpeedRaw_ == 0) {
      joySpeed_ = speed->data / maxSpeed_;

      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    }
  }

  void boundaryHandler(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
  {
    boundaryCloud_->clear();
    pcl::PointXYZI point, point1, point2;
    int boundarySize = boundary->polygon.points.size();

    if (boundarySize >= 1) {
      point2.x = boundary->polygon.points[0].x;
      point2.y = boundary->polygon.points[0].y;
      point2.z = boundary->polygon.points[0].z;
    }

    for (int i = 0; i < boundarySize; i++) {
      point1 = point2;

      point2.x = boundary->polygon.points[i].x;
      point2.y = boundary->polygon.points[i].y;
      point2.z = boundary->polygon.points[i].z;

      if (point1.z == point2.z) {
        float disX = point1.x - point2.x;
        float disY = point1.y - point2.y;
        float dis = sqrt(disX * disX + disY * disY);

        int pointNum = int(dis / terrainVoxelSize_) + 1;
        for (int pointID = 0; pointID < pointNum; pointID++) {
          point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
          point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
          point.z = 0;
          point.intensity = 100.0;

          for (int j = 0; j < pointPerPathThre_; j++) {
            boundaryCloud_->push_back(point);
          }
        }
      }
    }
  }

  void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr addedObstacles2)
  {
    addedObstacles_->clear();
    pcl::fromROSMsg(*addedObstacles2, *addedObstacles_);

    int addedObstaclesSize = addedObstacles_->points.size();
    for (int i = 0; i < addedObstaclesSize; i++) {
      addedObstacles_->points[i].intensity = 200.0;
    }
  }

  void checkObstacleHandler(const std_msgs::msg::Bool::ConstSharedPtr checkObs)
  {
    double checkObsTime = now().seconds();
    if (autonomyMode_ && checkObsTime - joyTime_ > joyToCheckObstacleDelay_) {
      checkObstacle_ = checkObs->data;
    }
  }

  // --- Helper Functions ---
  int readPlyHeader(FILE *filePtr)
  {
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
      val = fscanf(filePtr, "%s", str);
      if (val != 1) {
        RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      strLast = strCur;
      strCur = string(str);

      if (strCur == "vertex" && strLast == "element") {
        val = fscanf(filePtr, "%d", &pointNum);
        if (val != 1) {
          RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
          exit(1);
        }
      }
    }
    return pointNum;
  }

  void readStartPaths()
  {
    string fileName = pathFolder_ + "/startPaths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++) {
      val1 = fscanf(filePtr, "%f", &point.x);
      val2 = fscanf(filePtr, "%f", &point.y);
      val3 = fscanf(filePtr, "%f", &point.z);
      val4 = fscanf(filePtr, "%d", &groupID);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
        RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (groupID >= 0 && groupID < groupNum_) {
        startPaths_[groupID]->push_back(point);
      }
    }

    fclose(filePtr);
  }

  #if PLOTPATHSET == 1
  void readPaths()
  {
    string fileName = pathFolder_ + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
      val1 = fscanf(filePtr, "%f", &point.x);
      val2 = fscanf(filePtr, "%f", &point.y);
      val3 = fscanf(filePtr, "%f", &point.z);
      val4 = fscanf(filePtr, "%d", &pathID);
      val5 = fscanf(filePtr, "%f", &point.intensity);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (pathID >= 0 && pathID < pathNum_) {
        pointSkipCount++;
        if (pointSkipCount > pointSkipNum) {
          paths_[pathID]->push_back(point);
          pointSkipCount = 0;
        }
      }
    }

    fclose(filePtr);
  }
  #endif

  void readPathList()
  {
    string fileName = pathFolder_ + "/pathList.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    if (pathNum_ != readPlyHeader(filePtr)) {
      RCLCPP_INFO(get_logger(), "Incorrect path number, exit.");
      exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum_; i++) {
      val1 = fscanf(filePtr, "%f", &endX);
      val2 = fscanf(filePtr, "%f", &endY);
      val3 = fscanf(filePtr, "%f", &endZ);
      val4 = fscanf(filePtr, "%d", &pathID);
      val5 = fscanf(filePtr, "%d", &groupID);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (pathID >= 0 && pathID < pathNum_ && groupID >= 0 && groupID < groupNum_) {
        pathList_[pathID] = groupID;
        endDirPathList_[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
      }
    }

    fclose(filePtr);
  }

  void readCorrespondences()
  {
    string fileName = pathFolder_ + "/correspondences.txt";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      RCLCPP_INFO(get_logger(), "Cannot read input files, exit.");
      exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum_; i++) {
      val1 = fscanf(filePtr, "%d", &gridVoxelID);
      if (val1 != 1) {
        RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      while (1) {
        val1 = fscanf(filePtr, "%d", &pathID);
        if (val1 != 1) {
          RCLCPP_INFO(get_logger(), "Error reading input files, exit.");
            exit(1);
        }

        if (pathID != -1) {
          if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum_ && pathID >= 0 && pathID < pathNum_) {
            correspondences_[gridVoxelID].push_back(pathID);
          }
        } else {
          break;
        }
      }
    }

    fclose(filePtr);
  }

  // --- Main Logic ---
  void processLoop()
  {
    if (autonomyMode_ && joySpeed_ == 0) {
      joySpeed_ = autonomySpeed_ / maxSpeed_;
      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0) joySpeed_ = 1.0;
    }

    if (newLaserCloud_ || newTerrainCloud_ || newTerrainCloudExt_) {
      if (newLaserCloud_) {
        newLaserCloud_ = false;

        laserCloudStack_[laserCloudCount_]->clear();
        *laserCloudStack_[laserCloudCount_] = *laserCloudDwz_;
        laserCloudCount_ = (laserCloudCount_ + 1) % laserCloudStackNum_;

        plannerCloud_->clear();
        for (int i = 0; i < laserCloudStackNum_; i++) {
          *plannerCloud_ += *laserCloudStack_[i];
        }
      }

      if (newTerrainCloud_) {
        newTerrainCloud_ = false;

        plannerCloud_->clear();
        *plannerCloud_ = *terrainCloudDwz_;
      }

      // Merge extended terrain map (connectivity + time-decayed obstacles)
      if (newTerrainCloudExt_) {
        newTerrainCloudExt_ = false;
        *plannerCloud_ += *terrainCloudExtCrop_;
      }

      float sinVehicleYaw = sin(vehicleYaw_);
      float cosVehicleYaw = cos(vehicleYaw_);

      plannerCloudCrop_->clear();
      // O11: 预分配上界防止 push_back 触发多次 realloc（三个源云之和）
      plannerCloudCrop_->reserve(plannerCloud_->size() +
                                  boundaryCloud_->size() +
                                  addedObstacles_->size());

      // O3+O6: 三段重复过滤代码合并为 lambda，去除 sqrt（平方比较）
      const float adjacentRangeSq = adjacentRange_ * adjacentRange_;
      auto appendCroppedCloud = [&](const pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                                    bool checkZ) {
        pcl::PointXYZI p;
        for (const auto& pt : src->points) {
          float dx = pt.x - vehicleX_;
          float dy = pt.y - vehicleY_;
          float dz = pt.z - vehicleZ_;
          if (dx * dx + dy * dy >= adjacentRangeSq) continue;
          if (checkZ && !((dz > minRelZ_ && dz < maxRelZ_) || useTerrainAnalysis_)) continue;
          p.x = dx * cosVehicleYaw + dy * sinVehicleYaw;
          p.y = -dx * sinVehicleYaw + dy * cosVehicleYaw;
          p.z = dz;
          p.intensity = pt.intensity;
          plannerCloudCrop_->push_back(p);
        }
      };
      appendCroppedCloud(plannerCloud_,    true);   // 主点云：含 z 范围过滤
      appendCroppedCloud(boundaryCloud_,   false);  // 边界云：无 z 过滤
      appendCroppedCloud(addedObstacles_,  false);  // 添加障碍：无 z 过滤

      // Near-field emergency stop: if any obstacle is within nearFieldStopDis_
      // and above obstacleHeightThre_, immediately publish /stop
      if (checkObstacle_) {
        bool nearFieldObstacle = false;
        int plannerCloudCropSizeNF = plannerCloudCrop_->points.size();
        for (int i = 0; i < plannerCloudCropSizeNF; i++) {
          float px = plannerCloudCrop_->points[i].x;
          float py = plannerCloudCrop_->points[i].y;
          float h = plannerCloudCrop_->points[i].intensity;
          // Only check obstacles in front of the robot (positive x in body frame)
          // and within vehicle width
          if (px > 0 && px < nearFieldStopDis_ &&
              fabs(py) < vehicleWidth_ / 2.0 + 0.1 &&
              (h > obstacleHeightThre_ || !useTerrainAnalysis_)) {
            nearFieldObstacle = true;
            break;
          }
        }

        if (nearFieldObstacle && !nearFieldStopped_) {
          std_msgs::msg::Int8 stopMsg;
          stopMsg.data = 2;  // Level 2: full stop
          pubStop_->publish(stopMsg);
          nearFieldStopped_ = true;
          RCLCPP_WARN(get_logger(),
                      "NEAR-FIELD ESTOP: obstacle within %.2fm!", nearFieldStopDis_);
        } else if (!nearFieldObstacle && nearFieldStopped_) {
          std_msgs::msg::Int8 stopMsg;
          stopMsg.data = 0;  // Clear stop
          pubStop_->publish(stopMsg);
          nearFieldStopped_ = false;
        }
      }

      float pathRange = adjacentRange_;
      if (pathRangeBySpeed_) pathRange = adjacentRange_ * joySpeed_;
      if (pathRange < minPathRange_) pathRange = minPathRange_;
      float relativeGoalDis = adjacentRange_;

      int preSelectedGroupID = -1;
      if (autonomyMode_) {
        float relativeGoalX = ((goalX_ - vehicleX_) * cosVehicleYaw + (goalY_ - vehicleY_) * sinVehicleYaw);
        float relativeGoalY = (-(goalX_ - vehicleX_) * sinVehicleYaw + (goalY_ - vehicleY_) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir_ = atan2(relativeGoalY, relativeGoalX) * 180 / PI;
        
        if (fabs(joyDir_) > freezeAng_ && relativeGoalDis < goalBehindRange_) {
          relativeGoalDis = 0;
          joyDir_ = 0;
        }
        
        if (fabs(joyDir_) > freezeAng_ && freezeStatus_ == 0) {
          freezeStartTime_ = odomTime_;
          freezeStatus_ = 1;
        } else if (odomTime_ - freezeStartTime_ > freezeTime_ && freezeStatus_ == 1) {
          freezeStatus_ = 2;
        } else if (fabs(joyDir_) <= freezeAng_ && freezeStatus_ == 2) {
          freezeStatus_ = 0;
        }

        if (!twoWayDrive_) {
          if (joyDir_ > 95.0) {
            joyDir_ = 95.0;
            preSelectedGroupID = 0;
          } else if (joyDir_ < -95.0) {
            joyDir_ = -95.0;
            preSelectedGroupID = 6;
          }
        }
      } else {
        freezeStatus_ = 0;
      }

      if (freezeStatus_ == 1 && autonomyMode_) {
        relativeGoalDis = 0;
        joyDir_ = 0;
      }

      bool pathFound = false;
      float defPathScale = pathScale_;
      if (pathScaleBySpeed_) pathScale_ = defPathScale * joySpeed_;
      if (pathScale_ < minPathScale_) pathScale_ = minPathScale_;

      // O2: joyDir_ 在整帧内不变，预计算 36 个方向差（每帧一次替代 n×36 次）
      float angDiffList[36];
      for (int d = 0; d < 36; d++) {
        float a = std::fabs(joyDir_ - (10.0f * d - 180.0f));
        angDiffList[d] = (a > 180.0f) ? 360.0f - a : a;
      }

      // O5: 体素网格坐标变换常量（gridVoxelSize_ 整帧不变）
      const float kInvGridVoxelSize = 1.0f / gridVoxelSize_;
      const float kOffsetXHalf     = gridVoxelOffsetX_ + gridVoxelSize_ * 0.5f;
      const float kOffsetYHalf     = gridVoxelOffsetY_ + gridVoxelSize_ * 0.5f;
      const float kScaleYCoefA     = searchRadius_ / gridVoxelOffsetY_;
      const float kScaleYCoefB     = 1.0f / gridVoxelOffsetX_;

      // O10: vehicle geometry 常量（参数启动后不变），提升到 while 循环外
      const float kDiameter  = sqrtf(vehicleLength_ / 2.0f * vehicleLength_ / 2.0f +
                                     vehicleWidth_  / 2.0f * vehicleWidth_  / 2.0f);
      const float kAngOffset = std::atan2(vehicleWidth_, vehicleLength_) * 180.0f / PI;

      while (pathScale_ >= minPathScale_ && pathRange >= minPathRange_) {
        // O4: memset 比 for 循环清零快 2-3×（~12.6K 元素）
        memset(clearPathList_,            0, sizeof(int)   * 36 * pathNum_);
        memset(pathPenaltyList_,          0, sizeof(float) * 36 * pathNum_);
        memset(clearPathPerGroupScore_,   0, sizeof(float) * 36 * groupNum_);
        memset(clearPathPerGroupNum_,     0, sizeof(int)   * 36 * groupNum_);
        memset(pathPenaltyPerGroupScore_, 0, sizeof(float) * 36 * groupNum_);

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        // diameter 和 angOffset 已提升（见 O10）
        const float diameter  = kDiameter;
        const float angOffset = kAngOffset;
        int plannerCloudCropSize = plannerCloudCrop_->points.size();

        // O13: 方向有效性预计算——当前条件只依赖 rotDir/joyDir_/dirThre_
        // 无需在 N×36 内层循环中重复评估（N=1000 时节省 36K 次条件判断/帧）
        bool rotDirValid[36];
        {
          const bool toVehicle = dirToVehicle_;
          const float absJoyDir = fabs(joyDir_);
          const bool joyLe90 = absJoyDir <= 90.0f;
          const bool joyGt90 = absJoyDir > 90.0f;
          for (int d = 0; d < 36; d++) {
            float rotAngDegD = 10.0f * d - 180.0f;
            float rotDegD    = 10.0f * d;
            bool skip = (angDiffList[d] > dirThre_ && !toVehicle) ||
                        (fabs(rotAngDegD) > dirThre_ && joyLe90 && toVehicle) ||
                        ((rotDegD > dirThre_ && 360.0f - rotDegD > dirThre_) && joyGt90 && toVehicle);
            rotDirValid[d] = !skip;
          }
        }

        // O3: 平方阈值（pathScale_ 在 while 循环内可变，需在循环体内计算）
        const float kPathRangeScaleSq = (pathRange / pathScale_) * (pathRange / pathScale_);
        const float kDiameterScaleSq  = (diameter  / pathScale_) * (diameter  / pathScale_);
        const float kGoalClearScaleSq = ((relativeGoalDis + goalClearRange_) / pathScale_)
                                       * ((relativeGoalDis + goalClearRange_) / pathScale_);
        const float kRelGoalScaledSq  = (relativeGoalDis / pathScale_) * (relativeGoalDis / pathScale_);
        const float kHalfLenScale = vehicleLength_ / pathScale_ / 2.0f;
        const float kHalfWidScale = vehicleWidth_  / pathScale_ / 2.0f;

        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop_->points[i].x / pathScale_;
          float y = plannerCloudCrop_->points[i].y / pathScale_;
          float h = plannerCloudCrop_->points[i].intensity;
          float disSq = x * x + y * y;  // O3: 平方距离，不再 sqrt

          if (disSq < kPathRangeScaleSq && (disSq <= kGoalClearScaleSq || !pathCropByGoal_) && checkObstacle_) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              // O13: 方向有效性已预计算（rotDirValid[36]），直接查表
              if (!rotDirValid[rotDir]) continue;

              // O1: 使用 RotLUT 替代 cos(rotAng)/sin(rotAng)（预计算，无运行时三角）
              float x2 = kRotLUT.c[rotDir] * x + kRotLUT.s[rotDir] * y;
              float y2 = -kRotLUT.s[rotDir] * x + kRotLUT.c[rotDir] * y;

              // O5: 使用预计算常量替代内层重复除法
              float scaleY = x2 * kScaleYCoefB + kScaleYCoefA * (gridVoxelOffsetX_ - x2) * kScaleYCoefB;
              if (std::fabs(scaleY) < 1e-6f) continue;  // 防止 y2/scaleY 除零

              int indX = static_cast<int>((kOffsetXHalf - x2) * kInvGridVoxelSize);
              int indY = static_cast<int>((kOffsetYHalf - y2 / scaleY) * kInvGridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX_ && indY >= 0 && indY < gridVoxelNumY_) {
                int ind = gridVoxelNumY_ * indX + indY;
                int blockedPathByVoxelNum = correspondences_[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre_ || !useTerrainAnalysis_) {
                    clearPathList_[pathNum_ * rotDir + correspondences_[ind][j]]++;
                  } else {
                    if (pathPenaltyList_[pathNum_ * rotDir + correspondences_[ind][j]] < h && h > groundHeightThre_) {
                      pathPenaltyList_[pathNum_ * rotDir + correspondences_[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          // O3: 平方距离比较（旋转障碍物检测）
          if (disSq < kDiameterScaleSq && (fabs(x) > kHalfLenScale || fabs(y) > kHalfWidScale) &&
              (h > obstacleHeightThre_ || !useTerrainAnalysis_) && checkRotObstacle_) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum_; i++) {
          int rotDir = int(i / pathNum_);
          // O12: angDiffList 复用（与主规划循环共享预计算值），消除重复 fabs+归一化
          float angDiff = angDiffList[rotDir];
          float rotAngDeg = 10.0f * rotDir - 180.0f;  // 仅算一次
          float rotDeg    = 10.0f * rotDir;
          if ((angDiff > dirThre_ && !dirToVehicle_) || (fabs(rotAngDeg) > dirThre_ && fabs(joyDir_) <= 90.0 && dirToVehicle_) ||
              ((rotDeg > dirThre_ && 360.0f - rotDeg > dirThre_) && fabs(joyDir_) > 90.0 && dirToVehicle_)) {
            continue;
          }

          const int pathIdx = i % pathNum_;  // 减少重复取模
          if (clearPathList_[i] < pointPerPathThre_) {
            float dirDiff = fabs(joyDir_ - endDirPathList_[pathIdx] - rotAngDeg);
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float groupDirW = 4  - fabs(pathList_[pathIdx] - 3);
            float dw = std::fabs(dirWeight_ * dirDiff);  // 防御负参数导致 NaN
            // O9: sqrtf (float) 替代 std::sqrt (double)；dw^0.25 缓存一次；rotDirW^4 = (rotDirW^2)^2
            float sqrtSqrtDw = sqrtf(sqrtf(dw));
            float rotDirW2 = rotDirW * rotDirW;
            // 地形坡度惩罚: pathPenaltyList_ 存储路径上最大地形高度代价 (intensity)
            // slopeWeight_ > 0 时, 高代价路径评分下降, 规划器倾向选择平坦方向
            float terrainFactor = (slopeWeight_ > 0.0f)
                ? std::max(0.0f, 1.0f - slopeWeight_ * pathPenaltyList_[i])
                : 1.0f;
            float score = (1.0f - sqrtSqrtDw) * rotDirW2 * rotDirW2 * terrainFactor;
            if (relativeGoalDis < omniDirGoalThre_) score = (1.0f - sqrtSqrtDw) * groupDirW * groupDirW * terrainFactor;
            if (score > 0) {
              int groupIdx = groupNum_ * rotDir + pathList_[pathIdx];
              clearPathPerGroupScore_[groupIdx]     += score;
              clearPathPerGroupNum_[groupIdx]++;
              pathPenaltyPerGroupScore_[groupIdx] += pathPenaltyList_[i];
            }
          }
        }

        int selectedGroupID = -1;
        if (preSelectedGroupID >= 0) {
          selectedGroupID = preSelectedGroupID;
        } else {
          float maxScore = 0;
          for (int i = 0; i < 36 * groupNum_; i++) {
            int rotDir = int(i / groupNum_);
            // O14: rotAng*180/PI = 10*rotDir-180，消除无用 deg→rad→deg 往返转换
            float rotAngDeg = 10.0f * rotDir - 180.0f;
            float rotDeg    = 10.0f * rotDir;
            if (rotDeg > 180.0f) rotDeg -= 360.0f;
            if (maxScore < clearPathPerGroupScore_[i] && ((rotAngDeg > minObsAngCW && rotAngDeg < minObsAngCCW) ||
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive_) || !checkRotObstacle_)) {
              maxScore = clearPathPerGroupScore_[i];
              selectedGroupID = i;
            }
          }
        }

        float penaltyScore = 0;
        std_msgs::msg::Int8 slow;
        if (selectedGroupID >= 0) {
          int selectedPathNum = clearPathPerGroupNum_[selectedGroupID];
          if (selectedPathNum > 0) {
            penaltyScore = pathPenaltyPerGroupScore_[selectedGroupID] / selectedPathNum;
          }

          if (penaltyScore > costHeightThre1_) slow.data = 1;
          else if (penaltyScore > costHeightThre2_) slow.data = 2;
          else if (selectedPathNum < slowPathNumThre_ && fabs(selectedGroupID - 129) > slowGroupNumThre_) slow.data = 3;
          else slow.data = 0;
          pubSlowDown_->publish(slow);
        }

        nav_msgs::msg::Path path;
        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum_);
          // O1: RotLUT 消除路径输出段 2 次 trig（每规划周期一次）
          const float rc = kRotLUT.c[rotDir], rs = kRotLUT.s[rotDir];

          selectedGroupID = selectedGroupID % groupNum_;
          int selectedPathLength = startPaths_[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths_[selectedGroupID]->points[i].x;
            float y = startPaths_[selectedGroupID]->points[i].y;
            float z = startPaths_[selectedGroupID]->points[i].z;
            // O3: 平方距离比较，消除 sqrt
            float disSq = x * x + y * y;

            if (disSq <= kPathRangeScaleSq && disSq <= kRelGoalScaledSq) {
              path.poses[i].pose.position.x = pathScale_ * (rc * x - rs * y);
              path.poses[i].pose.position.y = pathScale_ * (rs * x + rc * y);
              path.poses[i].pose.position.z = pathScale_ * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
          path.header.frame_id = "body";  // Use body frame (consistent with Fast-LIO2 TF)
          pubPath_->publish(path);

          #if PLOTPATHSET == 1
          freePaths_->clear();
          freePaths_->reserve(36 * pathNum_);  // O11: 预分配防止多次 realloc
          for (int i = 0; i < 36 * pathNum_; i++) {
            int rotDir = int(i / pathNum_);
            // O1+O2+O14: RotLUT + angDiffList 复用，消除可视化循环内全部 trig
            float rotAngDeg = 10.0f * rotDir - 180.0f;
            float rotDegRaw = 10.0f * rotDir;   // wrap 前（用于 dirThre_ 比较）
            float rotDeg    = rotDegRaw;
            if (rotDeg > 180.0f) rotDeg -= 360.0f;
            float angDiff = angDiffList[rotDir];
            if ((angDiff > dirThre_ && !dirToVehicle_) || (fabs(rotAngDeg) > dirThre_ && fabs(joyDir_) <= 90.0 && dirToVehicle_) ||
                ((rotDegRaw > dirThre_ && 360.0f - rotDegRaw > dirThre_) && fabs(joyDir_) > 90.0 && dirToVehicle_) ||
                !((rotAngDeg > minObsAngCW && rotAngDeg < minObsAngCCW) ||
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive_) || !checkRotObstacle_)) {
              continue;
            }

            if (clearPathList_[i] < pointPerPathThre_) {
              const float rc = kRotLUT.c[rotDir], rs = kRotLUT.s[rotDir];
              int freePathLength = paths_[i % pathNum_]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                pcl::PointXYZI point = paths_[i % pathNum_]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                // O3: 平方距离比较，消除可视化循环内 sqrt
                float disSq = x * x + y * y;
                if (disSq <= kPathRangeScaleSq && (disSq <= kGoalClearScaleSq || !pathCropByGoal_)) {
                  point.x = pathScale_ * (rc * x - rs * y);
                  point.y = pathScale_ * (rs * x + rc * y);
                  point.z = pathScale_ * z;
                  point.intensity = 1.0;

                  freePaths_->push_back(point);
                }
              }
            }
          }

          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths_, freePaths2);
          freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
          freePaths2.header.frame_id = "body";  // Use body frame (consistent with Fast-LIO2 TF)
          pubFreePaths_->publish(freePaths2);
          #endif
        }

        if (selectedGroupID < 0) {
          if (pathScale_ >= minPathScale_ + pathScaleStep_) {
            pathScale_ -= pathScaleStep_;
            pathRange = adjacentRange_ * pathScale_ / defPathScale;
          } else {
            pathRange -= pathRangeStep_;
          }
        } else {
          // 找到路径 → 重置恢复状态
          recoveryState_    = 0;
          blockedStartTime_ = -1.0;
          pathFound = true;
          break;
        }
      }
      pathScale_ = defPathScale;

      if (!pathFound) {
        // ── 三阶段卡死恢复状态机 ─────────────────────────────────────
        // 状态 0 (NORMAL/等待): 停车，计时
        // 状态 1 (ROTATING):  向目标方向旋转的弧形路径
        // 状态 2 (BACKING_UP): 沿 body -X 方向后退路径
        if (blockedStartTime_ < 0) blockedStartTime_ = odomTime_;
        const double blockedDur = odomTime_ - blockedStartTime_;

        if (recoveryState_ == 0 && blockedDur >= recoveryBlockedThre_) {
          recoveryState_      = 1;
          recoveryPhaseStart_ = odomTime_;
          RCLCPP_WARN(get_logger(),
            "[Recovery] 路径全部受阻 %.1fs → 进入旋转恢复", blockedDur);
        } else if (recoveryState_ == 1 &&
                   odomTime_ - recoveryPhaseStart_ >= recoveryRotateTime_) {
          recoveryState_      = 2;
          recoveryPhaseStart_ = odomTime_;
          RCLCPP_WARN(get_logger(), "[Recovery] 旋转完成 → 进入后退");
        } else if (recoveryState_ == 2 &&
                   odomTime_ - recoveryPhaseStart_ >= recoveryBackupTime_) {
          recoveryState_    = 0;
          blockedStartTime_ = odomTime_;  // 重新计时
          RCLCPP_WARN(get_logger(), "[Recovery] 后退完成 → 重置恢复计时");
        }

        // ── 构建恢复路径 (body frame) ──────────────────────────────────
        nav_msgs::msg::Path recPath;
        recPath.header.stamp    = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        recPath.header.frame_id = "body";

        if (recoveryState_ == 1) {
          // 旋转阶段：向目标方向弯曲的弧形路径
          const float goalAngleWorld = atan2f(goalY_ - vehicleY_, goalX_ - vehicleX_);
          float relAngle = goalAngleWorld - vehicleYaw_;
          while (relAngle >  static_cast<float>(M_PI)) relAngle -= 2.0f * static_cast<float>(M_PI);
          while (relAngle < -static_cast<float>(M_PI)) relAngle += 2.0f * static_cast<float>(M_PI);
          const float turnDir = (relAngle >= 0.0f) ? 1.0f : -1.0f;

          for (int i = 1; i <= 6; ++i) {
            const float arc = turnDir * static_cast<float>(i) * 0.25f;
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = 0.15f * static_cast<float>(i) * cosf(arc);
            p.pose.position.y = 0.15f * static_cast<float>(i) * sinf(arc);
            recPath.poses.push_back(p);
          }

        } else if (recoveryState_ == 2) {
          // 后退阶段：沿 body -X 后退，pathFollower twoWayDrive_ 负速执行
          for (int i = 1; i <= 5; ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = -0.2f * static_cast<float>(i);
            p.pose.position.y = 0.0f;
            recPath.poses.push_back(p);
          }

        } else {
          // NORMAL 阶段（等待计时）：零点路径，pathFollower 停车
          recPath.poses.resize(1);
          recPath.poses[0].pose.position.x = 0;
          recPath.poses[0].pose.position.y = 0;
          recPath.poses[0].pose.position.z = 0;
        }

        pubPath_->publish(recPath);

        #if PLOTPATHSET == 1
        freePaths_->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths_, freePaths2);
        freePaths2.header.stamp    = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        freePaths2.header.frame_id = "body";
        pubFreePaths_->publish(freePaths2);
        #endif
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
