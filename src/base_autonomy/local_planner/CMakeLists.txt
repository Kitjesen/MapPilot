cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_core REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Local Planner Node
add_executable(localPlanner src/localPlanner.cpp)
target_link_libraries(localPlanner
  ${PCL_LIBRARIES}
)
ament_target_dependencies(localPlanner
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  nav_core
)

# Path Follower Node
add_executable(pathFollower src/pathFollower.cpp)
ament_target_dependencies(pathFollower
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  nav_core
)
target_link_libraries(pathFollower
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  localPlanner
  pathFollower
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  paths
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ── Algorithm Unit Tests (pure math, no ROS2 / hardware required) ──
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_local_planner_algorithms
    test/test_algorithms.cpp
  )

  # ── Performance & Correctness Regression Tests ──
  # 验证所有优化（O1-O6, P1-P3）不改变数值结果，并量化加速比
  ament_add_gtest(test_planner_perf
    test/test_planner_perf.cpp
  )
  target_compile_options(test_planner_perf PRIVATE -O2)  # 基准测试需要优化级别
endif()

ament_package()
