cmake_minimum_required(VERSION 3.16)
project(fastlio2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_default_ros_node OFF)
if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
  set(_default_ros_node ON)
endif()
option(FASTLIO2_BUILD_ROS_NODE "Build the ROS 2 lio_node wrapper" ${_default_ros_node})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenMP QUIET)

set(FASTLIO2_CORE_SOURCES
  src/map_builder/commons.cpp
  src/map_builder/ieskf.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
)

add_library(fastlio2_core STATIC ${FASTLIO2_CORE_SOURCES})
target_compile_features(fastlio2_core PUBLIC cxx_std_17)
target_include_directories(fastlio2_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/map_builder
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio2_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
set_target_properties(fastlio2_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(OpenMP_CXX_FOUND)
  target_compile_definitions(fastlio2_core PUBLIC MP_EN MP_PROC_NUM=4)
  target_link_libraries(fastlio2_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(fastlio2_core PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()

if(NOT FASTLIO2_BUILD_ROS_NODE)
  message(STATUS "fastlio2: ROS node disabled; building fastlio2_core only")
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(lio_node src/lio_node.cpp src/utils.cpp)
ament_target_dependencies(
  lio_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
  geometry_msgs
  interface
)
target_link_libraries(lio_node fastlio2_core yaml-cpp)

install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../launch/fastlio2_launch.py DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
