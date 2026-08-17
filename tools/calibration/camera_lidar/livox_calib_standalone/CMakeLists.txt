cmake_minimum_required(VERSION 3.10)
project(livox_calib_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Paths ──────────────────────────────────────────────────────────
set(ORIGINAL_CALIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../livox_camera_calib")

# Dependencies — NO catkin, NO ROS
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree sample_consensus)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

# Include order matters:
#   1. shim/  — fake ROS headers (shim ros/ros.h, rosbag, cv_bridge, etc.)
#   2. include/ — our common.h (no cv_bridge dependency)
#   3. original — lidar_camera_calib.hpp + original common.h (found after ours)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/shim
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORIGINAL_CALIB_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Suppress warnings from legacy calibration code
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-sign-compare)
endif()

# Main calibration executable
# lidar_camera_calib.hpp is header-only (all methods inline), so we only compile main.cpp
add_executable(livox_calib
  src/main.cpp
)

target_link_libraries(livox_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
