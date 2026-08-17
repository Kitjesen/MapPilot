cmake_minimum_required(VERSION 3.5)
project(allan_variance_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(COLCON_PACKAGE_DEPENDENCIES
  Boost
  Eigen3
  rclcpp
  rosbag2_cpp
  std_msgs
  yaml-cpp
  OpenMP
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED filesystem)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found will try to link!")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
src/ImuMeasurement.cpp
src/yaml_parsers.cpp
src/AllanVarianceComputor.cpp
)
# target_link_libraries(${PROJECT_NAME} ${COLCON_PACKAGE_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME} ${COLCON_PACKAGE_DEPENDENCIES})

add_executable(allan_variance src/allan_variance.cpp)
target_link_libraries(allan_variance ${PROJECT_NAME} )

add_executable(plot_imu src/plot_imu.cpp)
target_link_libraries(plot_imu PRIVATE ${PROJECT_NAME} rerun_sdk)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  allan_variance
  plot_imu
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_EXPORT_DEPENDS})
ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")