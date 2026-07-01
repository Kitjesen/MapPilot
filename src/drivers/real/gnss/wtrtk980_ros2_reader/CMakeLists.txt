cmake_minimum_required(VERSION 3.16)
project(wtrtk980_ros2_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(wtrtk980_core
  src/nmea_parser.cpp
  src/serial_detector.cpp
  src/serial_reader.cpp
)
target_include_directories(wtrtk980_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(wtrtk980_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(wtrtk980_reader src/cli_main.cpp)
target_link_libraries(wtrtk980_reader wtrtk980_core)
target_compile_options(wtrtk980_reader PRIVATE -Wall -Wextra -Wpedantic)

add_executable(wtrtk980_ros2_node src/ros2_node.cpp)
target_link_libraries(wtrtk980_ros2_node wtrtk980_core)
ament_target_dependencies(wtrtk980_ros2_node rclcpp sensor_msgs std_msgs)
target_compile_options(wtrtk980_ros2_node PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wtrtk980_core wtrtk980_reader wtrtk980_ros2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
