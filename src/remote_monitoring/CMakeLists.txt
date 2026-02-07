cmake_minimum_required(VERSION 3.8)
project(remote_monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- ROS2 dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)

# ---- Proto library (from robot_proto package) ----
find_package(robot_proto REQUIRED)

# ---- gRPC (for reflection + runtime) ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++)

# ---- WebRTC (optional) ----
find_package(LibDataChannel QUIET)
if(LibDataChannel_FOUND)
  message(STATUS "Found libdatachannel via CMake config: ${LibDataChannel_VERSION}")
  set(DATACHANNEL_FOUND TRUE)
  add_definitions(-DWEBRTC_ENABLED)
else()
  pkg_check_modules(DATACHANNEL libdatachannel)
  if(DATACHANNEL_FOUND)
    message(STATUS "Found libdatachannel via pkg-config: ${DATACHANNEL_LIBRARIES}")
    add_definitions(-DWEBRTC_ENABLED)
  else()
    message(WARNING "libdatachannel not found - WebRTC features will be disabled")
  endif()
endif()

# ---- Gateway sources ----
set(GATEWAY_SOURCES
  src/main.cpp
  src/status_aggregator.cpp
  src/grpc_gateway.cpp
  src/core/lease_manager.cpp
  src/core/event_buffer.cpp
  src/core/safety_gate.cpp
  src/core/mode_manager.cpp
  src/core/geofence_monitor.cpp
  src/core/health_monitor.cpp
  src/core/task_manager.cpp
  src/services/system_service.cpp
  src/services/control_service.cpp
  src/services/telemetry_service.cpp
  src/services/data_service.cpp
)

if(DATACHANNEL_FOUND)
  list(APPEND GATEWAY_SOURCES src/webrtc_bridge.cpp)
endif()

# ---- Build ----
add_executable(grpc_gateway ${GATEWAY_SOURCES})

target_include_directories(grpc_gateway PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link proto library (brings in protobuf + grpc headers/libs automatically)
target_link_libraries(grpc_gateway
  robot_proto::robot_proto
  grpc++_reflection
)

# Link WebRTC if available
if(LibDataChannel_FOUND)
  target_link_libraries(grpc_gateway LibDataChannel::LibDataChannel)
elseif(DATACHANNEL_FOUND)
  target_include_directories(grpc_gateway PRIVATE ${DATACHANNEL_INCLUDE_DIRS})
  target_link_libraries(grpc_gateway ${DATACHANNEL_LIBRARIES})
endif()

ament_target_dependencies(grpc_gateway
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2_ros interface
)

# ---- Install ----
install(TARGETS grpc_gateway
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
