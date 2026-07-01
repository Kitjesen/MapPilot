cmake_minimum_required(VERSION 3.8)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP QUIET)
find_package(Sophus REQUIRED)
find_package(Threads REQUIRED)

find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(POSE_GRAPH_OPT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../kernels/slam/pose_graph_opt")
set(POSE_GRAPH_OPT_MANIFEST "${POSE_GRAPH_OPT_ROOT}/Cargo.toml")
set(POSE_GRAPH_OPT_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/pose_graph_opt_cargo")
if(WIN32)
  if(MSVC)
    set(POSE_GRAPH_OPT_STATIC_LIB "${POSE_GRAPH_OPT_TARGET_DIR}/release/lingtu_pose_graph_opt.lib")
  else()
    set(POSE_GRAPH_OPT_STATIC_LIB "${POSE_GRAPH_OPT_TARGET_DIR}/release/liblingtu_pose_graph_opt.a")
  endif()
else()
  set(POSE_GRAPH_OPT_STATIC_LIB "${POSE_GRAPH_OPT_TARGET_DIR}/release/liblingtu_pose_graph_opt.a")
endif()

file(GLOB_RECURSE POSE_GRAPH_OPT_SOURCES CONFIGURE_DEPENDS
  "${POSE_GRAPH_OPT_ROOT}/src/*.rs"
  "${POSE_GRAPH_OPT_ROOT}/include/*.h"
  "${POSE_GRAPH_OPT_ROOT}/Cargo.lock"
)

add_custom_command(
  OUTPUT "${POSE_GRAPH_OPT_STATIC_LIB}"
  COMMAND ${CMAKE_COMMAND} -E env "CARGO_TARGET_DIR=${POSE_GRAPH_OPT_TARGET_DIR}" ${CARGO_EXECUTABLE} build --release --manifest-path "${POSE_GRAPH_OPT_MANIFEST}"
  DEPENDS "${POSE_GRAPH_OPT_MANIFEST}" ${POSE_GRAPH_OPT_SOURCES}
  WORKING_DIRECTORY "${POSE_GRAPH_OPT_ROOT}"
  VERBATIM
)
add_custom_target(hba_pose_graph_opt_rust DEPENDS "${POSE_GRAPH_OPT_STATIC_LIB}")
add_library(hba_lingtu_pose_graph_opt STATIC IMPORTED)
set_target_properties(hba_lingtu_pose_graph_opt PROPERTIES
  IMPORTED_LOCATION "${POSE_GRAPH_OPT_STATIC_LIB}"
)
add_dependencies(hba_lingtu_pose_graph_opt hba_pose_graph_opt_rust)

if(TARGET Sophus::Sophus)
  get_target_property(SOPHUS_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIRS}
)

set(SRC_LIST src/hba/commons.cpp
             src/hba/blam.cpp
             src/hba/hba.cpp)

add_executable(hba_node src/hba_node.cpp ${SRC_LIST})
target_include_directories(hba_node PRIVATE "${POSE_GRAPH_OPT_ROOT}/include")
target_compile_definitions(hba_node PRIVATE LT_POSE_GRAPH_OPT_STATIC)
ament_target_dependencies(hba_node rclcpp std_msgs sensor_msgs interface pcl_conversions)
target_link_libraries(hba_node ${PCL_LIBRARIES} hba_lingtu_pose_graph_opt yaml-cpp Sophus::Sophus Threads::Threads ${CMAKE_DL_LIBS})
if(UNIX AND NOT APPLE)
  target_link_libraries(hba_node m)
endif()
if(WIN32)
  if(MSVC)
    target_link_libraries(hba_node windows kernel32 ntdll userenv ws2_32 dbghelp)
  else()
    target_link_libraries(hba_node kernel32 ntdll userenv ws2_32 dbghelp)
  endif()
endif()

install(TARGETS hba_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../launch/hba_launch.py DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
