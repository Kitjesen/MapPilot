cmake_minimum_required(VERSION 3.8)
project(localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

option(LINGTU_REQUIRE_BBS3D "Fail configure if cpu_bbs3d is not available" OFF)
option(LINGTU_ENABLE_SMALL_GICP "Use small_gicp for relocalization refine when available" ON)
set(CPU_BBS3D_ROOT "" CACHE PATH "Optional local install prefix for cpu_bbs3d")
set(SMALL_GICP_ROOT "" CACHE PATH "Optional small_gicp source or install prefix")
if(NOT CPU_BBS3D_ROOT AND DEFINED ENV{CPU_BBS3D_ROOT})
  set(CPU_BBS3D_ROOT "$ENV{CPU_BBS3D_ROOT}")
endif()
if(NOT SMALL_GICP_ROOT AND DEFINED ENV{SMALL_GICP_ROOT})
  set(SMALL_GICP_ROOT "$ENV{SMALL_GICP_ROOT}")
endif()
if(NOT SMALL_GICP_ROOT)
  get_filename_component(_LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(_LINGTU_DEFAULT_SMALL_GICP_ROOT
    "${_LINGTU_REPO_ROOT}/third_party/research_localization/small_gicp")
  if(EXISTS "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}/include/small_gicp")
    set(SMALL_GICP_ROOT "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}" CACHE PATH
      "Optional small_gicp source or install prefix" FORCE)
  endif()
endif()

find_library(CPU_BBS3D_LIBRARY
  NAMES cpu_bbs3d libcpu_bbs3d.so
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local
)
find_path(CPU_BBS3D_INCLUDE_DIR
  NAMES cpu_bbs3d/bbs3d.hpp
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local /usr/local/include /usr/local/include/bbs3d
)

if(CPU_BBS3D_LIBRARY AND CPU_BBS3D_INCLUDE_DIR)
  message(STATUS "BBS3D enabled: ${CPU_BBS3D_LIBRARY}")
  add_compile_definitions(LINGTU_ENABLE_BBS3D=1)
  include_directories(${CPU_BBS3D_INCLUDE_DIR})
  set(CPU_BBS3D_LINK ${CPU_BBS3D_LIBRARY})
else()
  if(LINGTU_REQUIRE_BBS3D)
    message(FATAL_ERROR
      "LINGTU_REQUIRE_BBS3D=ON but cpu_bbs3d was not found. "
      "Set CPU_BBS3D_ROOT to the install prefix containing include/cpu_bbs3d/bbs3d.hpp and lib/libcpu_bbs3d.so.")
  endif()
  message(WARNING "BBS3D not found; building ICP-only localizer without global_relocalize")
  set(CPU_BBS3D_LINK "")
endif()
find_path(SMALL_GICP_INCLUDE_DIR
  NAMES small_gicp/pcl/pcl_registration.hpp
  HINTS ${SMALL_GICP_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local)
if(LINGTU_ENABLE_SMALL_GICP AND SMALL_GICP_INCLUDE_DIR AND OpenMP_CXX_FOUND)
  message(STATUS "small_gicp relocalization refine enabled: ${SMALL_GICP_INCLUDE_DIR}")
else()
  message(STATUS "small_gicp not found, disabled, or OpenMP unavailable; using PCL GICP refine")
  set(SMALL_GICP_INCLUDE_DIR "")
endif()

set(SRC_LIST
  src/localizers/commons.cpp
  src/localizers/icp_localizer.cpp
  src/localizers/bbs3d_global_localizer.cpp
)

add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
ament_target_dependencies(
  localizer_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  tf2_ros
  geometry_msgs
  interface
  std_srvs
)
target_link_libraries(
  localizer_node
  ${PCL_LIBRARIES}
  yaml-cpp
  ${CPU_BBS3D_LINK}
  OpenMP::OpenMP_CXX
)
target_include_directories(localizer_node PRIVATE ${SMALL_GICP_INCLUDE_DIR})
if(SMALL_GICP_INCLUDE_DIR)
  target_compile_definitions(localizer_node PRIVATE LINGTU_ENABLE_SMALL_GICP=1)
endif()

install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../launch/localizer_launch.py DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
