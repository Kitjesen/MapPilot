cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_endpoint LANGUAGES C CXX)

if(NOT UNIX)
  message(FATAL_ERROR "lingtu_nav_endpoint is a Linux robot-side runtime")
endif()

find_package(iceoryx_binding_c QUIET)
find_package(CycloneDDS REQUIRED)
find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../..")
set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
set(_LINGTU_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.h")
set(_LINGTU_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.c")
set(_NAV_PLAN_CPP_DIR "${_LINGTU_ROOT}/src/nav/services/plan/cpp")
set(_LOCAL_PLANNER_CPP_DIR "${_LINGTU_ROOT}/src/nav/services/plan/local_planner/cpp")
set(_NAV_KERNEL_DIR "${_LINGTU_ROOT}/src/nav/kernel")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_LINGTU_ROOT}/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/runtime")

add_custom_command(
  OUTPUT "${_LINGTU_IDL_HEADER}" "${_LINGTU_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${_LINGTU_IDL}"
  DEPENDS "${_LINGTU_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

add_subdirectory(
  "${_OCTOPLANNER3D_RUNTIME_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/octoplanner3d_runtime")

add_executable(lingtu_nav_native_endpoint
  nav_native_endpoint.cpp
  "${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"
  "${_NAV_KERNEL_DIR}/src/path_follower_core.cpp"
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_nav_native_endpoint PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_PLAN_CPP_DIR}"
  "${_LOCAL_PLANNER_CPP_DIR}"
  "${_NAV_KERNEL_DIR}/include"
  "${_OCTOPLANNER3D_RUNTIME_DIR}")

target_link_libraries(lingtu_nav_native_endpoint PRIVATE
  octoplanner3d_runtime
  CycloneDDS::ddsc
  pthread)

target_compile_options(lingtu_nav_native_endpoint PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

add_executable(lingtu_traversability_dds
  traversability_dds.cpp
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_traversability_dds PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_KERNEL_DIR}/include")

target_link_libraries(lingtu_traversability_dds PRIVATE
  CycloneDDS::ddsc
  pthread)

target_compile_options(lingtu_traversability_dds PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

add_executable(lingtu_nav_control
  nav_control.cpp
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_nav_control PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_nav_control PRIVATE
  CycloneDDS::ddsc
  pthread)

target_compile_options(lingtu_nav_control PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
