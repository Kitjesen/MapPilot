cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_cyclone_endpoint LANGUAGES C CXX)

if(NOT UNIX)
  message(FATAL_ERROR "lingtu_nav_cyclone_endpoint is a Linux robot-side runtime")
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

add_custom_command(
  OUTPUT "${_LINGTU_IDL_HEADER}" "${_LINGTU_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${_LINGTU_IDL}"
  DEPENDS "${_LINGTU_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

add_executable(lingtu_nav_cyclone_endpoint
  nav_cyclone_endpoint.cpp
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_nav_cyclone_endpoint PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_LINGTU_ROOT}/src/drivers/real/lidar/Livox-SDK2/3rdparty")

target_link_libraries(lingtu_nav_cyclone_endpoint PRIVATE
  CycloneDDS::ddsc
  pthread)

target_compile_options(lingtu_nav_cyclone_endpoint PRIVATE
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
