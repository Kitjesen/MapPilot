cmake_minimum_required(VERSION 3.16)
project(lingtu_mujoco_native_dds LANGUAGES C CXX)

if(NOT UNIX)
  message(FATAL_ERROR "MuJoCo native DDS helpers are Linux-side processes")
endif()

find_package(CycloneDDS REQUIRED)
find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(LINGTU_IDL "${LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
set(LINGTU_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.h")
set(LINGTU_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.c")

add_custom_command(
  OUTPUT "${LINGTU_IDL_HEADER}" "${LINGTU_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${LINGTU_IDL}"
  DEPENDS "${LINGTU_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

add_subdirectory(
  "${LINGTU_ROOT}/src/message/cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

add_executable(lingtu_mujoco_cmd_vel_tap
  cmd_vel_tap.cpp
  "${LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_mujoco_cmd_vel_tap PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${LINGTU_ROOT}/src")

target_link_libraries(lingtu_mujoco_cmd_vel_tap PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles)

target_compile_options(lingtu_mujoco_cmd_vel_tap PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
