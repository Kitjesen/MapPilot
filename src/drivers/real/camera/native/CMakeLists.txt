cmake_minimum_required(VERSION 3.16)
project(lingtu_camera_dds LANGUAGES C CXX)

if(NOT UNIX)
  message(FATAL_ERROR "lingtu_camera_dds is a Linux robot-side runtime")
endif()

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

add_subdirectory(
  "${_LINGTU_ROOT}/src/message/cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

add_executable(lingtu_camera_dds
  camera_dds.cpp
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_camera_dds PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_camera_dds PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)

target_compile_options(lingtu_camera_dds PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
