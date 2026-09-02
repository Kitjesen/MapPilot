cmake_minimum_required(VERSION 3.10)

project(lingtu_gnss_dds LANGUAGES C CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CycloneDDS REQUIRED)
find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")

add_subdirectory(
  "${_LINGTU_ROOT}/src/message/cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

set(_LINGTU_SLAM_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
set(_LINGTU_SLAM_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/messages.h")
set(_LINGTU_SLAM_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/messages.c")

add_custom_command(
  OUTPUT "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}"
          -l c
          "${_LINGTU_SLAM_IDL}"
  DEPENDS "${_LINGTU_SLAM_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

add_custom_target(lingtu_gnss_dds_idl_generate
  DEPENDS "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}")

add_executable(lingtu_gnss_dds
  native/gnss_dds.cpp
  native/module.cpp
  native/dds_module.cpp
  impl/wtrtk980/nmea.cpp
  impl/wtrtk980/serial.cpp
  "${_LINGTU_SLAM_IDL_SOURCE}")

add_dependencies(lingtu_gnss_dds lingtu_gnss_dds_idl_generate)

target_include_directories(lingtu_gnss_dds PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/impl"
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_gnss_dds PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_contracts)

if(UNIX)
  target_compile_options(lingtu_gnss_dds PRIVATE -Wall -Wextra)
  target_link_libraries(lingtu_gnss_dds PRIVATE pthread)
endif()

install(TARGETS lingtu_gnss_dds
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)
