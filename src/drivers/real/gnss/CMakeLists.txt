cmake_minimum_required(VERSION 3.10)

project(lingtu_gnss_dds LANGUAGES C CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CycloneDDS REQUIRED)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")

set(_LINGTU_SLAM_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
include("${_LINGTU_ROOT}/cmake/LingTuDDS.cmake")
lingtu_add_dds_c_messages(lingtu_gnss_dds_messages "${_LINGTU_SLAM_IDL}")

add_executable(lingtu_gnss_dds
  native/gnss_dds.cpp
  native/module.cpp
  native/dds_module.cpp
  impl/wtrtk980/nmea.cpp
  impl/wtrtk980/serial.cpp)

target_include_directories(lingtu_gnss_dds PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/impl"
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_gnss_dds PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_contracts
  lingtu_gnss_dds_messages)

if(UNIX)
  target_compile_options(lingtu_gnss_dds PRIVATE -Wall -Wextra)
  target_link_libraries(lingtu_gnss_dds PRIVATE pthread)
endif()

install(TARGETS lingtu_gnss_dds
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)
