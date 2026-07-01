cmake_minimum_required(VERSION 3.10)

project(lingtu_livox_sdk2_stream LANGUAGES CXX)

option(LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS "Build native CycloneDDS publisher mode" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING
  "Compatibility floor for vendored Livox-SDK2 CMake files")

set(LIVOX_SDK2_DIR "${CMAKE_CURRENT_LIST_DIR}/../Livox-SDK2" CACHE PATH "Livox SDK2 source directory")

add_subdirectory("${LIVOX_SDK2_DIR}/sdk_core" "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2_core")

add_executable(livox_sdk2_stream main.cpp)
target_include_directories(livox_sdk2_stream PRIVATE "${LIVOX_SDK2_DIR}/include")
target_link_libraries(livox_sdk2_stream PRIVATE livox_lidar_sdk_static)

if(LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS-CXX REQUIRED)
  find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

  set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../..")
  set(_LINGTU_SLAM_IDL "${_LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
  set(_LINGTU_SLAM_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.hpp")
  add_custom_command(
    OUTPUT "${_LINGTU_SLAM_IDL_HEADER}"
    COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}"
            -l "$<TARGET_FILE:CycloneDDS-CXX::idlcxx>"
            "${_LINGTU_SLAM_IDL}"
    DEPENDS "${_LINGTU_SLAM_IDL}" CycloneDDS-CXX::idlcxx
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    VERBATIM)
  add_custom_target(livox_sdk2_stream_idl_generate
    DEPENDS "${_LINGTU_SLAM_IDL_HEADER}")
  add_dependencies(livox_sdk2_stream livox_sdk2_stream_idl_generate)
  target_compile_definitions(livox_sdk2_stream PRIVATE
    LINGTU_LIVOX_SDK2_STREAM_HAS_DDS=1)
  target_include_directories(livox_sdk2_stream PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(livox_sdk2_stream PRIVATE CycloneDDS-CXX::ddscxx)
endif()

if(UNIX)
  target_compile_options(livox_sdk2_stream PRIVATE -Wall -Wextra)
  target_link_libraries(livox_sdk2_stream PRIVATE pthread)
endif()
