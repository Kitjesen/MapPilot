cmake_minimum_required(VERSION 3.10)

project(lingtu_livox_sdk2_stream LANGUAGES C CXX)

option(LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS "Build native CycloneDDS publisher mode" OFF)
option(LINGTU_LIVOX_SDK2_STREAM_BUILD_TESTS "Build Livox stream unit tests" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING
  "Compatibility floor for vendored Livox-SDK2 CMake files")

set(LIVOX_SDK2_DIR "${CMAKE_CURRENT_LIST_DIR}/../deps/livox/Livox-SDK2" CACHE PATH "Livox SDK2 source directory")

add_subdirectory("${LIVOX_SDK2_DIR}/sdk_core" "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2_core")

add_executable(livox_sdk2_stream main.cpp)
target_include_directories(livox_sdk2_stream PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/.."
  "${LIVOX_SDK2_DIR}/include")
target_link_libraries(livox_sdk2_stream PRIVATE livox_lidar_sdk_static)

if(LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS REQUIRED)
  find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

  set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../..")

  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")
  set(_LINGTU_SLAM_IDL "${_LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
  set(_LINGTU_SLAM_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.h")
  set(_LINGTU_SLAM_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.c")
  add_custom_command(
    OUTPUT "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}"
    COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}"
            -l c
            "${_LINGTU_SLAM_IDL}"
    DEPENDS "${_LINGTU_SLAM_IDL}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    VERBATIM)
  add_custom_target(livox_sdk2_stream_idl_generate
    DEPENDS "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}")
  add_dependencies(livox_sdk2_stream livox_sdk2_stream_idl_generate)
  target_sources(livox_sdk2_stream PRIVATE
    "${_LINGTU_SLAM_IDL_SOURCE}"
    "${CMAKE_CURRENT_LIST_DIR}/../native/dds_module.cpp")
  target_compile_definitions(livox_sdk2_stream PRIVATE
    LINGTU_LIVOX_SDK2_STREAM_HAS_DDS=1)
  target_include_directories(livox_sdk2_stream PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(livox_sdk2_stream PRIVATE CycloneDDS::ddsc lingtu_dds_qos_profiles)
endif()

if(UNIX)
  target_compile_options(livox_sdk2_stream PRIVATE -Wall -Wextra)
  target_link_libraries(livox_sdk2_stream PRIVATE pthread)
endif()

if(LINGTU_LIVOX_SDK2_STREAM_BUILD_TESTS)
  enable_testing()
  add_executable(
    test_livox_fallback_timestamp_clock
    tests/test_fallback_timestamp_clock.cpp)
  target_include_directories(
    test_livox_fallback_timestamp_clock
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  if(UNIX)
    target_compile_options(
      test_livox_fallback_timestamp_clock
      PRIVATE -Wall -Wextra -Werror)
  endif()
  add_test(
    NAME test_livox_fallback_timestamp_clock
    COMMAND test_livox_fallback_timestamp_clock)

  add_executable(
    test_livox_packet_timestamp_clock
    tests/test_packet_timestamp_clock.cpp)
  target_include_directories(
    test_livox_packet_timestamp_clock
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  if(UNIX)
    target_compile_options(
      test_livox_packet_timestamp_clock
      PRIVATE -Wall -Wextra -Werror)
  endif()
  add_test(
    NAME test_livox_packet_timestamp_clock
    COMMAND test_livox_packet_timestamp_clock)

  add_executable(
    test_livox_single_lidar_handle_guard
    tests/test_single_lidar_handle_guard.cpp)
  target_include_directories(
    test_livox_single_lidar_handle_guard
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  if(UNIX)
    target_compile_options(
      test_livox_single_lidar_handle_guard
      PRIVATE -Wall -Wextra -Werror)
  endif()
  add_test(
    NAME test_livox_single_lidar_handle_guard
    COMMAND test_livox_single_lidar_handle_guard)

  add_executable(
    test_livox_stream_exit_state
    tests/test_stream_exit_state.cpp)
  target_include_directories(
    test_livox_stream_exit_state
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  if(UNIX)
    target_compile_options(
      test_livox_stream_exit_state
      PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(test_livox_stream_exit_state PRIVATE pthread)
  endif()
  add_test(
    NAME test_livox_stream_exit_state
    COMMAND test_livox_stream_exit_state)

  add_executable(
    test_livox_replay_deadline_restamper
    tests/test_replay_deadline_restamper.cpp)
  target_include_directories(
    test_livox_replay_deadline_restamper
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  if(UNIX)
    target_compile_options(
      test_livox_replay_deadline_restamper
      PRIVATE -Wall -Wextra -Werror)
  endif()
  add_test(
    NAME test_livox_replay_deadline_restamper
    COMMAND test_livox_replay_deadline_restamper)
endif()
