cmake_minimum_required(VERSION 3.16)
project(lingtu_native_recording LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LINGTU_RECORDING_BUILD_DDS "Build CycloneDDS recorder/player binaries" ON)
option(LINGTU_RECORDING_BUILD_TESTS "Build native recording tests" ON)

if(LINGTU_RECORDING_BUILD_TESTS)
  enable_testing()
endif()

set(LINGTU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(LINGTU_RECORDING_TESTS_DIR "${LINGTU_ROOT}/tests/native/recording")

add_library(lingtu_recording_core STATIC
  src/camera_recording.cpp
  src/inspection_timeline.cpp
  src/mcap_session.cpp
  src/mcap_vendor.cpp
  src/recording_catalog.cpp
  src/recording_manager.cpp
  src/recording_core.cpp)
target_include_directories(lingtu_recording_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/mcap/include")
target_compile_definitions(lingtu_recording_core PUBLIC
  MCAP_COMPRESSION_NO_LZ4
  MCAP_COMPRESSION_NO_ZSTD)
if(WIN32)
  # This vendored one-TU MCAP implementation is linked statically. Its default
  # Windows visibility macro assumes a DLL and breaks MinGW/clang static links.
  target_compile_definitions(lingtu_recording_core PUBLIC MCAP_PUBLIC=)
endif()

add_executable(lingtu_camera_player src/camera_player_main.cpp)
target_link_libraries(lingtu_camera_player PRIVATE lingtu_recording_core)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(Threads REQUIRED)
  add_library(lingtu_recording_process_linux STATIC src/recording_linux.cpp)
  target_link_libraries(lingtu_recording_process_linux PUBLIC
    lingtu_recording_core
    Threads::Threads)

  add_library(lingtu_recording_camera_linux STATIC src/camera_linux.cpp)
  target_include_directories(lingtu_recording_camera_linux PRIVATE
    "${LINGTU_ROOT}/src")
  target_link_libraries(lingtu_recording_camera_linux PUBLIC
    lingtu_recording_core
    Threads::Threads
    rt)

  add_executable(lingtu_camera_recorder src/camera_recorder_main.cpp)
  target_link_libraries(lingtu_camera_recorder PRIVATE
    lingtu_recording_camera_linux)
endif()

if(LINGTU_RECORDING_BUILD_DDS)
  find_package(CycloneDDS REQUIRED)
  include("${LINGTU_ROOT}/src/message/cpp/CMakeLists.txt")
  lingtu_add_dds_c_messages(
    lingtu_recording_dds_messages
    "${LINGTU_ROOT}/src/message/idl/messages.idl")

  add_library(lingtu_recording_dds STATIC
    src/cyclone_cdr.cpp
    src/inspection_timeline_cdr.cpp
    src/topic_catalog.cpp)
  target_include_directories(lingtu_recording_dds PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${LINGTU_ROOT}/src")
  target_link_libraries(lingtu_recording_dds PUBLIC
    CycloneDDS::ddsc
    lingtu_dds_contracts
    lingtu_recording_core
    lingtu_recording_dds_messages)

  add_executable(lingtu_dds_recorder src/recorder_main.cpp)
  target_link_libraries(lingtu_dds_recorder PRIVATE
    lingtu_recording_dds
    Threads::Threads)
  target_compile_definitions(lingtu_dds_recorder PRIVATE
    LINGTU_RECORDING_DEFAULT_IDL="${LINGTU_ROOT}/src/message/idl/messages.idl")

  add_executable(lingtu_dds_player src/player_main.cpp)
  target_link_libraries(lingtu_dds_player PRIVATE lingtu_recording_dds)
  target_compile_definitions(lingtu_dds_player PRIVATE
    LINGTU_RECORDING_DEFAULT_IDL="${LINGTU_ROOT}/src/message/idl/messages.idl")

  if(LINGTU_RECORDING_BUILD_TESTS)
    add_executable(test_cyclone_cdr "${LINGTU_RECORDING_TESTS_DIR}/test_cyclone_cdr.cpp")
    target_link_libraries(test_cyclone_cdr PRIVATE lingtu_recording_dds)
    add_test(NAME native_recording_cyclone_cdr COMMAND test_cyclone_cdr)

    add_executable(test_inspection_timeline_cdr
      "${LINGTU_RECORDING_TESTS_DIR}/test_inspection_timeline_cdr.cpp")
    target_link_libraries(test_inspection_timeline_cdr PRIVATE lingtu_recording_dds)
    add_test(NAME native_recording_inspection_timeline_cdr
             COMMAND test_inspection_timeline_cdr)

    add_executable(test_imu_publisher "${LINGTU_RECORDING_TESTS_DIR}/test_imu_publisher.cpp")
    target_link_libraries(test_imu_publisher PRIVATE lingtu_recording_dds)

    add_executable(test_imu_subscriber "${LINGTU_RECORDING_TESTS_DIR}/test_imu_subscriber.cpp")
    target_link_libraries(test_imu_subscriber PRIVATE lingtu_recording_dds)
    add_test(
      NAME native_recording_cli_flow
      COMMAND bash "${LINGTU_RECORDING_TESTS_DIR}/test_cli_flow.sh"
        "$<TARGET_FILE:lingtu_dds_recorder>"
        "$<TARGET_FILE:lingtu_dds_player>"
        "$<TARGET_FILE:test_imu_publisher>"
        "$<TARGET_FILE:test_imu_subscriber>"
        "${LINGTU_ROOT}/src/message/idl/messages.idl")
  endif()
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(lingtu_recorder src/lingtu_recorder_main.cpp)
  target_link_libraries(lingtu_recorder PRIVATE
    lingtu_recording_process_linux)
  target_compile_definitions(lingtu_recorder PRIVATE
    LINGTU_RECORDING_DEFAULT_IDL="${LINGTU_ROOT}/src/message/idl/messages.idl")
  if(LINGTU_RECORDING_BUILD_DDS)
    target_compile_definitions(lingtu_recorder PRIVATE LINGTU_RECORDING_HAS_DDS=1)
    add_dependencies(lingtu_recorder lingtu_dds_recorder)
  else()
    target_compile_definitions(lingtu_recorder PRIVATE LINGTU_RECORDING_HAS_DDS=0)
  endif()
  add_dependencies(lingtu_recorder lingtu_camera_recorder)

  if(LINGTU_RECORDING_BUILD_TESTS)
    add_test(NAME native_recorder_help COMMAND lingtu_recorder --help)
    add_test(
      NAME native_recorder_dry_run
      COMMAND lingtu_recorder record
        --output-dir "$<TARGET_FILE_DIR:lingtu_recorder>/lingtu-dry-run-session"
        --dds off
        --camera on
        --dry-run)

    add_executable(test_recording_worker
      "${LINGTU_RECORDING_TESTS_DIR}/test_recording_worker.cpp")
    add_test(
      NAME native_recorder_control_flow
      COMMAND bash "${LINGTU_RECORDING_TESTS_DIR}/test_recorder_control.sh"
        "$<TARGET_FILE:lingtu_recorder>"
        "$<TARGET_FILE:test_recording_worker>")
    set_tests_properties(native_recorder_control_flow PROPERTIES TIMEOUT 20)
  endif()
endif()

if(LINGTU_RECORDING_BUILD_TESTS)
  add_test(NAME native_camera_player_help COMMAND lingtu_camera_player --help)
  if(TARGET lingtu_camera_recorder)
    add_test(NAME native_camera_recorder_help COMMAND lingtu_camera_recorder --help)

    add_executable(test_camera_linux "${LINGTU_RECORDING_TESTS_DIR}/test_camera_linux.cpp")
    target_include_directories(test_camera_linux PRIVATE "${LINGTU_ROOT}/src")
    target_link_libraries(test_camera_linux PRIVATE lingtu_recording_camera_linux)
    add_test(NAME native_camera_linux COMMAND test_camera_linux)

    # This stays an external runtime check: no FFmpeg headers or libraries are
    # linked into the recorder. When a generic FFmpeg executable is available,
    # prove that the supported software default can encode and decode a real MKV.
    find_program(LINGTU_RECORDING_FFMPEG_EXECUTABLE NAMES ffmpeg)
    if(LINGTU_RECORDING_FFMPEG_EXECUTABLE)
      add_executable(test_camera_ffmpeg_software
        "${LINGTU_RECORDING_TESTS_DIR}/test_camera_ffmpeg_software.cpp")
      target_link_libraries(test_camera_ffmpeg_software PRIVATE lingtu_recording_camera_linux)
      add_test(
        NAME native_camera_software_ffmpeg
        COMMAND test_camera_ffmpeg_software "${LINGTU_RECORDING_FFMPEG_EXECUTABLE}")
      set_tests_properties(native_camera_software_ffmpeg PROPERTIES
        LABELS "integration;software-ffmpeg"
        TIMEOUT 20)
    else()
      message(STATUS "FFmpeg not found; skipping optional software camera integration test")
    endif()

    add_executable(test_recording_process_linux
      "${LINGTU_RECORDING_TESTS_DIR}/test_recording_process_linux.cpp")
    target_link_libraries(test_recording_process_linux PRIVATE
      lingtu_recording_process_linux)
    add_test(NAME native_recording_process_linux COMMAND test_recording_process_linux)
  endif()

  add_executable(test_camera_recording
    "${LINGTU_RECORDING_TESTS_DIR}/test_camera_recording.cpp")
  target_link_libraries(test_camera_recording PRIVATE lingtu_recording_core)
  add_test(NAME native_camera_recording COMMAND test_camera_recording)

  add_executable(test_recording_core
    "${LINGTU_RECORDING_TESTS_DIR}/test_recording_core.cpp")
  target_link_libraries(test_recording_core PRIVATE lingtu_recording_core)
  add_test(NAME native_recording_core COMMAND test_recording_core)

  add_executable(test_recording_manager
    "${LINGTU_RECORDING_TESTS_DIR}/test_recording_manager.cpp")
  target_link_libraries(test_recording_manager PRIVATE lingtu_recording_core)
  add_test(NAME native_recording_manager COMMAND test_recording_manager)

  add_executable(test_recording_catalog
    "${LINGTU_RECORDING_TESTS_DIR}/test_recording_catalog.cpp")
  target_link_libraries(test_recording_catalog PRIVATE lingtu_recording_core)
  add_test(NAME native_recording_catalog COMMAND test_recording_catalog)
endif()

# Keep the operator-facing tools together so deployment can install and verify
# one flat, relocatable recording toolset. Reduced builds install only targets
# that are available for the selected platform and DDS option.
set(LINGTU_RECORDING_EXECUTABLES lingtu_camera_player)
foreach(recording_target IN ITEMS
    lingtu_recorder
    lingtu_dds_recorder
    lingtu_dds_player
    lingtu_camera_recorder)
  if(TARGET ${recording_target})
    list(APPEND LINGTU_RECORDING_EXECUTABLES ${recording_target})
  endif()
endforeach()
install(TARGETS ${LINGTU_RECORDING_EXECUTABLES}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)
