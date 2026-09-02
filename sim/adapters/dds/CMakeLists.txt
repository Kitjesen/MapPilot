cmake_minimum_required(VERSION 3.16)
project(lingtu_mujoco_native_dds LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

if(MSVC)
  set(_LINGTU_MUJOCO_WARNINGS /W4)
  set(_LINGTU_LOCAL_ENDPOINT_WARNINGS /W4 /WX)
else()
  set(_LINGTU_MUJOCO_WARNINGS -Wall -Wextra -Wpedantic)
  set(_LINGTU_LOCAL_ENDPOINT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
endif()

function(lingtu_mujoco_bundle_cyclonedds target)
  if(WIN32)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:CycloneDDS::ddsc>
        $<TARGET_FILE_DIR:${target}>
      COMMENT "Bundling the CycloneDDS runtime DLL for ${target}"
      VERBATIM)
  endif()
endfunction()

get_filename_component(LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_LINGTU_MUJOCO_TESTS_DIR "${LINGTU_ROOT}/tests/sim/adapters/dds")
set(_LINGTU_REAL_DRIVER_DIR
  "${LINGTU_ROOT}/src/drivers/real/motion")

# The simulation bridge reuses only the portable real-driver algorithms.
# Keep Brainstem, gRPC, Protobuf, and the driver's DDS runtime out of that
# imported sub-build; the explicit adapter targets below own their DDS seam.
set(LINGTU_DRIVER_BUILD_RUNTIME OFF CACHE BOOL
  "Build only the portable real-driver core for the MuJoCo bridge" FORCE)
add_subdirectory(
  "${_LINGTU_REAL_DRIVER_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/driver_core")

add_library(lingtu_mujoco_driver_bridge_core STATIC
  mujoco_driver_bridge_core.cpp
  mujoco_driver_bridge_protocol.cpp)
target_include_directories(lingtu_mujoco_driver_bridge_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(lingtu_mujoco_driver_bridge_core PUBLIC
  lingtu_driver_core)
target_compile_options(lingtu_mujoco_driver_bridge_core PRIVATE
  ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})

add_library(lingtu_mujoco_sensor_record_protocol STATIC
  sensor_record_protocol.cpp)
target_include_directories(lingtu_mujoco_sensor_record_protocol PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${LINGTU_ROOT}/src/drivers/real/camera/native"
  "${LINGTU_ROOT}/src/drivers/real/lidar")
target_compile_options(lingtu_mujoco_sensor_record_protocol PRIVATE
  ${_LINGTU_MUJOCO_WARNINGS})

add_library(lingtu_truth_odom_protocol STATIC
  truth_odom_protocol.cpp)
target_include_directories(lingtu_truth_odom_protocol PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_truth_odom_protocol PRIVATE
  ${_LINGTU_MUJOCO_WARNINGS})

add_library(lingtu_sim_imu_protocol STATIC
  imu_protocol.cpp)
target_include_directories(lingtu_sim_imu_protocol PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_sim_imu_protocol PRIVATE
  ${_LINGTU_MUJOCO_WARNINGS})

add_library(lingtu_local_endpoint_server STATIC
  local_endpoint_server.cpp)
target_include_directories(lingtu_local_endpoint_server PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_local_endpoint_server PRIVATE
  ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
if(WIN32)
  target_link_libraries(lingtu_local_endpoint_server PUBLIC
    ws2_32 crypt32 bcrypt)
endif()

add_library(lingtu_run_plan_process_environment STATIC
  run_plan_process_environment.cpp)
target_include_directories(lingtu_run_plan_process_environment PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_run_plan_process_environment PRIVATE
  ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  add_executable(test_run_plan_process_environment
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_run_plan_process_environment.cpp")
  target_include_directories(test_run_plan_process_environment PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(test_run_plan_process_environment PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
  target_link_libraries(test_run_plan_process_environment PRIVATE
    lingtu_run_plan_process_environment)
  add_test(
    NAME test_run_plan_process_environment
    COMMAND test_run_plan_process_environment)
  set_tests_properties(test_run_plan_process_environment PROPERTIES TIMEOUT 10)

  add_executable(test_local_endpoint_server
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_local_endpoint_server.cpp")
  target_link_libraries(test_local_endpoint_server PRIVATE
    lingtu_local_endpoint_server
    Threads::Threads)
  target_compile_options(test_local_endpoint_server PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
  add_test(
    NAME test_local_endpoint_server
    COMMAND test_local_endpoint_server)
  set_tests_properties(test_local_endpoint_server PROPERTIES TIMEOUT 20)

  if(Python3_Interpreter_FOUND)
    set(_LINGTU_LOCAL_ENDPOINT_PYTHON_COMPAT
      "${CMAKE_CURRENT_BINARY_DIR}/test_local_endpoint_python_compat.py")
    file(WRITE "${_LINGTU_LOCAL_ENDPOINT_PYTHON_COMPAT}" [=[
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path


executable = Path(sys.argv[1])
repository = Path(sys.argv[2])
sys.path.insert(0, str(repository / "src"))
sys.path.insert(0, str(repository))

from sim.scripts.mujoco.native_runtime_endpoint import (
    _connect_authenticated,
    _load_auth_key,
    load_local_endpoint,
)


product_session_id = "7" * 63
with tempfile.TemporaryDirectory(prefix="lingtu-cpp-endpoint-") as temporary:
    directory = Path(temporary).resolve()
    readiness = directory / "python.ready.json"
    process = subprocess.Popen(
        [str(executable), "--serve-python-compat", str(directory)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert process.stdout is not None
        assert process.stdout.readline() == "READY\n"
        endpoint = load_local_endpoint(
            readiness,
            role="driver_bridge",
            protocol="line-v1",
            product_session_id=product_session_id,
        )
        assert endpoint.host == "127.0.0.1"
        assert len(_load_auth_key(endpoint.auth_file)) == 32
        connection, _ = _connect_authenticated(
            readiness,
            role="driver_bridge",
            protocol="line-v1",
            product_session_id=product_session_id,
            timeout_s=2.0,
        )
        try:
            connection.sendall(b"PYTHON\n")
            assert connection.recv(4) == b"CPP\n"
        finally:
            connection.close()
        return_code = process.wait(timeout=5.0)
        assert process.stderr is not None
        error = process.stderr.read()
        assert return_code == 0, error
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)
]=])
    add_test(
      NAME test_local_endpoint_python_compat
      COMMAND
        "${Python3_EXECUTABLE}"
        "${_LINGTU_LOCAL_ENDPOINT_PYTHON_COMPAT}"
        "$<TARGET_FILE:test_local_endpoint_server>"
        "${LINGTU_ROOT}")
    set_tests_properties(
      test_local_endpoint_python_compat
      PROPERTIES TIMEOUT 20)
  endif()

  add_executable(test_mujoco_driver_bridge_core
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_mujoco_driver_bridge_core.cpp")
  target_link_libraries(test_mujoco_driver_bridge_core PRIVATE
    lingtu_mujoco_driver_bridge_core)
  target_compile_options(test_mujoco_driver_bridge_core PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
  add_test(
    NAME test_mujoco_driver_bridge_core
    COMMAND test_mujoco_driver_bridge_core)

  add_executable(test_sensor_record_protocol
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_record_protocol.cpp")
  target_link_libraries(test_sensor_record_protocol PRIVATE
    lingtu_mujoco_sensor_record_protocol)
  target_compile_options(test_sensor_record_protocol PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})
  add_test(
    NAME test_sensor_record_protocol
    COMMAND test_sensor_record_protocol)

  add_executable(test_navigation_fixture_observation
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_navigation_fixture_observation.cpp")
  target_include_directories(test_navigation_fixture_observation PRIVATE
    "${LINGTU_ROOT}/src/drivers/real/lidar")
  target_compile_options(test_navigation_fixture_observation PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
  add_test(
    NAME test_navigation_fixture_observation
    COMMAND test_navigation_fixture_observation)

  add_executable(test_truth_odom_protocol
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_truth_odom_protocol.cpp")
  target_link_libraries(test_truth_odom_protocol PRIVATE
    lingtu_truth_odom_protocol)
  target_compile_options(test_truth_odom_protocol PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})
  add_test(
    NAME test_truth_odom_protocol
    COMMAND test_truth_odom_protocol)

  add_executable(test_imu_protocol
    "${_LINGTU_MUJOCO_TESTS_DIR}/test_imu_protocol.cpp")
  target_link_libraries(test_imu_protocol PRIVATE lingtu_sim_imu_protocol)
  target_compile_options(test_imu_protocol PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})
  add_test(
    NAME test_imu_protocol
    COMMAND test_imu_protocol)
endif()

if(UNIX)
  set(_LINGTU_MUJOCO_NATIVE_DDS_RUNTIME_DEFAULT ON)
else()
  set(_LINGTU_MUJOCO_NATIVE_DDS_RUNTIME_DEFAULT OFF)
endif()
option(
  LINGTU_MUJOCO_NATIVE_DDS_BUILD_RUNTIME
  "Build the native CycloneDDS MuJoCo helper executables"
  ${_LINGTU_MUJOCO_NATIVE_DDS_RUNTIME_DEFAULT})

if(LINGTU_MUJOCO_NATIVE_DDS_BUILD_RUNTIME)
  get_property(
    _LINGTU_MUJOCO_TARGETS_BEFORE_DDS_RUNTIME
    DIRECTORY
    PROPERTY BUILDSYSTEM_TARGETS)
  find_package(CycloneDDS REQUIRED)

  # Windows runtime closure is app-local. Tests must not pass only because the
  # developer shell happened to contain a CycloneDDS SDK on PATH.
  set(_LINGTU_CYCLONEDDS_TEST_ENV "")

  add_subdirectory(
    "${LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")
  lingtu_add_dds_c_messages(
    lingtu_mujoco_dds_messages
    "${LINGTU_ROOT}/src/message/idl/messages.idl")

  if(WIN32)
    # All runtime-block executables share one app-local DLL directory. This
    # makes targeted builds and direct launches behave like the packaged app.
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  endif()

  add_executable(lingtu_mujoco_driver_bridge
    mujoco_driver_bridge.cpp)
  target_include_directories(lingtu_mujoco_driver_bridge PRIVATE
    "${LINGTU_ROOT}/src")
  target_link_libraries(lingtu_mujoco_driver_bridge PRIVATE
    lingtu_mujoco_driver_bridge_core
    lingtu_local_endpoint_server
    lingtu_run_plan_process_environment
    lingtu_mujoco_dds_messages
    lingtu_dds_contracts)
  target_compile_options(lingtu_mujoco_driver_bridge PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})

  add_executable(lingtu_mujoco_sensor_publisher
    sensor_publisher.cpp
    "${LINGTU_ROOT}/src/drivers/real/lidar/native/dds_module.cpp")
  if(MSVC)
    # The reused field DDS adapter still uses portable std::getenv.  Keep the
    # endpoint translation unit under /WX while suppressing only that known
    # MSVC deprecation in the imported source file.
    set_source_files_properties(
      "${LINGTU_ROOT}/src/drivers/real/lidar/native/dds_module.cpp"
      PROPERTIES COMPILE_OPTIONS "/wd4996")
  endif()
  target_include_directories(lingtu_mujoco_sensor_publisher PRIVATE
    "${LINGTU_ROOT}/src"
    "${LINGTU_ROOT}/src/drivers/real/lidar"
    "${LINGTU_ROOT}/src/drivers/real/lidar/sdk2_stream")
  target_link_libraries(lingtu_mujoco_sensor_publisher PRIVATE
    lingtu_mujoco_sensor_record_protocol
    lingtu_local_endpoint_server
    lingtu_run_plan_process_environment
    lingtu_mujoco_dds_messages
    lingtu_dds_contracts)
  target_compile_options(lingtu_mujoco_sensor_publisher PRIVATE
    ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})

  add_executable(lingtu_truth_odom_publisher
    truth_odom_publisher.cpp)
  target_include_directories(lingtu_truth_odom_publisher PRIVATE
    "${LINGTU_ROOT}/src")
  target_link_libraries(lingtu_truth_odom_publisher PRIVATE
    lingtu_truth_odom_protocol
    lingtu_mujoco_dds_messages
    lingtu_dds_contracts)
  target_compile_options(lingtu_truth_odom_publisher PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})

  add_executable(lingtu_imu_publisher
    imu_publisher.cpp)
  target_include_directories(lingtu_imu_publisher PRIVATE
    "${LINGTU_ROOT}/src")
  target_link_libraries(lingtu_imu_publisher PRIVATE
    lingtu_sim_imu_protocol
    lingtu_mujoco_dds_messages
    lingtu_dds_contracts)
  target_compile_options(lingtu_imu_publisher PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})

  # Kept as the existing diagnostic tap. It is not the C2 bridge runtime.
  add_executable(lingtu_mujoco_cmd_vel_tap
    cmd_vel_tap.cpp)

  target_include_directories(lingtu_mujoco_cmd_vel_tap PRIVATE
    "${LINGTU_ROOT}/src")

  target_link_libraries(lingtu_mujoco_cmd_vel_tap PRIVATE
    lingtu_mujoco_dds_messages
    lingtu_dds_contracts)

  target_compile_options(lingtu_mujoco_cmd_vel_tap PRIVATE
    ${_LINGTU_MUJOCO_WARNINGS})

  if(BUILD_TESTING)
    set(_LINGTU_WINDOWS_PRODUCT_DDS_DOMAIN 17)
    set(_LINGTU_TEST_DDS_DOMAIN_CAMERA_PAYLOAD_RATE 8)
    set(_LINGTU_TEST_DDS_DOMAIN_SENSOR_RATE 9)
    set(_LINGTU_TEST_DDS_DOMAIN_LOOPBACK 10)
    set(_LINGTU_TEST_DDS_DOMAIN_TRUTH_ODOM 11)
    set(_LINGTU_TEST_DDS_DOMAIN_IMU 12)
    set(_LINGTU_TEST_DDS_DOMAIN_SENSOR 13)
    set(_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT 14)
    set(_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT_FAILURE 15)
    set(_LINGTU_TEST_DDS_DOMAIN_CMD_VEL_TAP 16)
    set(_LINGTU_TEST_DDS_DOMAIN_DRIVER_BRIDGE 18)
    set(_LINGTU_TEST_DDS_DOMAIN_DRIVER_ENDPOINT 19)
    set(_LINGTU_FIRST_NAVIGATION_TEST_DDS_DOMAIN 20)
    set(_LINGTU_TEST_DDS_DOMAINS
      ${_LINGTU_TEST_DDS_DOMAIN_CAMERA_PAYLOAD_RATE}
      ${_LINGTU_TEST_DDS_DOMAIN_SENSOR_RATE}
      ${_LINGTU_TEST_DDS_DOMAIN_LOOPBACK}
      ${_LINGTU_TEST_DDS_DOMAIN_TRUTH_ODOM}
      ${_LINGTU_TEST_DDS_DOMAIN_IMU}
      ${_LINGTU_TEST_DDS_DOMAIN_SENSOR}
      ${_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT}
      ${_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT_FAILURE}
      ${_LINGTU_TEST_DDS_DOMAIN_CMD_VEL_TAP}
      ${_LINGTU_TEST_DDS_DOMAIN_DRIVER_BRIDGE}
      ${_LINGTU_TEST_DDS_DOMAIN_DRIVER_ENDPOINT})
    foreach(_LINGTU_TEST_DDS_DOMAIN IN LISTS _LINGTU_TEST_DDS_DOMAINS)
      if(_LINGTU_TEST_DDS_DOMAIN LESS 0 OR
          _LINGTU_TEST_DDS_DOMAIN GREATER_EQUAL _LINGTU_FIRST_NAVIGATION_TEST_DDS_DOMAIN)
        message(FATAL_ERROR
          "DDS test domain ${_LINGTU_TEST_DDS_DOMAIN} is outside the adapter low-domain slots")
      endif()
      if(_LINGTU_TEST_DDS_DOMAIN EQUAL _LINGTU_WINDOWS_PRODUCT_DDS_DOMAIN)
        message(FATAL_ERROR
          "DDS test domain ${_LINGTU_TEST_DDS_DOMAIN} is reserved for the Windows Product runtime")
      endif()
      math(EXPR _LINGTU_TEST_DDS_HIGHEST_FIXED_PORT
        "7400 + 250 * ${_LINGTU_TEST_DDS_DOMAIN} + 11")
      if(_LINGTU_TEST_DDS_HIGHEST_FIXED_PORT GREATER_EQUAL 49152)
        message(FATAL_ERROR
          "DDS test domain ${_LINGTU_TEST_DDS_DOMAIN} maps CycloneDDS fixed ports into the Windows dynamic port range")
      endif()
    endforeach()
    set(_LINGTU_TEST_DDS_UNIQUE_DOMAINS ${_LINGTU_TEST_DDS_DOMAINS})
    list(REMOVE_DUPLICATES _LINGTU_TEST_DDS_UNIQUE_DOMAINS)
    list(LENGTH _LINGTU_TEST_DDS_DOMAINS _LINGTU_TEST_DDS_DOMAIN_COUNT)
    list(LENGTH _LINGTU_TEST_DDS_UNIQUE_DOMAINS _LINGTU_TEST_DDS_UNIQUE_DOMAIN_COUNT)
    if(NOT _LINGTU_TEST_DDS_DOMAIN_COUNT EQUAL _LINGTU_TEST_DDS_UNIQUE_DOMAIN_COUNT)
      message(FATAL_ERROR "DDS test domains must be unique")
    endif()

    add_executable(test_mujoco_dds_loopback
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_mujoco_dds_loopback.cpp")
    target_include_directories(test_mujoco_dds_loopback PRIVATE
      "${LINGTU_ROOT}/src")
    target_link_libraries(test_mujoco_dds_loopback PRIVATE
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_mujoco_dds_loopback PRIVATE
      ${_LINGTU_MUJOCO_WARNINGS})
    target_compile_definitions(test_mujoco_dds_loopback PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_LOOPBACK})
    add_test(
      NAME test_mujoco_dds_loopback
      COMMAND test_mujoco_dds_loopback)
    set_tests_properties(test_mujoco_dds_loopback PROPERTIES
      TIMEOUT 10
      RESOURCE_LOCK lingtu_mujoco_dds
      ENVIRONMENT
        "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_LOOPBACK}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(
        test_mujoco_dds_loopback
        PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()

    add_executable(test_truth_odom_publisher_dds_process
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_truth_odom_publisher_dds_process.cpp")
    target_include_directories(test_truth_odom_publisher_dds_process PRIVATE
      "${LINGTU_ROOT}/src")
    target_link_libraries(test_truth_odom_publisher_dds_process PRIVATE
      lingtu_truth_odom_protocol
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_truth_odom_publisher_dds_process PRIVATE
      ${_LINGTU_MUJOCO_WARNINGS})
    target_compile_definitions(test_truth_odom_publisher_dds_process PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_TRUTH_ODOM})
    add_dependencies(test_truth_odom_publisher_dds_process
      lingtu_truth_odom_publisher)
    add_test(
      NAME test_truth_odom_publisher_dds_process
      COMMAND
        test_truth_odom_publisher_dds_process
        $<TARGET_FILE:lingtu_truth_odom_publisher>)
    set_tests_properties(
      test_truth_odom_publisher_dds_process
      PROPERTIES
        TIMEOUT 30
        RESOURCE_LOCK lingtu_mujoco_dds
        ENVIRONMENT
          "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_TRUTH_ODOM}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(
        test_truth_odom_publisher_dds_process
        PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()

    add_executable(test_imu_publisher_dds_process
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_imu_publisher_dds_process.cpp")
    target_include_directories(test_imu_publisher_dds_process PRIVATE
      "${LINGTU_ROOT}/src")
    target_link_libraries(test_imu_publisher_dds_process PRIVATE
      lingtu_sim_imu_protocol
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_imu_publisher_dds_process PRIVATE
      ${_LINGTU_MUJOCO_WARNINGS})
    target_compile_definitions(test_imu_publisher_dds_process PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_IMU})
    add_dependencies(test_imu_publisher_dds_process lingtu_imu_publisher)
    add_test(
      NAME test_imu_publisher_dds_process
      COMMAND test_imu_publisher_dds_process
        $<TARGET_FILE:lingtu_imu_publisher>)
    set_tests_properties(test_imu_publisher_dds_process PROPERTIES
      TIMEOUT 30
      RESOURCE_LOCK lingtu_mujoco_dds
      ENVIRONMENT
        "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_IMU}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(test_imu_publisher_dds_process PROPERTIES
        ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()

    add_executable(test_sensor_publisher_dds_process
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_publisher_dds_process.cpp")
    target_include_directories(test_sensor_publisher_dds_process PRIVATE
      "${LINGTU_ROOT}/src"
      "${LINGTU_ROOT}/src/drivers/real/lidar")
    target_link_libraries(test_sensor_publisher_dds_process PRIVATE
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_sensor_publisher_dds_process PRIVATE
      ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
    target_compile_definitions(test_sensor_publisher_dds_process PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR})
    add_dependencies(test_sensor_publisher_dds_process
      lingtu_mujoco_sensor_publisher)
    add_test(
      NAME test_sensor_publisher_dds_process
      COMMAND
        test_sensor_publisher_dds_process
        $<TARGET_FILE:lingtu_mujoco_sensor_publisher>)
    set_tests_properties(
      test_sensor_publisher_dds_process
      PROPERTIES
        TIMEOUT 20
        RESOURCE_LOCK lingtu_mujoco_dds
        ENVIRONMENT
          "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(
        test_sensor_publisher_dds_process
        PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()
    if(Python3_Interpreter_FOUND)
      add_test(
        NAME test_sensor_publisher_endpoint_process
        COMMAND
          "${Python3_EXECUTABLE}"
          "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_publisher_endpoint_process.py"
          $<TARGET_FILE:lingtu_mujoco_sensor_publisher>
          $<TARGET_FILE:test_sensor_publisher_dds_process>
          "${LINGTU_ROOT}")
      set_tests_properties(
        test_sensor_publisher_endpoint_process
        PROPERTIES
          TIMEOUT 45
          RUN_SERIAL TRUE
          RESOURCE_LOCK lingtu_mujoco_dds
          ENVIRONMENT
            "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT};LINGTU_TEST_DDS_FAILURE_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR_ENDPOINT_FAILURE}")
      if(_LINGTU_CYCLONEDDS_TEST_ENV)
        set_tests_properties(
          test_sensor_publisher_endpoint_process
          PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
      endif()

      add_executable(sensor_topic_rate_probe
        sensor_topic_rate_probe.cpp)
      target_include_directories(sensor_topic_rate_probe PRIVATE
        "${LINGTU_ROOT}/src")
      target_link_libraries(sensor_topic_rate_probe PRIVATE
        lingtu_mujoco_dds_messages
        lingtu_dds_contracts)
      target_compile_options(sensor_topic_rate_probe PRIVATE
        ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
      add_dependencies(sensor_topic_rate_probe
        lingtu_mujoco_sensor_publisher)
      add_test(
        NAME test_sensor_topic_rate_probe
        COMMAND
          "${Python3_EXECUTABLE}"
          "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_topic_rate_probe.py"
          $<TARGET_FILE:lingtu_mujoco_sensor_publisher>
          $<TARGET_FILE:sensor_topic_rate_probe>
          "${LINGTU_ROOT}")
      set_tests_properties(
        test_sensor_topic_rate_probe
        PROPERTIES
          TIMEOUT 40
          RESOURCE_LOCK lingtu_mujoco_sensor_topic_rate
          ENVIRONMENT
            "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR_RATE}")
      add_test(
        NAME test_sensor_topic_rate_qualification
        COMMAND
          "${Python3_EXECUTABLE}"
          "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_topic_rate_probe.py"
          $<TARGET_FILE:lingtu_mujoco_sensor_publisher>
          $<TARGET_FILE:sensor_topic_rate_probe>
          "${LINGTU_ROOT}")
      set_tests_properties(
        test_sensor_topic_rate_qualification
        PROPERTIES
          TIMEOUT 40
          RESOURCE_LOCK lingtu_mujoco_sensor_topic_rate
          ENVIRONMENT
            "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_SENSOR_RATE};LINGTU_SENSOR_RATE_QUALIFICATION=1")
      add_test(
        NAME test_sensor_camera_payload_rate
        COMMAND
          "${Python3_EXECUTABLE}"
          "${_LINGTU_MUJOCO_TESTS_DIR}/test_sensor_topic_rate_probe.py"
          $<TARGET_FILE:lingtu_mujoco_sensor_publisher>
          $<TARGET_FILE:sensor_topic_rate_probe>
          "${LINGTU_ROOT}"
          camera-payload)
      set_tests_properties(
        test_sensor_camera_payload_rate
        PROPERTIES
          TIMEOUT 30
          RESOURCE_LOCK lingtu_mujoco_sensor_topic_rate
          ENVIRONMENT
            "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_CAMERA_PAYLOAD_RATE}")
      if(_LINGTU_CYCLONEDDS_TEST_ENV)
        set_property(TEST
          test_sensor_topic_rate_probe test_sensor_topic_rate_qualification
          test_sensor_camera_payload_rate
          APPEND PROPERTY ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
      endif()
    endif()

    add_executable(test_cmd_vel_tap_dds_process
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_cmd_vel_tap_dds_process.cpp")
    target_include_directories(test_cmd_vel_tap_dds_process PRIVATE
      "${LINGTU_ROOT}/src")
    target_link_libraries(test_cmd_vel_tap_dds_process PRIVATE
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_cmd_vel_tap_dds_process PRIVATE
      ${_LINGTU_MUJOCO_WARNINGS})
    target_compile_definitions(test_cmd_vel_tap_dds_process PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_CMD_VEL_TAP})
    add_test(
      NAME test_cmd_vel_tap_dds_process
      COMMAND
        test_cmd_vel_tap_dds_process
        $<TARGET_FILE:lingtu_mujoco_cmd_vel_tap>)
    set_tests_properties(
      test_cmd_vel_tap_dds_process
      PROPERTIES
        TIMEOUT 20
        RESOURCE_LOCK lingtu_mujoco_dds
        ENVIRONMENT
          "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_CMD_VEL_TAP}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(
        test_cmd_vel_tap_dds_process
        PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()

    add_executable(test_mujoco_driver_bridge_dds_process
      "${_LINGTU_MUJOCO_TESTS_DIR}/test_mujoco_driver_bridge_dds_process.cpp")
    target_include_directories(test_mujoco_driver_bridge_dds_process PRIVATE
      "${LINGTU_ROOT}/src")
    target_link_libraries(test_mujoco_driver_bridge_dds_process PRIVATE
      lingtu_mujoco_dds_messages
      lingtu_dds_contracts)
    target_compile_options(test_mujoco_driver_bridge_dds_process PRIVATE
      ${_LINGTU_LOCAL_ENDPOINT_WARNINGS})
    target_compile_definitions(test_mujoco_driver_bridge_dds_process PRIVATE
      LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_DRIVER_BRIDGE})
    add_dependencies(test_mujoco_driver_bridge_dds_process
      lingtu_mujoco_driver_bridge)
    add_test(
      NAME test_mujoco_driver_bridge_dds_process
      COMMAND
        test_mujoco_driver_bridge_dds_process
        $<TARGET_FILE:lingtu_mujoco_driver_bridge>)
    set_tests_properties(
      test_mujoco_driver_bridge_dds_process
      PROPERTIES
        TIMEOUT 20
        RESOURCE_LOCK lingtu_mujoco_dds
        ENVIRONMENT
          "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_DRIVER_BRIDGE}")
    if(_LINGTU_CYCLONEDDS_TEST_ENV)
      set_tests_properties(
        test_mujoco_driver_bridge_dds_process
        PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
    endif()
    if(Python3_Interpreter_FOUND)
      add_test(
        NAME test_mujoco_driver_bridge_endpoint_process
        COMMAND
          "${Python3_EXECUTABLE}"
          "${_LINGTU_MUJOCO_TESTS_DIR}/test_mujoco_driver_bridge_endpoint_process.py"
          $<TARGET_FILE:lingtu_mujoco_driver_bridge>
          $<TARGET_FILE:test_mujoco_driver_bridge_dds_process>
          "${LINGTU_ROOT}")
      set_tests_properties(
        test_mujoco_driver_bridge_endpoint_process
        PROPERTIES
          TIMEOUT 35
          RESOURCE_LOCK lingtu_mujoco_dds
          ENVIRONMENT
            "LINGTU_TEST_DDS_DOMAIN_ID=${_LINGTU_TEST_DDS_DOMAIN_DRIVER_ENDPOINT}")
      if(_LINGTU_CYCLONEDDS_TEST_ENV)
        set_tests_properties(
          test_mujoco_driver_bridge_endpoint_process
          PROPERTIES ENVIRONMENT "${_LINGTU_CYCLONEDDS_TEST_ENV}")
      endif()
    endif()
  endif()

  if(WIN32)
    # Bundle every executable introduced by the DDS runtime block. Several
    # tests inherit ddsc through generated message libraries, so a direct-link
    # allowlist would silently miss them.
    get_property(
      _LINGTU_MUJOCO_TARGETS_AFTER_DDS_RUNTIME
      DIRECTORY
      PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_LINGTU_MUJOCO_DDS_TARGET IN LISTS
        _LINGTU_MUJOCO_TARGETS_AFTER_DDS_RUNTIME)
      if(NOT _LINGTU_MUJOCO_DDS_TARGET IN_LIST
          _LINGTU_MUJOCO_TARGETS_BEFORE_DDS_RUNTIME)
        get_target_property(
          _LINGTU_MUJOCO_DDS_TARGET_TYPE
          ${_LINGTU_MUJOCO_DDS_TARGET}
          TYPE)
        if(_LINGTU_MUJOCO_DDS_TARGET_TYPE STREQUAL "EXECUTABLE")
          lingtu_mujoco_bundle_cyclonedds(${_LINGTU_MUJOCO_DDS_TARGET})
        endif()
      endif()
    endforeach()
    unset(_LINGTU_MUJOCO_DDS_TARGET_TYPE)
    unset(_LINGTU_MUJOCO_DDS_TARGET)
    unset(_LINGTU_MUJOCO_TARGETS_AFTER_DDS_RUNTIME)
  endif()
  unset(_LINGTU_MUJOCO_TARGETS_BEFORE_DDS_RUNTIME)
endif()
