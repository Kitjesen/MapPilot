cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_endpoint LANGUAGES C CXX)

include(CTest)

if(NOT UNIX)
  message(FATAL_ERROR "lingtu_nav_endpoint is a Linux robot-side runtime")
endif()

find_package(iceoryx_binding_c QUIET)
find_package(CycloneDDS REQUIRED)
find_package(OpenMP QUIET)
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
set(_NAV_COMMANDS_CPP_DIR "${_LINGTU_ROOT}/src/nav/commands/cpp")
set(_EXPLORE_CPP_DIR "${_LINGTU_ROOT}/src/explore/cpp")
set(_PERCEPTION_INSPECTION_DIR "${_LINGTU_ROOT}/src/perception/inspection")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_LINGTU_ROOT}/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/runtime")

add_subdirectory(
  "${_LINGTU_ROOT}/src/maps"
  "${CMAKE_CURRENT_BINARY_DIR}/maps"
  EXCLUDE_FROM_ALL)
add_subdirectory(
  "${_LINGTU_ROOT}/src/nav/inspection"
  "${CMAKE_CURRENT_BINARY_DIR}/inspection"
  EXCLUDE_FROM_ALL)

function(lingtu_sources_exist out_var)
  set(_ok TRUE)
  foreach(_path IN LISTS ARGN)
    if(NOT EXISTS "${_path}")
      set(_ok FALSE)
    endif()
  endforeach()
  set("${out_var}" "${_ok}" PARENT_SCOPE)
endfunction()

add_custom_command(
  OUTPUT "${_LINGTU_IDL_HEADER}" "${_LINGTU_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${_LINGTU_IDL}"
  DEPENDS "${_LINGTU_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

if(EXISTS "${_OCTOPLANNER3D_RUNTIME_DIR}/CMakeLists.txt")
  add_subdirectory(
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/octoplanner3d_runtime")
  set(_HAS_OCTOPLANNER3D_RUNTIME TRUE)
else()
  set(_HAS_OCTOPLANNER3D_RUNTIME FALSE)
endif()

add_subdirectory(
  "${_LINGTU_ROOT}/src/message/cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

add_library(lingtu_nav_client SHARED
  "${_NAV_COMMANDS_CPP_DIR}/client.cpp"
  "${_NAV_COMMANDS_CPP_DIR}/client_c.cpp"
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_nav_client PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}")

target_link_libraries(lingtu_nav_client PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)

target_compile_options(lingtu_nav_client PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

add_library(lingtu_inspection_evidence_bridge SHARED
  "${_PERCEPTION_INSPECTION_DIR}/native_bridge.cpp"
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_inspection_evidence_bridge PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_inspection_evidence_bridge PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)

target_compile_options(lingtu_inspection_evidence_bridge PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

add_executable(test_nav_client
  "${_NAV_COMMANDS_CPP_DIR}/test_client.cpp")
target_include_directories(test_nav_client PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}")
target_link_libraries(test_nav_client PRIVATE
  lingtu_nav_client
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)
target_compile_options(test_nav_client PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
add_test(NAME test_nav_client COMMAND test_nav_client)

lingtu_sources_exist(_HAS_NAV_ENDPOINT_SOURCES
  "${CMAKE_CURRENT_LIST_DIR}/nav_native_endpoint.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nav_status_writer.cpp"
  "${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"
  "${_NAV_KERNEL_DIR}/src/path_follower_core.cpp")

if(_HAS_NAV_ENDPOINT_SOURCES AND _HAS_OCTOPLANNER3D_RUNTIME)
  add_executable(lingtu_nav_native_endpoint
    active_octomap_gate.cpp
    nav_native_endpoint.cpp
    global_plan_task.cpp
    nav_dds_runtime.cpp
    nav_endpoint_config.cpp
    input_gate.cpp
    nav_endpoint_messages.cpp
    nav_status_writer.cpp
    status_snapshot_file_writer.cpp
    motion_layer.cpp
    live_obstacle_layer.cpp
    teleop_safety.cpp
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
    lingtu_inspection_core
    lingtu_maps
    octoplanner3d_runtime
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_nav_native_endpoint PRIVATE OpenMP::OpenMP_CXX)
  endif()

  target_compile_options(lingtu_nav_native_endpoint PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
else()
  message(STATUS "Skipping lingtu_nav_native_endpoint: optional planner sources are incomplete")
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_global_plan_task.cpp")
  add_executable(test_global_plan_task
    test_global_plan_task.cpp
    global_plan_task.cpp)
  target_include_directories(test_global_plan_task PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_global_plan_task PRIVATE
    pthread)
  target_compile_options(test_global_plan_task PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_global_plan_task COMMAND test_global_plan_task)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_live_obstacle_layer.cpp")
  add_executable(test_live_obstacle_layer
    test_live_obstacle_layer.cpp
    motion_layer.cpp
    live_obstacle_layer.cpp)

  target_include_directories(test_live_obstacle_layer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_compile_options(test_live_obstacle_layer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_live_obstacle_layer COMMAND test_live_obstacle_layer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_motion_layer.cpp")
  add_executable(test_motion_layer
    test_motion_layer.cpp
    motion_layer.cpp)

  target_include_directories(test_motion_layer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_compile_options(test_motion_layer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_motion_layer COMMAND test_motion_layer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_nav_endpoint_messages.cpp")
  add_executable(test_nav_endpoint_messages
    test_nav_endpoint_messages.cpp
    nav_endpoint_messages.cpp
    teleop_safety.cpp
    "${_LINGTU_IDL_SOURCE}")

  target_include_directories(test_nav_endpoint_messages PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(test_nav_endpoint_messages PRIVATE
    CycloneDDS::ddsc
    pthread)

  target_compile_options(test_nav_endpoint_messages PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_nav_endpoint_messages COMMAND test_nav_endpoint_messages)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_final_velocity_writer.cpp")
  add_executable(test_final_velocity_writer
    test_final_velocity_writer.cpp
    nav_dds_runtime.cpp
    "${_LINGTU_IDL_SOURCE}")
  target_include_directories(test_final_velocity_writer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_final_velocity_writer PRIVATE
    lingtu_inspection_core
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)
  target_compile_options(test_final_velocity_writer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_final_velocity_writer
    COMMAND test_final_velocity_writer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_inspection_evidence_dds.cpp")
  add_executable(test_inspection_evidence_dds
    test_inspection_evidence_dds.cpp
    nav_dds_runtime.cpp
    "${_LINGTU_IDL_SOURCE}")
  target_include_directories(test_inspection_evidence_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_inspection_evidence_dds PRIVATE
    lingtu_inspection_core
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)
  target_compile_options(test_inspection_evidence_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_inspection_evidence_dds
    COMMAND test_inspection_evidence_dds)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_inspection_evidence_bridge.cpp")
  add_executable(test_inspection_evidence_bridge
    test_inspection_evidence_bridge.cpp
    "${_LINGTU_IDL_SOURCE}")
  target_include_directories(test_inspection_evidence_bridge PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(test_inspection_evidence_bridge PRIVATE
    lingtu_inspection_evidence_bridge
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)
  target_compile_options(test_inspection_evidence_bridge PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_inspection_evidence_bridge
    COMMAND test_inspection_evidence_bridge)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_input_gate.cpp")
  add_executable(test_input_gate test_input_gate.cpp input_gate.cpp)
  target_include_directories(test_input_gate PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_input_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_input_gate COMMAND test_input_gate)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_control_authority.cpp")
  add_executable(test_control_authority test_control_authority.cpp)
  target_include_directories(test_control_authority PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_control_authority PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_control_authority COMMAND test_control_authority)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_teleop_safety.cpp")
  add_executable(test_teleop_safety
    test_teleop_safety.cpp
    teleop_safety.cpp)
  target_include_directories(test_teleop_safety PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_teleop_safety PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_teleop_safety COMMAND test_teleop_safety)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_nav_endpoint_config.cpp")
  add_executable(test_nav_endpoint_config
    test_nav_endpoint_config.cpp
    nav_endpoint_config.cpp)
  target_include_directories(test_nav_endpoint_config PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_nav_endpoint_config PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_nav_endpoint_config COMMAND test_nav_endpoint_config)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_nav_status_debug_sampling.cpp")
  add_executable(test_nav_status_debug_sampling
    test_nav_status_debug_sampling.cpp)
  target_include_directories(test_nav_status_debug_sampling PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_nav_status_debug_sampling PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_nav_status_debug_sampling
    COMMAND test_nav_status_debug_sampling)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_status_snapshot_file_writer.cpp")
  add_executable(test_status_snapshot_file_writer
    test_status_snapshot_file_writer.cpp
    status_snapshot_file_writer.cpp)
  target_include_directories(test_status_snapshot_file_writer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(test_status_snapshot_file_writer PRIVATE pthread)
  target_compile_options(test_status_snapshot_file_writer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_status_snapshot_file_writer
    COMMAND test_status_snapshot_file_writer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_dds_drain_policy.cpp")
  add_executable(test_dds_drain_policy test_dds_drain_policy.cpp)
  target_include_directories(test_dds_drain_policy PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_dds_drain_policy PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_dds_drain_policy COMMAND test_dds_drain_policy)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_active_octomap_gate.cpp")
  add_executable(test_active_octomap_gate
    test_active_octomap_gate.cpp
    active_octomap_gate.cpp)
  target_include_directories(test_active_octomap_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_OCTOPLANNER3D_RUNTIME_DIR}")
  target_link_libraries(test_active_octomap_gate PRIVATE
    lingtu_maps
    octoplanner3d_runtime)
  target_compile_options(test_active_octomap_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_active_octomap_gate COMMAND test_active_octomap_gate)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_pose_buffer.cpp")
  add_executable(test_pose_buffer test_pose_buffer.cpp)
  target_include_directories(test_pose_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src/nav/kernel/include")
  target_compile_options(test_pose_buffer PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_pose_buffer COMMAND test_pose_buffer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_grid_inflation.cpp")
  add_executable(test_grid_inflation test_grid_inflation.cpp)
  target_include_directories(test_grid_inflation PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_grid_inflation PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_grid_inflation COMMAND test_grid_inflation)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_point_cloud_layout.cpp")
  add_executable(test_point_cloud_layout test_point_cloud_layout.cpp)
  target_include_directories(test_point_cloud_layout PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_point_cloud_layout PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_point_cloud_layout COMMAND test_point_cloud_layout)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_transform_buffer.cpp")
  add_executable(test_transform_buffer test_transform_buffer.cpp)
  target_include_directories(test_transform_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_transform_buffer PRIVATE -Wall -Wextra -Wpedantic)
  add_dependencies(test_transform_buffer lingtu_nav_client)
  add_test(NAME test_transform_buffer COMMAND test_transform_buffer)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_traversability_geometry.cpp")
  add_executable(test_traversability_geometry test_traversability_geometry.cpp)
  target_include_directories(test_traversability_geometry PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_traversability_geometry PRIVATE -Wall -Wextra -Wpedantic)
  add_dependencies(test_traversability_geometry lingtu_nav_client)
  add_test(NAME test_traversability_geometry COMMAND test_traversability_geometry)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_observed_free_cache.cpp")
  add_executable(test_observed_free_cache test_observed_free_cache.cpp)
  target_include_directories(test_observed_free_cache PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include")
  target_compile_options(test_observed_free_cache PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_observed_free_cache COMMAND test_observed_free_cache)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_safety_grid_probe.cpp")
  add_executable(test_safety_grid_probe test_safety_grid_probe.cpp)
  target_include_directories(test_safety_grid_probe PRIVATE
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include")
  target_compile_options(test_safety_grid_probe PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_safety_grid_probe COMMAND test_safety_grid_probe)
endif()

find_package(GTest CONFIG QUIET)
if(TARGET GTest::gtest_main AND EXISTS "${_NAV_PLAN_CPP_DIR}/tests/test_nav_loop.cpp")
  add_executable(test_nav_loop
    "${_NAV_PLAN_CPP_DIR}/tests/test_nav_loop.cpp"
    "${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"
    "${_NAV_KERNEL_DIR}/src/path_follower_core.cpp")
  target_include_directories(test_nav_loop PRIVATE
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_definitions(test_nav_loop PRIVATE
    LINGTU_TEST_PATH_LIBRARY="${_LOCAL_PLANNER_CPP_DIR}/../paths")
  target_link_libraries(test_nav_loop PRIVATE GTest::gtest_main)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_nav_loop PRIVATE OpenMP::OpenMP_CXX)
  endif()
  target_compile_options(test_nav_loop PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_nav_loop COMMAND test_nav_loop)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_assisted_teleop.cpp")
  add_executable(test_assisted_teleop
    test_assisted_teleop.cpp
    teleop_safety.cpp
    "${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"
    "${_NAV_KERNEL_DIR}/src/path_follower_core.cpp")
  target_include_directories(test_assisted_teleop PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_definitions(test_assisted_teleop PRIVATE
    LINGTU_TEST_PATH_LIBRARY="${_LOCAL_PLANNER_CPP_DIR}/../paths")
  target_link_libraries(test_assisted_teleop PRIVATE GTest::gtest_main)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_assisted_teleop PRIVATE OpenMP::OpenMP_CXX)
  endif()
  target_compile_options(test_assisted_teleop PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_assisted_teleop COMMAND test_assisted_teleop)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_KERNEL_DIR}/tests/test_path_follower_core.cpp")
  add_executable(test_path_follower_core
    "${_NAV_KERNEL_DIR}/tests/test_path_follower_core.cpp"
    "${_NAV_KERNEL_DIR}/src/path_follower_core.cpp")
  target_include_directories(test_path_follower_core PRIVATE
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_path_follower_core PRIVATE GTest::gtest_main)
  target_compile_options(test_path_follower_core PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_path_follower_core COMMAND test_path_follower_core)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${_LOCAL_PLANNER_CPP_DIR}/tests/test_local_planner_core.cpp")
  add_executable(test_local_planner_core
    "${_LOCAL_PLANNER_CPP_DIR}/tests/test_local_planner_core.cpp")
  target_include_directories(test_local_planner_core PRIVATE
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_local_planner_core PRIVATE GTest::gtest_main)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_local_planner_core PRIVATE OpenMP::OpenMP_CXX)
  endif()
  target_compile_options(test_local_planner_core PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_local_planner_core COMMAND test_local_planner_core)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/traversability_dds.cpp")
  add_executable(lingtu_traversability_dds
    traversability_dds.cpp
    "${_LINGTU_IDL_SOURCE}")

  target_include_directories(lingtu_traversability_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(lingtu_traversability_dds PRIVATE
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_traversability_dds PRIVATE OpenMP::OpenMP_CXX)
  endif()

  target_compile_options(lingtu_traversability_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
else()
  message(STATUS "Skipping lingtu_traversability_dds: optional source is missing")
endif()

lingtu_sources_exist(_HAS_EXPLORE_SOURCES
  "${CMAKE_CURRENT_LIST_DIR}/explore_dds.cpp"
  "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

if(_HAS_EXPLORE_SOURCES)
  add_executable(lingtu_explore_dds
    explore_dds.cpp
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp"
    "${_LINGTU_IDL_SOURCE}")

  target_include_directories(lingtu_explore_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}")

  target_link_libraries(lingtu_explore_dds PRIVATE
    lingtu_nav_client
    CycloneDDS::ddsc
    pthread)

  target_compile_options(lingtu_explore_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
else()
  message(STATUS "Skipping lingtu_explore_dds: optional explore sources are incomplete")
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/nav_control.cpp")
  add_executable(lingtu_nav_control
    nav_control.cpp
    "${_LINGTU_IDL_SOURCE}")

  target_include_directories(lingtu_nav_control PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_link_libraries(lingtu_nav_control PRIVATE
    lingtu_nav_client
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  target_compile_options(lingtu_nav_control PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
else()
  message(STATUS "Skipping lingtu_nav_control: optional source is missing")
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/motion_mock_dds.cpp")
  add_executable(lingtu_motion_mock_dds
    motion_mock_dds.cpp
    "${_LINGTU_IDL_SOURCE}")

  target_include_directories(lingtu_motion_mock_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_link_libraries(lingtu_motion_mock_dds PRIVATE
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  target_compile_options(lingtu_motion_mock_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
else()
  message(STATUS "Skipping lingtu_motion_mock_dds: optional source is missing")
endif()
