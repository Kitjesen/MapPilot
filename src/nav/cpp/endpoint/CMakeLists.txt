# Native endpoint targets are owned by src/nav/cpp/CMakeLists.txt.

find_package(iceoryx_binding_c QUIET)
find_package(CycloneDDS REQUIRED)
find_package(OpenMP QUIET)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")
set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
set(_NAV_CPP_DIR "${_LINGTU_ROOT}/src/nav/cpp")
set(_NAV_COMMANDS_CPP_DIR "${_NAV_CPP_DIR}/client")
set(_NAV_TESTS_DIR "${_NAV_CPP_DIR}/tests")
set(_EXPLORE_CPP_DIR "${_LINGTU_ROOT}/src/explore/cpp")
set(_PERCEPTION_INSPECTION_DIR "${_LINGTU_ROOT}/src/perception/inspection")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_NAV_CPP_DIR}/planning/global/octoplanner")

include("${_NAV_CPP_DIR}/cmake/NavCoreTargets.cmake")
set(_NAV_PLAN_CPP_DIR "${LINGTU_NAV_PLAN_CPP_DIR}")
set(_NAV_GLOBAL_CPP_DIR "${LINGTU_NAV_GLOBAL_CPP_DIR}")
set(_LOCAL_PLANNER_CPP_DIR "${LINGTU_NAV_LOCAL_CPP_DIR}")
set(_NAV_KERNEL_DIR "${LINGTU_NAV_KERNEL_DIR}")

if(NOT TARGET lingtu_maps)
  add_subdirectory(
    "${_LINGTU_ROOT}/src/maps"
    "${CMAKE_BINARY_DIR}/maps"
    EXCLUDE_FROM_ALL)
endif()
if(NOT TARGET lingtu_inspection_core)
  set(_LINGTU_NAV_SAVED_BUILD_TESTING "${BUILD_TESTING}")
  set(BUILD_TESTING "${LINGTU_NAV_CPP_BUILD_TESTS}")
  add_subdirectory(
    "${_LINGTU_ROOT}/src/nav/inspection"
    "${CMAKE_BINARY_DIR}/inspection")
  set(BUILD_TESTING "${_LINGTU_NAV_SAVED_BUILD_TESTING}")
  unset(_LINGTU_NAV_SAVED_BUILD_TESTING)
endif()
if(NOT TARGET lingtu_dds_qos_profiles)
  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_BINARY_DIR}/message_cpp")
endif()
lingtu_add_dds_c_messages(lingtu_dds_messages "${_LINGTU_IDL}")

function(lingtu_sources_exist out_var)
  set(_ok TRUE)
  foreach(_path IN LISTS ARGN)
    if(NOT EXISTS "${_path}")
      set(_ok FALSE)
    endif()
  endforeach()
  set("${out_var}" "${_ok}" PARENT_SCOPE)
endfunction()

if(EXISTS "${_OCTOPLANNER3D_RUNTIME_DIR}/CMakeLists.txt")
  set(
    OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT
    OFF
    CACHE BOOL
    "Keep PCL/VTK out of the online navigation endpoint"
    FORCE)
  add_subdirectory(
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    "${CMAKE_BINARY_DIR}/octoplanner3d_runtime")
  set(_HAS_OCTOPLANNER3D_RUNTIME TRUE)
else()
  set(_HAS_OCTOPLANNER3D_RUNTIME FALSE)
endif()

add_library(lingtu_nav_client SHARED
  "${_NAV_COMMANDS_CPP_DIR}/client.cpp"
  "${_NAV_COMMANDS_CPP_DIR}/client_c.cpp")

target_include_directories(lingtu_nav_client PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}")

target_link_libraries(lingtu_nav_client PRIVATE
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)

target_compile_options(lingtu_nav_client PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
set_target_properties(lingtu_nav_client PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN")

add_library(lingtu_inspection_evidence_bridge SHARED
  "${_PERCEPTION_INSPECTION_DIR}/native_bridge.cpp")

target_include_directories(lingtu_inspection_evidence_bridge PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_inspection_evidence_bridge PRIVATE
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)

target_compile_options(lingtu_inspection_evidence_bridge PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
set_target_properties(lingtu_inspection_evidence_bridge PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN")

if(LINGTU_NAV_CPP_BUILD_TESTS)
add_executable(test_nav_client
  "${_NAV_TESTS_DIR}/test_client.cpp")
target_include_directories(test_nav_client PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}")
target_link_libraries(test_nav_client PRIVATE
  lingtu_nav_client
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_qos_profiles
  pthread)
target_compile_options(test_nav_client PRIVATE
  -Wall
  -Wextra
  -Wpedantic)
add_test(NAME test_nav_client COMMAND test_nav_client)
endif()

lingtu_sources_exist(_HAS_NAV_ENDPOINT_SOURCES
  "${CMAKE_CURRENT_LIST_DIR}/nav_native_endpoint.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nav_status_writer.cpp")

if(_HAS_NAV_ENDPOINT_SOURCES AND _HAS_OCTOPLANNER3D_RUNTIME)
  add_executable(navd
    active_octomap_gate.cpp
    active_occupancy_gate.cpp
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
    teleop_safety.cpp)

  target_include_directories(navd PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    "${_OCTOPLANNER3D_RUNTIME_DIR}")

  target_link_libraries(navd PRIVATE
    lingtu_inspection_core
    lingtu_maps
    lingtu_nav_far
    lingtu_nav_plan_loop
    lingtu_dds_messages
    octoplanner3d_runtime
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(navd PRIVATE OpenMP::OpenMP_CXX)
  endif()

  target_compile_options(navd PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  lingtu_nav_enable_performance(navd)
  set_target_properties(navd PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "Skipping navd: optional planner sources are incomplete")
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_global_plan_task.cpp")
  add_executable(test_global_plan_task
    test_global_plan_task.cpp
    global_plan_task.cpp)
  target_include_directories(test_global_plan_task PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
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
    teleop_safety.cpp)

  target_include_directories(test_nav_endpoint_messages PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(test_nav_endpoint_messages PRIVATE
    lingtu_dds_messages
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
    nav_dds_runtime.cpp)
  target_include_directories(test_final_velocity_writer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_final_velocity_writer PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
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
    nav_dds_runtime.cpp)
  target_include_directories(test_inspection_evidence_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_inspection_evidence_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
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
    test_inspection_evidence_bridge.cpp)
  target_include_directories(test_inspection_evidence_bridge PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(test_inspection_evidence_bridge PRIVATE
    lingtu_inspection_evidence_bridge
    lingtu_dds_messages
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
  target_link_libraries(test_nav_endpoint_config PRIVATE lingtu_nav_far)
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
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_OCTOPLANNER3D_RUNTIME_DIR}")
  target_link_libraries(test_active_octomap_gate PRIVATE
    lingtu_maps
    octoplanner3d_runtime)
  target_compile_options(test_active_octomap_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_active_octomap_gate COMMAND test_active_octomap_gate)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_active_occupancy_gate.cpp")
  add_executable(test_active_occupancy_gate
    test_active_occupancy_gate.cpp
    active_occupancy_gate.cpp)
  target_include_directories(test_active_occupancy_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}")
  target_link_libraries(test_active_occupancy_gate PRIVATE
    lingtu_maps
    lingtu_nav_far)
  target_compile_options(test_active_occupancy_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_active_occupancy_gate COMMAND test_active_occupancy_gate)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_pose_buffer.cpp")
  add_executable(test_pose_buffer test_pose_buffer.cpp)
  target_include_directories(test_pose_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_KERNEL_DIR}/include")
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
  target_link_libraries(test_transform_buffer PRIVATE
    lingtu_dds_messages)
  target_compile_options(test_transform_buffer PRIVATE -Wall -Wextra -Wpedantic)
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
  target_link_libraries(test_traversability_geometry PRIVATE
    lingtu_dds_messages)
  target_compile_options(test_traversability_geometry PRIVATE -Wall -Wextra -Wpedantic)
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
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND TARGET GTest::gtest_main
    AND EXISTS "${_NAV_TESTS_DIR}/test_far_planner.cpp")
  add_executable(test_far_planner
    "${_NAV_TESTS_DIR}/test_far_planner.cpp")
  target_include_directories(test_far_planner PRIVATE
    "${_NAV_GLOBAL_CPP_DIR}")
  target_link_libraries(test_far_planner PRIVATE
    lingtu_nav_far
    GTest::gtest_main)
  target_compile_options(test_far_planner PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_far_planner COMMAND test_far_planner)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND TARGET GTest::gtest_main
    AND EXISTS "${_NAV_TESTS_DIR}/test_far_c_api.cpp")
  add_executable(test_far_c_api
    "${_NAV_TESTS_DIR}/test_far_c_api.cpp")
  target_include_directories(test_far_c_api PRIVATE
    "${_NAV_GLOBAL_CPP_DIR}")
  target_link_libraries(test_far_c_api PRIVATE
    lingtu_nav_far_c_api
    GTest::gtest_main)
  target_compile_options(test_far_c_api PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_far_c_api COMMAND test_far_c_api)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_TESTS_DIR}/test_nav_loop.cpp")
  add_executable(test_nav_loop
    "${_NAV_TESTS_DIR}/test_nav_loop.cpp")
  target_include_directories(test_nav_loop PRIVATE
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_definitions(test_nav_loop PRIVATE
    LINGTU_TEST_PATH_LIBRARY="${_NAV_CPP_DIR}/../local/paths")
  target_link_libraries(test_nav_loop PRIVATE
    lingtu_nav_plan_loop
    GTest::gtest_main)
  target_compile_options(test_nav_loop PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_nav_loop COMMAND test_nav_loop)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_assisted_teleop.cpp")
  add_executable(test_assisted_teleop
    test_assisted_teleop.cpp
    teleop_safety.cpp)
  target_include_directories(test_assisted_teleop PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_definitions(test_assisted_teleop PRIVATE
    LINGTU_TEST_PATH_LIBRARY="${_NAV_CPP_DIR}/../local/paths")
  target_link_libraries(test_assisted_teleop PRIVATE
    lingtu_nav_plan_loop
    GTest::gtest_main)
  target_compile_options(test_assisted_teleop PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_assisted_teleop COMMAND test_assisted_teleop)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_TESTS_DIR}/test_path_follower_core.cpp")
  add_executable(test_path_follower_core
    "${_NAV_TESTS_DIR}/test_path_follower_core.cpp")
  target_include_directories(test_path_follower_core PRIVATE
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_path_follower_core PRIVATE
    lingtu_nav_path_follower
    GTest::gtest_main)
  target_compile_options(test_path_follower_core PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_path_follower_core COMMAND test_path_follower_core)
endif()

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_TESTS_DIR}/test_local_planner_core.cpp")
  add_executable(test_local_planner_core
    "${_NAV_TESTS_DIR}/test_local_planner_core.cpp")
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
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/traversability_dds.cpp")
  add_executable(lingtu_traversability_dds
    traversability_dds.cpp
    rolling_exploration_map.cpp)

  target_include_directories(lingtu_traversability_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(lingtu_traversability_dds PRIVATE
    lingtu_dds_messages
    lingtu_maps
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
  lingtu_nav_enable_performance(lingtu_traversability_dds)
  set_target_properties(lingtu_traversability_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "Skipping lingtu_traversability_dds: optional source is missing")
endif()

lingtu_sources_exist(_HAS_EXPLORE_SOURCES
  "${CMAKE_CURRENT_LIST_DIR}/explore_dds.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/explore_input.cpp"
  "${_EXPLORE_CPP_DIR}/frontier.cpp"
  "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
  "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

if(_HAS_EXPLORE_SOURCES)
  add_executable(lingtu_explore_dds
    explore_dds.cpp
    explore_control.cpp
    explore_input.cpp
    status_snapshot_file_writer.cpp
    "${_EXPLORE_CPP_DIR}/frontier.cpp"
    "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

  target_include_directories(lingtu_explore_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(lingtu_explore_dds PRIVATE
    lingtu_nav_client
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  target_compile_options(lingtu_explore_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  set_target_properties(lingtu_explore_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "Skipping lingtu_explore_dds: optional explore sources are incomplete")
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_explore_control.cpp")
  add_executable(test_explore_control
    test_explore_control.cpp
    explore_control.cpp)
  target_include_directories(test_explore_control PRIVATE
    "${_LINGTU_ROOT}/src")
  target_compile_options(test_explore_control PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(NAME test_explore_control COMMAND test_explore_control)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test_explore_input.cpp")
  add_executable(test_explore_input
    test_explore_input.cpp
    explore_input.cpp)
  target_include_directories(test_explore_input PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_explore_input PRIVATE
    lingtu_dds_messages)
  target_compile_options(test_explore_input PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(NAME test_explore_input COMMAND test_explore_input)
endif()

if(EXISTS "${_EXPLORE_CPP_DIR}/test_tare_policy.cpp")
  add_executable(test_tare_policy
    "${_EXPLORE_CPP_DIR}/test_tare_policy.cpp"
    "${_EXPLORE_CPP_DIR}/frontier.cpp"
    "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp")
  target_include_directories(test_tare_policy PRIVATE
    "${_EXPLORE_CPP_DIR}")
  target_compile_options(test_tare_policy PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(NAME test_tare_policy COMMAND test_tare_policy)
endif()
endif()
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/nav_control.cpp")
  add_executable(lingtu_nav_control
    nav_control.cpp)

  target_include_directories(lingtu_nav_control PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_link_libraries(lingtu_nav_control PRIVATE
    lingtu_nav_client
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  target_compile_options(lingtu_nav_control PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  set_target_properties(lingtu_nav_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "Skipping lingtu_nav_control: optional source is missing")
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/motion_mock_dds.cpp")
  add_executable(lingtu_motion_mock_dds
    motion_mock_dds.cpp)

  target_include_directories(lingtu_motion_mock_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src")

  target_link_libraries(lingtu_motion_mock_dds PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)

  target_compile_options(lingtu_motion_mock_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  set_target_properties(lingtu_motion_mock_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "Skipping lingtu_motion_mock_dds: optional source is missing")
endif()

set(_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS)
foreach(_target IN ITEMS
    navd
    lingtu_traversability_dds
    lingtu_explore_dds
    lingtu_nav_control
    lingtu_motion_mock_dds)
  if(TARGET ${_target})
    list(APPEND _LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS ${_target})
    set_target_properties(${_target} PROPERTIES
      INSTALL_RPATH "$ORIGIN")
  endif()
endforeach()
if(_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS)
  install(TARGETS ${_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS}
    RUNTIME DESTINATION .)
endif()

install(TARGETS
  lingtu_nav_client
  lingtu_inspection_evidence_bridge
  LIBRARY DESTINATION .
  RUNTIME DESTINATION .)
