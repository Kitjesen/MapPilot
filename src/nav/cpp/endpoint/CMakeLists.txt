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
set(_NAV_ENDPOINT_TESTS_DIR "${_NAV_TESTS_DIR}/endpoint")
set(_EXPLORE_CPP_DIR "${_LINGTU_ROOT}/src/explore/cpp")
set(_PERCEPTION_INSPECTION_DIR "${_LINGTU_ROOT}/src/perception/inspection")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_NAV_CPP_DIR}/planning/global/octoplanner")

include("${_NAV_CPP_DIR}/cmake/NavCoreTargets.cmake")
set(_NAV_PLAN_CPP_DIR "${LINGTU_NAV_PLAN_CPP_DIR}")
set(_NAV_GLOBAL_CPP_DIR "${LINGTU_NAV_GLOBAL_CPP_DIR}")
set(_LOCAL_PLANNER_CPP_DIR "${LINGTU_NAV_LOCAL_CPP_DIR}")
set(_NAV_KERNEL_DIR "${LINGTU_NAV_KERNEL_DIR}")
set(_EP_DIRS
  "${CMAKE_CURRENT_LIST_DIR}"
  "${_LINGTU_ROOT}/src")

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

if(_HAS_OCTOPLANNER3D_RUNTIME)
  add_executable(navd
    plan/active_octomap_gate.cpp
    plan/active_occupancy_gate.cpp
    nav_native_endpoint.cpp
    endpoint_loop.cpp
    input/nav_input_state_projector.cpp
    inspection/inspection_command_coordinator.cpp
    inspection/inspection_runtime_controller.cpp
    plan/global_plan_task.cpp
    plan/goal_plan_controller.cpp
    nav_dds_runtime.cpp
    nav_endpoint_config.cpp
    plan/input_gate.cpp
    nav_endpoint_messages.cpp
    motion/control_loop_runtime_guard.cpp
    motion/operator_motion_authority.cpp
    status/control_loop_health.cpp
    status/nav_status_writer.cpp
    status/nav_status_publisher.cpp
    status/nav_status_endpoint_adapter.cpp
    status/navigation_state.cpp
    status/status_snapshot_file_writer.cpp
    status/inspection_status_file_writer.cpp
    status/active_inspection_map_cache.cpp
    motion/command_ingress_controller.cpp
    motion/motion_stop_coordinator.cpp
    motion/teleop_admission_controller.cpp
    motion/teleop_tick_controller.cpp
    motion/autonomy_tick_controller.cpp
    motion/motion_layer.cpp
    plan/live_obstacle_layer.cpp
    motion/teleop_safety.cpp
  )

  target_include_directories(navd PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    "${_EXPLORE_CPP_DIR}"
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    ${_EP_DIRS})

  target_link_libraries(navd PRIVATE
    lingtu_inspection_core
    lingtu_maps
    lingtu_nav_far
    lingtu_nav_plan_loop
    lingtu_nav_rolling_segment
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
  message(FATAL_ERROR "Cannot build required navd target: OctoPlanner3D runtime is missing")
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp")
  add_executable(test_global_plan_task
    "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp"
    plan/global_plan_task.cpp)
  target_include_directories(test_global_plan_task PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_global_plan_task PRIVATE
    pthread)
  target_compile_options(test_global_plan_task PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_global_plan_task COMMAND test_global_plan_task)
endif()

if(NOT TARGET test_goal_plan_controller
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_controller.cpp")
  add_executable(test_goal_plan_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_controller.cpp"
    plan/global_plan_task.cpp
    plan/goal_plan_controller.cpp)
  target_include_directories(test_goal_plan_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_controller PRIVATE pthread)
  target_compile_options(test_goal_plan_controller PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_goal_plan_controller COMMAND test_goal_plan_controller)
endif()

if(NOT TARGET test_goal_plan_completion
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp")
  add_executable(test_goal_plan_completion
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp"
    plan/global_plan_task.cpp
    plan/goal_plan_controller.cpp)
  target_include_directories(test_goal_plan_completion PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_completion PRIVATE pthread)
  target_compile_options(test_goal_plan_completion PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_goal_plan_completion COMMAND test_goal_plan_completion)
endif()

if(NOT TARGET test_rolling_map_segment_executor
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_map_segment_executor.cpp")
  add_executable(test_rolling_map_segment_executor
    "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_map_segment_executor.cpp")
  target_link_libraries(test_rolling_map_segment_executor PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_map_segment_executor PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_rolling_map_segment_executor
    COMMAND test_rolling_map_segment_executor)
endif()

if(NOT TARGET test_rolling_segment_lifecycle
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_segment_lifecycle.cpp")
  add_executable(test_rolling_segment_lifecycle
    "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_segment_lifecycle.cpp")
  target_include_directories(test_rolling_segment_lifecycle PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_rolling_segment_lifecycle PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_segment_lifecycle PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_rolling_segment_lifecycle
    COMMAND test_rolling_segment_lifecycle)
endif()

if(NOT TARGET test_rolling_segment_effect_coordinator
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_segment_effect_coordinator.cpp")
  add_executable(test_rolling_segment_effect_coordinator
    "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_segment_effect_coordinator.cpp")
  target_include_directories(test_rolling_segment_effect_coordinator PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_rolling_segment_effect_coordinator PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_segment_effect_coordinator PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_rolling_segment_effect_coordinator
    COMMAND test_rolling_segment_effect_coordinator)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_live_obstacle_layer.cpp")
  add_executable(test_live_obstacle_layer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_live_obstacle_layer.cpp"
    motion/motion_layer.cpp
    plan/live_obstacle_layer.cpp)

  target_include_directories(test_live_obstacle_layer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")

  target_compile_options(test_live_obstacle_layer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_live_obstacle_layer COMMAND test_live_obstacle_layer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer.cpp")
  add_executable(test_motion_layer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer.cpp"
    motion/motion_layer.cpp)

  target_include_directories(test_motion_layer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")

  target_compile_options(test_motion_layer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_motion_layer COMMAND test_motion_layer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_messages.cpp")
  add_executable(test_nav_endpoint_messages
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_messages.cpp"
    nav_endpoint_messages.cpp
    motion/teleop_safety.cpp
  )

  target_include_directories(test_nav_endpoint_messages PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    ${_EP_DIRS}
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

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_final_velocity_writer.cpp")
  add_executable(test_final_velocity_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_final_velocity_writer.cpp"
    nav_dds_runtime.cpp)
  target_include_directories(test_final_velocity_writer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
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

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_dds.cpp")
  add_executable(test_inspection_evidence_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_dds.cpp"
    nav_dds_runtime.cpp)
  target_include_directories(test_inspection_evidence_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
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

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_goal_status_dds.cpp")
  add_executable(test_navigation_goal_status_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_goal_status_dds.cpp"
    nav_dds_runtime.cpp)
  target_include_directories(test_navigation_goal_status_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_navigation_goal_status_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)
  target_compile_options(test_navigation_goal_status_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_navigation_goal_status_dds
    COMMAND test_navigation_goal_status_dds)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_bridge.cpp")
  add_executable(test_inspection_evidence_bridge
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_bridge.cpp")
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

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_input_gate.cpp")
  add_executable(test_input_gate "${_NAV_ENDPOINT_TESTS_DIR}/test_input_gate.cpp" plan/input_gate.cpp)
  target_include_directories(test_input_gate PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  target_compile_options(test_input_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_input_gate COMMAND test_input_gate)
endif()
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_input_state_projector.cpp")
  add_executable(test_nav_input_state_projector
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_input_state_projector.cpp"
    input/nav_input_state_projector.cpp
    nav_endpoint_messages.cpp
    plan/input_gate.cpp
    plan/live_obstacle_layer.cpp
    motion/motion_layer.cpp)
  target_include_directories(test_nav_input_state_projector PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_nav_input_state_projector PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    pthread)
  target_compile_options(test_nav_input_state_projector PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_nav_input_state_projector
    COMMAND test_nav_input_state_projector)
endif()


if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ack_journal.cpp")
  add_executable(test_command_ack_journal "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ack_journal.cpp")
  target_include_directories(test_command_ack_journal PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")
  target_compile_options(test_command_ack_journal PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_command_ack_journal COMMAND test_command_ack_journal)
endif()

if(NOT TARGET test_command_ingress_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ingress_controller.cpp")
  add_executable(test_command_ingress_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ingress_controller.cpp"
    motion/command_ingress_controller.cpp)
  target_include_directories(test_command_ingress_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")
  target_compile_options(test_command_ingress_controller PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_command_ingress_controller
    COMMAND test_command_ingress_controller)
endif()

if(NOT TARGET test_motion_stop_coordinator
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_stop_coordinator.cpp")
  add_executable(test_motion_stop_coordinator
    "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_stop_coordinator.cpp"
    motion/motion_stop_coordinator.cpp)
  target_include_directories(test_motion_stop_coordinator PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_compile_options(test_motion_stop_coordinator PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_motion_stop_coordinator
    COMMAND test_motion_stop_coordinator)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp")
  add_executable(test_operator_motion_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp"
    nav_dds_runtime.cpp)
  target_include_directories(test_operator_motion_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_operator_motion_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_qos_profiles
    pthread)
  target_compile_options(test_operator_motion_dds PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_operator_motion_dds
    COMMAND test_operator_motion_dds)
endif()
if(NOT TARGET test_operator_motion_authority
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_authority.cpp")
  add_executable(test_operator_motion_authority
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_authority.cpp"
    motion/operator_motion_authority.cpp)
  target_include_directories(test_operator_motion_authority PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_operator_motion_authority PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_operator_motion_authority
    COMMAND test_operator_motion_authority)
endif()

if(NOT TARGET test_operator_motion_output_evidence
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_output_evidence.cpp")
  add_executable(test_operator_motion_output_evidence
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_output_evidence.cpp")
  target_include_directories(test_operator_motion_output_evidence PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_operator_motion_output_evidence PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_operator_motion_output_evidence
    COMMAND test_operator_motion_output_evidence)
endif()
if(NOT TARGET test_control_loop_health
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/status/control_loop_health.cpp")
  add_executable(test_control_loop_health
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    status/control_loop_health.cpp)
  target_include_directories(test_control_loop_health PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_compile_options(test_control_loop_health PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_control_loop_health
    COMMAND test_control_loop_health)
endif()
if(NOT TARGET test_control_loop_runtime_guard
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_runtime_guard.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_LIST_DIR}/motion/control_loop_runtime_guard.cpp")
  add_executable(test_control_loop_runtime_guard
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_runtime_guard.cpp"
    motion/control_loop_runtime_guard.cpp)
  target_include_directories(test_control_loop_runtime_guard PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_compile_options(test_control_loop_runtime_guard PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_control_loop_runtime_guard
    COMMAND test_control_loop_runtime_guard)
endif()
if(NOT TARGET test_nav_status_publisher
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_publisher.cpp")
  add_executable(test_nav_status_publisher
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_publisher.cpp"
    status/nav_status_publisher.cpp
    status/nav_status_writer.cpp
    status/status_snapshot_file_writer.cpp)
  target_include_directories(test_nav_status_publisher PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_nav_status_publisher PRIVATE pthread)
  target_compile_options(test_nav_status_publisher PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_nav_status_publisher
    COMMAND test_nav_status_publisher)
endif()
if(NOT TARGET test_navigation_state
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_state.cpp")
  add_executable(test_navigation_state
    "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_state.cpp"
    status/navigation_state.cpp)
  target_include_directories(test_navigation_state PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_navigation_state PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_navigation_state
    COMMAND test_navigation_state)
endif()

if(NOT TARGET test_inspection_command_coordinator
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_command_coordinator.cpp")
  add_executable(test_inspection_command_coordinator
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_command_coordinator.cpp"
    inspection/inspection_command_coordinator.cpp)
  target_include_directories(test_inspection_command_coordinator PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_command_coordinator PRIVATE
    lingtu_inspection_core)
  target_compile_options(test_inspection_command_coordinator PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_inspection_command_coordinator
    COMMAND test_inspection_command_coordinator)
endif()
if(NOT TARGET test_inspection_runtime_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_runtime_controller.cpp")
  add_executable(test_inspection_runtime_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_runtime_controller.cpp"
    inspection/inspection_runtime_controller.cpp)
  target_include_directories(test_inspection_runtime_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_runtime_controller PRIVATE
    lingtu_inspection_core)
  target_compile_options(test_inspection_runtime_controller PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_inspection_runtime_controller
    COMMAND test_inspection_runtime_controller)
endif()



if(NOT TARGET test_teleop_admission_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_admission_controller.cpp")
  add_executable(test_teleop_admission_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_admission_controller.cpp"
    motion/teleop_admission_controller.cpp)
  target_include_directories(test_teleop_admission_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_teleop_admission_controller PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_teleop_admission_controller
    COMMAND test_teleop_admission_controller)
endif()

if(NOT TARGET test_teleop_tick_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_tick_controller.cpp")
  add_executable(test_teleop_tick_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_tick_controller.cpp"
    motion/teleop_tick_controller.cpp
    motion/teleop_safety.cpp)
  target_include_directories(test_teleop_tick_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_teleop_tick_controller PRIVATE
    lingtu_nav_plan_loop)
  target_compile_options(test_teleop_tick_controller PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_teleop_tick_controller
    COMMAND test_teleop_tick_controller)
endif()

if(NOT TARGET test_autonomy_tick_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_autonomy_tick_controller.cpp")
  add_executable(test_autonomy_tick_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_autonomy_tick_controller.cpp"
    motion/autonomy_tick_controller.cpp
    motion/teleop_safety.cpp)
  target_include_directories(test_autonomy_tick_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_autonomy_tick_controller PRIVATE
    lingtu_nav_plan_loop)
  target_compile_options(test_autonomy_tick_controller PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_autonomy_tick_controller
    COMMAND test_autonomy_tick_controller)
endif()

if(NOT TARGET test_active_inspection_map_cache
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_active_inspection_map_cache.cpp")
  add_executable(test_active_inspection_map_cache
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_inspection_map_cache.cpp"
    status/active_inspection_map_cache.cpp)
  target_include_directories(test_active_inspection_map_cache PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_active_inspection_map_cache PRIVATE pthread)
  target_compile_options(test_active_inspection_map_cache PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_active_inspection_map_cache
    COMMAND test_active_inspection_map_cache)
endif()

if(NOT TARGET test_inspection_status_file_writer
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp")
  add_executable(test_inspection_status_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp"
    status/inspection_status_file_writer.cpp
    status/status_snapshot_file_writer.cpp)
  target_include_directories(test_inspection_status_file_writer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_status_file_writer PRIVATE
    lingtu_inspection_core
    pthread)
  target_compile_options(test_inspection_status_file_writer PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_inspection_status_file_writer
    COMMAND test_inspection_status_file_writer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_stop_confirmation.cpp")
  add_executable(test_stop_confirmation "${_NAV_ENDPOINT_TESTS_DIR}/test_stop_confirmation.cpp")
  target_include_directories(test_stop_confirmation PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_stop_confirmation PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_stop_confirmation COMMAND test_stop_confirmation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp")
  add_executable(test_control_authority "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp")
  target_include_directories(test_control_authority PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_control_authority PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_control_authority COMMAND test_control_authority)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp")
  add_executable(test_teleop_safety
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp"
    motion/teleop_safety.cpp
  )
  target_include_directories(test_teleop_safety PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_teleop_safety PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_teleop_safety COMMAND test_teleop_safety)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp")
  add_executable(test_nav_endpoint_config
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp"
    nav_endpoint_config.cpp)
  target_include_directories(test_nav_endpoint_config PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_nav_endpoint_config PRIVATE lingtu_nav_far)
  target_compile_options(test_nav_endpoint_config PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_nav_endpoint_config COMMAND test_nav_endpoint_config)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_debug_sampling.cpp")
  add_executable(test_nav_status_debug_sampling
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_debug_sampling.cpp")
  target_include_directories(test_nav_status_debug_sampling PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_nav_status_debug_sampling PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(
    NAME test_nav_status_debug_sampling
    COMMAND test_nav_status_debug_sampling)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp")
  add_executable(test_status_snapshot_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp"
    status/status_snapshot_file_writer.cpp)
  target_include_directories(test_status_snapshot_file_writer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
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

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_dds_drain_policy.cpp")
  add_executable(test_dds_drain_policy "${_NAV_ENDPOINT_TESTS_DIR}/test_dds_drain_policy.cpp")
  target_include_directories(test_dds_drain_policy PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  target_compile_options(test_dds_drain_policy PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
  add_test(NAME test_dds_drain_policy COMMAND test_dds_drain_policy)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp")
  add_executable(test_active_octomap_gate
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp"
    plan/active_octomap_gate.cpp)
  target_include_directories(test_active_octomap_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_NAV_GLOBAL_CPP_DIR}"
    ${_EP_DIRS}
    "${_OCTOPLANNER3D_RUNTIME_DIR}")
  target_link_libraries(test_active_octomap_gate PRIVATE
    lingtu_maps
    octoplanner3d_runtime)
  target_compile_options(test_active_octomap_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_active_octomap_gate COMMAND test_active_octomap_gate)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_occupancy_gate.cpp")
  add_executable(test_active_occupancy_gate
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_occupancy_gate.cpp"
    plan/active_occupancy_gate.cpp)
  target_include_directories(test_active_occupancy_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_GLOBAL_CPP_DIR}")
  target_link_libraries(test_active_occupancy_gate PRIVATE
    lingtu_maps
    lingtu_nav_far)
  target_compile_options(test_active_occupancy_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_active_occupancy_gate COMMAND test_active_occupancy_gate)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  add_executable(test_pose_buffer "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  target_include_directories(test_pose_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_options(test_pose_buffer PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_pose_buffer COMMAND test_pose_buffer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  add_executable(test_grid_inflation "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  target_include_directories(test_grid_inflation PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  target_compile_options(test_grid_inflation PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_grid_inflation COMMAND test_grid_inflation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  add_executable(test_point_cloud_layout "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  target_include_directories(test_point_cloud_layout PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  target_compile_options(test_point_cloud_layout PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_point_cloud_layout COMMAND test_point_cloud_layout)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp")
  add_executable(test_transform_buffer "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp")
  target_include_directories(test_transform_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_transform_buffer PRIVATE
    lingtu_dds_messages)
  target_compile_options(test_transform_buffer PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_transform_buffer COMMAND test_transform_buffer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_traversability_geometry.cpp")
  add_executable(test_traversability_geometry "${_NAV_ENDPOINT_TESTS_DIR}/test_traversability_geometry.cpp")
  target_include_directories(test_traversability_geometry PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_traversability_geometry PRIVATE
    lingtu_dds_messages)
  target_compile_options(test_traversability_geometry PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_traversability_geometry COMMAND test_traversability_geometry)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_observed_free_cache.cpp")
  add_executable(test_observed_free_cache "${_NAV_ENDPOINT_TESTS_DIR}/test_observed_free_cache.cpp")
  target_include_directories(test_observed_free_cache PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS})
  target_compile_options(test_observed_free_cache PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_observed_free_cache COMMAND test_observed_free_cache)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_safety_grid_probe.cpp")
  add_executable(test_safety_grid_probe "${_NAV_ENDPOINT_TESTS_DIR}/test_safety_grid_probe.cpp")
  target_include_directories(test_safety_grid_probe PRIVATE
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS})
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

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_assisted_teleop.cpp")
  add_executable(test_assisted_teleop
    "${_NAV_ENDPOINT_TESTS_DIR}/test_assisted_teleop.cpp"
    motion/teleop_safety.cpp
  )
  target_include_directories(test_assisted_teleop PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_PLAN_CPP_DIR}"
    "${_LOCAL_PLANNER_CPP_DIR}"
    ${_EP_DIRS}
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

add_executable(lingtu_traversability_dds
    traversability/traversability_dds.cpp
    traversability/rolling_exploration_map.cpp
  )

  target_include_directories(lingtu_traversability_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})

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


add_executable(lingtu_explore_dds
    explore/explore_dds.cpp
    explore/explore_control.cpp
    explore/directed_exploration_intent.cpp
    explore/explore_goal_lifecycle.cpp
    explore/explore_input.cpp
    status/status_snapshot_file_writer.cpp
    "${_EXPLORE_CPP_DIR}/frontier.cpp"
    "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

  target_include_directories(lingtu_explore_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})

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

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp")
  add_executable(test_explore_control
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp"
    explore/explore_control.cpp)
  target_include_directories(test_explore_control PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_compile_options(test_explore_control PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(NAME test_explore_control COMMAND test_explore_control)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp")
  add_executable(test_directed_exploration_intent
    "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp"
    explore/directed_exploration_intent.cpp)
  target_include_directories(test_directed_exploration_intent PRIVATE
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    ${_EP_DIRS})
  target_compile_options(test_directed_exploration_intent PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(
    NAME test_directed_exploration_intent
    COMMAND test_directed_exploration_intent)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_lifecycle.cpp")
  add_executable(test_explore_goal_lifecycle
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_lifecycle.cpp"
    explore/explore_goal_lifecycle.cpp)
  target_include_directories(test_explore_goal_lifecycle PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_compile_options(test_explore_goal_lifecycle PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -UNDEBUG)
  add_test(
    NAME test_explore_goal_lifecycle
    COMMAND test_explore_goal_lifecycle)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_input.cpp")
  add_executable(test_explore_input
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_input.cpp"
    explore/explore_input.cpp)
  target_include_directories(test_explore_input PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
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
add_executable(lingtu_nav_control
    motion/nav_control.cpp)

  target_include_directories(lingtu_nav_control PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

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

add_executable(lingtu_motion_mock_dds
    motion/motion_mock_dds.cpp)

  target_include_directories(lingtu_motion_mock_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

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
